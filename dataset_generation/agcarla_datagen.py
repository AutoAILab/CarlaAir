import sys
import argparse
import carla
import airsim
import os
import time
import json
import queue
import numpy as np
import threading
import concurrent.futures
from datetime import datetime
from PIL import Image
import io

class AGCarlaGenerator:
    def __init__(self, args):
        self.args = args
        self.client_carla = carla.Client(args.host, args.port)
        self.client_carla.set_timeout(20.0)
        self.world = self.client_carla.get_world()
        self.blueprint_library = self.world.get_blueprint_library()
        self.map = self.world.get_map()
        
        # AirSim Setup (Multi-Client Pool for Thread-Safety)
        # 1. Bootstrap client to get vehicle list
        bootstrap_client = airsim.MultirotorClient(port=args.airsim_port)
        bootstrap_client.confirmConnection()
        self.uav_names = bootstrap_client.listVehicles()
        
        # 2. Initialize dedicated clients for each UAV
        self.uav_clients = {}
        print(f"Initializing AirSim Client Pool for {len(self.uav_names)} vehicles...")
        for name in self.uav_names:
            client = airsim.MultirotorClient(port=args.airsim_port)
            client.confirmConnection()
            self.uav_clients[name] = client
        
        self.recording = {} # Frame ID -> Actor Transforms
        
        # Threading for headless sync stability
        self.executor = concurrent.futures.ThreadPoolExecutor(max_workers=10)
        
        print(f"Connected to AirSim Swarm: {self.uav_names}")

        # Sync Mode Setup
        self.original_settings = self.world.get_settings()
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05 # 20 FPS
        self.world.apply_settings(settings)
        
        # Traffic Manager Setup
        # We use a custom port (8005) or retrieve the existing one to avoid bind conflicts 
        # with the auto_traffic.py script (which typically uses 8000).
        try:
            self.traffic_manager = self.client_carla.get_trafficmanager(8005)
            self.traffic_manager.set_synchronous_mode(True)
            print("Connected to Traffic Manager on port 8005.")
        except Exception as e:
            print(f"Warning: Could not bind to TM on 8005, attempting default: {e}")
            try:
                self.traffic_manager = self.client_carla.get_trafficmanager()
                self.traffic_manager.set_synchronous_mode(True)
                print("Connected to default Traffic Manager.")
            except Exception as e2:
                print(f"Error: All Traffic Manager connection attempts failed: {e2}")
                print("Proceeding without Traffic Manager (Traffic may not be synchronized).")
                self.traffic_manager = None
        
        self.actor_list = []
        self.sensor_queues = {}
        self.ugv = None
        
        # Directories
        self.output_dir = os.path.join(args.out, datetime.now().strftime("%Y%m%d_%H%M%S"))
        os.makedirs(self.output_dir, exist_ok=True)
        for sub in ['images', 'depth', 'seg', 'gbuffer', 'metadata']:
            os.makedirs(os.path.join(self.output_dir, sub), exist_ok=True)
        
        # Swarm Offsets Configuration (Height, HorizontalBackOffset, PitchDeg)
        self.swarm_config = {
            'UAV_1': (15, 12, -30),
            'UAV_2': (30, 12, -45),
            'UAV_3': (45, 12, -60),
            'UAV_4': (60, 12, -75),
            'UAV_5': (90, 0, -90)
        }
        
    def setup_ugv(self):
        """Spawns the Lead UGV and attaches ground sensors."""
        bp = self.blueprint_library.filter('vehicle.tesla.model3')[0]
        spawn_points = self.map.get_spawn_points()
        spawn_point = spawn_points[0] # To be randomized or selected
        
        self.ugv = self.world.spawn_actor(bp, spawn_point)
        self.actor_list.append(self.ugv)
        
        # Ensure we use the correct Traffic Manager for Autopilot
        if self.traffic_manager:
            tm_port = self.traffic_manager.get_port()
            self.ugv.set_autopilot(True, tm_port)
            print(f"UGV Autopilot active on TM port {tm_port}")
        else:
            self.ugv.set_autopilot(True)
            print("UGV Autopilot active on default TM port")
        
        # Attach CARLA sensors to UGV (RGB, Lidar, Semantic)
        self.add_carla_sensor('sensor.camera.rgb', carla.Transform(carla.Location(x=1.6, z=1.7)), 'ugv_rgb')
        self.add_carla_sensor('sensor.lidar.ray_cast', carla.Transform(carla.Location(z=2.5)), 'ugv_lidar')
        
    def add_carla_sensor(self, bp_name, transform, sensor_id):
        bp = self.blueprint_library.find(bp_name)
        if 'camera' in bp_name:
            bp.set_attribute('image_size_x', str(self.args.width))
            bp.set_attribute('image_size_y', str(self.args.height))
            bp.set_attribute('fov', '110')
            
        sensor = self.world.spawn_actor(bp, transform, attach_to=self.ugv)
        q = queue.Queue()
        sensor.listen(lambda data: self.on_carla_sensor_data(sensor_id, data))
        self.actor_list.append(sensor)
        self.sensor_queues[sensor_id] = q

        # G-Buffer Listeners (Only for UGV RGB camera)
        if sensor_id == 'ugv_rgb' and hasattr(sensor, 'listen_to_gbuffer'):
            for gb_id in [carla.GBufferTextureID.GBufferA, carla.GBufferTextureID.SceneDepth]:
                gb_q = queue.Queue()
                sensor.listen_to_gbuffer(gb_id, lambda data: self.on_gbuffer_data(f"ugv_{gb_id.name}", data))
                self.sensor_queues[f"ugv_{gb_id.name}"] = gb_q
        
        return sensor

    def on_carla_sensor_data(self, sensor_id, data):
        self.sensor_queues[sensor_id].put(data)

    def on_gbuffer_data(self, gb_key, data):
        self.sensor_queues[gb_key].put(data)

    def capture_frame(self, frame_id):
        """Captures synchronized data from 1 UGV and 5 UAVs using threaded synchronization."""
        print(f"\n[TRACE] Starting Capture Frame {frame_id}")
        
        if self.args.mode == 'replay':
            print("[TRACE] Applying replay transforms...")
            self.apply_replay_transforms(frame_id)

        # 1. Initiate UAV Requests (Parallel)
        # We start the AirSim requests BEFORE the CARLA tick.
        # This prevents the deadlock by allowing the tick to "push" the renderer while the RPC is pending.
        ugv_transform = self.ugv.get_transform()
        ugv_yaw_rad = np.radians(ugv_transform.rotation.yaw)
        uav_futures = {}
        
        print(f"[TRACE] Initiating {len(self.uav_names)} UAV requests...")
        for i, uav_name in enumerate(self.uav_names):
            uav_client = self.uav_clients[uav_name]
            if self.args.mode == 'record':
                # Get swarm configuration for this drone
                # Fallback to defaults if name not in config mapping
                height, back_offset, pitch_deg = self.swarm_config.get(uav_name, (30, 12, -45))
                
                # Calculate chase position in CARLA coordinates
                # Pitch/Yaw logic: offset behind the UGV based on its current yaw
                # bx, by represent the horizontal displacement vector from UGV to UAV
                bx = -back_offset * np.cos(ugv_yaw_rad)
                by = -back_offset * np.sin(ugv_yaw_rad)
                
                # Target Position in World Space
                target_pos_world = airsim.Vector3r(
                    ugv_transform.location.x + bx,
                    ugv_transform.location.y + by,
                    -(ugv_transform.location.z + height) # AirSim Z-down
                )
                
                # Orientation: Match UGV yaw, apply drone-specific pitch
                target_orient = airsim.to_quaternion(np.radians(pitch_deg), 0, ugv_yaw_rad)
                
                uav_client.simSetVehiclePose(airsim.Pose(target_pos_world, target_orient), True, vehicle_name=uav_name)
            
            # Start parallel capture using the DEDICATED client for this drone
            requests = [
                airsim.ImageRequest("0", airsim.ImageType.Scene),
                airsim.ImageRequest("0", airsim.ImageType.DepthPlanar, True),
                airsim.ImageRequest("0", airsim.ImageType.Segmentation)
            ]
            future = self.executor.submit(uav_client.simGetImages, requests, vehicle_name=uav_name)
            uav_futures[uav_name] = future

        # 2. Tick CARLA (The "Pulse")
        # This advance triggers the renderer cycle needed by effectively all headless simulators.
        print("[TRACE] Ticking CARLA world (The Pulse)...")
        frame = self.world.tick()
        print(f"[TRACE] CARLA Tick successful (Frame: {frame})")
        
        # Stabilizing Sleep
        time.sleep(0.05)
        
        # 3. Resolve UAV Data
        frame_transforms = {
            'ugv': self.get_transform_dict(ugv_transform),
            'npcs': self.get_npc_transforms(),
            'uavs': {}
        }
        
        for uav_name, future in uav_futures.items():
            try:
                responses = future.result(timeout=10.0)
                uav_client = self.uav_clients[uav_name]
                uav_pose = uav_client.simGetVehiclePose(vehicle_name=uav_name)
                frame_transforms['uavs'][uav_name] = self.get_pose_dict(uav_pose)
                
                if responses:
                    self.save_uav_data(frame_id, uav_name, responses)
                    print(f"[TRACE] UAV data received for {uav_name}")
                else:
                    print(f"Warning: No images received for {uav_name}")
            except Exception as e:
                print(f"[TRACE] Error resolving UAV {uav_name}: {e}")

        # 4. Save UGV Data
        print("[TRACE] Retrieving UGV sensor data...")
        try:
            image = self.sensor_queues['ugv_rgb'].get(timeout=5.0)
            img_name = f'ugv_{frame_id:06d}.png'
            image.save_to_disk(os.path.join(self.output_dir, 'images', img_name))
            self.save_ugv_gbuffer(frame_id)
            print("[TRACE] UGV data received.")
        except Exception as e:
            print(f"[TRACE] UGV Timeout or Error: {e}")

        # 5. Save Metadata
        self.recording[frame_id] = frame_transforms
        self.save_metadata(frame_id, frame_transforms)
        print(f"[TRACE] Frame {frame_id} Complete")

    def save_ugv_gbuffer(self, frame_id):
        gb_bundle = {}
        for gb_key in ['ugv_GBufferA', 'ugv_SceneDepth']:
            if gb_key in self.sensor_queues:
                try:
                    data = self.sensor_queues[gb_key].get(timeout=1.0)
                    # Convert CARLA GBuffer data to numpy
                    gb_bundle[gb_key.split('_')[1]] = np.frombuffer(data.raw_data, dtype=np.uint8).reshape((data.height, data.width, 4))
                except queue.Empty:
                    pass
        
        if gb_bundle:
            npz_path = os.path.join(self.output_dir, 'gbuffer', f'ugv_{frame_id:06d}.npz')
            np.savez_compressed(npz_path, **gb_bundle)

    def save_uav_data(self, frame_id, uav_name, responses):
        gbuffer_bundle = {}
        for response in responses:
            if response.image_type == airsim.ImageType.Scene:
                img_path = os.path.join(self.output_dir, 'images', f'{uav_name}_{frame_id:06d}.png')
                airsim.write_file(img_path, response.image_data_uint8)
            elif response.image_type == airsim.ImageType.DepthPlanar:
                gbuffer_bundle['depth'] = airsim.list_to_2d_float_array(response.image_data_float, response.width, response.height)
            elif response.image_type == airsim.ImageType.Segmentation:
                # Segmentation is returned as compressed PNG when pixels_as_float=False
                img = Image.open(io.BytesIO(response.image_data_uint8))
                gbuffer_bundle['seg'] = np.array(img.convert('RGB'))
        
        if gbuffer_bundle:
            npz_path = os.path.join(self.output_dir, 'gbuffer', f'{uav_name}_{frame_id:06d}.npz')
            np.savez_compressed(npz_path, **gbuffer_bundle)

    def get_ray_map(self):
        """Generates a static Ray Map for the camera configuration."""
        f = self.args.width / (2.0 * np.tan(110 * np.pi / 360.0)) # FOV 110
        x = np.arange(self.args.width)
        y = np.arange(self.args.height)
        xx, yy = np.meshgrid(x, y)
        rays = np.stack([xx - self.args.width/2.0, yy - self.args.height/2.0, np.full_like(xx, f)], axis=-1)
        rays /= np.linalg.norm(rays, axis=-1, keepdims=True)
        
        # Save ray map once
        np.save(os.path.join(self.output_dir, 'metadata', 'ray_map.npy'), rays.astype(np.float32))
        return rays.astype(np.float32)

    def apply_replay_transforms(self, frame_id):
        if frame_id not in self.recording:
            return
            
        transforms = self.recording[frame_id]
        
        # 1. Teleport UGV
        t = transforms['ugv']
        loc = carla.Location(x=t['x'], y=t['y'], z=t['z'])
        rot = carla.Rotation(pitch=t['p'], yaw=t['yaw'], roll=t['r'])
        self.ugv.set_transform(carla.Transform(loc, rot))
        
        # 2. Teleport NPCs
        all_actors = self.world.get_actors()
        for npc_id, t in transforms['npcs'].items():
            actor = all_actors.find(int(npc_id))
            if actor:
                loc = carla.Location(x=t['x'], y=t['y'], z=t['z'])
                rot = carla.Rotation(pitch=t['p'], yaw=t['yaw'], roll=t['r'])
                actor.set_transform(carla.Transform(loc, rot))

        # 3. Teleport UAVs
        for uav_name, p in transforms['uavs'].items():
            pos = airsim.Vector3r(p['x'], p['y'], p['z'])
            orient = airsim.Quaternionr(p['qx'], p['qy'], p['qz'], p['qw'])
            uav_client = self.uav_clients[uav_name]
            uav_client.simSetVehiclePose(airsim.Pose(pos, orient), True, vehicle_name=uav_name)

    def get_transform_dict(self, t):
        return {'x': t.location.x, 'y': t.location.y, 'z': t.location.z, 
                'p': t.rotation.pitch, 'yaw': t.rotation.yaw, 'r': t.rotation.roll}

    def get_pose_dict(self, p):
        return {'x': p.position.x_val, 'y': p.position.y_val, 'z': p.position.z_val,
                'qw': p.orientation.w_val, 'qx': p.orientation.x_val, 'qy': p.orientation.y_val, 'qz': p.orientation.z_val}

    def get_npc_transforms(self):
        npcs = {}
        for actor in self.world.get_actors().filter('vehicle.*'):
            if actor.id != self.ugv.id:
                npcs[actor.id] = self.get_transform_dict(actor.get_transform())
        return npcs

    def save_metadata(self, frame_id, data):
        with open(os.path.join(self.output_dir, 'metadata', f'{frame_id:06d}.json'), 'w') as f:
            json.dump(data, f)

    def cleanup(self):
        print("Cleaning up actors...")
        self.world.apply_settings(self.original_settings)
        for actor in self.actor_list:
            if actor.is_alive:
                actor.destroy()
        print("Done.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=2000)
    parser.add_argument('--airsim_port', type=int, default=41451)
    parser.add_argument('--tm_port', type=int, default=8005)
    parser.add_argument('--width', type=int, default=1280)
    parser.add_argument('--height', type=int, default=720)
    parser.add_argument('--mode', choices=['record', 'replay'], default='record')
    parser.add_argument('--out', default='data/agcarla')
    parser.add_argument('--num-frames', type=int, default=100)
    args = parser.parse_args()
    
    gen = AGCarlaGenerator(args)
    try:
        gen.setup_ugv()
        # Initialize Ray Map once
        gen.get_ray_map()
        
        for i in range(args.num_frames):
            gen.capture_frame(i)
            print(f"Captured Frame {i}")
    finally:
        gen.cleanup()
