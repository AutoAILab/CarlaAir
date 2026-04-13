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
        
        # AirSim Client
        self.client_airsim = airsim.MultirotorClient(port=args.airsim_port)
        self.client_airsim.confirmConnection()
        self.uav_names = self.client_airsim.listVehicles()
        print(f"Connected to AirSim Swarm: {self.uav_names}")

        # Sync Mode Setup
        self.original_settings = self.world.get_settings()
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05 # 20 FPS
        self.world.apply_settings(settings)
        
        self.traffic_manager = self.client_carla.get_trafficmanager(args.tm_port)
        self.traffic_manager.set_synchronous_mode(True)
        
        self.actor_list = []
        self.sensor_queues = {}
        self.ugv = None
        self.recording = {} # Frame ID -> Actor Transforms
        
        # Directories
        self.output_dir = os.path.join(args.out, datetime.now().strftime("%Y%m%d_%H%M%S"))
        os.makedirs(self.output_dir, exist_ok=True)
        for sub in ['images', 'depth', 'seg', 'gbuffer', 'metadata']:
            os.makedirs(os.path.join(self.output_dir, sub), exist_ok=True)
        
    def setup_ugv(self):
        """Spawns the Lead UGV and attaches ground sensors."""
        bp = self.blueprint_library.filter('vehicle.tesla.model3')[0]
        spawn_points = self.map.get_spawn_points()
        spawn_point = spawn_points[0] # To be randomized or selected
        
        self.ugv = self.world.spawn_actor(bp, spawn_point)
        self.actor_list.append(self.ugv)
        self.ugv.set_autopilot(True, self.args.tm_port)
        
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
        """Captures synchronized data from 1 UGV and 5 UAVs."""
        if self.args.mode == 'replay':
            self.apply_replay_transforms(frame_id)

        # 1. Tick CARLA
        self.world.tick()
        
        # 2. Get UGV State & Capture (CARLA)
        ugv_transform = self.ugv.get_transform()
        
        # Record transforms for replay
        frame_transforms = {
            'ugv': self.get_transform_dict(ugv_transform),
            'npcs': self.get_npc_transforms(),
            'uavs': {}
        }

        # 3. Handle UAV Swarm (AirSim)
        for i, uav_name in enumerate(self.uav_names):
            if self.args.mode == 'record':
                # Swarm Movement: Simple height-stacked follow
                target_pos = airsim.Vector3r(ugv_transform.location.x, ugv_transform.location.y, -15*(i+1))
                self.client_airsim.simSetVehiclePose(airsim.Pose(target_pos, airsim.to_quaternion(0, 0, 0)), True, vehicle_name=uav_name)
            
            uav_pose = self.client_airsim.simGetVehiclePose(vehicle_name=uav_name)
            frame_transforms['uavs'][uav_name] = self.get_pose_dict(uav_pose)

            # Capture Multi-modal
            responses = self.client_airsim.simGetImages([
                airsim.ImageRequest("0", airsim.ImageType.Scene),
                airsim.ImageRequest("0", airsim.ImageType.DepthPlanar, True),
                airsim.ImageRequest("0", airsim.ImageType.Segmentation)
            ], vehicle_name=uav_name)
            
            self.save_uav_data(frame_id, uav_name, responses)

        # 4. Save UGV Data
        try:
            image = self.sensor_queues['ugv_rgb'].get(timeout=2.0)
            img_name = f'ugv_{frame_id:06d}.png'
            image.save_to_disk(os.path.join(self.output_dir, 'images', img_name))
            
            # Save UGV G-Buffer if available
            self.save_ugv_gbuffer(frame_id)
        except Exception as e:
            print(f"Error saving UGV data: {e}")

        # 5. Save Metadata & Transforms
        self.recording[frame_id] = frame_transforms
        self.save_metadata(frame_id, frame_transforms)

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
                gbuffer_bundle['depth'] = airsim.get_public_log_binary_array(response)
            elif response.image_type == airsim.ImageType.Segmentation:
                gbuffer_bundle['seg'] = airsim.get_public_log_binary_array(response)
        
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
            self.client_airsim.simSetVehiclePose(airsim.Pose(pos, orient), True, vehicle_name=uav_name)

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
    parser.add_argument('--tm_port', type=int, default=8000)
    parser.add_argument('--width', type=int, default=1280)
    parser.add_argument('--height', type=int, default=720)
    parser.add_argument('--mode', choices=['record', 'replay'], default='record')
    parser.add_argument('--out', default='data/agcarla')
    args = parser.parse_args()
    
    gen = AGCarlaGenerator(args)
    try:
        gen.setup_ugv()
        for i in range(100):
            gen.capture_frame(i)
            print(f"Captured Frame {i}")
    finally:
        gen.cleanup()
