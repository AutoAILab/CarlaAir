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
from geometry import GeometryUtils
from motion import MotionManager

class AGCarlaGenerator:
    def __init__(self, args):
        self.args = args
        self.client_carla = carla.Client(args.host, args.port)
        self.client_carla.set_timeout(20.0)
        self.world = self.client_carla.get_world()
        self.blueprint_library = self.world.get_blueprint_library()
        self.map = self.world.get_map()
        
        # AirSim Setup (Multi-Client Pool for Thread-Safety)
        self.uav_names = []
        self.uav_clients = {}
        
        if not args.no_uavs:
            try:
                # 1. Bootstrap client to get vehicle list
                bootstrap_client = airsim.MultirotorClient(port=args.airsim_port)
                bootstrap_client.confirmConnection()
                self.uav_names = bootstrap_client.listVehicles()
                
                # Filter UAVs: Priority (uav_names > route_map > route > all)
                if args.uav_names:
                    self.uav_names = [n for n in self.uav_names if n in args.uav_names]
                    print(f"Filtered UAV list to explicit names: {self.uav_names}")
                elif args.route_map:
                    map_actors = list(args.route_map.keys())
                    self.uav_names = [n for n in self.uav_names if n in map_actors]
                    print(f"Filtered UAV list to route_map actors: {self.uav_names}")
                elif args.route:
                    try:
                        with open(args.route, 'r') as f:
                            route_data = json.load(f)
                            route_actors = {e['id'] for e in route_data}
                            self.uav_names = [n for n in self.uav_names if n in route_actors]
                            print(f"Filtered UAV list to {len(self.uav_names)} vehicles defined in route.")
                    except Exception as e:
                        print(f"Warning: Could not filter UAVs from route: {e}")

                print(f"Initializing AirSim Client Pool for {len(self.uav_names)} vehicles...")
                self.uav_clients = {}      # Sensor/Image Clients
                self.uav_ctrl_clients = {} # Motion/Control Clients
                
                for name in self.uav_names:
                    print(f"  [DEBUG] Connecting to {name}...")
                    
                    # Connection 1: Sensors
                    s_client = airsim.MultirotorClient(port=args.airsim_port)
                    s_client.confirmConnection()
                    self.uav_clients[name] = s_client
                    
                    # Connection 2: Control (Separate socket to avoid BufferError)
                    c_client = airsim.MultirotorClient(port=args.airsim_port)
                    c_client.confirmConnection()
                    c_client.enableApiControl(True, vehicle_name=name)
                    c_client.armDisarm(True, vehicle_name=name)
                    self.uav_ctrl_clients[name] = c_client
                    
                    print(f"  [DEBUG] {name} ready (dual-connection established).")
            except Exception as e:
                print(f"Warning: Could not connect to AirSim: {e}. UAV capture will be skipped.")
        else:
            print("UAV capture disabled via --no-uavs flag.")
        
        # 3. Calibrate CARLA-AirSim Coordinate Offset
        self.global_offset = self.calibrate_global_offsets()
        print(f"Global Coordinate Offset Calibrated: {self.global_offset}")
        
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
        self.world.tick() # Initial tick to "wake up" the simulator in synchronous mode
        
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
        for sub in ['images', 'depth', 'seg', 'gbuffer', 'lidar', 'metadata']:
            os.makedirs(os.path.join(self.output_dir, sub), exist_ok=True)
        
        # Motion Management
        self.motion_manager = None
        if args.route or args.route_map:
            # Pass our dedicated control clients to the motion manager
            self.motion_manager = MotionManager(self.world, self.uav_ctrl_clients, self.global_offset)
            if args.route:
                self.load_route(args.route)
        
        # Swarm Offsets Configuration (Height, HorizontalBackOffset, PitchDeg)
        self.swarm_config = {
            'UAV_1': (15, 12, -30),
            'UAV_2': (30, 12, -45),
            'UAV_3': (45, 12, -60),
            'UAV_4': (60, 12, -75),
            'UAV_5': (90, 0, -90)
        }

    def load_route(self, route_path):
        """Loads route JSON data for later registration."""
        with open(route_path, 'r') as f:
            self.route_data = json.load(f)
        print(f"Loaded Route: {route_path} ({len(self.route_data)} actors defined)")
            
    def register_route_actors(self):
        """Registers and spawns actors defined in the route data or route map."""
        # 1. Process Route Map (Explicit Actor -> File mapping)
        mapped_actors = []
        if self.args.route_map:
            print(f"Loading actors from route_map...")
            for actor_id_str, route_path in self.args.route_map.items():
                try:
                    with open(route_path, 'r') as f:
                        external_data = json.load(f)
                    
                    # Find the specific actor in the external file
                    found = False
                    for entry in external_data:
                        if entry['id'] == actor_id_str or len(external_data) == 1:
                            # Override ID to match the map key if it's the only one
                            entry['id'] = actor_id_str 
                            self._register_single_actor(entry)
                            mapped_actors.append(actor_id_str)
                            found = True
                            break
                    if not found:
                        print(f"Warning: Could not find actor {actor_id_str} in {route_path}")
                except Exception as e:
                    print(f"Error loading route map for {actor_id_str}: {e}")

        # 2. Process Default Route File (Fallback/General)
        if hasattr(self, 'route_data') and self.route_data:
            for entry in self.route_data:
                actor_id_str = entry['id']
                if actor_id_str in mapped_actors:
                    continue # Already registered via map
                self._register_single_actor(entry)

    def _register_single_actor(self, entry):
        """Internal helper to register a single actor entry from route data."""
        actor_id_str = entry['id']
        entry_type = entry.get('type', '').lower()
        
        # 1. Type Inference (Support legacy recordings without explicit type)
        if not entry_type or entry_type == 'unknown':
            if actor_id_str.startswith('UAV'):
                entry_type = 'drone'
            elif actor_id_str.startswith('UGV'):
                entry_type = 'car'
            print(f"  [DEBUG] Inferred type '{entry_type}' for {actor_id_str}")

        # 2. Strict Platform Validation
        if actor_id_str.startswith('UGV'):
            if entry_type not in ['car', 'vehicle']:
                print(f"Error: {actor_id_str} rejected route of type '{entry_type}'. Expected 'car' or 'vehicle'.")
                return
        elif actor_id_str.startswith('UAV'):
            if entry_type != 'drone':
                print(f"Error: {actor_id_str} rejected route of type '{entry_type}'. Expected 'drone'.")
                return
        
        # 3. Mode Enforcement (Prioritize Replay if path is given)
        if entry.get('path') and entry.get('mode') != 'lead':
            print(f"  [DEBUG] Forcing {actor_id_str} to 'lead' mode (Trajectory detected).")
            entry['mode'] = 'lead'
        
        # Application of perspective presets
        perspective_name = self.args.perspective.lower()
        presets = {
            'default': {'camera_pitch': None, 'height_offset': 0.0},
            'top-down': {'camera_pitch': -90.0, 'height_offset': 20.0},
            'oblique': {'camera_pitch': -45.0, 'height_offset': 10.0},
            'side': {'camera_pitch': 0.0, 'height_offset': 0.0}
        }
        p_config = presets.get(perspective_name, presets['default']).copy()
        
        # Manual Overrides from CLI or Config File
        if self.args.camera_pitch is not None:
            p_config['camera_pitch'] = self.args.camera_pitch
        if self.args.height_offset is not None:
            p_config['height_offset'] = self.args.height_offset
        if self.args.camera_offset_x is not None:
            p_config['camera_offset_x'] = self.args.camera_offset_x
        if self.args.camera_offset_y is not None:
            p_config['camera_offset_y'] = self.args.camera_offset_y
        if self.args.camera_offset_z is not None:
            p_config['camera_offset_z'] = self.args.camera_offset_z
            
        entry['perspective_cfg'] = p_config
        print(f"[DEBUG] Final Perspective for {actor_id_str}: {perspective_name} (Pitch: {p_config['camera_pitch']}, Offset: {p_config['height_offset']}m, CamPos: {p_config.get('camera_offset_x', 0)}, {p_config.get('camera_offset_y', 0)}, {p_config.get('camera_offset_z', 0)})")

        if actor_id_str == 'UGV_1':
            # Register the CARLA vehicle actor
            self.motion_manager.register_actor(self.ugv.id, entry['mode'], entry)
        elif actor_id_str in self.uav_names:
            # Register the AirSim drone actor (Lead or Follow)
            entry['airsim_port'] = self.args.airsim_port # Pass port for background thread clients
            self.motion_manager.register_actor(None, entry['mode'], entry, vehicle_name=actor_id_str)
        else:
            print(f"Warning: Actor ID '{actor_id_str}' in route not found in active simulation actors.")
        
    def calibrate_global_offsets(self):
        """Calculates the translation offset between CARLA world origin and AirSim local origin."""
        if not self.uav_names:
            return {'x': 0.0, 'y': 0.0, 'z': 0.0}

        drone_actor = None
        # Find any AirSim drone in CARLA space
        for a in self.world.get_actors():
            if "drone" in a.type_id.lower() or "airsim" in a.type_id.lower():
                drone_actor = a
                break
        
        if not drone_actor:
            print("Warning: Could not find AirSim drone in CARLA world. Offset defaulted to zero.")
            return {'x': 0.0, 'y': 0.0, 'z': 0.0}
            
        cl = drone_actor.get_location()
        # Get position from AirSim for the same drone
        ap = self.uav_clients[self.uav_names[0]].getMultirotorState().kinematics_estimated.position
        
        # offset = AirSim - CARLA
        return {
            'x': ap.x_val - cl.x,
            'y': ap.y_val - cl.y,
            'z': ap.z_val - (-cl.z) # CARLA Z is Up (positive), AirSim Z is Down (negative)
        }
        
    def setup_ugv(self):
        """Spawns the Lead UGV at a spawn point or its first recorded waypoint."""
        if self.args.no_ugv:
            print("UGV spawning skipped via --no-ugv flag.")
            return

        bp = self.blueprint_library.filter('vehicle.tesla.model3')[0]
        
        # Check if we have a starting position from a route
        spawn_point = None
        replaying = False
        
        # Check route map or loaded route data for UGV_1's first waypoint
        ugv_entry = None
        if self.args.route_map and 'UGV_1' in self.args.route_map:
            try:
                with open(self.args.route_map['UGV_1'], 'r') as f:
                    data = json.load(f)
                    for entry in data:
                        if entry['id'] == 'UGV_1' or len(data) == 1:
                            ugv_entry = entry
                            break
            except: pass
        elif hasattr(self, 'route_data'):
            for entry in self.route_data:
                if entry['id'] == 'UGV_1':
                    ugv_entry = entry
                    break
        
        if ugv_entry and ugv_entry.get('path'):
            wp = ugv_entry['path'][0]
            spawn_point = carla.Transform(
                carla.Location(x=wp['x'], y=wp['y'], z=wp['z'] + 0.5), # Add slight lift for safe spawning
                carla.Rotation(yaw=wp['yaw'])
            )
            replaying = True
            print(f"[DEBUG] Spawning UGV_1 at first waypoint: {wp['x']}, {wp['y']}")

        if not spawn_point:
            spawn_points = self.map.get_spawn_points()
            spawn_point = spawn_points[0]
        
        self.ugv = self.world.spawn_actor(bp, spawn_point)
        self.actor_list.append(self.ugv)
        
        # Initial tick to register transform in the world state
        self.world.tick()
        print(f"Lead UGV Spawned at: {self.ugv.get_transform().location}")
        
        if self.traffic_manager and not replaying:
            tm_port = self.traffic_manager.get_port()
            self.ugv.set_autopilot(True, tm_port)
            print("UGV Autopilot active on TM port", tm_port)
        elif replaying:
            print("UGV Autopilot skipped (Replay Mode active).")
        elif not self.args.route:
            self.ugv.set_autopilot(True)
            print("UGV Autopilot active on default TM port")
        
        # Attach CARLA sensors to UGV (RGB, Lidar, Semantic)
        self.add_carla_sensor('sensor.camera.rgb', carla.Transform(carla.Location(x=1.6, z=1.7)), 'ugv_rgb')
        self.add_carla_sensor('sensor.camera.depth', carla.Transform(carla.Location(x=1.6, z=1.7)), 'ugv_depth')
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

        # G-Buffer Listeners Disabled (Unstable in current headless setup)
        # if sensor_id == 'ugv_rgb' and hasattr(sensor, 'listen_to_gbuffer'):
        #     for gb_id in [carla.GBufferTextureID.GBufferA, carla.GBufferTextureID.SceneDepth]:
        #         gb_q = queue.Queue()
        #         sensor.listen_to_gbuffer(gb_id, lambda data: self.on_gbuffer_data(f"ugv_{gb_id.name}", data))
        #         self.sensor_queues[f"ugv_{gb_id.name}"] = gb_q
        
        return sensor

    def on_carla_sensor_data(self, sensor_id, data):
        self.sensor_queues[sensor_id].put(data)

    def capture_frame(self, frame_id):
        """Captures synchronized data from 1 UGV and 5 UAVs using threaded synchronization."""
        print(f"\n[TRACE] Starting Capture Frame {frame_id}")
        
        # 1. Update Motion (Waypoints/Physics)
        if self.motion_manager:
            self.motion_manager.update(self.args.fixed_dt, viz=self.args.viz_path)

        # 2. Initiate UAV Requests (Parallel)
        # We start the AirSim requests BEFORE the CARLA tick.
        # This prevents the deadlock by allowing the tick to "push" the renderer while the RPC is pending.
        ugv_transform = self.ugv.get_transform() if not self.args.no_ugv else None
        ugv_yaw_rad = np.radians(ugv_transform.rotation.yaw) if ugv_transform else 0.0
        uav_futures = {}
        
        print(f"[TRACE] Initiating {len(self.uav_names)} UAV requests...")
        for i, uav_name in enumerate(self.uav_names):
            uav_client = self.uav_clients[uav_name]
            
            # 2a. Manual Swarm Chase (Legacy/Record Mode only - Skip if using waypoint-based route)
            if self.args.mode == 'record' and not (self.args.route or self.args.route_map):
                # Get swarm configuration for this drone
                # Fallback to defaults if name not in config mapping
                height, back_offset, pitch_deg = self.swarm_config.get(uav_name, (30, 12, -45))
                
                # Calculate chase position in CARLA coordinates
                # Pitch/Yaw logic: offset behind the UGV based on its current yaw
                # bx, by represent the horizontal displacement vector from UGV to UAV
                bx = -back_offset * np.cos(ugv_yaw_rad)
                by = -back_offset * np.sin(ugv_yaw_rad)
                
                # Target Position in World Space (Compensating for AirSim Local Origin Offset)
                target_pos_world = airsim.Vector3r(
                    ugv_transform.location.x + bx + self.global_offset['x'],
                    ugv_transform.location.y + by + self.global_offset['y'],
                    -(ugv_transform.location.z + height) + self.global_offset['z']
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
            'ugv': self.get_transform_dict(ugv_transform) if ugv_transform else {},
            'npcs': self.get_npc_transforms(),
            'uavs': {}
        }
        
        for uav_name, future in uav_futures.items():
            try:
                responses = future.result(timeout=30.0)
                uav_client = self.uav_clients[uav_name]
                uav_pose = uav_client.simGetVehiclePose(vehicle_name=uav_name)
                frame_transforms['uavs'][uav_name] = self.get_pose_dict(uav_pose)
                
                if responses:
                    self.save_uav_data(frame_id, uav_name, responses)
                    print(f"[TRACE] UAV data received for {uav_name}")
                else:
                    print(f"Warning: No images received for {uav_name}")
                
                time.sleep(0.1) # Small pacing between drone processing
            except Exception as e:
                print(f"[TRACE] Error resolving UAV {uav_name}: {e}")

        # 4. Save UGV Data
        if self.args.no_ugv:
            print("[TRACE] UGV capture skipped.")
        else:
            print("[TRACE] Retrieving UGV sensor data...")
            try:
                # RGB
                image = self.sensor_queues['ugv_rgb'].get(timeout=5.0)
                img_name = f'ugv_{frame_id:06d}.png'
                image.save_to_disk(os.path.join(self.output_dir, 'images', img_name))
                
                # Depth (Standard sensor)
                try:
                    depth_img = self.sensor_queues['ugv_depth'].get(timeout=2.0)
                    # Convert CARLA depth Image to linear float32
                    # Standard CARLA depth camera returns 24-bit encoded depth in raw_data
                    encoded = np.frombuffer(depth_img.raw_data, dtype=np.uint8).reshape((depth_img.height, depth_img.width, 4))
                    depth_linear = GeometryUtils.decode_carla_depth(encoded)
                    npz_path = os.path.join(self.output_dir, 'gbuffer', f'ugv_{frame_id:06d}.npz')
                    np.savez_compressed(npz_path, SceneDepth=encoded, depth_linear=depth_linear)
                except queue.Empty:
                    print("Warning: UGV Depth data timed out.")

                # Lidar
                try:
                    lidar_data = self.sensor_queues['ugv_lidar'].get(timeout=1.0)
                    # Convert CARLA Lidar (raw_data) to numpy (N, 3 or N, 4)
                    points = np.frombuffer(lidar_data.raw_data, dtype=np.float32).reshape([-1, 4])
                    # Save as .npy
                    np.save(os.path.join(self.output_dir, 'lidar', f'ugv_{frame_id:06d}.npy'), points)
                except queue.Empty:
                    print("Warning: UGV Lidar data timed out.")

                print("[TRACE] UGV data received.")
            except Exception as e:
                print(f"[TRACE] UGV Timeout or Error: {e}")

        # 5. Save Metadata
        self.recording[frame_id] = frame_transforms
        self.save_metadata(frame_id, frame_transforms)
        
        # 6. Append to OpenLabel Manifest
        self.append_openlabel_frame(frame_id)
        
        print(f"[TRACE] Frame {frame_id} Complete")
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
        rays = GeometryUtils.get_ray_map(self.args.width, self.args.height)
        
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
        data['global_offset'] = self.global_offset
        with open(os.path.join(self.output_dir, 'metadata', f'{frame_id:06d}.json'), 'w') as f:
            json.dump(data, f)

    def get_odd_tags(self):
        """Extracts ODD tags (Weather, Town, SeqType) based on SCENARIO_METADATA.md."""
        tags = []
        
        # 1. Town Tag
        # Map name is usually '/Game/Carla/Maps/Town01', we want 'Town_01'
        town_match = self.map.name.split('/')[-1]
        if 'Town' in town_match:
            town_tag = f"Town_{town_match.replace('Town', '')}"
        else:
            town_tag = f"Map_{town_match}"
        tags.append(town_tag)
        
        # 2. Sequence Type Tag
        tags.append("SeqBaseline" if self.args.mode == 'record' else "SeqVar")
        
        # 3. Weather Tag
        w = self.world.get_weather()
        if w.cloudiness < 20 and w.precipitation == 0:
            tags.append("WeatherClear")
        elif w.precipitation > 30:
            tags.append("WeatherRain")
        elif w.fog_density > 30:
            tags.append("WeatherFog")
        else:
            tags.append("WeatherIntermediate")
            
        # 4. Swarm Presence Tags
        for uav_name in self.uav_names:
            tags.append(uav_name)
            
        return tags

    def append_openlabel_frame(self, frame_id):
        """Appends a frame entry to master_manifest.jsonl in OpenLABEL 1.0.0 format."""
        snapshot = self.world.get_snapshot()
        timestamp = snapshot.timestamp.elapsed_seconds
        
        # Construct frame-level metadata
        frame_data = {
            "openlabel": {
                "metadata": {
                    "schema_version": "1.0.0",
                    "generation_time": datetime.now().isoformat(),
                    "creator": "AGCarla Generator"
                },
                "frames": {
                    str(frame_id): {
                        "frame_properties": {
                            "timestamp": timestamp,
                            "tags": self.get_odd_tags(),
                            "external_resources": [
                                {"name": "ugv_rgb", "url": f"images/ugv_{frame_id:06d}.png", "type": "image/png"},
                                {"name": "ugv_gbuffer", "url": f"gbuffer/ugv_{frame_id:06d}.npz", "type": "application/x-npz"}
                            ]
                        }
                    }
                }
            }
        }
        
        # Add UAV resources
        for uav_name in self.uav_names:
            frame_data["openlabel"]["frames"][str(frame_id)]["frame_properties"]["external_resources"].extend([
                {"name": f"{uav_name}_rgb", "url": f"images/{uav_name}_{frame_id:06d}.png", "type": "image/png"},
                {"name": f"{uav_name}_gbuffer", "url": f"gbuffer/{uav_name}_{frame_id:06d}.npz", "type": "application/x-npz"}
            ])

        manifest_path = os.path.join(self.output_dir, 'master_manifest.jsonl')
        with open(manifest_path, 'a') as f:
            f.write(json.dumps(frame_data) + '\n')

    def cleanup(self):
        """Safe cleanup of CARLA and AirSim actors."""
        print("Cleaning up actors...")
        
        # 1. Restore CARLA settings (wrapped in try/except to avoid msgpack cast errors)
        try:
            if hasattr(self, 'original_settings'):
                self.world.apply_settings(self.original_settings)
        except Exception as e:
            print(f"  [DEBUG] Warning: Could not restore CARLA settings: {e}")

        # 2. Stop and Destroy CARLA actors
        for actor in self.actor_list:
            try:
                if actor is not None and actor.is_alive:
                    if hasattr(actor, 'stop'):
                        actor.stop() # Stop sensor listeners first
                    actor.destroy()
            except Exception as e:
                # Silently fail on individual actor destruction to allow others to be cleaned
                pass
        
        # 3. Release AirSim control
        for name, client in self.uav_ctrl_clients.items():
            try:
                client.armDisarm(False, vehicle_name=name)
                client.enableApiControl(False, vehicle_name=name)
            except: pass

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
    parser.add_argument('--route', help='Path to route.json for waypoint-based movement')
    parser.add_argument('--uav-names', nargs='+', help='Explicit list of UAV vehicle names in AirSim')
    parser.add_argument('--route-map', type=lambda x: json.loads(x), help='JSON string mapping Actor ID to Route File (e.g. {"UAV_1": "path/to/file.json"})')
    parser.add_argument('--no-uavs', action='store_true', help='Skip AirSim drone capture for performance')
    parser.add_argument('--no-ugv', action='store_true', help='Skip CARLA vehicle spawning and capture')
    parser.add_argument('--fixed-dt', type=float, default=0.05, help='Fixed delta time for physics sync')
    parser.add_argument('--viz-path', action='store_true', help='Render waypoints in CARLA')
    parser.add_argument('--perspective', choices=['default', 'top-down', 'oblique', 'side'], default='default', help='Custom viewing angle for drone leads')
    parser.add_argument('--camera-pitch', type=float, help='Manual override for camera pitch angle')
    parser.add_argument('--height-offset', type=float, help='Manual override for drone height offset (meters)')
    parser.add_argument('--camera-offset-x', type=float, help='Forward camera translation (meters)')
    parser.add_argument('--camera-offset-y', type=float, help='Right camera translation (meters)')
    parser.add_argument('--camera-offset-z', type=float, help='Downward camera translation (meters)')
    parser.add_argument('--config', help='Path to a JSON configuration file to override defaults')
    args = parser.parse_args()
    
    # Optional: Load from config file
    if args.config:
        print(f"Loading configuration from {args.config}...")
        with open(args.config, 'r') as f:
            config_data = json.load(f)
            # Override args with config values
            for key, value in config_data.items():
                if hasattr(args, key):
                    setattr(args, key, value)
                else:
                    print(f"Warning: Configuration key '{key}' not recognized.")
    
    gen = AGCarlaGenerator(args)
    try:
        gen.setup_ugv()
        # 3. Path Replay initialization (Lead Actors)
        if (gen.args.route or gen.args.route_map) and gen.motion_manager:
            gen.register_route_actors()
        # Initialize Ray Map once
        gen.get_ray_map()
        
        for i in range(args.num_frames):
            gen.capture_frame(i)
            print(f"Captured Frame {i}")
    finally:
        gen.cleanup()
