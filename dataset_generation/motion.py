import numpy as np
import json
import os
import sys
import carla
import airsim
import time
import logging

from collections import deque

# Add CARLA agents to path if not present (handled by CarlaAir environment usually)
# Ensure CARLA agents are in path
BASE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
AGENT_PATH = os.path.join(BASE_DIR, 'PythonAPI', 'carla')
if AGENT_PATH not in sys.path:
    sys.path.append(AGENT_PATH)

try:
    from agents.navigation.local_planner import LocalPlanner, RoadOption
    from agents.navigation.controller import VehiclePIDController
except ImportError as e:
    logging.warning(f"Could not import CARLA agents from {AGENT_PATH}: {e}")
    LocalPlanner = None
    LocalPlanner = None

class WaypointInterpolator:
    @staticmethod
    def catmull_rom(p0, p1, p2, p3, t, alpha=0.5):
        """
        Catmull-Rom Spline interpolation between p1 and p2.
        p0, p1, p2, p3 are points. t is [0, 1].
        """
        def get_t(t_prev, p_prev, p_curr):
            d = np.linalg.norm(p_curr - p_prev)
            return t_prev + d**alpha

        t0 = 0.0
        t1 = get_t(t0, p0, p1)
        t2 = get_t(t1, p1, p2)
        t3 = get_t(t2, p2, p3)

        t = t1 + t * (t2 - t1)

        a1 = (t1-t)/(t1-t0)*p0 + (t-t0)/(t1-t0)*p1
        a2 = (t2-t)/(t2-t1)*p1 + (t-t1)/(t2-t1)*p2
        a3 = (t3-t)/(t3-t2)*p2 + (t-t2)/(t3-t2)*p3

        b1 = (t2-t)/(t2-t0)*a1 + (t-t0)/(t2-t0)*a2
        b2 = (t3-t)/(t3-t1)*a2 + (t-t1)/(t3-t1)*a3

        c = (t2-t)/(t2-t1)*b1 + (t-t1)/(t2-t1)*b2
        return c

class MotionManager:
    def __init__(self, world, uav_clients, global_offset):
        self.world = world
        self.uav_clients = uav_clients
        self.global_offset = global_offset
        self.controllers = {}
        self.history = {} # actor_id -> deque of (timestamp, transform)
        
    def register_actor(self, actor_id, mode, config, vehicle_name=None):
        # Determine the unique key (AirSim Name or CARLA ID)
        key = vehicle_name if vehicle_name in self.uav_clients else actor_id
        
        if mode == 'lead':
            if vehicle_name in self.uav_clients:
                # Drone Lead
                self.controllers[key] = DronePathController(
                    self.world, self.uav_clients[vehicle_name], vehicle_name, config, self.global_offset
                )
            else:
                # CARLA Vehicle Lead
                self.controllers[key] = PathController(self.world, actor_id, config)
        elif mode == 'follow':
            uav_client = self.uav_clients.get(vehicle_name)
            self.controllers[key] = FollowController(self.world, actor_id, config, uav_client, self.global_offset)
        
        self.history[key] = deque(maxlen=400) # 20 seconds at 20fps
        logging.debug(f"Registered {mode} actor: {key} (ID: {actor_id}) with {type(self.controllers[key]).__name__}")

    def update(self, dt, viz=False):
        """Called before world.tick()"""
        current_time = time.time()
        
        # 1. Update Lead Actors First
        for actor_id, ctrl in self.controllers.items():
            if isinstance(ctrl, (PathController, DronePathController)):
                tf = ctrl.step(dt)
                if tf:
                    # For drones, actor_id is the vehicle name string
                    self.history[actor_id].append((current_time, tf))
                if viz and hasattr(ctrl, 'visualize'):
                    ctrl.visualize()

        # 2. Update Follower Actors
        for actor_id, ctrl in self.controllers.items():
            if isinstance(ctrl, FollowController):
                lead_history = self.history.get(ctrl.follow_id)
                tf = ctrl.step(lead_history, dt)
                if tf:
                    self.history[actor_id].append((current_time, tf))

class PathController:
    def __init__(self, world, actor_id, config):
        self.world = world
        self.actor = world.get_actor(actor_id)
        self.waypoints = config.get('path', [])
        self.speed = config.get('speed', 15.0)
        
        self.planner = None
        if self.actor.type_id.startswith('vehicle') and LocalPlanner:
            # Release handbrake and ensure physics is active
            self.actor.set_simulate_physics(True)
            control = carla.VehicleControl(hand_brake=False, brake=0.0, throttle=0.0)
            self.actor.apply_control(control)
            
            opt_dict = {
                'target_speed': self.speed * 3.6,
                'lateral_control_dict': {'K_P': 1.2, 'K_D': 0.05, 'K_I': 0.01, 'dt': 0.05},
                'longitudinal_control_dict': {'K_P': 1.2, 'K_D': 0.02, 'K_I': 0.01, 'dt': 0.05}
            }
            self.planner = LocalPlanner(self.actor, opt_dict=opt_dict)
            
            # Resolve coordinates to Waypoints and wrap in RoadOption tuples
            carla_map = self.world.get_map()
            plan = []
            for wp in self.waypoints:
                loc = carla.Location(x=wp['x'], y=wp['y'], z=wp['z'])
                waypoint = carla_map.get_waypoint(loc)
                plan.append((waypoint, RoadOption.LANEFOLLOW))
                
            self.planner.set_global_plan(plan)

    def step(self, dt):
        if self.planner:
            control = self.planner.run_step()
            # Override handbrake explicitly in every step to prevent locking
            control.hand_brake = False
            control.manual_gear_shift = False
            self.actor.apply_control(control)
            
            # Diagnostic reporting
            if control.throttle > 0.01 or abs(control.steer) > 0.01:
                loc = self.actor.get_location()
                logging.debug(f"UGV ID:{self.actor.id} | Pos:({loc.x:.1f},{loc.y:.1f}) | Throttle:{control.throttle:.2f} | Steer:{control.steer:.2f}")
                
            return self.actor.get_transform()
        return None

    def visualize(self):
        """Renders the waypoints and current target in CARLA."""
        if not self.waypoints:
            return
            
        debug = self.world.debug
        for i in range(len(self.waypoints)):
            wp = self.waypoints[i]
            loc = carla.Location(x=wp['x'], y=wp['y'], z=wp['z'] + 0.5)
            # Draw point
            debug.draw_point(loc, size=0.1, color=carla.Color(0, 255, 0), life_time=0.1)
            # Draw line to next
            if i + 1 < len(self.waypoints):
                next_wp = self.waypoints[i+1]
                next_loc = carla.Location(x=next_wp['x'], y=next_wp['y'], z=next_wp['z'] + 0.5)
                debug.draw_line(loc, next_loc, thickness=0.05, color=carla.Color(0, 200, 0), life_time=0.1)

class FollowController:
    def __init__(self, world, actor_id, config, uav_client=None, global_offset=None):
        self.world = world
        self.actor = world.get_actor(actor_id) if actor_id else None
        self.uav_client = uav_client
        self.global_offset = global_offset
        
        self.follow_id = config.get('follow_id')
        self.lag = config.get('temporal_lag', 1.0)
        self.offset = config.get('offset', {'x': -10, 'y': 0, 'z': 20})
        self.vehicle_name = config.get('id')

    def step(self, lead_history, dt):
        if not lead_history or not self.uav_client:
            return None
            
        target_time = time.time() - self.lag
        target_tf = None
        for timestamp, tf in reversed(lead_history):
            if timestamp <= target_time:
                target_tf = tf
                break
        
        if not target_tf:
            target_tf = lead_history[0][1]

        # Target in CARLA coord space
        target_loc = carla.Location(
            x=target_tf.location.x + self.offset.get('x', 0),
            y=target_tf.location.y + self.offset.get('y', 0),
            z=target_tf.location.z + self.offset.get('z', 20)
        )
        
        # Convert to AirSim NED
        # x_ned = x_carla + offset_x
        # y_ned = y_carla + offset_y
        # z_ned = -(z_carla + offset_z)
        target_pos_airsim = airsim.Vector3r(
            target_loc.x + self.global_offset['x'],
            target_loc.y + self.global_offset['y'],
            -(target_loc.z) + self.global_offset['z']
        )
        
        # Calculate velocity vector to reach target
        curr_pose = self.uav_client.simGetVehiclePose(vehicle_name=self.vehicle_name)
        curr_pos = curr_pose.position
        
        dist_vec = airsim.Vector3r(
            target_pos_airsim.x_val - curr_pos.x_val,
            target_pos_airsim.y_val - curr_pos.y_val,
            target_pos_airsim.z_val - curr_pos.z_val
        )
        
        # Simple P-control for velocity
        v_kp = 2.0
        vx = dist_vec.x_val * v_kp
        vy = dist_vec.y_val * v_kp
        vz = dist_vec.z_val * v_kp
        
        # Cap velocity (Increased to 25.0 for 15.0 m/s UGV follow capability)
        max_v = 25.0
        v_norm = np.sqrt(vx**2 + vy**2 + vz**2)
        if v_norm > max_v:
            vx = (vx / v_norm) * max_v
            vy = (vy / v_norm) * max_v
            vz = (vz / v_norm) * max_v
            
        self.uav_client.moveByVelocityAsync(vx, vy, vz, dt, vehicle_name=self.vehicle_name)
        
        return target_tf # Proxy for history tracking

class DronePathController:
    """Controls an AirSim drone to follow a recorded CARLA-space trajectory."""
    def __init__(self, world, uav_client, vehicle_name, config, global_offset):
        self.world = world
        self.uav_client = uav_client
        self.vehicle_name = vehicle_name
        self.config = config
        self.waypoints = config.get('path', [])
        self.global_offset = global_offset
        self.current_idx = 0
        
        # Perspective configuration
        perspective_cfg = config.get('perspective_cfg', {})
        self.camera_pitch = perspective_cfg.get('camera_pitch', None)
        self.height_offset = perspective_cfg.get('height_offset', 0.0)
        
        # Camera Translation Offset (NED: +X Forward, +Z Down)
        self.camera_offset_x = perspective_cfg.get('camera_offset_x', 0.0)
        self.camera_offset_y = perspective_cfg.get('camera_offset_y', 0.0)
        self.camera_offset_z = perspective_cfg.get('camera_offset_z', 0.0)
        
        self.camera_setup_done = False

    def step(self, dt):
        """Advances to the next waypoint in the recorded sequence."""
        if self.current_idx >= len(self.waypoints):
            return None
            
        # Lazy Camera Setup (Happens after first world tick to avoid RPC hang)
        if not self.camera_setup_done and self.camera_pitch is not None:
            def apply_perspective():
                logging.debug(f"Applying perspective camera pitch {self.camera_pitch} to {self.vehicle_name}...")
                try:
                    # Create a dedicated temporary client for this thread to avoid BufferError
                    # from sharing a socket with the main thread's simSetVehiclePose call.
                    port = self.config.get('airsim_port', 41451)
                    temp_client = airsim.MultirotorClient(port=port)
                    temp_client.confirmConnection()
                    
                    # Apply translation and rotation (NED Coordinate System)
                    temp_client.simSetCameraPose(
                        "0", 
                        airsim.Pose(
                            airsim.Vector3r(self.camera_offset_x, self.camera_offset_y, self.camera_offset_z), 
                            airsim.to_quaternion(np.radians(self.camera_pitch), 0, 0)
                        ), 
                        vehicle_name=self.vehicle_name
                    )
                    logging.debug(f"Perspective applied successfully (Offset: {self.camera_offset_x}, {self.camera_offset_y}, {self.camera_offset_z}).")
                except Exception as e:
                    logging.warning(f"Perspective failed: {e}")

            # Run in thread to avoid blocking the main sync-tick loop (Deadlock Prevention)
            import threading
            threading.Thread(target=apply_perspective, daemon=True).start()
            self.camera_setup_done = True
            
        wp = self.waypoints[self.current_idx]
        if self.current_idx % 20 == 0:
            logging.debug(f"DronePathController.step | Actor: {self.vehicle_name} | idx: {self.current_idx}/{len(self.waypoints)} | WP: {wp['x']:.1f}, {wp['y']:.1f}, {wp['z']:.1f}")
        
        self.current_idx += 1 # 1:1 playback at 20Hz
        
        # WP is in CARLA coords. Convert to AirSim NED.
        loc = carla.Location(x=wp['x'], y=wp['y'], z=wp['z'] + self.height_offset)
        target_pos_airsim = airsim.Vector3r(
            loc.x + self.global_offset['x'],
            loc.y + self.global_offset['y'],
            -(loc.z) + self.global_offset['z']
        )
        
        # Convert Yaw to Quat
        yaw_rad = np.radians(wp['yaw'])
        q = airsim.to_quaternion(0, 0, yaw_rad)
        
        # Use simSetVehiclePose for path-accurate playback in Record mode
        self.uav_client.simSetVehiclePose(airsim.Pose(target_pos_airsim, q), True, vehicle_name=self.vehicle_name)
        
        # Return transform in CARLA space for history tracking
        return carla.Transform(loc, carla.Rotation(yaw=wp['yaw']))

    def visualize(self):
        """Draws the drone path in CARLA for debugging."""
        if not self.waypoints: return
        debug = self.world.debug
        for i in range(len(self.waypoints)):
            wp = self.waypoints[i]
            loc = carla.Location(x=wp['x'], y=wp['y'], z=wp['z'] + 1.0)
            debug.draw_point(loc, size=0.1, color=carla.Color(0, 255, 255), life_time=0.1)
