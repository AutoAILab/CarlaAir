import carla
import numpy as np
import os
import sys
import time

# Ensure we can find GeometryUtils
sys.path.append(os.getcwd())
try:
    from dataset_generation.geometry import GeometryUtils
except ImportError:
    from geometry import GeometryUtils

def parity_check():
    client = carla.Client('localhost', 2000)
    world = client.get_world()
    
    vehicles = world.get_actors().filter('vehicle.*')
    ugv = None
    if not vehicles:
        print("Spawning temporary vehicle for parity check...")
        bp = world.get_blueprint_library().filter('vehicle.tesla.model3')[0]
        spawn_pt = world.get_map().get_spawn_points()[0]
        ugv = world.spawn_actor(bp, spawn_pt)
        # Give it a weird rotation to stress test the math
        tf = spawn_pt
        tf.rotation.yaw = 37.5
        tf.rotation.pitch = 5.2
        tf.rotation.roll = -2.1
        ugv.set_transform(tf)
        time.sleep(1.0)
    else:
        ugv = vehicles[0]
    
    tf = ugv.get_transform()
    
    # 1. Point in front (1 meter)
    p_local = carla.Location(x=1.0, y=0.0, z=0.0)
    p_world_carla = tf.transform(p_local)
    
    # 2. GeometryUtils Transform
    p_np = np.array([[1.0, 0.0, 0.0]])
    tf_dict = {
        'x': tf.location.x, 'y': tf.location.y, 'z': tf.location.z,
        'p': tf.rotation.pitch, 'yaw': tf.rotation.yaw, 'r': tf.rotation.roll
    }
    p_world_gu = GeometryUtils.camera_to_world(p_np, tf_dict, is_airsim=False)
    
    print(f"UGV TF: Pos=({tf.location.x:.3f}, {tf.location.y:.3f}, {tf.location.z:.3f}), Rot=({tf.rotation.yaw:.3f} Yaw)")
    print(f"CARLA World Point: ({p_world_carla.x:.4f}, {p_world_carla.y:.4f}, {p_world_carla.z:.4f})")
    print(f"GU World Point:    ({p_world_gu[0,0]:.4f}, {p_world_gu[0,1]:.4f}, {p_world_gu[0,2]:.4f})")
    
    diff = np.linalg.norm(np.array([p_world_carla.x, p_world_carla.y, p_world_carla.z]) - p_world_gu[0])
    print(f"Difference: {diff:.6f} meters")

if __name__ == "__main__":
    parity_check()
