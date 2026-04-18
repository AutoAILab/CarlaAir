import numpy as np
import json
from geometry import GeometryUtils

# Mock a simple case
width, height = 1280, 720
fov = 110
ray_map = GeometryUtils.get_ray_map(width, height, fov)

# Check center ray
center_ray = ray_map[height//2, width//2]
print(f"Center Ray: {center_ray}")
# Expected: roughly [0, 0, 1] in local space if Z is forward

# Mock a depth map with 10m everywhere
depth = np.full((height, width), 10.0)
points_local = GeometryUtils.reproject_to_camera_space(depth, ray_map)
print(f"Center Point Local: {points_local.reshape(height, width, 3)[height//2, width//2]}")
# Expected: [0, 0, 10]

# Mock a pose: 10m high, looking down 90 degrees (Nadir)
# Nadir Pitch = -90 in CARLA/AirSim
pose = {'x': 0, 'y': 0, 'z': -10, 'qw': 0.7071, 'qx': 0, 'qy': -0.7071, 'qz': 0} # 90 deg pitch down

# In NADIR view (looking straight down):
# Camera Forward (Z) -> World Down (Z_ned = +1)?
# Wait, AirSim PITCH= -90 means looking DOWN?
# In AirSim, Pitch -90 is looking DOWN.

p_world_check = GeometryUtils.camera_to_world(points_local, pose, is_airsim=True)
center_world = p_world_check.reshape(height, width, 3)[height//2, width//2]
print(f"Center Point World: {center_world}")
# If at 10m high (z=-10) and looking down (fwd=down), 
# 10m depth should reach ground (z=0 in CARLA sense).
# My camera_to_world negates Z at the end.
# If pos_z = -10, depth = 10 -> world_z_ned = 0 -> world_z_carla = 0.
