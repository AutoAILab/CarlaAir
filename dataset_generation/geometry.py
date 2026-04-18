import numpy as np
import os
from scipy.spatial.transform import Rotation as R

class GeometryUtils:
    @staticmethod
    def get_ray_map(width, height, fov=110):
        """Generates a static Ray Map for the camera configuration (Pinhole model)."""
        f = width / (2.0 * np.tan(fov * np.pi / 360.0))
        x = np.arange(width)
        y = np.arange(height)
        xx, yy = np.meshgrid(x, y)
        # AirSim/NED Local Camera Frame: X=Forward, Y=Right, Z=Down
        rays = np.stack([np.full_like(xx, f), xx - width/2.0, yy - height/2.0], axis=-1)
        rays /= np.linalg.norm(rays, axis=-1, keepdims=True)
        return rays.astype(np.float32)

    @staticmethod
    def reproject_to_camera_space(depth, ray_map):
        """
        Converts a depth map into 3D points in camera-local space.
        depth: (H, W) or (H, W, 1) float32
        ray_map: (H, W, 3) unit rays
        returns: (N, 3) pointcloud
        """
        if len(depth.shape) == 3:
            depth = depth.squeeze(-1)
        
        # Element-wise multiplication of scalar depth and unit rays
        points_3d = depth[:, :, np.newaxis] * ray_map
        return points_3d.reshape(-1, 3)

    @staticmethod
    def get_rotation_matrix(pitch_deg, yaw_deg, roll_deg):
        """Calculates rotation matrix for CARLA (Left-Handed Z-Up)."""
        p = np.radians(pitch_deg)
        y = np.radians(yaw_deg)
        r = np.radians(roll_deg)
        
        R_x = np.array([[1, 0, 0],
                        [0, np.cos(r), -np.sin(r)],
                        [0, np.sin(r), np.cos(r)]])
        
        R_y = np.array([[np.cos(p), 0, np.sin(p)],
                        [0, 1, 0],
                        [-np.sin(p), 0, np.cos(p)]])
        
        R_z = np.array([[np.cos(y), -np.sin(y), 0],
                        [np.sin(y), np.cos(y), 0],
                        [0, 0, 1]])
        
        # Order for CARLA: Yaw -> Pitch -> Roll (Z * Y * X)
        return R_z @ R_y @ R_x

    @staticmethod
    def euler_to_quaternion(pitch, yaw, roll):
        """Converts CARLA Euler angles to a unit quaternion [qw, qx, qy, qz]."""
        # scipy uses [x, y, z, w] order for its internal representation
        # CARLA uses (intrinsic) rotations: Yaw(Z), then Pitch(Y), then Roll(X)
        rot = R.from_euler('zyx', [yaw, pitch, roll], degrees=True)
        quat = rot.as_quat() # returns [qx, qy, qz, qw]
        return [quat[3], quat[0], quat[1], quat[2]]

    @staticmethod
    def camera_to_world(points, transform_dict, is_airsim=True, global_offset=None):
        """
        Transforms points from camera space to world space.
        transform_dict: {'x', 'y', 'z', 'p', 'yaw', 'r'} or with quat for airsim
        global_offset: {'x', 'y', 'z'} to bridge AirSim and CARLA origins
        """
        if is_airsim:
            # AirSim coordinates (NED, Right-Handed)
            pos = np.array([transform_dict['x'], transform_dict['y'], transform_dict['z']])
            
            # Apply Global Offset (AirSim_Global = CARLA_Global + Offset)
            # So CARLA_Global = AirSim_Global - Offset
            if global_offset:
                pos[0] -= global_offset['x']
                pos[1] -= global_offset['y']
                pos[2] -= global_offset['z']
            
            # Quat to Rot Matrix
            qw, qx, qy, qz = transform_dict['qw'], transform_dict['qx'], transform_dict['qy'], transform_dict['qz']
            R = np.array([
                [1 - 2*qy**2 - 2*qz**2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
                [2*qx*qy + 2*qz*qw, 1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qx*qw],
                [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2]
            ])
            
            # Transform local to AirSim World
            world_points = (R @ points.T).T + pos
            
            # Convert AirSim World (NED) to CARLA World (Z-Up) parity
            # CARLA_Z = -NED_Z
            world_points[:, 2] = -world_points[:, 2] 
            return world_points
        else:
            # CARLA coordinates (Left-Handed)
            pos = np.array([transform_dict['x'], transform_dict['y'], transform_dict['z']])
            R = GeometryUtils.get_rotation_matrix(transform_dict['p'], transform_dict['yaw'], transform_dict['r'])
            return (R @ points.T).T + pos

    @staticmethod
    def decode_carla_depth(encoded_depth):
        """Decodes 24-bit encoded depth from CARLA GBuffer."""
        # encoded_depth is (H, W, 4) or (H, W, 3)
        # R = [:,:,2], G=[:,:,1], B=[:,:,0]
        r = encoded_depth[:, :, 2].astype(np.float32)
        g = encoded_depth[:, :, 1].astype(np.float32)
        b = encoded_depth[:, :, 0].astype(np.float32)
        
        normalized = (r + g * 256.0 + b * 65536.0) / 16777215.0
        return normalized * 1000.0 # 1000m range

    @staticmethod
    def save_ply(filename, points):
        """Saves a (N, 3) numpy array as a basic ASCII PLY file."""
        header = f"""ply
format ascii 1.0
element vertex {len(points)}
property float x
property float y
property float z
end_header
"""
        with open(filename, 'w') as f:
            f.write(header)
            for p in points:
                f.write(f"{p[0]} {p[1]} {p[2]}\n")
