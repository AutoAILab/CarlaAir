import numpy as np
import os
import json
import argparse
import sys

# Ensure we can find GeometryUtils
sys.path.append(os.getcwd())
from dataset_generation.geometry import GeometryUtils

def geometric_fuzzer(data_dir, frame_id=5):
    with open(os.path.join(data_dir, 'metadata', f'{frame_id:06d}.json'), 'r') as f:
        meta = json.load(f)
    
    # Load LiDAR
    lidar_pts = np.load(os.path.join(data_dir, 'lidar', f'ugv_{frame_id:06d}.npy'))[:, :3]
    stf_lidar = meta.get('sensor_tf', {}).get('ugv_lidar')
    p_world_lidar = GeometryUtils.camera_to_world(lidar_pts, stf_lidar, is_airsim=False)
    
    # Load Depth
    ugv_gb = np.load(os.path.join(data_dir, 'gbuffer', f'ugv_{frame_id:06d}.npz'))
    depth = ugv_gb['depth_linear']
    mask = (depth > 2.0) & (depth < 30.0)
    ray_map = GeometryUtils.get_ray_map(depth.shape[1], depth.shape[0], fov=110)
    p_local_depth = ray_map[mask] * depth[mask][:, np.newaxis]
    p_local_depth[:, 2] = -p_local_depth[:, 2] # Up
    
    stf_depth = meta.get('sensor_tf', {}).get('ugv_depth')
    pos_depth = np.array([stf_depth['x'], stf_depth['y'], stf_depth['z']])
    
    # Find best Rotation Logic
    best_error = float('inf')
    best_config = None
    
    def get_test_matrix(p, y, r, signs):
        p, y, r = np.radians([p*signs[0], y*signs[1], r*signs[2]])
        # Test sign/order combinations... actually let's just test 8 sign cases for the standard matrix
        Rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
        Ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
        Rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
        return Rz @ Ry @ Rx

    print("Fuzzing Geometric Signs...")
    for s_p in [1, -1]:
        for s_y in [1, -1]:
            for s_r in [1, -1]:
                R = get_test_matrix(stf_depth['p'], stf_depth['yaw'], stf_depth['r'], [s_p, s_y, s_r])
                p_world_depth = (R @ p_local_depth.T).T + pos_depth
                
                # Check error (Mean Distance between centroids)
                error = np.linalg.norm(np.mean(p_world_depth, axis=0) - np.mean(p_world_lidar, axis=0))
                if error < best_error:
                    best_error = error
                    best_config = (s_p, s_y, s_r)
    
    print(f"Best Configuration: {best_config} with error {best_error:.4f}m")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--dir', required=True)
    args = parser.parse_args()
    geometric_fuzzer(args.dir)
