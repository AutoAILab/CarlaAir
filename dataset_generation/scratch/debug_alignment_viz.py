import numpy as np
import os
import json
import argparse
from PIL import Image
from geometry import GeometryUtils

def visualize_alignment(data_dir, frame_id):
    with open(os.path.join(data_dir, 'metadata', f'{frame_id:06d}.json'), 'r') as f:
        meta = json.load(f)
    
    # 1. Load LiDAR
    lidar_pts = np.load(os.path.join(data_dir, 'lidar', f'ugv_{frame_id:06d}.npy'))[:, :3]
    stf_lidar = meta.get('sensor_tf', {}).get('ugv_lidar', meta['ugv'])
    # If using meta['ugv'], we need manual offset. If sensor_tf, we don't.
    if 'sensor_tf' in meta:
        p_world_lidar = GeometryUtils.camera_to_world(lidar_pts, stf_lidar, is_airsim=False)
    else:
        p_world_lidar = GeometryUtils.camera_to_world(lidar_pts + np.array([0, 0, 2.5]), meta['ugv'], is_airsim=False)
        
    # 2. Load Depth
    ugv_gb = np.load(os.path.join(data_dir, 'gbuffer', f'ugv_{frame_id:06d}.npz'))
    depth = ugv_gb['depth_linear']
    mask = (depth > 1.0) & (depth < 50.0)
    ray_map = GeometryUtils.get_ray_map(depth.shape[1], depth.shape[0], fov=110)
    p_local = ray_map[mask] * depth[mask][:, np.newaxis]
    p_local[:, 2] = -p_local[:, 2] # Flip to Z-Up
    
    stf_depth = meta.get('sensor_tf', {}).get('ugv_depth', meta['ugv'])
    if 'sensor_tf' in meta:
        p_world_depth = GeometryUtils.camera_to_world(p_local, stf_depth, is_airsim=False)
    else:
        p_world_depth = GeometryUtils.camera_to_world(p_local + np.array([1.6, 0, 1.7]), meta['ugv'], is_airsim=False)

    # 3. Project to Top-Down Image
    # Range center
    center = np.mean(p_world_lidar, axis=0)
    scale = 20 # pixels per meter
    img_size = 1000
    
    img = np.zeros((img_size, img_size, 3), dtype=np.uint8)
    
    def project(pts, color):
        for p in pts[::10]: # Sample for speed
            ix = int((p[0] - center[0]) * scale + img_size // 2)
            iy = int((p[1] - center[1]) * scale + img_size // 2)
            if 0 <= ix < img_size and 0 <= iy < img_size:
                img[iy, ix] = color

    project(p_world_lidar, [0, 255, 0]) # Green
    project(p_world_depth, [255, 0, 0]) # Red
    
    out_path = f"alignment_debug_{frame_id}.png"
    Image.fromarray(img).save(out_path)
    print(f"Saved alignment debug image to: {out_path}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--dir', required=True)
    parser.add_argument('--frame', type=int, default=5)
    args = parser.parse_args()
    visualize_alignment(args.dir, args.frame)
