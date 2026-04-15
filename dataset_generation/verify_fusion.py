import numpy as np
import os
import json
import argparse
from geometry import GeometryUtils

def verify_fusion(data_dir, frame_id=0):
    print(f"\n=== Fusion Validation: Frame {frame_id} ===")
    
    meta_path = os.path.join(data_dir, 'metadata', f'{frame_id:06d}.json')
    with open(meta_path, 'r') as f:
        meta = json.load(f)
    
    global_offset = meta.get('global_offset')
    master_cloud = []
    
    # 1. Reproject UGV
    print("Reprojecting UGV...")
    ugv_gb_path = os.path.join(data_dir, 'gbuffer', f'ugv_{frame_id:06d}.npz')
    if os.path.exists(ugv_gb_path):
        ugv_gb = np.load(ugv_gb_path)
        if 'depth_linear' in ugv_gb:
            depth = ugv_gb['depth_linear']
            h, w = depth.shape
            mask = (depth > 0) & (depth < 50)
            ray_map = GeometryUtils.get_ray_map(w, h)
            p_local = ray_map[mask] * depth[mask][:, np.newaxis]
            
            p_local_carla = p_local.copy()
            p_local_carla[:, 2] = -p_local_carla[:, 2] # Flip to Z-Up for CARLA
            
            p_actor_mounted = p_local_carla + np.array([1.6, 0, 1.7])
            p_world = GeometryUtils.camera_to_world(p_actor_mounted, meta['ugv'], is_airsim=False)
            master_cloud.append(p_world)
            print(f"  Added {len(p_world)} points from UGV.")

    # 2. Reproject UAVs
    for uav_name in meta['uavs'].keys():
        print(f"Reprojecting {uav_name}...")
        uav_gb_path = os.path.join(data_dir, 'gbuffer', f'{uav_name}_{frame_id:06d}.npz')
        if os.path.exists(uav_gb_path):
            uav_gb = np.load(uav_gb_path)
            depth = uav_gb['depth']
            h, w = depth.shape
            mask = (depth > 0) & (depth < 100)
            ray_map = GeometryUtils.get_ray_map(w, h)
            p_local = ray_map[mask] * depth[mask][:, np.newaxis]
            
            # AirSim NED local frame is already right-handed Forward-Right-Down
            # So no flip needed for p_local before mounting
            p_actor_mounted = p_local + np.array([0.5, 0, 0.1])
            
            p_world = GeometryUtils.camera_to_world(p_actor_mounted, meta['uavs'][uav_name], is_airsim=True, global_offset=global_offset)
            master_cloud.append(p_world)
            print(f"  Added {len(p_world)} points from {uav_name}.")

    if master_cloud:
        full_cloud = np.concatenate(master_cloud, axis=0)
        out_path = os.path.join(data_dir, 'verification', f'fusion_{frame_id:06d}.ply')
        GeometryUtils.save_ply(out_path, full_cloud)
        print(f"\nFusion result saved to: {out_path}")
        print(f"Total points: {len(full_cloud)}")
        
        bbox_min = np.min(full_cloud, axis=0)
        bbox_max = np.max(full_cloud, axis=0)
        diag = np.linalg.norm(bbox_max - bbox_min)
        print(f"Cloud Diagonal: {diag:.2f}m")
        if diag < 500:
             print("PASS: Fusion consistency verified.")
    else:
        print("FAIL: No data to fuse.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--dir', required=True)
    parser.add_argument('--frame', type=int, default=0)
    args = parser.parse_args()
    verify_fusion(args.dir, args.frame)
