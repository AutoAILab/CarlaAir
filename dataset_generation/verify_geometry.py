import numpy as np
import os
import json
import argparse
from geometry import GeometryUtils

def verify_frame_geometry(data_dir, frame_id=0):
    print(f"\n=== Geometric Validation: Frame {frame_id} ===")
    
    # 1. Load Metadata
    meta_path = os.path.join(data_dir, 'metadata', f'{frame_id:06d}.json')
    if not os.path.exists(meta_path):
        print(f"Error: Metadata not found at {meta_path}")
        return
    
    with open(meta_path, 'r') as f:
        meta = json.load(f)
    
    global_offset = meta.get('global_offset')
    
    uav_names = list(meta['uavs'].keys())

    # --- STAGE 1: Semantic-Guided Absolute Alignment ---
    print("\n[Stage 1] Semantic Absolute Alignment...")
    if uav_names:
        uav_name = uav_names[0]
        uav_gb_path = os.path.join(data_dir, 'gbuffer', f'{uav_name}_{frame_id:06d}.npz')
        if os.path.exists(uav_gb_path):
            uav_gb = np.load(uav_gb_path)
            uav_depth = uav_gb['depth']
            uav_seg = uav_gb['seg']
            h, w = uav_depth.shape[:2]
            local_ray_map = GeometryUtils.get_ray_map(w, h)
            
            if len(uav_seg.shape) == 3: # RGB
                 vehicle_mask = (uav_seg[:,:,2] > 100) & (uav_seg[:,:,0] < 50)
            else:
                 vehicle_mask = (uav_seg == 10)
            
            if np.any(vehicle_mask):
                veh_depth = uav_depth[vehicle_mask]
                veh_rays = local_ray_map[vehicle_mask]
                p_local = veh_rays * veh_depth[:, np.newaxis]
                p_local_mounted = p_local + np.array([0.5, 0, 0.1])
                p_world = GeometryUtils.camera_to_world(p_local_mounted, meta['uavs'][uav_name], is_airsim=True, global_offset=global_offset)
                veh_centroid = np.mean(p_world, axis=0)
                ugv_pos = np.array([meta['ugv']['x'], meta['ugv']['y'], meta['ugv']['z']])
                dist = np.linalg.norm(veh_centroid - ugv_pos)
                print(f"  UGV Detection Centroid @ {veh_centroid}")
                print(f"  UGV Ground Truth @ {ugv_pos}")
                print(f"  Reprojection Distance: {dist:.4f}m")
                if dist < 10.0:
                    print("  PASS: High-confidence world alignment verified.")
                else:
                    print("  FAIL: Semantic centroid mismatch.")
            else:
                print("  SKIPPED: UGV not found in UAV view.")

    # --- STAGE 2: Corrected Project-Back Consistency ---
    print("\n[Stage 2] Sub-pixel Project-Back Check...")
    if len(uav_names) >= 2:
        uav1_name = uav_names[0]
        uav2_name = uav_names[1]
        u1_gb = np.load(os.path.join(data_dir, 'gbuffer', f'{uav1_name}_{frame_id:06d}.npz'))
        u2_gb = np.load(os.path.join(data_dir, 'gbuffer', f'{uav2_name}_{frame_id:06d}.npz'))
        
        u1_depth = u1_gb['depth']
        u2_depth = u2_gb['depth']
        h1, w1 = u1_depth.shape[:2]
        lrm1 = GeometryUtils.get_ray_map(w1, h1)
        
        valid = (u1_depth > 0) & (u1_depth < 60)
        if np.any(valid):
            y, x = np.where(valid)
            idx = len(y) // 2
            p_local_u1 = lrm1[y[idx], x[idx]] * u1_depth[y[idx], x[idx]]
            p_local_u1_mounted = p_local_u1 + np.array([0.5, 0, 0.1])
            p_world = GeometryUtils.camera_to_world(p_local_u1_mounted.reshape(1, 3), meta['uavs'][uav1_name], is_airsim=True, global_offset=global_offset)[0]
            
            p_world_ned = np.array([p_world[0] + global_offset['x'], p_world[1] + global_offset['y'], -p_world[2] + global_offset['z']])
            u2_pose = meta['uavs'][uav2_name]
            pos2 = np.array([u2_pose['x'], u2_pose['y'], u2_pose['z']])
            qw, qx, qy, qz = u2_pose['qw'], u2_pose['qx'], u2_pose['qy'], u2_pose['qz']
            R2 = np.array([
                [1 - 2*qy**2 - 2*qz**2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
                [2*qx*qy + 2*qz*qw, 1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qx*qw],
                [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2]
            ])
            p_local_u2_mounted = R2.T @ (p_world_ned - pos2)
            p_local_u2 = p_local_u2_mounted - np.array([0.5, 0, 0.1])
            
            if p_local_u2[0] > 0:
                h2, w2 = u2_depth.shape[:2]
                f2 = w2 / (2.0 * np.tan(110 * np.pi / 360.0))
                u_px = int((p_local_u2[1] / p_local_u2[0]) * f2 + w2 / 2.0)
                v_px = int((p_local_u2[2] / p_local_u2[0]) * f2 + h2 / 2.0)
                
                if 0 <= u_px < w2 and 0 <= v_px < h2:
                    actual_d = u2_depth[v_px, u_px]
                    expected_d = np.linalg.norm(p_local_u2)
                    err = np.abs(actual_d - expected_d)
                    print(f"  Reprojection Error: {err:.4f}m")
                    if err < 1.0:
                        print("  PASS: Multi-view geometric locking confirmed.")
                    else:
                        print("  FAIL: Parallax drift detected.")

    # --- STAGE 3: Raw Lidar Alignment (UGV) ---
    print("\n[Stage 3] Raw Lidar Alignment...")
    lidar_path = os.path.join(data_dir, 'lidar', f'ugv_{frame_id:06d}.npy')
    if os.path.exists(lidar_path):
        lidar_pts_raw = np.load(lidar_path)
        lidar_pts = lidar_pts_raw[:, :3]
        # CARLA Lidar is already Z-Up local
        p_actor_lidar = lidar_pts + np.array([0, 0, 2.5])
        p_world_lidar = GeometryUtils.camera_to_world(p_actor_lidar, meta['ugv'], is_airsim=False)
        
        ugv_gb_path = os.path.join(data_dir, 'gbuffer', f'ugv_{frame_id:06d}.npz')
        if os.path.exists(ugv_gb_path):
            ugv_gb = np.load(ugv_gb_path)
            depth_linear = ugv_gb.get('depth_linear')
            if depth_linear is not None:
                h, w = depth_linear.shape
                # IMPORTANT: CARLA Local Frame is Z-Up. Pinhole RayMap is Z-Down.
                # Must flip Z before world transform.
                ray_map = GeometryUtils.get_ray_map(w, h)
                mask = (depth_linear > 0) & (depth_linear < 50.0)
                p_local_depth = ray_map[mask] * depth_linear[mask][:, np.newaxis]
                
                p_local_carla = p_local_depth.copy()
                p_local_carla[:, 2] = -p_local_carla[:, 2] # Flip to Z-Up
                
                p_actor_depth = p_local_carla + np.array([1.6, 0, 1.7])
                p_world_depth = GeometryUtils.camera_to_world(p_actor_depth, meta['ugv'], is_airsim=False)
                
                if len(p_world_depth) > 0:
                    d_mean = np.linalg.norm(np.mean(p_world_depth, axis=0) - np.mean(p_world_lidar, axis=0))
                    print(f"  Cloud-to-Lidar Mean Offset: {d_mean:.4f}m")
                    if d_mean < 2.0:
                        print("  PASS: Depth-to-Lidar alignment verified.")
                    else:
                        print("  FAIL: Significant Depth-to-Lidar drift.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--dir', required=True)
    parser.add_argument('--frame', type=int, default=0)
    args = parser.parse_args()
    verify_frame_geometry(args.dir, args.frame)
