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
    
    # 2. Load Ray Map
    ray_map_path = os.path.join(data_dir, 'metadata', 'ray_map.npy')
    if not os.path.exists(ray_map_path):
        ray_map = GeometryUtils.get_ray_map(1280, 720)
    else:
        ray_map = np.load(ray_map_path)

    uav_names = list(meta['uavs'].keys())
    if not uav_names:
        print("No UAVs found.")
        return

    # --- STAGE 1: Semantic-Guided Absolute Alignment ---
    print("\n[Stage 1] Semantic Absolute Alignment...")
    uav_name = uav_names[0]
    uav_gb_path = os.path.join(data_dir, 'gbuffer', f'{uav_name}_{frame_id:06d}.npz')
    uav_gb = np.load(uav_gb_path)
    uav_depth = uav_gb['depth']
    uav_seg = uav_gb['seg'] # G-Buffer segmentation
    
    # CARLA Vehicle Class ID is 10
    vehicle_mask = (uav_seg == 10)
    
    if np.any(vehicle_mask):
        h, w = uav_depth.shape[:2]
        local_ray_map = GeometryUtils.get_ray_map(w, h)
        
        veh_depth = uav_depth[vehicle_mask]
        veh_rays = local_ray_map[vehicle_mask]
        
        p_local = veh_rays * veh_depth[:, np.newaxis]
        p_world = GeometryUtils.camera_to_world(p_local, meta['uavs'][uav_name], is_airsim=True, global_offset=global_offset)
        
        veh_centroid = np.mean(p_world, axis=0)
        ugv_pos = np.array([meta['ugv']['x'], meta['ugv']['y'], meta['ugv']['z']])
        
        dist = np.linalg.norm(veh_centroid - ugv_pos)
        print(f"  UGV Detection @ {veh_centroid}")
        print(f"  UGV Ground Truth @ {ugv_pos}")
        print(f"  Reprojection Distance: {dist:.4f}m")
        
        if dist < 3.0: # Vehicles are big, 3.0m is fair for centroid-to-origin
            print("  PASS: High-confidence world alignment verified via semantic projection.")
        else:
            print("  FAIL: Semantic centroid mismatch.")
    else:
        print("  SKIPPED: UGV not found in UAV view (Segmentation miss).")

    # --- STAGE 2: Corrected Project-Back Consistency ---
    print("\n[Stage 2] Sub-pixel Project-Back Check...")
    if len(uav_names) >= 2:
        uav2_name = uav_names[1]
        u2_pose = meta['uavs'][uav2_name]
        
        # Pick a point from UAV 1 (anything closer than sky)
        h, w = uav_depth.shape[:2]
        local_ray_map = GeometryUtils.get_ray_map(w, h)
        valid = (uav_depth > 0) & (uav_depth < 100)
        if np.any(valid):
            # Take first valid point
            y, x = np.where(valid)
            p_local_u1 = local_ray_map[y[0], x[0]] * uav_depth[y[0], x[0]]
            p_world = GeometryUtils.camera_to_world(p_local_u1.reshape(1, 3), meta['uavs'][uav_name], is_airsim=True, global_offset=global_offset)[0]
            
            # Map P_world to UAV 2 NED
            p_world_ned = np.array([p_world[0] + global_offset['x'], p_world[1] + global_offset['y'], -p_world[2] + global_offset['z']])
            
            # UAV 2 Transform
            pos2 = np.array([u2_pose['x'], u2_pose['y'], u2_pose['z']])
            qw, qx, qy, qz = u2_pose['qw'], u2_pose['qx'], u2_pose['qy'], u2_pose['qz']
            R2 = np.array([
                [1 - 2*qy**2 - 2*qz**2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
                [2*qx*qy + 2*qz*qw, 1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qx*qw],
                [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2]
            ])
            
            p_local_u2 = R2.T @ (p_world_ned - pos2)
            
            if p_local_u2[0] > 0:
                f = w / (2.0 * np.tan(110 * np.pi / 360.0))
                u_px = (p_local_u2[1] / p_local_u2[0]) * f + w / 2.0
                v_px = (p_local_u2[2] / p_local_u2[0]) * f + h / 2.0
                
                if 0 <= u_px < w and 0 <= v_px < h:
                    u2_depth = np.load(os.path.join(data_dir, 'gbuffer', f'{uav2_name}_{frame_id:06d}.npz'))['depth']
                    actual_d = u2_depth[int(v_px), int(u_px)]
                    expected_d = np.linalg.norm(p_local_u2)
                    err = np.abs(actual_d - expected_d)
                    print(f"  Reprojection Error (NED-Parity): {err:.4f}m")
                    if err < 0.2:
                        print("  PASS: Multi-view geometric locking confirmed.")
                    else:
                        print("  FAIL: Parallax drift detected.")
                else:
                    print("  SKIPPED: Point out of UAV 2 view.")
            else:
                print("  SKIPPED: Point behind UAV 2.")
        else:
            print("  SKIPPED: No valid points for consistency check.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--dir', required=True)
    parser.add_argument('--frame', type=int, default=0)
    args = parser.parse_args()
    verify_frame_geometry(args.dir, args.frame)
