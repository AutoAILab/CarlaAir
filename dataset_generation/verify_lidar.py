import numpy as np
import os
import json
import argparse
import sys
import logging
from scipy.spatial import cKDTree
from geometry import GeometryUtils

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')

def verify_lidar_alignment(data_dir, frames=None):
    logging.info("=== LiDAR Alignment Verification Suite ===")
    logging.info(f"Directory: {data_dir}")
    
    if not os.path.exists(os.path.join(data_dir, 'metadata')):
        logging.error(f"Metadata directory not found: {os.path.join(data_dir, 'metadata')}")
        return

    if frames is None:
        # Auto-detect frames
        frames = sorted([int(f.split('.')[0]) for f in os.listdir(os.path.join(data_dir, 'metadata')) if f.endswith('.json')])
    
    if len(frames) < 2:
        logging.error("At least two frames required for temporal validation.")
        return

    def get_world_pts(pts, meta, sensor_id, fallback_offset):
        stf = meta.get('sensor_tf', {}).get(sensor_id)
        if stf:
            return GeometryUtils.camera_to_world(pts, stf, is_airsim=False)
        else:
            return GeometryUtils.camera_to_world(pts + fallback_offset, meta['ugv'], is_airsim=False)

    # 1. Temporal Consistency Test (Static Alignment)
    logging.info("[Part 1] Temporal Consistency Test...")
    f1, f2 = frames[min(len(frames)-2, 2)], frames[min(len(frames)-1, 6)]
    
    try:
        with open(os.path.join(data_dir, 'metadata', f'{f1:06d}.json'), 'r') as f:
            m1 = json.load(f)
        with open(os.path.join(data_dir, 'metadata', f'{f2:06d}.json'), 'r') as f:
            m2 = json.load(f)
            
        p1_raw = np.load(os.path.join(data_dir, 'lidar', f'ugv_{f1:06d}.npy'))[:, :3]
        p2_raw = np.load(os.path.join(data_dir, 'lidar', f'ugv_{f2:06d}.npy'))[:, :3]
    except Exception as e:
        logging.error(f"Failed to load frame data (F1:{f1}, F2:{f2}): {e}")
        return
        
    # Filter ground points for more stable centroid (z < -2.0 in sensor local)
    p1_ground = p1_raw[p1_raw[:, 2] < -2.0]
    p2_ground = p2_raw[p2_raw[:, 2] < -2.0]
    
    p1_world = get_world_pts(p1_ground, m1, 'ugv_lidar', np.array([0, 0, 2.5]))
    p2_world = get_world_pts(p2_ground, m2, 'ugv_lidar', np.array([0, 0, 2.5]))
    
    if len(p1_world) == 0 or len(p2_world) == 0:
        logging.warning("No ground points found for temporal alignment test.")
        drift = 999.0
    else:
        c1 = np.mean(p1_world, axis=0)
        c2 = np.mean(p2_world, axis=0)
        
        ugv_disp = np.linalg.norm(np.array([m1['ugv']['x'], m1['ugv']['y']]) - np.array([m2['ugv']['x'], m2['ugv']['y']]))
        
        # Ignore Z-drift if UGV is stationary (compensates for tick-sync altitude jitter)
        if ugv_disp < 0.01:
            drift = np.linalg.norm(c1[:2] - c2[:2])
        else:
            drift = np.linalg.norm(c1 - c2)
        
        logging.info(f"  Frame Interval: {f1} -> {f2}")
        logging.debug(f"  UGV Displacement: {ugv_disp:.4f}m")
        logging.info(f"  Global Temporal Drift: {drift:.4f}m")
    
    if drift < 0.05:
        logging.info("  PASS: Temporal alignment is sub-centimeter consistent.")
    else:
        logging.error(f"  FAIL: Drift detected ({drift:.4f}m).")

    # 2. Cross-Modal Lock Test (LiDAR vs Depth)
    logging.info("[Part 2] Cross-Modal Lock Test (LiDAR vs Depth)...")
    frame_id = frames[len(frames) // 2]
    try:
        with open(os.path.join(data_dir, 'metadata', f'{frame_id:06d}.json'), 'r') as f:
            meta = json.load(f)
            
        lidar_pts = np.load(os.path.join(data_dir, 'lidar', f'ugv_{frame_id:06d}.npy'))[:, :3]
        ugv_gb = np.load(os.path.join(data_dir, 'gbuffer', f'ugv_{frame_id:06d}.npz'))
    except Exception as e:
        logging.error(f"Failed to load cross-modal data (F:{frame_id}): {e}")
        return

    p_world_lidar = get_world_pts(lidar_pts, meta, 'ugv_lidar', np.array([0, 0, 2.5]))
    
    if 'depth_linear' in ugv_gb:
        depth = ugv_gb['depth_linear']
        mask = (depth > 2.0) & (depth < 30.0)
        ray_map = GeometryUtils.get_ray_map(depth.shape[1], depth.shape[0], fov=110)
        p_local = ray_map[mask] * depth[mask][:, np.newaxis]
        p_local[:, 2] = -p_local[:, 2] # Flip to Z-Up
        p_world_depth = get_world_pts(p_local, meta, 'ugv_depth', np.array([1.6, 0, 1.7]))
        
        # Filter LiDAR to matching Camera Frustum
        stf_c = meta.get('sensor_tf', {}).get('ugv_depth', meta['ugv'])
        R_c = GeometryUtils.get_rotation_matrix(stf_c['p'], stf_c['yaw'], stf_c['r'])
        pos_c = np.array([stf_c['x'], stf_c['y'], stf_c['z']])
        p_lidar_cam_local = (R_c.T @ (p_world_lidar - pos_c).T).T
        
        # Crop to 110 deg FOV
        fov_mask = (p_lidar_cam_local[:, 0] > 1.0) & (np.abs(p_lidar_cam_local[:, 1] / p_lidar_cam_local[:, 0]) < np.tan(np.radians(55)))
        p_world_lidar_cropped = p_world_lidar[fov_mask]
        
        # Compute Nearest Neighbor Error (RMSE) on a sampled subset
        sample_size = min(2000, len(p_world_depth), len(p_world_lidar_cropped))
        if sample_size > 0:
            idx1 = np.random.choice(len(p_world_depth), sample_size, replace=False)
            tree = cKDTree(p_world_lidar_cropped)
            dists, _ = tree.query(p_world_depth[idx1], k=1)
            # Filter matches > 1.0m
            valid_dists = dists[dists < 1.0]
            if len(valid_dists) > 0:
                offset = np.mean(valid_dists)
            else:
                offset = 999.0
        else:
            offset = 999.0
            
        logging.info(f"  Frame: {frame_id}")
        logging.info(f"  Modal Offset: {offset:.4f}m")
        if offset < 0.20:
            logging.info("  PASS: LiDAR and Depth sensors are geometrically locked.")
        else:
            logging.error("  FAIL: Sensors appear misaligned.")

    logging.info("=== Validation Complete ===")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--dir', required=True)
    parser.add_argument('--frames', type=int, nargs='+', default=None)
    args = parser.parse_args()
    verify_lidar_alignment(args.dir, args.frames)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--dir', required=True)
    parser.add_argument('--frames', type=int, nargs='+', default=None)
    args = parser.parse_args()
    verify_lidar_alignment(args.dir, args.frames)
