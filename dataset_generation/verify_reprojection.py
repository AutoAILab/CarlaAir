import numpy as np
import os
import json
import argparse
import cv2
from PIL import Image
import sys
import logging

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')

# Ensure we can find GeometryUtils
sys.path.append(os.getcwd())
from dataset_generation.geometry import GeometryUtils

def verify_reprojection(data_dir, frame_id=5):
    logging.info(f"=== Reprojection Alignment Diagnostic ===")
    logging.info(f"Directory: {data_dir} | Frame: {frame_id}")
    
    metadata_path = os.path.join(data_dir, 'metadata', f'{frame_id:06d}.json')
    if not os.path.exists(metadata_path):
        logging.error(f"Metadata not found at {metadata_path}")
        return

    with open(metadata_path, 'r') as f:
        meta = json.load(f)
    
    # 1. Load Data
    lidar_path = os.path.join(data_dir, 'lidar', f'ugv_{frame_id:06d}.npy')
    rgb_path = os.path.join(data_dir, 'images', f'ugv_{frame_id:06d}.png')
    
    if not os.path.exists(lidar_path) or not os.path.exists(rgb_path):
        logging.error("LiDAR or RGB files missing.")
        return
        
    lidar_pts = np.load(lidar_path)[:, :3]
    rgb_img = cv2.imread(rgb_path)
    if rgb_img is None:
        logging.error(f"Could not load image: {rgb_path}")
        return
    h, w, _ = rgb_img.shape
    
    # 2. Project LiDAR to World
    stf_lidar = meta.get('sensor_tf', {}).get('ugv_lidar')
    if not stf_lidar:
        logging.error("No absolute sensor_tf found in metadata. Run a new capture.")
        return
        
    p_world = GeometryUtils.camera_to_world(lidar_pts, stf_lidar, is_airsim=False)
    
    # 3. Project World to Camera Local
    stf_cam = meta.get('sensor_tf', {}).get('ugv_rgb')
    if not stf_cam:
        logging.error("No camera sensor_tf found in metadata.")
        return
    
    pos_cam = np.array([stf_cam['x'], stf_cam['y'], stf_cam['z']])
    R_cam = GeometryUtils.get_rotation_matrix(stf_cam['p'], stf_cam['yaw'], stf_cam['r'])
    # R is orthogonal, so inverse is transpose
    R_cam_inv = R_cam.T
    
    p_cam_local = (R_cam_inv @ (p_world - pos_cam).T).T
    
    # 4. Project Camera Local to Image (Pinhole 110 FOV)
    # CARLA Camera: X=Forward, Y=Right, Z=Up
    fov = 110
    f = w / (2.0 * np.tan(fov * np.pi / 360.0))
    
    # Filter points behind camera (x <= 0)
    front_mask = p_cam_local[:, 0] > 0.1
    p_front = p_cam_local[front_mask]
    
    u = (w / 2.0) + (p_front[:, 1] * f / p_front[:, 0])
    v = (h / 2.0) - (p_front[:, 2] * f / p_front[:, 0])
    
    # 5. Draw
    for i in range(len(u)):
        ix, iy = int(u[i]), int(v[i])
        if 0 <= ix < w and 0 <= iy < h:
            # Color by distance (X)
            dist = p_front[i, 0]
            color = (int(min(255, dist*10)), 255, 0) # Green-ish
            cv2.circle(rgb_img, (ix, iy), 1, color, -1)

    out_path = f"reprojection_debug_{frame_id}.png"
    cv2.imwrite(out_path, rgb_img)
    logging.info(f"Reprojection debug image saved to: {out_path}")
    logging.info("Diagnostic Complete.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--dir', required=True)
    parser.add_argument('--frame', type=int, default=5)
    args = parser.parse_args()
    verify_reprojection(args.dir, args.frame)
