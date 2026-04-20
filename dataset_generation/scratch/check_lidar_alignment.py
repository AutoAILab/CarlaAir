import numpy as np
import os
import json
import argparse
import sys
# Add parent dir to path for imports
sys.path.append(os.path.join(os.getcwd(), 'dataset_generation'))
from geometry import GeometryUtils

def check_temporal_alignment(data_dir, f1=0, f2=1):
    print(f"Checking alignment between Frame {f1} and Frame {f2}...")
    
    with open(os.path.join(data_dir, 'metadata', f'{f1:06d}.json'), 'r') as f:
        m1 = json.load(f)
    with open(os.path.join(data_dir, 'metadata', f'{f2:06d}.json'), 'r') as f:
        m2 = json.load(f)
        
    p1_raw = np.load(os.path.join(data_dir, 'lidar', f'ugv_{f1:06d}.npy'))[:, :3]
    p2_raw = np.load(os.path.join(data_dir, 'lidar', f'ugv_{f2:06d}.npy'))[:, :3]
    
    # Transform to Actor (mount)
    p1_actor = p1_raw + np.array([0, 0, 2.5])
    p2_actor = p2_raw + np.array([0, 0, 2.5])
    
    # Transform to World
    p1_world = GeometryUtils.camera_to_world(p1_actor, m1['ugv'], is_airsim=False)
    p2_world = GeometryUtils.camera_to_world(p2_actor, m2['ugv'], is_airsim=False)
    
    # Save both for visual inspection in CloudCompare or MeshLab
    out_dir = os.path.join(data_dir, 'verification')
    os.makedirs(out_dir, exist_ok=True)
    
    out1 = os.path.join(out_dir, f'debug_f{f1}.ply')
    out2 = os.path.join(out_dir, f'debug_f{f2}.ply')
    
    GeometryUtils.save_ply(out1, p1_world)
    GeometryUtils.save_ply(out2, p2_world)
    
    # Simple metric: distance between centroids of a "stable" patch (e.g. far points)
    c1 = np.mean(p1_world, axis=0)
    c2 = np.mean(p2_world, axis=0)
    dist = np.linalg.norm(c1 - c2)
    
    print(f"Centroid F{f1}: {c1}")
    print(f"Centroid F{f2}: {c2}")
    print(f"Centroid Shift: {dist:.4f}m")
    
    # If the UGV moved, the centroid shift should match the UGV displacement
    ugv_disp = np.linalg.norm(np.array([m1['ugv']['x'], m1['ugv']['y']]) - np.array([m2['ugv']['x'], m2['ugv']['y']]))
    print(f"UGV Displacement: {ugv_disp:.4f}m")
    
    if dist < 0.1 + ugv_disp: # Allow some tolerance
        print("PASS: Lidar frames appear consistent.")
    else:
        print("FAIL: Frames are drifting significantly!")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--dir', required=True)
    parser.add_argument('--f1', type=int, default=0)
    parser.add_argument('--f2', type=int, default=1)
    args = parser.parse_args()
    check_temporal_alignment(args.dir, args.f1, args.f2)
