"""
AGCarla Route Curation & Optimization
-------------------------------------
Transforms raw recorded trajectories (from manual drive/flight sessions) 
into sparse, optimized route files for automated co-simulation replay.

Key Logic:
- Metadata Extraction: Parses per-frame JSON states from record sessions.
- RDP Thinning: Uses the Ramer-Douglas-Peucker algorithm to reduce 
  high-frequency jitter and create sparse waypoints while maintaining path accuracy.
- Multi-Actor Support: Synchronizes ground truth for UGV and all UAVs.

Usage:
    uv run dataset_generation/route_curation.py --input data/record_session --output curated_route.json --epsilon 0.5
"""
import os
import json
import argparse
import numpy as np
import logging
from datetime import datetime

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')

class RouteCurator:
    def __init__(self, input_dir):
        self.input_dir = input_dir
        self.metadata_dir = os.path.join(input_dir, 'metadata')

    def load_metadata(self):
        """Loads all frame metadata and groups by actor."""
        if not os.path.exists(self.metadata_dir):
            logging.error(f"Metadata directory not found: {self.metadata_dir}")
            return {}
            
        files = sorted([f for f in os.listdir(self.metadata_dir) if f.endswith('.json')])
        actor_tracks = {}

        for f in files:
            with open(os.path.join(self.metadata_dir, f), 'r') as j:
                data = json.load(j)
                
                # UGV
                ugv_id = 'UGV_1' # standard lead id in our generator
                if ugv_id not in actor_tracks: actor_tracks[ugv_id] = []
                u = data['ugv']
                actor_tracks[ugv_id].append({'x': u['x'], 'y': u['y'], 'z': u['z'], 'yaw': u['yaw']})

                # UAVs
                for name, p in data.get('uavs', {}).items():
                    if name not in actor_tracks: actor_tracks[name] = []
                    actor_tracks[name].append({'x': p['x'], 'y': p['y'], 'z': p['z']})

        return actor_tracks

    @staticmethod
    def point_line_distance(p, a, b):
        """Distance from point p to line segment ab."""
        pa = p - a
        ba = b - a
        dot_ba = np.dot(ba, ba)
        if dot_ba == 0:
            return np.linalg.norm(pa)
        t = np.dot(pa, ba) / dot_ba
        t = np.clip(t, 0, 1)
        dist = np.linalg.norm(pa - t * ba)
        return dist

    def rdp_downsample(self, points, epsilon):
        """Ramer-Douglas-Peucker algorithm for thinning."""
        if len(points) < 3:
            return points

        arr = np.array([[p['x'], p['y'], p['z']] for p in points])
        
        def _rdp(pts, eps):
            dmax = 0
            idx = 0
            for i in range(1, len(pts) - 1):
                d = self.point_line_distance(pts[i], pts[0], pts[-1])
                if d > dmax:
                    idx = i
                    dmax = d
            
            if dmax > eps:
                res1 = _rdp(pts[:idx+1], eps)
                res2 = _rdp(pts[idx:], eps)
                return np.vstack([res1[:-1], res2])
            else:
                return np.vstack([pts[0], pts[-1]])

        downsampled_arr = _rdp(arr, epsilon)
        
        # Map back to dicts
        results = []
        for p in downsampled_arr:
            results.append({'x': float(p[0]), 'y': float(p[1]), 'z': float(p[2])})
        return results

    def curate(self, output_path, epsilon=1.0):
        logging.info(f"Loading metadata from {self.input_dir}...")
        tracks = self.load_metadata()
        if not tracks:
            logging.warning("No tracks found to curate.")
            return
        
        route = []
        for actor_id, track in tracks.items():
            logging.info(f"Processing track for {actor_id} (Length: {len(track)})...")
            sparse_path = self.rdp_downsample(track, epsilon)
            
            mode = 'lead' if 'UGV' in actor_id else 'follow'
            entry = {
                'id': actor_id,
                'mode': mode,
                'path': sparse_path
            }
            if mode == 'follow':
                entry['follow_id'] = 'UGV_1'
                entry['temporal_lag'] = 1.0
                entry['offset'] = {'x': -10, 'y': 0, 'z': 20}
            
            parent_dir = os.path.dirname(output_path)
        if parent_dir:
            os.makedirs(parent_dir, exist_ok=True)
        with open(output_path, 'w') as f:
            json.dump(route, f, indent=4)
        logging.info(f"Successfully saved curated route to {output_path}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', required=True, help='Path to record session directory')
    parser.add_argument('--output', required=True, help='Output route.json path')
    parser.add_argument('--epsilon', type=float, default=0.5, help='Tolerance for RDP downsampling')
    args = parser.parse_args()

    curator = RouteCurator(args.input)
    curator.curate(args.output, args.epsilon)
