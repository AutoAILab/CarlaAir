"""
AGCarla Production Route Planner
--------------------------------
Generates high-fidelity trajectory bundles for multi-actor co-simulation.

Features:
- Global path planning between CARLA spawn points.
- Automatic swarm bootstrapping (1 UGV + 5 UAVs).
- Cluster-based temporal lag configuration:
    - Cluster A (UAV_1, UAV_2): Close proximity, 0.5s lag.
    - Cluster B (UAV_3, UAV_4, UAV_5): High altitude, 2.0s lag.

Usage:
    uv run dataset_generation/generate_complex_route.py --start 0 --end 10 --speed 15.0 --out route.json
"""
import sys
import os
import json
import argparse
import carla
import numpy as np
import logging

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')

# Ensure CARLA agents are in path
# Try to find the absolute path to the repo root
BASE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
AGENT_PATH = os.path.join(BASE_DIR, 'PythonAPI', 'carla')
sys.path.append(AGENT_PATH)

try:
    from agents.navigation.global_route_planner import GlobalRoutePlanner
except ImportError as e:
    logging.error(f"Could not import GlobalRoutePlanner: {e}")
    logging.error(f"Looked in: {AGENT_PATH}")
    logging.error(f"sys.path current: {sys.path}")
    sys.exit(1)

class ProductionRoutePlanner:
    def __init__(self, host='127.0.0.1', port=2000):
        self.client = carla.Client(host, port)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        self.map = self.world.get_map()
        self.sampling_resolution = 2.0
        self.grp = GlobalRoutePlanner(self.map, self.sampling_resolution)

    def generate_route(self, start_idx, end_idx, speed=5.0):
        spawn_points = self.map.get_spawn_points()
        if start_idx >= len(spawn_points) or end_idx >= len(spawn_points):
            raise ValueError(f"Invalid spawn point indices. Max is {len(spawn_points)-1}")

        start_loc = spawn_points[start_idx].location
        end_loc = spawn_points[end_idx].location
        
        logging.info(f"Planning route from SP {start_idx} to SP {end_idx}...")
        results = self.grp.trace_route(start_loc, end_loc)
        
        path = []
        for wp, road_option in results:
            path.append({
                'x': float(wp.transform.location.x),
                'y': float(wp.transform.location.y),
                'z': float(wp.transform.location.z),
                'yaw': float(wp.transform.rotation.yaw)
            })
            
        return path

    def create_production_bundle(self, path, output_path, speed=5.0):
        # Cluster Lag Strategy: 2 drones at 0.5s, 3 drones at 2.0s
        route_config = [
            {
                "id": "UGV_1",
                "mode": "lead",
                "path": path,
                "speed": speed
            },
            # Cluster 1: Close Follow (0.5s lag)
            {
                "id": "UAV_1",
                "mode": "follow",
                "follow_id": "UGV_1",
                "temporal_lag": 0.5,
                "offset": {"x": -10, "y": -5, "z": 15}
            },
            {
                "id": "UAV_2",
                "mode": "follow",
                "follow_id": "UGV_1",
                "temporal_lag": 0.5,
                "offset": {"x": -10, "y": 5, "z": 15}
            },
            # Cluster 2: Far Follow (2.0s lag)
            {
                "id": "UAV_3",
                "mode": "follow",
                "follow_id": "UGV_1",
                "temporal_lag": 2.0,
                "offset": {"x": -20, "y": -15, "z": 30}
            },
            {
                "id": "UAV_4",
                "mode": "follow",
                "follow_id": "UGV_1",
                "temporal_lag": 2.0,
                "offset": {"x": -20, "y": 15, "z": 30}
            },
            {
                "id": "UAV_5",
                "mode": "follow",
                "follow_id": "UGV_1",
                "temporal_lag": 2.0,
                "offset": {"x": -5, "y": 0, "z": 50}
            }
        ]

        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        with open(output_path, 'w') as f:
            json.dump(route_config, f, indent=4)
        logging.info(f"Production route saved to {output_path}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--start', type=int, default=0)
    parser.add_argument('--end', type=int, default=6)
    parser.add_argument('--speed', type=float, default=15.0)
    parser.add_argument('--out', required=True)
    args = parser.parse_args()

    planner = None
    try:
        planner = ProductionRoutePlanner()
        path = planner.generate_route(args.start, args.end, args.speed)
        planner.create_production_bundle(path, args.out, args.speed)
    except Exception as e:
        logging.error(f"Error during route generation: {e}")
    finally:
        if planner:
            logging.info("Cleaning up ProductionRoutePlanner...")
            # Any specific cleanup for planner can go here
            del planner

