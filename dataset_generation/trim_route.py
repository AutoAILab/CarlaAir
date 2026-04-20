import json
import sys
import os
import logging

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')

def trim_route(file_path):
    if not os.path.exists(file_path):
        logging.error(f"{file_path} not found.")
        return

    with open(file_path, 'r') as f:
        data = json.load(f)

    if not data or 'path' not in data[0]:
        logging.error("Invalid route format.")
        return

    path = data[0]['path']
    start_idx = 0
    first_wp = path[0]
    
    # Sensitivity threshold for motion
    threshold = 0.01 

    for i, wp in enumerate(path):
        dx = abs(wp['x'] - first_wp['x'])
        dy = abs(wp['y'] - first_wp['y'])
        # We check X and Y primarily. Z might drift slightly due to hover noise.
        if dx > threshold or dy > threshold:
            start_idx = i
            break
            
    if start_idx == 0:
        logging.info("No significant stationary period found at start.")
        return

    logging.info(f"Trimming {start_idx} stationary frames from the start.")
    data[0]['path'] = path[start_idx:]
    
    with open(file_path, 'w') as f:
        json.dump(data, f, indent=4)
    logging.info("Done.")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        logging.error("Usage: python3 trim_route.py <path_to_json>")
    else:
        trim_route(sys.argv[1])
