#!/usr/bin/env python3
import airsim
import cv2
import numpy as np
import os
import time
import math

def fly_and_record_street():
    # 1. Setup paths
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(script_dir)
    recordings_dir = os.path.join(project_root, "recordings")
    os.makedirs(recordings_dir, exist_ok=True)
    
    output_filename = os.path.join(recordings_dir, f"drone_street_flight_{int(time.time())}.mp4")
    
    # 2. Connect to AirSim
    print(f"Connecting to AirSim (port 41451)...")
    client = airsim.MultirotorClient(port=41451)
    client.confirmConnection()
    
    # Enable control
    client.enableApiControl(True)
    client.armDisarm(True)
    
    # 3. Initialize Video Writer
    responses = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)])
    if not responses or not responses[0].image_data_uint8:
        print("Error: Could not capture initial frame.")
        return

    response = responses[0]
    height, width = response.height, response.width
    print(f"Resolution: {width}x{height}")
    
    fps = 20.0
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(output_filename, fourcc, fps, (width, height))
    
    # 4. Start Flying!
    # Velocity in body frame: vx=5 (forward), vy=0, vz=0
    # Duration: 20 seconds
    duration = 20
    print(f"Executing flight along the street (5 m/s) for {duration}s...")
    
    # We use moveByVelocityAsync with current orientation to fly "forward"
    # Actually, moveByVelocityBodyFrameAsync is easier if available, but standard is moveByVelocityAsync
    q = client.getMultirotorState().kinematics_estimated.orientation
    yaw = airsim.to_eularian_angles(q)[2]
    vx = 5.0 * math.cos(yaw)
    vy = 5.0 * math.sin(yaw)
    
    # Start movement
    client.moveByVelocityAsync(vx, vy, 0, duration)
    
    # 5. Recording loop (coincides with flight)
    start_time = time.time()
    frames_recorded = 0
    
    try:
        while (time.time() - start_time) < duration:
            loop_start = time.time()
            
            # Capture frame
            responses = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)])
            response = responses[0]
            
            # Convert to BGR
            img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
            img_rgb = img1d.reshape(response.height, response.width, 3)
            img_bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
            
            # Write
            out.write(img_bgr)
            frames_recorded += 1
            
            # Maintain FPS
            elapsed_loop = time.time() - loop_start
            sleep_time = max(1.0/fps - elapsed_loop, 0)
            time.sleep(sleep_time)
            
    except KeyboardInterrupt:
        print("\nManually stopped.")
    finally:
        # Stop drone
        client.moveByVelocityAsync(0, 0, 0, 1)
        client.enableApiControl(False)
        out.release()
        print(f"\nFlight complete. Video saved to: {output_filename}")

if __name__ == "__main__":
    fly_and_record_street()
