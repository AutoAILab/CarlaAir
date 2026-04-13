#!/usr/bin/env python3
import airsim
import cv2
import numpy as np
import os
import time
import math

def takeoff_fly_record():
    # 1. Setup paths
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(script_dir)
    recordings_dir = os.path.join(project_root, "recordings")
    os.makedirs(recordings_dir, exist_ok=True)
    
    output_filename = os.path.join(recordings_dir, f"drone_flight_fixed_{int(time.time())}.mp4")
    
    # 2. Connect to AirSim
    print(f"Connecting to AirSim (port 41451)...")
    client = airsim.MultirotorClient(port=41451)
    client.confirmConnection()
    
    # Enable control and arm
    print("Enabling API control and arming...")
    client.enableApiControl(True)
    client.armDisarm(True)
    
    # 3. Takeoff and move to altitude
    print("Taking off...")
    client.takeoffAsync().join()
    
    print("Moving to 15m altitude...")
    # NED: -15 is 15m above the origin
    client.moveToZAsync(-15, 5).join()
    
    # 4. Initialize Video Writer
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
    
    # 5. Start Flying!
    duration = 20
    print(f"Executing flight (5 m/s) for {duration}s...")
    
    # Get current orientation to fly forward
    q = client.getMultirotorState().kinematics_estimated.orientation
    yaw = airsim.to_eularian_angles(q)[2]
    vx = 5.0 * math.cos(yaw)
    vy = 5.0 * math.sin(yaw)
    
    # Start movement
    client.moveByVelocityAsync(vx, vy, 0, duration)
    
    # 6. Recording loop
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
    takeoff_fly_record()
