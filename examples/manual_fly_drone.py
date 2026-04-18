#!/usr/bin/env python3
"""
manual_fly_drone.py — Manual Drone Flight with Pygame
======================================================
Control the drone using your keyboard while seeing the FPV view.
Bypasses the need for the main simulator window.

Controls:
    W / S           Forward / Backward
    A / D           Strafe Left / Right
    Space / Shift   Ascend / Descend
    Q / E           Yaw (Turn) Left / Right
    X               Toggle Path Recording (Save for Datagen)
    N               Next Weather
    ESC             Quit (Land & Disarm)

Usage:
    source .venv/bin/activate
    python3 examples/manual_fly_drone.py
"""

import carla
import airsim
import pygame
import numpy as np
import math
import time

W, H = 1280, 720
MAX_VEL = 15.0  # m/s
YAW_RATE = 40.0 # deg/s

WEATHERS = [
    ("Clear Day",    carla.WeatherParameters.ClearNoon),
    ("Sunset",       carla.WeatherParameters(
        cloudiness=30, precipitation=0, precipitation_deposits=0,
        wind_intensity=30, sun_azimuth_angle=180, sun_altitude_angle=5,
        fog_density=10, fog_distance=50, fog_falloff=2, wetness=0)),
    ("Rainy Day",    carla.WeatherParameters.HardRainNoon),
    ("Dense Fog",    carla.WeatherParameters(
        cloudiness=90, precipitation=0, precipitation_deposits=0,
        wind_intensity=10, sun_azimuth_angle=0, sun_altitude_angle=45,
        fog_density=80, fog_distance=10, fog_falloff=1, wetness=0)),
    ("Night",        carla.WeatherParameters(
        cloudiness=10, precipitation=0, precipitation_deposits=0,
        wind_intensity=5, sun_azimuth_angle=0, sun_altitude_angle=-90,
        fog_density=2, fog_distance=0, fog_falloff=0, wetness=0)),
]

def main():
    try:
        # 1. Connect
        print("\n  Connecting to CarlaAir...")
        carla_client = carla.Client("localhost", 2000)
        carla_client.set_timeout(10.0)
        world = carla_client.get_world()

        air_client = airsim.MultirotorClient(port=41451)
        air_client.confirmConnection()
        air_client.enableApiControl(True)
        air_client.armDisarm(True)

        print("  Taking off...")
        air_client.takeoffAsync().join()

        # 2. Pygame Setup
        pygame.init()
        display = pygame.display.set_mode((W, H))
        pygame.display.set_caption("CarlaAir — Drone Pilot | WASD=Fly Q/E=Yaw Space/Shift=Z N=Weather ESC=Quit")
        clock = pygame.time.Clock()
        font = pygame.font.SysFont("monospace", 18, bold=True)

        weather_idx = 0
        world.set_weather(WEATHERS[0][1])
        
        # ── Recording & Calibration ──
        global_offset = calibrate_offsets(world, air_client)
        recording = False
        recorded_path = []
        last_record_time = 0
        RECORD_HZ = 20
        RECORD_DT = 1.0 / RECORD_HZ
        
        running = True
        while running:
            clock.tick(30)
            now = time.perf_counter()

            # 3. Events
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    running = False
                elif ev.type == pygame.KEYDOWN:
                    if ev.key == pygame.K_ESCAPE:
                        running = False
                    elif ev.key == pygame.K_n:
                        weather_idx = (weather_idx + 1) % len(WEATHERS)
                        world.set_weather(WEATHERS[weather_idx][1])
                    elif ev.key == pygame.K_x:
                        recording = not recording
                        if recording:
                            recorded_path = []
                            print("  >>> Recording Started (X)")
                        else:
                            print(f"  >>> Recording Stopped. Captured {len(recorded_path)} waypoints.")
                            save_route(recorded_path, world.get_map().name.split('/')[-1])

            # 4. Input handling
            keys = pygame.key.get_pressed()
            
            vx = 0.0
            vy = 0.0
            vz = 0.0
            yaw_v = 0.0

            if keys[pygame.K_w]:     vx = MAX_VEL
            elif keys[pygame.K_s]:   vx = -MAX_VEL
            
            if keys[pygame.K_a]:     vy = -MAX_VEL
            elif keys[pygame.K_d]:   vy = MAX_VEL
            
            if keys[pygame.K_SPACE]: vz = -MAX_VEL   # NED coordinates! -Z is Up
            elif keys[pygame.K_LSHIFT] or keys[pygame.K_RSHIFT]: vz = MAX_VEL
            
            if keys[pygame.K_q]:     yaw_v = -YAW_RATE
            elif keys[pygame.K_e]:   yaw_v = YAW_RATE

            # Send command to AirSim (Body frame makes control intuitive)
            air_client.moveByVelocityBodyFrameAsync(vx, vy, vz, 0.1, 
                airsim.DrivetrainType.MaxDegreeOfFreedom, 
                airsim.YawMode(True, yaw_v))

            # ── Recording Logic (20Hz) ──
            if recording and (now - last_record_time >= RECORD_DT):
                last_record_time = now
                state = air_client.getMultirotorState()
                ap = state.kinematics_estimated.position
                ao = state.kinematics_estimated.orientation
                
                # Convert AirSim NED to CARLA World
                # cl.x = ap.x_val - offset_x
                # cl.y = ap.y_val - offset_y
                # cl.z = -(ap.z_val - offset_z)
                cx = ap.x_val - global_offset['x']
                cy = ap.y_val - global_offset['y']
                cz = -(ap.z_val - global_offset['z'])
                
                # Convert Quat to Yaw
                siny_cosp = 2 * (ao.w_val * ao.z_val + ao.x_val * ao.y_val)
                cosy_cosp = 1 - 2 * (ao.y_val * ao.y_val + ao.z_val * ao.z_val)
                yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))

                recorded_path.append({
                    "x": round(cx, 4), "y": round(cy, 4), "z": round(cz, 4), "yaw": round(yaw, 4)
                })

            # 5. Capture & Render
            # Request RGB image from AirSim (Camera 0)
            responses = air_client.simGetImages([
                airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)
            ])
            
            if responses and responses[0].width > 0:
                img1d = np.frombuffer(responses[0].image_data_uint8, dtype=np.uint8)
                img_rgb = img1d.reshape(responses[0].height, responses[0].width, 3)
                
                # Scale if needed
                surf = pygame.surfarray.make_surface(img_rgb.swapaxes(0, 1))
                if responses[0].width != W or responses[0].height != H:
                    surf = pygame.transform.scale(surf, (W, H))
                display.blit(surf, (0, 0))
            else:
                display.fill((20, 20, 30))
                msg = font.render("Waiting for camera stream...", True, (200, 200, 200))
                display.blit(msg, (W//2 - 100, H//2))

            # 6. HUD
            state = air_client.getMultirotorState()
            pos = state.kinematics_estimated.position
            vel = state.kinematics_estimated.linear_velocity
            speed = math.sqrt(vel.x_val**2 + vel.y_val**2 + vel.z_val**2) * 3.6 # km/h
            
            # Record indicator
            rec_str = ""
            if recording:
                dot = "●" if int(time.time() * 2) % 2 == 0 else " "
                rec_str = f" | {dot} REC ({len(recorded_path)})"

            hud = (f"WP: {WEATHERS[weather_idx][0]} | Spd: {speed:.1f} km/h{rec_str} | "
                   f"Alt: {-pos.z_val:.1f}m | WASD: Move  Q/E: Yaw  X: Record  ESC: Land")
            
            rec_color = (255, 50, 50) if recording else (0, 255, 180)
            hs = font.render(hud, True, rec_color)
            hbg = pygame.Surface((W, 26)); hbg.set_alpha(180); hbg.fill((0, 0, 0))
            display.blit(hbg, (0, H - 26))
            display.blit(hs, (10, H - 24))

            pygame.display.flip()

    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Final save if still recording
        if recording and recorded_path:
            save_route(recorded_path, world.get_map().name.split('/')[-1])
        
        print("\n  Landing & Disarming...")
        try:
            air_client.moveByVelocityAsync(0, 0, 0, 1).join()
            air_client.landAsync().join()
            air_client.armDisarm(False)
            air_client.enableApiControl(False)
        except: pass
        try: pygame.quit()
        except: pass
        print("  Done.\n")



def calibrate_offsets(world, air_client):
    """Calculates the translation offset between CARLA world origin and AirSim local origin."""
    drone_actor = None
    for a in world.get_actors():
        if "drone" in a.type_id.lower() or "airsim" in a.type_id.lower():
            drone_actor = a
            break
    
    if not drone_actor:
        print("Warning: Could not find AirSim drone in CARLA world for calibration.")
        return {'x': 0.0, 'y': 0.0, 'z': 0.0}
        
    cl = drone_actor.get_location()
    ap = air_client.getMultirotorState().kinematics_estimated.position
    
    # offset = AirSim - CARLA
    return {
        'x': ap.x_val - cl.x,
        'y': ap.y_val - cl.y,
        'z': ap.z_val - (-cl.z)
    }

def save_route(path, map_name):
    """Saves the drone path in a format compatible with agcarla_datagen.py"""
    import json
    import os
    from datetime import datetime
    
    if not path:
        return

    out_dir = "/home/df/data/jflinte/CarlaAir/dataset_generation/trajectories"
    os.makedirs(out_dir, exist_ok=True)
    
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = os.path.join(out_dir, f"route_drone_{ts}.json")
    
    route_data = [
        {
            "id": "UAV_1",
            "mode": "lead",
            "type": "drone",
            "path": path
        }
    ]
    
    with open(filename, 'w', encoding='utf-8') as f:
        json.dump(route_data, f, indent=4)
    
    print(f"\n  [SAVE] Drone Route saved: {filename}")
    print(f"  Total Waypoints: {len(path)}")

if __name__ == "__main__":
    main()
