#!/usr/bin/env python3
"""
manual_drive_vehicle.py — Manual Vehicle Driving with Pygame
=============================================================
Control a ground vehicle using your keyboard while seeing a chase camera.
Bypasses the need for the main simulator window.

Controls:
    W / S           Throttle / Brake
    A / D           Steering Left / Right
    Space           Handbrake
    R               Toggle Reverse Gear
    X               Toggle Path Recording (Save for Datagen)
    N               Next Weather
    ESC             Quit (Cleanup & Exit)

Usage:
    source .venv/bin/activate
    python3 examples/manual_drive_vehicle.py
"""

import carla
import pygame
import numpy as np
import math
import time

W, H = 1280, 720

WEATHERS = [
    ("Clear Day",    carla.WeatherParameters.ClearNoon),
    ("Sunset",       carla.WeatherParameters(
        cloudiness=30, precipitation=0, precipitation_deposits=0,
        wind_intensity=30, sun_azimuth_angle=180, sun_altitude_angle=5,
        fog_density=10, fog_distance=50, fog_falloff=2, wetness=0)),
    ("Hard Rain",    carla.WeatherParameters.HardRainNoon),
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
    actors = []
    try:
        # 1. Connect to CARLA
        print("\n  Connecting to CarlaAir...")
        client = carla.Client("localhost", 2000)
        client.set_timeout(10.0)
        world = client.get_world()
        bp_lib = world.get_blueprint_library()

        # Cleanup existing actors from previous runs if any
        for a in world.get_actors().filter("vehicle.*"):
            if a.attributes.get('role_name') == 'manual_pilot':
                a.destroy()

        # 2. Spawn Vehicle
        vehicle_bp = bp_lib.find("vehicle.tesla.model3")
        vehicle_bp.set_attribute('role_name', 'manual_pilot')
        
        spawn_points = world.get_map().get_spawn_points()
        vehicle = None
        for sp in spawn_points:
            try:
                vehicle = world.spawn_actor(vehicle_bp, sp)
                break
            except RuntimeError:
                continue
        
        if not vehicle:
            raise RuntimeError("Cannot spawn vehicle — all spawn points occupied.")
        actors.append(vehicle)
        print(f"  Vehicle: Tesla Model 3 spawned.")

        # 3. Setup Camera (Chase view)
        cam_bp = bp_lib.find("sensor.camera.rgb")
        cam_bp.set_attribute("image_size_x", str(W))
        cam_bp.set_attribute("image_size_y", str(H))
        cam_bp.set_attribute("fov", "100")
        
        # Chase offset: 6m back, 3m up
        cam_tf = carla.Transform(carla.Location(x=-6.5, z=2.8), carla.Rotation(pitch=-15))
        camera = world.spawn_actor(cam_bp, cam_tf, attach_to=vehicle)
        actors.append(camera)

        image_data = [None]
        def on_image(img):
            arr = np.frombuffer(img.raw_data, dtype=np.uint8)
            image_data[0] = arr.reshape((img.height, img.width, 4))[:, :, :3][:, :, ::-1]
        
        camera.listen(on_image)

        # 4. Pygame Setup
        pygame.init()
        display = pygame.display.set_mode((W, H))
        pygame.display.set_caption("CarlaAir — Vehicle Pilot | WASD=Drive Space=Brake R=Reverse N=Weather ESC=Quit")
        clock = pygame.time.Clock()
        font = pygame.font.SysFont("monospace", 18, bold=True)

        weather_idx = 0
        world.set_weather(WEATHERS[0][1])
        
        # ── Recording State ──
        recording = False
        recorded_path = []
        last_record_time = 0
        RECORD_HZ = 20
        RECORD_DT = 1.0 / RECORD_HZ
        
        reverse = False
        running = True
        while running:
            clock.tick(60)
            now = time.perf_counter()

            # 5. Events
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    running = False
                elif ev.type == pygame.KEYDOWN:
                    if ev.key == pygame.K_ESCAPE:
                        running = False
                    elif ev.key == pygame.K_n:
                        weather_idx = (weather_idx + 1) % len(WEATHERS)
                        world.set_weather(WEATHERS[weather_idx][1])
                    elif ev.key == pygame.K_r:
                        reverse = not reverse
                        print(f"  Gear: {'Reverse' if reverse else 'Drive'}")
                    elif ev.key == pygame.K_x:
                        recording = not recording
                        if recording:
                            recorded_path = []
                            print("  >>> Recording Started (X)")
                        else:
                            print(f"  >>> Recording Stopped. Captured {len(recorded_path)} waypoints.")
                            save_route(recorded_path, world.get_map().name.split('/')[-1])

            # 6. Input Handling
            keys = pygame.key.get_pressed()
            
            control = carla.VehicleControl()
            # Throttle / Brake
            if keys[pygame.K_w]:
                control.throttle = 0.75
                control.brake = 0.0
            elif keys[pygame.K_s]:
                control.throttle = 0.0
                control.brake = 1.0
            else:
                control.throttle = 0.0
                control.brake = 0.0
            
            # Steering
            if keys[pygame.K_a]:
                control.steer = -0.5
            elif keys[pygame.K_d]:
                control.steer = 0.5
            else:
                control.steer = 0.0
            
            control.hand_brake = keys[pygame.K_SPACE]
            control.reverse = reverse
            
            vehicle.apply_control(control)

            # 7. Recording Logic (20Hz)
            if recording and (now - last_record_time >= RECORD_DT):
                last_record_time = now
                tf = vehicle.get_transform()
                recorded_path.append({
                    "x": round(tf.location.x, 4),
                    "y": round(tf.location.y, 4),
                    "z": round(tf.location.z, 4),
                    "yaw": round(tf.rotation.yaw, 4)
                })

            # 7. Rendering
            if image_data[0] is not None:
                surf = pygame.surfarray.make_surface(image_data[0].swapaxes(0, 1))
                display.blit(surf, (0, 0))
            else:
                display.fill((15, 15, 20))
                msg = font.render("Connecting to camera stream...", True, (200, 200, 200))
                display.blit(msg, (W//2 - 120, H//2))

            # 8. HUD
            vel = vehicle.get_velocity()
            speed = 3.6 * math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
            gear = "R" if reverse else "D"
            
            # Record indicator
            rec_str = ""
            if recording:
                # Flash the REC dot every second
                dot = "●" if int(time.time() * 2) % 2 == 0 else " "
                rec_str = f" | {dot} REC ({len(recorded_path)})"

            hud = (f"WP: {WEATHERS[weather_idx][0]} | Gear: {gear} | Spd: {speed:.1f} km/h{rec_str} | "
                   f"WASD: Drive  R: Reverse  X: Record  N: Weather  ESC: Quit")
            
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
        
        print("\n  Cleaning up actors...")
        for a in actors:
            try:
                if hasattr(a, 'stop'): a.stop()
            except: pass
            try: a.destroy()
            except: pass
        try: pygame.quit()
        except: pass
        print("  Done.\n")



def save_route(path, map_name):
    """Saves the path in the format expected by agcarla_datagen.py"""
    import json
    import os
    from datetime import datetime
    
    if not path:
        return

    out_dir = "/home/df/data/jflinte/CarlaAir/dataset_generation/trajectories"
    os.makedirs(out_dir, exist_ok=True)
    
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = os.path.join(out_dir, f"route_manual_{ts}.json")
    
    # agcarla_datagen.py expects an array of actor configurations
    route_data = [
        {
            "id": "UGV_1",
            "mode": "lead",
            "type": "vehicle",
            "speed": 15.0, # Target speed for datagen physics
            "path": path
        },
        # UAV Followers (Auto-chase configuration)
        {"id": "UAV_1", "mode": "follow", "follow_id": "UGV_1", "temporal_lag": 1.0, "offset": {"x": -10, "y": 0, "z": 20}},
        {"id": "UAV_2", "mode": "follow", "follow_id": "UGV_1", "temporal_lag": 1.5, "offset": {"x": -15, "y": 5, "z": 25}},
        {"id": "UAV_3", "mode": "follow", "follow_id": "UGV_1", "temporal_lag": 2.0, "offset": {"x": -15, "y": -5, "z": 30}},
        {"id": "UAV_4", "mode": "follow", "follow_id": "UGV_1", "temporal_lag": 2.5, "offset": {"x": -20, "y": 0, "z": 40}},
        {"id": "UAV_5", "mode": "follow", "follow_id": "UGV_1", "temporal_lag": 3.0, "offset": {"x": 0, "y": 0, "z": 90}}
    ]
    
    with open(filename, 'w', encoding='utf-8') as f:
        json.dump(route_data, f, indent=4)
    
    print(f"\n  [SAVE] Route saved for agcarla_datagen.py: {filename}")
    print(f"  Total Waypoints: {len(path)}")

if __name__ == "__main__":
    main()
