import carla
import numpy as np
import time

def main():
    try:
        client = carla.Client("localhost", 2000)
        client.set_timeout(10.0)
        world = client.get_world()
        print(f"Connected to world: {world.get_map().name}")

        bp_lib = world.get_blueprint_library()
        cam_bp = bp_lib.find("sensor.camera.rgb")
        
        # Spawn camera at origin
        spawn_point = carla.Transform(carla.Location(z=10), carla.Rotation(pitch=-90))
        camera = world.spawn_actor(cam_bp, spawn_point)
        
        image_data = {"received": False}
        def callback(image):
            image_data["received"] = True
            print(f"Received image: {image.width}x{image.height}")
            
        camera.listen(callback)
        
        print("Waiting for image...")
        for _ in range(20):
            if image_data["received"]:
                break
            time.sleep(0.5)
            
        if not image_data["received"]:
            print("FAILED to receive image from camera.")
        else:
            print("SUCCESS: Camera is rendering and sending data.")
            
        camera.stop()
        camera.destroy()
        
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()
