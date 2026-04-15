import carla
import airsim
import time

def test_sync():
    print("Connecting to CARLA...")
    carla_client = carla.Client('localhost', 2000)
    carla_client.set_timeout(10.0)
    world = carla_client.get_world()
    
    print("Connecting to AirSim...")
    air_client = airsim.MultirotorClient(port=41451)
    air_client.confirmConnection()
    
    print("Setting Synchronous Mode...")
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)
    
    try:
        print("Ticking World...")
        world.tick()
        time.sleep(0.5)
        
        print("Requesting Image from UAV_1...")
        # Minimalist request
        responses = air_client.simGetImages([
            airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)
        ], vehicle_name="UAV_1")
        
        if responses:
            print(f"Success! Captured image {responses[0].width}x{responses[0].height}")
        else:
            print("Failed: No response.")
            
    finally:
        print("Disabling Synchronous Mode...")
        settings.synchronous_mode = False
        world.apply_settings(settings)

if __name__ == "__main__":
    test_sync()
