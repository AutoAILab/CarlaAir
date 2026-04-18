import carla
import airsim
import time

def check_poses():
    # CARLA
    client_carla = carla.Client("localhost", 2000)
    world = client_carla.get_world()
    
    # AirSim
    client_air = airsim.MultirotorClient(port=41451)
    
    # Find drone in CARLA
    drone_actor = None
    for a in world.get_actors():
        if "drone" in a.type_id.lower() or "airsim" in a.type_id.lower():
            drone_actor = a
            break
            
    if not drone_actor:
        print("No drone found.")
        return

    # Take 5 samples
    for i in range(5):
        cl = drone_actor.get_location()
        cr = drone_actor.get_transform().rotation
        
        ap = client_air.getMultirotorState().kinematics_estimated.position
        aq = client_air.getMultirotorState().kinematics_estimated.orientation
        
        print(f"\nSample {i}:")
        print(f"  CARLA: Loc(x={cl.x:.2f}, y={cl.y:.2f}, z={cl.z:.2f}) | Rot(p={cr.pitch:.2f}, y={cr.yaw:.2f}, r={cr.roll:.2f})")
        print(f"  AirSim: Loc(x={ap.x_val:.2f}, y={ap.y_val:.2f}, z={ap.z_val:.2f}) | Quat(w={aq.w_val:.2f}, x={aq.x_val:.2f}, y={aq.y_val:.2f}, z={aq.z_val:.2f})")
        
        time.sleep(1.0)

if __name__ == "__main__":
    check_poses()
