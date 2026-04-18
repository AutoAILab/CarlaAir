import carla
import airsim
import time

def check_full_alignment():
    client_carla = carla.Client("localhost", 2000)
    world = client_carla.get_world()
    client_air = airsim.MultirotorClient(port=41451)
    
    ugv = None
    drone = None
    
    for a in world.get_actors():
        if "tesla" in a.type_id.lower():
            ugv = a
        if "drone" in a.type_id.lower() or "airsim" in a.type_id.lower():
            drone = a
            
    if not ugv or not drone:
        print("Missing UGV or Drone.")
        return

    # CARLA Poses
    cl_ugv = ugv.get_location()
    cl_drone = drone.get_location()
    
    # AirSim Pose
    ap_drone = client_air.getMultirotorState().kinematics_estimated.position
    
    print(f"CARLA Ref: UGV @ ({cl_ugv.x:.2f}, {cl_ugv.y:.2f}, {cl_ugv.z:.2f})")
    print(f"CARLA Ref: Drone @ ({cl_drone.x:.2f}, {cl_drone.y:.2f}, {cl_drone.z:.2f})")
    print(f"AirSim Ref: Drone @ ({ap_drone.x_val:.2f}, {ap_drone.y_val:.2f}, {ap_drone.z_val:.2f})")

    # Calculated Offset (AirSim - CARLA)
    ox = ap_drone.x_val - cl_drone.x
    oy = ap_drone.y_val - cl_drone.y
    oz = ap_drone.z_val - (-cl_drone.z)
    
    print(f"Calculated Offset for Geometry: {{'x': {ox:.4f}, 'y': {oy:.4f}, 'z': {oz:.4f}}}")
    
    # Where SHOULD the UGV be in AirSim NED?
    # AirSim = CARLA + Offset
    # but Z is negated
    ax_ugv = cl_ugv.x + ox
    ay_ugv = cl_ugv.y + oy
    az_ugv = -cl_ugv.z + oz
    
    print(f"Expected UGV in AirSim NED: ({ax_ugv:.2f}, {ay_ugv:.2f}, {az_ugv:.2f})")

if __name__ == "__main__":
    check_full_alignment()
