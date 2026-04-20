import carla
import numpy as np

def get_true_carla_matrix():
    # Create a rotation: Yaw = 30 degrees, others 0
    rot = carla.Rotation(pitch=0, yaw=30, roll=0)
    # Get matrix from a transform
    trans = carla.Transform(carla.Location(0,0,0), rot)
    matrix = trans.get_matrix()
    print("CARLA 4x4 for Yaw=30:")
    print(np.array(matrix))
    
    # Check another one: Pitch = 30
    rot = carla.Rotation(pitch=30, yaw=0, roll=0)
    trans = carla.Transform(carla.Location(0,0,0), rot)
    matrix = trans.get_matrix()
    print("\nCARLA 4x4 for Pitch=30:")
    print(np.array(matrix))

if __name__ == "__main__":
    get_true_carla_matrix()
