import carla

def get_town_stats(map_name):
    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    
    if world.get_map().name != map_name:
        print(f"Loading {map_name}...")
        world = client.load_world(map_name)
    
    m = world.get_map()
    spawn_points = m.get_spawn_points()
    print(f"Town: {m.name}")
    print(f"Number of Spawn Points: {len(spawn_points)}")
    
    # Analyze a few clusters
    for i, sp in enumerate(spawn_points[:10]):
        print(f"SP {i}: {sp.location}")

if __name__ == "__main__":
    get_town_stats("Town10HD")
