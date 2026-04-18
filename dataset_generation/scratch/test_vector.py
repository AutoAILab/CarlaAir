import carla
v = carla.Vector3D(1.0, 2.0, 3.0)
try:
    u = v.make_unit_vector()
    print("make_unit_vector() works without arguments")
except Exception as e:
    print(f"make_unit_vector() failed: {e}")

try:
    import inspect
    print(dir(v))
except:
    pass
