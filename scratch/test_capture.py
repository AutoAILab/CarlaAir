import subprocess
import os

# Assume we already have a static config
res = subprocess.run(["uv", "run", "dataset_generation/agcarla_datagen.py", "--config", "dataset_generation/configs/run_single_uav_ugv_static.json", "--num-frames", "2", "--out", "data/test_capture"], cwd="/home/df/data/jflinte/CarlaAir", capture_output=True, text=True)

print("Stdout:")
print(res.stdout[-1000:])
print("Stderr:")
print(res.stderr)
