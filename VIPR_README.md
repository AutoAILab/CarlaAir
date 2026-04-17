# CarlaAir VIPR Environment Setup (UV)

This document outlines the specialized environment setup using `uv` for high-speed package management.

## Datasets
- [AGCarla](/home/df/data/datasets/AGCarla/README.md)

## 📦 Missing Binary? (Read First)
The current repository only contains the **source code** for the co-simulation environment and its workflows. It does **not** include the 10GB+ pre-built simulator binary required to run `carlaAir.sh`.

To get started without a complex 8+ hour compilation process, download the **v0.1.7 pre-built executable**:
- 📥 **Hugging Face**: [Download CarlaAir-v0.1.7](https://huggingface.co/tianlezeng/CarlaAIr-v0.1.7)
- 📥 **Baidu Pan**: [Download](https://pan.baidu.com/s/1RguWqwKrN-3KEgyKvWiiug?pwd=d5ai) (Pwd: `d5ai`)

Extract the contents and ensure the `CarlaUE4` directory exists at your project root.

## 🎮 Quick Start with UV

```bash
# 1. Download and extract CARLA-Air v0.1.7
tar xzf CarlaAir-v0.1.7.tar.gz
cd CarlaAir-v0.1.7

# 2. One-click environment setup (UV version)
bash env_setup/setup_env_uv.sh    # creates .venv, installs deps, deploys carla module
source .venv/bin/activate
bash env_setup/test_env.sh        # verify: should show all PASS

# 3. Launch the simulator (auto-spawns traffic)
./CarlaAir.sh Town10HD

# 4. Run the showcase! (in another terminal)
source .venv/bin/activate
python3 examples/quick_start_showcase.py

# 5. AGCarla Dataset Generation
source .venv/bin/activate
# Record a baseline sequence
python3 dataset_generation/agcarla_datagen.py --mode record --out data/baselines
# Replay under variations (Rain, etc. must be set in simulator first)
python3 dataset_generation/agcarla_datagen.py --mode replay --host 127.0.0.1
# Verify geometric consistency
python3 dataset_generation/verify_geometry.py --dir data/baselines/LATEST_SEQ
```

## 🎮 Manual Control & Recording

If the main simulator window appears black on your monitor (due to Vulkan display conflicts), use these scripts to control the actor and see the environment via Pygame viewports.

### 1. Manual Pilot Scripts
- **Drive a Car**: `python3 examples/manual_drive_vehicle.py` (Chase Camera)
- **Fly a Drone**: `python3 examples/manual_fly_drone.py` (FPV Camera)

**Controls**:
- **WASD**: Movement (Drive or Body-frame Velocity)
- **R / Space**: Reverse / Handbrake (Vehicle only)
- **Q / E**: Throttle Up / Down (Drone only)
- **N**: Cycle through Weather presets (Clear, Rain, Fog, Night)
- **X**: Toggle **Path Recording** (Vehicle or Drone)
- **T / G**: Tilt Camera Up / Down (Drone only)
- **ESC**: Cleanup actors and Exit

### 2. Path Recording Workflow
To create custom trajectories for automated data generation:
1. Run `python3 examples/manual_drive_vehicle.py` or `python3 examples/manual_fly_drone.py`.
2. Press **X** to start recording (A red `● REC` indicator will appear in the HUD).
3. Drive or Fly your desired path.
4. Press **X** again to save.
5. The **Route JSON** is saved to `dataset_generation/trajectories/`.

Recorded paths can be loaded into the swarm generator. Use the following flags to optimize performance or change viewing angles:

```bash
# General swarm run (1 UGV + 5 UAVs)
uv run dataset_generation/agcarla_datagen.py --route dataset_generation/trajectories/YOUR_ROUTE.json

# Optimized UGV-only run (Skip AirSim)
uv run dataset_generation/agcarla_datagen.py --route dataset_generation/trajectories/YOUR_ROUTE.json --no-uavs

# Optimized Drone-only run (Skip CARLA spawning)
uv run dataset_generation/agcarla_datagen.py --route dataset_generation/trajectories/YOUR_ROUTE.json --no-ugv
```

### 4. Drone Perspectives (Gimbal)
When replaying a drone route, you can override the viewing perspective (camera angle and height) to generate multi-angle data from a single flight:

```bash
# Use a preset (top-down, oblique, side)
uv run dataset_generation/agcarla_datagen.py --config [...] --perspective top-down

# Manual Overrides (Degrees and Meters)
uv run dataset_generation/agcarla_datagen.py --config [...] --camera-pitch -70 --height-offset 15
```

> [!NOTE]
> **Priority Hierarchy**: Manual settings (`--camera-pitch`, `--height-offset`) always take precedence over the `--perspective` preset. This allows you to pick a preset as a starting point and then fine-tune specific values.

## 📐 Dataset Architecture
For detailed information on the Air-Ground Swarm architecture and Ground Truth fusion, see the **[Architecture Guide](file:///home/df/data/jflinte/CarlaAir/dataset_generation/ARCHITECTURE.md)**.
Specifically, see **Section 4** for details on the Geometry Utility library and the Multi-View re-projection pipeline.
For a complete reference of all configuration flags and JSON keys, see the **[Configuration Guide](file:///home/df/data/jflinte/CarlaAir/dataset_generation/CONFIG_GUIDE.md)**.

## Why UV?
`uv` provides significantly faster environment resolution and installation compared to Conda. CarlaAir's `setup_env_uv.sh` handles the complex injection of the pre-compiled `carla` module into the local `.venv` automatically.
