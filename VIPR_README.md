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
```

## 📐 Dataset Architecture
For detailed information on the Air-Ground Swarm architecture and Ground Truth fusion, see the **[Architecture Guide](file:///home/df/data/jflinte/CarlaAir/dataset_generation/ARCHITECTURE.md)**.

## Why UV?
`uv` provides significantly faster environment resolution and installation compared to Conda. CarlaAir's `setup_env_uv.sh` handles the complex injection of the pre-compiled `carla` module into the local `.venv` automatically.
