# CarlaAir VIPR Environment Setup (UV)

This document outlines the specialized environment setup using `uv` for high-speed package management.

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
```

## Why UV?
`uv` provides significantly faster environment resolution and installation compared to Conda. CarlaAir's `setup_env_uv.sh` handles the complex injection of the pre-compiled `carla` module into the local `.venv` automatically.
