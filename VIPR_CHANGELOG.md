# VIPR Features Changelog

## 2026-04-11
- Created `.agents/workflows/ticket.md` and `.agents/workflows/complete.md` to automate VIPR branch and ticket handling.
- Introduced foundational architecture document in `.agents/rules/architecture.md` per VIPR README design tenets and constraints.
- Integrated automated Git-sync checking, explicit dependency on extension-patterns, and project governance review within workflow scripts.

## 2026-04-13
- Completed `modifications.md` task: finalized project workflows, architecture documentation, and ticket lifecycle management.
- Migrated default package management from Conda to UV.
- Created `env_setup/setup_env_uv.sh` for fast, reproducible environment setup.
- Updated `env_setup/test_env.sh` and `carlaAir.sh` to support UV-based virtual environments.
- Initialized `VIPR_README.md` with UV setup guide.
- Initialized AGCarla dataset pipeline documentation (ARCHITECTURE, TASKS, METADATA).
- Implemented core generation script `agcarla_datagen.py` with Air-Ground Swarm coordination (1 UGV + 5 UAVs).
- Developed a transform-based Record/Replay engine with path-accurate teleportation playback.
- Integrated lossless G-Buffer bundling (.npz) and per-pixel unit Ray-Map generation.

## 2026-04-15
- Completed `T02_swarm_coordination.md`: Finalized 5-drone chase logic and relative perspective offsets.
- Implemented multi-UAV coordinate mapping (CARLA to NED) for stable relative flight.
- Fixed AirSim segmentation image parsing in `agcarla_datagen.py` to handle compressed PNG responses.
- Verified swarm coordination via 5-frame smoke test in Town10HD.
