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
- Completed `T03_openlabel_manifest.md`: Implemented automated ASAM OpenLABEL 1.0.0 manifest generation.
- Integrated dynamic ODD tagging (Weather, Town, Sequence) and multi-UAV swarm tagging into the datagen pipeline.
- Developed real-time per-frame JSONL logging with external resource linking for multi-modal data.
- Verified manifest compliance and data integrity via unit tests in `verify_logic.py` and a 5-frame smoke test.
- Completed `T04_geometric_modalities.md`: Implemented and verified geometric consistency for the AGCarla dataset pipeline.
- Developed centralized geometry utility library (`geometry.py`) for pinhole re-projection and cross-engine coordinate alignment.
- Integrated automated Global Coordinate Offset Calibration to synchronize CARLA world and AirSim local origins.
- Implemented a hybrid validation framework (`verify_geometry.py`) featuring semantic-guided absolute alignment and project-back consistency checks.
- Confirmed sub-centimeter geometric precision floors for 16-bit depth quantization.
- Completed `T05_verification_pilot.md`: Finalized infrastructure and executed the 50-frame pilot capture for the AGCarla dataset.
- Upgraded `agcarla_datagen.py` with specialized UGV depth sensors and raw Lidar logging for zero-latency verification.
- Established a comprehensive verification suite including Sync GIFs, multi-view pointcloud fusion, and OpenLabel manifest validation.
- Completed `path_planning.md`: Documented path planning research and finalized the Record/Replay based movement system.
- Formalized "Route Files" as metadata-driven transform sequences for deterministic multi-ODD variation generation.
- Verified follow-mode stability for the 5-UAV swarm relative to a Lead UGV using CARLA Autopilot.

