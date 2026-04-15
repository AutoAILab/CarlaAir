# Ticket T01: Core Generation Script Setup

## Description
Establish the foundational data generation script (`agcarla_datagen.py`) that coordinates both CARLA and AirSim systems, supporting a **Multi-UAV Swarm** (5 drones) and a **Transform-based Path Playback** engine.

## Tasks
- [ ] **Infrastructure Setup**
    - [ ] Implement CARLA client/world connection with `synchronous_mode`.
    - [ ] **Swarm Integration**: Implement loops for accessing `UAV_1` through `UAV_5`.
- [ ] **Path Playback Engine**
    - [ ] **Record Mode**: Export actor transforms (Location/Rotation) to JSON for every frame.
    - [ ] **Replay Mode**: Implement frame-accurate teleportation (Transform application) with physics ENABLED.
- [ ] **Sensor Configuration**
    - [ ] Setup UGV and Swarm Drone Sensor Suites (RGB, Depth, Seg, LiDAR, IMU).
- [ ] **Real-time QA**
    - [ ] Implement `lane_type` validation for the UGV.

## References
- [ARCHITECTURE.md](../dataset_generation/ARCHITECTURE.md) (Section 3 & 6)
- [VERIFICATION.md](../dataset_generation/VERIFICATION.md)
