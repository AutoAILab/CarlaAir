# AGCarla Dataset Pipeline: Tasks & Subtasks

This document tracks the milestones and associated subtasks required to build and execute the AGCarla data generation pipeline.

## Milestone 1: Core Generation Script (`agcarla_datagen.py`)
- [ ] **Infrastructure Setup**
    - [ ] Implement CARLA client/world connection with `synchronous_mode`.
    - [ ] **Swarm Integration**: Iterate through `UAV_1` to `UAV_5` using the AirSim `MultirotorClient`.
    - [ ] **Path Playback**: Implement "Record" and "Replay" logic using actor transforms.
- [ ] **Sensor Configuration**
    - [ ] Setup UGV and Swarm Drone Sensor Suites (RGB, Depth, Seg, LiDAR, IMU).
    - [ ] **QA Hook**: Implement real-time `lane_type` validation.

## Milestone 2: Swarm Coordination & Movement
- [ ] **Swarm Flight Logic**
    - [ ] Configure `settings.json` for 5 simultaneous drones.
    - [ ] Implement UGV "Chase" mode for the entire swarm.
    - [ ] Assign unique height/pitch offsets per drone.

## Milestone 3: OpenLabel Manifest Generation
- [ ] **Schema Implementation**
    - [ ] Create `master_manifest.jsonl` in ASAM OpenLabel 1.0.0 format.
    - [ ] Implement multi-UAV tagging per frame.

## Milestone 4: G-Buffer & Auxiliary Modalities
- [ ] **G-Buffer Extraction**
    - [ ] Implement `listen_to_gbuffer` for Normals, Stencil, Depth, and Velocity.
    - [ ] **Storage**: Bundle modalities into compressed **`.npz`** files per frame.
- [ ] **Geometry Processing**
    - [ ] Calculate and save **Ray Maps** (per-pixel unit rays).

## Milestone 5: Pilot Run & Verification (Dry Run)
- [ ] **Verification**
    - [ ] Export 2Hz side-by-side **Sync GIFs** (UGV + Swarm).
    - [ ] **Replay Check**: Verify that teleportation replay accurately follows the baseline path.
    - [ ] Verify Depth-to-Lidar alignment for all 5 drone views.

## Milestone 6: Production Generation (10,000+ Aerial Frames)
- [ ] **Multi-Town Sweep**
    - [ ] Record 5 Baselines across different towns.
    - [ ] Generate 5-10 Variations per Baseline (Weather, Time).

## Milestone 7: Global Mapping & Open3D Merging
- [ ] **Map Merging**
    - [ ] Use Open3D to accumulate and voxel-downsample LiDAR maps per town.

## Milestone 8: 3D Ground Truth Fusion Script
- [ ] **Fusion Orchestration**
    - [ ] Implement `fuse_frame_pcs.py` to merge 6 synchronized views (1 UGV + 5 UAVs).
    - [ ] Use **Ray Maps** for sub-pixel reprojection accuracy.
    - [ ] Implement voxel-based normalization ($0.05m$) for final GT pointclouds.
    - [ ] Output format: `.ply` or `.pcd` optimized for Chamfer Distance benchmarking.
