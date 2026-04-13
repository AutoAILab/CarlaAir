# AGCarla Dataset Generation Architecture

This document describes the design and technical architecture of the **AGCarla** data generation pipeline.

## 1. Overview
The pipeline is designed to collect high-fidelity, multi-modal, synchronized data from both Ground (UGV) and Aerial (UAV) perspectives. It leverages the unified **CarlaAir** infrastructure, where CARLA manages the ground world and AirSim manages the aerial simulation.

## 2. Synchronization Mechanism
To ensure that UGV and UAV sensors capture the exact same world state (zero-latency sync), the pipeline uses **CARLA Synchronous Mode**:
- The simulation is "ticked" manually by the Python client (`world.tick()`).
- All sensors (RGB, Depth, Lidar) are registered to the same tick.
- The pipeline waits for all sensor buffers to fill before advancing the simulation.

## 3. Swarm Perspective Configuration (Air-Ground)
To maximize generation efficiency and viewpoint diversity, the pipeline utilizes a unified **Air-Ground Swarm Architecture**:

| Actor ID | Platform | Perspective | Purpose |
|----------|----------|-------------|---------|
| **UGV_1** | CARLA | Ground/Driver | Lead ground-truth actor |
| **UAV_1** | AirSim | 15m | Low-altitude urban detail |
| **UAV_2** | AirSim | 30m | Tactical urban overview |
| **UAV_3** | AirSim | 45m | Standard mapping perspective |
| **UAV_4** | AirSim | 60m | Wide-area coverage |
| **UAV_5** | AirSim | 90m | High-altitude strategic view |

- **Drone Swarm (Aerial)**: 5 drones (`UAV_1` to `UAV_5`) are spawned simultaneously in the world via AirSim.
- **Ground Perspective (UGV)**: The primary ground vehicle (`UGV_1`) is spawned via CARLA and acts as the "Lead" actor for the swarm.
- **Synchronized Capture**: For every frame, the pipeline captures 6 unique perspectives (1 Ground + 5 Aerial) in a single synchronized tick.
- **Perspectives**: Each aerial drone maintains a dedicated height (e.g., 15m to 90m), while the UGV captures the driver-level perspective.

## 4. Coordinate Systems & Alignment
CarlaAir provides exact mathematical alignment between the two physics engines:
- **UGV**: Uses CARLA Global Coordinates (Left-Handed Z-Up).
- **UAV**: Uses AirSim NED Coordinates (Right-Handed Z-Down).
- **Alignment**: A fixed transform offset (Town-dependent) is used to map CARLA ground actors to AirSim aerial viewpoints.

## 5. Multi-Modal Data Flow
For each captured frame, the following data is extracted:
- **RGB**: 8-bit color imagery (PNG).
- **Depth**: 16-bit linear depth (NPY/PNG).
- **Semantic Seg**: Pixel-wise category labels (NPY).
- **Lidar**: 3D point clouds (BIN/NPY).
- **Auxiliary (G-Buffer)**: Scene Normals, Stencils, and Velocity maps (Saved as grouped **`.npz`**).
- **Metadata**: Ego-poses, timestamps, and ODD tags (Weather, Town, Altitude).

## 6. 3D Ground Truth & Evaluation Support
To support high-fidelity evaluation of **3D Foundation Models** via **Chamfer Distance**, the architecture implements a specialized Ground Truth (GT) fusion pipeline:

### 6.1 Multi-View Pointcloud Fusion
For each synchronized frame, the pipeline merges all 6 perspectives (1 UGV + 5 UAVs) into a single coordinate-consistent dense pointcloud:
- **Reprojection**: Each depth map is reprojected using its corresponding **Ray Map** and world-space transform.
- **Occlusion Filling**: By fusing ground and aerial perspectives, the GT cloud fills areas that are typically occluded in single-view datasets (e.g., roof textures, undersides of vehicles).
- **Voxel Normalization**: The resulting pointcloud is voxel-downsampled (default $0.05m$) to ensure a uniform distribution for fair distance-based metrics.

### 6.2 Evaluation Readiness
- **Intrinsics**: Full intrinsic matrices and distortion-free Ray Maps are provided for each sensor.
- **Geometry**: The zero-latency sync guarantees that RGB and GT Pointcloud are perfectly aligned in the same temporal slice.

## 7. Record/Replay (Baseline + Variation) Logic
To ensure consistent path-following across different environmental conditions (e.g., Clear vs. Rain), the pipeline follows a two-stage process:

### 6.1 Baseline Record
- A "ClearNoon" sequence is recorded where the UGV drives a route.
- The precise transform (Location/Rotation) of the UGV, the Swarm drones, and all background vehicles/pedestrians is recorded to a `baseline_transforms.json`.

### 6.2 Variation Replay (Transform-based Playback)
- The baseline sequence is replayed under different ODD conditions (Rain, Night, Fog).
- **Transform Replay**: Actors are "teleported" to their recorded transforms every frame. 
- **Physics**: Physics remains **ENABLED** during replay. While teleportation dictates the path, the physics engine handles local interactions. Slight variations in actor suspension or orientations are accepted to prioritize realistic path-following over mathematical $0.000m$ error.

---

## 7. Related Documentation
- [agcarla_datagen.py](file:///home/df/data/jflinte/CarlaAir/dataset_generation/agcarla_datagen.py): The core generation orchestration script.
- **[TASKS.md](file:///home/df/data/jflinte/CarlaAir/dataset_generation/TASKS.md)**: Detailed implementation milestones and subtasks.
- **[VERIFICATION.md](file:///home/df/data/jflinte/CarlaAir/dataset_generation/VERIFICATION.md)**: Mandatory QA steps for geometric and data integrity.
- **[SCENARIO_METADATA.md](file:///home/df/data/jflinte/CarlaAir/dataset_generation/SCENARIO_METADATA.md)**: Full taxonomy of ODD tags and sensor attributes.
