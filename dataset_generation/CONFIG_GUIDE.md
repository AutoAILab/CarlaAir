# AGCarla Configuration Guide

This document provides a comprehensive reference for the configuration flags and JSON keys used in the `agcarla_datagen.py` pipeline.

## 1. Primary Execution Flags

These can be set via CLI or as root-level keys in a `.json` configuration file.

| Argument | JSON Key | Description | Default |
| :--- | :--- | :--- | :--- |
| `--config` | N/A | Path to a JSON configuration file. | None |
| `--num-frames` | `num_frames` | Number of frames to capture. | 100 |
| `--fixed-dt` | `fixed_dt` | Fixed delta time (seconds) per frame. | 0.05 (20 FPS) |
| `--out` | `out` | Directory to save the dataset. | `data/agcarla` |
| `--viz-path` | `viz_path` | Render replay waypoints in CARLA for debugging. | `false` |

## 2. Actor & Route Management

| Argument | JSON Key | Description |
| :--- | :--- | :--- |
| `--route` | `route` | Path to a single trajectory JSON file. |
| `--route-map` | `route_map` | A JSON object mapping specific Actor IDs to separate route files. |
| `--no-uavs` | `no_uavs` | Disable all AirSim drone spawning and capture (UGV-only mode). |
| `--no-ugv` | `no_ugv` | Disable CARLA vehicle spawning (Drone-only mode). |
| `--uav-names` | `uav_names` | Explicit list of UAV names (e.g., `["UAV_1", "UAV_2"]`). |

### Example: Route Mapping
The `route_map` allows you to mix independent trajectories:
```json
{
    "route_map": {
        "UAV_1": "trajectories/drone_long_flight.json",
        "UGV_1": "trajectories/car_inner_loop.json"
    }
}
```

## 3. Drone Perspective & Gimbal Control

These settings apply to any drone acting as a **Lead** (replaying a path).

| Argument | JSON Key | Description | Default |
| :--- | :--- | :--- | :--- |
| `--perspective` | `perspective` | Choose a preset: `default`, `top-down`, `oblique`, `side`. | `default` |
| `--camera-pitch` | `camera_pitch` | Manual override for camera gimbal pitch (degrees). | Preset Default |
| `--height-offset` | `height_offset` | Add altitude to the recorded path (meters). | Preset Default |
| `--camera-offset-x`| `camera_offset_x`| Forward camera translation (NED +X). | 0.0 |
| `--camera-offset-y`| `camera_offset_y`| Right camera translation (NED +Y). | 0.0 |
| `--camera-offset-z`| `camera_offset_z`| Downward camera translation (NED +Z). | 0.0 |

### Presets Reference
| Preset | Pitch | Height |
| :--- | :--- | :--- |
| `top-down` | -90.0 | +20.0m |
| `oblique` | -45.0 | +10.0m |
| `side` | 0.0 | +0.0m |

## 4. Stability & Architecture

*   **Dual-Client Mode**: The script automatically initializes two connections per drone (Sensor vs. Control) to prevent network buffer collisions (`BufferError`) during high-speed capture.
*   **Sync-Mode Deadlock**: Gimbal commands are executed in background threads to avoid blocking the main simulation tick, ensuring the script never freezes during initialization.
