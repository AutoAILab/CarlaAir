# AGCarla Trajectory Utilities

This directory contains two primary utilities for managing actor trajectories in the AGCarla ecosystem: **Route Generation** and **Route Curation**.

---

## 🏗️ 1. Route Generation (`generate_complex_route.py`)

Used to create a fresh "Production" route config from scratch using the CARLA Global Route Planner.

### Key Features
- **Deterministic Planning**: Finds the actual road-legal path between any two spawn point IDs.
- **Auto-Swarm Bootstrapping**: Generates a standard 6-actor bundle (1 UGV + 5 UAVs) automatically.
- **Cluster Lag Strategy**: Configures drones in tactical clusters (close follow vs. far follow) with built-in temporal lag.

### Usage
```bash
uv run dataset_generation/generate_complex_route.py \
    --start 0 \
    --end 15 \
    --speed 15.0 \
    --out route_production_A.json
```

---

## 🧹 2. Route Curation (`route_curation.py`)

Used to convert raw "Record" sessions into high-quality, reusable "Template" routes.

### Key Logic
- **Precision Thinning**: Uses the **Ramer-Douglas-Peucker (RDP)** algorithm to reduce frame-by-frame recordings into sparse, jitter-free waypoints.
- **Cross-Sim Sync**: Extracts and aligns data for all active actors (UGV and UAVs) into a unified co-simulation bundle.

### Usage
```bash
uv run dataset_generation/route_curation.py \
    --input data/your_record_session \
    --output curated_template.json \
    --epsilon 0.5
```
> **Tip:** Increasing `--epsilon` creates a more sparse (thinner) path, while decreasing it stays truer to the original jittery recording.

---

## 🔄 Workflow Integration

| Scenario Type | Tool to Use |
| :--- | :--- |
| **New AI Scenario** | `generate_complex_route.py` |
| **Manual Replay/Variation**| `route_curation.py` |

Both tools produce `.json` files compatible with the `--route` and `--route-map` flags in `agcarla_datagen.py`.
