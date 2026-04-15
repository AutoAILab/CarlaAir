# Path Planning and UAV/UGV Control [COMPLETED]

## Description
A user should be able to control how the UGV and UAVs move in the simulation for capturing the dataset.

## Research & Approach Evaluation

### Approach 1: CARLA Traffic Manager / Autopilot
- **Pros**: Highly realistic behavior, handles collisions, easy to setup for variety.
- **Cons**: Random paths, hard to replicate exactly across different weather variations (OOD testing).

### Approach 2: Waypoint Route Files (Manual)
- **Pros**: Perfectly deterministic.
- **Cons**: Extremely tedious to create for complex city maneuvers.

### Approach 3: Record/Replay (Selected)
- **Pros**: Best of both worlds. We record a "Baseline" run (using Autopilot for realism), saving frame-by-frame transforms to metadata. We then "Replay" for variations (Rain, Night) by teleporting actors to these recorded transforms.
- **Cons**: Replay actors don't react to new physics changes, but this is acceptable for consistent dataset variation generation.

## Final Implementation

### 1. Swarm "Chase" Mode
Implemented in `agcarla_datagen.py`. The UGV is set to Autopilot as the "Lead" actor. Five UAVs are programmed to follow the UGV using fixed height and horizontal offsets (e.g., UAV_1 at 15m, UAV_5 at 90m nadir).

### 2. Route Persistence (Record/Replay)
- **Record**: The script captures `metadata/*.json` for every frame, containing the global $XYZ$ and $RPY$ of all actors.
- **Replay**: Using the `--mode replay` flag, the generator reads these metadata files and teleports actors to the exact recorded positions, ensuring that a "Rainy Town 10" sequence has identical geometry to the "Clear Town 10" sequence.

## Acceptance Criteria
- [x] Research how path planning is done in CarlaAir.
- [x] Identify pros/cons of different approaches.
- [x] Implementation of route/follow mode.
- [x] Verification of consistent replay across variations.