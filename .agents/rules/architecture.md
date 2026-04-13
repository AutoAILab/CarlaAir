---
trigger: always_on
glob: "*.py, *.cpp, *.h"
description: Core architectural principles and system structure for the CarlaAir repository.
---

# CarlaAir Architecture Rules

This document outlines the high-level architecture of CarlaAir so you can understand the repository structure without digging through the entire source code. Use these rules when conceptualizing new features or modifying the system.

## 1. Single Process Composition
- **Architecture**: CARLA and AirSim run entirely within a single **Unreal Engine 4.26** process. 
- **Core Component**: The root class managing this is `ASimWorldGameMode` (in C++ space).
- **Rationale**: This eliminates RPC bridging latency between the two physics engines. There is no external bridge or socket orchestrator syncing timestamps.

## 2. Initialization Sequence
- During the `BeginPlay()` phase of `ASimWorldGameMode`:
  1. The **CARLA Subsystem** initializes the Episode, Traffic Manager, Weather, and RPC Server (Port `2000`).
  2. The **AirSim Subsystem** is bootstrapped simultaneously, loading drone settings, HUD widgets, and its own RPC Server (Port `41451`).
- The result is two mutually exclusive APIs that interface directly with the same underlying world components.

## 3. Absolute Coordinate Alignment
- **Zero Error Rule**: The simulator relies on an exact `0.000m` mathematical error when mapping Left-Handed CARLA coordinates to Right-Handed NED AirSim coordinates.
- **Implementation**: The world origin is deliberately **not dynamically moved** (`SetNewWorldOrigin` is suppressed). This ensures reliable generation of ground-truth geometric data across domains.

## 4. Sensor Data & Component Flow
- **Sensors**: CARLA manages ground vehicle sensors (Cameras, LiDAR, Radar) through its `SensorManager`, outputting via the Port 2000 RPC server.
- **Drone Vision**: AirSim handles aerial drone vision directly via its `PIPCamera` RenderTargets, outputting via Port 41451.
- Both systems share the identical weather system, physical actors, and rendering frame without knowing about each other's native code.

## 5. The "Unchanged Upstream" Principle
- **Core Methodology**: To ensure maximum stability and modularity, CarlaAir strictly limits modifications to the foundational CARLA or AirSim source codes.
- **Rule of Thumb**: Modifications to upstream code files are restricted to expose protected variables or add critical hooks (often less than 50 lines modified globally). Do not introduce sprawling inter-dependencies within the actual C++ UE4 source unless strictly required. Use the `extension-patterns` if complex functionality is added.

## 6. Datasets & Evaluation Scope
- CarlaAir is an environment engine, **not** an evaluation framework.
- Automation logic, metric testing, and dataset storage should always exist outside of the simulator layer within isolated Python orchestration scripts.
