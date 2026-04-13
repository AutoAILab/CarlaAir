---
trigger: always_on
glob: "*.py, *.cpp, *.h"
description: Standard programming and scripting guidelines for CarlaAir.
---

# CarlaAir Programming Guidelines

This document outlines the standard coding practices across the Python APIs and the C++ Unreal Engine backend. Adhere to these guidelines when authoring evaluation scripts, automation logic, or when venturing into the engine code.

## 1. Dual API Integration (Python)
- **Concurrent Clients**: Always instantiate both API clients explicitly within the scripting context:
  ```python
  import carla
  import airsim
  
  carla_client = carla.Client("localhost", 2000)
  air_client = airsim.MultirotorClient(port=41451)
  ```
- **Same World Interaction**: Act on the environment through either API. Understand that weather changes or actor spawn routines performed in CARLA immediately manifest within the AirSim viewport, and vice versa.

## 2. Python Script Lifecycle Management
- **Strict Teardown**: CARLA-Air operates in a persistent state. Always mandate a setup, execution, and teardown phase. 
- Utilize a `try...except...finally` block in primary orchestration scripts to guarantee sensor de-registration and actor destruction.
- **Pre-Execution Cleanup**: Prefix any major world spawning logic with a dedicated `cleanup_previous()` function to purge orphaned actors or unresolved sensor `listen()` bindings from prior failed runs.

## 3. Coordinate Transformations
- **Translation Utility**: When programmatically dictating drone movements based on CARLA actor positions, you **must** translate Right-Hand Z-Up (Left-Handed) coordinates to NED coordinates using accurate dimensional scaling.
- Include a robust `carla_to_ned` utility similar to this when performing cross-world trajectory management:
  ```python
  def carla_to_ned(cx, cy, cz, offset_x, offset_y, offset_z):
      return cx + offset_x, cy + offset_y, -cz + offset_z
  ```

## 4. C++ Back-End Rules (Unreal Engine 4.26)
- **Subclassing Over Modification**: When adding logic tracking to the simulator (e.g., custom managers like `ASimWorldGameMode`), do not clutter foundational Unreal Engine or basic `CarlaGameModeBase` classes. Extend through deliberate subclasses.
- **Minimal Upstream Exposure**: Only modify upstream code (`CarlaEpisode`, `SimModeBase`) to expose inherently private variables out as `protected` variables or add `friend` class designations. Do not introduce broad new method logic inside foreign classes.
- **Memory Management**: Stick to UE4 standards (`UPROPERTY()`, `USTRUCT()`, `TArray<>`) when allocating resources inside the engine. Avoid raw pointer heaps that accumulate over Tick intervals. 

## 5. Extension Protocols
- **Adhere to `extension-patterns`**: When creating a patch that absolutely requires C++ side manipulation, construct the new features externally to the core repo files, linking them during compilation. Do not attempt "dirty" inline hacks into core loops without exceptional, documented cause.
