# AGCarla Scenario Metadata Taxonomy

This document defines the structured scenario-level metadata (tags and attributes) used in the **AGCarla** dataset. ASAM OpenLabel 1.0.0 standards should be held.

## 1. Aerial Perspective (UAV Swarm)
These attributes describe the state of the swarm drones.

| Actor ID | Platform | Perspective | Purpose |
|----------|----------|-------------|---------|
| **UGV_1** | CARLA | Ground/Driver | Lead ground-truth actor |
| **UAV_1** | AirSim | 15m | Low-altitude urban detail |
| **UAV_2** | AirSim | 30m | Tactical urban overview |
| **UAV_3** | AirSim | 45m | Standard mapping perspective |
| **UAV_4** | AirSim | 60m | Wide-area coverage |
| **UAV_5** | AirSim | 90m | High-altitude strategic view |

> [!IMPORTANT]
> **Simultaneous Capture**: All 5 drones are captured in the same simulation tick to provide synchronized multi-viewpoint coverage of a single UGV ground state.

## 2. Auxiliary Modalities (G-Buffer)
Extracted directly from the Unreal Render Pipeline and saved as grouped **`.npz`** files.

| Texture ID | Modalitiy | Description |
|------------|-----------|-------------|
| `GBufferA` | **Normals** | World-space normal vectors (RGB). |
| `SceneDepth`| **Linear Depth** | Exact linear depth in world units. |
| `GBufferB` | **Physical** | Metallic, Specular, and Roughness maps. |
| `Velocity` | **Flow** | Screen-space velocity vectors. |
| `SceneStencil`| **Stencils**| Object-level masking/layering. |

## 3. Environment (CARLA)
### 3.1 Map & Location
| Attribute | Example Values | OpenLabel Tag |
|-----------|----------------|---------------|
| **Town/Map** | `Town01`, `Town03`, `Town10HD` | `Town_01`, `Town_10HD` |
| **Sequence Type**| `Baseline`, `Variation` | `SeqBaseline`, `SeqVar` |

### 3.2 Weather Conditions
Following ASAM ODD taxonomy.
| Condition | Parameter Range | OpenLabel Tag |
|-----------|-----------------|---------------|
| **Clear** | Cloudiness < 20%, Precip: 0 | `WeatherClear` |
| **Rainy** | Cloudiness > 50%, Precip: 30-80 | `WeatherRain` |
| **Foggy** | Fog Density: 30-100 | `WeatherFog` |

## 4. Usage in Manifest
All scenario metadata is stored in the `frame_properties.tags` array of the `master_manifest.jsonl`. 

**Example**:
```json
"tags": [
  "Town_10HD", 
  "SeqVar",
  "UAV_1", "UAV_2", "UAV_3", "UAV_4", "UAV_5",
  "WeatherRain", 
  "Normal_Captured",
  "TrafficMedium"
]
```
