# Ticket: LiDAR Intra-Frame Ego-Motion Distortion (Skew/Ghosting)

## Problem Statement
During dynamic pipeline runs (where the UGV is actively navigating the `Town10HD` map), the generated global point cloud reconstruction exhibits severe horizontal "ghosting" and physical smearing. Objects like buildings and pillars appear duplicated or stretched along the vehicle's vector of travel. 

Importantly, **static verification runs** (with the trajectory path mathematically disabled) produce perfectly crisp map reconstructions with exactly `0.000m` of horizontal geometric error, isolating the issue entirely to vehicle velocity.

## Technical Root Cause Analysis
The artifact is driven by CARLA's internal sensor architecture interacting with our Python orchestration synchronization:

1. **Continuous LiDAR Sweep Simulation:** CARLA's raycast LiDAR actively swept and aggregates points over the full duration of the Engine's Tick interval (`fixed_dt = 0.05s`). 
2. **Discrete Transform Polling:** Our synchronization pipeline (`agcarla_datagen.py`) polls the absolute sensor transform (`sensor_tf = self.ugv.get_transform()`) discretely either directly before or immediately after the tick.
3. **The Dislocation Lag:** Points that were struck by the laser at the *beginning* of the `0.05s` interval (`t = 0.00s`) were fired from a physical coordinate origin that the vehicle had not yet advanced from. However, because our script transforms the *entire* resulting unitary array of point coordinates using the identical discrete position logged at `t = 0.05s`, those earlier sweep points are erroneously warped forward by the exact distance the UGV travelled during that 50-millisecond window.

**Result**: Classic rolling-shutter "Slant" or "Ghosting", scaling linearly with the magnitude of the translational/rotational velocity.

## Required Implementation / Possible Solutions

To close this ticket and achieve crisp moving point clouds, one of the following architectural changes must be implemented:

### Option A: Analytical Post-Process Motion Descanning (Recommended)
Because CARLA does not natively embed high-precision timestamps *per-point* in the default LiDAR array API output:
1. Extract linear velocity, angular velocity, and the previous frame's transform for the UGV.
2. Approximate a continual SLERP (Spherical Linear Interpolation) across the `0.05` physics window.
3. Radially subdivide the numpy point array using the internal rotational angle of the LiDAR (`channels`, `rotation_frequency`, and `FOV`).
4. Inverse-project the continuous motion vector out of the raw coordinates *before* applying the absolute World-Transform.

### Option B: Stop-and-Go Capture Architecture
Modify the trajectory playback engine inside the script to halt the LocalPlanner.
- Traverse to Waypoint A. 
- Halt entirely (Velocity = 0).
- Sleep 1 tick. Capture precise motionless arrays.
- Proceed to Waypoint B.
*(Note: Breaks real-world continuous driving paradigms, but ensures optimal geometric lock)*

### Option C: Simulation Tick Subdivision
Dramatically decrease the physics `fixed_delta_seconds` (e.g. `0.005s` / 200 Hz) to close the physical travel gap per-tick, but maintain the sensor capture threshold. Increases computation overhead exponentially.

## Regression Test Metrics
- Run `verify_lidar.py` on a standard moving `run_single_uav_ugv.json` batch.
- **Pass Criteria:** `Global Temporal Drift` drops below < 0.10m. Horizontal structural smearing in visual reconstructions (`verify_reprojection.py`) must be completely eliminated.
