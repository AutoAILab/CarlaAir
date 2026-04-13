# AGCarla Dataset Verification & QA Protocol

This document outlines the mandatory testing and verification steps to ensure the **AGCarla** dataset meets high standards for geometric accuracy, realistic motion, and data integrity.

## 1. Geometric & Motion Verification

### 1.1 Road Alignment (UGV)
To ensure the UGV is following realistic paths and not driving "off-road" or through buildings:
- **Automated Check**: For every frame, verify that the UGV's `lane_type` is `Driving`.
- **Metric**: Record the percentage of frames on designated roads in the capture log.
- **Fail Condition**: Any frame with `lane_type` NOT in `[Driving, Shoulder, Sidewalk]` (unless specific off-road scenarios are intended).

### 1.2 UAV Following & Stability
To ensure the drone is maintaining the "Consistent Pose" rule and tracking the UGV effectively:
- **Precision Check**: Calculate the variance of UAV Altitude and Pitch within a sequence.
- **Metric**: Tolerance should be $\sigma < 0.05$ for both altitude and pitch.
- **Occlusion Check**: Automated check for "Black Pixels" or "Building Collisions" in the aerial RGB stream. If the drone clips through a building, the sequence must be discarded.

## 2. Synchronization Validation

### 2.1 Cross-Perspective Sync (Visual)
Since UGV and UAV operate in different physics engines (CARLA and AirSim), we must verify zero-latency sync:
- **Verification Script**: Generate a 2Hz "Sync GIF" showing the UGV front camera and UAV aerial camera side-by-side.
- **Visual Marker**: Confirm that dynamic actors (e.g., a specific red car) are at the exact same lane position and intersection phase in both views.

## 3. Sensor Data Integrity

### 3.1 Depth-to-RGB Alignment
To verify the accuracy of the saved Ray Maps and Depth maps:
- **Reprojection Test**: Randomly select 1% of frames and reproject the Depth map into a 3D point cloud using the Ray Map.
- **Validation**: Project the Lidar points into the same frame. The Lidar points must align perfectly with the Depth edges.

### 3.2 Semantic Tag Consistency
- **Check**: Verify that the class IDs in the Segmentation map correspond to the correct object types (e.g., Lidar points on a car must have the `Vehicle` semantic tag).

## 4. Manifest & Metadata QA

### 4.1 OpenLabel Schema Validation
- **Automated Test**: Run the `master_manifest.jsonl` through an ASAM OpenLabel 1.0.0 schema validator.
- **Integrity**: Verify that every `frame_id` in the manifest has corresponding files in `data/rgb/`, `data/depth/`, etc.

### 4.2 Scenario Diversity Report
After each regional capture (e.g., 1000 frames), generate a summary report:
- Distribution of **Altitudes** (15m vs 90m).
- Distribution of **Pitches** (Nadir vs Oblique).
- Distribution of **Weather** and **Illumination** states.
