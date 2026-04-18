Integrating a full **LiDAR reconstruction** (a global point cloud or mesh) into an OpenLABEL dataset requires moving beyond the "per-frame" sensor logic. In OpenLABEL, you treat a global reconstruction as a **Static Resource** or a **Context Stream** that anchors all other temporal data.

Here is how you would organize and code this into your `master_manifest.json`.

### 1. The Physical Organization
Store your reconstruction in a dedicated `maps/` or `reconstructions/` folder. For large-scale outdoor environments, **LAS/LAZ** or **PLY** are standard, but if you're working with Neural Reconstruction (like Gaussian Splatting), you might store a `.ply` or `.exr` feature map.

```text
ground_air_dataset/
├── data/
│   └── (per-frame rgb, lidar, depth...)
├── maps/
│   └── town_01_global_reconstruction.laz  <-- The full reconstruction
└── master_manifest.json
```

---

### 2. Defining the Global Stream
In your `streams` section, you declare the reconstruction as a non-temporal data source. You use a custom `stream_type` and link it to your global coordinate system.

```json
"streams": {
  "global_map_lidar": {
    "type": "other", 
    "uri": "maps/town_01_global_reconstruction.laz",
    "stream_properties": {
      "is_static": true,
      "coordinate_system": "world",
      "encoding": "point_cloud_binary"
    }
  }
}
```

---

### 3. The Coordinate System Tree
This is the "glue" that makes the reconstruction useful. You define a **world** frame. Every frame in your sequence will then provide an `ego_pose` that tells the model exactly where the drone/car is within that global reconstruction.

```json
"coordinate_systems": {
  "world": {
    "type": "global",
    "parent": "" 
  },
  "ego_vehicle": {
    "type": "local",
    "parent": "world",
    "pose_wrt_parent": {
      "translation": [x, y, z],
      "quaternion": [qw, qx, qy, qz]
    }
  }
}
```

---

### 4. Semantic Relationships
To make your dataset "intelligent," you can use the `relations` module to link specific per-frame LiDAR sweeps to the global map. This is vital for **Test-Time Training (TTT)**, as it tells the optimizer exactly which raw data points correspond to which parts of the reconstructed "truth."

```json
"relations": {
  "rel_001": {
    "name": "is_registered_to",
    "type": "spatial",
    "rdf_subjects": ["stream/lidar_front_frame_001"],
    "rdf_objects": ["stream/global_map_lidar"]
  }
}
```

---

### 5. Why this is the "Best Practice"
By putting the reconstruction in OpenLABEL format rather than just a loose file:
* **Queryable Metadata:** You can store the reconstruction method (e.g., "COLMAP," "LIO-SAM") and the precision (e.g., "0.02m RMSE") directly in the JSON.
* **Synchronized Benchmarking:** Your benchmarking suite can automatically pull the `world` transform to align DepthAnything3’s relative depth predictions with the absolute "truth" of the reconstruction.
* **Sub-mapping:** If you have a massive town, you can define "Contexts" that point to specific *sub-sections* of the reconstruction URI, so you don't load a 10GB file into RAM just to evaluate a 50-meter sequence.



This setup transforms your data from a collection of files into a **Digital Twin** environment, which is the exact standard used by European automotive labs for validating vision foundation models.

## Current Technical Implementation (Updated: 2026-04-18)

The LiDAR and OpenLABEL integration is now **fully implemented** and active in the `agcarla_datagen.py` pipeline.

### 1. Sensor Configuration
- **Type**: `sensor.lidar.ray_cast` (CARLA native).
- **Mount Point**: Mounted on `UGV_1` at `carla.Transform(carla.Location(z=2.5))`.
- **Coordinate System**: All sensors are synchronized and registered to the `world` root frame using unit quaternions `[qw, qx, qy, qz]`.

### 2. Data Storage & Formatting
- **Storage Path**: `data/lidar/ugv_{frame_id:06d}.npy`
- **Global Map**: Generated automatically during the finalization phase as `maps/global_reconstruction.ply`. This map is fused from UGV LiDAR and UAV Depth sensors with a 10cm voxel downsampling.

### 3. OpenLABEL Manifest Integration
- **Status**: **✅ Completed**.
- **Execution**: The `.jsonl` frame logs are consolidated into a structured `master_manifest.json` at the conclusion of every run.
- **Relations**: Every frame includes `is_registered_to` spatial relations (Option B), linking UGV and UAV observations directly to the global reconstruction resource.

### 4. Verification & Fusion
- **Validation**: Use `verify_geometry.py` to check real-time alignment.
- **Fusion**: The generator now integrates the fusion logic natively, so manual use of `verify_fusion.py` is only required for debugging specific frame slices.

---

The system now uses a ground-truth fusion engine to generate the global reconstruction `.ply` as described above.