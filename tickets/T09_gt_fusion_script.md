# Ticket T09: 3D Multi-View Fusion Script

## Description
Develop a specialized post-processing utility to generate high-fidelity, unified ground-truth pointclouds by fusing 6 synchronized perspectivas of the same Air-Ground scenario.

## Tasks
- [ ] **Reprojection Engine**
    - [ ] Implement `fuse_frame_pcs.py` (located in `./dataset_generation`).
    - [ ] Use **Ray Maps** and intrinsic camera matrices for depth-pixel-to-3D-point conversion.
    - [ ] Apply global CARLA transforms to align the 6 views into a single coordinate space.
- [ ] **Cloud Normalization**
    - [ ] Implement voxel-based downsampling (default $0.05m$) to ensure uniform point density.
    - [ ] Export final Ground Truth as `.ply` or `.pcd` files.
- [ ] **Verification**
    - [ ] Verify geometric alignment of fused clouds against a sample baseline simulation.

## References
- [ARCHITECTURE.md](../dataset_generation/ARCHITECTURE.md) (Section 6)
- [TASKS.md](../dataset_generation/TASKS.md) (Milestone 8)
