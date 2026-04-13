# Ticket T04: Geometric Modalities & Ray Maps

## Description
implement advanced data processing for geometric modalities, specifically Ray Maps and 16-bit depth.

## Tasks
- [ ] **Ray Map Calculation**
    - [ ] Implement utility to calculate per-pixel unit rays from camera intrinsics.
    - [ ] Save Ray Maps as `.npy` artifacts.
- [ ] **Depth Processing**
    - [ ] Save synchronized 16-bit depth maps (Linear Depth).
    - [ ] Implement depth-to-pointcloud reprojection utility for QA.

## References
- [ARCHITECTURE.md](../dataset_generation/ARCHITECTURE.md) (Section 4)
- [VERIFICATION.md](../dataset_generation/VERIFICATION.md) (Section 3)
