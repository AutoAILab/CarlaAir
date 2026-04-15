# Ticket T04: Geometric Modalities & Ray Maps

## Description
implement advanced data processing for geometric modalities, specifically Ray Maps and 16-bit depth.

## Tasks
- [x] **Ray Map Calculation** (Completed in T01)
    - [x] Implement utility to calculate per-pixel unit rays from camera intrinsics.
    - [x] Save Ray Maps as `.npy` artifacts.
- [ ] **Projection Validation**
    - [ ] Implement a reprojection loop to verify $(X, Y, Z)$ consistency using the Ray Map + 16-bit Depth.
    - [ ] Measure sub-pixel reprojection error against a known static actor.

## References
- [ARCHITECTURE.md](../dataset_generation/ARCHITECTURE.md) (Section 4)
- [VERIFICATION.md](../dataset_generation/VERIFICATION.md) (Section 3)
