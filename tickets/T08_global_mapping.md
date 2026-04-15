# Ticket T08: Global Mapping & Open3D Merging

## Description
Develop an automated script to accumulate LiDAR segments into a single consistent global map for each town.

## Tasks
- [ ] **Open3D Integration**
    - [ ] Implement LiDAR colorization and global transform application.
    - [ ] Implement voxel downsampling (`voxel_size = 0.1m`) for stability.
- [ ] **Automated Merging Script**
    - [ ] Create `merge_town_maps.py` (in `./dataset_generation`) to crawl the dataset directory, group segments by town, and output a single `.ply` map per town.
- [ ] **QA Verification**
    - [ ] Verify global alignment accuracy between merged segments.

## References
- [TASKS.md](../dataset_generation/TASKS.md) (Milestone 7)
- [VERIFICATION.md](../dataset_generation/VERIFICATION.md) (Section 3.1)
