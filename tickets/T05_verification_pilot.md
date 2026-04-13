# Ticket T05: Pilot Run & Verification (Dry Run)

## Description
Execute a small-scale pilot run (50-100 frames) to verify the entire pipeline from capture to manifest generation.

## Tasks
- [ ] **Execution**
    - [ ] Run 50-frame sync sequence in `Town10HD`.
- [ ] **Validation**
    - [ ] Export 2Hz side-by-side **Sync GIFs** to verify zero-latency UGV/UAV alignment.
    - [ ] Validate `master_manifest.jsonl` against ASAM OpenLabel schema.
    - [ ] **Geometric Test**: Reproject Depth-to-Pointcloud and verify alignment with raw Lidar data.

## References
- [VERIFICATION.md](../dataset_generation/VERIFICATION.md) (All Sections)
- [TASKS.md](../dataset_generation/TASKS.md) (Milestone 5)
