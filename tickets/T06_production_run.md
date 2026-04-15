# Ticket T06: Production Generation (5000+ Frames)

## Description
Execute the full-scale dataset generation across multiple maps and environmental conditions.

## Tasks
- [ ] **Multi-Town Sweep**
    - [ ] Execute 2,000 frames each in 5 different towns (Town01, Town03, Town04, Town05, Town10HD).
    - [ ] **Swarm Multiplier**: 2,000 steps = 12,000 multi-modal frames (UGV+UAV).
- [ ] **Environmental Variation** (Replay Engine)
    - [ ] Replay all baseline sequences under **Rainy**, **Night**, and **Sunset** variations using the Transform Record.
- [ ] **Final Packaging**
    - [ ] Generate `README.md` for the final dataset release in the target directory.
    - [ ] Perform final manifest integrity check (5,000+ entries).

## References
- [TASKS.md](../dataset_generation/TASKS.md) (Milestone 6)
- [SCENARIO_METADATA.md](../dataset_generation/SCENARIO_METADATA.md)
