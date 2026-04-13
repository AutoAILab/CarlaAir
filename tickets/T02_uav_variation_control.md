# Ticket T02: UAV Variation & Pose Control

## Description
Implement the randomized yet consistent UAV pose logic and the UGV following behavior.

## Tasks
- [ ] **Adaptive Flight Logic**
    - [ ] Implement "Follow-Lead" logic where UAV tracks UGV position.
    - [ ] Add randomization for UAV Altitude (15m - 90m) per sequence.
    - [ ] Add randomization for UAV Pitch (0° to -90°) per sequence.
    - [ ] **Consistency Check**: Ensure pose remains constant ($\sigma < 0.05$) throughout the capture run.
- [ ] **Obstacle Avoidance**
    - [ ] Implement height-clamping to prevent building collisions in urban maps.

## References
- [ARCHITECTURE.md](../dataset_generation/ARCHITECTURE.md) (Section 6)
- [SCENARIO_METADATA.md](../dataset_generation/SCENARIO_METADATA.md) (Section 1)
