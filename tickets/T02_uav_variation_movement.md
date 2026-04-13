# Ticket T02: Swarm Coordination & Chase Logic

## Description
implement the flight logic for the 5-drone swarm, ensuring they maintain relative perspectives to the UGV during the capture sequence.

## Tasks
- [ ] **Swarm Configuration**
    - [ ] Update `settings.json` with 5 drone definitions (`UAV_1` to `UAV_5`).
- [ ] **Chase Behavior**
    - [ ] Implement UGV "Chase" mode for the entire swarm.
    - [ ] **Perspective Offsets**: Assign fixed height/pitch per drone (15m, 30m, 45m, 60m, 90m).
- [ ] **Movement Stability**
    - [ ] Ensure smooth transform application during Replay.

## References
- [ARCHITECTURE.md](../dataset_generation/ARCHITECTURE.md) (Section 3)
- [SCENARIO_METADATA.md](../dataset_generation/SCENARIO_METADATA.md) (Section 1)
