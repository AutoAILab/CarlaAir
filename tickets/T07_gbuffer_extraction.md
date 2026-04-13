# Ticket T07: G-Buffer & Auxiliary Modalities

## Description
Leverage the Unreal Engine G-Buffer to extract high-fidelity auxiliary modalities for the UGV and the 5-drone swarm.

## Tasks
- [ ] **G-Buffer Listener**
    - [ ] Implement `listen_to_gbuffer` for the following textures:
        - `GBufferA` (Scene Normals)
        - `SceneDepth` (Linear Depth)
        - `GBufferB` (Surface Physics)
        - `Velocity` (Motion Flow)
        - `SceneStencil` (Object Masks)
- [ ] **Data Bundling**
    - [ ] Implement automated bundling of these textures into per-frame **`.npz`** files for ALL drones in the swarm.
- [ ] **Verification**
    - [ ] Verify that G-Buffer data is correctly synchronized across the swarm.

## References
- [ARCHITECTURE.md](../dataset_generation/ARCHITECTURE.md) (Section 5)
- [SCENARIO_METADATA.md](../dataset_generation/SCENARIO_METADATA.md) (Section 2)
