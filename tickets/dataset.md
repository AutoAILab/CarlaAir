# AGCarla dataset 

## Description
A ground/air dataset for autonomous driving research. 

The purpose of this ticket is to create a detailed plan for creating a dataset with the properties specified in this document. Once a master plan is established, tickets will be made for each of the subtasks. The user should be relied upon for creation of the master plan.

## Dataset requirements 
- 3000-5000 images, maybe 10,000+ images 
- UAV and UGV perspectives 

## Format/Structure
Data is located here: /home/df/data/datasets/AGCarla/data
README for dataset: /home/df/data/datasets/AGCarla/README.md
Master Manifest: /home/df/data/datasets/AGCarla/master_manifest.jsonl

Datatypes: 
- ?


Perspectives: 
- UGV 
- UAV 
    - Height should range from 15 to 90 meters 
    - Pitch should range from 0 (looking forward) to -90 (nadir) degrees 

---

## Sub-tickets
- [x] **[T01: Core Generation Script Setup](file:///home/df/data/jflinte/CarlaAir/tickets/T01_core_datagen_script.md)**
- [ ] **[T02: Swarm Coordination & Control](file:///home/df/data/jflinte/CarlaAir/tickets/T02_swarm_coordination.md)**
- [ ] **[T03: OpenLabel Manifest Generation](file:///home/df/data/jflinte/CarlaAir/tickets/T03_openlabel_manifest.md)**
- [ ] **[T04: Projection Validation & Ray Maps](file:///home/df/data/jflinte/CarlaAir/tickets/T04_geometric_modalities.md)**
- [ ] **[T05: Pilot Run & Verification](file:///home/df/data/jflinte/CarlaAir/tickets/T05_verification_pilot.md)**
- [ ] **[T06: Production Generation](file:///home/df/data/jflinte/CarlaAir/tickets/T06_production_run.md)**
- [ ] **[T07: G-Buffer & Auxiliary Modalities](file:///home/df/data/jflinte/CarlaAir/tickets/T07_gbuffer_extraction.md)**
- [ ] **[T08: Global Mapping & Open3D Merging](file:///home/df/data/jflinte/CarlaAir/tickets/T08_global_mapping.md)**
- [ ] **[T09: 3D Multi-View Fusion Script](file:///home/df/data/jflinte/CarlaAir/tickets/T09_gt_fusion_script.md)**
