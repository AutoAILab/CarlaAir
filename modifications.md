# CarlaAir

## Simulator 

CarlaAir (louiszengCN/CarlaAir: CarlaAir: Fly Drones Inside a CARLA World!! A Unified Infrastructure for Air-Ground Embodied Intelligence) 

Objective: Have a simple, yet powerful simulator that can represent UAV and UGV in outdoor environments under different conditions and record data using multiple sensor modalities. 

Simple but powerful 

Able to run/start with minimal intermediate steps (don’t have to be an übernerd to run) 

Note that SkyScenes also might be able to run inside of CarlaAir, as Carla API remains unchanged 

UxV representation 

It must be able to accurately represent UxVs 

Outdoor scenes 

It must be able to accurately represent outdoor environments in various conditions, including adverse ones, such as rain, night, etc. 

Scripts 

There should be a collection of clear, logically defined scripts that provide various functionality 

Must be simple yet not simplistic, as well as modular and maintainable 

Data capture 

There must be scripts that allow for the automated, reproducible capturing of data from UAV and UGV perspectives 

Must be able to capture different sensor modalities including depth maps, ray maps, rgb images, and LiDAR  

Additional requirements 

No datasets are stored in the simulator, as it is out of scope 

No evaluations are performed inside of the simulator, as they are out of scope 

CarlaAir highlights 

Single process composition (aligns with simple requirement) 

Absolute coordinate alignment (aligns with data capture) 

18 Sensor modalities (aligns with data capture) 

Zero-modification code migration (aligns with simple requirement) 

~20 FPS joint workloads (aligns with simulation capabilities and UxV representation) 

3-hour stability verified (aligns with simple) 

Built-in FPS drone control (aligns with UxV representation) 

Realistic urban traffic (aligns with UxV representation and outdoor scenes) 

Extensible asset pipeline (aligns with UxV representation and outdoor scenes) 