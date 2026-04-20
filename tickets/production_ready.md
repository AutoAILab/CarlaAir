# Production Ready

## Description
This repository needs to be production ready. This means that it needs to be checked for bugs and errors. Best practices in the code should be followed and maintained. Everything should be modular and easy to understand. 

## Best Practices & Standard Guidelines

To achieve production readiness, the repository enforces the following standards:

1. **Package Management (`uv`)**
   - **Preference:** `uv` is the standard for fast and reproducible environment locking.
   - **Execution:** Always prefix script executions utilizing dependent dependencies with `uv run ...` or directly via `.venv/bin/python`.
   - **Environment:** Initialize environments via provided setup scripts (e.g., `bash env_setup/setup_env_uv.sh`) and explicitly place custom `.tar.gz` dependencies into `site-packages`.

2. **Architecture & Simulator Scope**
   - **Single Process:** Both CARLA and AirSim APIs run within the same Unreal Engine process (Ports `2000` and `41451` respectively) mapped without RPC bridges dynamically translating timestamps.
   - **Zero Error Rule:** The simulation assumes $0.000m$ alignment errors across coordinate boundaries.
   - **Script Locality:** All automation logic, test suites, and route configuration must exist independently of the foundational physics environments (outside core Unreal Engine C++ space).

3. **Python Script Lifecycle & Error Handling**
   - **Strict Teardown:** Simulation states are persistent. All operational scripts directly contacting APIs *must* wrap connections in robust `try...except...finally` block definitions to gracefully stop sensors, disarm control APIs, and destroy actors on completion or failure.
   - **Pre-execution Cleans:** Begin complex spawn operations by utilizing or initiating routines identical to `cleanup_previous()` to ensure previous orphaned sessions don't clutter the engine tick limits.
   - **Modularity:** Ensure clear distinction between generation logic (`datagen`), parsing routines, and verification logic.
   
4. **Coordinate Transformation Consistency**
   - **World Alignment:** Apply transformations systematically when bridging spaces.
   - Ensure the use of mathematically coherent mappings such as explicit Left-Hand Z-Up (CARLA) to Right-Hand NED dimensions (AirSim).
   - Use standard `GeometryUtils` when mapping point sets.

5. **Logging**
   - Deprecate bare `print()` outputs across operational python logic.
   - Implement `logging.INFO` for overarching startup/teardown processes.
   - Use `logging.DEBUG` strictly where localized metrics, diagnostic readouts, or specific actor states are traced per frame.

## Tasks
- [x] Compilation of Best Practices.
- [ ] Refactoring `dataset_generation` scripts formatting to incorporate Logging module and `try...finally` boundaries.
- [ ] Verify `uv` documentation paths on relevant files.

## Technical Details

## References
- `ARCHITECTURE.md`
- `package-mangagement.md`
- `programming-guidelines.md`
