# Package Management with UV

## Description
Migrate the project's default package management strategy from Conda to UV to improve environment configuration speed and reliability.

## Context
CarlaAir currently uses `conda` to orchestrate Python configuration (e.g., `carlaAir` env with Python 3.10) prior to injecting a custom-built Carla Python wheel (via `env_setup/setup_env.sh`). Given UX issues from Conda's installer speed and resolution limits, we will offer/switch to `uv` as the primary environment manager. 

## Constraints & Style
- Ensure there is NO sensitive information.
- Follow `extension-patterns`: do not directly demolish the old `conda` dependencies in core system configurations if they are deeply intertwined, but instead create isolated UV variants or gently extend the existing bash checks.
- Do not modify core UE4 dependencies or Carla core upstream C++ files.
- The pre-built Carla module (`carla_python_module.tar.gz`) extraction into `site-packages` must be perfectly preserved in the `.venv` directory instead of the Conda directory.

## Acceptance Criteria
- `env_setup/setup_env_uv.sh` (or `setup_env.sh` updated) flawlessly provisions a UV `.venv` with `python=3.10`, `pygame`, `airsim`, `numpy`, and `Pillow`.
- Pre-compiled `carla` module correctly extracts into the `.venv` site-packages.
- `env_setup/test_env.sh` passes successfully within the new `.venv`.
- `VIPR_README.md` and `VIPR_CHANGELOG.md` are updated to document the UV adoption.

## Execution Plan
1. Create `env_setup/setup_env_uv.sh` based on `setup_env.sh`, altering the initialization from `conda create` to `uv venv` and `uv pip install`.
2. Extract the `carla_python_module.tar.gz` to the `.venv`'s `site-packages` accurately.
3. Update `env_setup/test_env.sh` to correctly check the `.venv` path if necessary.
4. Test the pipeline by activating the `uv` virtual environment and validating the Python bindings.
5. Update `VIPR_README.md` instructions and log the feature in `VIPR_CHANGELOG.md`.
