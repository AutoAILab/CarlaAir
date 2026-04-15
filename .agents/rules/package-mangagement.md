---
trigger: always_on
glob: "*.sh, *.py"
description: Standardized package and environment management protocols for CarlaAir.
---

# Package Management Rules

## 1. Preferred Manager: UV
- **Standard**: Always prefer `uv` for environment management and package installation.
- **Workflow**: 
  - Initialization: `bash env_setup/setup_env_uv.sh`
  - Activation: `source .venv/bin/activate`
  - **Execution**: Always run scripts using `uv run <script_name>` or by explicitly using the `.venv/bin/python` interpreter.
- **Rationale**: `uv` provides significantly faster installation and more reproducible environment locking compared to Conda.

## 2. Legacy Support: Conda
- **Status**: Deprecated/Alternative.
- **Workflow**: `bash env_setup/setup_env.sh` followed by `conda activate carlaAir`.
- **Constraint**: Do not remove legacy Conda support unless explicitly requested, but always default new documentation to UV.

## 3. Carla Python Module
- **Mandate**: The system relies on a pre-compiled Carla module (`carla_python_module.tar.gz`). 
- **Rule**: When setting up an environment, this module must be extracted directly into the `site-packages` of the target environment to ensure ABI compatibility with the simulation server.
