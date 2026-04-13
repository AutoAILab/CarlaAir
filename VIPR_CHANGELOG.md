# VIPR Features Changelog

## 2026-04-11
- Created `.agents/workflows/ticket.md` and `.agents/workflows/complete.md` to automate VIPR branch and ticket handling.
- Introduced foundational architecture document in `.agents/rules/architecture.md` per VIPR README design tenets and constraints.
- Integrated automated Git-sync checking, explicit dependency on extension-patterns, and project governance review within workflow scripts.

## 2026-04-13
- Completed `modifications.md` task: finalized project workflows, architecture documentation, and ticket lifecycle management.
- Migrated default package management from Conda to UV.
- Created `env_setup/setup_env_uv.sh` for fast, reproducible environment setup.
- Updated `env_setup/test_env.sh` and `carlaAir.sh` to support UV-based virtual environments.
- Initialized `VIPR_README.md` with UV setup guide.
