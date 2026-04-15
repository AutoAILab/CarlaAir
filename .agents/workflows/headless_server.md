---
description: Workflow for starting and managing the CarlaAir co-simulation server in headless mode.
---

# Headless Server Workflow

This workflow describes how to launch the CarlaAir simulator (CARLA + AirSim) in headless mode, which is essential for remote servers or automated data generation pipelines that do not require a graphical display.

## 1. Prerequisites
- **Environment**: Ensure the Python environment is activated (preferably via `uv`).
  ```bash
  source .venv/bin/activate
  ```
- **Dependencies**: The system should have necessary Vulkan/OpenGL drivers installed.

## 2. Launching the Server
The main launcher `carlaAir.sh` handles the headless configuration automatically using the `--headless` flag.

### Command
```bash
./carlaAir.sh [MAP_NAME] --headless
```
*Example:* `./carlaAir.sh Town10HD --headless`

### What happens in Headless mode:
- The `-RenderOffScreen` flag is passed to the Unreal Engine binary.
- Window resolution is set to the default (1280x720) but no window is created.
- Audio and user attendance are disabled (`-nosound -unattended`).
- Logs are redirected to `CarlaAir.log`.
- **Auto-Traffic**: If a valid Python environment is found, `auto_traffic.py` is automatically launched in the background to populate the city.

## 3. Verification
Once started, the script will wait for the RPC ports to become active. You can manually verify the status.

### Check Active Ports
```bash
ss -tlnp | grep -E '2000|41451'
```
- Port `2000`: CARLA RPC Server
- Port `41451`: AirSim RPC Server

### Monitor Logs
```bash
# Live main simulator log
./carlaAir.sh --log

# Traffic generation log
tail -f traffic.log
```

## 4. Stopping the Server
To ensure a clean exit and destroy all spawned actors (including traffic), use the built-in kill command.

### Command
```bash
./carlaAir.sh --kill
```
This command sends a `SIGTERM` to the traffic script for graceful actor cleanup before terminating the main Unreal Engine process.

## 5. Troubleshooting
- **Process Died**: If the server fails to start, check `CarlaAir.log` for Vulkan or Driver initialization errors.
- **Vulkan Errors**: If Vulkan is not available on the server, try the OpenGL fallback:
  ```bash
  ./carlaAir.sh --headless --opengl
  ```
