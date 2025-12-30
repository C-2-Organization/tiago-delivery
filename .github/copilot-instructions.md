# Copilot Instructions for TIAGo Delivery Robot

## Project Overview
- **Purpose:** Autonomous indoor delivery robot for apartment buildings, using Isaac Sim and ROS2.
- **Major Components:**
  - **Orchestrator:** State machine/task manager (see `ros2_ws/src/orchestrator/`)
  - **Navigation:** Nav2-based, handles indoor movement and elevator logic (`ros2_ws/src/navigation/`)
  - **Perception:** OCR and vision for reading delivery labels and detecting objects (`ros2_ws/src/perception/`)
  - **Manipulation:** MoveIt-based pick & place (`ros2_ws/src/manipulation/`)
  - **Simulation:** Isaac Sim environments and robot models (`external/tiago_isaac/`)

## Developer Workflows
- **Initial Setup:**
  - Run `./setup.sh` to fetch dependencies and set up symlinks.
  - Use `vcstool` for dependency management (see `dependencies.repos`).
- **Dockerized ROS2:**
  - Build image: `docker build -t tiago-delivery-ros2 -f docker/Dockerfile.ros2-humble .`
  - Use provided aliases (source `docker/aliases.sh`):
    - `isaac-sim`: Launch Isaac Sim
    - `tiago-ws`: Start ROS2 container
    - `tiago-ws-build`: Rebuild and start container
- **ROS2 Workspace:**
  - Enter container: `tiago-ws`
  - Build: `cd ros2_ws && colcon build --symlink-install`
  - Source: `source install/setup.bash`
- **Simulation:**
  - Launch Isaac Sim, load `tiago_dual_functional_light.usd` from `external/tiago_isaac/`
  - Enable `omni.tiago_omnidirectional` extension (see `exts/`)
- **Testing Communication:**
  - Use `ros2 topic list` to verify topics (e.g., `/cmd_vel`, `/joint_states`, `/scan_front`)
  - Publish test command: `ros2 topic pub /cmd_vel geometry_msgs/msg/Twist ...`

## Project Conventions
- **ROS_DOMAIN_ID:** Set to `110` for all ROS2 nodes.
- **RMW_IMPLEMENTATION:** Use `rmw_cyclonedds_cpp`.
- **External dependencies:** Managed via `dependencies.repos` and `external/`.
- **Simulation assets:** All USD and robot files in `external/tiago_isaac/`.
- **Custom messages/services:** Place in `ros2_ws/src/interfaces/`.

## Integration Points
- **Isaac Sim <-> ROS2:**
  - Communication via ROS2 bridge, topics as above.
  - Extensions in `external/tiago_isaac/exts/`.
- **OCR:** Perception node uses OCR for label reading (see `perception/`).
- **Elevator logic:** Navigation node handles elevator calls and rides.

## Troubleshooting
- **Extension not loading:** Check extension path in Isaac Sim (`exts/`).
- **Topics missing:** Verify domain ID and RMW implementation.
- **Robot not moving:** Ensure simulation is playing and extension is enabled.

## References
- See `README.md` for full setup, architecture, and troubleshooting details.
- Key files/directories: `setup.sh`, `docker/aliases.sh`, `external/tiago_isaac/`, `ros2_ws/src/`
