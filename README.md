# TIAGo Delivery Robot

An autonomous indoor delivery robot system for apartment buildings, built with Isaac Sim and ROS2.

## Overview

### Problem Statement

In delivery services, the "last mile" accounts for approximately 53% of total logistics costs. Within this segment, the **"last 100 meters"**—from a building's entrance to each residential unit—presents significant inefficiencies, particularly in Korean high-rise apartments.

Delivery drivers must navigate security entrances, wait for elevators, and traverse multiple floors repeatedly. A single apartment building delivery can occupy an elevator for 10-20 minutes, causing friction with residents. This creates problems for all parties: drivers waste time on building navigation, residents face security concerns with strangers in their building, and elevator congestion leads to conflicts.

### Solution

We propose a dedicated indoor delivery robot that handles the last 100m of delivery:

1. Delivery drivers drop packages at a designated drop zone at the building entrance
2. The robot autonomously picks up packages, reads delivery labels via OCR, and navigates to each unit
3. Packages are placed at residents' doorsteps

This approach reduces driver workload, enhances building security, and eliminates elevator congestion issues.

### Key Objectives

| Objective | Description | Technology |
|-----------|-------------|------------|
| Simulation Environment | Apartment environment with lobby, elevator, and corridors | Isaac Sim |
| Indoor Navigation | Autonomous navigation with obstacle avoidance and elevator handling | Nav2 |
| Destination Recognition | Read apartment/unit numbers from delivery labels | OCR |
| Object Manipulation | Pick packages from drop zone and place at destinations | MoveIt |
| E2E Validation | Complete delivery scenario testing in simulation | Isaac Sim + ROS2 |

## System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        Orchestrator                              │
│                    (State Machine / Task Manager)                │
└──────────┬───────────────────┬───────────────────┬──────────────┘
           │                   │                   │
           ▼                   ▼                   ▼
    ┌─────────────┐     ┌─────────────┐     ┌─────────────┐
    │  Navigation │     │ Perception  │     │Manipulation │
    │    (Nav2)   │     │ (OCR/Vision)│     │  (MoveIt)   │
    └─────────────┘     └─────────────┘     └─────────────┘
           │                   │                   │
           └───────────────────┴───────────────────┘
                               │
                               ▼
                    ┌─────────────────────┐
                    │   Isaac Sim + ROS2  │
                    │   (TIAGo++ Robot)   │
                    └─────────────────────┘
```

## Repository Structure

```
tiago-delivery/
├── README.md
├── dependencies.repos          # External dependencies (vcs)
├── setup.sh                    # Initial setup script
│
├── external/                   # External repos (gitignored)
│   └── tiago_isaac/            # TIAGo++ Isaac Sim simulation
│
├── ros2_ws/
│   └── src/
│       ├── tiago_rviz2/        # Symlink to tiago_isaac package
│       ├── simulation/         # Isaac Sim environments
│       ├── navigation/         # Nav2 configuration
│       ├── manipulation/       # MoveIt pick & place
│       ├── perception/         # Vision & OCR
│       ├── interfaces/         # Custom msg/srv/action
│       └── orchestrator/       # Task management
│
└── docker/                     # Docker configurations
```

## Prerequisites

- AWS EC2 with NVIDIA GPU (g5.2xlarge or higher recommended)
- NVIDIA Isaac Sim 4.2.0
- Docker
- Python 3.10

## Environment Setup (AWS EC2)

This project is designed to run on AWS EC2 with NVIDIA's official Isaac Sim AMI and Docker-based ROS2 Humble environment.

### 1. Launch EC2 Instance

1. **AMI**: Search for "NVIDIA Isaac Sim" in AWS Marketplace
   - Select the Ubuntu 24.04 based official NVIDIA AMI
   - This comes with Isaac Sim pre-installed at `/opt/IsaacSim`

2. **Instance Type**: `g5.2xlarge` or higher (RTX GPU required)

3. **Storage**: 200GB+ recommended

4. **Security Group**: Open the following ports
   | Port | Protocol | Purpose |
   |------|----------|---------|
   | 22 | TCP | SSH |
   | 49100 | TCP | Isaac Sim WebRTC Streaming |
   | 8211 | TCP | Isaac Sim Livestream |
   | 8899 | TCP | Isaac Sim Nucleus |

### 2. Initial EC2 Setup

SSH into your instance and run:

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install Docker (if not installed)
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker $USER
newgrp docker

# Install vcstool (for dependency management)
pip install vcstool
```

### 3. Clone and Setup Project

```bash
cd /home/ubuntu
git clone https://github.com/C-2-Organization/tiago-delivery.git
cd tiago-delivery

# Fetch external dependencies (tiago_isaac)
./setup.sh
```

### 4. Build Docker Image

```bash
cd /home/ubuntu/tiago-delivery
docker build -t tiago-delivery-ros2 -f docker/Dockerfile.ros2-humble .
```

### 5. Configure Shell Aliases

Add aliases to your `.bashrc` for convenient usage:

```bash
echo "source /home/ubuntu/tiago-delivery/docker/aliases.sh" >> ~/.bashrc
source ~/.bashrc
```

This provides the following commands:

| Alias | Description |
|-------|-------------|
| `isaac-sim` | Launch Isaac Sim with streaming and TIAGo extension |
| `tiago-ws` | Start ROS2 Docker container |
| `tiago-ws-build` | Rebuild Docker image and start container |

### 6. Build ROS2 Workspace

```bash
# Enter the container
tiago-ws

# Inside container: run setup and build
cd /home/ros/tiago-delivery
./setup.sh
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Running the System

### Terminal 1: Launch Isaac Sim

```bash
# On EC2 host (not in Docker)
isaac-sim
```

Access the streaming viewer at:
```
https://<your-ec2-public-ip>:49100/streaming/webrtc-client
```

Then in Isaac Sim:
1. Open `File → Open`
2. Navigate to `/home/ubuntu/tiago-delivery/external/tiago_isaac/`
3. Load `tiago_dual_functional_light.usd`
4. Press `Play` to start simulation

### Terminal 2: Launch ROS2 Nodes

```bash
# Enter container
tiago-ws

# Inside container
source /home/ros/tiago-delivery/ros2_ws/install/setup.bash

# Test robot control
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {z: 0.0}}" --once
```

### Verify Communication

```bash
# In container - check available topics
ros2 topic list

# Expected topics from Isaac Sim:
# /cmd_vel
# /joint_command
# /joint_states
# /tf
# /scan_front
# /scan_rear
# /gemini2/rgb
# /gemini2/depth
```

## Installation (Local Development)

If you're running locally instead of EC2, ensure you have:
- Ubuntu 22.04
- ROS2 Humble
- NVIDIA Isaac Sim 5.0.0 installed
- NVIDIA RTX GPU

### 1. Clone the Repository

```bash
git clone https://github.com/C-2-Organization/tiago-delivery.git
cd tiago-delivery
```

### 2. Run Setup Script

The setup script will fetch external dependencies and configure symbolic links.

```bash
./setup.sh
```

This will:
- Clone `tiago_isaac` into `external/` directory
- Create `ros2_ws/src/` structure
- Set up symbolic links for TIAGo ROS2 packages

### 3. Build ROS2 Workspace

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 4. Configure Isaac Sim Extension

Add the TIAGo extension path to Isaac Sim:

1. Open Isaac Sim
2. Navigate to `Window → Extensions`
3. Add extension search path:
   ```
   <project_root>/external/tiago_isaac/exts/
   ```
4. Enable `omni.tiago_omnidirectional` extension

## Troubleshooting

### Isaac Sim extension not loading
Ensure the extension path is correctly set:
```bash
# Check if extension folder exists
ls /home/ubuntu/tiago-delivery/external/tiago_isaac/exts/

# Manually add in Isaac Sim:
# Window → Extensions → Gear icon → Add search path
```

### ROS2 topics not visible
```bash
# Verify ROS_DOMAIN_ID matches
echo $ROS_DOMAIN_ID  # Should be 110

# Check RMW implementation
echo $RMW_IMPLEMENTATION  # Should be rmw_cyclonedds_cpp
```

### Robot not moving with /cmd_vel
1. Ensure Isaac Sim simulation is playing (not paused)
2. Verify `omni.tiago_omnidirectional` extension is enabled
3. Check the action graph is loaded in USD file

## Robot Platform

This project uses the **TIAGo++** robot simulation from [AIS-Bonn/tiago_isaac](https://github.com/AIS-Bonn/tiago_isaac).

**Key features:**
- Dual-arm mobile manipulator
- Mecanum wheel omnidirectional base
- RGB-D camera (Gemini2)
- Dual laser scanners
- Full ROS2 integration

## Team

**Team Name:** TBD

| Role | Member | Responsibilities |
|------|--------|------------------|
| Simulation | - | Isaac Sim environment, robot setup |
| Navigation | - | Nav2 configuration, elevator integration |
| Manipulation | - | MoveIt pick & place |
| Perception | - | Box detection, grasp estimation, OCR |
| Integration | - | State machine, task orchestration |

## References

- [TIAGo++ Isaac Sim](https://github.com/AIS-Bonn/tiago_isaac)
- [Nav2 Documentation](https://docs.nav2.org/)
- [MoveIt 2 Documentation](https://moveit.picknik.ai/)
- [Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/)

## License

This project is licensed under the [MIT License](LICENSE).

Note: The `tiago_isaac` dependency is licensed under AGPL-3.0.
