# Last 100m Delivery Robot

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

- Ubuntu 22.04
- ROS2 Humble
- NVIDIA Isaac Sim 5.0.0
- NVIDIA RTX GPU (for Isaac Sim)
- Python 3.10

## Installation

### 1. Clone the Repository

```bash
git clone https://github.com/C-2-Organization/tiago-delivery.git
cd tiago-delivery
```

### 2. Run Setup Script

The setup script will fetch external dependencies and configure symbolic links.

```bash
chmod +x ./setup.sh
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

## Usage

### Launch Simulation

```bash
# Terminal 1: Set ROS domain ID
export ROS_DOMAIN_ID=110 # 106 ~ 110 are available

# Open Isaac Sim and load the scene
# File: external/tiago_isaac/tiago_dual_functional_light.usd
```

### Run Navigation Stack

```bash
# Terminal 2
export ROS_DOMAIN_ID=110
source ros2_ws/install/setup.bash
ros2 launch navigation nav_launch.py
```

### Run Perception

```bash
# Terminal 3
export ROS_DOMAIN_ID=110
source ros2_ws/install/setup.bash
ros2 launch perception perception_launch.py
```

### Run Full System

```bash
# Terminal 4
export ROS_DOMAIN_ID=110
source ros2_ws/install/setup.bash
ros2 launch orchestrator full_system_launch.py
```

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
