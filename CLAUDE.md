# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Multi-agent drone formation control research project comparing controller types under wind disturbances:
- **IT2-FLS**: PID + Interval Type-2 Fuzzy Logic (FOU-based uncertainty)
- **GT2-FLS**: PID + General Type-2 Fuzzy Logic (FOU + secondary MF with alpha-planes)
- **PID**: Pure PID baseline

Uses ROS2 Humble with Gazebo simulation (Crazyflie 2.1 physics model).

## Build Commands

```bash
# Source ROS2 (required in every terminal)
source /opt/ros/humble/setup.bash

# Build all packages
colcon build --symlink-install

# Source the workspace (after build)
source install/setup.bash

# Build specific package
colcon build --symlink-install --packages-select agent_control_pkg

# Clean build
./clean_build.sh
```

## Running the Simulation

```bash
# 9-drone comparison (IT2+Fuzzy, PD, PID) - headless
ros2 launch agent_control_pkg formation_comparison_demo.launch.py gazebo_gui:=false rviz:=false

# 6-drone IT2 vs GT2 comparison - headless
ros2 launch agent_control_pkg it2_vs_gt2_comparison.launch.py gazebo_gui:=false

# 12-drone 3-way comparison (IT2, GT2, PID) - headless
ros2 launch agent_control_pkg twelve_drone_comparison.launch.py gazebo_gui:=false

# Quick wind test (60s headless simulation with KPI output)
./scripts/quick_wind_test.sh

# Full system test
./scripts/test_full_system.sh
```

## Running Tests

```bash
# Run all tests
colcon test --packages-select agent_control_pkg

# View test results
colcon test-result --verbose
```

Unit tests are in `agent_control_pkg/test/` using Google Test framework.

## Monitoring Dashboard

```bash
# Terminal 1: Backend (requires ROS2 sourced)
cd monitoring_dashboard/backend
python3 app.py

# Terminal 2: Run simulation
ros2 launch agent_control_pkg formation_comparison_demo.launch.py

# Access: http://localhost:8000/ui
```

## Architecture

### Three-Layer Control Stack

1. **Formation Coordinator** (`formation_coordinator_pkg/`) - Particle swarm optimization for multi-agent target assignment
2. **Agent Controllers** (`agent_control_pkg/src/ros/agent_controller_node.cpp`) - 200Hz control loop with PID/PD/Fuzzy controllers
3. **Gazebo Plugin** (`agent_control_pkg/plugins/simple_drone_plugin.cpp`) - Physics simulation and odometry publishing

### Key Source Files

- `agent_control_pkg/src/pid_controller.cpp` - PID control with anti-windup and derivative filtering
- `agent_control_pkg/src/gt2_fuzzy_logic_system.cpp` - Type-2 fuzzy logic for disturbance compensation
- `agent_control_pkg/src/config_reader.cpp` - YAML configuration parser
- `agent_control_pkg/src/ros/agent_controller_node.cpp` - Main ROS2 control node

### Configuration

Controller parameters are in YAML files:
- `agent_control_pkg/config/ros2/agent_controller_default.yaml` - PID gains, output limits
- `agent_control_pkg/config/fuzzy_params_crazyflie.yaml` - IT2-FLS parameters (FOU bounds)
- `agent_control_pkg/config/gt2_fuzzy_params_crazyflie.yaml` - GT2-FLS parameters (FOU + secondary MF)

### ROS2 Topics (per agent X=0-N)

- `/agent_X/odom` - Position/velocity (nav_msgs/Odometry)
- `/agent_X/cmd_accel` - Control output (geometry_msgs/Vector3)
- `/agent_X/target_pose` - Target position (geometry_msgs/PoseStamped)
- `/agent_X/metrics` - Performance data (custom MetricsData msg)
- `/wind/velocity` - Wind disturbance

### Drone Groups (varies by launch file)

**it2_vs_gt2_comparison.launch.py** (6 drones):
- Group 0 (agents 0-2): PID + IT2-FLS (Y=-10m lane)
- Group 1 (agents 3-5): PID + GT2-FLS (Y=10m lane)

**formation_comparison_demo.launch.py** (9 drones):
- Group 0 (agents 0-2): PID + IT2-Fuzzy
- Group 1 (agents 3-5): PD controller
- Group 2 (agents 6-8): PID controller

**twelve_drone_comparison.launch.py** (12 drones):
- Group 0 (agents 0-3): PID + IT2-FLS
- Group 1 (agents 4-7): PID + GT2-FLS
- Group 2 (agents 8-11): Pure PID

## Package Structure

- `agent_control_pkg/` - Main ROS2 package with controllers, plugins, launch files, and models
- `other_packages/formation_coordinator_pkg/` - Formation coordination with PSO
- `other_packages/my_custom_interfaces_pkg/` - Custom ROS2 message/service definitions
- `monitoring_dashboard/` - FastAPI backend + React/TypeScript frontend

## Code Style

- C++17 for ROS2 packages
- Python: Black formatter (88 char), isort, flake8
- Configuration in `pyproject.toml` and `.flake8`
