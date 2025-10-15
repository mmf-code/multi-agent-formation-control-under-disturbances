# ROS2 Integration Plan

This document outlines how to integrate the agent-level controller (PID/PD/PID+GT2 Fuzzy, prefilter, FF) into ROS 2 simulation and real robots.

## Node Architecture
- agent_controller_node (C++)
  - Subscriptions:
    - target_pose (geometry_msgs/PoseStamped) or target_point (x,y)
    - Optional wind_state
  - Publications:
    - attitude_setpoint (roll, pitch, yaw) and thrust
    - Diagnostics (u_pid, u_fuzzy, ax_ff, metrics)
  - Parameters (ROS2 YAML): mirrors current YAML (pid gains, fuzzy params file, prefilter, feedforward, physics)
- formation_coordinator_node (C++)
  - Generates waypoints for each agent; already stubbed in other_packages/formation_coordinator_pkg

## Control Mapping (2D accel -> Attitude/Thrust)
- roll_cmd ~= -ay_des / g
- pitch_cmd ~=  ax_des / g
- yaw fixed or tracked separately
- thrust_cmd = m * (g + az_des)
- Limiters: |roll|, |pitch| <= 20..30 deg; rate limits and slew-rate

## Integration Steps
1. Extract controlloop core into a reusable library (already modular): PID, Fuzzy, prefilter, FF.
2. Create agent_controller_node:
   - Read params via rclcpp::Node and YAML
   - Timer at dt, run: prefilter -> controller -> FF -> mapping -> publish
3. Bridge to SITL:
   - Use MAVROS or PX4 ROS2 bridge to feed setpoints to Gazebo/Ignition SITL
   - Validate with time-varying wind worlds
4. Formation:
   - Hook coordinator outputs to per-agent targets
   - Log formation error metrics (centroid/baseline deviations)

## Topics and Params (example)
- /agent_i/target_point (Float64MultiArray: [x,y])
- /agent_i/attitude_setpoint (roll,pitch,yaw,thrust)
- /agent_i/controller/params (parameter events)

## Testing Matrix
- Scenarios: no-wind, bias, wind (sinusoid/step/gust), wind+bias
- Controllers: PD, PID(2-DOF+prefilter+FF), PID+Fuzzy
- Metrics: OS, Ts5, RMSE, IAE; formation error for multi-agent

## Baseline Implementation (2025-10)
- `agent_control_pkg/src/ros/agent_controller_node.cpp:1` hosts the 200 Hz control loop; subscribes to `~/target_pose` (`geometry_msgs/PoseStamped`) and `~/odom` (`nav_msgs/Odometry`), publishes `~/cmd_accel` (`geometry_msgs/Vector3`) plus `~/diagnostics`. Controllers are created from ROS parameters (PID, P/PI/PD, GT2 fuzzy, or hybrid) using adapters under `agent_control_pkg/src/controllers/`.
- Parameters mirror legacy YAML via dot notation (`pid.kp`, `output_limits.x.min`, etc.) in `agent_control_pkg/config/ros2/agent_controller_default.yaml:1`. **Note**: each agent is namespaced (`agent_0/agent_controller/ros__parameters`), so parameters apply automatically when the launch file sets `namespace='agent_0'`.
- `formation_coordinator_pkg/src/formation_coordinator_node.cpp:1` publishes per-agent targets at configurable rates and exposes `/set_formation` (`my_custom_interfaces_pkg/srv/UpdateFormation`) to change shape, spacing, center, yaw, and agent list on the fly. Formation state is broadcast via `my_custom_interfaces_pkg/msg/FormationState`.
- Launchers:
  - `agent_control_pkg/launch/single_agent_test.launch.py:1` → runs one controller under `agent_0` namespace with the coordinator set to a single target.
  - `agent_control_pkg/launch/multi_agent_formation.launch.py:1` → spawns three controllers (`agent_0..2`) plus the coordinator, sourcing defaults from `formation_coordinator_pkg/config/formation_config.yaml:1`.
- Custom interfaces are generated in `my_custom_interfaces_pkg` (CMake+package exporting `FormationState` msg & `UpdateFormation` srv). Build order: `colcon build --packages-select my_custom_interfaces_pkg formation_coordinator_pkg agent_control_pkg`.
- Diagnostics:
  - Controller node exposes last PID/fuzzy contributions via `std_msgs/Float64MultiArray` for quick plotting.
  - Coordinator publishes shape metadata for logging/rviz overlays.

## Launch & Verification Commands
```
source /opt/ros/humble/setup.bash
colcon build --packages-select my_custom_interfaces_pkg formation_coordinator_pkg agent_control_pkg
source install/setup.bash

# 2D Gazebo run (single drone → target at 5,5,0)
./start_gazebo_sim.sh

# Single agent harness (namespaced controller, coordinator keeps target fixed)
ros2 launch agent_control_pkg single_agent_test.launch.py

# Three-agent formation (triangle offsets, 10 Hz updates)
ros2 launch agent_control_pkg multi_agent_formation.launch.py

# Inspect topics
ros2 topic list
ros2 topic echo /agent_0/cmd_accel
ros2 service call /set_formation my_custom_interfaces_pkg/srv/UpdateFormation "{shape: 'line', spacing: 5.0, center_x: 0.0, center_y: 0.0, center_z: 0.0, yaw_deg: 0.0, agent_ids: []}"
```

Resetting a Gazebo run without closing the GUI:
```
ros2 service call /reset_simulation std_srvs/srv/Empty {}
```
