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

## Gazebo Integration (2025-01)

### Architecture Overview
The Gazebo integration implements the 3-layer control architecture:

```
┌─────────────────────────────────────────────────┐
│ Formation Coordinator Node                      │
│ - Publishes /agent_X/target_pose                │
│ - Service: /set_formation (UpdateFormation)     │
└──────────────────┬──────────────────────────────┘
                   │ geometry_msgs/PoseStamped
┌──────────────────▼──────────────────────────────┐
│ Agent Controller Node (agent_controller_node)   │
│ - Subscribes: /agent_X/odom, /agent_X/target    │
│ - Publishes: /agent_X/cmd_accel                 │
│ - Controller: PID/Fuzzy/Hybrid from YAML        │
│ - Frequency: 200 Hz control loop                │
└──────────────────┬──────────────────────────────┘
                   │ geometry_msgs/Vector3 (m/s²)
┌──────────────────▼──────────────────────────────┐
│ Gazebo + SimpleDronePlugin                      │
│ - Physics: F = m * a_cmd (X-Y plane)            │
│ - Z-axis: Gravity comp + altitude hold (0.5m)   │
│ - Publishes: /agent_X/odom (nav_msgs/Odometry)  │
│ - Update rate: ~1 kHz (Gazebo physics step)     │
└──────────────────────────────────────────────────┘
```

### Implementation Details

#### SimpleDronePlugin (`agent_control_pkg/plugins/simple_drone_plugin.cpp`)
- **Purpose**: Bridges ROS2 control commands to Gazebo physics
- **Force Application**: Direct force injection `F = m * a_cmd` for X-Y motion
- **Z-Axis Control**: Automatic altitude hold at 0.5m using proportional control (Kp=10)
  - Gravity compensation: `F_z = m * g` (9.81 m/s²)
  - Position error feedback: `F_z += m * (z_target - z_current) * 10.0`
- **ROS2 Topics**:
  - Subscribes: `/agent_X/cmd_accel` (geometry_msgs/Vector3)
  - Publishes: `/agent_X/odom` (nav_msgs/Odometry) at Gazebo physics rate
- **Configuration**: Plugin namespace set in world file via `<namespace>agent_0</namespace>`

#### World File (`agent_control_pkg/worlds/minimal_test.world`)
- **Drone Model**: 0.4×0.4×0.1m box, mass 1.5 kg
- **Initial Pose**: `(0, 0, 0)` - spawns at ground level, rises to z=0.5m automatically
- **Physics**: ODE solver, 1000 Hz update rate, real-time factor ~1.0
- **Plugin Loading**: SimpleDronePlugin attached to base_link with namespace configuration

#### Launch File (`agent_control_pkg/launch/gazebo_single_agent.launch.py`)
- **Components**:
  1. `gzserver` - Physics simulation engine (headless)
  2. `gzclient` - 3D visualization GUI (optional with `gui:=false`)
  3. `agent_controller_node` - PID/Fuzzy controller
  4. `formation_coordinator_node` - Target pose publisher
- **Launch Arguments**:
  - `use_sim_time:=true` (default) - Sync all nodes with Gazebo clock
  - `gui:=false` - Disable visualization for faster simulation
  - `verbose:=true` - Enable Gazebo debug output

#### CSV Logging (`scripts/log_simulation_csv.py`)
- **Purpose**: Record simulation data for post-analysis and comparison with standalone C++ sims
- **Data Captured**:
  - State: `pos_xyz`, `vel_xyz` (from `/agent_X/odom`)
  - Control: `cmd_ax`, `cmd_ay`, `cmd_az` (from `/agent_X/cmd_accel`)
  - Reference: `target_x`, `target_y`, `target_z` (from `/agent_X/target_pose`)
- **Usage**:
  ```bash
  python3 scripts/log_simulation_csv.py \
    --namespace agent_0 \
    --output outputs/logs/gazebo_run_001.csv
  ```
- **Analysis**: Compatible with existing `analysis/plot_dynamics_2d.py` tooling

### Configuration Files

#### Controller Config (`agent_control_pkg/config/ros2/agent_controller_default.yaml`)
```yaml
agent_0:
  agent_controller:
    ros__parameters:
      controller_type: pid  # pid, fuzzy, pid_fuzzy, p, pi, pd
      dt: 0.005
      control_frequency_hz: 200.0

      # Tuned PID gains (~10% overshoot, 4s settling)
      pid.kp: 0.538
      pid.ki: 0.145
      pid.kd: 1.368
      pid.enable_derivative_filter: true
      pid.derivative_filter_alpha: 0.1

      # Fuzzy controller (disabled for pure PID)
      fuzzy.enable: false

      # Output limits (m/s²)
      output_limits.x.min: -10.0
      output_limits.x.max: 10.0
      output_limits.y.min: -8.0
      output_limits.y.max: 12.0
```

**Note**: After modifying YAML, rebuild is required:
```bash
colcon build --packages-select agent_control_pkg
source install/setup.bash
```

## Launch & Verification Commands

### Gazebo Simulation (Recommended)
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

# Launch complete Gazebo simulation with GUI
ros2 launch agent_control_pkg gazebo_single_agent.launch.py

# Headless mode (no GUI, faster)
ros2 launch agent_control_pkg gazebo_single_agent.launch.py gui:=false

# In separate terminal: log data to CSV
python3 scripts/log_simulation_csv.py \
  --namespace agent_0 \
  --output outputs/logs/gazebo_test.csv
```

### Standalone ROS2 Nodes (No Gazebo)
```bash
# Single agent harness (namespaced controller, coordinator keeps target fixed)
ros2 launch agent_control_pkg single_agent_test.launch.py

# Three-agent formation (triangle offsets, 10 Hz updates)
ros2 launch agent_control_pkg multi_agent_formation.launch.py
```

### Monitoring & Debugging
```bash
# List all topics
ros2 topic list | grep agent_0

# Monitor real-time data
ros2 topic echo /agent_0/target_pose  # Formation coordinator output
ros2 topic echo /agent_0/odom         # Gazebo state feedback
ros2 topic echo /agent_0/cmd_accel    # Controller output
ros2 topic echo /agent_0/diagnostics  # PID/Fuzzy contributions

# Check topic frequencies
ros2 topic hz /agent_0/odom           # Should be ~1000 Hz (Gazebo physics)
ros2 topic hz /agent_0/cmd_accel      # Should be ~200 Hz (controller)

# Verify PID parameters loaded correctly
ros2 param get /agent_0/agent_controller pid.kp  # Expected: 0.538
ros2 param get /agent_0/agent_controller pid.ki  # Expected: 0.145
ros2 param get /agent_0/agent_controller pid.kd  # Expected: 1.368

# Change formation (for multi-agent)
ros2 service call /set_formation my_custom_interfaces_pkg/srv/UpdateFormation \
  "{shape: 'line', spacing: 5.0, center_x: 0.0, center_y: 0.0, \
    center_z: 0.0, yaw_deg: 0.0, agent_ids: []}"
```

### Troubleshooting

**Problem: Drone oscillates around target**
- **Cause**: PID gains not loading from YAML (using defaults Kp=0.6, Ki=0, Kd=0)
- **Solution**:
  1. Verify YAML structure matches node namespace
  2. Rebuild: `colcon build --packages-select agent_control_pkg`
  3. Check loaded values: `ros2 param get /agent_0/agent_controller pid.kp`

**Problem: Gazebo freezes or crashes**
- **Solution**: Kill all processes and restart
  ```bash
  pkill -9 gzserver gzclient
  pkill -9 -f agent_controller_node
  ros2 launch agent_control_pkg gazebo_single_agent.launch.py
  ```

**Problem: No movement in Gazebo**
- **Checks**:
  1. Verify cmd_accel is being published: `ros2 topic hz /agent_0/cmd_accel`
  2. Check plugin loaded: `ros2 topic list | grep odom`
  3. Ensure target is different from start position

**Problem: Config changes not taking effect**
- **Solution**: Config files are installed during build. After editing YAML:
  ```bash
  colcon build --packages-select agent_control_pkg
  source install/setup.bash
  # Now relaunch
  ```
