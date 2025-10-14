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
