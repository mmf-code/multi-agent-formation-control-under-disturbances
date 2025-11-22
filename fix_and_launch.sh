#!/bin/bash
set -e

# 1. Configure Environment to disable SHM and use FastRTPS
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export RMW_FASTRTPS_USE_SHM=0

# 2. Source ROS 2
source /opt/ros/humble/setup.bash
if [ -f install/setup.bash ]; then
    source install/setup.bash
fi

# 3. Reset ROS 2 Daemon to clear stale state
echo "Resetting ROS 2 Daemon..."
ros2 daemon stop
ros2 daemon start

# 4. Launch Simulation (Full GUI for visualization)
echo "Launching simulation with GUI..."
ros2 launch agent_control_pkg formation_comparison_demo.launch.py gazebo_gui:=true rviz:=true
