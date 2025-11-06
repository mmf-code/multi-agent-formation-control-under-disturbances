#!/bin/bash

# Quick test to check drone spawn positions
pkill -9 gzserver gzclient rviz2 2>/dev/null
sleep 2

cd /home/mmf/Documents/GitHub/multi-agent-formation-control-under-disturbances
source install/setup.bash

echo "Starting headless simulation..."
ros2 launch agent_control_pkg formation_step_response_demo.launch.py gazebo_gui:=false rviz:=false > /tmp/launch.log 2>&1 &
LAUNCH_PID=$!

echo "Waiting 12s for simulation to initialize..."
sleep 12

echo ""
echo "=== DRONE POSITIONS AT T=12s ==="
echo ""

for i in 0 3 6; do
    echo "Agent $i:"
    timeout 2 ros2 topic echo /agent_$i/odom --once 2>/dev/null | grep -A 3 "position:" || echo "  No data"
    echo ""
done

echo "Cleaning up..."
kill $LAUNCH_PID 2>/dev/null
sleep 1
pkill -9 gzserver gzclient 2>/dev/null
sleep 1

echo "Done!"
