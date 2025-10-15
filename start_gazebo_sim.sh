#!/bin/bash
# Gazebo Formation Control Simulation Launcher
# Usage: ./start_gazebo_sim.sh

set -e

echo "🚀 Starting Gazebo Formation Control Simulation..."
echo ""

# Navigate to project directory
cd "$(dirname "$0")"

# Source ROS2 and workspace
echo "📦 Sourcing ROS2 environment..."
source /opt/ros/humble/setup.bash
source install/setup.bash

# Set Gazebo paths
echo "🔧 Setting up Gazebo paths..."
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(pwd)/install/agent_control_pkg/share/agent_control_pkg/models
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:$(pwd)/install/agent_control_pkg/lib

echo ""
echo "✅ Environment ready!"
echo ""
echo "🎬 Launching Gazebo simulation..."
echo "   - This may take 30-60 seconds to load"
echo "   - Gazebo GUI will open in a new window"
echo "   - Look for the blue box (drone) and green cylinder (target)"
echo ""
echo "⏸️  Press Ctrl+C to stop the simulation"
echo ""

# Launch
ros2 launch agent_control_pkg gazebo_single_agent.launch.py
