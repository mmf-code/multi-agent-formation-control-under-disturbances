#!/bin/bash
#
# Crazyflie Formation Demo
# Runs Crazyswarm2 sim + TF bridge + our controller
#
# WARNING: This script requires ros-humble-crazyflie package (Crazyswarm2).
#          Install with: sudo apt install ros-humble-crazyflie
#          For Gazebo simulation without Crazyswarm2, use:
#            ros2 launch agent_control_pkg formation_comparison_demo.launch.py
#
# Usage:
#   ./scripts/run_crazyflie_demo.sh           # 1 drone (default)
#   ./scripts/run_crazyflie_demo.sh 3         # 3 drones
#   ./scripts/run_crazyflie_demo.sh 1 --gui   # with Crazyswarm2 GUI
#

set -e

# Check if Crazyswarm2 is installed
if ! ros2 pkg list 2>/dev/null | grep -q "crazyflie"; then
    echo "ERROR: ros-humble-crazyflie (Crazyswarm2) is not installed!"
    echo ""
    echo "Options:"
    echo "  1. Install Crazyswarm2: sudo apt install ros-humble-crazyflie"
    echo "  2. Use Gazebo simulation instead:"
    echo "     ros2 launch agent_control_pkg formation_comparison_demo.launch.py"
    echo ""
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"

NUM_AGENTS="${1:-1}"
GUI_FLAG="${2:-}"

echo "=========================================="
echo "  Crazyflie Formation Demo"
echo "  Agents: $NUM_AGENTS"
echo "=========================================="

# Source ROS2
source /opt/ros/humble/setup.bash
source "$WS_DIR/install/setup.bash"

# Cleanup function
cleanup() {
    echo ""
    echo "Shutting down..."
    pkill -f "crazyflie_server" 2>/dev/null || true
    pkill -f "tf_to_odom_bridge" 2>/dev/null || true
    pkill -f "agent_controller_node" 2>/dev/null || true
    pkill -f "teleop" 2>/dev/null || true
    sleep 1
    echo "Done."
}
trap cleanup EXIT

# Determine GUI setting
if [[ "$GUI_FLAG" == "--gui" ]]; then
    CF_GUI="True"
    CF_RVIZ="True"
else
    CF_GUI="False"
    CF_RVIZ="False"
fi

echo "[1/4] Starting Crazyswarm2 simulator (backend:=sim)..."
ros2 launch crazyflie launch.py backend:=sim gui:=$CF_GUI rviz:=$CF_RVIZ &
CF_PID=$!
sleep 3

echo "[2/4] Starting TF to Odom bridge..."
python3 "$SCRIPT_DIR/tf_to_odom_bridge.py" &
BRIDGE_PID=$!
sleep 1

echo "[3/4] Starting formation controller (num_agents:=$NUM_AGENTS)..."
ros2 launch agent_control_pkg crazyflie_formation.launch.py num_agents:=$NUM_AGENTS &
CTRL_PID=$!
sleep 2

echo "[4/4] Sending initial target pose..."
# Send target to move drone to (1, 0, 0)
ros2 topic pub /cf231/target_pose geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: 'world'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}}}" -r 10 &
TARGET_PID=$!

echo ""
echo "=========================================="
echo "  Demo Running!"
echo "=========================================="
echo ""
echo "Topics:"
echo "  /cf231/odom          - Drone position (from TF bridge)"
echo "  /cf231/target_pose   - Target position"
echo "  /cf231/cmd_full_state - Control output"
echo ""
echo "Commands:"
echo "  # Change target position:"
echo "  ros2 topic pub /cf231/target_pose geometry_msgs/msg/PoseStamped \\"
echo "      \"{pose: {position: {x: 2.0, y: 1.0, z: 0.5}}}\" -1"
echo ""
echo "Press Ctrl+C to stop..."
echo ""

# Wait for any process to exit
wait $CF_PID $BRIDGE_PID $CTRL_PID
