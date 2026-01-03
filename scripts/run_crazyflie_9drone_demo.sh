#!/bin/bash
#
# Crazyflie 9-Drone Formation Demo
# =================================
# Runs the complete 9-drone formation control system with Crazyswarm2.
#
# WARNING: This script requires ros-humble-crazyflie package (Crazyswarm2).
#          This package is NOT installed by default.
#          For Gazebo simulation without Crazyswarm2, use:
#            ros2 launch agent_control_pkg formation_comparison_demo.launch.py
#            ros2 launch agent_control_pkg it2_vs_gt2_comparison.launch.py
#
# This script:
# 1. Launches Crazyswarm2 simulation (9 Crazyflies)
# 2. Launches TF->Odom bridge
# 3. Launches 9 agent controllers (3 groups: PID+Fuzzy, PD, PID)
# 4. Launches 3 formation coordinators
#
# Usage:
#   ./scripts/run_crazyflie_9drone_demo.sh           # Full demo
#   ./scripts/run_crazyflie_9drone_demo.sh --sim     # Simulation only
#   ./scripts/run_crazyflie_9drone_demo.sh --real    # Real hardware
#
# Prerequisites:
#   - ROS2 Humble
#   - ros-humble-crazyflie package (sudo apt install ros-humble-crazyflie)
#   - colcon build --cmake-args -DENABLE_CRAZYFLIE=ON

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"
PKG_DIR="$WORKSPACE_DIR/agent_control_pkg"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Default settings
USE_SIM=true
DURATION=60
GUI=false

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --sim)
            USE_SIM=true
            shift
            ;;
        --real)
            USE_SIM=false
            shift
            ;;
        --gui)
            GUI=true
            shift
            ;;
        --duration)
            DURATION="$2"
            shift 2
            ;;
        -h|--help)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --sim         Use Crazyswarm2 simulation (default)"
            echo "  --real        Use real Crazyflie hardware"
            echo "  --gui         Enable RViz visualization"
            echo "  --duration N  Run for N seconds (default: 60)"
            echo "  -h, --help    Show this help message"
            exit 0
            ;;
        *)
            echo -e "${RED}Unknown option: $1${NC}"
            exit 1
            ;;
    esac
done

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  Crazyflie 9-Drone Formation Demo${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""
echo -e "Mode: ${GREEN}$([ "$USE_SIM" = true ] && echo 'SIMULATION' || echo 'REAL HARDWARE')${NC}"
echo -e "Duration: ${GREEN}${DURATION}s${NC}"
echo -e "GUI: ${GREEN}$([ "$GUI" = true ] && echo 'Enabled' || echo 'Disabled')${NC}"
echo ""

# Source ROS2
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

# Source workspace
if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    source "$WORKSPACE_DIR/install/setup.bash"
else
    echo -e "${YELLOW}Warning: Workspace not built. Running colcon build...${NC}"
    cd "$WORKSPACE_DIR"
    colcon build --symlink-install --cmake-args -DENABLE_CRAZYFLIE=ON
    source install/setup.bash
fi

# Check if crazyflie package is available
if ! ros2 pkg list | grep -q "^crazyflie$"; then
    echo -e "${RED}Error: crazyflie package not found!${NC}"
    echo -e "Install with: ${YELLOW}sudo apt install ros-humble-crazyflie${NC}"
    exit 1
fi

# Cleanup function
cleanup() {
    echo ""
    echo -e "${YELLOW}Shutting down...${NC}"
    pkill -f "crazyflie" 2>/dev/null || true
    pkill -f "agent_controller_node" 2>/dev/null || true
    pkill -f "tf_to_odom_bridge" 2>/dev/null || true
    pkill -f "formation_coordinator" 2>/dev/null || true
    pkill -f "rviz2" 2>/dev/null || true
    echo -e "${GREEN}Cleanup complete.${NC}"
}
trap cleanup EXIT

# Copy our crazyflies config to override default
CRAZYFLIES_CONFIG="$PKG_DIR/config/crazyflies_9drone.yaml"
if [ -f "$CRAZYFLIES_CONFIG" ]; then
    echo -e "${BLUE}Using custom crazyflies config: ${CRAZYFLIES_CONFIG}${NC}"
fi

echo ""
echo -e "${BLUE}Step 1: Starting Crazyswarm2...${NC}"

if [ "$USE_SIM" = true ]; then
    # Simulation mode - launch Crazyswarm2 sim backend
    ros2 launch crazyflie launch.py \
        backend:=sim \
        gui:=false \
        rviz:=false \
        crazyflies_yaml_file:="$CRAZYFLIES_CONFIG" &
    CS2_PID=$!
    sleep 5  # Wait for simulation to start
else
    # Real hardware mode
    echo -e "${YELLOW}Real hardware mode - ensure Crazyradio is connected${NC}"
    ros2 launch crazyflie launch.py \
        backend:=cflib \
        gui:=false \
        rviz:=false \
        crazyflies_yaml_file:="$CRAZYFLIES_CONFIG" &
    CS2_PID=$!
    sleep 8  # More time for real hardware connection
fi

echo -e "${GREEN}Crazyswarm2 started (PID: $CS2_PID)${NC}"

echo ""
echo -e "${BLUE}Step 2: Starting Formation Controllers...${NC}"

# Launch our 9-drone formation system
# Crazyswarm2 sim publishes /clock, so use_sim_time:=true for simulation
# For real hardware, use use_sim_time:=false
if [ "$USE_SIM" = true ]; then
    SIM_TIME_ARG="true"
else
    SIM_TIME_ARG="false"
fi

ros2 launch agent_control_pkg crazyflie_9drone_formation.launch.py \
    use_sim_time:=$SIM_TIME_ARG \
    launch_bridge:=true &
CTRL_PID=$!
sleep 3

echo -e "${GREEN}Controllers started (PID: $CTRL_PID)${NC}"

# Optional RViz
if [ "$GUI" = true ]; then
    echo ""
    echo -e "${BLUE}Step 3: Starting RViz...${NC}"
    rviz2 &
    RVIZ_PID=$!
    echo -e "${GREEN}RViz started (PID: $RVIZ_PID)${NC}"
fi

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  Demo Running!${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo -e "Drone Groups:"
echo -e "  ${BLUE}Group 0 (PID+IT2-FLS):${NC} cf231, cf232, cf233"
echo -e "  ${YELLOW}Group 1 (PD):${NC}          cf234, cf235, cf236"
echo -e "  ${GREEN}Group 2 (PID):${NC}         cf237, cf238, cf239"
echo ""
echo -e "Topics to monitor:"
echo -e "  ros2 topic echo /cf231/odom"
echo -e "  ros2 topic echo /agent_0/target_pose"
echo -e "  ros2 topic echo /cf231/cmd_full_state"
echo ""
echo -e "Press ${RED}Ctrl+C${NC} to stop"
echo ""

# Run for specified duration
sleep $DURATION

echo ""
echo -e "${GREEN}Demo completed after ${DURATION}s${NC}"
