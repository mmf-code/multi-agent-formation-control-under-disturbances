#!/bin/bash
###############################################################################
# Professional Formation Control Demonstration - Quick Launch Script
#
# This script provides a convenient way to launch the ROS2 + Gazebo simulation
# with proper environment setup and error checking.
#
# Usage:
#   ./scripts/run_demo.sh              # Full demo (Gazebo + RViz2)
#   ./scripts/run_demo.sh --headless   # No Gazebo GUI
#   ./scripts/run_demo.sh --no-rviz    # No RViz2
#
# Author: Multi-Agent Formation Control Team
# Date: 2025-10-17
###############################################################################

set -e  # Exit on error

# Colors for terminal output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Parse command line arguments
GAZEBO_GUI=true
RVIZ=true
CONTROLLER_TYPE="pid"

while [[ $# -gt 0 ]]; do
  case $1 in
    --headless)
      GAZEBO_GUI=false
      shift
      ;;
    --no-rviz)
      RVIZ=false
      shift
      ;;
    --controller)
      CONTROLLER_TYPE="$2"
      shift 2
      ;;
    --help)
      echo "Usage: $0 [OPTIONS]"
      echo ""
      echo "Options:"
      echo "  --headless          Run Gazebo without GUI"
      echo "  --no-rviz           Don't launch RViz2"
      echo "  --controller TYPE   Controller type (pid, fuzzy, pid_fuzzy, p, pi, pd)"
      echo "  --help              Show this help message"
      exit 0
      ;;
    *)
      echo -e "${RED}Unknown option: $1${NC}"
      exit 1
      ;;
  esac
done

# Print banner
echo -e "${BLUE}"
echo "╔══════════════════════════════════════════════════════════════╗"
echo "║                                                              ║"
echo "║   Multi-Agent Formation Control - Professional Demo         ║"
echo "║   ROS2 Humble + Gazebo Simulation                           ║"
echo "║                                                              ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo -e "${NC}"

# Check if ROS2 is sourced
if [ -z "$ROS_DISTRO" ]; then
  echo -e "${YELLOW}ROS2 not sourced. Sourcing /opt/ros/humble/setup.bash...${NC}"
  source /opt/ros/humble/setup.bash
fi

# Check if workspace is built
WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [ ! -d "$WORKSPACE_ROOT/install" ]; then
  echo -e "${RED}Error: Workspace not built!${NC}"
  echo -e "${YELLOW}Please run: colcon build${NC}"
  exit 1
fi

# Source workspace
echo -e "${GREEN}Sourcing workspace: $WORKSPACE_ROOT${NC}"
source "$WORKSPACE_ROOT/install/setup.bash"

# Check if required packages are available
REQUIRED_PACKAGES=("agent_control_pkg" "formation_coordinator_pkg" "my_custom_interfaces_pkg")
for pkg in "${REQUIRED_PACKAGES[@]}"; do
  if ! ros2 pkg prefix "$pkg" > /dev/null 2>&1; then
    echo -e "${RED}Error: Package '$pkg' not found!${NC}"
    echo -e "${YELLOW}Please build the workspace first.${NC}"
    exit 1
  fi
done

# Kill any existing Gazebo processes
echo -e "${YELLOW}Checking for existing Gazebo processes...${NC}"
if pgrep -x "gzserver" > /dev/null; then
  echo -e "${YELLOW}Killing existing gzserver...${NC}"
  pkill -9 gzserver || true
fi
if pgrep -x "gzclient" > /dev/null; then
  echo -e "${YELLOW}Killing existing gzclient...${NC}"
  pkill -9 gzclient || true
fi
sleep 1

# Display configuration
echo -e "${GREEN}Configuration:${NC}"
echo "  • Controller Type: $CONTROLLER_TYPE"
echo "  • Gazebo GUI: $GAZEBO_GUI"
echo "  • RViz2: $RVIZ"
echo ""

# Launch the demo
echo -e "${GREEN}Launching formation control demo...${NC}"
echo -e "${YELLOW}Press Ctrl+C to stop${NC}"
echo ""

ros2 launch agent_control_pkg demo_presentation.launch.py \
  gazebo_gui:=$GAZEBO_GUI \
  rviz:=$RVIZ \
  controller_config:=$CONTROLLER_TYPE

# Cleanup on exit
echo -e "${YELLOW}Cleaning up...${NC}"
pkill -9 gzserver || true
pkill -9 gzclient || true
echo -e "${GREEN}Demo stopped.${NC}"
