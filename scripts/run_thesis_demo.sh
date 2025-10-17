#!/bin/bash
###############################################################################
# Thesis Multi-Controller Comparison Demo - Quick Launch Script
#
# This script provides a convenient way to launch the comprehensive thesis
# demonstration with 5 drones running different controllers simultaneously.
#
# Usage:
#   ./scripts/run_thesis_demo.sh                # Full demo (GUI + RViz)
#   ./scripts/run_thesis_demo.sh --headless     # No Gazebo GUI (higher FPS)
#   ./scripts/run_thesis_demo.sh --no-rviz      # No RViz2 (maximum FPS)
#   ./scripts/run_thesis_demo.sh --performance  # Headless + No RViz (best FPS)
#   ./scripts/run_thesis_demo.sh --triangle     # Only 3 drones (triangle formation)
#
# Controllers:
#   Agent 0 (Red):     P-only
#   Agent 1 (Green):   PI
#   Agent 2 (Blue):    PD
#   Agent 3 (Yellow):  PID
#   Agent 4 (Magenta): PID+Fuzzy
#
# Author: Multi-Agent Formation Control Research Team
# Date: 2025-10-17
###############################################################################

set -e  # Exit on error

# Colors for terminal output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
MAGENTA='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Parse command line arguments
GAZEBO_GUI=true
RVIZ=true
NUM_AGENTS=5

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
    --performance)
      GAZEBO_GUI=false
      RVIZ=false
      shift
      ;;
    --triangle)
      NUM_AGENTS=3
      shift
      ;;
    --help)
      echo "Usage: $0 [OPTIONS]"
      echo ""
      echo "Options:"
      echo "  --headless          Run Gazebo without GUI (higher FPS)"
      echo "  --no-rviz           Don't launch RViz2 (save resources)"
      echo "  --performance       Headless + No RViz (maximum FPS)"
      echo "  --triangle          Only spawn 3 drones (triangle formation)"
      echo "  --help              Show this help message"
      echo ""
      echo "Controllers:"
      echo "  Agent 0 (Red):     P-only controller"
      echo "  Agent 1 (Green):   PI controller"
      echo "  Agent 2 (Blue):    PD controller"
      echo "  Agent 3 (Yellow):  PID controller"
      echo "  Agent 4 (Magenta): PID+Fuzzy controller"
      exit 0
      ;;
    *)
      echo -e "${RED}Unknown option: $1${NC}"
      exit 1
      ;;
  esac
done

# Print banner
echo -e "${CYAN}"
echo "╔════════════════════════════════════════════════════════════════════╗"
echo "║                                                                    ║"
echo "║   🎓 Thesis Multi-Controller Comparison Demonstration             ║"
echo "║   ROS2 + Gazebo + 5 Controllers + Triangle Formation              ║"
echo "║                                                                    ║"
echo "╚════════════════════════════════════════════════════════════════════╝"
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
echo -e "${CYAN}╔════════════════════════════════════════════════════════════════╗${NC}"
echo -e "${CYAN}║${NC} ${GREEN}Configuration:${NC}                                                ${CYAN}║${NC}"
echo -e "${CYAN}╠════════════════════════════════════════════════════════════════╣${NC}"
echo -e "${CYAN}║${NC}  Number of Agents: ${YELLOW}$NUM_AGENTS${NC}                                        ${CYAN}║${NC}"
echo -e "${CYAN}║${NC}  Gazebo GUI:       ${YELLOW}$GAZEBO_GUI${NC}                                       ${CYAN}║${NC}"
echo -e "${CYAN}║${NC}  RViz2:            ${YELLOW}$RVIZ${NC}                                        ${CYAN}║${NC}"
echo -e "${CYAN}╠════════════════════════════════════════════════════════════════╣${NC}"
echo -e "${CYAN}║${NC} ${GREEN}Controllers:${NC}                                                   ${CYAN}║${NC}"
echo -e "${CYAN}╠════════════════════════════════════════════════════════════════╣${NC}"
echo -e "${CYAN}║${NC}  ${RED}Agent 0 (Red):${NC}     P-only controller                          ${CYAN}║${NC}"
echo -e "${CYAN}║${NC}  ${GREEN}Agent 1 (Green):${NC}   PI controller                             ${CYAN}║${NC}"
echo -e "${CYAN}║${NC}  ${BLUE}Agent 2 (Blue):${NC}    PD controller                             ${CYAN}║${NC}"
if [ "$NUM_AGENTS" -ge 4 ]; then
  echo -e "${CYAN}║${NC}  ${YELLOW}Agent 3 (Yellow):${NC}  PID controller                            ${CYAN}║${NC}"
fi
if [ "$NUM_AGENTS" -ge 5 ]; then
  echo -e "${CYAN}║${NC}  ${MAGENTA}Agent 4 (Magenta):${NC} PID+Fuzzy controller                      ${CYAN}║${NC}"
fi
echo -e "${CYAN}╚════════════════════════════════════════════════════════════════╝${NC}"
echo ""

# Launch the thesis demo
echo -e "${GREEN}🚀 Launching multi-controller thesis demonstration...${NC}"
echo -e "${YELLOW}⚠️  Press Ctrl+C to stop${NC}"
echo ""

ros2 launch agent_control_pkg thesis_multi_controller_demo.launch.py \
  gazebo_gui:=$GAZEBO_GUI \
  rviz:=$RVIZ \
  num_agents:=$NUM_AGENTS

# Cleanup on exit
echo -e "${YELLOW}Cleaning up...${NC}"
pkill -9 gzserver || true
pkill -9 gzclient || true
echo -e "${GREEN}Thesis demo stopped.${NC}"
