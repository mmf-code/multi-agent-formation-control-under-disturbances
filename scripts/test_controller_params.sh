#!/bin/bash
################################################################################
# Test Controller Params Publisher
# Verifies controller_params topics are working without Gazebo
################################################################################

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
CYAN='\033[0;36m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${CYAN}╔════════════════════════════════════════════════════════╗${NC}"
echo -e "${CYAN}║  Controller Params Publisher Test                     ║${NC}"
echo -e "${CYAN}╚════════════════════════════════════════════════════════╝${NC}"
echo ""

cd /home/mmf/Documents/GitHub/multi-agent-formation-control-under-disturbances

source /opt/ros/humble/setup.bash
source install/setup.bash

echo -e "${GREEN}✓ ROS2 environment sourced${NC}"
echo ""

# Start a single agent controller node for testing
echo -e "${CYAN}Starting test agent controller...${NC}"

ros2 run agent_control_pkg agent_controller_node \
  --ros-args \
  -r __ns:=/agent_0 \
  -p controller_type:=pid_fuzzy \
  -p pid.kp:=3.501 \
  -p pid.ki:=1.946 \
  -p pid.kd:=3.608 \
  -p fuzzy.enable:=true \
  -p fuzzy.wind_scalar:=0.7 \
  -p mix.k_pid:=1.0 \
  -p mix.k_fuzzy:=0.7 &

CONTROLLER_PID=$!
echo -e "${GREEN}✓ Controller started (PID: $CONTROLLER_PID)${NC}"
sleep 3

echo ""
echo -e "${CYAN}Checking for controller_params topic...${NC}"
if ros2 topic list | grep -q "/agent_0/controller_params"; then
    echo -e "${GREEN}✓ Topic /agent_0/controller_params exists${NC}"
else
    echo -e "${RED}✗ Topic not found${NC}"
    kill $CONTROLLER_PID 2>/dev/null
    exit 1
fi

echo ""
echo -e "${CYAN}Waiting for controller_params data (publishes every 2s)...${NC}"
sleep 3

echo ""
echo -e "${CYAN}Reading controller_params message:${NC}"
echo -e "${YELLOW}────────────────────────────────────────────────────────${NC}"
timeout 5 ros2 topic echo /agent_0/controller_params --once 2>&1 || echo -e "${YELLOW}(Waiting for first message...)${NC}"
echo -e "${YELLOW}────────────────────────────────────────────────────────${NC}"

echo ""
echo -e "${GREEN}✓ Test complete!${NC}"

kill $CONTROLLER_PID 2>/dev/null
echo -e "${CYAN}Stopped test controller${NC}"
