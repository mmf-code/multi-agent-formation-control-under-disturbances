#!/bin/bash
################################################################################
# Formation Comparison Demo - Visual Presentation
#
# Bu script hocaya göstermek için kullanılır.
# 9 drone, 3 grup halinde üçgen formasyonda rüzgar altında uçar.
#
# Gruplar:
#   - Grup 0 (agent_0,1,2): PID+Fuzzy  - Magenta renk  - En iyi rüzgar dayanımı
#   - Grup 1 (agent_3,4,5): PD         - Cyan renk     - En hızlı settling
#   - Grup 2 (agent_6,7,8): PID        - Yellow renk   - Dengeli performans
#
# Kullanım:
#   ./scripts/run_formation_demo.sh              # Full GUI (Gazebo + RViz)
#   ./scripts/run_formation_demo.sh --headless   # Sadece Gazebo GUI, RViz yok
################################################################################

set -e

# Renk tanımları
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
MAGENTA='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Başlık
echo -e "${MAGENTA}╔════════════════════════════════════════════════════════════╗${NC}"
echo -e "${MAGENTA}║  Multi-Agent Formation Control - Comparison Demo          ║${NC}"
echo -e "${MAGENTA}║  9 Drones | 3 Groups | Wind Disturbance Testing           ║${NC}"
echo -e "${MAGENTA}╚════════════════════════════════════════════════════════════╝${NC}"
echo ""

# Parse arguments
RVIZ_ENABLED="true"
if [[ "$1" == "--headless" ]]; then
    RVIZ_ENABLED="false"
    echo -e "${YELLOW}Mode: Headless (Gazebo GUI only, no RViz)${NC}"
else
    echo -e "${GREEN}Mode: Full GUI (Gazebo + RViz)${NC}"
fi
echo ""

# Workspace kontrolü
WORKSPACE_DIR="/home/mmf/Documents/GitHub/multi-agent-formation-control-under-disturbances"
if [ ! -d "$WORKSPACE_DIR" ]; then
    echo -e "${RED}ERROR: Workspace not found at $WORKSPACE_DIR${NC}"
    exit 1
fi

cd "$WORKSPACE_DIR"

# ROS2 setup
echo -e "${CYAN}[1/3] Setting up ROS2 environment...${NC}"
source /opt/ros/humble/setup.bash
source install/setup.bash

# Controller özeti
echo ""
echo -e "${CYAN}[2/3] Controller Configuration:${NC}"
echo -e "  ${MAGENTA}● Group 0 (PID+Fuzzy):${NC} Kp=3.501, Ki=1.946, Kd=3.608, k_fuzzy=0.7"
echo -e "    └─ Wind-aware fuzzy logic + PID hybrid"
echo -e "    └─ Best disturbance rejection (theoretical)"
echo ""
echo -e "  ${CYAN}● Group 1 (PD):${NC} Kp=3.5, Kd=3.61"
echo -e "    └─ Fast settling time"
echo -e "    └─ Low overshoot (0.13%)"
echo ""
echo -e "  ${YELLOW}● Group 2 (PID):${NC} Kp=3.501, Ki=1.946, Kd=3.608"
echo -e "    └─ Balanced performance"
echo -e "    └─ Integral action for steady-state"
echo ""

# Wind parametreleri
echo -e "${CYAN}Wind Disturbance:${NC}"
echo -e "  ${YELLOW}DISABLED${NC} (for debugging drone spawn issues)"
echo ""

# TEMPORARILY DISABLED: Wind publisher causing drone control issues
# echo -e "${CYAN}[2.5/3] Starting wind publisher node...${NC}"
# bash scripts/run_wind.sh --x 4.0 --y 1.2 --z 0.0 --duration 999999 >/dev/null 2>&1 &
# WIND_PID=$!
# echo -e "  → Wind publisher PID: ${WIND_PID}"
# echo ""

# cleanup() {
#   echo ""
#   echo -e "${YELLOW}Stopping wind publisher (PID ${WIND_PID})...${NC}"
#   if kill -0 "${WIND_PID}" 2>/dev/null; then
#     kill "${WIND_PID}" 2>/dev/null || true
#   fi
# }

# trap cleanup EXIT INT TERM

# Launch
echo -e "${CYAN}[3/3] Launching simulation...${NC}"
echo ""
echo -e "${GREEN}Press Ctrl+C to stop the demo${NC}"
echo ""
sleep 2

# Launch komutu
ros2 launch agent_control_pkg formation_comparison_demo.launch.py \
    gazebo_gui:=true \
    rviz:=$RVIZ_ENABLED

echo ""
echo -e "${GREEN}Demo completed!${NC}"
