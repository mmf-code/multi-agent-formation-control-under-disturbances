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
#   ./scripts/run_formation_demo.sh                      # Full GUI (Gazebo + RViz)
#   ./scripts/run_formation_demo.sh --headless           # Sadece Gazebo GUI, RViz yok
#   ./scripts/run_formation_demo.sh --record             # Orijinal sim + CSV kayıt (60s)
#   ./scripts/run_formation_demo.sh --record-duration=90 # Orijinal sim + CSV kayıt (90s)
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
WIND_MODE="tool"  # default: Python wind tool (matches previous behaviour)
# Default: orijinal demo her çalıştığında CSV kaydeder (tez uyumlu).
RECORD_ENABLED="true"
RECORD_DURATION=60

for arg in "$@"; do
  case "$arg" in
    --headless)
      RVIZ_ENABLED="false"
      ;;
    --wind-mode=*)
      WIND_MODE="${arg#*=}"
      ;;
    --record)
      RECORD_ENABLED="true"
      ;;
    --record-duration=*)
      RECORD_ENABLED="true"
      RECORD_DURATION="${arg#*=}"
      ;;
    --no-record)
      RECORD_ENABLED="false"
      ;;
  esac
done

if [[ "$RVIZ_ENABLED" == "false" ]]; then
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
echo -e "  Gazebo world wind plugin: ${RED}8.0N${NC} mean, ${RED}3.0N${NC} variance"
echo -e "  Wind mode: ${YELLOW}${WIND_MODE}${NC} (plugin: /wind/velocity, tool: /wind/force + /wind/velocity)"
echo ""

WIND_PID=""
SIM_PID=""
cleanup() {
  if [[ -n "${WIND_PID}" ]]; then
    echo ""
    echo -e "${YELLOW}Stopping wind publisher (PID ${WIND_PID})...${NC}"
    if kill -0 "${WIND_PID}" 2>/dev/null; then
      kill "${WIND_PID}" 2>/dev/null || true
    fi
  fi
  if [[ -n "${SIM_PID}" ]]; then
    echo -e "${YELLOW}Stopping simulation (PID ${SIM_PID})...${NC}"
    kill "${SIM_PID}" 2>/dev/null || true
    pkill -9 gzserver 2>/dev/null || true
    pkill -9 gzclient 2>/dev/null || true
    pkill -9 rviz2 2>/dev/null || true
  fi
}

trap cleanup EXIT INT TERM

if [[ "${WIND_MODE}" == "tool" ]]; then
  echo -e "${CYAN}[2.5/3] Starting Python wind tool (force + velocity)...${NC}"
  # Use bash explicitly so script does not need +x
  bash scripts/run_wind.sh --x 8.0 --y 2.4 --z 0.0 --duration 999999 --mode both >/dev/null 2>&1 &
  WIND_PID=$!
  echo -e "  → Wind publisher PID: ${WIND_PID}"
  echo ""
else
  echo -e "${CYAN}[2.5/3] Using Gazebo world wind plugin only (no Python wind tool).${NC}"
  echo ""
fi

# Launch
echo -e "${CYAN}[3/3] Launching simulation...${NC}"
echo ""
echo -e "${GREEN}Press Ctrl+C to stop the demo${NC}"
echo ""
sleep 2

if [[ "${RECORD_ENABLED}" == "true" ]]; then
  # Orijinal sim + CSV kayıt (tez formatında)
  DATE=$(date +%Y-%m-%d)
  TIME=$(date +%H-%M-%S)
  OUTPUT_DIR="thesis_data/${DATE}/${TIME}_full_demo"
  mkdir -p "$OUTPUT_DIR"

  echo -e "${CYAN}Recording enabled:${NC} ${YELLOW}${RECORD_DURATION}s${NC}"
  echo -e "${CYAN}Output directory:${NC} ${YELLOW}${OUTPUT_DIR}${NC}"
  echo ""

  # Simülasyonu arka planda başlat, log'u kaydet
  TOTAL_TIME=$((RECORD_DURATION + 15))
  timeout ${TOTAL_TIME} ros2 launch agent_control_pkg formation_comparison_demo.launch.py \
      gazebo_gui:=true \
      rviz:=$RVIZ_ENABLED > "${OUTPUT_DIR}/simulation.log" 2>&1 &
  SIM_PID=$!

  echo -e "${YELLOW}Waiting 10s for simulation initialization...${NC}"
  sleep 10
  echo -e "${GREEN}Simulation running (PID: ${SIM_PID})${NC}"
  echo ""
  echo -e "${CYAN}Starting metrics logger (agents 0,3,6)...${NC}"

  python3 scripts/simple_metrics_logger.py \
      --output-dir "${OUTPUT_DIR}" \
      --duration "${RECORD_DURATION}" \
      --agents 0 3 6

  echo ""
  echo -e "${CYAN}Stopping simulation...${NC}"
  kill "${SIM_PID}" 2>/dev/null || true
  pkill -9 gzserver 2>/dev/null || true
  pkill -9 gzclient 2>/dev/null || true
  pkill -9 rviz2 2>/dev/null || true
  SIM_PID=""

  echo ""
  echo -e "${GREEN}Demo completed! Data saved to:${NC} ${YELLOW}${OUTPUT_DIR}${NC}"
else
  # Eski davranış: sadece demo, kayıt yok
  ros2 launch agent_control_pkg formation_comparison_demo.launch.py \
      gazebo_gui:=true \
      rviz:=$RVIZ_ENABLED

  echo ""
  echo -e "${GREEN}Demo completed!${NC}"
fi
