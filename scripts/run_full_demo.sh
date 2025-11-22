#!/bin/bash
################################################################################
# FULL DEMO - All-in-One Script
#
# Bu script TEK KOMUTLA her şeyi yapar:
#   1. Workspace'e gider
#   2. ROS2 setup yapar
#   3. Simulation başlatır (Gazebo + RViz)
#   4. CSV kayıt yapar
#   5. Bitince sonuçları gösterir
#
# Kullanım:
#   bash ~/Documents/GitHub/multi-agent-formation-control-under-disturbances/scripts/run_full_demo.sh
#
#   VEYA workspace içinden:
#   ./scripts/run_full_demo.sh [duration]
#
# Örnekler:
#   ./scripts/run_full_demo.sh        # 60 saniye (default)
#   ./scripts/run_full_demo.sh 90     # 90 saniye
#   ./scripts/run_full_demo.sh 120    # 120 saniye
################################################################################

set -e

# Renk tanımları
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
MAGENTA='\033[0;35m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m' # No Color

# Duration
DURATION=${1:-60}
WIND_PID=""

# Banner
if [ -t 1 ]; then clear 2>/dev/null || true; fi
echo -e "${MAGENTA}${BOLD}"
echo "╔═══════════════════════════════════════════════════════════════╗"
echo "║                                                               ║"
echo "║         MULTI-AGENT FORMATION CONTROL DEMO                    ║"
echo "║         Full System Demo + CSV Data Recording                 ║"
echo "║                                                               ║"
echo "╚═══════════════════════════════════════════════════════════════╝"
echo -e "${NC}"
echo ""
echo -e "${CYAN}Duration: ${YELLOW}${DURATION} seconds${NC}"
echo ""

# Workspace (resolve repo root based on this script's location)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Prefer Git repo root if available; otherwise use parent of scripts/
if ROOT_DIR_GIT=$(git -C "$SCRIPT_DIR" rev-parse --show-toplevel 2>/dev/null); then
    WORKSPACE_DIR="$ROOT_DIR_GIT"
else
    WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
fi

# 1. CD to workspace
echo -e "${CYAN}[1/8] Navigating to workspace...${NC}"
cd "$WORKSPACE_DIR"
echo -e "${GREEN}✓ Current directory: ${PWD}${NC}"
echo ""

# 2. ROS2 Setup
echo -e "${CYAN}[2/8] Setting up ROS2 environment...${NC}"
source /opt/ros/humble/setup.bash
source install/setup.bash
echo -e "${GREEN}✓ ROS2 Humble loaded${NC}"
echo -e "${GREEN}✓ Workspace sourced${NC}"
echo ""

# 3. Output directory
DATE=$(date +%Y-%m-%d)
TIME=$(date +%H-%M-%S)
OUTPUT_DIR="thesis_data/${DATE}/${TIME}_full_demo"
mkdir -p "$OUTPUT_DIR"
echo -e "${CYAN}[3/8] Output directory created${NC}"
echo -e "${GREEN}✓ ${OUTPUT_DIR}${NC}"
echo ""

# 4. Cleanup old processes
echo -e "${CYAN}[4/8] Cleaning up old processes...${NC}"
pkill -9 gzserver 2>/dev/null || true
pkill -9 gzclient 2>/dev/null || true
pkill -9 rviz2 2>/dev/null || true
sleep 2
echo -e "${GREEN}✓ Clean slate${NC}"
echo ""

# 5. Wind publisher fallback (if Gazebo wind plugin is missing)
TOTAL_TIME=$((DURATION + 15))
WIND_SCRIPT="${WORKSPACE_DIR}/scripts/run_wind.sh"
if ! find /opt/ros/humble /usr/lib -name "libgazebo_ros_wind.so" -print -quit | grep -q .; then
    echo -e "${YELLOW}[5/8] Gazebo wind plugin not found; starting Python wind publisher...${NC}"
    if [ -f "${WIND_SCRIPT}" ]; then
        # Run slightly longer than simulation timeout to keep wind active until shutdown
        bash "${WIND_SCRIPT}" --x 2.0 --y 8.0 --z 0.0 --duration "${TOTAL_TIME}" --mode both >/dev/null 2>&1 &
        WIND_PID=$!
        echo -e "${GREEN}✓ Wind publisher running (PID: ${WIND_PID})${NC}"
    else
        echo -e "${RED}✗ Wind script missing at ${WIND_SCRIPT}${NC}"
    fi
else
    echo -e "${GREEN}[5/8] Gazebo wind plugin found (libgazebo_ros_wind.so); no Python wind fallback needed.${NC}"
fi
echo ""

# 6. Launch simulation (background)
echo -e "${CYAN}[6/8] Launching Gazebo + RViz...${NC}"
timeout ${TOTAL_TIME} ros2 launch agent_control_pkg formation_comparison_demo.launch.py \
    gazebo_gui:=true rviz:=true > "$OUTPUT_DIR/simulation.log" 2>&1 &
SIM_PID=$!

echo -e "${YELLOW}  Waiting for simulation initialization (15 seconds)...${NC}"
sleep 15
echo -e "${GREEN}✓ Simulation running (PID: ${SIM_PID})${NC}"
echo ""

# 7. Check topics
echo -e "${CYAN}[7/8] Verifying ROS2 topics...${NC}"
TOPIC_COUNT=$(ros2 topic list --no-daemon 2>/dev/null | grep -E "/agent_[036]/metrics" | wc -l)
if [ "$TOPIC_COUNT" -eq 3 ]; then
    echo -e "${GREEN}✓ All 3 metrics topics active${NC}"
else
    echo -e "${RED}✗ ERROR: Expected 3 topics, found ${TOPIC_COUNT}${NC}"
    kill $SIM_PID 2>/dev/null || true
    exit 1
fi
echo ""

# 8. Start CSV recording
echo -e "${CYAN}[8/8] Starting CSV data recording...${NC}"
echo ""
echo -e "${MAGENTA}╔═══════════════════════════════════════════════════════════════╗${NC}"
echo -e "${MAGENTA}║                  DEMO IS RUNNING!                             ║${NC}"
echo -e "${MAGENTA}╚═══════════════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "${CYAN}What you see:${NC}"
echo -e "  ${MAGENTA}● Group 0 (Magenta Drones)${NC} → PID+Fuzzy  → Best wind rejection"
echo -e "  ${CYAN}● Group 1 (Cyan Drones)${NC}    → PD         → Fastest settling"
echo -e "  ${YELLOW}● Group 2 (Yellow Drones)${NC}  → PID        → Balanced"
echo ""
echo -e "${CYAN}Wind Disturbance:${NC} ${RED}8.0N mean, 3.0N variance${NC}"
echo ""
echo -e "${YELLOW}Recording ${DURATION}s of CSV data...${NC}"
echo ""

# Progress indicator
(
    for i in $(seq 1 $DURATION); do
        PERCENT=$((i * 100 / DURATION))
        BAR_LENGTH=$((PERCENT / 2))
        BAR=$(printf "%-50s" "$(printf '#%.0s' $(seq 1 $BAR_LENGTH))")
        echo -ne "${CYAN}Progress: [${GREEN}${BAR}${CYAN}] ${YELLOW}${PERCENT}%${CYAN} (${NC}$((DURATION - i))s left${CYAN})${NC}\r"
        sleep 1
    done
    echo -ne "\n"
) &
PROGRESS_PID=$!

# Run CSV logger
python3 scripts/simple_metrics_logger.py \
    --output-dir "$OUTPUT_DIR" \
    --duration $DURATION \
    --agents 0 3 6 2>/dev/null

# Stop progress bar
kill $PROGRESS_PID 2>/dev/null || true

# Cleanup
echo ""
echo -e "${CYAN}Stopping simulation...${NC}"
kill $SIM_PID 2>/dev/null || true
sleep 1
pkill -9 gzserver 2>/dev/null || true
pkill -9 gzclient 2>/dev/null || true
pkill -9 rviz2 2>/dev/null || true
if [ -n "${WIND_PID}" ]; then
    kill "${WIND_PID}" 2>/dev/null || true
fi
sleep 1

# Results
if [ -t 1 ]; then clear 2>/dev/null || true; fi
echo -e "${GREEN}${BOLD}"
echo "╔═══════════════════════════════════════════════════════════════╗"
echo "║                                                               ║"
echo "║                    DEMO COMPLETE!                             ║"
echo "║                                                               ║"
echo "╚═══════════════════════════════════════════════════════════════╝"
echo -e "${NC}"
echo ""
echo -e "${CYAN}Data saved to:${NC}"
echo -e "${YELLOW}${OUTPUT_DIR}${NC}"
echo ""
echo -e "${CYAN}═══════════════════════════════════════════════════════════════${NC}"
echo -e "${CYAN}Files created:${NC}"
echo ""

# CSV analysis
for agent_num in 0 3 6; do
    controller=""
    case $agent_num in
        0) controller="pidfuzzy" ;;
        3) controller="pd" ;;
        6) controller="pid" ;;
    esac

    CSV_FILE="$OUTPUT_DIR/agent_${agent_num}_${controller}.csv"

    if [ -f "$CSV_FILE" ]; then
        LINES=$(wc -l < "$CSV_FILE")
        SAMPLES=$((LINES - 1))
        SIZE=$(du -h "$CSV_FILE" | cut -f1)

        echo -e "${GREEN}✓ agent_${agent_num}_${controller}.csv${NC}"
        echo -e "  Samples: ${YELLOW}${SAMPLES}${NC}"
        echo -e "  Size: ${YELLOW}${SIZE}${NC}"

        # Get first and last error values
        if [ $SAMPLES -gt 2 ]; then
            FIRST_ERROR=$(awk -F',' 'NR==2 {printf "%.3f", $6}' "$CSV_FILE")
            LAST_ERROR=$(awk -F',' 'END {printf "%.3f", $6}' "$CSV_FILE")
            FIRST_TIME=$(awk -F',' 'NR==2 {printf "%.1f", $2}' "$CSV_FILE")
            LAST_TIME=$(awk -F',' 'END {printf "%.1f", $2}' "$CSV_FILE")

            echo -e "  Error @ t=${FIRST_TIME}s: ${RED}${FIRST_ERROR}m${NC} → t=${LAST_TIME}s: ${GREEN}${LAST_ERROR}m${NC}"
        fi
        echo ""
    else
        echo -e "${RED}✗ agent_${agent_num}_${controller}.csv NOT FOUND${NC}"
        echo ""
    fi
done

echo -e "${CYAN}═══════════════════════════════════════════════════════════════${NC}"
echo ""
echo -e "${CYAN}Simulation log:${NC}"
LOG_SIZE=$(du -h "$OUTPUT_DIR/simulation.log" | cut -f1)
echo -e "  ${OUTPUT_DIR}/simulation.log (${LOG_SIZE})"
echo ""

# Final summary
echo -e "${GREEN}${BOLD}All data successfully recorded!${NC}"
echo ""
echo -e "${CYAN}Next steps:${NC}"
echo -e "  1. Analyze CSV files for thesis"
echo -e "  2. Compare RMSE/IAE between controllers"
echo -e "  3. Plot time-series data"
echo ""
echo -e "${YELLOW}To run again: ./scripts/run_full_demo.sh [duration]${NC}"
echo ""
