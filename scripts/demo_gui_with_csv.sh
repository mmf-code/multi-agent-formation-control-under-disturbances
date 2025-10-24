#!/bin/bash
################################################################################
# Demo with Gazebo GUI + CSV Recording
#
# Bu script Gazebo GUI'yi GÖRÜNÜR halde açar ve CSV kaydeder
################################################################################

set -e

DURATION=${1:-60}

# Renk tanımları
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${CYAN}╔════════════════════════════════════════════════════════════╗${NC}"
echo -e "${CYAN}║  Gazebo GUI Demo + CSV Recording                          ║${NC}"
echo -e "${CYAN}╚════════════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "${YELLOW}Duration: ${DURATION}s${NC}"
echo ""

# Workspace
cd /home/mmf/Documents/GitHub/multi-agent-formation-control-under-disturbances

# ROS2 setup
source /opt/ros/humble/setup.bash
source install/setup.bash

# Output dir
DATE=$(date +%Y-%m-%d)
TIME=$(date +%H-%M-%S)
OUTPUT_DIR="thesis_data/${DATE}/${TIME}_gui_demo"
mkdir -p "$OUTPUT_DIR"

echo -e "${GREEN}Output: ${OUTPUT_DIR}${NC}"
echo ""

# Cleanup
echo -e "${CYAN}[1/4] Cleaning old processes...${NC}"
pkill -9 gzserver 2>/dev/null || true
pkill -9 gzclient 2>/dev/null || true
pkill -9 rviz2 2>/dev/null || true
sleep 2
echo -e "${GREEN}✓ Clean${NC}"
echo ""

# Launch Gazebo WITH GUI
echo -e "${CYAN}[2/4] Launching Gazebo (GUI VISIBLE)...${NC}"
TOTAL_TIME=$((DURATION + 20))
timeout ${TOTAL_TIME} ros2 launch agent_control_pkg formation_comparison_demo.launch.py \
    gazebo_gui:=true \
    rviz:=false > "$OUTPUT_DIR/gazebo.log" 2>&1 &
SIM_PID=$!

# CSV LOGGER'I HEMEN BAŞLAT (background'da)
echo -e "${CYAN}[3/4] Starting CSV logger (background)...${NC}"
echo -e "${YELLOW}  CSV recording started immediately - no data loss!${NC}"
python3 scripts/simple_metrics_logger.py \
    --output-dir "$OUTPUT_DIR" \
    --duration $DURATION \
    --agents 0 3 6 > "$OUTPUT_DIR/csv_logger.log" 2>&1 &
CSV_PID=$!

echo -e "${YELLOW}  Waiting for Gazebo GUI to open (10 seconds)...${NC}"
sleep 10
echo -e "${GREEN}✓ Gazebo should be visible now!${NC}"
echo ""

# Check topics
echo -e "${CYAN}[4/4] Monitoring...${NC}"
TOPIC_COUNT=$(ros2 topic list 2>/dev/null | grep -E "/agent_[036]/metrics" | wc -l)
if [ "$TOPIC_COUNT" -eq 3 ]; then
    echo -e "${GREEN}✓ All 3 metrics topics active${NC}"
else
    echo -e "${YELLOW}⚠ Found ${TOPIC_COUNT}/3 topics (will retry)${NC}"
fi
echo ""
echo -e "${YELLOW}Watch the Gazebo window - drones should be moving!${NC}"
echo -e "${YELLOW}CSV recording in progress (${DURATION} seconds)...${NC}"
echo ""

# Wait for CSV logger to finish
wait $CSV_PID 2>/dev/null || true

# Cleanup
echo ""
echo -e "${CYAN}Stopping simulation...${NC}"
kill $SIM_PID 2>/dev/null || true
sleep 1
pkill -9 gzserver 2>/dev/null || true
pkill -9 gzclient 2>/dev/null || true

# Results
echo ""
echo -e "${GREEN}╔════════════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║  Recording Complete!                                       ║${NC}"
echo -e "${GREEN}╚════════════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "${CYAN}Data saved to: ${OUTPUT_DIR}${NC}"
echo ""
ls -lh "$OUTPUT_DIR"
echo ""

# Quick check
for agent in agent_0_pidfuzzy agent_3_pd agent_6_pid; do
    CSV_FILE="$OUTPUT_DIR/${agent}.csv"
    if [ -f "$CSV_FILE" ]; then
        LINES=$(wc -l < "$CSV_FILE")
        SAMPLES=$((LINES - 1))
        echo -e "${GREEN}✓ ${agent}.csv${NC} - ${YELLOW}${SAMPLES} samples${NC}"
    fi
done

echo ""
