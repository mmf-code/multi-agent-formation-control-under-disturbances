#!/bin/bash
################################################################################
# Demo + CSV Recording - Visual Demo with Data Logging
#
# Bu script hocaya gösterirken AYNI ANDA hem görsel demo hem CSV kaydı yapar.
# Gazebo GUI + RViz açık olur, arka planda CSV kaydedilir.
#
# Kullanım:
#   ./scripts/demo_with_recording.sh [duration_seconds]
#
# Örnek:
#   ./scripts/demo_with_recording.sh     # 60 saniye (default)
#   ./scripts/demo_with_recording.sh 90  # 90 saniye
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

# Parse duration
DURATION=${1:-60}

# Başlık
echo -e "${MAGENTA}╔════════════════════════════════════════════════════════════╗${NC}"
echo -e "${MAGENTA}║  Visual Demo + CSV Recording                               ║${NC}"
echo -e "${MAGENTA}║  Watch the demo while data is being recorded!              ║${NC}"
echo -e "${MAGENTA}╚════════════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "${CYAN}Recording Duration: ${YELLOW}${DURATION}s${NC}"
echo ""

# Workspace kontrolü
WORKSPACE_DIR="/home/mmf/Documents/GitHub/multi-agent-formation-control-under-disturbances"
if [ ! -d "$WORKSPACE_DIR" ]; then
    echo -e "${RED}ERROR: Workspace not found${NC}"
    exit 1
fi

cd "$WORKSPACE_DIR"

# Output directory oluştur
DATE=$(date +%Y-%m-%d)
TIME=$(date +%H-%M-%S)
OUTPUT_DIR="thesis_data/${DATE}/${TIME}_demo_with_recording"
mkdir -p "$OUTPUT_DIR"

echo -e "${GREEN}Output Directory: ${NC}$OUTPUT_DIR"
echo ""

# ROS2 setup
source /opt/ros/humble/setup.bash
source install/setup.bash

# Eski process'leri temizle
echo -e "${CYAN}[1/3] Cleaning up old processes...${NC}"
pkill -9 gzserver 2>/dev/null || true
pkill -9 gzclient 2>/dev/null || true
pkill -9 rviz2 2>/dev/null || true
sleep 2

# Simulation başlat (background, GUI AÇIK)
echo -e "${CYAN}[2/3] Launching simulation with GUI...${NC}"
TOTAL_DURATION=$((DURATION + 10))
timeout ${TOTAL_DURATION} ros2 launch agent_control_pkg formation_comparison_demo.launch.py \
    gazebo_gui:=true rviz:=true > "$OUTPUT_DIR/demo.log" 2>&1 &
SIM_PID=$!

# Simulation başlaması için bekle
echo -e "${CYAN}Waiting for simulation to start (10 seconds)...${NC}"
sleep 10

# Topics kontrolü
echo ""
echo -e "${CYAN}Checking topics...${NC}"
TOPICS=$(ros2 topic list | grep -E "/agent_[036]/metrics" || true)
if [ -z "$TOPICS" ]; then
    echo -e "${RED}ERROR: Metrics topics not found!${NC}"
    kill $SIM_PID 2>/dev/null || true
    exit 1
fi
echo -e "${GREEN}✓ Metrics topics active${NC}"
echo ""

# CSV logger başlat (background)
echo -e "${CYAN}[3/3] Starting CSV recording...${NC}"
echo -e "${YELLOW}Recording ${DURATION}s of data while you watch the demo...${NC}"
echo ""

python3 scripts/simple_metrics_logger.py \
    --output-dir "$OUTPUT_DIR" \
    --duration $DURATION \
    --agents 0 3 6 &
LOGGER_PID=$!

# Bilgilendirme
echo -e "${GREEN}╔════════════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║  Demo is running!                                          ║${NC}"
echo -e "${GREEN}╚════════════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "${CYAN}What you see:${NC}"
echo -e "  ${MAGENTA}● Group 0 (PID+Fuzzy)${NC} - Best wind rejection"
echo -e "  ${CYAN}● Group 1 (PD)${NC} - Fastest settling"
echo -e "  ${YELLOW}● Group 2 (PID)${NC} - Balanced performance"
echo ""
echo -e "${CYAN}Data being recorded to:${NC} $OUTPUT_DIR"
echo ""
echo -e "${YELLOW}Demo will run for ${DURATION} seconds...${NC}"
echo -e "${YELLOW}Press Ctrl+C to stop early${NC}"
echo ""

# Logger'ın bitmesini bekle
wait $LOGGER_PID 2>/dev/null || true

# Simulation'ı durdur
echo ""
echo -e "${CYAN}Stopping simulation...${NC}"
kill $SIM_PID 2>/dev/null || true
pkill -9 gzserver 2>/dev/null || true
pkill -9 gzclient 2>/dev/null || true
pkill -9 rviz2 2>/dev/null || true
sleep 2

# Sonuçlar
echo ""
echo -e "${GREEN}╔════════════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║  Demo Complete! Data Saved!                                ║${NC}"
echo -e "${GREEN}╚════════════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "${CYAN}Data location:${NC} $OUTPUT_DIR"
echo ""
echo -e "${CYAN}Files created:${NC}"
ls -lh "$OUTPUT_DIR"
echo ""

# CSV quick summary
for agent in agent_0_pidfuzzy agent_3_pd agent_6_pid; do
    CSV_FILE="$OUTPUT_DIR/${agent}.csv"
    if [ -f "$CSV_FILE" ]; then
        LINES=$(wc -l < "$CSV_FILE")
        SAMPLES=$((LINES - 1))
        SIZE=$(du -h "$CSV_FILE" | cut -f1)
        echo -e "${GREEN}✓ ${agent}.csv${NC} - ${YELLOW}${SAMPLES} samples${NC} (${SIZE})"
    fi
done

echo ""
