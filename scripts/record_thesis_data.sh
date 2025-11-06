#!/bin/bash
################################################################################
# Thesis Data Recording Script
#
# Bu script bitirme tezi için CSV veri toplar.
# Headless modda çalışır (maksimum FPS için GUI yok).
#
# Çıktı yapısı:
#   thesis_data/YYYY-MM-DD/HH-MM-SS_formation/
#     ├── agent_0_pidfuzzy.csv  (PID+Fuzzy controller data)
#     ├── agent_3_pd.csv        (PD controller data)
#     ├── agent_6_pid.csv       (PID controller data)
#     └── gazebo.log            (Simulation log)
#
# CSV formatı:
#   timestamp, elapsed_time, error_x, error_y, error_z, error_magnitude,
#   rmse_x, rmse_y, rmse_z, rmse_total, iae_x, iae_y, settling_time, is_settled
#
# Kullanım:
#   ./scripts/record_thesis_data.sh [duration_seconds]
#
# Örnek:
#   ./scripts/record_thesis_data.sh       # 60 saniye kayıt (default)
#   ./scripts/record_thesis_data.sh 120   # 120 saniye kayıt
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
echo -e "${BLUE}╔════════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║  Thesis Data Recording - Formation Comparison             ║${NC}"
echo -e "${BLUE}║  Headless Mode | CSV Logging | Organized Output           ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════════════════════════╝${NC}"
echo ""

# Parse duration
DURATION=${1:-60}
echo -e "${CYAN}Recording Duration: ${YELLOW}${DURATION}s${NC}"
echo ""

# Workspace (resolve repo root based on this script's location)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Prefer Git repo root if available; otherwise use parent of scripts/
if ROOT_DIR_GIT=$(git -C "$SCRIPT_DIR" rev-parse --show-toplevel 2>/dev/null); then
    WORKSPACE_DIR="$ROOT_DIR_GIT"
else
    WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
fi

if [ ! -d "$WORKSPACE_DIR" ]; then
    echo -e "${RED}ERROR: Workspace not found at $WORKSPACE_DIR${NC}"
    exit 1
fi

cd "$WORKSPACE_DIR"

# Output directory oluştur
DATE=$(date +%Y-%m-%d)
TIME=$(date +%H-%M-%S)
OUTPUT_DIR="thesis_data/${DATE}/${TIME}_formation"
mkdir -p "$OUTPUT_DIR"

echo -e "${GREEN}Output Directory: ${NC}$OUTPUT_DIR"
echo ""

# ROS2 setup
echo -e "${CYAN}[1/4] Setting up ROS2 environment...${NC}"
source /opt/ros/humble/setup.bash
source install/setup.bash

# Eski process'leri temizle
echo -e "${CYAN}[2/4] Cleaning up old processes...${NC}"
pkill -9 gzserver 2>/dev/null || true
pkill -9 gzclient 2>/dev/null || true
pkill -9 rviz2 2>/dev/null || true
sleep 2

# Simulation başlat (background)
echo -e "${CYAN}[3/4] Launching Gazebo simulation (headless mode)...${NC}"
TOTAL_DURATION=$((DURATION + 10))  # +10s for initialization
timeout ${TOTAL_DURATION} ros2 launch agent_control_pkg formation_comparison_demo.launch.py \
    gazebo_gui:=false rviz:=false > "$OUTPUT_DIR/gazebo.log" 2>&1 &
SIM_PID=$!

# Simulation başlaması için bekle
echo -e "${CYAN}Waiting for simulation initialization...${NC}"
sleep 8

# Topics kontrolü
echo ""
echo -e "${CYAN}Checking metrics topics...${NC}"
TOPICS=$(ros2 topic list | grep -E "/agent_[036]/metrics" || true)
if [ -z "$TOPICS" ]; then
    echo -e "${RED}ERROR: Metrics topics not found!${NC}"
    echo -e "${YELLOW}Available topics:${NC}"
    ros2 topic list
    kill $SIM_PID 2>/dev/null || true
    exit 1
fi
echo -e "${GREEN}✓ Metrics topics found${NC}"
echo ""

# CSV logger başlat
echo -e "${CYAN}[4/4] Recording CSV data...${NC}"
echo -e "${YELLOW}Agents: agent_0 (PID+Fuzzy), agent_3 (PD), agent_6 (PID)${NC}"
echo ""

# Progress bar için
PROGRESS_PID=""
(
    for i in $(seq 1 $DURATION); do
        PERCENT=$((i * 100 / DURATION))
        echo -ne "${CYAN}Progress: ${GREEN}[$PERCENT%]${CYAN} - ${NC}$((DURATION - i))s remaining\r"
        sleep 1
    done
    echo -e "${GREEN}Progress: [100%] - Complete!${NC}                    "
) &
PROGRESS_PID=$!

# CSV logger çalıştır
python3 scripts/simple_metrics_logger.py \
    --output-dir "$OUTPUT_DIR" \
    --duration $DURATION \
    --agents 0 3 6

# Progress bar'ı durdur
kill $PROGRESS_PID 2>/dev/null || true

# Simulation'ı durdur
echo ""
echo -e "${CYAN}Stopping simulation...${NC}"
kill $SIM_PID 2>/dev/null || true
pkill -9 gzserver 2>/dev/null || true
pkill -9 gzclient 2>/dev/null || true
sleep 1

# Sonuçlar
echo ""
echo -e "${GREEN}╔════════════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║  Recording Complete!                                       ║${NC}"
echo -e "${GREEN}╚════════════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "${CYAN}Data saved to:${NC} $OUTPUT_DIR"
echo ""
echo -e "${CYAN}Files created:${NC}"
ls -lh "$OUTPUT_DIR"
echo ""

# CSV preview
for agent in agent_0_pidfuzzy agent_3_pd agent_6_pid; do
    CSV_FILE="$OUTPUT_DIR/${agent}.csv"
    if [ -f "$CSV_FILE" ]; then
        LINES=$(wc -l < "$CSV_FILE")
        SAMPLES=$((LINES - 1))  # -1 for header
        echo -e "${GREEN}✓ ${agent}.csv${NC} - ${YELLOW}${SAMPLES} samples${NC}"

        # İlk birkaç satır göster
        if [ $SAMPLES -gt 0 ]; then
            echo -e "${CYAN}  Preview (first 3 data rows):${NC}"
            head -4 "$CSV_FILE" | tail -3 | while read line; do
                echo -e "    $line"
            done
        else
            echo -e "${RED}  WARNING: No data recorded!${NC}"
        fi
        echo ""
    else
        echo -e "${RED}✗ ${agent}.csv${NC} - ${RED}NOT FOUND${NC}"
    fi
done

echo ""
echo -e "${CYAN}Next steps:${NC}"
echo -e "  1. Analyze with: ${YELLOW}python3 analysis/analyze_thesis_data.py $OUTPUT_DIR${NC}"
echo -e "  2. Plot results: ${YELLOW}python3 analysis/plot_comparison.py $OUTPUT_DIR${NC}"
echo ""
