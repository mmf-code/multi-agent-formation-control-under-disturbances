#!/bin/bash
################################################################################
# Long Thesis Demo - 5 Minute Zigzag Scenario
#
# Features:
#   - 5-phase zigzag trajectory (waypoint-based)
#   - 9 drones in 3 groups (PID+Fuzzy, PD, PID)
#   - Checkpoint saves every 60s
#   - Enhanced metrics logging
#   - Atmospheric wind disturbances
#   - Real-time dashboard
#
# Duration: 5 minutes simulation time (runs faster in Gazebo with RTF >1.0)
#
# Usage:
#   ./scripts/run_long_thesis_demo.sh [OPTIONS]
#
# Options:
#   --duration SECONDS    Recording duration (default: 300)
#   --headless            No Gazebo GUI (faster)
#   --no-rviz             No RViz visualization
#   --test-mode           Quick 1-minute test (60s)
#
# Examples:
#   ./scripts/run_long_thesis_demo.sh                  # Full 5-minute demo
#   ./scripts/run_long_thesis_demo.sh --test-mode      # Quick 1-minute test
#   ./scripts/run_long_thesis_demo.sh --headless       # Headless (max FPS)
################################################################################

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
MAGENTA='\033[0;35m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

# Default parameters
DURATION=300
GAZEBO_GUI="true"
RVIZ="true"
TEST_MODE=false

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --duration)
            DURATION="$2"
            shift 2
            ;;
        --headless)
            GAZEBO_GUI="false"
            shift
            ;;
        --no-rviz)
            RVIZ="false"
            shift
            ;;
        --test-mode)
            TEST_MODE=true
            DURATION=60
            shift
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

# Test mode adjustments
if [ "$TEST_MODE" = true ]; then
    echo -e "${YELLOW}TEST MODE: Running 1-minute quick test${NC}"
fi

# Banner
clear 2>/dev/null || true
echo -e "${MAGENTA}${BOLD}"
echo "╔════════════════════════════════════════════════════════════════╗"
echo "║                                                                ║"
echo "║         LONG THESIS DEMO - 5 Phase Zigzag Scenario            ║"
echo "║         Multi-Agent Formation Control Under Disturbances       ║"
echo "║                                                                ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo -e "${NC}"
echo ""
echo -e "${CYAN}Configuration:${NC}"
echo -e "  Duration:     ${YELLOW}${DURATION}s${NC} ($(echo "scale=1; $DURATION/60" | bc) minutes)"
echo -e "  Gazebo GUI:   ${YELLOW}${GAZEBO_GUI}${NC}"
echo -e "  RViz:         ${YELLOW}${RVIZ}${NC}"
echo ""
echo -e "${CYAN}Scenario:${NC}"
echo -e "  • 5 phases with zigzag trajectory"
echo -e "  • Altitude changes (1.0m → 3.0m → 1.5m → 2.5m → 2.0m)"
echo -e "  • Checkpoints saved every 60s"
echo -e "  • ${MAGENTA}Group 0${NC}: PID+Fuzzy (agents 0,1,2)"
echo -e "  • ${CYAN}Group 1${NC}: PD (agents 3,4,5)"
echo -e "  • ${YELLOW}Group 2${NC}: PID (agents 6,7,8)"
echo ""

# Workspace (resolve repo root based on this script's location)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Prefer Git repo root if available; otherwise use parent of scripts/
if ROOT_DIR_GIT=$(git -C "$SCRIPT_DIR" rev-parse --show-toplevel 2>/dev/null); then
    WORKSPACE_DIR="$ROOT_DIR_GIT"
else
    WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
fi

# Navigate
echo -e "${CYAN}[1/8] Navigating to workspace...${NC}"
cd "$WORKSPACE_DIR"
echo -e "${GREEN}✓ ${PWD}${NC}"
echo ""

# ROS2 Setup
echo -e "${CYAN}[2/8] Setting up ROS2 environment...${NC}"
source /opt/ros/humble/setup.bash 2>/dev/null || true
source install/setup.bash 2>/dev/null || true
echo -e "${GREEN}✓ ROS2 environment ready${NC}"
echo ""

# Output directory
DATE=$(date +%Y-%m-%d)
TIME=$(date +%H-%M-%S)
OUTPUT_DIR="thesis_data/${DATE}/${TIME}_long_scenario"
mkdir -p "$OUTPUT_DIR"
echo -e "${CYAN}[3/8] Output directory created${NC}"
echo -e "${GREEN}✓ ${OUTPUT_DIR}${NC}"
echo ""

# Cleanup
echo -e "${CYAN}[4/8] Cleaning up old processes...${NC}"
pkill -9 gzserver 2>/dev/null || true
pkill -9 gzclient 2>/dev/null || true
pkill -9 rviz2 2>/dev/null || true
sleep 2
echo -e "${GREEN}✓ Clean slate${NC}"
echo ""

# Check if we need to build
if [ ! -f "install/setup.bash" ]; then
    echo -e "${YELLOW}⚠ Workspace not built, building now...${NC}"
    colcon build --packages-select my_custom_interfaces_pkg formation_coordinator_pkg agent_control_pkg
    source install/setup.bash
fi

# Launch simulation (background)
echo -e "${CYAN}[5/8] Launching Gazebo + RViz (Long Scenario)...${NC}"
TOTAL_TIME=$((DURATION + 20))

# Launch the LONG SCENARIO formation demo with waypoint-based configs
timeout ${TOTAL_TIME} ros2 launch agent_control_pkg formation_long_scenario_demo.launch.py \
    gazebo_gui:=${GAZEBO_GUI} rviz:=${RVIZ} > "$OUTPUT_DIR/simulation.log" 2>&1 &
SIM_PID=$!

echo -e "${YELLOW}  Waiting for simulation initialization (15 seconds)...${NC}"
sleep 15
echo -e "${GREEN}✓ Simulation running (PID: ${SIM_PID})${NC}"
echo ""

# Verify topics
echo -e "${CYAN}[6/8] Verifying ROS2 topics...${NC}"
TOPIC_COUNT=$(ros2 topic list 2>/dev/null | grep -E "/agent_[036]/metrics" | wc -l)
if [ "$TOPIC_COUNT" -eq 3 ]; then
    echo -e "${GREEN}✓ All 3 metrics topics active${NC}"
else
    echo -e "${YELLOW}⚠ Found ${TOPIC_COUNT}/3 topics (may still work)${NC}"
fi
echo ""

# Start enhanced logger
echo -e "${CYAN}[7/8] Starting Enhanced Metrics Logger...${NC}"
echo ""
echo -e "${MAGENTA}╔════════════════════════════════════════════════════════════════╗${NC}"
echo -e "${MAGENTA}║                  RECORDING IN PROGRESS                         ║${NC}"
echo -e "${MAGENTA}╚════════════════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "${CYAN}Scenario Progress:${NC}"
echo -e "  ${GREEN}Phase 1 (0-60s)${NC}:   Initial formation + light wind"
echo -e "  ${GREEN}Phase 2 (60-120s)${NC}: Zigzag maneuver + medium wind"
echo -e "  ${GREEN}Phase 3 (120-180s)${NC}: Altitude change + heavy gusts"
echo -e "  ${GREEN}Phase 4 (180-240s)${NC}: Backward motion + variable wind"
echo -e "  ${GREEN}Phase 5 (240-300s)${NC}: Final positioning + stress test"
echo ""
echo -e "${YELLOW}Real-time Dashboard:${NC}"
echo ""

# Run enhanced logger
python3 scripts/enhanced_metrics_logger.py \
    --output-dir "$OUTPUT_DIR" \
    --duration $DURATION \
    --agents 0 3 6

# Cleanup
echo ""
echo ""
echo -e "${CYAN}[8/8] Stopping simulation...${NC}"
kill $SIM_PID 2>/dev/null || true
sleep 1
pkill -9 gzserver 2>/dev/null || true
pkill -9 gzclient 2>/dev/null || true
pkill -9 rviz2 2>/dev/null || true
sleep 1

# Results
clear 2>/dev/null || true
echo -e "${GREEN}${BOLD}"
echo "╔════════════════════════════════════════════════════════════════╗"
echo "║                                                                ║"
echo "║                LONG DEMO COMPLETE!                             ║"
echo "║                                                                ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo -e "${NC}"
echo ""
echo -e "${CYAN}Data Location:${NC}"
echo -e "${YELLOW}${OUTPUT_DIR}${NC}"
echo ""
echo -e "${CYAN}Output Structure:${NC}"
echo -e "  ${GREEN}raw_data/${NC}         - Full 300s CSV files"
echo -e "  ${GREEN}checkpoints/${NC}      - Phase-by-phase data (60s intervals)"
echo -e "  ${GREEN}final_results/${NC}    - Summary + complete datasets"
echo ""

# Show checkpoint info
if [ -d "$OUTPUT_DIR/checkpoints" ]; then
    echo -e "${CYAN}Checkpoints Saved:${NC}"
    ls -1 "$OUTPUT_DIR/checkpoints" | while read phase_dir; do
        echo -e "  ${GREEN}✓${NC} $phase_dir"
    done
    echo ""
fi

# File listing
echo -e "${CYAN}Raw Data Files:${NC}"
ls -lh "$OUTPUT_DIR/raw_data" 2>/dev/null | tail -n +2 | while read line; do
    echo "  $line"
done
echo ""

# Next steps
echo -e "${CYAN}═══════════════════════════════════════════════════════════════${NC}"
echo -e "${YELLOW}${BOLD}Next Steps:${NC}"
echo ""
echo -e "${CYAN}1. Quick Analysis:${NC}"
echo -e "   python3 analysis/analyze_checkpoint_data.py $OUTPUT_DIR"
echo ""
echo -e "${CYAN}2. Generate Plots:${NC}"
echo -e "   python3 analysis/plot_long_scenario.py $OUTPUT_DIR"
echo ""
echo -e "${CYAN}3. Thesis Report:${NC}"
echo -e "   python3 analysis/generate_thesis_report.py $OUTPUT_DIR"
echo ""
echo -e "${CYAN}4. Compare Controllers:${NC}"
echo -e "   python3 analysis/compare_phases.py $OUTPUT_DIR"
echo ""
echo -e "${CYAN}═══════════════════════════════════════════════════════════════${NC}"
echo ""
