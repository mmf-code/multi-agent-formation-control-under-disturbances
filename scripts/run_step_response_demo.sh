#!/bin/bash
################################################################################
# Step Response Demo - Fast Controller Comparison
#
# Features:
#   - Fast step response (60 seconds)
#   - Direct motion (no waypoints)
#   - 9 drones in 3 groups (PID+Fuzzy, PD, PID)
#   - MATLAB stepinfo() equivalent analysis
#   - Publication-ready results
#
# Step Characteristics:
#   - Initial: X=-15m (3 lanes: Y=-5, 0, +5)
#   - Target: X=5m (20m forward step)
#   - Velocity: 2 m/s (fast but stable)
#   - Duration: 60s (settling analysis)
#
# Usage:
#   ./scripts/run_step_response_demo.sh [OPTIONS]
#
# Options:
#   --duration SECONDS    Recording duration (default: 60)
#   --headless            No Gazebo GUI (faster)
#   --no-rviz             No RViz visualization
#   --quick-test          Ultra-fast 30s test
#
# Examples:
#   ./scripts/run_step_response_demo.sh                  # Full 60s demo
#   ./scripts/run_step_response_demo.sh --quick-test     # 30s test
#   ./scripts/run_step_response_demo.sh --headless       # Headless (max FPS)
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
DURATION=60
GAZEBO_GUI="true"
RVIZ="true"
QUICK_TEST=false

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
        --quick-test)
            QUICK_TEST=true
            DURATION=30
            shift
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

# Quick test adjustments
if [ "$QUICK_TEST" = true ]; then
    echo -e "${YELLOW}QUICK TEST: Running 30s ultra-fast test${NC}"
fi

# Banner
clear 2>/dev/null || true
echo -e "${MAGENTA}${BOLD}"
echo "╔════════════════════════════════════════════════════════════════╗"
echo "║                                                                ║"
echo "║         STEP RESPONSE DEMO - Fast Controller Comparison       ║"
echo "║         Multi-Agent Formation Control Under Disturbances       ║"
echo "║                                                                ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo -e "${NC}"
echo ""
echo -e "${CYAN}Configuration:${NC}"
echo -e "  Duration:     ${YELLOW}${DURATION}s${NC}"
echo -e "  Gazebo GUI:   ${YELLOW}${GAZEBO_GUI}${NC}"
echo -e "  RViz:         ${YELLOW}${RVIZ}${NC}"
echo ""
echo -e "${CYAN}Step Response Scenario:${NC}"
echo -e "  • Fast forward step: -15m → +5m (20m distance)"
echo -e "  • Velocity: 2 m/s (constant)"
echo -e "  • 3 formation groups in parallel lanes"
echo -e "  • ${MAGENTA}Group 0${NC}: PID+Fuzzy (agents 0,1,2) - Lane Y=-5m"
echo -e "  • ${CYAN}Group 1${NC}: PD (agents 3,4,5) - Lane Y=0m"
echo -e "  • ${YELLOW}Group 2${NC}: PID (agents 6,7,8) - Lane Y=+5m"
echo ""
echo -e "${CYAN}Metrics Analyzed:${NC}"
echo -e "  • Rise time (10%-90%)"
echo -e "  • Overshoot (%)"
echo -e "  • Settling time (±2% band)"
echo -e "  • Peak time"
echo -e "  • Steady-state error"
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
OUTPUT_DIR="thesis_data/${DATE}/${TIME}_step_response"
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
echo -e "${CYAN}[5/8] Launching Gazebo + RViz (Step Response)...${NC}"
TOTAL_TIME=$((DURATION + 20))

# Launch the STEP RESPONSE formation demo
timeout ${TOTAL_TIME} ros2 launch agent_control_pkg formation_step_response_demo.launch.py \
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
echo -e "${CYAN}Step Response Timeline:${NC}"
echo -e "  ${GREEN}0-10s${NC}:   Formation accelerates (2 m/s)"
echo -e "  ${GREEN}10-20s${NC}:  Approaching target (settling begins)"
echo -e "  ${GREEN}20-40s${NC}:  Fine settling (±2% band)"
echo -e "  ${GREEN}40-60s${NC}:  Steady-state analysis"
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

# Run step response analysis
echo ""
echo -e "${CYAN}Running Step Response Analysis...${NC}"
echo ""
python3 scripts/analyze_step_response.py "$OUTPUT_DIR/final_results" --output-dir "$OUTPUT_DIR/analysis"

# Results
clear 2>/dev/null || true
echo -e "${GREEN}${BOLD}"
echo "╔════════════════════════════════════════════════════════════════╗"
echo "║                                                                ║"
echo "║                STEP RESPONSE DEMO COMPLETE!                    ║"
echo "║                                                                ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo -e "${NC}"
echo ""
echo -e "${CYAN}Data Location:${NC}"
echo -e "${YELLOW}${OUTPUT_DIR}${NC}"
echo ""
echo -e "${CYAN}Output Structure:${NC}"
echo -e "  ${GREEN}raw_data/${NC}         - Full CSV files (60s)"
echo -e "  ${GREEN}final_results/${NC}    - Summary + complete datasets"
echo -e "  ${GREEN}analysis/${NC}         - Step response metrics + plots"
echo ""

# Show analysis results
if [ -f "$OUTPUT_DIR/analysis/step_response_summary.txt" ]; then
    echo -e "${CYAN}Step Response Results:${NC}"
    echo ""
    cat "$OUTPUT_DIR/analysis/step_response_summary.txt"
    echo ""
fi

# List plots
if [ -d "$OUTPUT_DIR/analysis" ]; then
    echo -e "${CYAN}Generated Plots:${NC}"
    ls -1 "$OUTPUT_DIR/analysis"/*.png 2>/dev/null | while read plot; do
        echo -e "  ${GREEN}✓${NC} $(basename $plot)"
    done
    echo ""
fi

# Next steps
echo -e "${CYAN}═══════════════════════════════════════════════════════════════${NC}"
echo -e "${YELLOW}${BOLD}Next Steps:${NC}"
echo ""
echo -e "${CYAN}1. View Step Response Plots:${NC}"
echo -e "   xdg-open $OUTPUT_DIR/analysis/step_response_error_magnitude.png"
echo ""
echo -e "${CYAN}2. Detailed Metrics CSV:${NC}"
echo -e "   cat $OUTPUT_DIR/analysis/step_response_metrics.csv"
echo ""
echo -e "${CYAN}3. Compare with Long Scenario:${NC}"
echo -e "   python3 scripts/compare_controllers.py $OUTPUT_DIR [LONG_SCENARIO_DIR]"
echo ""
echo -e "${CYAN}4. Re-analyze (if needed):${NC}"
echo -e "   python3 scripts/analyze_step_response.py $OUTPUT_DIR/final_results"
echo ""
echo -e "${CYAN}═══════════════════════════════════════════════════════════════${NC}"
echo ""
