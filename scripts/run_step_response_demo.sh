#!/bin/bash
################################################################################
# Multi-Checkpoint Step Response Demo
#
# Features:
#   - 4 sequential step responses per controller (60 seconds total)
#   - Adaptive time windowing (15s intervals)
#   - 9 drones in 3 groups (PID+Fuzzy, PD, PID)
#   - MATLAB stepinfo() equivalent analysis per step
#   - Publication-ready results with statistical comparison
#
# Multi-Checkpoint Trajectory:
#   - Step 1 (t=0-15s):   X=-15 → X=-10 (5m step)
#   - Step 2 (t=15-30s):  X=-10 → X=-5  (5m step)
#   - Step 3 (t=30-45s):  X=-5  → X=0   (5m step)
#   - Step 4 (t=45-60s):  X=0   → X=5   (5m step)
#   - 15s intervals allow all controllers to settle (even slow ones)
#   - Fast controllers reach target early and wait at checkpoint
#
# Lanes: Group 0 (Y=-5m), Group 1 (Y=0m), Group 2 (Y=+5m)
#
# Usage:
#   ./scripts/run_step_response_demo.sh [OPTIONS]
#
# Options:
#   --duration SECONDS    Recording duration (default: 60)
#   --headless            No Gazebo GUI (faster)
#   --no-rviz             No RViz visualization
#   --quick-test          Ultra-fast 30s test (2 steps only)
#
# Examples:
#   ./scripts/run_step_response_demo.sh                  # Full 60s (4 steps)
#   ./scripts/run_step_response_demo.sh --quick-test     # 30s (2 steps)
#   ./scripts/run_step_response_demo.sh --headless       # Headless (max FPS)
################################################################################

set -e

# DDS middleware selection happens after ROS is sourced (see below)

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
echo "║      MULTI-CHECKPOINT STEP RESPONSE - Controller Comparison   ║"
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
echo -e "${CYAN}3D Zigzag Trajectory (4 Maneuvers, 60s):${NC}"
echo -e "  • ${YELLOW}Maneuver 1 (t=0-15s):${NC}   Forward+Right+Up   (light disturbance)"
echo -e "  • ${YELLOW}Maneuver 2 (t=15-30s):${NC}  Forward+Left+Down  (medium disturbance)"
echo -e "  • ${YELLOW}Maneuver 3 (t=30-45s):${NC}  Forward+Right+Up   (heavy disturbance)"
echo -e "  • ${YELLOW}Maneuver 4 (t=45-60s):${NC}  Forward+Center     (variable disturbance)"
echo ""
echo -e "${CYAN}Trajectory Details:${NC}"
echo -e "  • X-axis: -15m → 5m (forward progression)"
echo -e "  • Y-axis: Zigzag pattern (±3m lateral)"
echo -e "  • Z-axis: Three altitude lanes (separated to prevent collisions)"
echo ""
echo -e "${CYAN}Formation Groups (3 altitude lanes):${NC}"
echo -e "  • ${MAGENTA}Group 0${NC}: PID+Fuzzy (agents 0,1,2) - Y=-5m, Z=1.0-1.8m (lowest)"
echo -e "  • ${CYAN}Group 1${NC}: PD (agents 3,4,5) - Y=0m, Z=4.0-4.8m (middle, +3m)"
echo -e "  • ${YELLOW}Group 2${NC}: PID (agents 6,7,8) - Y=+5m, Z=7.0-7.8m (highest, +6m)"
echo ""
echo -e "${CYAN}Metrics per Step:${NC}"
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

# Choose DDS implementation with fallback
if ros2 pkg prefix rmw_cyclonedds_cpp >/dev/null 2>&1; then
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
else
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export FASTDDS_SHM_DISABLE=${FASTDDS_SHM_DISABLE:-1}
    export FASTDDS_SHM_ON=0
    export RMW_FASTRTPS_USE_SHM=0
    rm -f /dev/shm/fastrtps_* 2>/dev/null || true
fi

source install/setup.bash 2>/dev/null || true

# Ensure Gazebo can find our models and plugins explicitly (belt-and-suspenders)
PKG_PREFIX=$(ros2 pkg prefix agent_control_pkg 2>/dev/null || echo "")
if [ -n "$PKG_PREFIX" ]; then
  export GAZEBO_MODEL_PATH="${PKG_PREFIX}/share/agent_control_pkg/models:${GAZEBO_MODEL_PATH}"
  export GAZEBO_PLUGIN_PATH="${PKG_PREFIX}/lib:${GAZEBO_PLUGIN_PATH}"
  export LD_LIBRARY_PATH="${PKG_PREFIX}/lib:${LD_LIBRARY_PATH}"
fi
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
pkill -9 -f agent_controller_node 2>/dev/null || true
pkill -9 -f metrics_publisher_node 2>/dev/null || true
pkill -9 -f formation_coordinator_node 2>/dev/null || true
pkill -9 -f path_visualizer_node 2>/dev/null || true
pkill -9 -f enhanced_metrics_logger.py 2>/dev/null || true
rm -f /dev/shm/fastrtps_* 2>/dev/null || true
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

# Verify topics (odom + target_pose + metrics)
echo -e "${CYAN}[6/8] Verifying ROS2 topics...${NC}"

check_topic() {
    local T="$1"; local LABEL="$2"
    if ros2 topic list 2>/dev/null | grep -q "$T"; then
        echo -e "  ${GREEN}• Found${NC} ${LABEL}: ${T}"
        return 0
    else
        echo -e "  ${RED}• Missing${NC} ${LABEL}: ${T}"
        return 1
    fi
}

OK=0
check_topic "/agent_0/odom" "odom a0" || OK=1
check_topic "/agent_3/odom" "odom a3" || OK=1
check_topic "/agent_6/odom" "odom a6" || OK=1
check_topic "/agent_0/target_pose" "target a0" || OK=1
check_topic "/agent_3/target_pose" "target a3" || OK=1
check_topic "/agent_6/target_pose" "target a6" || OK=1
check_topic "/agent_0/metrics" "metrics a0" || OK=1
check_topic "/agent_3/metrics" "metrics a3" || OK=1
check_topic "/agent_6/metrics" "metrics a6" || OK=1

if [ "$OK" -eq 0 ]; then
    echo -e "${GREEN}✓ All required topics detected${NC}"
else
    echo -e "${YELLOW}⚠ Some topics not detected. Continuing; logger may show 0 samples.${NC}"
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
sleep 2
pkill -9 gzserver 2>/dev/null || true
pkill -9 gzclient 2>/dev/null || true
pkill -9 rviz2 2>/dev/null || true
sleep 2

# Run trajectory tracking analysis
echo ""
echo -e "${CYAN}Running Trajectory Tracking Analysis...${NC}"
echo ""
python3 scripts/analyze_trajectory_tracking.py "$OUTPUT_DIR/final_results"

# Show results (don't clear - keep analysis visible)
echo ""
echo ""
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
echo -e "  ${GREEN}raw_data/${NC}              - Full CSV files (60s, 20Hz sampling)"
echo -e "  ${GREEN}final_results/${NC}         - Summary + complete datasets"
echo -e "  ${GREEN}final_results/analysis/${NC} - Trajectory tracking metrics + plots"
echo ""

# List generated files
if [ -d "$OUTPUT_DIR/final_results/analysis" ]; then
    echo -e "${CYAN}Generated Files:${NC}"
    ls -1 "$OUTPUT_DIR/final_results/analysis"/*.png 2>/dev/null | while read plot; do
        echo -e "  ${GREEN}✓${NC} $(basename $plot)"
    done
    if [ -f "$OUTPUT_DIR/final_results/analysis/trajectory_tracking_comparison.csv" ]; then
        echo -e "  ${GREEN}✓${NC} trajectory_tracking_comparison.csv"
    fi
    echo ""
fi

# Next steps
echo -e "${CYAN}═══════════════════════════════════════════════════════════════${NC}"
echo -e "${YELLOW}${BOLD}Next Steps:${NC}"
echo ""
echo -e "${CYAN}1. View Trajectory Tracking Plots:${NC}"
echo -e "   xdg-open $OUTPUT_DIR/final_results/analysis/tracking_errors_time.png"
echo -e "   xdg-open $OUTPUT_DIR/final_results/analysis/trajectory_3d.png"
echo ""
echo -e "${CYAN}2. View Performance Comparison:${NC}"
echo -e "   xdg-open $OUTPUT_DIR/final_results/analysis/rmse_comparison.png"
echo -e "   xdg-open $OUTPUT_DIR/final_results/analysis/iae_comparison.png"
echo ""
echo -e "${CYAN}3. Detailed Metrics CSV:${NC}"
echo -e "   cat $OUTPUT_DIR/final_results/analysis/trajectory_tracking_comparison.csv"
echo ""
echo -e "${CYAN}4. Re-analyze (if needed):${NC}"
echo -e "   python3 scripts/analyze_trajectory_tracking.py $OUTPUT_DIR/final_results"
echo ""
echo -e "${CYAN}═══════════════════════════════════════════════════════════════${NC}"
echo ""
