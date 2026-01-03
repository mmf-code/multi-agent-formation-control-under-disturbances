#!/bin/bash
# Quick Wind Disturbance Test
#
# Runs a short simulation with wind and calculates KPIs.
# Use this for rapid validation of wind integration.
#
# Usage:
#   ./scripts/quick_wind_test.sh [duration_seconds]
#   ./scripts/quick_wind_test.sh 90

set -e

# Configuration
DURATION=${1:-60}  # Default: 60 seconds
SCENARIO="moderate_wind_moderate_turbulence"
OUTPUT_DIR="thesis_data/quick_wind_tests"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}=========================================="
echo "Quick Wind Disturbance Test"
echo "=========================================="
echo -e "Duration: ${DURATION}s"
echo -e "Scenario: ${SCENARIO}"
echo -e "==========================================${NC}\n"

# Check if workspace is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}Error: ROS2 not sourced!${NC}"
    echo "Run: source /opt/ros/humble/setup.bash"
    exit 1
fi

if [ ! -d "install" ]; then
    echo -e "${RED}Error: Workspace not built!${NC}"
    echo "Run: colcon build --symlink-install"
    exit 1
fi

# Source workspace
source install/setup.bash

# Create output directory
mkdir -p "$OUTPUT_DIR"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
LOG_FILE="$OUTPUT_DIR/wind_test_${TIMESTAMP}.log"

# Launch simulation with timeout
echo -e "${YELLOW}Launching simulation...${NC}"
timeout ${DURATION}s ros2 launch agent_control_pkg formation_comparison_demo.launch.py \
    gazebo_gui:=false \
    rviz:=false \
    wind_profile:=vonkarman \
    2>&1 | tee "$LOG_FILE" &

LAUNCH_PID=$!

# Wait for simulation to start
echo -e "${YELLOW}Waiting for simulation to initialize...${NC}"
sleep 5

# Monitor wind topics
echo -e "${YELLOW}Monitoring wind topics...${NC}"
echo "Checking if per-drone wind is active..."

# Check if spatial wind is publishing
AGENT_WIND_CHECK=$(timeout 2 ros2 topic list | grep "/agent_0/wind/velocity" || echo "")

if [ -n "$AGENT_WIND_CHECK" ]; then
    echo -e "${GREEN}✓ Per-drone wind topics detected (spatial coherence active)${NC}"
    WIND_MODE="spatial"
else
    echo -e "${YELLOW}⚠ Using global wind topic (spatial coherence not active)${NC}"
    WIND_MODE="global"
fi

# Sample wind data
echo "Sampling wind velocity..."
timeout 2 ros2 topic echo /agent_0/wind/velocity --once 2>/dev/null || \
timeout 2 ros2 topic echo /wind/velocity --once 2>/dev/null || \
echo "No wind data"

# Wait for simulation to complete
echo -e "\n${YELLOW}Running test for ${DURATION}s...${NC}"
wait $LAUNCH_PID || true  # Ignore timeout error

echo -e "\n${GREEN}✓ Simulation complete${NC}"

# Process results if CSV metrics exist
METRICS_CSV=$(find thesis_data -name "metrics_*.csv" -type f -mmin -5 | head -1)

if [ -n "$METRICS_CSV" ]; then
    echo -e "\n${BLUE}Calculating Wind KPIs...${NC}"

    # Run KPI calculator
    python3 scripts/wind_kpi_calculator.py "$METRICS_CSV" \
        --output "$OUTPUT_DIR/kpis_${TIMESTAMP}.json" \
        --plot

    KPI_STATUS=$?

    if [ $KPI_STATUS -eq 0 ]; then
        echo -e "${GREEN}✓ All KPIs within target range!${NC}"
    else
        echo -e "${YELLOW}⚠ Some KPIs outside target range${NC}"
    fi

    # Display key metrics
    echo -e "\n${BLUE}Key Metrics Summary:${NC}"
    grep -E "DRR|WNRMSE|Mean Wind" "$OUTPUT_DIR/kpis_${TIMESTAMP}.json" || true

else
    echo -e "${YELLOW}⚠ No metrics CSV found. Check if data logging is enabled.${NC}"
fi

# Summary
echo -e "\n${BLUE}=========================================="
echo "Test Complete"
echo "=========================================="
echo -e "Wind Mode: ${WIND_MODE}"
echo -e "Log File: ${LOG_FILE}"
if [ -n "$METRICS_CSV" ]; then
    echo -e "Metrics CSV: ${METRICS_CSV}"
    echo -e "KPIs JSON: $OUTPUT_DIR/kpis_${TIMESTAMP}.json"
fi
echo -e "==========================================${NC}\n"

exit 0
