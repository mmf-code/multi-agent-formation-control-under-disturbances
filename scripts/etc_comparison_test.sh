#!/bin/bash
# ETC Comparison Test Script
# Runs simulation twice: once with ETC disabled (baseline), once with ETC enabled
# Compares event counts, bandwidth reduction, and control performance

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

# Source ROS2
source /opt/ros/humble/setup.bash
source "$PROJECT_DIR/install/setup.bash"

DURATION=60  # seconds
OUTPUT_DIR="$PROJECT_DIR/etc_test_results"
mkdir -p "$OUTPUT_DIR"

echo "=============================================="
echo "  ETC Comparison Test"
echo "=============================================="
echo "Duration: ${DURATION}s per run"
echo "Output: $OUTPUT_DIR"
echo ""

# Function to run simulation and collect ETC metrics
run_test() {
    local ETC_ENABLE=$1
    local RUN_NAME=$2
    local OUTPUT_FILE="$OUTPUT_DIR/${RUN_NAME}_metrics.txt"

    echo "----------------------------------------------"
    echo "Running: $RUN_NAME (etc.enable=$ETC_ENABLE)"
    echo "----------------------------------------------"

    # Kill any existing ROS/Gazebo processes
    pkill -9 gzserver 2>/dev/null || true
    pkill -9 gzclient 2>/dev/null || true
    pkill -9 ros2 2>/dev/null || true
    sleep 2

    # Start simulation in background with ETC override
    timeout ${DURATION}s ros2 launch agent_control_pkg twelve_drone_comparison.launch.py \
        gazebo_gui:=false \
        -p "etc.enable:=$ETC_ENABLE" \
        -p "etc.epsilon_pos:=0.05" \
        -p "etc.min_period_sec:=0.02" \
        -p "etc.max_period_sec:=0.5" \
        > /dev/null 2>&1 &

    SIM_PID=$!

    # Wait for simulation to start
    sleep 10

    # Record ETC metrics at the end
    echo "Recording metrics..."
    sleep $((DURATION - 15))

    # Capture final ETC metrics from all formation coordinators
    {
        echo "=== $RUN_NAME (etc.enable=$ETC_ENABLE) ==="
        echo "Timestamp: $(date)"
        echo ""

        # Try to get metrics from each formation coordinator
        for ns in formation_0 formation_1 formation_2 formation_3; do
            echo "--- $ns ---"
            timeout 2s ros2 topic echo --once /$ns/etc_metrics 2>/dev/null || echo "No metrics available"
        done
    } > "$OUTPUT_FILE"

    # Cleanup
    kill $SIM_PID 2>/dev/null || true
    pkill -9 gzserver 2>/dev/null || true
    sleep 3

    echo "Results saved to: $OUTPUT_FILE"
    echo ""
}

# Run baseline (ETC disabled)
run_test "false" "baseline_etc_off"

# Run with ETC enabled
run_test "true" "etc_on"

echo "=============================================="
echo "  Comparison Complete"
echo "=============================================="
echo ""
echo "Results:"
ls -la "$OUTPUT_DIR"
echo ""
echo "To view results:"
echo "  cat $OUTPUT_DIR/baseline_etc_off_metrics.txt"
echo "  cat $OUTPUT_DIR/etc_on_metrics.txt"
