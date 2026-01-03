#!/bin/bash
###############################################################################
# Full System Integration Test
#
# Quick smoke test for wind model implementation:
# - Turbulence model switching (vonkarman → dryden)
# - Spatial wind field publishing
# - Cumulative metrics tracking
# - Short duration (40s), headless mode
#
# Usage:
#   ./scripts/test_full_system.sh
#
# Exit codes:
#   0 = PASS (all tests successful)
#   1 = FAIL (one or more tests failed)
#
# Author: Multi-Agent Formation Control Research
# Date: 2026-01-02
###############################################################################

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Test configuration
TEST_DURATION=40  # seconds
TEST_TIMEOUT=60   # seconds (with margin)
LOG_DIR="/tmp/system_test_$(date +%Y%m%d_%H%M%S)"

# Print functions
print_header() {
    echo -e "\n${BLUE}========================================${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}========================================${NC}\n"
}

print_test() {
    echo -e "${YELLOW}[TEST]${NC} $1"
}

print_pass() {
    echo -e "${GREEN}[PASS]${NC} $1"
}

print_fail() {
    echo -e "${RED}[FAIL]${NC} $1"
}

print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

# Cleanup function
cleanup() {
    print_info "Cleaning up test processes..."

    # Kill any remaining ROS2 nodes
    pkill -f "agent_controller_node" 2>/dev/null || true
    pkill -f "formation_coordinator" 2>/dev/null || true
    pkill -f "wind_publisher" 2>/dev/null || true
    pkill -f "gzserver" 2>/dev/null || true
    pkill -f "gzclient" 2>/dev/null || true

    sleep 2

    print_info "Cleanup complete"
}

# Trap to ensure cleanup on exit
trap cleanup EXIT

# Main test routine
main() {
    print_header "FULL SYSTEM INTEGRATION TEST"
    print_info "Test duration: ${TEST_DURATION}s (headless mode)"
    print_info "Log directory: ${LOG_DIR}"

    # Create log directory
    mkdir -p "${LOG_DIR}"

    # Check if ROS2 is sourced
    print_test "Checking ROS2 environment..."
    if [ -z "$ROS_DISTRO" ]; then
        print_fail "ROS2 not sourced. Please run: source /opt/ros/humble/setup.bash"
        exit 1
    fi
    print_pass "ROS2 environment OK (distro: $ROS_DISTRO)"

    # Check if workspace is built
    print_test "Checking workspace build..."
    if [ ! -f "install/setup.bash" ]; then
        print_fail "Workspace not built. Please run: colcon build --symlink-install"
        exit 1
    fi

    # Source workspace
    source install/setup.bash
    print_pass "Workspace sourced"

    # Test 1: Turbulence model validation (Python)
    print_header "TEST 1: Turbulence Model Validation"
    print_test "Running turbulence PSD validation..."

    if python3 scripts/validate_wind_model.py > "${LOG_DIR}/turbulence_test.log" 2>&1; then
        print_pass "Turbulence models validated (von Kármán + Dryden)"

        # Check if figures were generated
        if [ -f "validation_results/vonkarman_vs_dryden_comparison.png" ]; then
            print_pass "PSD figures generated"
        else
            print_fail "PSD figures missing"
            exit 1
        fi
    else
        print_fail "Turbulence validation failed (see ${LOG_DIR}/turbulence_test.log)"
        exit 1
    fi

    # Test 2: Spatial coherence validation
    print_header "TEST 2: Spatial Coherence Validation"
    print_test "Running spatial coherence validation..."

    if python3 scripts/validate_spatial_coherence.py > "${LOG_DIR}/coherence_test.log" 2>&1; then
        print_pass "Spatial coherence validated (IEC model)"

        # Check if figures were generated
        if [ -f "validation_results/iec_coherence_validation.png" ]; then
            print_pass "Coherence figures generated"
        else
            print_fail "Coherence figures missing"
            exit 1
        fi
    else
        print_fail "Coherence validation failed (see ${LOG_DIR}/coherence_test.log)"
        exit 1
    fi

    # Test 3: Wind scenario library
    print_header "TEST 3: Wind Scenario Library"
    print_test "Checking wind scenario library..."

    if python3 scripts/wind_scenarios.py --print-table > "${LOG_DIR}/scenarios_test.log" 2>&1; then
        # Check that we have expected number of scenarios
        scenario_count=$(grep -c "CALM\|MODERATE\|CHALLENGING" "${LOG_DIR}/scenarios_test.log" || echo "0")
        if [ "$scenario_count" -ge 10 ]; then
            print_pass "Wind scenario library OK ($scenario_count scenarios)"
        else
            print_fail "Wind scenario library incomplete (found $scenario_count scenarios)"
            exit 1
        fi
    else
        print_fail "Wind scenario library test failed"
        exit 1
    fi

    # Test 4: Short simulation run (headless)
    print_header "TEST 4: Short Simulation Run (Headless)"
    print_test "Launching formation demo (headless, ${TEST_DURATION}s)..."

    # Launch simulation in background
    timeout ${TEST_TIMEOUT} ros2 launch agent_control_pkg formation_comparison_demo.launch.py \
        gazebo_gui:=false \
        rviz:=false \
        > "${LOG_DIR}/simulation.log" 2>&1 &

    LAUNCH_PID=$!

    # Wait for simulation to start
    print_info "Waiting for simulation to initialize..."
    sleep 10

    # Check if Gazebo is running
    if pgrep -x "gzserver" > /dev/null; then
        print_pass "Gazebo server started"
    else
        print_fail "Gazebo server failed to start"
        exit 1
    fi

    # Test 5: Check ROS2 topics
    print_header "TEST 5: ROS2 Topic Verification"

    print_test "Checking wind velocity topic..."
    if ros2 topic info /wind/velocity > /dev/null 2>&1; then
        print_pass "Wind velocity topic exists"
    else
        print_fail "Wind velocity topic missing"
        exit 1
    fi

    print_test "Checking agent odometry topics..."
    agent_odom_count=0
    for i in {0..8}; do
        if ros2 topic info /agent_${i}/odom > /dev/null 2>&1; then
            agent_odom_count=$((agent_odom_count + 1))
        fi
    done

    if [ "$agent_odom_count" -eq 9 ]; then
        print_pass "All 9 agent odometry topics exist"
    else
        print_fail "Only $agent_odom_count/9 agent odometry topics found"
        exit 1
    fi

    print_test "Checking agent metrics topics..."
    agent_metrics_count=0
    for i in {0..8}; do
        if ros2 topic info /agent_${i}/metrics > /dev/null 2>&1; then
            agent_metrics_count=$((agent_metrics_count + 1))
        fi
    done

    if [ "$agent_metrics_count" -eq 9 ]; then
        print_pass "All 9 agent metrics topics exist"
    else
        print_fail "Only $agent_metrics_count/9 agent metrics topics found"
        exit 1
    fi

    # Test 6: Sample topic data
    print_header "TEST 6: Topic Data Sampling"

    print_test "Sampling wind velocity (5s)..."
    timeout 5 ros2 topic echo /wind/velocity --once > "${LOG_DIR}/wind_sample.txt" 2>&1 || true

    if [ -f "${LOG_DIR}/wind_sample.txt" ] && [ -s "${LOG_DIR}/wind_sample.txt" ]; then
        print_pass "Wind velocity data received"
    else
        print_fail "No wind velocity data"
        exit 1
    fi

    print_test "Sampling agent_0 metrics (5s)..."
    timeout 5 ros2 topic echo /agent_0/metrics --once > "${LOG_DIR}/metrics_sample.txt" 2>&1 || true

    if [ -f "${LOG_DIR}/metrics_sample.txt" ] && [ -s "${LOG_DIR}/metrics_sample.txt" ]; then
        print_pass "Agent metrics data received"

        # Check for mission_rmse field (cumulative metrics)
        if grep -q "mission_rmse" "${LOG_DIR}/metrics_sample.txt"; then
            print_pass "Cumulative metrics (mission_rmse) present"
        else
            print_fail "Cumulative metrics missing"
            exit 1
        fi
    else
        print_fail "No agent metrics data"
        exit 1
    fi

    # Wait for remaining test duration
    remaining_time=$((TEST_DURATION - 20))
    if [ $remaining_time -gt 0 ]; then
        print_info "Running simulation for ${remaining_time} more seconds..."
        sleep $remaining_time
    fi

    # Test 7: Thesis figures generation
    print_header "TEST 7: Thesis Figure Generation"
    print_test "Generating all thesis figures..."

    if python3 scripts/generate_thesis_figures.py > "${LOG_DIR}/figures_test.log" 2>&1; then
        print_pass "Thesis figures generated"

        # Check that all 5 figures exist
        figure_count=0
        for fig in validation_results/fig{1..5}_*.png; do
            if [ -f "$fig" ]; then
                figure_count=$((figure_count + 1))
            fi
        done

        if [ $figure_count -eq 5 ]; then
            print_pass "All 5 thesis figures present"
        else
            print_fail "Only $figure_count/5 thesis figures found"
            exit 1
        fi
    else
        print_fail "Thesis figure generation failed"
        exit 1
    fi

    # Final summary
    print_header "TEST SUMMARY"
    print_pass "All integration tests PASSED"

    echo ""
    print_info "Test logs saved to: ${LOG_DIR}"
    print_info "Validation figures in: validation_results/"
    echo ""

    # List generated figures
    echo -e "${BLUE}Generated figures:${NC}"
    ls -1 validation_results/fig*.png 2>/dev/null | while read fig; do
        echo "  - $(basename $fig)"
    done

    echo ""
    print_header "✓ SYSTEM READY FOR THESIS DEFENSE"

    return 0
}

# Run main test
main

# Exit code (0 = success)
exit 0
