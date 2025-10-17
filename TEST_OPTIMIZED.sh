#!/bin/bash
###############################################################################
# Optimized Demo Test Script
# Tests the minimal world with explicit PID parameters
###############################################################################

echo "=========================================="
echo "  OPTIMIZED DEMO TEST"
echo "=========================================="
echo ""
echo "Changes applied:"
echo "  ✓ World: demo_presentation.world → minimal_test.world"
echo "  ✓ PID gains: Explicit override in launch file"
echo "  ✓ Documentation: Cleaned up (TEST_NOW.txt, RUN_COMMANDS.txt removed)"
echo ""
echo "Expected improvements:"
echo "  • Gazebo startup: 2-3 minutes → 5-10 seconds"
echo "  • Settling time: ~42s → ~8-12s (with explicit PID)"
echo "  • Overshoot: ~50% → ~10-15%"
echo ""
echo "----------------------------------------"
echo "Starting test..."
echo "----------------------------------------"
echo ""

# Cleanup
pkill -9 gzserver gzclient 2>/dev/null || true
sleep 1

# Source and launch
cd /home/mmf/Documents/GitHub/multi-agent-formation-control-under-disturbances
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "Launching with minimal world..."
echo ""

ros2 launch agent_control_pkg demo_presentation.launch.py
