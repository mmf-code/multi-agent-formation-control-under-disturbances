#!/bin/bash
# Extended Scenario Demo - Longer distance and stronger wind

set -e
DURATION=${1:-120}

cd /home/mmf/Documents/GitHub/multi-agent-formation-control-under-disturbances
source /opt/ros/humble/setup.bash
source install/setup.bash

DATE=$(date +%Y-%m-%d)
TIME=$(date +%H-%M-%S)
OUTPUT_DIR="thesis_data/${DATE}/${TIME}_extended"
mkdir -p "$OUTPUT_DIR"

echo "Extended Scenario: 45m distance, 6.0N wind, ${DURATION}s"
echo "Output: $OUTPUT_DIR"

pkill -9 gzserver gzclient rviz2 2>/dev/null || true
sleep 2

# Swap configs
cp agent_control_pkg/worlds/formation_comparison.world agent_control_pkg/worlds/formation_comparison_TEMP.world
cp agent_control_pkg/worlds/formation_comparison_extended.world agent_control_pkg/worlds/formation_comparison.world

cp other_packages/formation_coordinator_pkg/config/formation_group0_fuzzy.yaml other_packages/formation_coordinator_pkg/config/formation_group0_fuzzy_TEMP.yaml
cp other_packages/formation_coordinator_pkg/config/formation_group0_fuzzy_extended.yaml other_packages/formation_coordinator_pkg/config/formation_group0_fuzzy.yaml

cp other_packages/formation_coordinator_pkg/config/formation_group1_pd.yaml other_packages/formation_coordinator_pkg/config/formation_group1_pd_TEMP.yaml
cp other_packages/formation_coordinator_pkg/config/formation_group1_pd_extended.yaml other_packages/formation_coordinator_pkg/config/formation_group1_pd.yaml

cp other_packages/formation_coordinator_pkg/config/formation_group2_pid.yaml other_packages/formation_coordinator_pkg/config/formation_group2_pid_TEMP.yaml
cp other_packages/formation_coordinator_pkg/config/formation_group2_pid_extended.yaml other_packages/formation_coordinator_pkg/config/formation_group2_pid.yaml

echo "Files swapped. Launching..."

TOTAL_TIME=$((DURATION + 20))
timeout ${TOTAL_TIME} ros2 launch agent_control_pkg formation_comparison_demo.launch.py gazebo_gui:=true rviz:=false > "$OUTPUT_DIR/gazebo.log" 2>&1 &
SIM_PID=$!

python3 scripts/simple_metrics_logger.py --output-dir "$OUTPUT_DIR" --duration $DURATION --agents 0 3 6 > "$OUTPUT_DIR/csv.log" 2>&1 &
CSV_PID=$!

echo "Gazebo opening (15s)..."
sleep 15
echo "Recording ${DURATION}s..."

wait $CSV_PID 2>/dev/null || true
kill $SIM_PID 2>/dev/null || true
pkill -9 gzserver gzclient 2>/dev/null || true

# Restore
echo "Restoring original files..."
mv agent_control_pkg/worlds/formation_comparison_TEMP.world agent_control_pkg/worlds/formation_comparison.world
mv other_packages/formation_coordinator_pkg/config/formation_group0_fuzzy_TEMP.yaml other_packages/formation_coordinator_pkg/config/formation_group0_fuzzy.yaml
mv other_packages/formation_coordinator_pkg/config/formation_group1_pd_TEMP.yaml other_packages/formation_coordinator_pkg/config/formation_group1_pd.yaml
mv other_packages/formation_coordinator_pkg/config/formation_group2_pid_TEMP.yaml other_packages/formation_coordinator_pkg/config/formation_group2_pid.yaml

echo "Complete. Data: $OUTPUT_DIR"
ls -lh "$OUTPUT_DIR"
