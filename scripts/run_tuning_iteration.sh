#!/bin/bash
# Single tuning iteration - runs simulation and extracts metrics
# Usage: ./run_tuning_iteration.sh <duration_seconds> <output_file>

DURATION=${1:-40}
OUTPUT_FILE=${2:-"/tmp/tuning_output.txt"}
BASE_DIR="/home/mmf/Documents/GitHub/multi-agent-formation-control-under-disturbances"

# Kill any existing simulation
killall -9 gzserver agent_controller_node metrics_publisher_node formation_coordinator_node 2>/dev/null
sleep 2

# Source ROS
source /opt/ros/humble/setup.bash
source "$BASE_DIR/install/setup.bash"

echo "Starting simulation for ${DURATION}s..."
echo "Output: $OUTPUT_FILE"

# Run simulation and capture output
timeout $DURATION ros2 launch agent_control_pkg formation_comparison_demo.launch.py \
    gazebo_gui:=false rviz:=false 2>&1 | tee "$OUTPUT_FILE"

# Kill simulation
killall -9 gzserver agent_controller_node metrics_publisher_node formation_coordinator_node 2>/dev/null

# Extract and summarize metrics
echo ""
echo "============================================"
echo "METRICS SUMMARY"
echo "============================================"

# Extract RMSE values per agent and calculate averages
for agent in 0 1 2 3 4 5 6 7 8; do
    rmse_values=$(grep "agent_${agent}.metrics_publisher" "$OUTPUT_FILE" | grep "RMSE=" | sed 's/.*RMSE=\([0-9.]*\)m.*/\1/' | tail -20)
    if [ -n "$rmse_values" ]; then
        avg=$(echo "$rmse_values" | awk '{sum+=$1; count++} END {if(count>0) printf "%.3f", sum/count; else print "N/A"}')
        echo "Agent $agent: avg_RMSE=${avg}m ($(echo "$rmse_values" | wc -l) samples)"
    fi
done

echo ""
echo "Group Averages:"
for group in 0 1 2; do
    start=$((group * 3))
    end=$((start + 2))
    group_rmse=""
    for agent in $(seq $start $end); do
        rmse=$(grep "agent_${agent}.metrics_publisher" "$OUTPUT_FILE" | grep "RMSE=" | sed 's/.*RMSE=\([0-9.]*\)m.*/\1/' | tail -10 | awk '{sum+=$1; count++} END {if(count>0) printf "%.3f", sum/count}')
        if [ -n "$rmse" ]; then
            group_rmse="$group_rmse $rmse"
        fi
    done
    if [ -n "$group_rmse" ]; then
        avg=$(echo $group_rmse | tr ' ' '\n' | grep -v "^$" | awk '{sum+=$1; count++} END {if(count>0) printf "%.3f", sum/count; else print "N/A"}')
        case $group in
            0) echo "  Group 0 (PID+Fuzzy): avg_RMSE=${avg}m" ;;
            1) echo "  Group 1 (PD):        avg_RMSE=${avg}m" ;;
            2) echo "  Group 2 (PID):       avg_RMSE=${avg}m" ;;
        esac
    fi
done

echo "============================================"
