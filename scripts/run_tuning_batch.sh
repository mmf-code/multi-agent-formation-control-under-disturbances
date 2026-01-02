#!/bin/bash
# Run multiple tuning iterations with different parameters
# Each iteration: modify config -> rebuild -> run sim -> collect metrics

BASE_DIR="/home/mmf/Documents/GitHub/multi-agent-formation-control-under-disturbances"
CONFIG_FILE="$BASE_DIR/agent_control_pkg/config/ros2/agent_controller_default.yaml"
RESULTS_DIR="$BASE_DIR/tuning_results"
DURATION=45

source /opt/ros/humble/setup.bash
source "$BASE_DIR/install/setup.bash"

mkdir -p "$RESULTS_DIR"

run_iteration() {
    local name="$1"
    local pid_kp="$2"
    local pid_ki="$3"
    local pid_kd="$4"
    local pd_kp="$5"
    local pd_kd="$6"
    local k_fuzzy="$7"
    local wind_scalar="$8"

    echo ""
    echo "========================================"
    echo "ITERATION: $name"
    echo "PID: Kp=$pid_kp Ki=$pid_ki Kd=$pid_kd"
    echo "PD:  Kp=$pd_kp Kd=$pd_kd"
    echo "Fuzzy: k_fuzzy=$k_fuzzy wind_scalar=$wind_scalar"
    echo "========================================"

    # Kill any running sim
    killall -9 gzserver agent_controller_node 2>/dev/null
    sleep 2

    # Modify config - Group 0 (PID+Fuzzy) and Group 2 (PID) use pid_* params
    # Group 1 (PD) uses pd_* params
    sed -i "s/kp: [0-9.]*/kp: $pid_kp/g" "$CONFIG_FILE"
    sed -i "s/ki: [0-9.]*/ki: $pid_ki/g" "$CONFIG_FILE"
    sed -i "s/kd: [0-9.]*/kd: $pid_kd/g" "$CONFIG_FILE"
    sed -i "s/k_fuzzy: [0-9.]*/k_fuzzy: $k_fuzzy/g" "$CONFIG_FILE"
    sed -i "s/wind_scalar: [0-9.]*/wind_scalar: $wind_scalar/g" "$CONFIG_FILE"

    # Rebuild
    echo "Rebuilding..."
    cd "$BASE_DIR"
    colcon build --symlink-install --packages-select agent_control_pkg > /dev/null 2>&1

    # Run simulation
    echo "Running simulation (${DURATION}s)..."
    timeout $DURATION ros2 launch agent_control_pkg formation_comparison_demo.launch.py \
        gazebo_gui:=false rviz:=false 2>&1 | tee "$RESULTS_DIR/${name}.log"

    # Kill sim
    killall -9 gzserver agent_controller_node 2>/dev/null
    sleep 2

    # Extract metrics
    export LC_ALL=C
    echo ""
    echo "Results for $name:"
    g0=$(grep "RMSE=" "$RESULTS_DIR/${name}.log" | grep -E "agent_[012]\." | tail -15 | sed 's/.*RMSE=\([0-9.]*\)m.*/\1/' | awk '{sum+=$1; count++} END {if(count>0) printf "%.3f", sum/count; else print "N/A"}')
    g1=$(grep "RMSE=" "$RESULTS_DIR/${name}.log" | grep -E "agent_[345]\." | tail -15 | sed 's/.*RMSE=\([0-9.]*\)m.*/\1/' | awk '{sum+=$1; count++} END {if(count>0) printf "%.3f", sum/count; else print "N/A"}')
    g2=$(grep "RMSE=" "$RESULTS_DIR/${name}.log" | grep -E "agent_[678]\." | tail -15 | sed 's/.*RMSE=\([0-9.]*\)m.*/\1/' | awk '{sum+=$1; count++} END {if(count>0) printf "%.3f", sum/count; else print "N/A"}')

    echo "  Group 0 (PID+Fuzzy): ${g0}m"
    echo "  Group 1 (PD):        ${g1}m"
    echo "  Group 2 (PID):       ${g2}m"

    # Save summary
    echo "$name,$pid_kp,$pid_ki,$pid_kd,$pd_kp,$pd_kd,$k_fuzzy,$wind_scalar,$g0,$g1,$g2" >> "$RESULTS_DIR/summary.csv"
}

# Initialize summary file
echo "name,pid_kp,pid_ki,pid_kd,pd_kp,pd_kd,k_fuzzy,wind_scalar,g0_rmse,g1_rmse,g2_rmse" > "$RESULTS_DIR/summary.csv"

# Run iterations
# Format: name pid_kp pid_ki pid_kd pd_kp pd_kd k_fuzzy wind_scalar

# Iteration 1: Higher gains
run_iteration "high_gains" 5.0 3.5 4.5 6.0 5.5 0.8 2.0

# Iteration 2: Higher Ki for wind rejection
run_iteration "high_ki" 4.0 4.0 4.0 5.0 5.0 0.8 2.0

# Iteration 3: Higher Kd for damping
run_iteration "high_kd" 4.5 2.5 5.5 5.0 6.0 0.7 1.5

# Iteration 4: Balanced
run_iteration "balanced" 4.2 2.8 4.0 4.5 4.2 0.7 1.8

echo ""
echo "========================================"
echo "ALL ITERATIONS COMPLETE"
echo "========================================"
cat "$RESULTS_DIR/summary.csv"
