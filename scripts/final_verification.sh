#!/bin/bash
# Final Verification - Optimized Parameters (k_fuzzy=0.35, wind_scalar=1.0)
set -e
export LC_ALL=C

BASE_DIR="/home/mmf/Documents/GitHub/multi-agent-formation-control-under-disturbances"
RESULTS_DIR="$BASE_DIR/verification_results"
mkdir -p "$RESULTS_DIR"

source /opt/ros/humble/setup.bash
source "$BASE_DIR/install/setup.bash"

TIMESTAMP=$(date +%Y%m%d_%H%M%S)
SUMMARY="$RESULTS_DIR/final_verification_${TIMESTAMP}.csv"
REPORT="$RESULTS_DIR/final_verification_${TIMESTAMP}.txt"

log() { echo "$1" | tee -a "$REPORT"; }

cleanup() {
    killall -9 gzserver gzclient agent_controller_node 2>/dev/null || true
    rm -rf /dev/shm/fastrtps* 2>/dev/null || true
    sleep 5
}

run_test() {
    local test_name="$1"
    local log_file="$RESULTS_DIR/${test_name}_${TIMESTAMP}.log"

    log ""
    log "=== $test_name ==="
    cleanup

    timeout 55 ros2 launch agent_control_pkg formation_comparison_demo.launch.py \
        gazebo_gui:=false rviz:=false 2>&1 | tee "$log_file" || true
    sleep 2

    g0=$(grep "RMSE=" "$log_file" | grep -E "agent_[012]\." | tail -10 | \
         sed 's/.*RMSE=\([0-9.]*\)m.*/\1/' | \
         awk '$1 < 5.0 {sum+=$1; count++} END {if(count>0) printf "%.4f", sum/count; else print "N/A"}')
    g1=$(grep "RMSE=" "$log_file" | grep -E "agent_[345]\." | tail -10 | \
         sed 's/.*RMSE=\([0-9.]*\)m.*/\1/' | \
         awk '$1 < 5.0 {sum+=$1; count++} END {if(count>0) printf "%.4f", sum/count; else print "N/A"}')
    g2=$(grep "RMSE=" "$log_file" | grep -E "agent_[678]\." | tail -10 | \
         sed 's/.*RMSE=\([0-9.]*\)m.*/\1/' | \
         awk '$1 < 5.0 {sum+=$1; count++} END {if(count>0) printf "%.4f", sum/count; else print "N/A"}')

    log "  G0 (PID+Fuzzy): ${g0}m | G1 (PD): ${g1}m | G2 (PID): ${g2}m"

    if [ "$g0" != "N/A" ] && [ "$g2" != "N/A" ]; then
        pid_lt_pd=$(echo "$g2 < $g1" | bc -l)
        fuzzy_lt_pid=$(echo "$g0 < $g2" | bc -l)

        [ "$pid_lt_pd" = "1" ] && log "  [PASS] PID < PD" || log "  [FAIL] PID >= PD"
        [ "$fuzzy_lt_pid" = "1" ] && log "  [PASS] Fuzzy < PID" || log "  [FAIL] Fuzzy >= PID"

        echo "$test_name,$g0,$g1,$g2,$pid_lt_pd,$fuzzy_lt_pid" >> "$SUMMARY"
    fi
}

log "=========================================="
log "FINAL VERIFICATION (Optimized k_fuzzy=0.35)"
log "Timestamp: $TIMESTAMP"
log "=========================================="

echo "test,g0_rmse,g1_rmse,g2_rmse,pid_lt_pd,fuzzy_lt_pid" > "$SUMMARY"

run_test "final_run_1"
run_test "final_run_2"
run_test "final_run_3"

cleanup

log ""
log "=========================================="
log "SUMMARY"
log "=========================================="
cat "$SUMMARY" | tee -a "$REPORT"

pass_count=$(grep ",1,1$" "$SUMMARY" | wc -l)
total_count=$(tail -n +2 "$SUMMARY" | wc -l)

log ""
log "Pass rate: $pass_count / $total_count"

if [ "$pass_count" -ge 2 ]; then
    log "[SUCCESS] THESIS REQUIREMENTS VERIFIED"
else
    log "[WARNING] Review results"
fi
