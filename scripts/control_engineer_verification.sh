#!/bin/bash
# Control Engineer Verification Script
# Runs multiple checks to verify controller behavior meets thesis requirements
# Defense-industry grade verification

set -e
export LC_ALL=C

BASE_DIR="/home/mmf/Documents/GitHub/multi-agent-formation-control-under-disturbances"
RESULTS_DIR="$BASE_DIR/verification_results"
mkdir -p "$RESULTS_DIR"

source /opt/ros/humble/setup.bash
source "$BASE_DIR/install/setup.bash"

TIMESTAMP=$(date +%Y%m%d_%H%M%S)
REPORT="$RESULTS_DIR/verification_report_${TIMESTAMP}.txt"

log() {
    echo "$1" | tee -a "$REPORT"
}

run_test() {
    local test_name="$1"
    local duration="$2"
    local log_file="$RESULTS_DIR/${test_name}_${TIMESTAMP}.log"

    log ""
    log "========================================"
    log "TEST: $test_name"
    log "Duration: ${duration}s"
    log "========================================"

    # Kill any running sim
    killall -9 gzserver agent_controller_node 2>/dev/null || true
    sleep 3

    # Run simulation
    log "Starting simulation..."
    timeout $duration ros2 launch agent_control_pkg formation_comparison_demo.launch.py \
        gazebo_gui:=false rviz:=false 2>&1 | tee "$log_file"

    # Kill sim
    killall -9 gzserver agent_controller_node 2>/dev/null || true
    sleep 2

    # Extract metrics
    log ""
    log "--- METRICS EXTRACTION ---"

    # Check motor model is enabled
    motor_count=$(grep "Motor model ENABLED" "$log_file" | wc -l)
    log "Motor model enabled count: $motor_count (expected: 9)"

    # Check controller types
    fuzzy_count=$(grep "type=pid_fuzzy" "$log_file" | wc -l)
    pd_count=$(grep "type=pd" "$log_file" | wc -l)
    pid_count=$(grep "type=pid," "$log_file" | wc -l)
    log "Controller types: PID+Fuzzy=$fuzzy_count, PD=$pd_count, PID=$pid_count (expected: 3,3,3)"

    # Extract RMSE per group (last 15 samples each)
    g0=$(grep "RMSE=" "$log_file" | grep -E "agent_[012]\." | tail -15 | sed 's/.*RMSE=\([0-9.]*\)m.*/\1/' | awk '{sum+=$1; count++} END {if(count>0) printf "%.4f", sum/count; else print "N/A"}')
    g1=$(grep "RMSE=" "$log_file" | grep -E "agent_[345]\." | tail -15 | sed 's/.*RMSE=\([0-9.]*\)m.*/\1/' | awk '{sum+=$1; count++} END {if(count>0) printf "%.4f", sum/count; else print "N/A"}')
    g2=$(grep "RMSE=" "$log_file" | grep -E "agent_[678]\." | tail -15 | sed 's/.*RMSE=\([0-9.]*\)m.*/\1/' | awk '{sum+=$1; count++} END {if(count>0) printf "%.4f", sum/count; else print "N/A"}')

    log ""
    log "Group 0 (PID+Fuzzy): RMSE = ${g0}m"
    log "Group 1 (PD):        RMSE = ${g1}m"
    log "Group 2 (PID):       RMSE = ${g2}m"

    # Thesis requirement checks
    log ""
    log "--- THESIS REQUIREMENTS ---"

    # Check PID < PD
    pid_better=$(echo "$g2 < $g1" | bc -l)
    if [ "$pid_better" = "1" ]; then
        log "[PASS] PID ($g2) < PD ($g1)"
    else
        log "[FAIL] PID ($g2) >= PD ($g1)"
    fi

    # Check Fuzzy < PID
    fuzzy_better=$(echo "$g0 < $g2" | bc -l)
    if [ "$fuzzy_better" = "1" ]; then
        log "[PASS] PID+Fuzzy ($g0) < PID ($g2)"
    else
        log "[FAIL] PID+Fuzzy ($g0) >= PID ($g2)"
    fi

    # Check reasonable error bounds (< 2m for all)
    g0_ok=$(echo "$g0 < 2.0" | bc -l)
    g1_ok=$(echo "$g1 < 2.0" | bc -l)
    g2_ok=$(echo "$g2 < 2.0" | bc -l)

    if [ "$g0_ok" = "1" ] && [ "$g1_ok" = "1" ] && [ "$g2_ok" = "1" ]; then
        log "[PASS] All RMSE < 2.0m (acceptable for 3.5m/s wind)"
    else
        log "[WARN] Some RMSE >= 2.0m"
    fi

    # Store results
    echo "$test_name,$g0,$g1,$g2,$pid_better,$fuzzy_better" >> "$RESULTS_DIR/verification_summary.csv"

    log ""
}

# Initialize
log "========================================"
log "CONTROL ENGINEER VERIFICATION"
log "Timestamp: $TIMESTAMP"
log "========================================"
log ""
log "System Configuration:"
log "  - 9 drones (3 groups x 3 agents)"
log "  - Group 0: PID + IT2-Fuzzy (agents 0,1,2)"
log "  - Group 1: PD only (agents 3,4,5)"
log "  - Group 2: PID only (agents 6,7,8)"
log "  - Wind: 3.5 m/s gust profile"
log "  - Motor model: sim_cf2 Crazyflie"
log ""

# Initialize summary
echo "test_name,g0_rmse,g1_rmse,g2_rmse,pid_lt_pd,fuzzy_lt_pid" > "$RESULTS_DIR/verification_summary.csv"

# Run multiple tests
run_test "verification_run_1" 50
run_test "verification_run_2" 50
run_test "verification_run_3" 50

# Final summary
log ""
log "========================================"
log "FINAL VERIFICATION SUMMARY"
log "========================================"

log ""
log "All test results:"
cat "$RESULTS_DIR/verification_summary.csv" | tee -a "$REPORT"

log ""
log "Pass/Fail Analysis:"
pass_count=$(grep ",1,1$" "$RESULTS_DIR/verification_summary.csv" | wc -l)
total_count=$(tail -n +2 "$RESULTS_DIR/verification_summary.csv" | wc -l)
log "  Tests passed: $pass_count / $total_count"

if [ "$pass_count" = "$total_count" ]; then
    log ""
    log "[SUCCESS] ALL VERIFICATION TESTS PASSED"
    log "System is thesis-ready."
else
    log ""
    log "[WARNING] Some tests did not meet all requirements"
    log "Review individual results for details."
fi

log ""
log "Report saved to: $REPORT"
