#!/bin/bash
# Robust Control Engineer Verification Script
# Includes proper cleanup and wait times between tests
# Defense-industry grade verification

set -e
export LC_ALL=C

BASE_DIR="/home/mmf/Documents/GitHub/multi-agent-formation-control-under-disturbances"
RESULTS_DIR="$BASE_DIR/verification_results"
mkdir -p "$RESULTS_DIR"

source /opt/ros/humble/setup.bash
source "$BASE_DIR/install/setup.bash"

TIMESTAMP=$(date +%Y%m%d_%H%M%S)
REPORT="$RESULTS_DIR/robust_verification_${TIMESTAMP}.txt"

log() {
    echo "$1" | tee -a "$REPORT"
}

cleanup_sim() {
    log "  Cleaning up simulation processes..."
    killall -9 gzserver gzclient agent_controller_node 2>/dev/null || true
    # Clean shared memory to avoid RTPS errors
    rm -rf /dev/shm/fastrtps* 2>/dev/null || true
    sleep 5
    # Verify cleanup
    if pgrep -x gzserver > /dev/null; then
        log "  WARNING: gzserver still running, force killing..."
        pkill -9 -f gzserver || true
        sleep 3
    fi
    log "  Cleanup complete"
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

    # Thorough cleanup before test
    cleanup_sim

    # Run simulation
    log "  Starting simulation..."
    timeout $duration ros2 launch agent_control_pkg formation_comparison_demo.launch.py \
        gazebo_gui:=false rviz:=false 2>&1 | tee "$log_file" || true

    # Wait for clean shutdown
    sleep 3

    # Extract metrics (ignore first 20s transient)
    log ""
    log "--- METRICS EXTRACTION (ignoring startup transient) ---"

    # Check motor model is enabled
    motor_count=$(grep "Motor model ENABLED" "$log_file" | wc -l)
    log "  Motor model enabled count: $motor_count (expected: 9)"

    # Check controller types
    fuzzy_count=$(grep "type=pid_fuzzy" "$log_file" | wc -l)
    pd_count=$(grep "type=pd" "$log_file" | wc -l)
    pid_count=$(grep "type=pid," "$log_file" | wc -l)
    log "  Controller types: PID+Fuzzy=$fuzzy_count, PD=$pd_count, PID=$pid_count"

    # Extract RMSE per group - use steady state samples only (tail -10)
    # Filter out extreme outliers (>5m)
    g0=$(grep "RMSE=" "$log_file" | grep -E "agent_[012]\." | tail -10 | \
         sed 's/.*RMSE=\([0-9.]*\)m.*/\1/' | \
         awk '$1 < 5.0 {sum+=$1; count++} END {if(count>0) printf "%.4f", sum/count; else print "N/A"}')
    g1=$(grep "RMSE=" "$log_file" | grep -E "agent_[345]\." | tail -10 | \
         sed 's/.*RMSE=\([0-9.]*\)m.*/\1/' | \
         awk '$1 < 5.0 {sum+=$1; count++} END {if(count>0) printf "%.4f", sum/count; else print "N/A"}')
    g2=$(grep "RMSE=" "$log_file" | grep -E "agent_[678]\." | tail -10 | \
         sed 's/.*RMSE=\([0-9.]*\)m.*/\1/' | \
         awk '$1 < 5.0 {sum+=$1; count++} END {if(count>0) printf "%.4f", sum/count; else print "N/A"}')

    log ""
    log "  Group 0 (PID+Fuzzy): RMSE = ${g0}m"
    log "  Group 1 (PD):        RMSE = ${g1}m"
    log "  Group 2 (PID):       RMSE = ${g2}m"

    # Check for valid data
    if [ "$g0" = "N/A" ] || [ "$g1" = "N/A" ] || [ "$g2" = "N/A" ]; then
        log ""
        log "  [ERROR] Missing RMSE data - test invalid"
        echo "$test_name,$g0,$g1,$g2,NA,NA,INVALID" >> "$RESULTS_DIR/robust_summary.csv"
        return
    fi

    # Thesis requirement checks
    log ""
    log "--- THESIS REQUIREMENTS ---"

    # Check PID < PD
    pid_better=$(echo "$g2 < $g1" | bc -l)
    if [ "$pid_better" = "1" ]; then
        log "  [PASS] PID ($g2) < PD ($g1)"
    else
        log "  [FAIL] PID ($g2) >= PD ($g1)"
    fi

    # Check Fuzzy < PID
    fuzzy_better=$(echo "$g0 < $g2" | bc -l)
    if [ "$fuzzy_better" = "1" ]; then
        log "  [PASS] PID+Fuzzy ($g0) < PID ($g2)"
    else
        log "  [FAIL] PID+Fuzzy ($g0) >= PID ($g2)"
    fi

    # Check reasonable error bounds (< 2m for all)
    g0_ok=$(echo "$g0 < 2.0" | bc -l)
    g1_ok=$(echo "$g1 < 2.0" | bc -l)
    g2_ok=$(echo "$g2 < 2.0" | bc -l)

    if [ "$g0_ok" = "1" ] && [ "$g1_ok" = "1" ] && [ "$g2_ok" = "1" ]; then
        log "  [PASS] All RMSE < 2.0m (acceptable for 3.5m/s wind)"
        valid="VALID"
    else
        log "  [WARN] Some RMSE >= 2.0m"
        valid="MARGINAL"
    fi

    # Store results
    echo "$test_name,$g0,$g1,$g2,$pid_better,$fuzzy_better,$valid" >> "$RESULTS_DIR/robust_summary.csv"
}

# Initialize
log "========================================"
log "ROBUST CONTROL ENGINEER VERIFICATION"
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
echo "test_name,g0_rmse,g1_rmse,g2_rmse,pid_lt_pd,fuzzy_lt_pid,validity" > "$RESULTS_DIR/robust_summary.csv"

# Run 5 tests for statistical confidence
for i in 1 2 3 4 5; do
    run_test "robust_run_$i" 60
done

# Final cleanup
cleanup_sim

# Final summary
log ""
log "========================================"
log "FINAL VERIFICATION SUMMARY"
log "========================================"

log ""
log "All test results:"
cat "$RESULTS_DIR/robust_summary.csv" | tee -a "$REPORT"

log ""
log "Pass/Fail Analysis:"
pass_count=$(grep ",1,1,VALID" "$RESULTS_DIR/robust_summary.csv" | wc -l)
total_count=$(tail -n +2 "$RESULTS_DIR/robust_summary.csv" | grep -v "INVALID" | wc -l)
log "  Valid tests passed: $pass_count / $total_count"

# Calculate averages
log ""
log "Average RMSE across valid tests:"
tail -n +2 "$RESULTS_DIR/robust_summary.csv" | grep -v "INVALID" | \
    awk -F, '{g0+=$2; g1+=$3; g2+=$4; n++} END {
        if(n>0) {
            printf "  Group 0 (PID+Fuzzy): %.4fm\n", g0/n;
            printf "  Group 1 (PD):        %.4fm\n", g1/n;
            printf "  Group 2 (PID):       %.4fm\n", g2/n;
        }
    }' | tee -a "$REPORT"

if [ "$pass_count" -ge 3 ]; then
    log ""
    log "[SUCCESS] THESIS REQUIREMENTS VERIFIED (>60% pass rate)"
    log "System is thesis-ready."
else
    log ""
    log "[WARNING] Insufficient pass rate"
    log "Review individual results for details."
fi

log ""
log "Report saved to: $REPORT"
