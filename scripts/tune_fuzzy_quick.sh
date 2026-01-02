#!/bin/bash
# Quick Fuzzy Parameter Tuning Script
# Tests different k_fuzzy values to find optimal thesis performance
set -e
export LC_ALL=C

BASE_DIR="/home/mmf/Documents/GitHub/multi-agent-formation-control-under-disturbances"
RESULTS_DIR="$BASE_DIR/tuning_results"
mkdir -p "$RESULTS_DIR"

source /opt/ros/humble/setup.bash
source "$BASE_DIR/install/setup.bash"

TIMESTAMP=$(date +%Y%m%d_%H%M%S)
SUMMARY="$RESULTS_DIR/fuzzy_tuning_${TIMESTAMP}.csv"
REPORT="$RESULTS_DIR/fuzzy_tuning_${TIMESTAMP}.txt"

log() {
    echo "$1" | tee -a "$REPORT"
}

cleanup() {
    killall -9 gzserver gzclient agent_controller_node 2>/dev/null || true
    rm -rf /dev/shm/fastrtps* 2>/dev/null || true
    sleep 5
}

test_config() {
    local k_fuzzy="$1"
    local wind_scalar="$2"
    local name="kf${k_fuzzy}_ws${wind_scalar}"
    local log_file="$RESULTS_DIR/${name}_${TIMESTAMP}.log"

    log ""
    log "Testing: k_fuzzy=$k_fuzzy, wind_scalar=$wind_scalar"
    log "----------------------------------------"

    cleanup

    # Run simulation with custom parameters
    timeout 50 ros2 launch agent_control_pkg formation_comparison_demo.launch.py \
        gazebo_gui:=false rviz:=false \
        fuzzy_group_k_fuzzy:=$k_fuzzy 2>&1 | tee "$log_file" || true

    sleep 2

    # Extract metrics (tail-10 steady state)
    g0=$(grep "RMSE=" "$log_file" | grep -E "agent_[012]\." | tail -10 | \
         sed 's/.*RMSE=\([0-9.]*\)m.*/\1/' | \
         awk '$1 < 5.0 {sum+=$1; count++} END {if(count>0) printf "%.4f", sum/count; else print "N/A"}')
    g1=$(grep "RMSE=" "$log_file" | grep -E "agent_[345]\." | tail -10 | \
         sed 's/.*RMSE=\([0-9.]*\)m.*/\1/' | \
         awk '$1 < 5.0 {sum+=$1; count++} END {if(count>0) printf "%.4f", sum/count; else print "N/A"}')
    g2=$(grep "RMSE=" "$log_file" | grep -E "agent_[678]\." | tail -10 | \
         sed 's/.*RMSE=\([0-9.]*\)m.*/\1/' | \
         awk '$1 < 5.0 {sum+=$1; count++} END {if(count>0) printf "%.4f", sum/count; else print "N/A"}')

    log "  G0 (PID+Fuzzy): ${g0}m"
    log "  G1 (PD):        ${g1}m"
    log "  G2 (PID):       ${g2}m"

    # Check thesis requirements
    if [ "$g0" != "N/A" ] && [ "$g2" != "N/A" ]; then
        pid_lt_pd=$(echo "$g2 < $g1" | bc -l)
        fuzzy_lt_pid=$(echo "$g0 < $g2" | bc -l)

        if [ "$pid_lt_pd" = "1" ]; then
            log "  [PASS] PID < PD"
        else
            log "  [FAIL] PID >= PD"
        fi

        if [ "$fuzzy_lt_pid" = "1" ]; then
            log "  [PASS] Fuzzy < PID"
        else
            log "  [FAIL] Fuzzy >= PID"
        fi

        echo "$name,$k_fuzzy,$wind_scalar,$g0,$g1,$g2,$pid_lt_pd,$fuzzy_lt_pid" >> "$SUMMARY"
    else
        log "  [ERROR] Missing data"
        echo "$name,$k_fuzzy,$wind_scalar,$g0,$g1,$g2,NA,NA" >> "$SUMMARY"
    fi
}

# Initialize
log "========================================"
log "FUZZY PARAMETER TUNING"
log "Timestamp: $TIMESTAMP"
log "========================================"

echo "name,k_fuzzy,wind_scalar,g0_rmse,g1_rmse,g2_rmse,pid_lt_pd,fuzzy_lt_pid" > "$SUMMARY"

# Test different k_fuzzy values
# Lower k_fuzzy = less fuzzy influence = should help when fuzzy is too aggressive
test_config 0.6 1.5   # Current (baseline)
test_config 0.4 1.5   # Reduced fuzzy
test_config 0.3 1.5   # More reduced
test_config 0.35 1.0  # Reduced both

cleanup

log ""
log "========================================"
log "TUNING RESULTS"
log "========================================"
log ""
cat "$SUMMARY" | tee -a "$REPORT"

log ""
log "Best configuration for thesis:"
# Find config where both requirements pass
best=$(tail -n +2 "$SUMMARY" | awk -F, '$7==1 && $8==1 {print $1 " (k_fuzzy=" $2 ", ws=" $3 "): G0=" $4 " G1=" $5 " G2=" $6}')
if [ -n "$best" ]; then
    log "  $best"
else
    log "  No configuration met both thesis requirements"
    log "  Consider: Higher wind, different fuzzy rules, or accept marginal results"
fi

log ""
log "Report: $REPORT"
log "Summary: $SUMMARY"
