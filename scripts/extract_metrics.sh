#!/bin/bash
# Extract metrics from simulation log

LOG_FILE="${1:-tuning_results/baseline.log}"

echo "=== METRICS FROM: $LOG_FILE ==="
echo ""

for group in 0 1 2; do
    case $group in
        0) echo "Group 0 (PID+Fuzzy):" ;;
        1) echo "Group 1 (PD):" ;;
        2) echo "Group 2 (PID):" ;;
    esac

    start=$((group * 3))
    total_rmse=0
    count=0

    for agent in $start $((start+1)) $((start+2)); do
        rmse_vals=$(grep "agent_${agent}.metrics_publisher" "$LOG_FILE" 2>/dev/null | grep "RMSE=" | tail -10 | sed 's/.*RMSE=\([0-9.]*\)m.*/\1/')
        if [ -n "$rmse_vals" ]; then
            avg=$(echo "$rmse_vals" | awk '{sum+=$1; count++} END {if(count>0) printf "%.3f", sum/count}')
            echo "  Agent $agent: avg_RMSE=${avg}m"
            total_rmse=$(echo "$total_rmse + $avg" | bc)
            count=$((count + 1))
        fi
    done

    if [ $count -gt 0 ]; then
        group_avg=$(echo "scale=3; $total_rmse / $count" | bc)
        echo "  >> Group Average: ${group_avg}m"
    fi
    echo ""
done
