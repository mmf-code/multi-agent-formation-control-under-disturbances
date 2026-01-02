#!/bin/bash
export LC_ALL=C

LOG="tuning_results/baseline.log"

echo "=== BASELINE SUMMARY ==="

echo "Group 0 (PID+Fuzzy):"
g0=$(grep "RMSE=" "$LOG" | grep -E "agent_[012]\." | tail -15 | sed 's/.*RMSE=\([0-9.]*\)m.*/\1/' | awk '{sum+=$1; count++} END {if(count>0) printf "%.3f", sum/count; else print "N/A"}')
echo "  Avg RMSE: ${g0}m"

echo "Group 1 (PD):"
g1=$(grep "RMSE=" "$LOG" | grep -E "agent_[345]\." | tail -15 | sed 's/.*RMSE=\([0-9.]*\)m.*/\1/' | awk '{sum+=$1; count++} END {if(count>0) printf "%.3f", sum/count; else print "N/A"}')
echo "  Avg RMSE: ${g1}m"

echo "Group 2 (PID):"
g2=$(grep "RMSE=" "$LOG" | grep -E "agent_[678]\." | tail -15 | sed 's/.*RMSE=\([0-9.]*\)m.*/\1/' | awk '{sum+=$1; count++} END {if(count>0) printf "%.3f", sum/count; else print "N/A"}')
echo "  Avg RMSE: ${g2}m"

echo ""
echo "Thesis Check:"
echo "  PID < PD: $(echo "$g2 < $g1" | bc) (${g2} vs ${g1})"
echo "  Fuzzy < PID: $(echo "$g0 < $g2" | bc) (${g0} vs ${g2})"
