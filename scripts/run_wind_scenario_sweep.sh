#!/bin/bash
# Wind Scenario Sweep for Systematic Testing
#
# Runs all 12 standard wind scenarios and compares controller performance.
# Generates thesis-ready data and figures.
#
# Usage:
#   ./scripts/run_wind_scenario_sweep.sh [duration_per_scenario]
#   ./scripts/run_wind_scenario_sweep.sh 180  # 3 minutes each

set -e

# Configuration
DURATION=${1:-180}  # Default: 3 minutes per scenario
REPEATS=${2:-3}     # Number of repetitions for statistical significance
OUTPUT_DIR="thesis_data/wind_scenario_sweep"

# Scenarios from wind_scenarios.py
SCENARIOS=(
    "indoor_baseline"
    "calm_outdoor"
    "low_wind_low_turbulence"
    "moderate_wind_moderate_turbulence"
    "high_wind_high_turbulence"
    "extreme_gusts"
    "urban_canyon"
    "forest_canopy"
    "thesis_baseline"
    "thesis_stress"
)

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${BLUE}============================================"
echo "Wind Scenario Sweep"
echo "============================================"
echo -e "Scenarios: ${#SCENARIOS[@]}"
echo -e "Duration per scenario: ${DURATION}s"
echo -e "Repeats: ${REPEATS}"
echo -e "Total time: ~$((${#SCENARIOS[@]} * DURATION * REPEATS / 60)) minutes"
echo -e "============================================${NC}\n"

# Check prerequisites
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}Error: ROS2 not sourced!${NC}"
    exit 1
fi

source install/setup.bash

# Create output directory
mkdir -p "$OUTPUT_DIR"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
SWEEP_DIR="$OUTPUT_DIR/sweep_${TIMESTAMP}"
mkdir -p "$SWEEP_DIR"

# Log file
SUMMARY_LOG="$SWEEP_DIR/sweep_summary.txt"
echo "Wind Scenario Sweep - $TIMESTAMP" > "$SUMMARY_LOG"
echo "================================" >> "$SUMMARY_LOG"

# Total progress
TOTAL_RUNS=$((${#SCENARIOS[@]} * REPEATS))
CURRENT_RUN=0

# Run each scenario
for scenario in "${SCENARIOS[@]}"; do
    echo -e "\n${BLUE}=========================================="
    echo "Scenario: $scenario"
    echo -e "==========================================${NC}"

    # Get scenario parameters
    SCENARIO_INFO=$(python3 scripts/wind_scenarios.py --scenario "$scenario" --print-params 2>/dev/null || echo "")

    echo "$SCENARIO_INFO"

    # Create scenario directory
    SCENARIO_DIR="$SWEEP_DIR/$scenario"
    mkdir -p "$SCENARIO_DIR"

    # Repeat for statistical significance
    for repeat in $(seq 1 $REPEATS); do
        CURRENT_RUN=$((CURRENT_RUN + 1))
        echo -e "\n${YELLOW}Run $repeat/$REPEATS (Overall: $CURRENT_RUN/$TOTAL_RUNS)${NC}"

        # Run simulation
        echo "Starting simulation..."
        RUN_DIR="$SCENARIO_DIR/run_$repeat"
        mkdir -p "$RUN_DIR"

        # Launch with timeout
        timeout ${DURATION}s ros2 launch agent_control_pkg formation_comparison_demo.launch.py \
            gazebo_gui:=false \
            rviz:=false \
            wind_profile:=vonkarman \
            turbulence_intensity:=0.15 \
            2>&1 | tee "$RUN_DIR/simulation.log" || true

        # Move generated data
        if [ -d "thesis_data" ]; then
            LATEST_CSV=$(find thesis_data -name "metrics_*.csv" -type f -mmin -5 | head -1)
            if [ -n "$LATEST_CSV" ]; then
                cp "$LATEST_CSV" "$RUN_DIR/metrics.csv"

                # Calculate KPIs
                python3 scripts/wind_kpi_calculator.py "$RUN_DIR/metrics.csv" \
                    --output "$RUN_DIR/kpis.json" \
                    --plot

                echo -e "${GREEN}✓ Run $repeat complete${NC}"

                # Log to summary
                echo "[$scenario] Run $repeat: $(grep -o '"DRR": [0-9.]*' $RUN_DIR/kpis.json | cut -d' ' -f2)" >> "$SUMMARY_LOG"
            else
                echo -e "${RED}✗ No metrics CSV found for run $repeat${NC}"
            fi
        fi

        # Cool-down between runs
        sleep 3
    done

    echo -e "${GREEN}✓ Scenario $scenario complete${NC}"
done

# Generate comparison report
echo -e "\n${BLUE}=========================================="
echo "Generating Comparison Report"
echo -e "==========================================${NC}"

# Aggregate all KPIs
python3 << EOF
import json
import pandas as pd
from pathlib import Path

sweep_dir = Path("$SWEEP_DIR")
all_kpis = []

for scenario_dir in sweep_dir.glob("*/"):
    scenario = scenario_dir.name
    for run_dir in scenario_dir.glob("run_*"):
        kpi_file = run_dir / "kpis.json"
        if kpi_file.exists():
            with open(kpi_file) as f:
                kpis = json.load(f)
                kpis['scenario'] = scenario
                kpis['run'] = int(run_dir.name.split('_')[1])
                all_kpis.append(kpis)

df = pd.DataFrame(all_kpis)
df.to_csv(sweep_dir / "aggregated_kpis.csv", index=False)

# Summary statistics
summary = df.groupby('scenario').agg({
    'DRR': ['mean', 'std'],
    'WNRMSE': ['mean', 'std'],
    'mean_rmse': ['mean', 'std'],
    'mean_wind_magnitude': 'mean'
}).round(3)

print("\nSummary Statistics:")
print(summary)

summary.to_csv(sweep_dir / "summary_statistics.csv")
print(f"\n✓ Aggregated results saved to: {sweep_dir}/aggregated_kpis.csv")
EOF

# Generate comparison plots
echo -e "\n${BLUE}Generating comparison plots...${NC}"

python3 << EOF
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path

sweep_dir = Path("$SWEEP_DIR")
df = pd.read_csv(sweep_dir / "aggregated_kpis.csv")

# Set style
sns.set_style("whitegrid")

# Figure 1: DRR comparison across scenarios
fig, axes = plt.subplots(2, 2, figsize=(14, 10))

# Plot 1: DRR by scenario
ax = axes[0, 0]
scenario_order = sorted(df['scenario'].unique())
sns.boxplot(data=df, x='scenario', y='DRR', ax=ax, order=scenario_order)
ax.axhline(y=0.5, color='r', linestyle='--', label='Target (< 0.5)')
ax.set_xticklabels(ax.get_xticklabels(), rotation=45, ha='right')
ax.set_title('Disturbance Rejection Ratio (DRR) by Scenario', fontweight='bold')
ax.set_ylabel('DRR [-]')
ax.legend()
ax.grid(True, alpha=0.3)

# Plot 2: WNRMSE by scenario
ax = axes[0, 1]
sns.boxplot(data=df, x='scenario', y='WNRMSE', ax=ax, order=scenario_order)
ax.axhline(y=0.8, color='r', linestyle='--', label='Target (< 0.8)')
ax.set_xticklabels(ax.get_xticklabels(), rotation=45, ha='right')
ax.set_title('Wind-Normalized RMSE by Scenario', fontweight='bold')
ax.set_ylabel('WNRMSE [-]')
ax.legend()
ax.grid(True, alpha=0.3)

# Plot 3: DRR vs Wind Magnitude
ax = axes[1, 0]
sns.scatterplot(data=df, x='mean_wind_magnitude', y='DRR', hue='scenario', ax=ax, s=100)
ax.set_xlabel('Mean Wind Magnitude [m/s]')
ax.set_ylabel('DRR [-]')
ax.set_title('DRR vs Wind Magnitude', fontweight='bold')
ax.grid(True, alpha=0.3)
ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left', fontsize=8)

# Plot 4: Performance summary table
ax = axes[1, 1]
ax.axis('off')

summary_stats = df.groupby('scenario').agg({
    'DRR': 'mean',
    'WNRMSE': 'mean',
    'mean_wind_magnitude': 'mean'
}).round(3)

table_data = summary_stats.head(5).values
row_labels = summary_stats.head(5).index
col_labels = ['DRR', 'WNRMSE', 'Wind [m/s]']

table = ax.table(cellText=table_data, rowLabels=row_labels, colLabels=col_labels,
                 cellLoc='center', loc='center', bbox=[0, 0, 1, 1])
table.auto_set_font_size(False)
table.set_fontsize(9)
ax.set_title('Top 5 Scenarios - Mean Performance', fontweight='bold', pad=20)

plt.tight_layout()
output_file = sweep_dir / "wind_scenario_comparison.png"
plt.savefig(output_file, dpi=300, bbox_inches='tight')
print(f"✓ Saved comparison plot: {output_file}")
EOF

# Final summary
echo -e "\n${GREEN}=========================================="
echo "Sweep Complete!"
echo "==========================================${NC}"
echo -e "Total runs: $TOTAL_RUNS"
echo -e "Output directory: $SWEEP_DIR"
echo -e "Files generated:"
echo -e "  - aggregated_kpis.csv"
echo -e "  - summary_statistics.csv"
echo -e "  - wind_scenario_comparison.png"
echo -e "==========================================\n"

cat "$SUMMARY_LOG"

echo -e "\n${BLUE}To view results:${NC}"
echo -e "  cat $SWEEP_DIR/summary_statistics.csv"
echo -e "  xdg-open $SWEEP_DIR/wind_scenario_comparison.png"

exit 0
