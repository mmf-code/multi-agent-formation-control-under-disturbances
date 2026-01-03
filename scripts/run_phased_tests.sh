#!/bin/bash
#
# Run Phased Test Framework for Thesis Validation
#
# Usage:
#   ./scripts/run_phased_tests.sh              # Run all phases once
#   ./scripts/run_phased_tests.sh --phase 3    # Run only Phase 3
#   ./scripts/run_phased_tests.sh --runs 5     # Run all phases 5 times each
#   ./scripts/run_phased_tests.sh --report     # Only generate report from existing results
#
# Output:
#   results/phase_{id}/run_{k}/  - Per-run CSV data
#   docs/PHASED_TEST_RESULTS.md  - Consolidated markdown report

set -e

# Default parameters
PHASE=""           # Empty = run all phases
RUNS=1             # Number of runs per phase
OUTPUT_DIR="results"
REPORT_ONLY=false
SEED=-1            # -1 = random

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --phase|-p)
            PHASE="$2"
            shift 2
            ;;
        --runs|-r)
            RUNS="$2"
            shift 2
            ;;
        --output|-o)
            OUTPUT_DIR="$2"
            shift 2
            ;;
        --seed|-s)
            SEED="$2"
            shift 2
            ;;
        --report)
            REPORT_ONLY=true
            shift
            ;;
        --help|-h)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --phase, -p N     Run only phase N (1-6)"
            echo "  --runs, -r N      Number of runs per phase (default: 1)"
            echo "  --output, -o DIR  Output directory (default: results)"
            echo "  --seed, -s N      Random seed (-1 for random)"
            echo "  --report          Only generate report from existing results"
            echo "  --help, -h        Show this help"
            echo ""
            echo "Phases:"
            echo "  1: BASELINE      - No wind (reference)"
            echo "  2: STEADY_WIND   - Constant 3 m/s @ 45 deg"
            echo "  3: TURBULENCE    - Von Karman TI sweep [0.15, 0.25, 0.35]"
            echo "  4: GUST          - Periodic gusts"
            echo "  5: COMBINED      - Stochastic (turbulence + gusts)"
            echo "  6: ENDURANCE     - Long-run 300s"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

# Source ROS2 if not already sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "Sourcing ROS2 Humble..."
    source /opt/ros/humble/setup.bash
fi

# Source workspace if available
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"
if [ -f "$WS_DIR/install/setup.bash" ]; then
    echo "Sourcing workspace..."
    source "$WS_DIR/install/setup.bash"
fi

# Create output directory
mkdir -p "$OUTPUT_DIR"

echo "=============================================="
echo " PHASED TEST FRAMEWORK"
echo "=============================================="
echo " Output:  $OUTPUT_DIR"
echo " Runs:    $RUNS per phase"
echo " Seed:    $SEED"
echo "=============================================="

# Report only mode
if [ "$REPORT_ONLY" = true ]; then
    echo ""
    echo "Generating report from existing results..."
    python3 "$WS_DIR/agent_control_pkg/scripts/phased_report.py" \
        --input "$OUTPUT_DIR" \
        --output docs/PHASED_TEST_RESULTS.md
    echo ""
    echo "Report complete: docs/PHASED_TEST_RESULTS.md"
    exit 0
fi

# Determine phases to run
if [ -n "$PHASE" ]; then
    PHASES=("$PHASE")
else
    PHASES=(1 2 3 4 5 6)
fi

# Phase 3 has TI sweep
PHASE3_SWEEPS=(0 1 2)  # TI = 0.15, 0.25, 0.35

# Run each phase
for phase_id in "${PHASES[@]}"; do
    echo ""
    echo "=============================================="
    echo " PHASE $phase_id"
    echo "=============================================="

    # Handle Phase 3 TI sweep
    if [ "$phase_id" -eq 3 ]; then
        for sweep_idx in "${PHASE3_SWEEPS[@]}"; do
            for ((run=1; run<=RUNS; run++)); do
                echo ""
                echo "--- Phase 3, TI sweep $sweep_idx, Run $run ---"

                # Calculate seed for this run
                if [ "$SEED" -eq -1 ]; then
                    RUN_SEED=$((RANDOM))
                else
                    RUN_SEED=$((SEED + run + sweep_idx * 100))
                fi

                # Run the phase
                timeout 120 ros2 launch agent_control_pkg phased_comparison.launch.py \
                    phase:=3 \
                    sweep_index:="$sweep_idx" \
                    run_index:="$run" \
                    seed:="$RUN_SEED" \
                    output_dir:="$OUTPUT_DIR" \
                    gazebo_gui:=false \
                    2>&1 | tee -a "$OUTPUT_DIR/phase_3_ti${sweep_idx}_run${run}.log" || {
                        echo "Phase 3 TI$sweep_idx Run $run completed or timed out"
                    }

                # Small delay between runs
                sleep 2
            done
        done
    else
        # Regular phases
        for ((run=1; run<=RUNS; run++)); do
            echo ""
            echo "--- Phase $phase_id, Run $run ---"

            # Calculate seed
            if [ "$SEED" -eq -1 ]; then
                RUN_SEED=$((RANDOM))
            else
                RUN_SEED=$((SEED + run + phase_id * 1000))
            fi

            # Determine timeout based on phase duration
            if [ "$phase_id" -eq 6 ]; then
                TIMEOUT=360  # 6 minutes for 300s phase
            else
                TIMEOUT=120  # 2 minutes for 60s phases
            fi

            # Run the phase
            timeout $TIMEOUT ros2 launch agent_control_pkg phased_comparison.launch.py \
                phase:="$phase_id" \
                run_index:="$run" \
                seed:="$RUN_SEED" \
                output_dir:="$OUTPUT_DIR" \
                gazebo_gui:=false \
                2>&1 | tee -a "$OUTPUT_DIR/phase_${phase_id}_run${run}.log" || {
                    echo "Phase $phase_id Run $run completed or timed out"
                }

            # Small delay between runs
            sleep 2
        done
    fi
done

echo ""
echo "=============================================="
echo " ALL PHASES COMPLETE"
echo "=============================================="

# Generate final report
echo ""
echo "Generating consolidated report..."
python3 "$WS_DIR/agent_control_pkg/scripts/phased_report.py" \
    --input "$OUTPUT_DIR" \
    --output docs/PHASED_TEST_RESULTS.md

echo ""
echo "=============================================="
echo " RESULTS"
echo "=============================================="
echo " Data:   $OUTPUT_DIR/"
echo " Report: docs/PHASED_TEST_RESULTS.md"
echo "=============================================="
