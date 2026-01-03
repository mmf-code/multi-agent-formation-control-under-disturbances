#!/usr/bin/env python3
"""
Phased Report Generator for 6-Phase Thesis Test Framework

Reads all phase CSV results and generates:
- Consolidated summary tables
- Performance improvement vs PD baseline per phase
- Markdown thesis-ready summary document
- Optional visualization plots

Usage:
    python3 phased_report.py --input results --output docs/PHASED_TEST_RESULTS.md

    # With plots
    python3 phased_report.py --input results --output docs/PHASED_TEST_RESULTS.md --plots
"""

import os
import sys
import csv
import json
import argparse
import statistics
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Any
from datetime import datetime


@dataclass
class PhaseResult:
    """Results from a single phase run."""
    phase_id: int
    phase_name: str
    run_index: int
    duration_sec: float
    groups: Dict[str, Dict[str, float]] = field(default_factory=dict)
    wind_stats: Dict[str, float] = field(default_factory=dict)


@dataclass
class ControllerStats:
    """Aggregated statistics for a controller type."""
    controller_type: str
    rmse_mean: float = 0.0
    rmse_std: float = 0.0
    itae_mean: float = 0.0
    itae_std: float = 0.0
    settling_mean: float = 0.0
    settling_std: float = 0.0
    overshoot_mean: float = 0.0
    improvement_vs_pd: float = 0.0  # Percentage improvement in RMSE vs PD


PHASE_NAMES = {
    1: "BASELINE",
    2: "STEADY_WIND",
    3: "TURBULENCE",
    4: "GUST",
    5: "COMBINED",
    6: "ENDURANCE",
}


def load_group_summary(filepath: str) -> Dict[str, Dict[str, float]]:
    """Load group_summary.csv and return dict of controller -> metrics."""
    groups = {}
    if not os.path.exists(filepath):
        return groups

    with open(filepath, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            ctrl = row.get('controller_type', 'UNKNOWN')
            groups[ctrl] = {
                'mean_rmse': float(row.get('mean_rmse', 0)),
                'std_rmse': float(row.get('std_rmse', 0)),
                'mean_itae': float(row.get('mean_itae', 0)),
                'std_itae': float(row.get('std_itae', 0)),
                'mean_settling_time': float(row.get('mean_settling_time', 0)),
                'mean_max_overshoot': float(row.get('mean_max_overshoot', 0)),
                'agents_settled': int(row.get('agents_settled', 0)),
                'agent_count': int(row.get('agent_count', 0)),
            }
    return groups


def load_phase_metadata(filepath: str) -> Dict[str, Any]:
    """Load phase_metadata.json."""
    if not os.path.exists(filepath):
        return {}

    with open(filepath, 'r') as f:
        return json.load(f)


def scan_results_dir(results_dir: str) -> List[PhaseResult]:
    """Scan results directory and load all phase results."""
    results = []

    if not os.path.exists(results_dir):
        print(f"Results directory not found: {results_dir}")
        return results

    for phase_dir in sorted(os.listdir(results_dir)):
        if not phase_dir.startswith('phase_'):
            continue

        phase_path = os.path.join(results_dir, phase_dir)
        if not os.path.isdir(phase_path):
            continue

        # Extract phase ID
        try:
            phase_id = int(phase_dir.split('_')[1])
        except (IndexError, ValueError):
            continue

        # Scan run directories
        for run_dir in sorted(os.listdir(phase_path)):
            if not run_dir.startswith('run_'):
                continue

            run_path = os.path.join(phase_path, run_dir)
            if not os.path.isdir(run_path):
                continue

            # Extract run index
            try:
                run_index = int(run_dir.split('_')[1])
            except (IndexError, ValueError):
                continue

            # Load group summary
            summary_file = os.path.join(run_path, 'group_summary.csv')
            groups = load_group_summary(summary_file)

            # Load metadata
            metadata_file = os.path.join(run_path, 'phase_metadata.json')
            metadata = load_phase_metadata(metadata_file)

            if groups:
                result = PhaseResult(
                    phase_id=phase_id,
                    phase_name=PHASE_NAMES.get(phase_id, f"PHASE_{phase_id}"),
                    run_index=run_index,
                    duration_sec=metadata.get('duration_sec', 60),
                    groups=groups,
                )
                results.append(result)
                print(f"Loaded: {phase_dir}/{run_dir} - {len(groups)} groups")

    return results


def compute_phase_summary(results: List[PhaseResult]) -> Dict[int, Dict[str, ControllerStats]]:
    """Aggregate results by phase and compute statistics."""
    phase_data: Dict[int, Dict[str, List[Dict]]] = {}

    # Group by phase
    for r in results:
        if r.phase_id not in phase_data:
            phase_data[r.phase_id] = {}

        for ctrl, metrics in r.groups.items():
            if ctrl not in phase_data[r.phase_id]:
                phase_data[r.phase_id][ctrl] = []
            phase_data[r.phase_id][ctrl].append(metrics)

    # Compute statistics
    summaries: Dict[int, Dict[str, ControllerStats]] = {}

    for phase_id, ctrl_data in phase_data.items():
        summaries[phase_id] = {}

        # Get PD baseline for improvement calculation
        pd_rmse = None
        if 'PD' in ctrl_data and ctrl_data['PD']:
            pd_rmses = [m['mean_rmse'] for m in ctrl_data['PD']]
            pd_rmse = statistics.mean(pd_rmses) if pd_rmses else None

        for ctrl, metrics_list in ctrl_data.items():
            if not metrics_list:
                continue

            rmses = [m['mean_rmse'] for m in metrics_list]
            itaes = [m['mean_itae'] for m in metrics_list]
            settlings = [m['mean_settling_time'] for m in metrics_list]
            overshoots = [m['mean_max_overshoot'] for m in metrics_list]

            stats = ControllerStats(
                controller_type=ctrl,
                rmse_mean=statistics.mean(rmses) if rmses else 0,
                rmse_std=statistics.stdev(rmses) if len(rmses) > 1 else 0,
                itae_mean=statistics.mean(itaes) if itaes else 0,
                itae_std=statistics.stdev(itaes) if len(itaes) > 1 else 0,
                settling_mean=statistics.mean(settlings) if settlings else 0,
                settling_std=statistics.stdev(settlings) if len(settlings) > 1 else 0,
                overshoot_mean=statistics.mean(overshoots) if overshoots else 0,
            )

            # Improvement vs PD
            if pd_rmse and pd_rmse > 0:
                stats.improvement_vs_pd = ((pd_rmse - stats.rmse_mean) / pd_rmse) * 100

            summaries[phase_id][ctrl] = stats

    return summaries


def generate_markdown_report(
    summaries: Dict[int, Dict[str, ControllerStats]],
    results: List[PhaseResult],
    output_path: str
):
    """Generate thesis-ready markdown report."""

    lines = []
    lines.append("# Phased Test Framework Results")
    lines.append("")
    lines.append(f"*Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}*")
    lines.append("")
    lines.append("## Overview")
    lines.append("")
    lines.append("This report summarizes the 6-phase controller comparison test results.")
    lines.append("")
    lines.append("**Controllers Tested:**")
    lines.append("- **PD**: Proportional-Derivative (baseline)")
    lines.append("- **PID**: Proportional-Integral-Derivative")
    lines.append("- **IT2**: PID + Interval Type-2 Fuzzy Logic System")
    lines.append("- **GT2**: PID + General Type-2 Fuzzy Logic System")
    lines.append("")

    # Phase descriptions
    lines.append("## Test Phases")
    lines.append("")
    lines.append("| Phase | Name | Purpose | Duration |")
    lines.append("|-------|------|---------|----------|")
    lines.append("| 1 | BASELINE | Reference (no wind) | 60s |")
    lines.append("| 2 | STEADY_WIND | DC rejection (3 m/s @ 45 deg) | 60s |")
    lines.append("| 3 | TURBULENCE | Von Karman TI sweep [0.15, 0.25, 0.35] | 60s |")
    lines.append("| 4 | GUST | Periodic gusts (5 m/s, 1.5s, 10s interval) | 60s |")
    lines.append("| 5 | COMBINED | Stochastic: turbulence + gusts + wander | 60s |")
    lines.append("| 6 | ENDURANCE | Long-run stability | 300s |")
    lines.append("")

    # Results per phase
    lines.append("## Results by Phase")
    lines.append("")

    for phase_id in sorted(summaries.keys()):
        phase_name = PHASE_NAMES.get(phase_id, f"PHASE_{phase_id}")
        phase_stats = summaries[phase_id]

        lines.append(f"### Phase {phase_id}: {phase_name}")
        lines.append("")
        lines.append("| Controller | RMSE (m) | ITAE | Settling (s) | Overshoot (m) | vs PD |")
        lines.append("|------------|----------|------|--------------|---------------|-------|")

        for ctrl in ["PD", "PID", "IT2", "GT2"]:
            if ctrl not in phase_stats:
                continue
            s = phase_stats[ctrl]
            improvement = f"+{s.improvement_vs_pd:.1f}%" if s.improvement_vs_pd > 0 else f"{s.improvement_vs_pd:.1f}%"
            if ctrl == "PD":
                improvement = "baseline"
            lines.append(
                f"| {ctrl} | {s.rmse_mean:.4f} +- {s.rmse_std:.4f} | "
                f"{s.itae_mean:.2f} +- {s.itae_std:.2f} | "
                f"{s.settling_mean:.2f} | {s.overshoot_mean:.4f} | {improvement} |"
            )

        lines.append("")

    # Overall summary
    lines.append("## Overall Performance Summary")
    lines.append("")

    # Find best performer per phase
    lines.append("### Best Performer by Phase")
    lines.append("")
    lines.append("| Phase | Best RMSE | Best ITAE | Best Settling |")
    lines.append("|-------|-----------|-----------|---------------|")

    for phase_id in sorted(summaries.keys()):
        phase_stats = summaries[phase_id]
        if not phase_stats:
            continue

        best_rmse = min(phase_stats.items(), key=lambda x: x[1].rmse_mean)
        best_itae = min(phase_stats.items(), key=lambda x: x[1].itae_mean)
        best_settling = min(phase_stats.items(), key=lambda x: x[1].settling_mean)

        lines.append(
            f"| {phase_id} | {best_rmse[0]} ({best_rmse[1].rmse_mean:.4f}) | "
            f"{best_itae[0]} ({best_itae[1].itae_mean:.2f}) | "
            f"{best_settling[0]} ({best_settling[1].settling_mean:.2f}s) |"
        )

    lines.append("")

    # Conclusions
    lines.append("## Conclusions")
    lines.append("")
    lines.append("*TODO: Add analysis conclusions based on results*")
    lines.append("")
    lines.append("### Key Findings")
    lines.append("")
    lines.append("1. **Baseline Performance**: [Analysis of Phase 1 results]")
    lines.append("2. **Steady Wind**: [IT2/GT2 advantage over PID/PD]")
    lines.append("3. **Turbulence**: [Effect of TI on controller ranking]")
    lines.append("4. **Gust Recovery**: [Transient response comparison]")
    lines.append("5. **Combined Disturbance**: [Realistic scenario results]")
    lines.append("6. **Endurance**: [Long-term stability assessment]")
    lines.append("")

    # Limitations
    lines.append("## Limitations")
    lines.append("")
    lines.append("- Simulation-based results (Gazebo ODE physics)")
    lines.append("- Single random seed per run (consider multiple seeds)")
    lines.append("- Fixed PID gains across all controllers")
    lines.append("")

    # Validation checks
    lines.append("## Validation Checks")
    lines.append("")
    lines.append("- [ ] Wind topic active with expected dynamics")
    lines.append("- [ ] GT2 controllers receiving normalized wind [0,1]")
    lines.append("- [ ] IT2 controllers receiving signed wind [-10,10] m/s")
    lines.append("- [ ] Metrics windows aligned with phase boundaries")
    lines.append("")

    # Write to file
    os.makedirs(os.path.dirname(output_path) if os.path.dirname(output_path) else '.', exist_ok=True)
    with open(output_path, 'w') as f:
        f.write('\n'.join(lines))

    print(f"\nReport generated: {output_path}")


def main():
    parser = argparse.ArgumentParser(description='Generate phased test framework report')
    parser.add_argument('--input', '-i', default='results',
                        help='Input results directory')
    parser.add_argument('--output', '-o', default='docs/PHASED_TEST_RESULTS.md',
                        help='Output markdown file')
    parser.add_argument('--plots', action='store_true',
                        help='Generate visualization plots (requires matplotlib)')

    args = parser.parse_args()

    print(f"Scanning results directory: {args.input}")
    results = scan_results_dir(args.input)

    if not results:
        print("No results found. Run phased tests first:")
        print("  ros2 launch agent_control_pkg phased_comparison.launch.py phase:=1")
        return 1

    print(f"\nLoaded {len(results)} phase runs")

    # Compute summaries
    summaries = compute_phase_summary(results)
    print(f"Computed summaries for {len(summaries)} phases")

    # Generate report
    generate_markdown_report(summaries, results, args.output)

    # Generate plots if requested
    if args.plots:
        try:
            import matplotlib.pyplot as plt
            print("\nPlot generation not yet implemented")
        except ImportError:
            print("\nMatplotlib not available for plots")

    return 0


if __name__ == "__main__":
    sys.exit(main())
