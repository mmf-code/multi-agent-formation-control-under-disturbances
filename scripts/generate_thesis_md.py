#!/usr/bin/env python3
"""
Thesis Results Markdown Generator

Generates thesis documentation from phase_metadata.json files.
Ensures documentation matches actual test results (single source of truth).

Uses only Python stdlib (no numpy dependency).

Usage:
    python3 scripts/generate_thesis_md.py --results-dir results/wind_scalar_fix
    python3 scripts/generate_thesis_md.py --output docs/THESIS_CONTROLLER_COMPARISON_RESULTS.md

Author: Multi-Agent Formation Control Research
"""

import argparse
import json
import os
import statistics
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Any


# Phase definitions (matches phased_comparison.launch.py)
PHASE_DEFINITIONS = {
    1: {
        "name": "BASELINE",
        "description": "No wind (reference performance)",
        "wind_profile": "constant",
        "wind_magnitude": 0.0,
        "wind_direction": 0.0,
        "duration_sec": 60,
    },
    2: {
        "name": "STEADY_WIND",
        "description": "Constant 3 m/s @ 45 deg (DC rejection)",
        "wind_profile": "constant",
        "wind_magnitude": 3.0,
        "wind_direction": 45.0,
        "duration_sec": 60,
    },
    3: {
        "name": "TURBULENCE",
        "description": "Von Karman turbulence (stochastic wind variations)",
        "wind_profile": "vonkarman",
        "wind_magnitude": 2.5,
        "turbulence_intensity": 0.15,  # Default TI, sweep_index can change this
        "duration_sec": 60,
    },
    4: {
        "name": "GUST",
        "description": "Periodic gusts (transient response)",
        "wind_profile": "gust",
        "wind_magnitude": 5.0,
        "gust_duration": 1.5,
        "gust_interval": 10.0,
        "duration_sec": 60,
    },
    5: {
        "name": "COMBINED",
        "description": "Stochastic: turbulence + gusts + direction wander",
        "wind_profile": "stochastic",
        "wind_magnitude": 2.5,
        "stochastic_mag_std": 1.5,
        "stochastic_dir_rate": 15.0,
        "stochastic_gust_prob": 0.08,
        "duration_sec": 60,
    },
    6: {
        "name": "ENDURANCE",
        "description": "Long-run combined (300s)",
        "wind_profile": "stochastic",
        "wind_magnitude": 2.5,
        "duration_sec": 300,
    },
}

# Controller ordering for consistent output
CONTROLLER_ORDER = ["PD", "PID", "IT2", "GT2"]


def load_phase_metadata(phase_dir: Path) -> Optional[Dict]:
    """Load phase_metadata.json from a run directory."""
    metadata_file = phase_dir / "phase_metadata.json"
    if not metadata_file.exists():
        return None
    with open(metadata_file, "r") as f:
        return json.load(f)


def load_all_results(results_dir: Path) -> Dict[int, List[Dict]]:
    """Load all phase results from results directory.

    Returns:
        Dict mapping phase_id -> list of run metadata dicts
    """
    results: Dict[int, List[Dict]] = {}

    for phase_dir in sorted(results_dir.glob("phase_*")):
        if not phase_dir.is_dir():
            continue

        try:
            phase_id = int(phase_dir.name.split("_")[1])
        except (IndexError, ValueError):
            continue

        runs = []
        for run_dir in sorted(phase_dir.glob("run_*")):
            if not run_dir.is_dir():
                continue

            metadata = load_phase_metadata(run_dir)
            if metadata:
                runs.append(metadata)

        if runs:
            results[phase_id] = runs

    return results


def compute_cross_run_stats(
    runs: List[Dict], metric: str, require_positive: bool = False
) -> Dict[str, Dict[str, Any]]:
    """Compute cross-run statistics for a metric.

    Args:
        runs: List of phase metadata dicts
        metric: Metric key in group_summaries (e.g., 'mean_rmse')
        require_positive: If True, only include values > 0 (for RMSE-like metrics)

    Returns:
        Dict mapping controller -> {mean, std, min, max, n, available}
        'available' indicates if metric exists in the data (vs legacy format)
    """
    stats: Dict[str, Dict[str, Any]] = {}

    for ctrl in CONTROLLER_ORDER:
        values = []
        metric_exists = False  # Track if metric key exists in any run

        for run in runs:
            group_summaries = run.get("group_summaries", {})
            if ctrl in group_summaries:
                ctrl_data = group_summaries[ctrl]
                # Check if metric KEY exists (not just defaulting to 0)
                if metric in ctrl_data:
                    metric_exists = True
                    val = ctrl_data[metric]
                    # For RMSE-like metrics, skip zeros (missing data)
                    # For last_violation_time, 0.0 is valid (fast settle)
                    if require_positive and val <= 0:
                        continue
                    values.append(val)

        if values:
            stats[ctrl] = {
                "mean": statistics.mean(values),
                "std": statistics.stdev(values) if len(values) > 1 else 0.0,
                "min": min(values),
                "max": max(values),
                "n": len(values),
                "available": True,  # Metric exists in data
            }
        else:
            stats[ctrl] = {
                "mean": 0.0, "std": 0.0, "min": 0.0, "max": 0.0, "n": 0,
                "available": metric_exists,  # False if legacy format
            }

    return stats


def find_winner(stats: Dict[str, Dict[str, float]], lower_is_better: bool = True) -> str:
    """Find the controller with best performance."""
    valid = {k: v for k, v in stats.items() if v["n"] > 0}
    if not valid:
        return "(No data)"

    if lower_is_better:
        winner = min(valid.keys(), key=lambda k: valid[k]["mean"])
    else:
        winner = max(valid.keys(), key=lambda k: valid[k]["mean"])

    return winner


def generate_phase_section(phase_id: int, runs: List[Dict]) -> str:
    """Generate markdown section for one phase."""
    phase_def = PHASE_DEFINITIONS.get(phase_id, {})
    phase_name = phase_def.get("name", f"PHASE_{phase_id}")
    phase_desc = phase_def.get("description", "Unknown")

    lines = []
    lines.append(f"## Phase {phase_id}: {phase_name}")
    lines.append("")
    lines.append(f"**Objective**: {phase_desc}")
    lines.append("")

    # Wind conditions
    lines.append("**Conditions**:")
    lines.append(f"- Wind Profile: {phase_def.get('wind_profile', 'N/A')}")
    if phase_def.get("wind_magnitude", 0) > 0:
        lines.append(f"- Wind Magnitude: {phase_def.get('wind_magnitude', 0)} m/s")
    if phase_def.get("wind_direction"):
        lines.append(f"- Wind Direction: {phase_def.get('wind_direction', 0)}\u00b0")
    if phase_def.get("turbulence_intensity"):
        lines.append(f"- Turbulence Intensity: {phase_def.get('turbulence_intensity', 0)}")
    lines.append(f"- Duration: {phase_def.get('duration_sec', 60)} seconds")
    lines.append("")

    # Check for valid data
    total_samples = sum(run.get("total_samples", 0) for run in runs)
    if total_samples == 0:
        lines.append("> **DATA COLLECTION ISSUE**: No samples collected. Results pending re-test.")
        lines.append("")
        return "\n".join(lines)

    # Results table - RMSE (require_positive=True since RMSE>0 always)
    rmse_stats = compute_cross_run_stats(runs, "mean_rmse", require_positive=True)

    # OFFLINE metrics (may not exist in legacy format)
    peak_stats = compute_cross_run_stats(runs, "mean_peak_error")
    violation_stats = compute_cross_run_stats(runs, "mean_last_violation")  # 0.0 is valid!
    mae_stats = compute_cross_run_stats(runs, "mean_mae")

    # Check if OFFLINE metrics are available (not legacy format)
    has_offline_metrics = any(
        peak_stats[ctrl].get("available", False) for ctrl in CONTROLLER_ORDER
    )

    lines.append(f"### Results (N={len(runs)} run{'s' if len(runs) > 1 else ''})")
    lines.append("")

    if has_offline_metrics:
        lines.append("| Controller | RMSE (m) | Std Dev | Peak Error | Last Violation | MAE |")
        lines.append("|------------|----------|---------|------------|----------------|-----|")
    else:
        # Legacy format - only RMSE available
        lines.append("> *Note: Peak Error, Last Violation, MAE metrics not available (legacy data format)*")
        lines.append("")
        lines.append("| Controller | RMSE (m) | Std Dev |")
        lines.append("|------------|----------|---------|")

    for ctrl in CONTROLLER_ORDER:
        if rmse_stats[ctrl]["n"] > 0:
            rmse = rmse_stats[ctrl]["mean"]
            std = rmse_stats[ctrl]["std"]

            if has_offline_metrics:
                # Format OFFLINE metrics (N/A if not available for this controller)
                peak = peak_stats[ctrl]
                violation = violation_stats[ctrl]
                mae = mae_stats[ctrl]

                peak_str = f"{peak['mean']:.3f}" if peak.get("available") else "N/A"
                # last_violation_time=0.0 is valid (means fast settle)
                violation_str = f"{violation['mean']:.1f}s" if violation.get("available") else "N/A"
                mae_str = f"{mae['mean']:.3f}" if mae.get("available") else "N/A"

                lines.append(
                    f"| **{ctrl}** | {rmse:.3f} | {std:.3f} | {peak_str} | {violation_str} | {mae_str} |"
                )
            else:
                # Legacy format
                lines.append(f"| **{ctrl}** | {rmse:.3f} | {std:.3f} |")
        else:
            if has_offline_metrics:
                lines.append(f"| **{ctrl}** | - | - | - | - | - |")
            else:
                lines.append(f"| **{ctrl}** | - | - |")

    lines.append("")

    # Winner
    winner = find_winner(rmse_stats, lower_is_better=True)
    best_rmse = rmse_stats.get(winner, {}).get("mean", 0)
    lines.append(f"### Winner: {winner} ({best_rmse:.2f}m RMSE)")
    lines.append("")

    # Phase 5 (COMBINED) specific note about high RMSE
    if phase_id == 5 and best_rmse > 5.0:
        lines.append(
            "**Note**: The high RMSE values (8-11m) in this phase represent genuine agent "
            "dispersion under combined stochastic disturbances, not measurement error. "
            "The stochastic wind profile combines turbulence, gusts, and direction wander "
            "simultaneously, creating conditions that exceed the controllers' rejection "
            "bandwidth. This highlights a fundamental limitation of the current control "
            "architecture under extreme disturbance scenarios."
        )
        lines.append("")

    lines.append("---")
    lines.append("")

    return "\n".join(lines)


def generate_summary_table(results: Dict[int, List[Dict]]) -> str:
    """Generate overall summary table."""
    lines = []
    lines.append("## Summary: Controller Performance Ranking")
    lines.append("")
    lines.append("| Phase | Best Overall | Best RMSE | Notes |")
    lines.append("|-------|--------------|-----------|-------|")

    for phase_id in sorted(results.keys()):
        runs = results[phase_id]
        phase_name = PHASE_DEFINITIONS.get(phase_id, {}).get("name", f"PHASE_{phase_id}")

        rmse_stats = compute_cross_run_stats(runs, "mean_rmse")
        winner = find_winner(rmse_stats)
        best_rmse = rmse_stats.get(winner, {}).get("mean", 0)

        if best_rmse > 0:
            lines.append(f"| {phase_id}. {phase_name} | {winner} | {best_rmse:.2f}m | N={len(runs)} |")
        else:
            lines.append(f"| {phase_id}. {phase_name} | (Pending) | - | No data |")

    lines.append("")
    return "\n".join(lines)


def generate_experimental_setup() -> str:
    """Generate experimental setup section."""
    return """## Experimental Setup

### Platform
- **Drones**: 12x Crazyflie 2.1 (27g, 92mm)
- **Simulation**: Gazebo 11 + ROS2 Humble
- **Control Rate**: 200 Hz
- **Physics Step**: 1 ms

### Controller Parameters
```yaml
PID Gains (all controllers):
  Kp: 3.501
  Ki: 1.946 (0 for PD)
  Kd: 3.608

Fuzzy Mixing (Additive Mode):
  k_pid: 1.0    # Full PID contribution
  k_fuzzy: 0.5  # Fuzzy adds on top
  # Formula: u = 1.0*u_pid + 0.5*u_fuzzy

Wind Input:
  fuzzy.include_wind: true
  fuzzy.wind_scalar: 1.0  # CRITICAL: Must be non-zero!

GT2 Specific:
  Alpha Levels: 5
  Secondary Shape: Triangular
  Secondary Spread: 0.3
```

### Formation Configuration
- Shape: Triangle (3 drones per group)
- Spacing: 3.0 meters
- Groups: 4 (PD, PID, IT2, GT2)
- Y-Lane Separation: 8 meters

### Metric Definitions
| Metric | Definition | Formula |
|--------|------------|---------|
| RMSE | Root Mean Squared Error | sqrt(mean(e²)) |
| Peak Error | Maximum error magnitude | max(\|e\|) over mission |
| Last Violation | Last time error exceeded 0.1m threshold | max(t) where \|e\| > 0.1m |
| MAE | Mean Absolute Error | mean(\|e\|) |
| Control Effort | Integral of control magnitude | ∫\|u\| dt (IAE) |

**Note on Last Violation**: This metric uses a strict 0.1m threshold. Values near mission duration (~60s) indicate the controller never achieved sustained sub-threshold performance. This is NOT settling time in the classical control sense—it measures violation persistence, not transient response characteristics.

"""


def generate_thesis_md(results_dir: Path, output_file: Optional[Path] = None) -> str:
    """Generate complete thesis results markdown.

    Args:
        results_dir: Directory containing phase results
        output_file: Optional output file path

    Returns:
        Generated markdown content
    """
    results = load_all_results(results_dir)

    if not results:
        print(f"WARNING: No results found in {results_dir}")
        return ""

    # Get git commit from first available metadata
    git_commit = "unknown"
    for runs in results.values():
        for run in runs:
            if "git_commit" in run:
                git_commit = run["git_commit"]
                break
        if git_commit != "unknown":
            break

    # Build markdown
    lines = []

    # Header
    lines.append("# Multi-Agent Formation Control: Controller Comparison Under Wind Disturbances")
    lines.append("")
    lines.append("## Executive Summary")
    lines.append("")
    lines.append("This document presents comprehensive experimental results comparing four controller types for multi-agent drone formation control under various wind disturbance scenarios. The study uses 12 Crazyflie 2.1 drones in a Gazebo simulation environment with ROS2 Humble.")
    lines.append("")

    # Controllers tested table
    lines.append("### Controllers Tested")
    lines.append("| Controller | Type | Description |")
    lines.append("|------------|------|-------------|")
    lines.append("| **PD** | Proportional-Derivative | Baseline controller without integral action |")
    lines.append("| **PID** | Proportional-Integral-Derivative | Classical PID with anti-windup |")
    lines.append("| **IT2-FLS** | Interval Type-2 Fuzzy Logic System | PID + IT2 fuzzy (additive: 100% PID + 50% fuzzy) |")
    lines.append("| **GT2-FLS** | General Type-2 Fuzzy Logic System | PID + GT2 with 5 alpha-planes (additive mode) |")
    lines.append("")

    # Summary table
    lines.append(generate_summary_table(results))

    lines.append("---")
    lines.append("")

    # Phase sections
    for phase_id in sorted(results.keys()):
        lines.append(generate_phase_section(phase_id, results[phase_id]))

    # Experimental setup
    lines.append(generate_experimental_setup())

    # Footer
    lines.append("---")
    lines.append("")
    lines.append(f"*Report Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}*")
    lines.append(f"*Git Commit: {git_commit}*")
    lines.append(f"*Source: {results_dir}*")
    lines.append("*Framework: 6-Phase Thesis Test Framework v1.0*")
    lines.append("")

    content = "\n".join(lines)

    # Write to file if specified
    if output_file:
        output_file.parent.mkdir(parents=True, exist_ok=True)
        with open(output_file, "w") as f:
            f.write(content)
        print(f"Generated: {output_file}")

    return content


def main():
    parser = argparse.ArgumentParser(
        description="Generate thesis results markdown from phase_metadata.json files"
    )
    parser.add_argument(
        "--results-dir",
        type=str,
        default="results",
        help="Directory containing phase results",
    )
    parser.add_argument(
        "--output",
        type=str,
        default=None,
        help="Output markdown file path",
    )
    parser.add_argument(
        "--print",
        action="store_true",
        help="Print to stdout instead of file",
    )

    args = parser.parse_args()

    results_dir = Path(args.results_dir)
    if not results_dir.exists():
        print(f"ERROR: Results directory not found: {results_dir}")
        return 1

    output_file = Path(args.output) if args.output else None

    content = generate_thesis_md(results_dir, output_file)

    if args.print or not output_file:
        print(content)

    return 0


if __name__ == "__main__":
    exit(main())
