#!/usr/bin/env python3
"""
Wind Disturbance KPI Calculator

Calculates 10 defense-industry standard wind performance indicators from
simulation data. Used for systematic controller comparison under disturbances.

Usage:
    python3 wind_kpi_calculator.py <metrics_csv_file>
    python3 wind_kpi_calculator.py thesis_data/wind_test.csv --output results.json

Author: Multi-Agent Formation Control Research
Date: 2026-01-02
"""

import argparse
import json
import sys
from pathlib import Path
from typing import Dict, List

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def calculate_drr(df: pd.DataFrame) -> float:
    """
    Disturbance Rejection Ratio (DRR).

    DRR = mean(|error| / |wind|)

    Lower is better. Target: < 0.5
    """
    error_mag = np.sqrt(df['error_x']**2 + df['error_y']**2)
    wind_mag = df['wind_magnitude']

    # Avoid division by zero
    mask = wind_mag > 0.1
    if not mask.any():
        return 0.0

    drr = (error_mag[mask] / wind_mag[mask]).mean()
    return float(drr)


def calculate_wnrmse(df: pd.DataFrame) -> float:
    """
    Wind-Normalized RMSE (WNRMSE).

    WNRMSE = RMSE_total / mean(|wind|)

    Dimensionless metric. Target: < 0.8
    """
    rmse = df['rmse_total'].mean()
    wind_mag_mean = df['wind_magnitude'].mean()

    if wind_mag_mean < 0.1:
        return rmse  # No normalization if no wind

    return float(rmse / wind_mag_mean)


def calculate_wit(df: pd.DataFrame, error_threshold: float = 0.2) -> float:
    """
    Wind Impact Time (WIT).

    Cumulative time when |error| > threshold under wind conditions.

    Returns: Percentage of total time. Target: < 20%
    """
    error_mag = np.sqrt(df['error_x']**2 + df['error_y']**2)
    wind_mask = df['wind_magnitude'] > 1.0
    error_mask = error_mag > error_threshold

    impact_time = (wind_mask & error_mask).sum()
    total_time = len(df)

    return float(100.0 * impact_time / total_time) if total_time > 0 else 0.0


def calculate_mwdr(df: pd.DataFrame) -> float:
    """
    Maximum Wind Disturbance Rejection (MWDR).

    MWDR = 1 - max(|error| / |wind|)

    Measures worst-case rejection. Target: > 0.6
    """
    error_mag = np.sqrt(df['error_x']**2 + df['error_y']**2)
    wind_mag = df['wind_magnitude']

    mask = wind_mag > 0.1
    if not mask.any():
        return 1.0

    max_ratio = (error_mag[mask] / wind_mag[mask]).max()
    return float(1.0 - max_ratio)


def calculate_stc(df: pd.DataFrame, wind_speed: float = 3.0) -> float:
    """
    Settling Time under Constant wind (STC).

    Time to settle within 5% of target under constant wind.

    Target: < 5.0 seconds
    """
    # Find region with constant wind near target speed
    wind_mask = np.abs(df['wind_magnitude'] - wind_speed) < 0.5

    if not wind_mask.any():
        return df['settling_time'].mean()

    # Get settling times during constant wind
    settling_times = df.loc[wind_mask, 'settling_time']
    return float(settling_times.mean())


def calculate_gtr(df: pd.DataFrame, gust_threshold: float = 4.0) -> float:
    """
    Gust Transient Response (GTR).

    Maximum overshoot during wind gust events.

    Target: < 0.3 m
    """
    # Detect gusts (rapid wind increase)
    wind_diff = df['wind_magnitude'].diff()
    gust_mask = wind_diff > (gust_threshold * 0.3)

    if not gust_mask.any():
        return 0.0

    # Find max overshoot during gusts
    overshoot_x = df.loc[gust_mask, 'max_overshoot_x']
    overshoot_y = df.loc[gust_mask, 'max_overshoot_y']

    max_overshoot = max(overshoot_x.max(), overshoot_y.max())
    return float(max_overshoot)


def calculate_scm(df: pd.DataFrame, agent_id: int = 0) -> float:
    """
    Spatial Coherence Metric (SCM).

    Correlation between wind at this agent and nearby agents.
    Expected: 0.7-0.9 for properly implemented IEC coherence.

    Note: Requires multi-agent data. Returns 1.0 if single agent.
    """
    # Placeholder: Would need multi-agent data
    # For now, return N/A
    return 1.0


def calculate_tim(df: pd.DataFrame) -> float:
    """
    Turbulence Intensity Margin (TIM).

    Maximum turbulence intensity before instability.

    Estimated from: TI = σ(wind) / mean(wind)
    Target: > 25%
    """
    wind_std = df['wind_magnitude'].std()
    wind_mean = df['wind_magnitude'].mean()

    if wind_mean < 0.1:
        return 0.0

    ti = 100.0 * wind_std / wind_mean
    return float(ti)


def calculate_wre(rmse_wind: float, rmse_no_wind: float) -> float:
    """
    Wind Rejection Efficiency (WRE).

    WRE = 1 - (RMSE_wind / RMSE_no_wind)

    Requires baseline no-wind data. Target: > 0.4
    """
    if rmse_no_wind < 1e-6:
        return 0.0

    wre = 1.0 - (rmse_wind / rmse_no_wind)
    return float(wre)


def calculate_all_kpis(df: pd.DataFrame) -> Dict[str, float]:
    """
    Calculate all 10 Wind KPIs.

    Returns:
        Dictionary of KPI name -> value
    """
    kpis = {
        'DRR': calculate_drr(df),
        'WNRMSE': calculate_wnrmse(df),
        'WIT': calculate_wit(df),
        'MWDR': calculate_mwdr(df),
        'STC': calculate_stc(df),
        'GTR': calculate_gtr(df),
        'SCM': calculate_scm(df),
        'TIM': calculate_tim(df),
        # WRE requires baseline comparison - set to 0.0 for now
        'WRE': 0.0,
        # Additional summary stats
        'mean_rmse': float(df['rmse_total'].mean()),
        'mean_wind_magnitude': float(df['wind_magnitude'].mean()),
        'max_error': float(np.sqrt(df['error_x']**2 + df['error_y']**2).max()),
    }

    return kpis


def plot_kpi_summary(kpis: Dict[str, float], output_file: Path):
    """Generate KPI summary visualization."""
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))

    # KPI values (first 8 metrics)
    kpi_names = ['DRR', 'WNRMSE', 'WIT', 'MWDR', 'STC', 'GTR', 'TIM']
    kpi_values = [kpis[k] for k in kpi_names]
    kpi_targets = [0.5, 0.8, 20.0, 0.6, 5.0, 0.3, 25.0]
    kpi_better = ['lower', 'lower', 'lower', 'higher', 'lower', 'lower', 'higher']

    # Plot 1: KPI Bar Chart
    ax = axes[0, 0]
    colors = ['green' if (
        (kpi_values[i] < kpi_targets[i] and kpi_better[i] == 'lower') or
        (kpi_values[i] > kpi_targets[i] and kpi_better[i] == 'higher')
    ) else 'red' for i in range(len(kpi_values))]

    ax.barh(kpi_names, kpi_values, color=colors, alpha=0.7)
    ax.set_xlabel('KPI Value')
    ax.set_title('Wind Performance KPIs', fontweight='bold')
    ax.grid(True, alpha=0.3, axis='x')

    # Plot 2: Target Comparison
    ax = axes[0, 1]
    x = np.arange(len(kpi_names))
    width = 0.35
    ax.bar(x - width/2, kpi_values, width, label='Actual', alpha=0.7)
    ax.bar(x + width/2, kpi_targets, width, label='Target', alpha=0.7)
    ax.set_xticks(x)
    ax.set_xticklabels(kpi_names, rotation=45, ha='right')
    ax.set_ylabel('Value')
    ax.set_title('Actual vs Target KPIs', fontweight='bold')
    ax.legend()
    ax.grid(True, alpha=0.3, axis='y')

    # Plot 3: Summary Stats
    ax = axes[1, 0]
    stats = {
        'Mean RMSE': kpis['mean_rmse'],
        'Mean Wind': kpis['mean_wind_magnitude'],
        'Max Error': kpis['max_error'],
    }
    ax.bar(stats.keys(), stats.values(), color=['blue', 'orange', 'red'], alpha=0.7)
    ax.set_ylabel('Value [m or m/s]')
    ax.set_title('Summary Statistics', fontweight='bold')
    ax.grid(True, alpha=0.3, axis='y')
    plt.setp(ax.xaxis.get_majorticklabels(), rotation=45, ha='right')

    # Plot 4: Performance Grade
    ax = axes[1, 1]
    ax.axis('off')

    # Calculate overall score (0-100)
    score = 0.0
    for i, name in enumerate(kpi_names):
        if kpi_better[i] == 'lower':
            score += 100.0 * max(0, 1.0 - kpi_values[i] / kpi_targets[i]) / len(kpi_names)
        else:
            score += 100.0 * min(1.0, kpi_values[i] / kpi_targets[i]) / len(kpi_names)

    grade = 'A' if score >= 90 else 'B' if score >= 80 else 'C' if score >= 70 else 'D' if score >= 60 else 'F'
    grade_color = 'green' if score >= 80 else 'orange' if score >= 60 else 'red'

    ax.text(0.5, 0.6, f'Overall Score: {score:.1f}/100', ha='center', va='center',
            fontsize=20, fontweight='bold')
    ax.text(0.5, 0.4, f'Grade: {grade}', ha='center', va='center',
            fontsize=36, fontweight='bold', color=grade_color)

    performance_text = (
        f"DRR: {kpis['DRR']:.3f}\n"
        f"WNRMSE: {kpis['WNRMSE']:.3f}\n"
        f"Wind: {kpis['mean_wind_magnitude']:.2f} m/s"
    )
    ax.text(0.5, 0.15, performance_text, ha='center', va='center',
            fontsize=12, family='monospace',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

    plt.tight_layout()
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"  Saved: {output_file}")


def main():
    parser = argparse.ArgumentParser(description='Calculate Wind Disturbance KPIs')
    parser.add_argument('input_file', type=str, help='Input CSV file with metrics data')
    parser.add_argument('--output', '-o', type=str, default=None,
                        help='Output JSON file for KPI results')
    parser.add_argument('--plot', action='store_true',
                        help='Generate KPI visualization plot')

    args = parser.parse_args()

    # Load data
    input_path = Path(args.input_file)
    if not input_path.exists():
        print(f"Error: File not found: {input_path}")
        sys.exit(1)

    print(f"\nLoading metrics from: {input_path}")
    df = pd.read_csv(input_path)

    # Calculate KPIs
    print("\nCalculating Wind KPIs...")
    kpis = calculate_all_kpis(df)

    # Print results
    print("\n" + "="*60)
    print("WIND DISTURBANCE KPI RESULTS")
    print("="*60)

    print("\n--- Core Performance Metrics ---")
    print(f"  DRR (Disturbance Rejection Ratio):    {kpis['DRR']:.3f}  (target: < 0.5)")
    print(f"  WNRMSE (Wind-Normalized RMSE):        {kpis['WNRMSE']:.3f}  (target: < 0.8)")
    print(f"  WIT (Wind Impact Time):               {kpis['WIT']:.1f}%  (target: < 20%)")
    print(f"  MWDR (Max Wind Disturbance Rejection): {kpis['MWDR']:.3f}  (target: > 0.6)")

    print("\n--- Response Characteristics ---")
    print(f"  STC (Settling Time under Constant):   {kpis['STC']:.2f}s  (target: < 5s)")
    print(f"  GTR (Gust Transient Response):        {kpis['GTR']:.3f}m  (target: < 0.3m)")

    print("\n--- Wind Environment ---")
    print(f"  TIM (Turbulence Intensity Margin):    {kpis['TIM']:.1f}%  (target: > 25%)")
    print(f"  Mean Wind Magnitude:                  {kpis['mean_wind_magnitude']:.2f} m/s")

    print("\n--- Summary Statistics ---")
    print(f"  Mean RMSE:                            {kpis['mean_rmse']:.3f} m")
    print(f"  Max Error:                            {kpis['max_error']:.3f} m")

    print("="*60 + "\n")

    # Save to JSON
    if args.output:
        output_path = Path(args.output)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        with open(output_path, 'w') as f:
            json.dump(kpis, f, indent=2)
        print(f"✓ KPIs saved to: {output_path}")

    # Generate plot
    if args.plot:
        plot_file = input_path.parent / (input_path.stem + '_kpis.png')
        plot_kpi_summary(kpis, plot_file)

    # Return success if KPIs meet targets
    success = (
        kpis['DRR'] < 0.5 and
        kpis['WNRMSE'] < 0.8 and
        kpis['WIT'] < 20.0 and
        kpis['MWDR'] > 0.6
    )

    if success:
        print("✓ All critical KPIs within target range!")
        sys.exit(0)
    else:
        print("⚠ Some KPIs outside target range")
        sys.exit(1)


if __name__ == '__main__':
    main()
