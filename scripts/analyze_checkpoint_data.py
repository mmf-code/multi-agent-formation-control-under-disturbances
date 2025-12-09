#!/usr/bin/env python3
"""
Checkpoint Data Analyzer
Analyzes phase-by-phase performance for thesis

Calculates for each phase:
- Mean/std RMSE, IAE, ITAE
- Settling time statistics
- Overshoot percentages
- Controller comparison metrics
"""

import pandas as pd
import numpy as np
from pathlib import Path
import sys
import argparse


def analyze_checkpoint(checkpoint_dir):
    """Analyze a single checkpoint directory"""
    checkpoint_dir = Path(checkpoint_dir)

    if not checkpoint_dir.exists():
        print(f"ERROR: Checkpoint not found: {checkpoint_dir}")
        return None

    phase_name = checkpoint_dir.name
    print(f"\n{'='*70}")
    print(f"  CHECKPOINT: {phase_name}")
    print(f"{'='*70}\n")

    # Find all CSV files
    csv_files = list(checkpoint_dir.glob("*.csv"))

    if not csv_files:
        print("  No CSV files found")
        return None

    results = {}

    for csv_file in csv_files:
        agent_name = csv_file.stem  # e.g., "agent_0_pidfuzzy"

        try:
            df = pd.read_csv(csv_file)

            if len(df) == 0:
                print(f"  {agent_name}: No data")
                continue

            # Calculate statistics
            stats = {
                'samples': len(df),
                'rmse_mean': df['rmse_total'].mean(),
                'rmse_std': df['rmse_total'].std(),
                'rmse_final': df['rmse_total'].iloc[-1] if len(df) > 0 else np.nan,
                'iae_x_final': df['iae_x'].iloc[-1] if len(df) > 0 else np.nan,
                'iae_y_final': df['iae_y'].iloc[-1] if len(df) > 0 else np.nan,
                'itae_x_final': df['itae_x'].iloc[-1] if len(df) > 0 else np.nan,
                'itae_y_final': df['itae_y'].iloc[-1] if len(df) > 0 else np.nan,
                'max_overshoot_x': df['max_overshoot_x'].max(),
                'max_overshoot_y': df['max_overshoot_y'].max(),
                'settling_time': df['settling_time'].iloc[-1] if len(df) > 0 else np.nan,
                'settled_pct': (df['is_settled'].sum() / len(df) * 100) if len(df) > 0 else 0,
            }

            results[agent_name] = stats

            # Print summary
            print(f"  {agent_name:25s}")
            print(f"    Samples:        {stats['samples']:6d}")
            print(f"    RMSE (mean):    {stats['rmse_mean']:8.4f} m")
            print(f"    RMSE (final):   {stats['rmse_final']:8.4f} m")
            print(f"    IAE (x/y):      {stats['iae_x_final']:8.3f} / {stats['iae_y_final']:8.3f}")
            print(f"    ITAE (x/y):     {stats['itae_x_final']:8.2f} / {stats['itae_y_final']:8.2f}")
            print(f"    Overshoot:      {stats['max_overshoot_x']:7.3f} / {stats['max_overshoot_y']:7.3f} m")
            print(f"    Settled:        {stats['settled_pct']:6.1f}%")
            print()

        except Exception as e:
            print(f"  ERROR reading {agent_name}: {e}")

    return results


def compare_controllers(all_results):
    """Compare all controllers across checkpoints"""
    print("\n" + "="*70)
    print("  CONTROLLER COMPARISON (All Phases)")
    print("="*70 + "\n")

    # Group by controller type
    controller_groups = {
        'PID+Fuzzy': [],
        'PD': [],
        'PID': []
    }

    for checkpoint, results in all_results.items():
        for agent_name, stats in results.items():
            if 'pidfuzzy' in agent_name:
                controller_groups['PID+Fuzzy'].append(stats)
            elif agent_name.endswith('_pd'):
                controller_groups['PD'].append(stats)
            elif agent_name.endswith('_pid'):
                controller_groups['PID'].append(stats)

    # Print comparison table
    print(f"{'Controller':<15} {'Avg RMSE':>12} {'Avg IAE_x':>12} {'Avg IAE_y':>12} {'Avg ITAE_x':>12} {'Samples':>10}")
    print("-" * 75)

    for controller, stats_list in controller_groups.items():
        if not stats_list:
            continue

        avg_rmse = np.mean([s['rmse_mean'] for s in stats_list])
        avg_iae_x = np.mean([s['iae_x_final'] for s in stats_list if not np.isnan(s['iae_x_final'])])
        avg_iae_y = np.mean([s['iae_y_final'] for s in stats_list if not np.isnan(s['iae_y_final'])])
        avg_itae_x = np.mean([s['itae_x_final'] for s in stats_list if not np.isnan(s['itae_x_final'])])
        total_samples = sum([s['samples'] for s in stats_list])

        print(f"{controller:<15} {avg_rmse:12.4f} {avg_iae_x:12.3f} {avg_iae_y:12.3f} {avg_itae_x:12.2f} {total_samples:10d}")

    print()


def main():
    parser = argparse.ArgumentParser(description='Analyze checkpoint data')
    parser.add_argument('output_dir', help='Output directory containing checkpoints/')

    args = parser.parse_args()

    output_dir = Path(args.output_dir)
    checkpoints_dir = output_dir / 'checkpoints'

    if not checkpoints_dir.exists():
        print(f"ERROR: Checkpoints directory not found: {checkpoints_dir}")
        sys.exit(1)

    # Find all checkpoint directories
    checkpoint_dirs = sorted([d for d in checkpoints_dir.iterdir() if d.is_dir()])

    if not checkpoint_dirs:
        print("ERROR: No checkpoint directories found")
        sys.exit(1)

    print("\n" + "="*70)
    print("  CHECKPOINT DATA ANALYSIS")
    print("="*70)
    print(f"\nOutput Directory: {output_dir}")
    print(f"Checkpoints Found: {len(checkpoint_dirs)}\n")

    # Analyze each checkpoint
    all_results = {}
    for checkpoint_dir in checkpoint_dirs:
        results = analyze_checkpoint(checkpoint_dir)
        if results:
            all_results[checkpoint_dir.name] = results

    # Overall comparison
    if all_results:
        compare_controllers(all_results)

    print("\n" + "="*70)
    print("  ANALYSIS COMPLETE")
    print("="*70 + "\n")


if __name__ == '__main__':
    main()
