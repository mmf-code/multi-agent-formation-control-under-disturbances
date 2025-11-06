#!/usr/bin/env python3
"""
Controller Comparison Script
Compare PID, PD, and PID+Fuzzy controllers across different scenarios

Features:
- Side-by-side controller comparison
- Statistical analysis (mean, std, min, max)
- Performance tables
- Publication-ready comparison plots
- Support for both step response and long scenario data

Usage:
    # Compare single scenario
    python3 scripts/compare_controllers.py <data_dir>

    # Compare step response vs long scenario
    python3 scripts/compare_controllers.py <step_response_dir> --long-scenario <long_scenario_dir>

Author: Multi-Agent Formation Control Research Team
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import argparse
import sys
from typing import Dict, List, Tuple


class ControllerComparator:
    """Compare controller performance across scenarios"""

    def __init__(self, step_response_dir=None, long_scenario_dir=None):
        self.step_response_dir = Path(step_response_dir) if step_response_dir else None
        self.long_scenario_dir = Path(long_scenario_dir) if long_scenario_dir else None

        self.controllers = {
            'agent_0': 'PID+Fuzzy',
            'agent_3': 'PD',
            'agent_6': 'PID'
        }

        self.colors = {
            'PID+Fuzzy': '#2E7D32',  # Green
            'PD': '#1976D2',          # Blue
            'PID': '#D32F2F'          # Red
        }

        self.data = {}

    def load_data(self):
        """Load CSV data from all scenarios"""
        print("\n" + "="*70)
        print("  CONTROLLER COMPARISON")
        print("="*70)

        if self.step_response_dir:
            print(f"\nStep Response: {self.step_response_dir}")
            self.data['step_response'] = self._load_scenario_data(
                self.step_response_dir,
                'step_response'
            )

        if self.long_scenario_dir:
            print(f"\nLong Scenario: {self.long_scenario_dir}")
            self.data['long_scenario'] = self._load_scenario_data(
                self.long_scenario_dir,
                'long_scenario'
            )

        if not self.data:
            print("\n⚠ ERROR: No data loaded!")
            sys.exit(1)

        print("\n" + "="*70 + "\n")

    def _load_scenario_data(self, data_dir, scenario_name):
        """Load data for a specific scenario"""
        scenario_data = {}

        # Search for CSV files in multiple locations
        search_dirs = [
            data_dir,
            data_dir / 'final_results',
            data_dir / 'raw_data'
        ]

        for agent_id, controller_name in self.controllers.items():
            found = False

            for search_dir in search_dirs:
                if not search_dir.exists():
                    continue

                csv_pattern = f"{agent_id}_*_full.csv"
                csv_files = list(search_dir.glob(csv_pattern))

                if csv_files:
                    csv_path = csv_files[0]
                    print(f"  ✓ {controller_name}: {csv_path.name}")

                    df = pd.read_csv(csv_path)
                    scenario_data[controller_name] = df
                    found = True
                    break

            if not found:
                print(f"  ⚠ {controller_name}: No data found")

        return scenario_data

    def compute_statistics(self):
        """Compute statistical metrics for all controllers"""
        print("="*70)
        print("  STATISTICAL ANALYSIS")
        print("="*70 + "\n")

        self.stats = {}

        for scenario_name, scenario_data in self.data.items():
            print(f"{scenario_name.upper().replace('_', ' ')}:")
            print("-"*70)

            self.stats[scenario_name] = {}

            for controller_name, df in scenario_data.items():
                stats = self._compute_controller_stats(df)
                self.stats[scenario_name][controller_name] = stats

                print(f"\n  {controller_name}:")
                print(f"    Error Magnitude:")
                print(f"      Mean:   {stats['error_mean']:.4f} m")
                print(f"      Std:    {stats['error_std']:.4f} m")
                print(f"      Max:    {stats['error_max']:.4f} m")
                print(f"      Min:    {stats['error_min']:.4f} m")
                print(f"    RMSE Total:")
                print(f"      Mean:   {stats['rmse_mean']:.4f} m")
                print(f"      Final:  {stats['rmse_final']:.4f} m")
                print(f"    Settling:")
                print(f"      Time:   {stats['settling_time']:.3f} s")
                print(f"      Ratio:  {stats['settling_ratio']:.2f} %")

            print()

        print("="*70 + "\n")

    def _compute_controller_stats(self, df):
        """Compute statistics for a single controller"""
        error = df['error_magnitude'].values
        rmse = df['rmse_total'].values
        settled = df['is_settled'].values if 'is_settled' in df.columns else None

        stats = {
            'error_mean': np.mean(error),
            'error_std': np.std(error),
            'error_max': np.max(error),
            'error_min': np.min(error),
            'rmse_mean': np.mean(rmse),
            'rmse_final': rmse[-1] if len(rmse) > 0 else np.nan,
        }

        # Settling statistics
        if settled is not None:
            settling_indices = np.where(settled == 1)[0]
            if len(settling_indices) > 0:
                stats['settling_time'] = df['elapsed_time'].values[settling_indices[0]]
                stats['settling_ratio'] = (np.sum(settled) / len(settled)) * 100
            else:
                stats['settling_time'] = np.nan
                stats['settling_ratio'] = 0.0
        else:
            stats['settling_time'] = np.nan
            stats['settling_ratio'] = np.nan

        return stats

    def print_comparison_tables(self):
        """Print comparison tables"""
        print("="*70)
        print("  COMPARISON TABLES")
        print("="*70 + "\n")

        for scenario_name in self.stats.keys():
            print(f"{scenario_name.upper().replace('_', ' ')}:")
            print("-"*70)

            # Performance metrics table
            print(f"\n  {'Metric':<25} | {'PID':<12} | {'PD':<12} | {'PID+Fuzzy':<12}")
            print(f"  {'-'*25}|{'-'*14}|{'-'*14}|{'-'*14}")

            metrics_info = [
                ('Mean Error (m)', 'error_mean', '{:.4f}'),
                ('Std Error (m)', 'error_std', '{:.4f}'),
                ('Max Error (m)', 'error_max', '{:.4f}'),
                ('Mean RMSE (m)', 'rmse_mean', '{:.4f}'),
                ('Final RMSE (m)', 'rmse_final', '{:.4f}'),
                ('Settling Time (s)', 'settling_time', '{:.3f}'),
                ('Settled Ratio (%)', 'settling_ratio', '{:.2f}')
            ]

            for metric_name, metric_key, fmt in metrics_info:
                values = {}
                for ctrl in ['PID', 'PD', 'PID+Fuzzy']:
                    if ctrl in self.stats[scenario_name]:
                        val = self.stats[scenario_name][ctrl][metric_key]
                        if np.isnan(val):
                            values[ctrl] = 'N/A'
                        else:
                            values[ctrl] = fmt.format(val)
                    else:
                        values[ctrl] = 'N/A'

                print(f"  {metric_name:<25} | {values.get('PID', 'N/A'):>12} | "
                      f"{values.get('PD', 'N/A'):>12} | {values.get('PID+Fuzzy', 'N/A'):>12}")

            print()

    def plot_comparison(self, output_dir):
        """Generate comparison plots"""
        print("="*70)
        print("  GENERATING COMPARISON PLOTS")
        print("="*70 + "\n")

        output_dir = Path(output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)

        # 1. Error magnitude comparison (all scenarios)
        self._plot_error_comparison(output_dir)

        # 2. Statistical metrics bar charts
        self._plot_statistics_bars(output_dir)

        # 3. Performance radar chart
        self._plot_performance_radar(output_dir)

        print(f"\n✓ Plots saved to: {output_dir}\n")

    def _plot_error_comparison(self, output_dir):
        """Plot error magnitude comparison"""
        n_scenarios = len(self.data)

        if n_scenarios == 1:
            fig, axes = plt.subplots(1, 1, figsize=(12, 6))
            axes = [axes]
        else:
            fig, axes = plt.subplots(1, n_scenarios, figsize=(14 * n_scenarios, 6))

        for idx, (scenario_name, scenario_data) in enumerate(self.data.items()):
            ax = axes[idx] if n_scenarios > 1 else axes[0]

            for controller_name, df in scenario_data.items():
                time = df['elapsed_time'].values
                error = df['error_magnitude'].values

                ax.plot(time, error,
                       label=controller_name,
                       color=self.colors[controller_name],
                       linewidth=2)

            ax.set_xlabel('Time (s)', fontsize=12)
            ax.set_ylabel('Formation Error (m)', fontsize=12)
            ax.set_title(f'{scenario_name.replace("_", " ").title()}',
                        fontsize=14, fontweight='bold')
            ax.legend(fontsize=11, loc='upper right')
            ax.grid(True, alpha=0.3)
            ax.set_xlim(0, None)
            ax.set_ylim(0, None)

        plt.tight_layout()
        plot_path = output_dir / 'controller_comparison_error.png'
        plt.savefig(plot_path, dpi=300, bbox_inches='tight')
        plt.close()

        print(f"  ✓ Error comparison: {plot_path.name}")

    def _plot_statistics_bars(self, output_dir):
        """Plot statistical metrics as bar charts"""
        if not self.stats:
            return

        # Select first scenario for detailed stats
        scenario_name = list(self.stats.keys())[0]
        stats = self.stats[scenario_name]

        fig, axes = plt.subplots(2, 3, figsize=(16, 10))
        axes = axes.flatten()

        metrics_info = [
            ('Mean Error (m)', 'error_mean'),
            ('Std Error (m)', 'error_std'),
            ('Max Error (m)', 'error_max'),
            ('Mean RMSE (m)', 'rmse_mean'),
            ('Settling Time (s)', 'settling_time'),
            ('Settled Ratio (%)', 'settling_ratio')
        ]

        for idx, (metric_name, metric_key) in enumerate(metrics_info):
            ax = axes[idx]

            controllers = []
            values = []
            colors = []

            for ctrl_name in ['PID', 'PD', 'PID+Fuzzy']:
                if ctrl_name in stats:
                    val = stats[ctrl_name][metric_key]
                    if not np.isnan(val):
                        controllers.append(ctrl_name)
                        values.append(val)
                        colors.append(self.colors[ctrl_name])

            if controllers:
                bars = ax.bar(controllers, values, color=colors, alpha=0.7, edgecolor='black')

                # Add value labels
                for bar in bars:
                    height = bar.get_height()
                    ax.text(bar.get_x() + bar.get_width()/2., height,
                           f'{height:.3f}',
                           ha='center', va='bottom', fontsize=10)

            ax.set_ylabel(metric_name, fontsize=11)
            ax.set_title(metric_name, fontsize=12, fontweight='bold')
            ax.grid(True, alpha=0.3, axis='y')

        plt.suptitle(f'Controller Performance Metrics - {scenario_name.replace("_", " ").title()}',
                    fontsize=16, fontweight='bold', y=1.00)
        plt.tight_layout()

        plot_path = output_dir / 'controller_statistics_bars.png'
        plt.savefig(plot_path, dpi=300, bbox_inches='tight')
        plt.close()

        print(f"  ✓ Statistics bars: {plot_path.name}")

    def _plot_performance_radar(self, output_dir):
        """Plot performance radar chart"""
        if not self.stats:
            return

        # Select first scenario
        scenario_name = list(self.stats.keys())[0]
        stats = self.stats[scenario_name]

        # Prepare data (normalize metrics to 0-1 range, lower is better)
        categories = ['Mean\nError', 'Std\nError', 'Max\nError', 'Mean\nRMSE', 'Settling\nTime']
        metric_keys = ['error_mean', 'error_std', 'error_max', 'rmse_mean', 'settling_time']

        # Get max values for normalization
        max_values = {}
        for key in metric_keys:
            values = [stats[ctrl][key] for ctrl in stats.keys() if not np.isnan(stats[ctrl][key])]
            max_values[key] = max(values) if values else 1.0

        # Compute angles
        N = len(categories)
        angles = np.linspace(0, 2 * np.pi, N, endpoint=False).tolist()
        angles += angles[:1]  # Complete the circle

        # Create radar plot
        fig, ax = plt.subplots(figsize=(10, 10), subplot_kw=dict(projection='polar'))

        for ctrl_name, ctrl_stats in stats.items():
            values = []
            for key in metric_keys:
                val = ctrl_stats[key]
                if np.isnan(val):
                    values.append(0)
                else:
                    # Normalize (invert so lower is better = outer ring)
                    normalized = 1.0 - (val / max_values[key])
                    values.append(normalized)

            values += values[:1]  # Complete the circle

            ax.plot(angles, values,
                   'o-', linewidth=2,
                   label=ctrl_name,
                   color=self.colors[ctrl_name])
            ax.fill(angles, values, alpha=0.15, color=self.colors[ctrl_name])

        ax.set_xticks(angles[:-1])
        ax.set_xticklabels(categories, fontsize=11)
        ax.set_ylim(0, 1)
        ax.set_yticks([0.2, 0.4, 0.6, 0.8, 1.0])
        ax.set_yticklabels(['', '', '', '', 'Best'], fontsize=9)
        ax.legend(loc='upper right', bbox_to_anchor=(1.3, 1.1), fontsize=12)
        ax.set_title(f'Controller Performance Radar - {scenario_name.replace("_", " ").title()}',
                    fontsize=14, fontweight='bold', pad=20)
        ax.grid(True)

        plt.tight_layout()
        plot_path = output_dir / 'controller_performance_radar.png'
        plt.savefig(plot_path, dpi=300, bbox_inches='tight')
        plt.close()

        print(f"  ✓ Performance radar: {plot_path.name}")

    def save_results(self, output_dir):
        """Save comparison results to CSV"""
        output_dir = Path(output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)

        if not self.stats:
            return

        # Save statistics for each scenario
        for scenario_name, stats in self.stats.items():
            df = pd.DataFrame(stats).T
            csv_path = output_dir / f'{scenario_name}_comparison.csv'
            df.to_csv(csv_path)
            print(f"  ✓ Saved: {csv_path.name}")


def main():
    parser = argparse.ArgumentParser(
        description='Compare controller performance across scenarios'
    )
    parser.add_argument('data_dir', type=str,
                       help='Primary data directory (step response or long scenario)')
    parser.add_argument('--long-scenario', '-l', type=str, default=None,
                       help='Long scenario data directory (for comparison)')
    parser.add_argument('--output-dir', '-o', type=str, default=None,
                       help='Output directory for comparison results')

    args = parser.parse_args()

    # Determine scenario types
    data_dir = Path(args.data_dir)

    if 'step_response' in str(data_dir):
        step_response_dir = data_dir
        long_scenario_dir = args.long_scenario
    else:
        step_response_dir = None
        long_scenario_dir = data_dir

    # Output directory
    if args.output_dir:
        output_dir = Path(args.output_dir)
    else:
        output_dir = data_dir / 'comparison'

    # Initialize comparator
    comparator = ControllerComparator(
        step_response_dir=step_response_dir,
        long_scenario_dir=long_scenario_dir
    )

    # Load data
    comparator.load_data()

    # Compute statistics
    comparator.compute_statistics()

    # Print tables
    comparator.print_comparison_tables()

    # Generate plots
    comparator.plot_comparison(output_dir)

    # Save results
    comparator.save_results(output_dir)

    print("\n" + "="*70)
    print("  COMPARISON COMPLETE!")
    print("="*70 + "\n")


if __name__ == '__main__':
    main()
