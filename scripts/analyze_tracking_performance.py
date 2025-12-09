#!/usr/bin/env python3
"""
Tracking Performance Analyzer for Formation Control

Analyzes controller performance for smooth trajectory tracking under wind disturbance.
More appropriate than step response for interpolated waypoint systems.

Metrics:
- RMSE (Root Mean Square Error) - overall tracking accuracy
- MAE (Mean Absolute Error) - average deviation
- Peak Error - maximum deviation (during gusts)
- Steady-State Error - long-term accuracy
- IAE (Integral Absolute Error) - cumulative error
- Recovery Time - time to return to threshold after disturbance

Compares: PID, PD, PID+Fuzzy controllers
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import argparse
from dataclasses import dataclass
from typing import Dict, List


@dataclass
class TrackingMetrics:
    """Tracking performance metrics"""
    controller: str
    rmse_x: float
    rmse_y: float
    rmse_total: float
    mae_x: float
    mae_y: float
    peak_error_x: float
    peak_error_y: float
    peak_error_total: float
    steady_state_error: float
    iae_x: float
    iae_y: float
    time_in_threshold: float  # % of time within 0.1m


class TrackingPerformanceAnalyzer:
    """Analyze trajectory tracking performance under wind disturbance"""

    def __init__(self, data_dir):
        self.data_dir = Path(data_dir)
        self.results = {}
        self.data = {}

        # Controller mapping
        self.controllers = {
            'agent_0': 'PID+Fuzzy',
            'agent_3': 'PD',
            'agent_6': 'PID'
        }

        # Colors for plots
        self.colors = {
            'PID+Fuzzy': '#2E7D32',  # Green
            'PD': '#1976D2',          # Blue
            'PID': '#D32F2F'          # Red
        }

    def load_data(self):
        """Load CSV data for all agents"""
        print("\n" + "="*70)
        print("  TRACKING PERFORMANCE ANALYZER")
        print("="*70)
        print(f"Loading data from: {self.data_dir}\n")

        for agent_id, controller_name in self.controllers.items():
            csv_files = list(self.data_dir.glob(f"**/{agent_id}_*.csv"))

            if not csv_files:
                print(f"WARNING: No data for {agent_id} ({controller_name})")
                continue

            csv_path = csv_files[0]
            print(f"Loading {agent_id} ({controller_name}): {csv_path.name}")

            df = pd.read_csv(csv_path)
            self.data[controller_name] = df

        print(f"\nLoaded {len(self.data)} controllers\n")

    def analyze_controller(self, controller_name: str) -> TrackingMetrics:
        """Compute tracking metrics for one controller"""
        df = self.data[controller_name]

        # Get error columns
        error_x = df['error_x'].values
        error_y = df['error_y'].values

        # Skip first 5 seconds (initial takeoff/settling)
        time = df['elapsed_time'].values
        mask = time > 5.0

        error_x = error_x[mask]
        error_y = error_y[mask]
        time = time[mask]

        if len(error_x) == 0:
            return None

        # Compute metrics
        error_total = np.sqrt(error_x**2 + error_y**2)

        rmse_x = np.sqrt(np.mean(error_x**2))
        rmse_y = np.sqrt(np.mean(error_y**2))
        rmse_total = np.sqrt(np.mean(error_total**2))

        mae_x = np.mean(np.abs(error_x))
        mae_y = np.mean(np.abs(error_y))

        peak_error_x = np.max(np.abs(error_x))
        peak_error_y = np.max(np.abs(error_y))
        peak_error_total = np.max(error_total)

        # Steady-state error (average of last 10 seconds)
        last_10s = time > (time[-1] - 10.0)
        if np.sum(last_10s) > 10:
            ss_error = np.mean(error_total[last_10s])
        else:
            ss_error = np.mean(error_total[-50:])

        # IAE
        dt = np.mean(np.diff(time))
        iae_x = np.sum(np.abs(error_x)) * dt
        iae_y = np.sum(np.abs(error_y)) * dt

        # Time within threshold (0.1m)
        in_threshold = error_total < 0.1
        time_in_threshold = 100.0 * np.sum(in_threshold) / len(in_threshold)

        return TrackingMetrics(
            controller=controller_name,
            rmse_x=rmse_x,
            rmse_y=rmse_y,
            rmse_total=rmse_total,
            mae_x=mae_x,
            mae_y=mae_y,
            peak_error_x=peak_error_x,
            peak_error_y=peak_error_y,
            peak_error_total=peak_error_total,
            steady_state_error=ss_error,
            iae_x=iae_x,
            iae_y=iae_y,
            time_in_threshold=time_in_threshold
        )

    def print_results(self):
        """Print comparison table"""
        print("\n" + "="*70)
        print("  TRACKING PERFORMANCE COMPARISON")
        print("="*70)

        # Header
        print(f"\n{'Metric':<25} {'PID+Fuzzy':>12} {'PD':>12} {'PID':>12}")
        print("-" * 65)

        # Get metrics in order
        order = ['PID+Fuzzy', 'PD', 'PID']
        metrics = {name: self.results[name] for name in order if name in self.results}

        if len(metrics) == 0:
            print("No results to display")
            return

        # Print each metric
        def print_row(label, attr, fmt=".4f"):
            values = [getattr(metrics[n], attr) if n in metrics else 0 for n in order]
            # Mark best value (minimum for errors)
            best_idx = np.argmin(values)
            row = f"{label:<25}"
            for i, v in enumerate(values):
                marker = "*" if i == best_idx else " "
                row += f" {v:>10{fmt}}{marker}"
            print(row)

        print_row("RMSE X (m)", "rmse_x")
        print_row("RMSE Y (m)", "rmse_y")
        print_row("RMSE Total (m)", "rmse_total")
        print("-" * 65)
        print_row("MAE X (m)", "mae_x")
        print_row("MAE Y (m)", "mae_y")
        print("-" * 65)
        print_row("Peak Error X (m)", "peak_error_x")
        print_row("Peak Error Y (m)", "peak_error_y")
        print_row("Peak Error Total (m)", "peak_error_total")
        print("-" * 65)
        print_row("Steady-State Error (m)", "steady_state_error")
        print_row("IAE X (m*s)", "iae_x", ".2f")
        print_row("IAE Y (m*s)", "iae_y", ".2f")
        print("-" * 65)

        # Time in threshold (higher is better)
        values = [metrics[n].time_in_threshold if n in metrics else 0 for n in order]
        best_idx = np.argmax(values)
        row = f"{'Time in 0.1m (%)':<25}"
        for i, v in enumerate(values):
            marker = "*" if i == best_idx else " "
            row += f" {v:>10.1f}{marker}"
        print(row)

        print("\n* = Best performance for metric")

    def plot_error_comparison(self):
        """Plot error time series comparison"""
        output_dir = self.data_dir / 'analysis'
        output_dir.mkdir(exist_ok=True)

        fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)

        for controller_name, df in self.data.items():
            time = df['elapsed_time'].values
            color = self.colors[controller_name]

            # Error X
            axes[0].plot(time, df['error_x'].values,
                        label=controller_name, color=color, alpha=0.8, linewidth=1)

            # Error Y
            axes[1].plot(time, df['error_y'].values,
                        label=controller_name, color=color, alpha=0.8, linewidth=1)

            # Error magnitude
            error_mag = np.sqrt(df['error_x'].values**2 + df['error_y'].values**2)
            axes[2].plot(time, error_mag,
                        label=controller_name, color=color, alpha=0.8, linewidth=1)

        axes[0].set_ylabel('Error X (m)', fontsize=11)
        axes[0].axhline(y=0, color='k', linestyle='--', alpha=0.3)
        axes[0].legend(loc='upper right')
        axes[0].grid(True, alpha=0.3)
        axes[0].set_title('Tracking Error Comparison Under Wind Disturbance', fontsize=13, fontweight='bold')

        axes[1].set_ylabel('Error Y (m)', fontsize=11)
        axes[1].axhline(y=0, color='k', linestyle='--', alpha=0.3)
        axes[1].grid(True, alpha=0.3)

        axes[2].set_ylabel('Error Magnitude (m)', fontsize=11)
        axes[2].axhline(y=0.1, color='r', linestyle='--', alpha=0.5, label='Threshold (0.1m)')
        axes[2].set_xlabel('Time (s)', fontsize=11)
        axes[2].grid(True, alpha=0.3)

        plt.tight_layout()
        plot_path = output_dir / 'tracking_error_comparison.png'
        plt.savefig(plot_path, dpi=300, bbox_inches='tight')
        print(f"Saved: {plot_path}")
        plt.close()

    def plot_metrics_bar_chart(self):
        """Bar chart comparing key metrics"""
        output_dir = self.data_dir / 'analysis'
        output_dir.mkdir(exist_ok=True)

        order = ['PID+Fuzzy', 'PD', 'PID']
        metrics_to_plot = [
            ('rmse_total', 'RMSE (m)', 'Total Tracking Error'),
            ('peak_error_total', 'Peak Error (m)', 'Maximum Error (Wind Gust)'),
            ('steady_state_error', 'SS Error (m)', 'Steady-State Error'),
            ('time_in_threshold', 'Time in 0.1m (%)', 'Time Within Threshold')
        ]

        fig, axes = plt.subplots(2, 2, figsize=(12, 8))
        axes = axes.flatten()

        for ax_idx, (attr, ylabel, title) in enumerate(metrics_to_plot):
            ax = axes[ax_idx]
            values = []
            colors_list = []

            for name in order:
                if name in self.results:
                    values.append(getattr(self.results[name], attr))
                    colors_list.append(self.colors[name])
                else:
                    values.append(0)
                    colors_list.append('gray')

            x = np.arange(len(order))
            bars = ax.bar(x, values, color=colors_list, alpha=0.8, edgecolor='black')

            ax.set_xticks(x)
            ax.set_xticklabels(order, fontsize=10)
            ax.set_ylabel(ylabel, fontsize=10)
            ax.set_title(title, fontsize=11, fontweight='bold')
            ax.grid(True, axis='y', alpha=0.3)

            # Add value labels on bars
            for bar, val in zip(bars, values):
                height = bar.get_height()
                ax.annotate(f'{val:.3f}',
                           xy=(bar.get_x() + bar.get_width() / 2, height),
                           xytext=(0, 3),
                           textcoords="offset points",
                           ha='center', va='bottom', fontsize=9)

        plt.tight_layout()
        plot_path = output_dir / 'tracking_metrics_comparison.png'
        plt.savefig(plot_path, dpi=300, bbox_inches='tight')
        print(f"Saved: {plot_path}")
        plt.close()

    def save_csv(self):
        """Save results to CSV"""
        output_dir = self.data_dir / 'analysis'
        output_dir.mkdir(exist_ok=True)

        rows = []
        for name, m in self.results.items():
            rows.append({
                'controller': m.controller,
                'rmse_x': m.rmse_x,
                'rmse_y': m.rmse_y,
                'rmse_total': m.rmse_total,
                'mae_x': m.mae_x,
                'mae_y': m.mae_y,
                'peak_error_x': m.peak_error_x,
                'peak_error_y': m.peak_error_y,
                'peak_error_total': m.peak_error_total,
                'steady_state_error': m.steady_state_error,
                'iae_x': m.iae_x,
                'iae_y': m.iae_y,
                'time_in_threshold_pct': m.time_in_threshold
            })

        df = pd.DataFrame(rows)
        csv_path = output_dir / 'tracking_performance_summary.csv'
        df.to_csv(csv_path, index=False)
        print(f"Saved: {csv_path}")

    def run_analysis(self):
        """Main analysis pipeline"""
        self.load_data()

        for controller_name in self.data.keys():
            metrics = self.analyze_controller(controller_name)
            if metrics:
                self.results[controller_name] = metrics

        self.print_results()
        self.plot_error_comparison()
        self.plot_metrics_bar_chart()
        self.save_csv()

        print("\n" + "="*70)
        print("  ANALYSIS COMPLETE!")
        print("="*70)


def main():
    parser = argparse.ArgumentParser(description='Tracking Performance Analyzer')
    parser.add_argument('data_dir', type=str, help='Directory with CSV files')
    args = parser.parse_args()

    analyzer = TrackingPerformanceAnalyzer(args.data_dir)
    analyzer.run_analysis()


if __name__ == '__main__':
    main()
