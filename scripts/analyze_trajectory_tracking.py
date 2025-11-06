#!/usr/bin/env python3
"""
3D Trajectory Tracking Performance Analyzer
Analyzes RMSE, IAE, ITAE, max errors for zigzag trajectory

Compares: PID, PD, PID+Fuzzy controllers
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import argparse
import sys


class TrajectoryTrackingAnalyzer:
    """Analyze trajectory tracking performance"""

    def __init__(self, data_dir):
        self.data_dir = Path(data_dir)
        self.results = {}

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
        print("  3D TRAJECTORY TRACKING ANALYZER")
        print("="*70)
        print(f"Loading data from: {self.data_dir}\n")

        self.data = {}

        for agent_id, controller_name in self.controllers.items():
            csv_pattern = f"{agent_id}_*_full.csv"
            csv_files = list(self.data_dir.glob(f"**/{csv_pattern}"))

            if not csv_files:
                print(f"⚠ WARNING: No data found for {agent_id} ({controller_name})")
                continue

            csv_path = csv_files[0]
            print(f"✓ Loading {agent_id} ({controller_name}): {csv_path.name}")

            df = pd.read_csv(csv_path)
            required_cols = ['elapsed_time', 'current_x', 'current_y', 'current_z',
                           'target_x', 'target_y', 'target_z', 'error_magnitude']

            if not all(col in df.columns for col in required_cols):
                print(f"⚠ WARNING: Missing columns in {csv_path.name}")
                continue

            self.data[controller_name] = df

        print(f"\n✓ Loaded {len(self.data)} controllers\n")

        if len(self.data) == 0:
            print("ERROR: No data found!")
            sys.exit(1)

    def analyze_controller(self, controller_name):
        """Analyze tracking performance for one controller"""
        df = self.data[controller_name]

        time = df['elapsed_time'].values
        error_x = df['error_x'].values if 'error_x' in df.columns else np.zeros(len(df))
        error_y = df['error_y'].values if 'error_y' in df.columns else np.zeros(len(df))
        error_z = df['error_z'].values if 'error_z' in df.columns else np.zeros(len(df))
        error_mag = df['error_magnitude'].values

        # Compute tracking metrics
        rmse_x = np.sqrt(np.mean(error_x**2))
        rmse_y = np.sqrt(np.mean(error_y**2))
        rmse_z = np.sqrt(np.mean(error_z**2))
        rmse_3d = np.sqrt(np.mean(error_mag**2))

        # IAE (Integral Absolute Error)
        dt = np.diff(time)
        dt = np.append(dt, dt[-1])  # Pad for last element
        iae_x = np.sum(np.abs(error_x) * dt)
        iae_y = np.sum(np.abs(error_y) * dt)
        iae_z = np.sum(np.abs(error_z) * dt)
        iae_3d = np.sum(error_mag * dt)

        # ISE (Integral Square Error)
        ise_x = np.sum((error_x**2) * dt)
        ise_y = np.sum((error_y**2) * dt)
        ise_z = np.sum((error_z**2) * dt)
        ise_3d = np.sum((error_mag**2) * dt)

        # Max errors
        max_error_x = np.max(np.abs(error_x))
        max_error_y = np.max(np.abs(error_y))
        max_error_z = np.max(np.abs(error_z))
        max_error_3d = np.max(error_mag)

        # Mean absolute errors
        mae_x = np.mean(np.abs(error_x))
        mae_y = np.mean(np.abs(error_y))
        mae_z = np.mean(np.abs(error_z))
        mae_3d = np.mean(error_mag)

        # Settling characteristics
        settled_threshold = 0.1  # 10cm
        settled_mask = error_mag < settled_threshold
        percent_settled = 100 * np.sum(settled_mask) / len(error_mag)

        # Time to first settle
        first_settled_idx = np.where(settled_mask)[0]
        time_to_settle = time[first_settled_idx[0]] if len(first_settled_idx) > 0 else np.nan

        return {
            'rmse_x': rmse_x,
            'rmse_y': rmse_y,
            'rmse_z': rmse_z,
            'rmse_3d': rmse_3d,
            'iae_x': iae_x,
            'iae_y': iae_y,
            'iae_z': iae_z,
            'iae_3d': iae_3d,
            'ise_x': ise_x,
            'ise_y': ise_y,
            'ise_z': ise_z,
            'ise_3d': ise_3d,
            'max_error_x': max_error_x,
            'max_error_y': max_error_y,
            'max_error_z': max_error_z,
            'max_error_3d': max_error_3d,
            'mae_x': mae_x,
            'mae_y': mae_y,
            'mae_z': mae_z,
            'mae_3d': mae_3d,
            'percent_settled': percent_settled,
            'time_to_settle': time_to_settle,
            'duration': time[-1]
        }

    def print_results(self):
        """Print formatted comparison table"""
        print("\n" + "="*70)
        print("  TRAJECTORY TRACKING PERFORMANCE COMPARISON")
        print("="*70)

        # Print detailed metrics for each controller
        for controller_name, metrics in self.results.items():
            print(f"\n{controller_name}:")
            print("─" * 60)
            print(f"  RMSE:           X={metrics['rmse_x']:.4f}m  Y={metrics['rmse_y']:.4f}m  Z={metrics['rmse_z']:.4f}m  3D={metrics['rmse_3d']:.4f}m")
            print(f"  MAE:            X={metrics['mae_x']:.4f}m  Y={metrics['mae_y']:.4f}m  Z={metrics['mae_z']:.4f}m  3D={metrics['mae_3d']:.4f}m")
            print(f"  Max Error:      X={metrics['max_error_x']:.4f}m  Y={metrics['max_error_y']:.4f}m  Z={metrics['max_error_z']:.4f}m  3D={metrics['max_error_3d']:.4f}m")
            print(f"  IAE:            X={metrics['iae_x']:.2f}  Y={metrics['iae_y']:.2f}  Z={metrics['iae_z']:.2f}  3D={metrics['iae_3d']:.2f}")
            print(f"  ISE:            {metrics['ise_3d']:.2f}")
            print(f"  Time to Settle: {metrics['time_to_settle']:.2f}s")
            print(f"  % Time Settled: {metrics['percent_settled']:.1f}%")

        # Comparison summary
        print("\n" + "="*70)
        print("  PERFORMANCE RANKING (Lower is Better)")
        print("="*70)

        metrics_to_compare = ['rmse_3d', 'mae_3d', 'max_error_3d', 'iae_3d', 'ise_3d']
        metric_names = ['RMSE', 'MAE', 'Max Error', 'IAE', 'ISE']

        for metric, name in zip(metrics_to_compare, metric_names):
            sorted_controllers = sorted(self.results.items(), key=lambda x: x[1][metric])
            print(f"\n{name}:")
            for rank, (controller, metrics) in enumerate(sorted_controllers, 1):
                print(f"  {rank}. {controller:12s}: {metrics[metric]:.4f}")

    def save_results_csv(self):
        """Save results to CSV"""
        output_dir = self.data_dir / 'analysis'
        output_dir.mkdir(exist_ok=True)

        # Create comparison dataframe
        rows = []
        for controller_name, metrics in self.results.items():
            row = {'controller': controller_name}
            row.update(metrics)
            rows.append(row)

        df = pd.DataFrame(rows)
        csv_path = output_dir / 'trajectory_tracking_comparison.csv'
        df.to_csv(csv_path, index=False)
        print(f"\n✓ Saved comparison: {csv_path}")

    def plot_tracking_errors(self):
        """Plot tracking errors over time"""
        output_dir = self.data_dir / 'analysis'
        output_dir.mkdir(exist_ok=True)

        fig, axes = plt.subplots(4, 1, figsize=(14, 12))

        for controller_name in self.data.keys():
            df = self.data[controller_name]
            time = df['elapsed_time'].values

            # Error magnitude
            axes[0].plot(time, df['error_magnitude'].values,
                        label=controller_name, color=self.colors[controller_name],
                        linewidth=1.5, alpha=0.8)

            # X error
            if 'error_x' in df.columns:
                axes[1].plot(time, df['error_x'].values,
                            label=controller_name, color=self.colors[controller_name],
                            linewidth=1.5, alpha=0.8)

            # Y error
            if 'error_y' in df.columns:
                axes[2].plot(time, df['error_y'].values,
                            label=controller_name, color=self.colors[controller_name],
                            linewidth=1.5, alpha=0.8)

            # Z error
            if 'error_z' in df.columns:
                axes[3].plot(time, df['error_z'].values,
                            label=controller_name, color=self.colors[controller_name],
                            linewidth=1.5, alpha=0.8)

        axes[0].set_ylabel('3D Error (m)', fontsize=11, fontweight='bold')
        axes[0].set_title('3D Trajectory Tracking Error Comparison', fontsize=13, fontweight='bold')
        axes[0].grid(True, alpha=0.3)
        axes[0].legend(loc='best', fontsize=10)
        axes[0].axhline(y=0.1, color='gray', linestyle='--', alpha=0.5, label='Settling threshold')

        axes[1].set_ylabel('X Error (m)', fontsize=11, fontweight='bold')
        axes[1].grid(True, alpha=0.3)
        axes[1].legend(loc='best', fontsize=10)

        axes[2].set_ylabel('Y Error (m)', fontsize=11, fontweight='bold')
        axes[2].grid(True, alpha=0.3)
        axes[2].legend(loc='best', fontsize=10)

        axes[3].set_ylabel('Z Error (m)', fontsize=11, fontweight='bold')
        axes[3].set_xlabel('Time (s)', fontsize=11, fontweight='bold')
        axes[3].grid(True, alpha=0.3)
        axes[3].legend(loc='best', fontsize=10)

        plt.tight_layout()
        plot_path = output_dir / 'tracking_errors_time.png'
        plt.savefig(plot_path, dpi=300, bbox_inches='tight')
        print(f"✓ Saved error plot: {plot_path}")
        plt.close()

    def plot_comparison_bars(self):
        """Bar charts comparing key metrics"""
        output_dir = self.data_dir / 'analysis'
        output_dir.mkdir(exist_ok=True)

        metrics_to_plot = [
            ('rmse_3d', 'RMSE (m)', 'rmse_comparison.png'),
            ('mae_3d', 'MAE (m)', 'mae_comparison.png'),
            ('max_error_3d', 'Max Error (m)', 'max_error_comparison.png'),
            ('iae_3d', 'IAE (m·s)', 'iae_comparison.png')
        ]

        for metric, ylabel, filename in metrics_to_plot:
            fig, ax = plt.subplots(figsize=(10, 6))

            controllers = list(self.results.keys())
            values = [self.results[c][metric] for c in controllers]
            colors = [self.colors[c] for c in controllers]

            bars = ax.bar(controllers, values, color=colors, alpha=0.8, edgecolor='black', linewidth=1.5)

            # Add value labels on bars
            for bar in bars:
                height = bar.get_height()
                ax.text(bar.get_x() + bar.get_width()/2., height,
                       f'{height:.4f}',
                       ha='center', va='bottom', fontsize=11, fontweight='bold')

            ax.set_ylabel(ylabel, fontsize=12, fontweight='bold')
            ax.set_title(f'{ylabel} Comparison', fontsize=14, fontweight='bold')
            ax.grid(True, axis='y', alpha=0.3)

            plt.tight_layout()
            plot_path = output_dir / filename
            plt.savefig(plot_path, dpi=300, bbox_inches='tight')
            print(f"✓ Saved comparison: {plot_path}")
            plt.close()

    def plot_3d_trajectory(self):
        """3D trajectory visualization"""
        output_dir = self.data_dir / 'analysis'
        output_dir.mkdir(exist_ok=True)

        fig = plt.figure(figsize=(14, 10))
        ax = fig.add_subplot(111, projection='3d')

        for controller_name in self.data.keys():
            df = self.data[controller_name]

            current_x = df['current_x'].values
            current_y = df['current_y'].values
            current_z = df['current_z'].values

            target_x = df['target_x'].values
            target_y = df['target_y'].values
            target_z = df['target_z'].values

            # Plot actual trajectory
            ax.plot(current_x, current_y, current_z,
                   label=f'{controller_name} (actual)',
                   color=self.colors[controller_name],
                   linewidth=2, alpha=0.8)

            # Plot target (only once)
            if controller_name == list(self.data.keys())[0]:
                ax.plot(target_x, target_y, target_z,
                       label='Target trajectory',
                       color='black', linestyle='--',
                       linewidth=2, alpha=0.5)

        ax.set_xlabel('X Position (m)', fontsize=11, fontweight='bold')
        ax.set_ylabel('Y Position (m)', fontsize=11, fontweight='bold')
        ax.set_zlabel('Z Position (m)', fontsize=11, fontweight='bold')
        ax.set_title('3D Zigzag Trajectory Tracking', fontsize=14, fontweight='bold')
        ax.legend(loc='best', fontsize=10)
        ax.grid(True, alpha=0.3)

        plt.tight_layout()
        plot_path = output_dir / 'trajectory_3d.png'
        plt.savefig(plot_path, dpi=300, bbox_inches='tight')
        print(f"✓ Saved 3D trajectory: {plot_path}")
        plt.close()

    def run_analysis(self):
        """Main analysis pipeline"""
        self.load_data()

        # Analyze each controller
        for controller_name in self.data.keys():
            metrics = self.analyze_controller(controller_name)
            self.results[controller_name] = metrics

        # Output results
        self.print_results()
        self.save_results_csv()
        self.plot_tracking_errors()
        self.plot_comparison_bars()
        self.plot_3d_trajectory()

        print("\n" + "="*70)
        print("  ANALYSIS COMPLETE!")
        print("="*70)
        print(f"\nResults saved to: {self.data_dir / 'analysis'}/")
        print("\nGenerated files:")
        print("  • trajectory_tracking_comparison.csv  - Performance metrics")
        print("  • tracking_errors_time.png            - Error vs time")
        print("  • rmse_comparison.png                 - RMSE comparison")
        print("  • mae_comparison.png                  - MAE comparison")
        print("  • max_error_comparison.png            - Max error comparison")
        print("  • iae_comparison.png                  - IAE comparison")
        print("  • trajectory_3d.png                   - 3D trajectory plot")
        print()


def main():
    parser = argparse.ArgumentParser(
        description='3D Trajectory Tracking Performance Analyzer'
    )
    parser.add_argument(
        'data_dir',
        type=str,
        help='Directory containing CSV files'
    )

    args = parser.parse_args()

    analyzer = TrajectoryTrackingAnalyzer(args.data_dir)
    analyzer.run_analysis()


if __name__ == '__main__':
    main()
