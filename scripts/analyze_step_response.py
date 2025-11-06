#!/usr/bin/env python3
"""
Step Response Analyzer for Multi-Agent Formation Control
MATLAB stepinfo() benzeri analiz - Publication-ready results

Metrics Calculated:
- Settling Time: Time to enter and stay within ±2% of final value
- Overshoot: Maximum overshoot percentage
- Rise Time: Time from 10% to 90% of final value
- Peak Time: Time to first peak
- Steady-State Error: Final error from target

Compares: PID, PD, PID+Fuzzy controllers
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import argparse
import sys
from scipy import signal


class StepResponseAnalyzer:
    """Analyze step response characteristics of formation controllers"""

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
        print("  STEP RESPONSE ANALYZER")
        print("="*70)
        print(f"Loading data from: {self.data_dir}\n")

        self.data = {}

        for agent_id, controller_name in self.controllers.items():
            # Find CSV file
            csv_pattern = f"{agent_id}_*_full.csv"
            csv_files = list(self.data_dir.glob(f"**/{csv_pattern}"))

            if not csv_files:
                print(f"⚠ WARNING: No data found for {agent_id} ({controller_name})")
                continue

            csv_path = csv_files[0]
            print(f"✓ Loading {agent_id} ({controller_name}): {csv_path.name}")

            df = pd.read_csv(csv_path)
            self.data[controller_name] = df

        print(f"\n✓ Loaded {len(self.data)} controllers\n")

        if len(self.data) == 0:
            print("ERROR: No data found!")
            sys.exit(1)

    def analyze_step_response(self, time, signal_data, step_target, threshold=0.02):
        """
        Analyze step response characteristics (MATLAB stepinfo equivalent)

        Args:
            time: Time array
            signal_data: Signal data array
            step_target: Target value for step input
            threshold: Settling threshold (default: 2% = 0.02)

        Returns:
            dict with step response metrics
        """
        # Find initial and final values
        initial_value = np.mean(signal_data[:10])  # Average first 10 samples
        final_value = np.mean(signal_data[-50:])   # Average last 50 samples

        # Normalize to step response (0 to 1)
        if abs(step_target - initial_value) < 1e-6:
            # No step occurred
            return None

        normalized = (signal_data - initial_value) / (step_target - initial_value)

        # 1. Rise Time (10% to 90%)
        rise_time = self._calculate_rise_time(time, normalized)

        # 2. Overshoot
        peak_value = np.max(normalized)
        overshoot_percent = (peak_value - 1.0) * 100 if peak_value > 1.0 else 0.0

        # 3. Peak Time
        peak_idx = np.argmax(normalized)
        peak_time = time[peak_idx]

        # 4. Settling Time (±2% band)
        settling_time = self._calculate_settling_time(time, normalized, threshold)

        # 5. Steady-State Error
        steady_state_error = abs(final_value - step_target)
        steady_state_error_percent = (steady_state_error / abs(step_target)) * 100 if step_target != 0 else 0

        return {
            'rise_time': rise_time,
            'overshoot_percent': overshoot_percent,
            'peak_time': peak_time,
            'settling_time': settling_time,
            'steady_state_error': steady_state_error,
            'steady_state_error_percent': steady_state_error_percent,
            'initial_value': initial_value,
            'final_value': final_value,
            'peak_value': peak_value * (step_target - initial_value) + initial_value
        }

    def _calculate_rise_time(self, time, normalized_signal):
        """Calculate rise time (10% to 90%)"""
        # Find first crossing of 10%
        idx_10 = np.where(normalized_signal >= 0.1)[0]
        if len(idx_10) == 0:
            return np.nan
        t_10 = time[idx_10[0]]

        # Find first crossing of 90%
        idx_90 = np.where(normalized_signal >= 0.9)[0]
        if len(idx_90) == 0:
            return np.nan
        t_90 = time[idx_90[0]]

        return t_90 - t_10

    def _calculate_settling_time(self, time, normalized_signal, threshold=0.02):
        """Calculate settling time (time to stay within ±threshold of final value)"""
        # Find where signal enters and stays in ±threshold band around 1.0
        in_band = np.abs(normalized_signal - 1.0) <= threshold

        # Find last point outside the band
        outside_band_indices = np.where(~in_band)[0]

        if len(outside_band_indices) == 0:
            # Signal always in band
            return 0.0

        # Settling time is when it enters band and never leaves
        last_outside = outside_band_indices[-1]

        # Check if signal stays in band after this point
        if last_outside < len(in_band) - 10:  # At least 10 samples in band
            return time[last_outside + 1]
        else:
            # Signal didn't settle
            return np.nan

    def analyze_all_controllers(self):
        """Analyze step response for all controllers"""
        print("="*70)
        print("  STEP RESPONSE ANALYSIS")
        print("="*70 + "\n")

        for controller_name, df in self.data.items():
            print(f"Analyzing {controller_name}...")

            time = df['elapsed_time'].values

            # Analyze X-axis step response (main motion axis)
            error_x = df['error_x'].values

            # Detect step target from data
            # Assuming formation starts at -15 and moves to +5 (20m step)
            initial_x = error_x[0]
            final_x = np.mean(error_x[-50:])
            step_target = 0.0  # Target error is 0

            # Analyze position tracking (use error magnitude)
            error_magnitude = df['error_magnitude'].values

            metrics = self.analyze_step_response(
                time,
                error_magnitude,
                step_target=0.0,  # Target is zero error
                threshold=0.02
            )

            if metrics:
                self.results[controller_name] = metrics
                self._print_metrics(controller_name, metrics)
            else:
                print(f"  ⚠ No valid step response detected\n")

        print("\n" + "="*70)
        print("  COMPARISON TABLE")
        print("="*70 + "\n")
        self._print_comparison_table()

    def _print_metrics(self, controller_name, metrics):
        """Print metrics for a controller"""
        print(f"\n  {controller_name} Results:")
        print(f"  {'─'*60}")
        print(f"    Rise Time:            {metrics['rise_time']:.3f} s")
        print(f"    Overshoot:            {metrics['overshoot_percent']:.2f} %")
        print(f"    Peak Time:            {metrics['peak_time']:.3f} s")
        print(f"    Settling Time (±2%):  {metrics['settling_time']:.3f} s")
        print(f"    Steady-State Error:   {metrics['steady_state_error']:.4f} m")
        print(f"    SS Error (%):         {metrics['steady_state_error_percent']:.2f} %")
        print()

    def _print_comparison_table(self):
        """Print comparison table"""
        if not self.results:
            print("  No results to compare!\n")
            return

        # Header
        print(f"  {'Metric':<25} | {'PID':<12} | {'PD':<12} | {'PID+Fuzzy':<12}")
        print(f"  {'-'*25}|{'-'*14}|{'-'*14}|{'-'*14}")

        # Metrics to compare
        metrics_info = [
            ('Rise Time (s)', 'rise_time', '{:.3f}'),
            ('Overshoot (%)', 'overshoot_percent', '{:.2f}'),
            ('Peak Time (s)', 'peak_time', '{:.3f}'),
            ('Settling Time (s)', 'settling_time', '{:.3f}'),
            ('SS Error (m)', 'steady_state_error', '{:.4f}'),
            ('SS Error (%)', 'steady_state_error_percent', '{:.2f}')
        ]

        for metric_name, metric_key, fmt in metrics_info:
            values = {}
            for ctrl in ['PID', 'PD', 'PID+Fuzzy']:
                if ctrl in self.results and metric_key in self.results[ctrl]:
                    val = self.results[ctrl][metric_key]
                    if np.isnan(val):
                        values[ctrl] = 'N/A'
                    else:
                        values[ctrl] = fmt.format(val)
                else:
                    values[ctrl] = 'N/A'

            print(f"  {metric_name:<25} | {values.get('PID', 'N/A'):>12} | "
                  f"{values.get('PD', 'N/A'):>12} | {values.get('PID+Fuzzy', 'N/A'):>12}")

        print()

    def plot_step_responses(self, output_dir=None):
        """Create publication-ready step response plots"""
        if not self.data:
            print("⚠ No data to plot!")
            return

        print("="*70)
        print("  GENERATING PLOTS")
        print("="*70 + "\n")

        # Create output directory
        if output_dir is None:
            output_dir = self.data_dir / 'plots'
        else:
            output_dir = Path(output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)

        # 1. Error magnitude comparison
        self._plot_error_magnitude(output_dir)

        # 2. Step response comparison (normalized)
        self._plot_normalized_response(output_dir)

        # 3. Metrics bar chart
        self._plot_metrics_comparison(output_dir)

        # 4. Individual controller plots
        for controller_name in self.data.keys():
            self._plot_individual_controller(controller_name, output_dir)

        print(f"\n✓ Plots saved to: {output_dir}\n")

    def _plot_error_magnitude(self, output_dir):
        """Plot error magnitude comparison"""
        fig, ax = plt.subplots(figsize=(12, 6))

        for controller_name, df in self.data.items():
            time = df['elapsed_time'].values
            error = df['error_magnitude'].values

            ax.plot(time, error,
                   label=controller_name,
                   color=self.colors[controller_name],
                   linewidth=2)

        ax.set_xlabel('Time (s)', fontsize=12)
        ax.set_ylabel('Formation Error (m)', fontsize=12)
        ax.set_title('Step Response: Formation Error Magnitude', fontsize=14, fontweight='bold')
        ax.legend(fontsize=11, loc='upper right')
        ax.grid(True, alpha=0.3)
        ax.set_xlim(0, None)
        ax.set_ylim(0, None)

        plt.tight_layout()
        plot_path = output_dir / 'step_response_error_magnitude.png'
        plt.savefig(plot_path, dpi=300, bbox_inches='tight')
        plt.close()

        print(f"  ✓ Error magnitude plot: {plot_path.name}")

    def _plot_normalized_response(self, output_dir):
        """Plot normalized step response"""
        fig, ax = plt.subplots(figsize=(12, 6))

        for controller_name, df in self.data.items():
            time = df['elapsed_time'].values
            error = df['error_magnitude'].values

            # Normalize
            initial = np.mean(error[:10])
            final = np.mean(error[-50:])

            if abs(initial) < 1e-6:
                normalized = error
            else:
                normalized = error / initial

            ax.plot(time, normalized,
                   label=controller_name,
                   color=self.colors[controller_name],
                   linewidth=2)

        # Add ±2% settling band
        ax.axhline(y=0.02, color='gray', linestyle='--', linewidth=1, alpha=0.5, label='±2% band')
        ax.axhline(y=-0.02, color='gray', linestyle='--', linewidth=1, alpha=0.5)

        ax.set_xlabel('Time (s)', fontsize=12)
        ax.set_ylabel('Normalized Error', fontsize=12)
        ax.set_title('Step Response: Normalized Error', fontsize=14, fontweight='bold')
        ax.legend(fontsize=11, loc='upper right')
        ax.grid(True, alpha=0.3)
        ax.set_xlim(0, None)

        plt.tight_layout()
        plot_path = output_dir / 'step_response_normalized.png'
        plt.savefig(plot_path, dpi=300, bbox_inches='tight')
        plt.close()

        print(f"  ✓ Normalized response plot: {plot_path.name}")

    def _plot_metrics_comparison(self, output_dir):
        """Plot metrics comparison bar chart"""
        if not self.results:
            return

        fig, axes = plt.subplots(2, 3, figsize=(16, 10))
        axes = axes.flatten()

        metrics_info = [
            ('Rise Time (s)', 'rise_time'),
            ('Overshoot (%)', 'overshoot_percent'),
            ('Peak Time (s)', 'peak_time'),
            ('Settling Time (s)', 'settling_time'),
            ('Steady-State Error (m)', 'steady_state_error'),
            ('SS Error (%)', 'steady_state_error_percent')
        ]

        for idx, (metric_name, metric_key) in enumerate(metrics_info):
            ax = axes[idx]

            controllers = []
            values = []
            colors = []

            for ctrl_name in ['PID', 'PD', 'PID+Fuzzy']:
                if ctrl_name in self.results and metric_key in self.results[ctrl_name]:
                    val = self.results[ctrl_name][metric_key]
                    if not np.isnan(val):
                        controllers.append(ctrl_name)
                        values.append(val)
                        colors.append(self.colors[ctrl_name])

            if controllers:
                bars = ax.bar(controllers, values, color=colors, alpha=0.7, edgecolor='black')

                # Add value labels on bars
                for bar in bars:
                    height = bar.get_height()
                    ax.text(bar.get_x() + bar.get_width()/2., height,
                           f'{height:.3f}',
                           ha='center', va='bottom', fontsize=10)

            ax.set_ylabel(metric_name, fontsize=11)
            ax.set_title(metric_name, fontsize=12, fontweight='bold')
            ax.grid(True, alpha=0.3, axis='y')

        plt.suptitle('Step Response Metrics Comparison', fontsize=16, fontweight='bold', y=1.00)
        plt.tight_layout()

        plot_path = output_dir / 'step_response_metrics_comparison.png'
        plt.savefig(plot_path, dpi=300, bbox_inches='tight')
        plt.close()

        print(f"  ✓ Metrics comparison plot: {plot_path.name}")

    def _plot_individual_controller(self, controller_name, output_dir):
        """Plot detailed analysis for individual controller"""
        df = self.data[controller_name]

        fig, axes = plt.subplots(2, 2, figsize=(14, 10))

        time = df['elapsed_time'].values

        # 1. Error magnitude
        ax = axes[0, 0]
        ax.plot(time, df['error_magnitude'].values,
               color=self.colors[controller_name], linewidth=2)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Error Magnitude (m)')
        ax.set_title('Formation Error')
        ax.grid(True, alpha=0.3)

        # 2. X-Y errors
        ax = axes[0, 1]
        ax.plot(time, df['error_x'].values, label='Error X', linewidth=2)
        ax.plot(time, df['error_y'].values, label='Error Y', linewidth=2)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Error (m)')
        ax.set_title('X-Y Position Errors')
        ax.legend()
        ax.grid(True, alpha=0.3)

        # 3. RMSE
        ax = axes[1, 0]
        ax.plot(time, df['rmse_total'].values,
               color=self.colors[controller_name], linewidth=2)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('RMSE (m)')
        ax.set_title('Root Mean Square Error')
        ax.grid(True, alpha=0.3)

        # 4. Settling status
        ax = axes[1, 1]
        ax.plot(time, df['is_settled'].values,
               color=self.colors[controller_name], linewidth=2)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Settled (0/1)')
        ax.set_title('Settling Status')
        ax.set_ylim(-0.1, 1.1)
        ax.grid(True, alpha=0.3)

        plt.suptitle(f'{controller_name} - Detailed Step Response',
                    fontsize=16, fontweight='bold')
        plt.tight_layout()

        plot_path = output_dir / f'step_response_{controller_name.lower().replace("+", "_")}_detail.png'
        plt.savefig(plot_path, dpi=300, bbox_inches='tight')
        plt.close()

        print(f"  ✓ {controller_name} detail plot: {plot_path.name}")

    def save_results(self, output_dir=None):
        """Save analysis results to CSV and text files"""
        if output_dir is None:
            output_dir = self.data_dir
        else:
            output_dir = Path(output_dir)

        output_dir.mkdir(parents=True, exist_ok=True)

        # Save metrics to CSV
        if self.results:
            results_df = pd.DataFrame(self.results).T
            csv_path = output_dir / 'step_response_metrics.csv'
            results_df.to_csv(csv_path)
            print(f"  ✓ Metrics saved: {csv_path.name}")

        # Save summary report
        report_path = output_dir / 'step_response_summary.txt'
        with open(report_path, 'w') as f:
            f.write("="*70 + "\n")
            f.write("  STEP RESPONSE ANALYSIS SUMMARY\n")
            f.write("="*70 + "\n\n")

            for controller_name, metrics in self.results.items():
                f.write(f"{controller_name}:\n")
                f.write(f"{'─'*60}\n")
                for key, value in metrics.items():
                    f.write(f"  {key:<30}: {value:.4f}\n")
                f.write("\n")

        print(f"  ✓ Summary saved: {report_path.name}")


def main():
    parser = argparse.ArgumentParser(
        description='Step Response Analyzer for Multi-Agent Formation Control'
    )
    parser.add_argument('data_dir', type=str,
                       help='Directory containing CSV data files')
    parser.add_argument('--output-dir', '-o', type=str, default=None,
                       help='Output directory for plots and results (default: data_dir/plots)')
    parser.add_argument('--no-plots', action='store_true',
                       help='Skip plot generation')

    args = parser.parse_args()

    # Initialize analyzer
    analyzer = StepResponseAnalyzer(args.data_dir)

    # Load data
    analyzer.load_data()

    # Analyze
    analyzer.analyze_all_controllers()

    # Generate plots
    if not args.no_plots:
        analyzer.plot_step_responses(args.output_dir)

    # Save results
    analyzer.save_results(args.output_dir or analyzer.data_dir)

    print("\n" + "="*70)
    print("  ANALYSIS COMPLETE!")
    print("="*70 + "\n")


if __name__ == '__main__':
    main()
