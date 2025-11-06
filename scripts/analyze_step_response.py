#!/usr/bin/env python3
"""
Multi-Checkpoint Step Response Analyzer for Formation Control
Adaptive windowing for multiple sequential step responses

Analyzes 4 step responses per controller:
- Step 1 (t=0-15s):   X=-15 → X=-10
- Step 2 (t=15-30s):  X=-10 → X=-5
- Step 3 (t=30-45s):  X=-5  → X=0
- Step 4 (t=45-60s):  X=0   → X=5

Metrics per step:
- Rise Time (10%-90%)
- Overshoot (%)
- Settling Time (±2%)
- Peak Time
- Steady-State Error

Compares: PID, PD, PID+Fuzzy controllers
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import argparse
import sys
from scipy import signal
from dataclasses import dataclass
from typing import List, Dict, Optional


@dataclass
class StepMetrics:
    """Step response metrics for one segment"""
    step_number: int
    step_time: float
    rise_time: float
    overshoot_percent: float
    peak_time: float
    settling_time: float
    steady_state_error: float
    initial_value: float
    final_value: float
    peak_value: float
    step_magnitude: float


class MultiCheckpointStepAnalyzer:
    """Analyze multi-checkpoint step response with adaptive windowing"""

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

        # Expected waypoint times (from config)
        self.waypoint_times = [0.0, 15.0, 30.0, 45.0]
        self.waypoint_positions = [-15.0, -10.0, -5.0, 0.0, 5.0]

    def load_data(self):
        """Load CSV data for all agents"""
        print("\n" + "="*70)
        print("  MULTI-CHECKPOINT STEP RESPONSE ANALYZER")
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

            # Validate required columns
            required_cols = ['elapsed_time', 'current_x', 'target_x']
            if not all(col in df.columns for col in required_cols):
                print(f"⚠ WARNING: Missing required columns in {csv_path.name}")
                print(f"   Required: {required_cols}")
                print(f"   Found: {list(df.columns)}")
                continue

            self.data[controller_name] = df

        print(f"\n✓ Loaded {len(self.data)} controllers\n")

        if len(self.data) == 0:
            print("ERROR: No data found or missing required columns!")
            sys.exit(1)

    def detect_step_transitions(self, time, target_signal, min_step_size=0.5):
        """
        Detect when target makes step changes

        For interpolated waypoints, we look for changes in velocity (acceleration)
        Returns: List of step transition indices and values
        """
        # Option 1: Look for velocity changes (better for smooth trajectories)
        velocity = np.diff(target_signal) / np.diff(time)
        accel = np.diff(velocity)

        # Find where target direction/speed changes significantly
        change_indices = []
        for i in range(len(accel)):
            if abs(accel[i]) > 0.01:  # Acceleration threshold
                # Check if this is start of new segment
                if i == 0 or (i - change_indices[-1] if change_indices else 999) > 200:  # 10s apart
                    change_indices.append(i + 1)

        # If no changes detected, fall back to simple diff
        if len(change_indices) == 0:
            target_diff = np.diff(target_signal)
            step_indices = np.where(np.abs(target_diff) > min_step_size)[0]
        else:
            step_indices = change_indices

        steps = []
        for idx in step_indices:
            if idx + 1 < len(target_signal):
                steps.append({
                    'index': idx,
                    'time': time[idx],
                    'initial': target_signal[idx],
                    'final': target_signal[idx + 1],
                    'magnitude': abs(target_signal[idx + 1] - target_signal[idx])
                })

        return steps

    def analyze_single_step(self, time, response_signal, target_signal,
                           step_info, settling_threshold=0.02):
        """
        Analyze one step response segment with adaptive windowing

        Args:
            time: Time array
            response_signal: Actual position (current_x)
            target_signal: Target position (target_x)
            step_info: Dict with step transition info
            settling_threshold: ±2% settling band
        """
        step_idx = step_info['index']
        step_time = step_info['time']
        initial_target = step_info['initial']
        final_target = step_info['final']
        step_magnitude = step_info['magnitude']

        # Define analysis window
        # Start: 1 second before step (to capture initial state)
        # End: 14 seconds after step (before next step at t+15s)
        window_start_idx = max(0, step_idx - 20)  # 1s before at 20Hz
        window_end_idx = min(len(time) - 1, step_idx + 280)  # 14s after

        # Extract segment
        seg_time = time[window_start_idx:window_end_idx]
        seg_response = response_signal[window_start_idx:window_end_idx]
        seg_target = target_signal[window_start_idx:window_end_idx]

        if len(seg_time) < 10:
            return None

        # Normalize time to step occurrence
        seg_time_norm = seg_time - step_time

        # Find actual step index in segment
        step_in_seg = np.where(seg_time_norm >= 0)[0]
        if len(step_in_seg) == 0:
            return None
        step_seg_idx = step_in_seg[0]

        # Initial value: average 0.5s before step
        pre_step_indices = (seg_time_norm >= -0.5) & (seg_time_norm < 0)
        if np.sum(pre_step_indices) > 0:
            initial_value = np.mean(seg_response[pre_step_indices])
        else:
            initial_value = seg_response[step_seg_idx]

        # Final value: average last 2 seconds of window
        post_step_time = seg_time_norm[step_seg_idx:]
        if len(post_step_time) > 40:  # At least 2s of data
            final_value = np.mean(seg_response[-40:])  # Last 2s at 20Hz
        else:
            final_value = seg_response[-1]

        # Response after step
        response_after_step = seg_response[step_seg_idx:]
        time_after_step = seg_time_norm[step_seg_idx:]

        if len(time_after_step) < 5:
            return None

        # Calculate normalized response (0 to 1)
        if abs(final_target - initial_value) < 0.1:
            # Very small step, avoid division issues
            normalized = response_after_step - initial_value
        else:
            normalized = (response_after_step - initial_value) / (final_target - initial_value)

        # === RISE TIME (10% to 90%) ===
        rise_time = None
        try:
            t10_idx = np.where(normalized >= 0.1)[0]
            t90_idx = np.where(normalized >= 0.9)[0]

            if len(t10_idx) > 0 and len(t90_idx) > 0:
                t10 = time_after_step[t10_idx[0]]
                t90 = time_after_step[t90_idx[0]]
                rise_time = t90 - t10
        except:
            rise_time = None

        # === OVERSHOOT ===
        overshoot_percent = 0.0
        peak_value = initial_value
        peak_time = 0.0

        try:
            # Find peak after reaching 50% of final value
            halfway_idx = np.where(normalized >= 0.5)[0]
            if len(halfway_idx) > 0:
                search_start = halfway_idx[0]
                search_response = response_after_step[search_start:]
                search_time = time_after_step[search_start:]

                if len(search_response) > 0:
                    peak_idx = np.argmax(np.abs(search_response - final_target))
                    peak_value = search_response[peak_idx]
                    peak_time = search_time[peak_idx]

                    # Overshoot percentage relative to step magnitude
                    overshoot = peak_value - final_target
                    if abs(step_magnitude) > 0.1:
                        overshoot_percent = (abs(overshoot) / abs(step_magnitude)) * 100.0
        except:
            pass

        # === SETTLING TIME (±2% band) ===
        settling_time = None
        try:
            settling_band = settling_threshold * abs(step_magnitude)
            error_from_final = np.abs(response_after_step - final_target)

            # Find last time error exceeded band
            exceeded_indices = np.where(error_from_final > settling_band)[0]

            if len(exceeded_indices) > 0:
                settling_time = time_after_step[exceeded_indices[-1]]
            else:
                # Settled immediately
                settling_time = 0.0
        except:
            settling_time = None

        # === STEADY-STATE ERROR ===
        steady_state_error = abs(final_value - final_target)

        return StepMetrics(
            step_number=0,  # Will be set later
            step_time=step_time,
            rise_time=rise_time if rise_time is not None else np.nan,
            overshoot_percent=overshoot_percent,
            peak_time=peak_time,
            settling_time=settling_time if settling_time is not None else np.nan,
            steady_state_error=steady_state_error,
            initial_value=initial_value,
            final_value=final_value,
            peak_value=peak_value,
            step_magnitude=step_magnitude
        )

    def analyze_controller(self, controller_name):
        """Analyze all steps for one controller"""
        df = self.data[controller_name]

        time = df['elapsed_time'].values
        current_x = df['current_x'].values
        target_x = df['target_x'].values

        # Detect step transitions
        steps = self.detect_step_transitions(time, target_x, min_step_size=2.0)

        if len(steps) == 0:
            print(f"⚠ WARNING: No steps detected for {controller_name}")
            return []

        print(f"\n{controller_name}:")
        print(f"  Detected {len(steps)} step transitions")

        # Analyze each step
        all_metrics = []
        for i, step_info in enumerate(steps):
            metrics = self.analyze_single_step(
                time, current_x, target_x, step_info, settling_threshold=0.02
            )

            if metrics:
                metrics.step_number = i + 1
                all_metrics.append(metrics)

                print(f"  Step {i+1} (t={step_info['time']:.1f}s): "
                      f"X={step_info['initial']:.1f}→{step_info['final']:.1f}m")

        return all_metrics

    def print_results(self):
        """Print formatted results table"""
        print("\n" + "="*70)
        print("  STEP RESPONSE ANALYSIS RESULTS")
        print("="*70)

        for controller_name in self.results:
            metrics_list = self.results[controller_name]

            print(f"\n{controller_name}:")
            print("─" * 60)

            if len(metrics_list) == 0:
                print("  No valid step responses analyzed")
                continue

            # Print each step
            for metrics in metrics_list:
                print(f"\n  Step {metrics.step_number} "
                      f"(t={metrics.step_time:.1f}s, Δ={metrics.step_magnitude:.1f}m):")
                print(f"    Rise Time:        {metrics.rise_time:.3f} s" if not np.isnan(metrics.rise_time) else "    Rise Time:        N/A")
                print(f"    Overshoot:        {metrics.overshoot_percent:.2f} %")
                print(f"    Peak Time:        {metrics.peak_time:.3f} s")
                print(f"    Settling Time:    {metrics.settling_time:.3f} s" if not np.isnan(metrics.settling_time) else "    Settling Time:    N/A")
                print(f"    SS Error:         {metrics.steady_state_error:.4f} m")

            # Compute averages
            if len(metrics_list) > 0:
                valid_rise = [m.rise_time for m in metrics_list if not np.isnan(m.rise_time)]
                valid_settling = [m.settling_time for m in metrics_list if not np.isnan(m.settling_time)]

                print(f"\n  Average Across {len(metrics_list)} Steps:")
                if len(valid_rise) > 0:
                    print(f"    Avg Rise Time:    {np.mean(valid_rise):.3f} s (±{np.std(valid_rise):.3f})")
                print(f"    Avg Overshoot:    {np.mean([m.overshoot_percent for m in metrics_list]):.2f} %")
                if len(valid_settling) > 0:
                    print(f"    Avg Settling:     {np.mean(valid_settling):.3f} s (±{np.std(valid_settling):.3f})")
                print(f"    Avg SS Error:     {np.mean([m.steady_state_error for m in metrics_list]):.4f} m")

    def save_results_csv(self):
        """Save detailed results to CSV"""
        output_dir = self.data_dir / 'analysis'
        output_dir.mkdir(exist_ok=True)

        # Detailed metrics per step
        rows = []
        for controller_name, metrics_list in self.results.items():
            for metrics in metrics_list:
                rows.append({
                    'controller': controller_name,
                    'step_number': metrics.step_number,
                    'step_time': metrics.step_time,
                    'step_magnitude': metrics.step_magnitude,
                    'rise_time': metrics.rise_time,
                    'overshoot_percent': metrics.overshoot_percent,
                    'peak_time': metrics.peak_time,
                    'settling_time': metrics.settling_time,
                    'steady_state_error': metrics.steady_state_error,
                    'initial_value': metrics.initial_value,
                    'final_value': metrics.final_value,
                    'peak_value': metrics.peak_value
                })

        df_detailed = pd.DataFrame(rows)
        csv_path = output_dir / 'step_response_detailed_metrics.csv'
        df_detailed.to_csv(csv_path, index=False)
        print(f"\n✓ Saved detailed metrics: {csv_path}")

        # Summary averages
        summary_rows = []
        for controller_name, metrics_list in self.results.items():
            if len(metrics_list) == 0:
                continue

            valid_rise = [m.rise_time for m in metrics_list if not np.isnan(m.rise_time)]
            valid_settling = [m.settling_time for m in metrics_list if not np.isnan(m.settling_time)]

            summary_rows.append({
                'controller': controller_name,
                'num_steps': len(metrics_list),
                'avg_rise_time': np.mean(valid_rise) if len(valid_rise) > 0 else np.nan,
                'std_rise_time': np.std(valid_rise) if len(valid_rise) > 0 else np.nan,
                'avg_overshoot_percent': np.mean([m.overshoot_percent for m in metrics_list]),
                'std_overshoot_percent': np.std([m.overshoot_percent for m in metrics_list]),
                'avg_settling_time': np.mean(valid_settling) if len(valid_settling) > 0 else np.nan,
                'std_settling_time': np.std(valid_settling) if len(valid_settling) > 0 else np.nan,
                'avg_ss_error': np.mean([m.steady_state_error for m in metrics_list]),
                'std_ss_error': np.std([m.steady_state_error for m in metrics_list])
            })

        df_summary = pd.DataFrame(summary_rows)
        csv_path_summary = output_dir / 'step_response_summary.csv'
        df_summary.to_csv(csv_path_summary, index=False)
        print(f"✓ Saved summary metrics: {csv_path_summary}")

    def plot_multi_step_comparison(self):
        """Plot all steps for all controllers"""
        output_dir = self.data_dir / 'analysis'
        output_dir.mkdir(exist_ok=True)

        # Determine max number of steps
        max_steps = max(len(metrics_list) for metrics_list in self.results.values())

        if max_steps == 0:
            print("⚠ No steps to plot")
            return

        fig, axes = plt.subplots(max_steps, 1, figsize=(14, 4 * max_steps))
        if max_steps == 1:
            axes = [axes]

        for step_num in range(1, max_steps + 1):
            ax = axes[step_num - 1]

            for controller_name, metrics_list in self.results.items():
                if step_num <= len(metrics_list):
                    metrics = metrics_list[step_num - 1]

                    # Load data for this controller
                    df = self.data[controller_name]
                    time = df['elapsed_time'].values
                    current_x = df['current_x'].values
                    target_x = df['target_x'].values

                    # Plot window around this step
                    step_time = metrics.step_time
                    window_mask = (time >= step_time - 1) & (time <= step_time + 14)

                    time_window = time[window_mask] - step_time
                    current_window = current_x[window_mask]
                    target_window = target_x[window_mask]

                    # Plot response
                    ax.plot(time_window, current_window,
                           label=f'{controller_name}',
                           color=self.colors[controller_name],
                           linewidth=2, alpha=0.8)

                    # Plot target (dashed)
                    ax.plot(time_window, target_window, '--',
                           color=self.colors[controller_name],
                           linewidth=1, alpha=0.5)

            ax.set_xlabel('Time from step (s)', fontsize=11)
            ax.set_ylabel('Position X (m)', fontsize=11)
            ax.set_title(f'Step {step_num} Response Comparison', fontsize=13, fontweight='bold')
            ax.grid(True, alpha=0.3)
            ax.legend(loc='best', fontsize=10)

        plt.tight_layout()
        plot_path = output_dir / 'multi_step_comparison.png'
        plt.savefig(plot_path, dpi=300, bbox_inches='tight')
        print(f"✓ Saved comparison plot: {plot_path}")
        plt.close()

    def plot_metrics_bar_charts(self):
        """Bar charts comparing metrics across controllers and steps"""
        output_dir = self.data_dir / 'analysis'
        output_dir.mkdir(exist_ok=True)

        # Prepare data for bar chart
        controller_names = list(self.results.keys())
        max_steps = max(len(metrics_list) for metrics_list in self.results.values())

        if max_steps == 0:
            return

        metrics_to_plot = [
            ('rise_time', 'Rise Time (s)', 'rise_time_comparison.png'),
            ('overshoot_percent', 'Overshoot (%)', 'overshoot_comparison.png'),
            ('settling_time', 'Settling Time (s)', 'settling_time_comparison.png')
        ]

        for metric_name, ylabel, filename in metrics_to_plot:
            fig, ax = plt.subplots(figsize=(12, 6))

            x = np.arange(max_steps)
            width = 0.25

            for i, controller_name in enumerate(controller_names):
                metrics_list = self.results[controller_name]
                values = []

                for step_num in range(1, max_steps + 1):
                    if step_num <= len(metrics_list):
                        val = getattr(metrics_list[step_num - 1], metric_name)
                        values.append(val if not np.isnan(val) else 0)
                    else:
                        values.append(0)

                offset = width * (i - 1)
                ax.bar(x + offset, values, width,
                      label=controller_name,
                      color=self.colors[controller_name],
                      alpha=0.8)

            ax.set_xlabel('Step Number', fontsize=12, fontweight='bold')
            ax.set_ylabel(ylabel, fontsize=12, fontweight='bold')
            ax.set_title(f'{ylabel} Comparison Across Steps', fontsize=14, fontweight='bold')
            ax.set_xticks(x)
            ax.set_xticklabels([f'Step {i+1}' for i in range(max_steps)])
            ax.legend(fontsize=11)
            ax.grid(True, axis='y', alpha=0.3)

            plt.tight_layout()
            plot_path = output_dir / filename
            plt.savefig(plot_path, dpi=300, bbox_inches='tight')
            print(f"✓ Saved {ylabel.lower()} plot: {plot_path}")
            plt.close()

    def run_analysis(self):
        """Main analysis pipeline"""
        self.load_data()

        # Analyze each controller
        for controller_name in self.data.keys():
            metrics_list = self.analyze_controller(controller_name)
            self.results[controller_name] = metrics_list

        # Output results
        self.print_results()
        self.save_results_csv()
        self.plot_multi_step_comparison()
        self.plot_metrics_bar_charts()

        print("\n" + "="*70)
        print("  ANALYSIS COMPLETE!")
        print("="*70)
        print(f"\nResults saved to: {self.data_dir / 'analysis'}/")
        print("\nGenerated files:")
        print("  • step_response_detailed_metrics.csv  - Per-step metrics")
        print("  • step_response_summary.csv           - Average metrics")
        print("  • multi_step_comparison.png           - Response curves")
        print("  • rise_time_comparison.png            - Rise time bars")
        print("  • overshoot_comparison.png            - Overshoot bars")
        print("  • settling_time_comparison.png        - Settling time bars")
        print()


def main():
    parser = argparse.ArgumentParser(
        description='Multi-Checkpoint Step Response Analyzer'
    )
    parser.add_argument(
        'data_dir',
        type=str,
        help='Directory containing final_results CSV files'
    )

    args = parser.parse_args()

    analyzer = MultiCheckpointStepAnalyzer(args.data_dir)
    analyzer.run_analysis()


if __name__ == '__main__':
    main()
