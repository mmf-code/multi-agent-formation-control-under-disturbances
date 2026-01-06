#!/usr/bin/env python3
"""Plot phase results - trajectory and performance comparison"""

import os
import sys
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

# Set up paths
PROJECT_ROOT = Path(__file__).parent.parent

def load_agent_data(phase_dir):
    """Load all agent data from a phase directory"""
    agents = {}
    for f in sorted(Path(phase_dir).glob("agent_*.csv")):
        name = f.stem  # e.g., agent_0_pd
        try:
            df = pd.read_csv(f)
            agents[name] = df
        except Exception as e:
            print(f"Error loading {f}: {e}")
    return agents

def get_group_mapping():
    """Return agent to group mapping"""
    return {
        'agent_0_pd': 'PD', 'agent_1_pd': 'PD', 'agent_2_pd': 'PD',
        'agent_3_pid': 'PID', 'agent_4_pid': 'PID', 'agent_5_pid': 'PID',
        'agent_6_it2': 'IT2', 'agent_7_it2': 'IT2', 'agent_8_it2': 'IT2',
        'agent_9_gt2': 'GT2', 'agent_10_gt2': 'GT2', 'agent_11_gt2': 'GT2',
    }

def plot_trajectories(agents, title, output_file):
    """Plot X-Y trajectories for all groups"""
    group_map = get_group_mapping()
    colors = {'PD': 'blue', 'PID': 'green', 'IT2': 'orange', 'GT2': 'red'}

    fig, axes = plt.subplots(2, 2, figsize=(14, 12))
    axes = axes.flatten()

    groups = ['PD', 'PID', 'IT2', 'GT2']

    for idx, group in enumerate(groups):
        ax = axes[idx]
        group_agents = [k for k, v in group_map.items() if v == group]

        for agent_name in group_agents:
            if agent_name not in agents:
                continue
            df = agents[agent_name]

            # Get position columns (handle different naming conventions)
            if 'current_x' in df.columns:
                x_col, y_col = 'current_x', 'current_y'
            elif 'pos_x' in df.columns:
                x_col, y_col = 'pos_x', 'pos_y'
            else:
                x_col, y_col = 'x', 'y'

            tx_col = 'target_x' if 'target_x' in df.columns else None
            ty_col = 'target_y' if 'target_y' in df.columns else None

            if x_col in df.columns and y_col in df.columns:
                # Actual trajectory
                ax.plot(df[x_col].values, df[y_col].values,
                       color=colors[group], alpha=0.7, linewidth=1,
                       label=f'{agent_name} actual')

                # Target trajectory (if available)
                if tx_col and tx_col in df.columns:
                    ax.plot(df[tx_col].values, df[ty_col].values,
                           '--', color='gray', alpha=0.5, linewidth=0.5)

                # Mark start and end
                ax.scatter(df[x_col].values[0], df[y_col].values[0],
                          marker='o', s=50, color='green', zorder=5)
                ax.scatter(df[x_col].values[-1], df[y_col].values[-1],
                          marker='x', s=50, color='red', zorder=5)

        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title(f'{group} Group Trajectories')
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')
        ax.legend(fontsize=8)

    plt.suptitle(title, fontsize=14)
    plt.tight_layout()
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"Saved: {output_file}")
    plt.close()

def plot_error_over_time(agents, title, output_file):
    """Plot position error over time for each group"""
    group_map = get_group_mapping()
    colors = {'PD': 'blue', 'PID': 'green', 'IT2': 'orange', 'GT2': 'red'}

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    axes = axes.flatten()

    groups = ['PD', 'PID', 'IT2', 'GT2']

    for idx, group in enumerate(groups):
        ax = axes[idx]
        group_agents = [k for k, v in group_map.items() if v == group]

        for agent_name in group_agents:
            if agent_name not in agents:
                continue
            df = agents[agent_name]

            # Calculate error
            time_col = 'timestamp' if 'timestamp' in df.columns else 'time'
            error_col = 'error_magnitude' if 'error_magnitude' in df.columns else 'position_error'

            if error_col in df.columns:
                error = df[error_col].values
            elif 'current_x' in df.columns and 'target_x' in df.columns:
                # Calculate error from position data
                error = np.sqrt((df['current_x'].values - df['target_x'].values)**2 +
                               (df['current_y'].values - df['target_y'].values)**2)
            elif error_col:
                error = df[error_col].values
            else:
                continue

            if time_col in df.columns:
                time = df[time_col].values
            else:
                time = np.arange(len(error)) * 0.1  # Assume 10Hz

            ax.plot(time, error, color=colors[group], alpha=0.7, linewidth=1,
                   label=agent_name.split('_')[1])

        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Position Error (m)')
        ax.set_title(f'{group} Group - Position Error')
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8)
        ax.set_ylim(0, 8)

    plt.suptitle(title, fontsize=14)
    plt.tight_layout()
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"Saved: {output_file}")
    plt.close()

def plot_comparison_bar(phase1_dir, phase5_dir, output_file):
    """Plot comparison bar chart for Phase 1 and Phase 5"""
    import json

    data = {}

    for phase, pdir in [('Phase 1 (No Wind)', phase1_dir), ('Phase 5 (Wind)', phase5_dir)]:
        meta_file = Path(pdir) / 'phase_metadata.json'
        if not meta_file.exists():
            print(f"Warning: {meta_file} not found")
            continue

        with open(meta_file) as f:
            meta = json.load(f)

        groups = meta.get('group_summaries', {})
        for gname, gdata in groups.items():
            key = (phase, gname)
            data[key] = {
                'RMSE': gdata.get('mean_rmse', 0),
                'SS RMSE': gdata.get('mean_ss_rmse', 0),
                'MAE': gdata.get('mean_mae', 0),
                'Peak': gdata.get('mean_peak_error', 0),
            }

    if not data:
        print("No data to plot")
        return

    # Create grouped bar chart
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    axes = axes.flatten()

    metrics = ['RMSE', 'SS RMSE', 'MAE', 'Peak']
    phases = ['Phase 1 (No Wind)', 'Phase 5 (Wind)']
    controllers = ['PD', 'PID', 'IT2', 'GT2']
    colors = {'PD': 'blue', 'PID': 'green', 'IT2': 'orange', 'GT2': 'red'}

    for idx, metric in enumerate(metrics):
        ax = axes[idx]
        x = np.arange(len(controllers))
        width = 0.35

        for i, phase in enumerate(phases):
            values = []
            for ctrl in controllers:
                key = (phase, ctrl)
                if key in data:
                    values.append(data[key].get(metric, 0))
                else:
                    values.append(0)

            offset = (i - 0.5) * width
            bars = ax.bar(x + offset, values, width, label=phase, alpha=0.8)

        ax.set_xlabel('Controller')
        ax.set_ylabel(f'{metric} (m)')
        ax.set_title(f'{metric} Comparison')
        ax.set_xticks(x)
        ax.set_xticklabels(controllers)
        ax.legend()
        ax.grid(True, alpha=0.3, axis='y')

    plt.suptitle('Controller Performance Comparison: Phase 1 vs Phase 5', fontsize=14)
    plt.tight_layout()
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"Saved: {output_file}")
    plt.close()

def plot_group_average_trajectory(agents, title, output_file):
    """Plot average trajectory for each group"""
    group_map = get_group_mapping()
    colors = {'PD': 'blue', 'PID': 'green', 'IT2': 'orange', 'GT2': 'red'}

    fig, ax = plt.subplots(figsize=(12, 10))

    groups = ['PD', 'PID', 'IT2', 'GT2']

    for group in groups:
        group_agents = [k for k, v in group_map.items() if v == group]

        # Collect all positions
        all_x = []
        all_y = []
        all_tx = []
        all_ty = []

        for agent_name in group_agents:
            if agent_name not in agents:
                continue
            df = agents[agent_name]

            # Get position columns (handle different naming conventions)
            if 'current_x' in df.columns:
                x_col, y_col = 'current_x', 'current_y'
            elif 'pos_x' in df.columns:
                x_col, y_col = 'pos_x', 'pos_y'
            else:
                x_col, y_col = 'x', 'y'

            if x_col in df.columns:
                all_x.append(df[x_col].values)
                all_y.append(df[y_col].values)

                if 'target_x' in df.columns:
                    all_tx.append(df['target_x'].values)
                    all_ty.append(df['target_y'].values)

        if not all_x:
            continue

        # Find minimum length and truncate
        min_len = min(len(x) for x in all_x)
        all_x = [x[:min_len] for x in all_x]
        all_y = [y[:min_len] for y in all_y]

        # Calculate mean
        mean_x = np.mean(all_x, axis=0)
        mean_y = np.mean(all_y, axis=0)

        # Plot group average
        ax.plot(mean_x, mean_y, color=colors[group], linewidth=2,
               label=f'{group} (avg)', alpha=0.8)

        # Mark start and end
        ax.scatter(mean_x[0], mean_y[0], marker='o', s=100,
                  color=colors[group], zorder=5, edgecolors='black')
        ax.scatter(mean_x[-1], mean_y[-1], marker='s', s=100,
                  color=colors[group], zorder=5, edgecolors='black')

        # Target trajectory (if available)
        if all_tx:
            all_tx = [t[:min_len] for t in all_tx]
            all_ty = [t[:min_len] for t in all_ty]
            mean_tx = np.mean(all_tx, axis=0)
            mean_ty = np.mean(all_ty, axis=0)
            ax.plot(mean_tx, mean_ty, '--', color=colors[group],
                   linewidth=1, alpha=0.5, label=f'{group} target')

    ax.set_xlabel('X (m)', fontsize=12)
    ax.set_ylabel('Y (m)', fontsize=12)
    ax.set_title(title, fontsize=14)
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=10)
    ax.set_aspect('equal')

    plt.tight_layout()
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"Saved: {output_file}")
    plt.close()

def main():
    # Directories
    phase1_dir = PROJECT_ROOT / 'results/video_test/phase_1/run_1'
    phase5_dir = PROJECT_ROOT / 'results/phase5_test/phase_5/run_1'
    output_dir = PROJECT_ROOT / 'results/plots'
    output_dir.mkdir(parents=True, exist_ok=True)

    print("=" * 60)
    print("Phase Results Plotting")
    print("=" * 60)

    # Plot Phase 1
    if phase1_dir.exists():
        print(f"\nLoading Phase 1 data from {phase1_dir}")
        agents1 = load_agent_data(phase1_dir)
        print(f"Loaded {len(agents1)} agents")

        plot_trajectories(agents1, "Phase 1 (No Wind) - Individual Trajectories",
                         output_dir / "phase1_trajectories.png")
        plot_error_over_time(agents1, "Phase 1 (No Wind) - Position Error",
                            output_dir / "phase1_errors.png")
        plot_group_average_trajectory(agents1, "Phase 1 (No Wind) - Group Average Trajectories",
                                     output_dir / "phase1_group_avg.png")

    # Plot Phase 5
    if phase5_dir.exists():
        print(f"\nLoading Phase 5 data from {phase5_dir}")
        agents5 = load_agent_data(phase5_dir)
        print(f"Loaded {len(agents5)} agents")

        plot_trajectories(agents5, "Phase 5 (Stochastic Wind) - Individual Trajectories",
                         output_dir / "phase5_trajectories.png")
        plot_error_over_time(agents5, "Phase 5 (Stochastic Wind) - Position Error",
                            output_dir / "phase5_errors.png")
        plot_group_average_trajectory(agents5, "Phase 5 (Stochastic Wind) - Group Average Trajectories",
                                     output_dir / "phase5_group_avg.png")

    # Comparison
    if phase1_dir.exists() and phase5_dir.exists():
        print(f"\nCreating comparison plots")
        plot_comparison_bar(phase1_dir, phase5_dir,
                           output_dir / "phase_comparison.png")

    print("\n" + "=" * 60)
    print(f"All plots saved to: {output_dir}")
    print("=" * 60)

if __name__ == "__main__":
    main()
