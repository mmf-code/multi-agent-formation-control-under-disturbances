#!/usr/bin/env python3
"""
Trajectory Mapping Visualization for Lemniscate Formation Control

Plots actual vs target trajectories for all controller groups.
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path
import sys

# Controller groups configuration
GROUPS = {
    'PD': {'agents': [0, 1, 2], 'color': '#e74c3c', 'marker': 'o'},      # Red
    'PID': {'agents': [3, 4, 5], 'color': '#3498db', 'marker': 's'},     # Blue
    'IT2': {'agents': [6, 7, 8], 'color': '#2ecc71', 'marker': '^'},     # Green
    'GT2': {'agents': [9, 10, 11], 'color': '#9b59b6', 'marker': 'd'},   # Purple
}

def load_agent_data(results_dir, agent_id, controller_type):
    """Load trajectory data for a single agent."""
    filename = f"agent_{agent_id}_{controller_type.lower()}.csv"
    filepath = Path(results_dir) / filename
    if filepath.exists():
        return pd.read_csv(filepath)
    return None

def plot_trajectories(results_dir, output_path=None, title_suffix=""):
    """Create trajectory mapping visualization."""
    fig, axes = plt.subplots(2, 2, figsize=(14, 12))
    fig.suptitle(f'Lemniscate Trajectory Mapping - Controller Comparison{title_suffix}',
                 fontsize=14, fontweight='bold')

    # Plot each group in a subplot
    for idx, (group_name, group_info) in enumerate(GROUPS.items()):
        ax = axes[idx // 2, idx % 2]

        # Plot target trajectory (from first agent, they're all the same)
        first_agent = group_info['agents'][0]
        data = load_agent_data(results_dir, first_agent, group_name)

        if data is not None:
            # Plot target (reference) trajectory - dashed line
            ax.plot(data['target_x'].values, data['target_y'].values,
                   '--', color='gray', alpha=0.7, linewidth=2,
                   label='Target (Lemniscate)', zorder=1)

            # Plot actual trajectories for each agent
            for agent_id in group_info['agents']:
                agent_data = load_agent_data(results_dir, agent_id, group_name)
                if agent_data is not None:
                    ax.plot(agent_data['current_x'].values, agent_data['current_y'].values,
                           color=group_info['color'], alpha=0.7, linewidth=1.5,
                           label=f'Agent {agent_id}' if agent_id == first_agent else None,
                           zorder=2)

                    # Mark start and end points
                    ax.scatter(agent_data['current_x'].iloc[0], agent_data['current_y'].iloc[0],
                              color=group_info['color'], marker='o', s=100,
                              edgecolors='white', linewidths=2, zorder=3)
                    ax.scatter(agent_data['current_x'].iloc[-1], agent_data['current_y'].iloc[-1],
                              color=group_info['color'], marker='X', s=100,
                              edgecolors='white', linewidths=2, zorder=3)

        ax.set_xlabel('X Position (m)', fontsize=10)
        ax.set_ylabel('Y Position (m)', fontsize=10)
        ax.set_title(f'{group_name} Controller', fontsize=12, fontweight='bold',
                    color=group_info['color'])
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')
        ax.legend(loc='upper right', fontsize=8)

        # Add error statistics
        if data is not None:
            errors = np.sqrt((data['current_x'].values - data['target_x'].values)**2 +
                           (data['current_y'].values - data['target_y'].values)**2)
            rmse = np.sqrt(np.mean(errors**2))
            ax.text(0.02, 0.98, f'RMSE: {rmse:.3f}m', transform=ax.transAxes,
                   fontsize=9, verticalalignment='top',
                   bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

    plt.tight_layout()

    if output_path:
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f"Saved: {output_path}")

    return fig

def plot_combined_trajectory(results_dir, output_path=None, title_suffix=""):
    """Create combined trajectory plot showing all groups."""
    fig, ax = plt.subplots(figsize=(12, 10))

    # Plot target trajectory once
    first_data = load_agent_data(results_dir, 0, 'pd')
    if first_data is not None:
        # Theoretical lemniscate
        t = np.linspace(0, 2*np.pi, 500)
        a, b = 5.0, 2.0  # Lemniscate parameters
        x_ref = a * np.sin(t)
        y_ref = b * np.sin(2*t)
        ax.plot(x_ref, y_ref - 12, '--', color='gray', alpha=0.5, linewidth=2,
               label='Reference Lemniscate')

    # Plot each group with offset
    y_offsets = {'PD': -12, 'PID': -4, 'IT2': 4, 'GT2': 12}

    for group_name, group_info in GROUPS.items():
        # Use first agent as representative
        first_agent = group_info['agents'][0]
        data = load_agent_data(results_dir, first_agent, group_name)

        if data is not None:
            # Plot actual trajectory
            ax.plot(data['current_x'].values, data['current_y'].values,
                   color=group_info['color'], alpha=0.8, linewidth=2,
                   label=f'{group_name}', zorder=2)

            # Plot target for this group
            ax.plot(data['target_x'].values, data['target_y'].values,
                   '--', color=group_info['color'], alpha=0.3, linewidth=1,
                   zorder=1)

    ax.set_xlabel('X Position (m)', fontsize=12)
    ax.set_ylabel('Y Position (m)', fontsize=12)
    ax.set_title(f'Combined Lemniscate Trajectories - All Controllers{title_suffix}',
                fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper right', fontsize=10)
    ax.set_aspect('equal')

    # Add lane labels
    for group_name, y_offset in y_offsets.items():
        ax.text(-8, y_offset, group_name, fontsize=10, fontweight='bold',
               color=GROUPS[group_name]['color'], verticalalignment='center')

    plt.tight_layout()

    if output_path:
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f"Saved: {output_path}")

    return fig

def plot_error_over_time(results_dir, output_path=None, title_suffix=""):
    """Plot tracking error over time for all controllers."""
    fig, ax = plt.subplots(figsize=(12, 6))

    for group_name, group_info in GROUPS.items():
        first_agent = group_info['agents'][0]
        data = load_agent_data(results_dir, first_agent, group_name)

        if data is not None:
            errors = np.sqrt((data['current_x'].values - data['target_x'].values)**2 +
                           (data['current_y'].values - data['target_y'].values)**2)
            ax.plot(data['timestamp'].values, errors,
                   color=group_info['color'], alpha=0.8, linewidth=1.5,
                   label=f'{group_name} (RMSE: {np.sqrt(np.mean(errors**2)):.3f}m)')

    ax.set_xlabel('Time (s)', fontsize=12)
    ax.set_ylabel('Tracking Error (m)', fontsize=12)
    ax.set_title(f'Tracking Error Over Time{title_suffix}', fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper right', fontsize=10)
    ax.set_xlim(0, None)
    ax.set_ylim(0, None)

    plt.tight_layout()

    if output_path:
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f"Saved: {output_path}")

    return fig

if __name__ == "__main__":
    # Default to phase_1 results
    results_dir = Path("results/phase_1/run_1")
    if len(sys.argv) > 1:
        results_dir = Path(sys.argv[1])

    if not results_dir.exists():
        print(f"Error: Results directory not found: {results_dir}")
        sys.exit(1)

    output_dir = Path("results/plots")
    output_dir.mkdir(parents=True, exist_ok=True)

    # Generate plots
    print("Generating trajectory plots...")

    plot_trajectories(results_dir, output_dir / "trajectory_per_controller.png")
    plot_combined_trajectory(results_dir, output_dir / "trajectory_combined.png")
    plot_error_over_time(results_dir, output_dir / "tracking_error.png")

    print("\nAll plots saved to results/plots/")
    plt.show()
