#!/usr/bin/env python3
"""
Quick Trajectory Plot Generator for Thesis
"""

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from pathlib import Path

COLORS = {'pd': '#E74C3C', 'pid': '#3498DB', 'it2': '#27AE60', 'gt2': '#9B59B6'}
PHASE_NAMES = {1: 'BASELINE', 2: 'STEADY_WIND', 3: 'TURBULENCE', 4: 'GUST', 5: 'COMBINED'}

def generate_trajectory_plot(phase_dir, output_dir, phase_id):
    """Generate trajectory plot for one phase."""
    fig, axes = plt.subplots(2, 2, figsize=(14, 12))

    # Group files by controller
    ctrl_files = {'pd': [], 'pid': [], 'it2': [], 'gt2': []}
    for f in Path(phase_dir).glob('agent_*.csv'):
        parts = f.stem.split('_')
        if len(parts) >= 3:
            ctrl = parts[2].lower()
            if ctrl in ctrl_files:
                ctrl_files[ctrl].append(f)

    # Plot each controller
    for idx, (ctrl, files) in enumerate(ctrl_files.items()):
        ax = axes[idx // 2, idx % 2]

        for f in files:
            df = pd.read_csv(f)
            if 'current_x' in df.columns and 'target_x' in df.columns:
                # Actual trajectory
                ax.plot(df['current_x'].values, df['current_y'].values,
                       color=COLORS[ctrl], alpha=0.7, linewidth=1.5)
                # Target trajectory (dashed)
                ax.plot(df['target_x'].values, df['target_y'].values,
                       '--', color='gray', alpha=0.4, linewidth=1)

        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title(f'{ctrl.upper()} Controller', fontweight='bold', color=COLORS[ctrl])
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')

    phase_name = PHASE_NAMES.get(phase_id, '')
    fig.suptitle(f'Phase {phase_id}: {phase_name} - Lemniscate Trajectory Tracking',
                fontsize=14, fontweight='bold')
    plt.tight_layout()

    output_path = Path(output_dir) / f'phase_{phase_id}_trajectory.png'
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'  Saved: {output_path}')

def generate_error_plot(phase_dir, output_dir, phase_id):
    """Generate tracking error plot."""
    fig, ax = plt.subplots(figsize=(12, 6))

    for ctrl in ['pd', 'pid', 'it2', 'gt2']:
        for f in Path(phase_dir).glob(f'agent_*_{ctrl}.csv'):
            df = pd.read_csv(f)
            if 'timestamp' in df.columns and 'error_magnitude' in df.columns:
                ax.plot(df['timestamp'].values, df['error_magnitude'].values,
                       color=COLORS[ctrl], alpha=0.5, linewidth=1)
            break  # One agent per controller

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Tracking Error (m)')
    ax.set_title(f'Phase {phase_id}: Tracking Error Over Time')
    ax.grid(True, alpha=0.3)
    ax.set_xlim(0, None)
    ax.set_ylim(0, None)
    ax.legend(['PD', 'PID', 'IT2', 'GT2'], loc='upper right')
    plt.tight_layout()

    output_path = Path(output_dir) / f'phase_{phase_id}_tracking_error.png'
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'  Saved: {output_path}')

def main():
    base_dir = Path('results/thesis_lemniscate')
    output_dir = base_dir / 'plots' / 'trajectories'
    output_dir.mkdir(parents=True, exist_ok=True)

    print('Generating trajectory plots...')

    for phase_id in range(1, 6):
        phase_dir = base_dir / f'phase_{phase_id}' / 'run_1'
        if phase_dir.exists():
            print(f'\nPhase {phase_id}:')
            generate_trajectory_plot(phase_dir, output_dir, phase_id)
            generate_error_plot(phase_dir, output_dir, phase_id)

    print('\nDone!')

if __name__ == '__main__':
    main()
