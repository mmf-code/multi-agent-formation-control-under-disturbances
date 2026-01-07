#!/usr/bin/env python3
"""
Thesis-Quality Plot Generator for Scientific Validation
========================================================
Generates publication-ready plots matching thesis_lemniscate style:

1. Per-Phase Comprehensive (10 panels):
   - RMSE Evolution, XYZ Error Decomposition, SS Error Distribution
   - Min Inter-Agent Distance, Control Effort, Jerk Analysis
   - 2D Trajectory, Altitude Holding, Wind-Error Correlation
   - Wind Profile (full width)

2. Cross-Phase Comparison:
   - RMSE Heatmap (Phase × Controller)
   - Controller Win Count
   - SS-RMSE Across Phases
   - Control Effort Across Phases

3. Statistical Tables for Thesis
"""

import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.colors import LinearSegmentedColormap
from pathlib import Path
from scipy import stats
import warnings
warnings.filterwarnings('ignore')

# Use a clean style
plt.rcParams.update({
    'figure.facecolor': 'white',
    'axes.facecolor': 'white',
    'axes.grid': True,
    'grid.alpha': 0.3,
    'font.size': 9,
    'axes.titlesize': 11,
    'axes.labelsize': 10,
    'legend.fontsize': 8,
    'figure.dpi': 150,
    'savefig.dpi': 300,
    'savefig.bbox': 'tight',
    'savefig.facecolor': 'white'
})

# Professional color scheme
COLORS = {
    'PD': '#e74c3c',    # Red
    'PID': '#3498db',   # Blue
    'IT2': '#27ae60',   # Green
    'GT2': '#9b59b6'    # Purple
}

CONTROLLER_ORDER = ['PD', 'PID', 'IT2', 'GT2']

BASE_DIR = Path("/home/mmf/Documents/GitHub/multi-agent-formation-control-under-disturbances/results/scientific_validation")
OUTPUT_DIR = BASE_DIR / "plots" / "thesis_quality"

PHASES = {
    1: "BASELINE",
    2: "STEADY_WIND",
    3: "TURBULENCE",
    4: "GUST",
    5: "COMBINED",
    7: "VIDEO_DEMO"
}

PHASE_NAMES_SHORT = {
    1: "Baseline",
    2: "Steady Wind",
    3: "Turbulence",
    4: "Gust",
    5: "Combined",
    7: "Video Demo"
}


def load_all_data():
    """Load all phase data."""
    all_groups = []
    all_agents = {}
    all_wind = {}

    for phase_id in PHASES.keys():
        phase_dir = BASE_DIR / f"phase_{phase_id}"
        phase_agents = []
        phase_wind = []

        for run in [1, 2, 3]:
            run_dir = phase_dir / f"run_{run}"
            if not run_dir.exists():
                continue

            # Group summary
            group_file = run_dir / "group_summary.csv"
            if group_file.exists():
                df = pd.read_csv(group_file)
                df['phase'] = phase_id
                df['run'] = run
                all_groups.append(df)

            # Wind data (use run 1)
            if run == 1:
                wind_file = run_dir / "wind_data.csv"
                if wind_file.exists():
                    phase_wind = pd.read_csv(wind_file)

            # Agent data (use run 1)
            if run == 1:
                for agent_file in sorted(run_dir.glob("agent_*.csv")):
                    df = pd.read_csv(agent_file)
                    name = agent_file.stem
                    if '_pd' in name.lower():
                        df['controller'] = 'PD'
                    elif '_pid' in name.lower():
                        df['controller'] = 'PID'
                    elif '_it2' in name.lower():
                        df['controller'] = 'IT2'
                    elif '_gt2' in name.lower():
                        df['controller'] = 'GT2'
                    df['agent'] = name
                    phase_agents.append(df)

        all_agents[phase_id] = phase_agents
        all_wind[phase_id] = phase_wind

    groups = pd.concat(all_groups, ignore_index=True) if all_groups else pd.DataFrame()
    return groups, all_agents, all_wind


def plot_comprehensive_phase(phase_id, groups, agents, wind, output_dir):
    """Generate 10-panel comprehensive plot for a phase (thesis_lemniscate style)."""

    phase_name = PHASES[phase_id]
    phase_groups = groups[groups['phase'] == phase_id]

    # Create figure with custom layout
    fig = plt.figure(figsize=(16, 14))
    gs = gridspec.GridSpec(4, 3, figure=fig, hspace=0.35, wspace=0.3,
                          height_ratios=[1, 1, 1, 0.6])

    # Row 1: RMSE Evolution, XYZ Decomposition, SS Error Distribution
    # Panel 1: RMSE Evolution
    ax1 = fig.add_subplot(gs[0, 0])
    if agents:
        for ctrl in CONTROLLER_ORDER:
            ctrl_agents = [df for df in agents if df['controller'].iloc[0] == ctrl]
            if ctrl_agents:
                df = ctrl_agents[0]
                ax1.plot(df['timestamp'].values, df['error_magnitude'].values,
                        label=ctrl, color=COLORS[ctrl], linewidth=1.2, alpha=0.9)
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Position Error (m)')
    ax1.set_title(f'RMSE Evolution - Phase {phase_id} ({phase_name})')
    ax1.legend(loc='upper right', framealpha=0.9)
    ax1.set_xlim(left=0)

    # Panel 2: XYZ Error Decomposition
    ax2 = fig.add_subplot(gs[0, 1])
    if agents:
        x_pos = np.arange(len(CONTROLLER_ORDER))
        width = 0.25

        for i, axis in enumerate(['X-axis', 'Y-axis', 'Z-axis']):
            axis_col = f'error_{axis[0].lower()}'
            means = []
            for ctrl in CONTROLLER_ORDER:
                ctrl_agents = [df for df in agents if df['controller'].iloc[0] == ctrl]
                if ctrl_agents:
                    # Calculate mean absolute error for this axis
                    all_errors = np.concatenate([np.abs(df[axis_col].values) for df in ctrl_agents])
                    means.append(np.mean(all_errors))
                else:
                    means.append(0)

            colors = ['#3498db', '#e74c3c', '#27ae60'][i]
            ax2.bar(x_pos + (i-1)*width, means, width, label=axis, color=colors, alpha=0.8)

    ax2.set_xlabel('Controller')
    ax2.set_ylabel('Mean Absolute Error (m)')
    ax2.set_title(f'XYZ Error Decomposition - Phase {phase_id}')
    ax2.set_xticks(x_pos)
    ax2.set_xticklabels(CONTROLLER_ORDER)
    ax2.legend(loc='upper right')

    # Panel 3: Steady-State Error Distribution (Boxplot)
    ax3 = fig.add_subplot(gs[0, 2])
    if not phase_groups.empty:
        data = []
        for ctrl in CONTROLLER_ORDER:
            ctrl_data = phase_groups[phase_groups['controller_type'] == ctrl]['ss_rmse'].values
            data.append(ctrl_data if len(ctrl_data) > 0 else [0])

        bp = ax3.boxplot(data, labels=CONTROLLER_ORDER, patch_artist=True)
        for patch, ctrl in zip(bp['boxes'], CONTROLLER_ORDER):
            patch.set_facecolor(COLORS[ctrl])
            patch.set_alpha(0.7)

    ax3.set_xlabel('Controller')
    ax3.set_ylabel('Error Magnitude (m)')
    ax3.set_title(f'Steady-State Error Distribution - Phase {phase_id}')

    # Row 2: Min Distance, Control Effort, Jerk Analysis
    # Panel 4: Min Inter-Agent Distance
    ax4 = fig.add_subplot(gs[1, 0])
    if agents:
        for ctrl in CONTROLLER_ORDER:
            ctrl_agents = [df for df in agents if df['controller'].iloc[0] == ctrl]
            if len(ctrl_agents) >= 2:
                # Calculate pairwise distances
                df1, df2 = ctrl_agents[0], ctrl_agents[1]
                min_len = min(len(df1), len(df2))
                dist = np.sqrt(
                    (df1['current_x'].values[:min_len] - df2['current_x'].values[:min_len])**2 +
                    (df1['current_y'].values[:min_len] - df2['current_y'].values[:min_len])**2
                )
                samples = np.arange(len(dist))
                ax4.plot(samples, dist, label=ctrl, color=COLORS[ctrl], linewidth=1, alpha=0.8)

        ax4.axhline(y=0.4, color='orange', linestyle='--', linewidth=1.5, label='Safety Threshold (0.4m)')
        ax4.axhline(y=0.5, color='red', linestyle='--', linewidth=1.5, label='Collision Threshold (0.5m)')

    ax4.set_xlabel('Sample Index')
    ax4.set_ylabel('Min Distance (m)')
    ax4.set_title(f'Min Inter-Agent Distance - Phase {phase_id}')
    ax4.legend(loc='lower right', fontsize=7)

    # Panel 5: Control Effort (IAE)
    ax5 = fig.add_subplot(gs[1, 1])
    if not phase_groups.empty:
        means = []
        for ctrl in CONTROLLER_ORDER:
            ctrl_data = phase_groups[phase_groups['controller_type'] == ctrl]['control_effort_iae']
            means.append(ctrl_data.mean() if len(ctrl_data) > 0 else 0)

        bars = ax5.bar(CONTROLLER_ORDER, means, color=[COLORS[c] for c in CONTROLLER_ORDER], alpha=0.85)

        # Add value labels
        for bar, val in zip(bars, means):
            ax5.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 5,
                    f'{val:.0f}', ha='center', va='bottom', fontsize=8)

    ax5.set_xlabel('Controller')
    ax5.set_ylabel('Control Effort (IAE)')
    ax5.set_title(f'Control Effort (IAE) - Phase {phase_id}')

    # Panel 6: Jerk Analysis (Smoothness)
    ax6 = fig.add_subplot(gs[1, 2])
    if agents:
        jerk_means = []
        for ctrl in CONTROLLER_ORDER:
            ctrl_agents = [df for df in agents if df['controller'].iloc[0] == ctrl]
            if ctrl_agents:
                jerks = []
                for df in ctrl_agents:
                    if 'velocity_x' in df.columns:
                        # Jerk = derivative of acceleration ≈ second derivative of velocity
                        vx = df['velocity_x'].values
                        dt = np.diff(df['timestamp'].values)
                        dt[dt == 0] = 0.01
                        ax_arr = np.diff(vx) / dt
                        jerk = np.abs(np.diff(ax_arr) / dt[:-1])
                        jerks.extend(jerk[np.isfinite(jerk)])
                jerk_means.append(np.mean(jerks) if jerks else 0)
            else:
                jerk_means.append(0)

        bars = ax6.bar(CONTROLLER_ORDER, jerk_means, color=[COLORS[c] for c in CONTROLLER_ORDER], alpha=0.85)

        for bar, val in zip(bars, jerk_means):
            ax6.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.5,
                    f'{val:.2f}', ha='center', va='bottom', fontsize=8)

    ax6.set_xlabel('Controller')
    ax6.set_ylabel('Mean Jerk (m/s³)')
    ax6.set_title(f'Jerk Analysis (Smoothness) - Phase {phase_id}')

    # Row 3: 2D Trajectory, Altitude Holding, Wind-Error Correlation
    # Panel 7: 2D Trajectory (Top View)
    ax7 = fig.add_subplot(gs[2, 0])
    if agents:
        for ctrl in CONTROLLER_ORDER:
            ctrl_agents = [df for df in agents if df['controller'].iloc[0] == ctrl]
            for i, df in enumerate(ctrl_agents[:2]):
                ax7.plot(df['current_x'].values, df['current_y'].values,
                        color=COLORS[ctrl], linewidth=0.8, alpha=0.7,
                        label=ctrl if i == 0 else None)

        # Target trajectory (lemniscate)
        if agents:
            df = agents[0]
            ax7.plot(df['target_x'].values, df['target_y'].values,
                    'k--', linewidth=1, alpha=0.5, label='Target')

    ax7.set_xlabel('X (m)')
    ax7.set_ylabel('Y (m)')
    ax7.set_title(f'2D Trajectory (Top View) - Phase {phase_id}')
    ax7.legend(loc='upper right', fontsize=7)
    ax7.set_aspect('equal', adjustable='box')

    # Panel 8: Altitude Holding (Z vs Time)
    ax8 = fig.add_subplot(gs[2, 1])
    if agents:
        for ctrl in CONTROLLER_ORDER:
            ctrl_agents = [df for df in agents if df['controller'].iloc[0] == ctrl]
            if ctrl_agents:
                df = ctrl_agents[0]
                ax8.plot(df['timestamp'].values, df['current_z'].values,
                        label=ctrl, color=COLORS[ctrl], linewidth=1, alpha=0.8)

        # Target Z
        if agents:
            df = agents[0]
            ax8.plot(df['timestamp'].values, df['target_z'].values,
                    'k--', linewidth=1, alpha=0.5, label='Target')

    ax8.set_xlabel('Time (s)')
    ax8.set_ylabel('Altitude Z (m)')
    ax8.set_title(f'Altitude Holding (Z vs Time) - Phase {phase_id}')
    ax8.legend(loc='upper right', fontsize=7)

    # Panel 9: Wind vs Error Correlation
    ax9 = fig.add_subplot(gs[2, 2])
    if agents and isinstance(wind, pd.DataFrame) and not wind.empty:
        for ctrl in CONTROLLER_ORDER:
            ctrl_agents = [df for df in agents if df['controller'].iloc[0] == ctrl]
            if ctrl_agents:
                df = ctrl_agents[0]
                # Resample to match wind data
                n_samples = min(len(df), len(wind))
                error_resampled = df['error_magnitude'].values[:n_samples]
                wind_resampled = wind['wind_magnitude'].values[:n_samples]

                ax9.scatter(wind_resampled[::10], error_resampled[::10],
                           c=COLORS[ctrl], alpha=0.5, s=10, label=ctrl)

    ax9.set_xlabel('Wind Magnitude (m/s)')
    ax9.set_ylabel('Error Magnitude (m)')
    ax9.set_title(f'Wind vs Error Correlation - Phase {phase_id}')
    ax9.legend(loc='upper left', fontsize=7)

    # Row 4: Wind Profile (full width)
    ax10 = fig.add_subplot(gs[3, :])
    if isinstance(wind, pd.DataFrame) and not wind.empty:
        t = np.linspace(0, 90, len(wind))

        ax10.fill_between(t, 0, wind['wind_magnitude'].values, alpha=0.3, color='#3498db', label='Magnitude')
        ax10.plot(t, wind['wind_x'].values, color='#e74c3c', linewidth=0.8, alpha=0.8, label='Wind X')
        ax10.plot(t, wind['wind_y'].values, color='#27ae60', linewidth=0.8, alpha=0.8, label='Wind Y')
        ax10.plot(t, wind['wind_magnitude'].values, color='#2c3e50', linewidth=1, label='Magnitude')

        # Add statistics
        wind_mean = wind['wind_magnitude'].mean()
        wind_max = wind['wind_magnitude'].max()
        ax10.text(0.98, 0.95, f'Mean: {wind_mean:.2f} m/s\nMax: {wind_max:.2f} m/s',
                 transform=ax10.transAxes, ha='right', va='top',
                 bbox=dict(boxstyle='round', facecolor='white', alpha=0.8), fontsize=8)

    ax10.set_xlabel('Time (s)')
    ax10.set_ylabel('Wind Velocity (m/s)')
    ax10.set_title(f'Wind Disturbance Profile - Phase {phase_id} ({phase_name})')
    ax10.legend(loc='upper left', fontsize=8)
    ax10.set_xlim(0, 90)

    plt.suptitle(f'Phase {phase_id}: {phase_name} - Comprehensive Analysis',
                fontsize=14, fontweight='bold', y=0.98)

    plt.savefig(output_dir / f"phase_{phase_id}_comprehensive.png", bbox_inches='tight')
    plt.savefig(output_dir / f"phase_{phase_id}_comprehensive.pdf", bbox_inches='tight')
    plt.close()
    print(f"  [+] Phase {phase_id} comprehensive plot saved")


def plot_cross_phase_comparison(groups, output_dir):
    """Generate cross-phase comparison plot (4 panels)."""

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    # Panel 1: RMSE Heatmap (Phase × Controller)
    ax1 = axes[0, 0]

    phase_ids = list(PHASES.keys())
    phase_names = [PHASES[p] for p in phase_ids]

    heatmap_data = np.zeros((len(phase_ids), len(CONTROLLER_ORDER)))

    for i, phase_id in enumerate(phase_ids):
        phase_data = groups[groups['phase'] == phase_id]
        for j, ctrl in enumerate(CONTROLLER_ORDER):
            ctrl_data = phase_data[phase_data['controller_type'] == ctrl]['mean_rmse']
            heatmap_data[i, j] = ctrl_data.mean() if len(ctrl_data) > 0 else np.nan

    # Custom colormap (green = good, red = bad)
    cmap = LinearSegmentedColormap.from_list('custom', ['#27ae60', '#f1c40f', '#e74c3c'])

    im = ax1.imshow(heatmap_data, cmap=cmap, aspect='auto')

    # Add text annotations
    for i in range(len(phase_ids)):
        for j in range(len(CONTROLLER_ORDER)):
            val = heatmap_data[i, j]
            color = 'white' if val > 1.5 else 'black'
            ax1.text(j, i, f'{val:.2f}', ha='center', va='center', color=color, fontsize=9)

    ax1.set_xticks(range(len(CONTROLLER_ORDER)))
    ax1.set_xticklabels(CONTROLLER_ORDER)
    ax1.set_yticks(range(len(phase_ids)))
    ax1.set_yticklabels(phase_names)
    ax1.set_title('RMSE Heatmap (Phase × Controller)', fontweight='bold')

    cbar = plt.colorbar(im, ax=ax1, shrink=0.8)
    cbar.set_label('RMSE (m)')

    # Panel 2: Controller Win Count
    ax2 = axes[0, 1]

    win_counts = {ctrl: 0 for ctrl in CONTROLLER_ORDER}
    for phase_id in phase_ids:
        phase_data = groups[groups['phase'] == phase_id]
        best_rmse = float('inf')
        winner = None
        for ctrl in CONTROLLER_ORDER:
            ctrl_rmse = phase_data[phase_data['controller_type'] == ctrl]['mean_rmse'].mean()
            if ctrl_rmse < best_rmse:
                best_rmse = ctrl_rmse
                winner = ctrl
        if winner:
            win_counts[winner] += 1

    bars = ax2.bar(CONTROLLER_ORDER, [win_counts[c] for c in CONTROLLER_ORDER],
                   color=[COLORS[c] for c in CONTROLLER_ORDER], alpha=0.85)

    for bar, val in zip(bars, [win_counts[c] for c in CONTROLLER_ORDER]):
        if val > 0:
            ax2.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.1,
                    str(val), ha='center', va='bottom', fontsize=12, fontweight='bold')

    ax2.set_xlabel('Controller')
    ax2.set_ylabel('Win Count')
    ax2.set_title('Controller Win Count (Best RMSE)', fontweight='bold')
    ax2.set_ylim(0, len(phase_ids) + 0.5)

    # Panel 3: SS-RMSE Across Phases
    ax3 = axes[1, 0]

    for ctrl in CONTROLLER_ORDER:
        ss_rmse_values = []
        for phase_id in phase_ids:
            phase_data = groups[groups['phase'] == phase_id]
            ctrl_data = phase_data[phase_data['controller_type'] == ctrl]['ss_rmse']
            ss_rmse_values.append(ctrl_data.mean() if len(ctrl_data) > 0 else np.nan)

        ax3.plot(range(len(phase_ids)), ss_rmse_values,
                marker='o', markersize=8, linewidth=2,
                color=COLORS[ctrl], label=ctrl)

    ax3.set_xlabel('Phase')
    ax3.set_ylabel('SS-RMSE (m)')
    ax3.set_title('Steady-State RMSE Across Phases', fontweight='bold')
    ax3.set_xticks(range(len(phase_ids)))
    ax3.set_xticklabels([f'P{p}' for p in phase_ids])
    ax3.legend(loc='upper left')

    # Panel 4: Control Effort Across Phases
    ax4 = axes[1, 1]

    for ctrl in CONTROLLER_ORDER:
        effort_values = []
        for phase_id in phase_ids:
            phase_data = groups[groups['phase'] == phase_id]
            ctrl_data = phase_data[phase_data['controller_type'] == ctrl]['control_effort_iae']
            effort_values.append(ctrl_data.mean() if len(ctrl_data) > 0 else np.nan)

        ax4.plot(range(len(phase_ids)), effort_values,
                marker='s', markersize=8, linewidth=2,
                color=COLORS[ctrl], label=ctrl)

    ax4.set_xlabel('Phase')
    ax4.set_ylabel('Control Effort (IAE)')
    ax4.set_title('Control Effort Across Phases', fontweight='bold')
    ax4.set_xticks(range(len(phase_ids)))
    ax4.set_xticklabels([f'P{p}' for p in phase_ids])
    ax4.legend(loc='upper left')

    plt.suptitle('Cross-Phase Controller Comparison', fontsize=16, fontweight='bold', y=0.98)
    plt.tight_layout()

    plt.savefig(output_dir / "all_phases_comparison.png", bbox_inches='tight')
    plt.savefig(output_dir / "all_phases_comparison.pdf", bbox_inches='tight')
    plt.close()
    print("  [+] Cross-phase comparison saved")


def plot_statistical_summary(groups, output_dir):
    """Generate statistical summary table as image."""

    fig, ax = plt.subplots(figsize=(14, 8))
    ax.axis('off')

    # Prepare data
    phase_ids = list(PHASES.keys())

    # Create table data
    headers = ['Phase', 'Metric', 'PD', 'PID', 'IT2', 'GT2', 'Best', 'Improvement']
    rows = []

    for phase_id in phase_ids:
        phase_data = groups[groups['phase'] == phase_id]
        phase_name = PHASE_NAMES_SHORT[phase_id]

        # RMSE row
        rmse_vals = {}
        for ctrl in CONTROLLER_ORDER:
            ctrl_data = phase_data[phase_data['controller_type'] == ctrl]['mean_rmse']
            rmse_vals[ctrl] = ctrl_data.mean() if len(ctrl_data) > 0 else np.nan

        best_ctrl = min(rmse_vals, key=rmse_vals.get)
        pd_val = rmse_vals['PD']
        best_val = rmse_vals[best_ctrl]
        improvement = ((pd_val - best_val) / pd_val * 100) if pd_val > 0 else 0

        rows.append([
            phase_name, 'RMSE (m)',
            f"{rmse_vals['PD']:.3f}",
            f"{rmse_vals['PID']:.3f}",
            f"{rmse_vals['IT2']:.3f}",
            f"{rmse_vals['GT2']:.3f}",
            best_ctrl,
            f"{improvement:.1f}%"
        ])

    # Create table
    table = ax.table(cellText=rows, colLabels=headers, loc='center', cellLoc='center')
    table.auto_set_font_size(False)
    table.set_fontsize(10)
    table.scale(1.2, 1.8)

    # Style header
    for i, key in enumerate(headers):
        cell = table[(0, i)]
        cell.set_facecolor('#3498db')
        cell.set_text_props(color='white', fontweight='bold')

    # Highlight best values
    for row_idx, row in enumerate(rows):
        best = row[6]  # Best controller column
        for col_idx, ctrl in enumerate(['PD', 'PID', 'IT2', 'GT2']):
            if ctrl == best:
                table[(row_idx + 1, col_idx + 2)].set_facecolor('#d5f4e6')

    plt.title('Statistical Summary - RMSE by Phase and Controller', fontsize=14, fontweight='bold', pad=20)

    plt.savefig(output_dir / "statistical_summary_table.png", bbox_inches='tight')
    plt.savefig(output_dir / "statistical_summary_table.pdf", bbox_inches='tight')
    plt.close()
    print("  [+] Statistical summary table saved")


def plot_improvement_analysis(groups, output_dir):
    """Generate improvement analysis plots."""

    fig, axes = plt.subplots(1, 3, figsize=(15, 5))

    phase_ids = list(PHASES.keys())

    # Panel 1: Fuzzy vs PID Improvement
    ax1 = axes[0]

    it2_improvement = []
    gt2_improvement = []

    for phase_id in phase_ids:
        phase_data = groups[groups['phase'] == phase_id]

        pid_rmse = phase_data[phase_data['controller_type'] == 'PID']['mean_rmse'].mean()
        it2_rmse = phase_data[phase_data['controller_type'] == 'IT2']['mean_rmse'].mean()
        gt2_rmse = phase_data[phase_data['controller_type'] == 'GT2']['mean_rmse'].mean()

        it2_imp = (pid_rmse - it2_rmse) / pid_rmse * 100 if pid_rmse > 0 else 0
        gt2_imp = (pid_rmse - gt2_rmse) / pid_rmse * 100 if pid_rmse > 0 else 0

        it2_improvement.append(it2_imp)
        gt2_improvement.append(gt2_imp)

    x = np.arange(len(phase_ids))
    width = 0.35

    bars1 = ax1.bar(x - width/2, it2_improvement, width, label='IT2 vs PID', color=COLORS['IT2'], alpha=0.85)
    bars2 = ax1.bar(x + width/2, gt2_improvement, width, label='GT2 vs PID', color=COLORS['GT2'], alpha=0.85)

    ax1.axhline(y=0, color='black', linestyle='-', linewidth=0.5)
    ax1.set_xlabel('Phase')
    ax1.set_ylabel('Improvement (%)')
    ax1.set_title('Fuzzy Controllers Improvement over PID', fontweight='bold')
    ax1.set_xticks(x)
    ax1.set_xticklabels([f'P{p}' for p in phase_ids])
    ax1.legend()

    # Panel 2: All Controllers vs PD Improvement
    ax2 = axes[1]

    improvements = {ctrl: [] for ctrl in ['PID', 'IT2', 'GT2']}

    for phase_id in phase_ids:
        phase_data = groups[groups['phase'] == phase_id]
        pd_rmse = phase_data[phase_data['controller_type'] == 'PD']['mean_rmse'].mean()

        for ctrl in ['PID', 'IT2', 'GT2']:
            ctrl_rmse = phase_data[phase_data['controller_type'] == ctrl]['mean_rmse'].mean()
            imp = (pd_rmse - ctrl_rmse) / pd_rmse * 100 if pd_rmse > 0 else 0
            improvements[ctrl].append(imp)

    for ctrl in ['PID', 'IT2', 'GT2']:
        ax2.plot(range(len(phase_ids)), improvements[ctrl],
                marker='o', markersize=8, linewidth=2,
                color=COLORS[ctrl], label=f'{ctrl} vs PD')

    ax2.axhline(y=0, color='black', linestyle='-', linewidth=0.5)
    ax2.set_xlabel('Phase')
    ax2.set_ylabel('Improvement over PD (%)')
    ax2.set_title('Controller Improvement over PD Baseline', fontweight='bold')
    ax2.set_xticks(range(len(phase_ids)))
    ax2.set_xticklabels([f'P{p}' for p in phase_ids])
    ax2.legend()

    # Panel 3: Average Improvement Summary
    ax3 = axes[2]

    avg_improvements = []
    for ctrl in CONTROLLER_ORDER:
        phase_improvements = []
        for phase_id in phase_ids:
            phase_data = groups[groups['phase'] == phase_id]
            pd_rmse = phase_data[phase_data['controller_type'] == 'PD']['mean_rmse'].mean()
            ctrl_rmse = phase_data[phase_data['controller_type'] == ctrl]['mean_rmse'].mean()
            imp = (pd_rmse - ctrl_rmse) / pd_rmse * 100 if pd_rmse > 0 else 0
            phase_improvements.append(imp)
        avg_improvements.append(np.mean(phase_improvements))

    bars = ax3.bar(CONTROLLER_ORDER, avg_improvements,
                   color=[COLORS[c] for c in CONTROLLER_ORDER], alpha=0.85)

    for bar, val in zip(bars, avg_improvements):
        ypos = bar.get_height() + 1 if bar.get_height() >= 0 else bar.get_height() - 3
        ax3.text(bar.get_x() + bar.get_width()/2, ypos,
                f'{val:.1f}%', ha='center', va='bottom', fontsize=11, fontweight='bold')

    ax3.axhline(y=0, color='black', linestyle='-', linewidth=0.5)
    ax3.set_xlabel('Controller')
    ax3.set_ylabel('Average Improvement over PD (%)')
    ax3.set_title('Average Improvement Across All Phases', fontweight='bold')

    plt.suptitle('Controller Performance Improvement Analysis', fontsize=14, fontweight='bold', y=1.02)
    plt.tight_layout()

    plt.savefig(output_dir / "improvement_analysis.png", bbox_inches='tight')
    plt.savefig(output_dir / "improvement_analysis.pdf", bbox_inches='tight')
    plt.close()
    print("  [+] Improvement analysis saved")


def main():
    print("=" * 70)
    print("THESIS-QUALITY PLOT GENERATOR")
    print("=" * 70)

    # Load all data
    print("\n[1/5] Loading data...")
    groups, all_agents, all_wind = load_all_data()
    print(f"  Loaded {len(groups)} group summaries")

    # Generate per-phase comprehensive plots
    print("\n[2/5] Generating comprehensive phase plots...")
    for phase_id in PHASES.keys():
        agents = all_agents.get(phase_id, [])
        wind = all_wind.get(phase_id, pd.DataFrame())
        plot_comprehensive_phase(phase_id, groups, agents, wind, OUTPUT_DIR)

    # Generate cross-phase comparison
    print("\n[3/5] Generating cross-phase comparison...")
    plot_cross_phase_comparison(groups, OUTPUT_DIR)

    # Generate statistical summary
    print("\n[4/5] Generating statistical summary...")
    plot_statistical_summary(groups, OUTPUT_DIR)

    # Generate improvement analysis
    print("\n[5/5] Generating improvement analysis...")
    plot_improvement_analysis(groups, OUTPUT_DIR)

    # Summary
    total_files = len(list(OUTPUT_DIR.glob("*.png")))
    print("\n" + "=" * 70)
    print("COMPLETE!")
    print("=" * 70)
    print(f"Output directory: {OUTPUT_DIR}")
    print(f"Total plots generated: {total_files} PNG files (+ PDF versions)")
    print("\nFiles:")
    for f in sorted(OUTPUT_DIR.glob("*.png")):
        print(f"  - {f.name}")


if __name__ == "__main__":
    main()
