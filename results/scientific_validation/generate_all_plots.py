#!/usr/bin/env python3
"""
Scientific Validation - Comprehensive Plot Generator
=====================================================
Generates all plots for thesis:
1. Individual metric plots (RMSE, MAE, Peak Error, etc.)
2. Comprehensive multi-panel plots per phase
3. Trajectory plots (2D bird's eye view)
4. Wind profile plots
5. Data quality report
"""

import os
import sys
import json
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from pathlib import Path
from datetime import datetime

# Style settings - use compatible style
try:
    plt.style.use('seaborn-whitegrid')
except:
    plt.style.use('ggplot')
plt.rcParams['figure.dpi'] = 150
plt.rcParams['savefig.dpi'] = 300
plt.rcParams['font.size'] = 10
plt.rcParams['axes.titlesize'] = 12
plt.rcParams['axes.labelsize'] = 10

# Color scheme for controllers
COLORS = {
    'PD': '#e74c3c',    # Red
    'PID': '#f39c12',   # Orange
    'IT2': '#27ae60',   # Green
    'GT2': '#3498db'    # Blue
}

MARKERS = {'PD': 's', 'PID': '^', 'IT2': 'o', 'GT2': 'D'}

BASE_DIR = Path("/home/mmf/Documents/GitHub/multi-agent-formation-control-under-disturbances/results/scientific_validation")
PLOT_DIR = BASE_DIR / "plots"

PHASES = {
    1: "Baseline (No Wind)",
    2: "Steady Wind (3 m/s)",
    3: "Turbulence (Von Karman)",
    4: "Gust (Periodic)",
    5: "Combined (Stochastic)",
    7: "Video Demo (Calibrated)"
}

def load_phase_data(phase_id):
    """Load all runs for a phase."""
    phase_dir = BASE_DIR / f"phase_{phase_id}"
    all_agents = []
    all_groups = []
    all_wind = []

    for run in [1, 2, 3]:
        run_dir = phase_dir / f"run_{run}"
        if not run_dir.exists():
            continue

        # Load group summary
        group_file = run_dir / "group_summary.csv"
        if group_file.exists():
            df = pd.read_csv(group_file)
            df['run'] = run
            df['phase'] = phase_id
            all_groups.append(df)

        # Load wind data
        wind_file = run_dir / "wind_data.csv"
        if wind_file.exists():
            df = pd.read_csv(wind_file)
            df['run'] = run
            all_wind.append(df)

        # Load agent data (use run 1 for detailed plots)
        if run == 1:
            for agent_file in run_dir.glob("agent_*.csv"):
                df = pd.read_csv(agent_file)
                # Extract controller type from filename
                name = agent_file.stem
                if '_pd' in name:
                    df['controller'] = 'PD'
                elif '_pid' in name:
                    df['controller'] = 'PID'
                elif '_it2' in name:
                    df['controller'] = 'IT2'
                elif '_gt2' in name:
                    df['controller'] = 'GT2'
                df['agent'] = name
                all_agents.append(df)

    groups = pd.concat(all_groups) if all_groups else pd.DataFrame()
    agents = all_agents  # List of DataFrames
    wind = pd.concat(all_wind) if all_wind else pd.DataFrame()

    return groups, agents, wind


def plot_rmse_comparison_bar(save_dir):
    """Bar chart comparing RMSE across all phases."""
    fig, ax = plt.subplots(figsize=(12, 6))

    phase_ids = list(PHASES.keys())
    x = np.arange(len(phase_ids))
    width = 0.2

    for i, ctrl in enumerate(['GT2', 'IT2', 'PID', 'PD']):
        means = []
        stds = []
        for phase_id in phase_ids:
            groups, _, _ = load_phase_data(phase_id)
            if not groups.empty:
                ctrl_data = groups[groups['controller_type'] == ctrl]['mean_rmse']
                means.append(ctrl_data.mean())
                stds.append(ctrl_data.std())
            else:
                means.append(0)
                stds.append(0)

        offset = (i - 1.5) * width
        bars = ax.bar(x + offset, means, width, yerr=stds, label=ctrl,
                     color=COLORS[ctrl], capsize=3, alpha=0.85)

    ax.set_xlabel('Phase')
    ax.set_ylabel('RMSE (m)')
    ax.set_title('Controller RMSE Comparison Across All Phases')
    ax.set_xticks(x)
    ax.set_xticklabels([f"P{p}\n{PHASES[p][:15]}" for p in phase_ids], fontsize=8)
    ax.legend(loc='upper left')
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(save_dir / "rmse_all_phases_bar.png", bbox_inches='tight')
    plt.savefig(save_dir / "rmse_all_phases_bar.pdf", bbox_inches='tight')
    plt.close()
    print("  [+] RMSE bar chart saved")


def plot_rmse_line(save_dir):
    """Line plot showing RMSE trend across phases."""
    fig, ax = plt.subplots(figsize=(10, 6))

    phase_ids = list(PHASES.keys())

    for ctrl in ['GT2', 'IT2', 'PID', 'PD']:
        means = []
        stds = []
        for phase_id in phase_ids:
            groups, _, _ = load_phase_data(phase_id)
            if not groups.empty:
                ctrl_data = groups[groups['controller_type'] == ctrl]['mean_rmse']
                means.append(ctrl_data.mean())
                stds.append(ctrl_data.std())
            else:
                means.append(np.nan)
                stds.append(0)

        ax.errorbar(range(len(phase_ids)), means, yerr=stds,
                   label=ctrl, color=COLORS[ctrl], marker=MARKERS[ctrl],
                   linewidth=2, markersize=8, capsize=4)

    ax.set_xlabel('Phase')
    ax.set_ylabel('RMSE (m)')
    ax.set_title('Controller Performance Trend Across Phases')
    ax.set_xticks(range(len(phase_ids)))
    ax.set_xticklabels([f"P{p}" for p in phase_ids])
    ax.legend(loc='upper left')
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(save_dir / "rmse_trend_line.png", bbox_inches='tight')
    plt.savefig(save_dir / "rmse_trend_line.pdf", bbox_inches='tight')
    plt.close()
    print("  [+] RMSE trend line saved")


def plot_individual_metrics(save_dir):
    """Generate individual metric plots for thesis."""
    metrics = {
        'mean_rmse': ('RMSE', 'm'),
        'ss_rmse': ('Steady-State RMSE', 'm'),
        'mean_mae': ('MAE', 'm'),
        'mean_peak_error': ('Peak Error', 'm'),
        'control_effort_iae': ('Control Effort (IAE)', 'N*s'),
    }

    for metric, (name, unit) in metrics.items():
        fig, axes = plt.subplots(2, 3, figsize=(14, 8))
        axes = axes.flatten()

        for idx, (phase_id, phase_name) in enumerate(PHASES.items()):
            ax = axes[idx]
            groups, _, _ = load_phase_data(phase_id)

            if groups.empty:
                ax.text(0.5, 0.5, 'No Data', ha='center', va='center')
                ax.set_title(f"Phase {phase_id}")
                continue

            ctrl_data = []
            ctrl_names = []
            for ctrl in ['GT2', 'IT2', 'PID', 'PD']:
                data = groups[groups['controller_type'] == ctrl][metric].values
                if len(data) > 0:
                    ctrl_data.append(data)
                    ctrl_names.append(ctrl)

            bp = ax.boxplot(ctrl_data, labels=ctrl_names, patch_artist=True)
            for patch, ctrl in zip(bp['boxes'], ctrl_names):
                patch.set_facecolor(COLORS[ctrl])
                patch.set_alpha(0.7)

            ax.set_title(f"P{phase_id}: {phase_name[:20]}")
            ax.set_ylabel(f"{name} ({unit})")
            ax.grid(True, alpha=0.3)

        # Hide extra subplot if any
        if len(PHASES) < 6:
            axes[-1].set_visible(False)

        plt.suptitle(f"{name} Distribution by Controller", fontsize=14, fontweight='bold')
        plt.tight_layout()
        plt.savefig(save_dir / f"{metric}_boxplot.png", bbox_inches='tight')
        plt.savefig(save_dir / f"{metric}_boxplot.pdf", bbox_inches='tight')
        plt.close()
        print(f"  [+] {name} boxplot saved")


def plot_error_evolution(phase_id, agents, save_dir):
    """Plot error magnitude over time for a phase."""
    fig, ax = plt.subplots(figsize=(12, 5))

    # Group agents by controller and average
    for ctrl in ['GT2', 'IT2', 'PID', 'PD']:
        ctrl_agents = [df for df in agents if df['controller'].iloc[0] == ctrl]
        if not ctrl_agents:
            continue

        # Use first agent of each controller type
        df = ctrl_agents[0]
        ax.plot(df['timestamp'].values, df['error_magnitude'].values,
               label=ctrl, color=COLORS[ctrl], linewidth=1.5, alpha=0.8)

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Position Error (m)')
    ax.set_title(f"Phase {phase_id}: {PHASES[phase_id]} - Error Evolution")
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)
    ax.set_xlim(0, 90)

    plt.tight_layout()
    plt.savefig(save_dir / f"phase_{phase_id}_error_evolution.png", bbox_inches='tight')
    plt.savefig(save_dir / f"phase_{phase_id}_error_evolution.pdf", bbox_inches='tight')
    plt.close()


def plot_trajectory_2d(phase_id, agents, save_dir):
    """Plot 2D trajectory (bird's eye view)."""
    fig, axes = plt.subplots(2, 2, figsize=(12, 12))
    axes = axes.flatten()

    for idx, ctrl in enumerate(['GT2', 'IT2', 'PID', 'PD']):
        ax = axes[idx]
        ctrl_agents = [df for df in agents if df['controller'].iloc[0] == ctrl]

        for i, df in enumerate(ctrl_agents[:3]):  # Max 3 agents per controller
            # Current position
            ax.plot(df['current_x'].values, df['current_y'].values,
                   color=COLORS[ctrl], linewidth=1, alpha=0.7,
                   label=f'Agent {i}' if i == 0 else None)
            # Target position (dashed)
            ax.plot(df['target_x'].values, df['target_y'].values,
                   color='gray', linewidth=0.5, linestyle='--', alpha=0.5)
            # Start point
            ax.scatter(df['current_x'].iloc[0], df['current_y'].iloc[0],
                      color=COLORS[ctrl], s=50, marker='o', zorder=5)
            # End point
            ax.scatter(df['current_x'].iloc[-1], df['current_y'].iloc[-1],
                      color=COLORS[ctrl], s=50, marker='x', zorder=5)

        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title(f'{ctrl} Controller')
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')
        ax.legend(loc='upper right')

    plt.suptitle(f"Phase {phase_id}: {PHASES[phase_id]} - 2D Trajectories",
                fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig(save_dir / f"phase_{phase_id}_trajectory_2d.png", bbox_inches='tight')
    plt.savefig(save_dir / f"phase_{phase_id}_trajectory_2d.pdf", bbox_inches='tight')
    plt.close()


def plot_wind_profile(phase_id, wind, save_dir):
    """Plot wind profile over time."""
    if wind.empty:
        return

    fig, axes = plt.subplots(2, 1, figsize=(12, 6), sharex=True)

    # Use run 1 data
    run1 = wind[wind['run'] == 1]
    if run1.empty:
        run1 = wind

    # Create time index
    t = np.arange(len(run1)) / (len(run1) / 90)  # Assume 90s simulation

    # Wind magnitude
    ax1 = axes[0]
    ax1.plot(t, run1['wind_magnitude'].values, color='#2c3e50', linewidth=1)
    ax1.fill_between(t, 0, run1['wind_magnitude'].values, alpha=0.3, color='#3498db')
    ax1.set_ylabel('Wind Magnitude (m/s)')
    ax1.set_title(f"Phase {phase_id}: {PHASES[phase_id]} - Wind Profile")
    ax1.grid(True, alpha=0.3)

    # Wind components
    ax2 = axes[1]
    ax2.plot(t, run1['wind_x'].values, label='Wind X', color='#e74c3c', linewidth=1)
    ax2.plot(t, run1['wind_y'].values, label='Wind Y', color='#27ae60', linewidth=1)
    if 'wind_z' in run1.columns:
        ax2.plot(t, run1['wind_z'].values, label='Wind Z', color='#9b59b6', linewidth=1)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Wind Component (m/s)')
    ax2.legend(loc='upper right')
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(save_dir / f"phase_{phase_id}_wind_profile.png", bbox_inches='tight')
    plt.savefig(save_dir / f"phase_{phase_id}_wind_profile.pdf", bbox_inches='tight')
    plt.close()


def plot_comprehensive(phase_id, groups, agents, wind, save_dir):
    """Generate comprehensive multi-panel plot for a phase."""
    fig = plt.figure(figsize=(16, 12))
    gs = gridspec.GridSpec(3, 3, figure=fig, hspace=0.3, wspace=0.3)

    # 1. RMSE Bar Chart (top-left)
    ax1 = fig.add_subplot(gs[0, 0])
    if not groups.empty:
        rmse_means = groups.groupby('controller_type')['mean_rmse'].mean()
        rmse_stds = groups.groupby('controller_type')['mean_rmse'].std()
        ctrls = ['GT2', 'IT2', 'PID', 'PD']
        x = np.arange(len(ctrls))
        bars = ax1.bar(x, [rmse_means.get(c, 0) for c in ctrls],
                      yerr=[rmse_stds.get(c, 0) for c in ctrls],
                      color=[COLORS[c] for c in ctrls], capsize=5, alpha=0.85)
        ax1.set_xticks(x)
        ax1.set_xticklabels(ctrls)
        ax1.set_ylabel('RMSE (m)')
        ax1.set_title('RMSE Comparison')
        ax1.grid(True, alpha=0.3)

    # 2. SS-RMSE Bar Chart (top-middle)
    ax2 = fig.add_subplot(gs[0, 1])
    if not groups.empty:
        ss_means = groups.groupby('controller_type')['ss_rmse'].mean()
        ss_stds = groups.groupby('controller_type')['ss_rmse'].std()
        bars = ax2.bar(x, [ss_means.get(c, 0) for c in ctrls],
                      yerr=[ss_stds.get(c, 0) for c in ctrls],
                      color=[COLORS[c] for c in ctrls], capsize=5, alpha=0.85)
        ax2.set_xticks(x)
        ax2.set_xticklabels(ctrls)
        ax2.set_ylabel('SS-RMSE (m)')
        ax2.set_title('Steady-State RMSE')
        ax2.grid(True, alpha=0.3)

    # 3. Peak Error Bar Chart (top-right)
    ax3 = fig.add_subplot(gs[0, 2])
    if not groups.empty:
        peak_means = groups.groupby('controller_type')['mean_peak_error'].mean()
        peak_stds = groups.groupby('controller_type')['mean_peak_error'].std()
        bars = ax3.bar(x, [peak_means.get(c, 0) for c in ctrls],
                      yerr=[peak_stds.get(c, 0) for c in ctrls],
                      color=[COLORS[c] for c in ctrls], capsize=5, alpha=0.85)
        ax3.set_xticks(x)
        ax3.set_xticklabels(ctrls)
        ax3.set_ylabel('Peak Error (m)')
        ax3.set_title('Maximum Error')
        ax3.grid(True, alpha=0.3)

    # 4. Error Evolution (middle-left, spans 2 cols)
    ax4 = fig.add_subplot(gs[1, :2])
    if agents:
        for ctrl in ['GT2', 'IT2', 'PID', 'PD']:
            ctrl_agents = [df for df in agents if df['controller'].iloc[0] == ctrl]
            if ctrl_agents:
                df = ctrl_agents[0]
                ax4.plot(df['timestamp'].values, df['error_magnitude'].values,
                        label=ctrl, color=COLORS[ctrl], linewidth=1.5, alpha=0.8)
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Position Error (m)')
        ax4.set_title('Error Evolution Over Time')
        ax4.legend(loc='upper right')
        ax4.grid(True, alpha=0.3)
        ax4.set_xlim(0, 90)

    # 5. Wind Profile (middle-right)
    ax5 = fig.add_subplot(gs[1, 2])
    if not wind.empty:
        run1 = wind[wind['run'] == 1] if 'run' in wind.columns else wind
        if not run1.empty:
            t = np.arange(len(run1)) / (len(run1) / 90)
            ax5.plot(t, run1['wind_magnitude'].values, color='#2c3e50', linewidth=1)
            ax5.fill_between(t, 0, run1['wind_magnitude'].values, alpha=0.3, color='#3498db')
            ax5.set_xlabel('Time (s)')
            ax5.set_ylabel('Wind (m/s)')
            ax5.set_title('Wind Magnitude')
            ax5.grid(True, alpha=0.3)

    # 6. 2D Trajectory Overview (bottom, spans all cols)
    ax6 = fig.add_subplot(gs[2, :])
    if agents:
        for ctrl in ['GT2', 'IT2', 'PID', 'PD']:
            ctrl_agents = [df for df in agents if df['controller'].iloc[0] == ctrl]
            for i, df in enumerate(ctrl_agents[:1]):  # Just one agent per controller
                ax6.plot(df['current_x'].values, df['current_y'].values,
                        color=COLORS[ctrl], linewidth=1.5, alpha=0.8,
                        label=ctrl if i == 0 else None)
        # Plot target trajectory (from any agent)
        if agents:
            df = agents[0]
            ax6.plot(df['target_x'].values, df['target_y'].values,
                    color='gray', linewidth=1, linestyle='--', alpha=0.5, label='Target')
        ax6.set_xlabel('X (m)')
        ax6.set_ylabel('Y (m)')
        ax6.set_title('2D Trajectory (Bird\'s Eye View)')
        ax6.legend(loc='upper right')
        ax6.grid(True, alpha=0.3)
        ax6.set_aspect('equal')

    plt.suptitle(f"Phase {phase_id}: {PHASES[phase_id]} - Comprehensive Analysis",
                fontsize=16, fontweight='bold')

    plt.savefig(save_dir / f"phase_{phase_id}_comprehensive.png", bbox_inches='tight')
    plt.savefig(save_dir / f"phase_{phase_id}_comprehensive.pdf", bbox_inches='tight')
    plt.close()


def generate_data_quality_report():
    """Generate data quality report."""
    report = []
    report.append("=" * 70)
    report.append("DATA QUALITY REPORT")
    report.append(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    report.append("=" * 70)

    for phase_id, phase_name in PHASES.items():
        report.append(f"\n{'─' * 70}")
        report.append(f"PHASE {phase_id}: {phase_name}")
        report.append('─' * 70)

        groups, agents, wind = load_phase_data(phase_id)

        # Check runs
        runs_found = len(groups['run'].unique()) if not groups.empty else 0
        report.append(f"  Runs found: {runs_found}/3")

        # Check agent data
        if agents:
            total_samples = sum(len(df) for df in agents)
            avg_samples = total_samples / len(agents)
            report.append(f"  Agent files: {len(agents)}")
            report.append(f"  Total samples: {total_samples}")
            report.append(f"  Avg samples/agent: {avg_samples:.0f}")

            # Check for NaN values
            nan_count = sum(df.isna().sum().sum() for df in agents)
            report.append(f"  NaN values: {nan_count}")

            # Check timestamp range
            min_t = min(df['timestamp'].min() for df in agents)
            max_t = max(df['timestamp'].max() for df in agents)
            report.append(f"  Time range: {min_t:.1f}s - {max_t:.1f}s")

        # Check wind data
        if not wind.empty:
            report.append(f"  Wind samples: {len(wind)}")
            report.append(f"  Wind max: {wind['wind_magnitude'].max():.2f} m/s")
            report.append(f"  Wind mean: {wind['wind_magnitude'].mean():.2f} m/s")

        # RMSE stats
        if not groups.empty:
            for ctrl in ['GT2', 'IT2', 'PID', 'PD']:
                ctrl_data = groups[groups['controller_type'] == ctrl]['mean_rmse']
                if len(ctrl_data) > 0:
                    cv = ctrl_data.std() / ctrl_data.mean() * 100
                    status = "OK" if cv < 15 else "HIGH_VAR"
                    report.append(f"  {ctrl} RMSE: {ctrl_data.mean():.3f}±{ctrl_data.std():.3f}m (CV={cv:.1f}%) [{status}]")

    report.append("\n" + "=" * 70)
    report.append("END OF REPORT")
    report.append("=" * 70)

    return "\n".join(report)


def main():
    print("=" * 70)
    print("SCIENTIFIC VALIDATION - PLOT GENERATOR")
    print("=" * 70)

    # 1. Individual metric plots
    print("\n[1/5] Generating individual metric plots...")
    plot_individual_metrics(PLOT_DIR / "individual")

    # 2. Overall comparison plots
    print("\n[2/5] Generating overall comparison plots...")
    plot_rmse_comparison_bar(PLOT_DIR / "individual")
    plot_rmse_line(PLOT_DIR / "individual")

    # 3. Per-phase comprehensive plots
    print("\n[3/5] Generating comprehensive plots per phase...")
    for phase_id in PHASES.keys():
        print(f"  Processing Phase {phase_id}...")
        groups, agents, wind = load_phase_data(phase_id)

        # Comprehensive plot
        plot_comprehensive(phase_id, groups, agents, wind, PLOT_DIR / "comprehensive")

        # Error evolution
        if agents:
            plot_error_evolution(phase_id, agents, PLOT_DIR / "individual")

        # Trajectory
        if agents:
            plot_trajectory_2d(phase_id, agents, PLOT_DIR / "trajectories")

        # Wind profile
        if not wind.empty:
            plot_wind_profile(phase_id, wind, PLOT_DIR / "wind")

    # 4. Data quality report
    print("\n[4/5] Generating data quality report...")
    report = generate_data_quality_report()
    report_path = PLOT_DIR / "data_quality" / "quality_report.txt"
    with open(report_path, 'w') as f:
        f.write(report)
    print(report)

    # 5. Summary
    print("\n[5/5] Plot generation complete!")
    print(f"\nPlots saved to: {PLOT_DIR}")
    print("\nFolder structure:")
    print("  plots/")
    print("  ├── individual/     - Single metric plots (RMSE, MAE, etc.)")
    print("  ├── comprehensive/  - Multi-panel phase summaries")
    print("  ├── trajectories/   - 2D trajectory plots")
    print("  ├── wind/           - Wind profile plots")
    print("  └── data_quality/   - Quality report")

    # Count files
    total_files = sum(1 for _ in PLOT_DIR.rglob("*.png"))
    print(f"\nTotal plots generated: {total_files} PNG files")


if __name__ == "__main__":
    main()
