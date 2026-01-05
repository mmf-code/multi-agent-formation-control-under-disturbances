#!/usr/bin/env python3
"""
Tez Seviyesinde Kapsamlı Raporlama Scripti
==========================================

9 Kategori Grafik + Çapraz Faz Karşılaştırma:
1. PERFORMANS: RMSE Evolution, XYZ Error Decomposition, SS Error Box Plot
2. GÜVENLİK: Min Inter-Agent Distance, Collision Risk Histogram
3. ENERJİ: Control Effort, Jerk Analysis
4. YÖRÜNGE: 2D Trajectory, Altitude Hold

Kullanım:
    python3 scripts/generate_comprehensive_plots.py \
        --results-dir results/thesis_final \
        --output-dir results/thesis_final/plots \
        --format both

Author: Claude Code (Tez Raporlama)
Date: 2026-01-05
"""

import os
import sys
import json
import argparse
from pathlib import Path
from datetime import datetime
from typing import Dict, List, Optional, Tuple, Any
from dataclasses import dataclass
import warnings

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.backends.backend_pdf import PdfPages

# Suppress warnings for cleaner output
warnings.filterwarnings('ignore')

# Try to import seaborn for better styling
try:
    import seaborn as sns
    HAS_SEABORN = True
except ImportError:
    HAS_SEABORN = False
    print("Warning: seaborn not installed. Using matplotlib defaults.")


# =============================================================================
# KONFIGÜRASYON - Boolean Flag'ler
# =============================================================================
PLOT_ENABLED = {
    # Kategori 1: PERFORMANS (Tracking)
    "rmse_evolution": True,           # RMSE zaman serisi
    "xyz_error_decomposition": True,  # X, Y, Z hata ayrımı
    "ss_error_boxplot": True,         # Steady-state hata dağılımı

    # Kategori 2: GÜVENLİK (Safety)
    "min_inter_agent_distance": True, # Min ajan arası mesafe
    "collision_risk_histogram": True, # Çarpışma riski histogram

    # Kategori 3: ENERJİ (Effort)
    "control_effort": True,           # Kontrol eforu (IAE)
    "jerk_analysis": True,            # Sarsıntı analizi

    # Kategori 4: YÖRÜNGE (Spatial)
    "trajectory_2d": True,            # Kuş bakışı X-Y
    "altitude_hold": True,            # Yükseklik koruma Z-t

    # BONUS Grafikler
    "wind_profile": True,             # Rüzgar profili zaman serisi (YENİ)
    "wind_correlation": True,         # Rüzgar-hata korelasyonu
    "phase_comparison_heatmap": True, # Faz×Kontrolcü heatmap
    "controller_ranking": True,       # Kontrolcü sıralaması
}

# Stil Ayarları
STYLE_CONFIG = {
    "dpi": 300,
    "figsize_single": (10, 6),
    "figsize_multi": (16, 12),
    "font_size": 10,
    "title_size": 12,
    "legend_size": 9,
}

# Kontrolcü Renkleri (Tutarlılık için)
CONTROLLER_COLORS = {
    "PD": "#E74C3C",   # Kırmızı - baseline
    "PID": "#3498DB",  # Mavi - klasik
    "IT2": "#27AE60",  # Yeşil - interval type-2
    "GT2": "#9B59B6",  # Mor - general type-2
}

# Faz İsimleri
PHASE_NAMES = {
    1: "BASELINE",
    2: "STEADY_WIND",
    3: "TURBULENCE",
    4: "GUST",
    5: "COMBINED",
    6: "ENDURANCE",
}

# Güvenlik Eşikleri
SAFETY_DISTANCE = 0.8  # [m] Güvenli mesafe sınırı
COLLISION_DISTANCE = 0.5  # [m] Çarpışma mesafesi


# =============================================================================
# VERİ YÜKLEME FONKSİYONLARI
# =============================================================================

@dataclass
class PhaseData:
    """Bir faz için tüm veriler."""
    phase_id: int
    run_index: int
    agents: Dict[str, pd.DataFrame]  # agent_id -> DataFrame
    group_summary: pd.DataFrame
    wind_data: pd.DataFrame
    metadata: Dict[str, Any]


def load_agent_csv(filepath: Path) -> pd.DataFrame:
    """Ajan CSV dosyasını yükle."""
    try:
        df = pd.read_csv(filepath)
        return df
    except Exception as e:
        print(f"Warning: Could not load {filepath}: {e}")
        return pd.DataFrame()


def load_phase_data(phase_dir: Path) -> Optional[PhaseData]:
    """Bir faz klasöründen tüm verileri yükle."""
    if not phase_dir.exists():
        return None

    # Find run directories
    run_dirs = sorted(phase_dir.glob("run_*"))
    if not run_dirs:
        return None

    # Use first run (or could aggregate multiple runs)
    run_dir = run_dirs[0]
    run_index = int(run_dir.name.split("_")[1])

    # Load agent CSVs
    agents = {}
    agent_files = sorted(run_dir.glob("agent_*.csv"))
    for agent_file in agent_files:
        # Extract agent_id and controller type from filename
        parts = agent_file.stem.split("_")
        if len(parts) >= 3:
            agent_id = f"{parts[0]}_{parts[1]}"
            controller_type = parts[2].upper()
            df = load_agent_csv(agent_file)
            if not df.empty:
                df['controller_type'] = controller_type
                df['agent_id'] = agent_id
                agents[agent_id] = df

    # Load group summary
    group_summary_path = run_dir / "group_summary.csv"
    group_summary = pd.DataFrame()
    if group_summary_path.exists():
        group_summary = pd.read_csv(group_summary_path)

    # Load wind data
    wind_path = run_dir / "wind_data.csv"
    wind_data = pd.DataFrame()
    if wind_path.exists():
        wind_data = pd.read_csv(wind_path)

    # Load metadata
    metadata_path = run_dir / "phase_metadata.json"
    metadata = {}
    if metadata_path.exists():
        with open(metadata_path) as f:
            metadata = json.load(f)

    # Extract phase_id from directory name
    phase_id = int(phase_dir.name.split("_")[1])

    return PhaseData(
        phase_id=phase_id,
        run_index=run_index,
        agents=agents,
        group_summary=group_summary,
        wind_data=wind_data,
        metadata=metadata,
    )


def load_all_phases(results_dir: Path) -> Dict[int, PhaseData]:
    """Tüm fazları yükle."""
    phases = {}
    for phase_dir in sorted(results_dir.glob("phase_*")):
        if phase_dir.is_dir():
            phase_data = load_phase_data(phase_dir)
            if phase_data:
                phases[phase_data.phase_id] = phase_data
                print(f"Loaded Phase {phase_data.phase_id}: "
                      f"{len(phase_data.agents)} agents, "
                      f"{len(phase_data.wind_data)} wind samples")
    return phases


# =============================================================================
# YARDIMCI FONKSİYONLAR
# =============================================================================

def get_controller_data(phase: PhaseData, controller_type: str) -> pd.DataFrame:
    """Belirli bir kontrolcü tipinin tüm ajan verilerini birleştir."""
    dfs = []
    for agent_id, df in phase.agents.items():
        if 'controller_type' in df.columns and df['controller_type'].iloc[0] == controller_type:
            dfs.append(df)
    if dfs:
        return pd.concat(dfs, ignore_index=True)
    return pd.DataFrame()


def get_steady_state_data(df: pd.DataFrame, ss_start: float = 15.0) -> pd.DataFrame:
    """Steady-state verilerini (t > ss_start) al."""
    if 'timestamp' in df.columns:
        return df[df['timestamp'] >= ss_start]
    return df


def compute_inter_agent_distances(phase: PhaseData) -> Dict[str, List[float]]:
    """Tüm ajanlar arasındaki minimum mesafeleri hesapla."""
    distances = {ct: [] for ct in CONTROLLER_COLORS.keys()}

    # Group agents by controller type
    controller_agents = {}
    for agent_id, df in phase.agents.items():
        if 'controller_type' in df.columns and not df.empty:
            ct = df['controller_type'].iloc[0]
            if ct not in controller_agents:
                controller_agents[ct] = []
            controller_agents[ct].append(df)

    # For each controller type, compute min distances within group
    for ct, agent_dfs in controller_agents.items():
        if len(agent_dfs) < 2:
            continue

        # Get common timestamps
        min_len = min(len(df) for df in agent_dfs)

        for t_idx in range(0, min_len, 10):  # Sample every 10 points
            positions = []
            for df in agent_dfs:
                if 'current_x' in df.columns and 'current_y' in df.columns:
                    x = df['current_x'].iloc[t_idx]
                    y = df['current_y'].iloc[t_idx]
                    positions.append((x, y))

            if len(positions) >= 2:
                # Compute all pairwise distances
                min_dist = float('inf')
                for i in range(len(positions)):
                    for j in range(i + 1, len(positions)):
                        dist = np.sqrt((positions[i][0] - positions[j][0])**2 +
                                       (positions[i][1] - positions[j][1])**2)
                        min_dist = min(min_dist, dist)
                if min_dist < float('inf'):
                    distances[ct].append(min_dist)

    return distances


def compute_jerk(velocity: np.ndarray, dt: float = 0.1) -> np.ndarray:
    """Velocity verisinden jerk (acceleration derivative) hesapla."""
    if len(velocity) < 3:
        return np.array([])

    # First compute acceleration
    accel = np.diff(velocity) / dt
    # Then compute jerk
    jerk = np.diff(accel) / dt
    return jerk


# =============================================================================
# GRAFİK FONKSİYONLARI
# =============================================================================

def plot_rmse_evolution(phase: PhaseData, ax: plt.Axes):
    """RMSE zaman serisi grafiği."""
    ax.set_title(f"RMSE Evolution - Phase {phase.phase_id} ({PHASE_NAMES.get(phase.phase_id, '')})",
                 fontsize=STYLE_CONFIG['title_size'])

    for ct in ['PD', 'PID', 'IT2', 'GT2']:
        df = get_controller_data(phase, ct)
        if df.empty or 'timestamp' not in df.columns:
            continue

        # Group by timestamp and compute mean error magnitude
        grouped = df.groupby('timestamp')['error_magnitude'].agg(['mean', 'std'])
        timestamps = grouped.index.to_numpy()
        mean_error = grouped['mean'].to_numpy()
        std_error = grouped['std'].fillna(0).to_numpy()

        color = CONTROLLER_COLORS.get(ct, 'gray')
        ax.plot(timestamps, mean_error, label=ct, color=color, linewidth=1.5)
        ax.fill_between(timestamps, mean_error - std_error, mean_error + std_error,
                        alpha=0.2, color=color)

    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Error Magnitude [m]")
    ax.legend(loc='upper right', fontsize=STYLE_CONFIG['legend_size'])
    ax.grid(True, alpha=0.3)
    ax.set_xlim(left=0)
    ax.set_ylim(bottom=0)


def plot_xyz_error_decomposition(phase: PhaseData, ax: plt.Axes):
    """X, Y, Z eksenleri için hata ayrımı."""
    ax.set_title(f"XYZ Error Decomposition - Phase {phase.phase_id}",
                 fontsize=STYLE_CONFIG['title_size'])

    bar_width = 0.2
    controllers = ['PD', 'PID', 'IT2', 'GT2']
    x = np.arange(len(controllers))

    for i, axis in enumerate(['X', 'Y', 'Z']):
        col = f'error_{axis.lower()}'
        values = []
        for ct in controllers:
            df = get_controller_data(phase, ct)
            if not df.empty and col in df.columns:
                ss_df = get_steady_state_data(df)
                values.append(ss_df[col].abs().mean())
            else:
                values.append(0)

        colors = ['#3498DB', '#2ECC71', '#E74C3C'][i]
        ax.bar(x + i * bar_width, values, bar_width, label=f'{axis}-axis', color=colors)

    ax.set_xlabel("Controller")
    ax.set_ylabel("Mean Absolute Error [m]")
    ax.set_xticks(x + bar_width)
    ax.set_xticklabels(controllers)
    ax.legend(fontsize=STYLE_CONFIG['legend_size'])
    ax.grid(True, alpha=0.3, axis='y')


def plot_ss_error_boxplot(phase: PhaseData, ax: plt.Axes):
    """Steady-state hata dağılımı kutu grafiği."""
    ax.set_title(f"Steady-State Error Distribution - Phase {phase.phase_id}",
                 fontsize=STYLE_CONFIG['title_size'])

    data = []
    labels = []
    colors = []

    for ct in ['PD', 'PID', 'IT2', 'GT2']:
        df = get_controller_data(phase, ct)
        if df.empty or 'error_magnitude' not in df.columns:
            continue

        ss_df = get_steady_state_data(df)
        if not ss_df.empty:
            data.append(ss_df['error_magnitude'].values)
            labels.append(ct)
            colors.append(CONTROLLER_COLORS.get(ct, 'gray'))

    if data:
        bp = ax.boxplot(data, labels=labels, patch_artist=True)
        for patch, color in zip(bp['boxes'], colors):
            patch.set_facecolor(color)
            patch.set_alpha(0.7)

    ax.set_xlabel("Controller")
    ax.set_ylabel("Error Magnitude [m]")
    ax.grid(True, alpha=0.3, axis='y')


def plot_min_inter_agent_distance(phase: PhaseData, ax: plt.Axes):
    """Minimum ajan arası mesafe grafiği."""
    ax.set_title(f"Min Inter-Agent Distance - Phase {phase.phase_id}",
                 fontsize=STYLE_CONFIG['title_size'])

    distances = compute_inter_agent_distances(phase)

    has_data = False
    for ct in ['PD', 'PID', 'IT2', 'GT2']:
        if ct in distances and distances[ct]:
            color = CONTROLLER_COLORS.get(ct, 'gray')
            ax.plot(distances[ct], label=ct, color=color, linewidth=1)
            has_data = True

    if has_data:
        ax.axhline(y=SAFETY_DISTANCE, color='orange', linestyle='--',
                   label=f'Safety Threshold ({SAFETY_DISTANCE}m)')
        ax.axhline(y=COLLISION_DISTANCE, color='red', linestyle='--',
                   label=f'Collision Threshold ({COLLISION_DISTANCE}m)')
        ax.legend(loc='lower right', fontsize=STYLE_CONFIG['legend_size'])
    else:
        ax.text(0.5, 0.5, "No position data available\n(requires updated logger)",
                ha='center', va='center', transform=ax.transAxes,
                fontsize=12, color='gray')

    ax.set_xlabel("Sample Index")
    ax.set_ylabel("Min Distance [m]")
    ax.grid(True, alpha=0.3)
    ax.set_ylim(bottom=0)


def plot_collision_risk_histogram(phase: PhaseData, ax: plt.Axes):
    """Çarpışma riski histogram."""
    ax.set_title(f"Collision Risk Histogram - Phase {phase.phase_id}",
                 fontsize=STYLE_CONFIG['title_size'])

    distances = compute_inter_agent_distances(phase)

    has_data = False
    for ct in ['PD', 'PID', 'IT2', 'GT2']:
        if ct in distances and distances[ct]:
            color = CONTROLLER_COLORS.get(ct, 'gray')
            ax.hist(distances[ct], bins=30, alpha=0.5, label=ct, color=color)
            has_data = True

    if has_data:
        ax.axvline(x=SAFETY_DISTANCE, color='orange', linestyle='--',
                   label=f'Safety ({SAFETY_DISTANCE}m)')
        ax.axvline(x=COLLISION_DISTANCE, color='red', linestyle='--',
                   label=f'Collision ({COLLISION_DISTANCE}m)')
        ax.legend(fontsize=STYLE_CONFIG['legend_size'])
    else:
        ax.text(0.5, 0.5, "No position data available",
                ha='center', va='center', transform=ax.transAxes,
                fontsize=12, color='gray')

    ax.set_xlabel("Distance [m]")
    ax.set_ylabel("Frequency")
    ax.grid(True, alpha=0.3)


def plot_control_effort(phase: PhaseData, ax: plt.Axes):
    """Kontrol eforu (IAE) karşılaştırma."""
    ax.set_title(f"Control Effort (IAE) - Phase {phase.phase_id}",
                 fontsize=STYLE_CONFIG['title_size'])

    if phase.group_summary.empty or 'control_effort_iae' not in phase.group_summary.columns:
        ax.text(0.5, 0.5, "No control effort data",
                ha='center', va='center', transform=ax.transAxes)
        return

    controllers = phase.group_summary['controller_type'].tolist()
    efforts = phase.group_summary['control_effort_iae'].tolist()
    colors = [CONTROLLER_COLORS.get(ct, 'gray') for ct in controllers]

    bars = ax.bar(controllers, efforts, color=colors, alpha=0.8)

    # Add value labels
    for bar, val in zip(bars, efforts):
        ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 5,
                f'{val:.1f}', ha='center', va='bottom', fontsize=9)

    ax.set_xlabel("Controller")
    ax.set_ylabel("Control Effort (IAE)")
    ax.grid(True, alpha=0.3, axis='y')


def plot_jerk_analysis(phase: PhaseData, ax: plt.Axes):
    """Jerk (sarsıntı) analizi."""
    ax.set_title(f"Jerk Analysis (Smoothness) - Phase {phase.phase_id}",
                 fontsize=STYLE_CONFIG['title_size'])

    jerk_stats = []
    controllers = []

    for ct in ['PD', 'PID', 'IT2', 'GT2']:
        df = get_controller_data(phase, ct)
        if df.empty:
            continue

        if 'velocity_x' in df.columns and 'velocity_y' in df.columns:
            vx = df['velocity_x'].values
            vy = df['velocity_y'].values

            # Compute jerk magnitude
            jerk_x = compute_jerk(vx)
            jerk_y = compute_jerk(vy)

            if len(jerk_x) > 0 and len(jerk_y) > 0:
                min_len = min(len(jerk_x), len(jerk_y))
                jerk_mag = np.sqrt(jerk_x[:min_len]**2 + jerk_y[:min_len]**2)
                jerk_stats.append(np.mean(np.abs(jerk_mag)))
                controllers.append(ct)

    if jerk_stats:
        colors = [CONTROLLER_COLORS.get(ct, 'gray') for ct in controllers]
        bars = ax.bar(controllers, jerk_stats, color=colors, alpha=0.8)

        for bar, val in zip(bars, jerk_stats):
            ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.01,
                    f'{val:.3f}', ha='center', va='bottom', fontsize=9)

        ax.set_xlabel("Controller")
        ax.set_ylabel("Mean Jerk [m/s³]")
        ax.grid(True, alpha=0.3, axis='y')
    else:
        ax.text(0.5, 0.5, "No velocity data available\n(requires updated logger)",
                ha='center', va='center', transform=ax.transAxes,
                fontsize=12, color='gray')


def plot_trajectory_2d(phase: PhaseData, ax: plt.Axes):
    """2D yörünge grafiği (kuş bakışı X-Y)."""
    ax.set_title(f"2D Trajectory (Top View) - Phase {phase.phase_id}",
                 fontsize=STYLE_CONFIG['title_size'])

    has_data = False
    for ct in ['PD', 'PID', 'IT2', 'GT2']:
        df = get_controller_data(phase, ct)
        if df.empty:
            continue

        if 'current_x' in df.columns and 'current_y' in df.columns:
            # Get one representative agent
            first_agent = df[df['agent_id'] == df['agent_id'].iloc[0]]
            x = first_agent['current_x'].values
            y = first_agent['current_y'].values

            color = CONTROLLER_COLORS.get(ct, 'gray')
            ax.plot(x, y, label=ct, color=color, linewidth=1, alpha=0.7)
            ax.scatter(x[0], y[0], color=color, marker='o', s=50, zorder=5)  # Start
            ax.scatter(x[-1], y[-1], color=color, marker='s', s=50, zorder=5)  # End
            has_data = True

            # Plot target trajectory if available
            if 'target_x' in first_agent.columns and 'target_y' in first_agent.columns:
                tx = first_agent['target_x'].values
                ty = first_agent['target_y'].values
                ax.plot(tx, ty, '--', color=color, alpha=0.3, linewidth=1)

    if has_data:
        ax.legend(fontsize=STYLE_CONFIG['legend_size'])
        ax.set_aspect('equal')
    else:
        ax.text(0.5, 0.5, "No position data available\n(requires updated logger)",
                ha='center', va='center', transform=ax.transAxes,
                fontsize=12, color='gray')

    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.grid(True, alpha=0.3)


def plot_altitude_hold(phase: PhaseData, ax: plt.Axes):
    """Yükseklik koruma grafiği (Z-time)."""
    ax.set_title(f"Altitude Holding (Z vs Time) - Phase {phase.phase_id}",
                 fontsize=STYLE_CONFIG['title_size'])

    has_data = False
    for ct in ['PD', 'PID', 'IT2', 'GT2']:
        df = get_controller_data(phase, ct)
        if df.empty:
            continue

        if 'current_z' in df.columns and 'timestamp' in df.columns:
            grouped = df.groupby('timestamp')['current_z'].mean()
            color = CONTROLLER_COLORS.get(ct, 'gray')
            # Convert to numpy arrays to avoid pandas indexing issues
            ax.plot(grouped.index.to_numpy(), grouped.values, label=ct, color=color, linewidth=1)
            has_data = True

    if has_data:
        ax.legend(fontsize=STYLE_CONFIG['legend_size'])
    else:
        ax.text(0.5, 0.5, "No altitude data available\n(requires updated logger)",
                ha='center', va='center', transform=ax.transAxes,
                fontsize=12, color='gray')

    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Altitude Z [m]")
    ax.grid(True, alpha=0.3)


def plot_wind_profile(phase: PhaseData, ax: plt.Axes):
    """Rüzgar profili zaman serisi - bozucu görselleştirme."""
    ax.set_title(f"Wind Disturbance Profile - Phase {phase.phase_id} ({PHASE_NAMES.get(phase.phase_id, '')})",
                 fontsize=STYLE_CONFIG['title_size'])

    if phase.wind_data.empty:
        ax.text(0.5, 0.5, "No wind data available",
                ha='center', va='center', transform=ax.transAxes,
                fontsize=12, color='gray')
        return

    timestamps = phase.wind_data['timestamp'].to_numpy()
    wind_x = phase.wind_data['wind_x'].to_numpy()
    wind_y = phase.wind_data['wind_y'].to_numpy()
    wind_mag = phase.wind_data['wind_magnitude'].to_numpy()

    # Plot wind components
    ax.plot(timestamps, wind_x, label='Wind X', color='#E74C3C', linewidth=1, alpha=0.7)
    ax.plot(timestamps, wind_y, label='Wind Y', color='#3498DB', linewidth=1, alpha=0.7)
    ax.plot(timestamps, wind_mag, label='Magnitude', color='#2C3E50', linewidth=2)

    # Add phase-specific annotations
    phase_id = phase.phase_id
    if phase_id == 1:
        ax.axhline(y=0, color='green', linestyle='--', alpha=0.5, label='No Wind')
    elif phase_id == 2:
        mean_mag = np.mean(wind_mag[wind_mag > 0.1]) if np.any(wind_mag > 0.1) else 0
        ax.axhline(y=mean_mag, color='orange', linestyle='--', alpha=0.7,
                   label=f'Steady: {mean_mag:.1f} m/s')
    elif phase_id == 3:
        # Turbulence - show variation band
        mean_mag = np.mean(wind_mag[wind_mag > 0.1])
        std_mag = np.std(wind_mag[wind_mag > 0.1])
        ax.axhspan(mean_mag - std_mag, mean_mag + std_mag, alpha=0.2, color='purple',
                   label=f'TI band: {mean_mag:.1f}±{std_mag:.2f}')
    elif phase_id == 4:
        # Gust - mark gust peaks
        gust_threshold = np.max(wind_mag) * 0.5
        ax.axhline(y=gust_threshold, color='red', linestyle=':', alpha=0.5,
                   label=f'Gust threshold')

    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Wind Velocity [m/s]")
    ax.legend(loc='upper right', fontsize=STYLE_CONFIG['legend_size'])
    ax.grid(True, alpha=0.3)
    ax.set_xlim(left=0)
    ax.set_ylim(bottom=-0.5)


def plot_wind_correlation(phase: PhaseData, ax: plt.Axes):
    """Rüzgar-hata korelasyonu."""
    ax.set_title(f"Wind vs Error Correlation - Phase {phase.phase_id}",
                 fontsize=STYLE_CONFIG['title_size'])

    if phase.wind_data.empty:
        ax.text(0.5, 0.5, "No wind data available",
                ha='center', va='center', transform=ax.transAxes)
        return

    wind_mag = phase.wind_data['wind_magnitude'].values

    for ct in ['PD', 'PID', 'IT2', 'GT2']:
        df = get_controller_data(phase, ct)
        if df.empty or 'error_magnitude' not in df.columns:
            continue

        # Resample error to match wind data length
        error_mag = df.groupby('timestamp')['error_magnitude'].mean().values
        min_len = min(len(wind_mag), len(error_mag))

        if min_len > 10:
            color = CONTROLLER_COLORS.get(ct, 'gray')
            ax.scatter(wind_mag[:min_len], error_mag[:min_len],
                       alpha=0.3, s=10, color=color, label=ct)

    ax.set_xlabel("Wind Magnitude [m/s]")
    ax.set_ylabel("Error Magnitude [m]")
    ax.legend(fontsize=STYLE_CONFIG['legend_size'])
    ax.grid(True, alpha=0.3)


# =============================================================================
# RAPOR ÜRETİM FONKSİYONLARI
# =============================================================================

def generate_phase_report(phase: PhaseData, output_dir: Path, fmt: str = 'both'):
    """Tek bir faz için tüm grafikleri üret."""
    print(f"\nGenerating Phase {phase.phase_id} report...")

    # Setup figure with 4x3 grid (expanded for wind profile)
    fig = plt.figure(figsize=(18, 20))
    gs = gridspec.GridSpec(4, 3, figure=fig, hspace=0.3, wspace=0.3)

    # Apply style
    if HAS_SEABORN:
        sns.set_style("whitegrid")
        sns.set_palette("husl")

    plt.rcParams.update({
        'font.size': STYLE_CONFIG['font_size'],
        'axes.titlesize': STYLE_CONFIG['title_size'],
        'legend.fontsize': STYLE_CONFIG['legend_size'],
    })

    # Row 1: Performance
    if PLOT_ENABLED['rmse_evolution']:
        ax1 = fig.add_subplot(gs[0, 0])
        plot_rmse_evolution(phase, ax1)

    if PLOT_ENABLED['xyz_error_decomposition']:
        ax2 = fig.add_subplot(gs[0, 1])
        plot_xyz_error_decomposition(phase, ax2)

    if PLOT_ENABLED['ss_error_boxplot']:
        ax3 = fig.add_subplot(gs[0, 2])
        plot_ss_error_boxplot(phase, ax3)

    # Row 2: Safety & Energy
    if PLOT_ENABLED['min_inter_agent_distance']:
        ax4 = fig.add_subplot(gs[1, 0])
        plot_min_inter_agent_distance(phase, ax4)

    if PLOT_ENABLED['control_effort']:
        ax5 = fig.add_subplot(gs[1, 1])
        plot_control_effort(phase, ax5)

    if PLOT_ENABLED['jerk_analysis']:
        ax6 = fig.add_subplot(gs[1, 2])
        plot_jerk_analysis(phase, ax6)

    # Row 3: Trajectory & Wind
    if PLOT_ENABLED['trajectory_2d']:
        ax7 = fig.add_subplot(gs[2, 0])
        plot_trajectory_2d(phase, ax7)

    if PLOT_ENABLED['altitude_hold']:
        ax8 = fig.add_subplot(gs[2, 1])
        plot_altitude_hold(phase, ax8)

    if PLOT_ENABLED['wind_correlation']:
        ax9 = fig.add_subplot(gs[2, 2])
        plot_wind_correlation(phase, ax9)

    # Row 4: Wind Profile (yeni eklenen - bozucu görselleştirme)
    if PLOT_ENABLED['wind_profile']:
        ax10 = fig.add_subplot(gs[3, :])  # Tüm satırı kullan - geniş görünüm
        plot_wind_profile(phase, ax10)

    # Main title
    phase_name = PHASE_NAMES.get(phase.phase_id, 'UNKNOWN')
    fig.suptitle(f"Phase {phase.phase_id}: {phase_name} - Comprehensive Analysis",
                 fontsize=14, fontweight='bold', y=0.98)

    # Save outputs
    output_dir.mkdir(parents=True, exist_ok=True)

    if fmt in ['pdf', 'both']:
        pdf_path = output_dir / f"phase_{phase.phase_id}_comprehensive.pdf"
        fig.savefig(pdf_path, dpi=STYLE_CONFIG['dpi'], bbox_inches='tight')
        print(f"  Saved: {pdf_path}")

    if fmt in ['png', 'both']:
        png_path = output_dir / f"phase_{phase.phase_id}_comprehensive.png"
        fig.savefig(png_path, dpi=STYLE_CONFIG['dpi'], bbox_inches='tight')
        print(f"  Saved: {png_path}")

    plt.close(fig)


def generate_cross_phase_comparison(phases: Dict[int, PhaseData], output_dir: Path, fmt: str = 'both'):
    """Tüm fazlar arası karşılaştırma grafiği."""
    print("\nGenerating cross-phase comparison...")

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    if HAS_SEABORN:
        sns.set_style("whitegrid")

    # 1. RMSE Heatmap (Phase x Controller)
    ax1 = axes[0, 0]
    rmse_data = []
    phase_ids = []
    for pid, phase in sorted(phases.items()):
        if not phase.group_summary.empty and 'mean_rmse' in phase.group_summary.columns:
            row = {'Phase': PHASE_NAMES.get(pid, str(pid))}
            for _, r in phase.group_summary.iterrows():
                row[r['controller_type']] = r['mean_rmse']
            rmse_data.append(row)
            phase_ids.append(pid)

    if rmse_data:
        rmse_df = pd.DataFrame(rmse_data)
        rmse_df = rmse_df.set_index('Phase')

        if HAS_SEABORN:
            sns.heatmap(rmse_df, annot=True, fmt='.3f', cmap='RdYlGn_r',
                        ax=ax1, cbar_kws={'label': 'RMSE [m]'})
        else:
            im = ax1.imshow(rmse_df.values, cmap='RdYlGn_r', aspect='auto')
            plt.colorbar(im, ax=ax1, label='RMSE [m]')
            ax1.set_xticks(range(len(rmse_df.columns)))
            ax1.set_xticklabels(rmse_df.columns)
            ax1.set_yticks(range(len(rmse_df.index)))
            ax1.set_yticklabels(rmse_df.index)

    ax1.set_title("RMSE Heatmap (Phase × Controller)")

    # 2. Controller Ranking (Win Count)
    ax2 = axes[0, 1]
    wins = {ct: 0 for ct in CONTROLLER_COLORS.keys()}
    for pid, phase in phases.items():
        if not phase.group_summary.empty and 'mean_rmse' in phase.group_summary.columns:
            best = phase.group_summary.loc[phase.group_summary['mean_rmse'].idxmin()]
            if best['controller_type'] in wins:
                wins[best['controller_type']] += 1

    colors = [CONTROLLER_COLORS.get(ct, 'gray') for ct in wins.keys()]
    ax2.bar(wins.keys(), wins.values(), color=colors, alpha=0.8)
    ax2.set_title("Controller Win Count (Best RMSE)")
    ax2.set_xlabel("Controller")
    ax2.set_ylabel("Win Count")
    ax2.grid(True, alpha=0.3, axis='y')

    # 3. SS-RMSE Comparison
    ax3 = axes[1, 0]
    for ct in ['PD', 'PID', 'IT2', 'GT2']:
        ss_rmses = []
        pids = []
        for pid, phase in sorted(phases.items()):
            if not phase.group_summary.empty:
                row = phase.group_summary[phase.group_summary['controller_type'] == ct]
                if not row.empty and 'ss_rmse' in row.columns:
                    ss_rmses.append(row['ss_rmse'].values[0])
                    pids.append(pid)

        if ss_rmses:
            color = CONTROLLER_COLORS.get(ct, 'gray')
            ax3.plot(pids, ss_rmses, 'o-', label=ct, color=color, linewidth=2, markersize=8)

    ax3.set_title("Steady-State RMSE Across Phases")
    ax3.set_xlabel("Phase")
    ax3.set_ylabel("SS-RMSE [m]")
    ax3.legend(fontsize=STYLE_CONFIG['legend_size'])
    ax3.grid(True, alpha=0.3)

    # 4. Control Effort Comparison
    ax4 = axes[1, 1]
    for ct in ['PD', 'PID', 'IT2', 'GT2']:
        efforts = []
        pids = []
        for pid, phase in sorted(phases.items()):
            if not phase.group_summary.empty:
                row = phase.group_summary[phase.group_summary['controller_type'] == ct]
                if not row.empty and 'control_effort_iae' in row.columns:
                    efforts.append(row['control_effort_iae'].values[0])
                    pids.append(pid)

        if efforts:
            color = CONTROLLER_COLORS.get(ct, 'gray')
            ax4.plot(pids, efforts, 'o-', label=ct, color=color, linewidth=2, markersize=8)

    ax4.set_title("Control Effort Across Phases")
    ax4.set_xlabel("Phase")
    ax4.set_ylabel("Control Effort (IAE)")
    ax4.legend(fontsize=STYLE_CONFIG['legend_size'])
    ax4.grid(True, alpha=0.3)

    fig.suptitle("Cross-Phase Controller Comparison", fontsize=14, fontweight='bold')
    plt.tight_layout(rect=[0, 0, 1, 0.96])

    # Save
    if fmt in ['pdf', 'both']:
        fig.savefig(output_dir / "all_phases_comparison.pdf",
                    dpi=STYLE_CONFIG['dpi'], bbox_inches='tight')
    if fmt in ['png', 'both']:
        fig.savefig(output_dir / "all_phases_comparison.png",
                    dpi=STYLE_CONFIG['dpi'], bbox_inches='tight')

    plt.close(fig)
    print(f"  Saved: all_phases_comparison.{'pdf/png' if fmt == 'both' else fmt}")


def generate_plot_report_md(phases: Dict[int, PhaseData], output_dir: Path):
    """Aktif/Pasif grafik raporu oluştur."""
    report_path = output_dir.parent / f"PLOT_REPORT_{datetime.now().strftime('%Y-%m-%d')}.md"

    with open(report_path, 'w') as f:
        f.write("# Thesis Plot Report\n\n")
        f.write(f"**Generated:** {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")

        f.write("## Plot Configuration\n\n")
        f.write("| Plot | Status | Description |\n")
        f.write("|------|--------|-------------|\n")

        plot_descriptions = {
            "rmse_evolution": "RMSE time series with std dev shading",
            "xyz_error_decomposition": "Error breakdown by X/Y/Z axis",
            "ss_error_boxplot": "Steady-state error distribution",
            "min_inter_agent_distance": "Minimum inter-agent distance over time",
            "collision_risk_histogram": "Distribution of inter-agent distances",
            "control_effort": "Control effort (IAE) comparison",
            "jerk_analysis": "Jerk (smoothness) analysis",
            "trajectory_2d": "Top-view XY trajectory",
            "altitude_hold": "Altitude (Z) over time",
            "wind_profile": "Wind disturbance time series (X/Y/magnitude)",
            "wind_correlation": "Wind magnitude vs error correlation",
            "phase_comparison_heatmap": "Cross-phase RMSE heatmap",
            "controller_ranking": "Controller win count",
        }

        for plot_name, enabled in PLOT_ENABLED.items():
            status = "ACTIVE" if enabled else "DISABLED"
            desc = plot_descriptions.get(plot_name, "")
            f.write(f"| {plot_name} | {status} | {desc} |\n")

        f.write("\n## Phase Summary\n\n")

        for pid, phase in sorted(phases.items()):
            phase_name = PHASE_NAMES.get(pid, 'UNKNOWN')
            f.write(f"### Phase {pid}: {phase_name}\n\n")

            if not phase.group_summary.empty:
                f.write("| Controller | RMSE | SS-RMSE | MAE | Control Effort |\n")
                f.write("|------------|------|---------|-----|----------------|\n")

                for _, row in phase.group_summary.iterrows():
                    ct = row.get('controller_type', 'N/A')
                    rmse = row.get('mean_rmse', 0)
                    ss_rmse = row.get('ss_rmse', 0)
                    mae = row.get('mean_mae', 0)
                    effort = row.get('control_effort_iae', 0)
                    f.write(f"| {ct} | {rmse:.4f} | {ss_rmse:.4f} | {mae:.4f} | {effort:.2f} |\n")

                # Find winner
                best_idx = phase.group_summary['mean_rmse'].idxmin()
                winner = phase.group_summary.loc[best_idx, 'controller_type']
                f.write(f"\n**Winner:** {winner}\n\n")
            else:
                f.write("No summary data available.\n\n")

    print(f"  Saved: {report_path}")


# =============================================================================
# MAIN
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="Tez Seviyesinde Kapsamlı Raporlama Scripti",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Örnek Kullanım:
    python3 scripts/generate_comprehensive_plots.py \\
        --results-dir results/thesis_final \\
        --output-dir results/thesis_final/plots \\
        --format both
        """
    )

    parser.add_argument('--results-dir', '-r', type=str, default='results',
                        help='Simülasyon sonuçları dizini')
    parser.add_argument('--output-dir', '-o', type=str, default=None,
                        help='Grafik çıktı dizini (varsayılan: results-dir/plots)')
    parser.add_argument('--format', '-f', type=str, choices=['pdf', 'png', 'both'],
                        default='both', help='Çıktı formatı')
    parser.add_argument('--phase', '-p', type=int, default=None,
                        help='Sadece belirli bir faz için grafik üret')

    args = parser.parse_args()

    results_dir = Path(args.results_dir)
    output_dir = Path(args.output_dir) if args.output_dir else results_dir / 'plots'

    print("=" * 60)
    print("  TEZ SEVİYESİNDE KAPSAMLI RAPORLAMA SCRIPTİ")
    print("=" * 60)
    print(f"  Results: {results_dir}")
    print(f"  Output:  {output_dir}")
    print(f"  Format:  {args.format}")
    print("=" * 60)

    # Load data
    phases = load_all_phases(results_dir)

    if not phases:
        print("\nERROR: No phase data found!")
        print(f"Make sure {results_dir} contains phase_X/run_Y directories.")
        sys.exit(1)

    print(f"\nLoaded {len(phases)} phases: {list(phases.keys())}")

    # Generate reports
    if args.phase:
        if args.phase in phases:
            generate_phase_report(phases[args.phase], output_dir, args.format)
        else:
            print(f"ERROR: Phase {args.phase} not found!")
            sys.exit(1)
    else:
        for pid, phase in sorted(phases.items()):
            generate_phase_report(phase, output_dir, args.format)

        if len(phases) > 1:
            generate_cross_phase_comparison(phases, output_dir, args.format)

    # Generate MD report
    generate_plot_report_md(phases, output_dir)

    print("\n" + "=" * 60)
    print("  COMPLETED!")
    print("=" * 60)


if __name__ == "__main__":
    main()
