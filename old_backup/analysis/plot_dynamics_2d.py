import argparse
import glob
import os
from datetime import datetime

import matplotlib.pyplot as plt
import pandas as pd
import numpy as np


def find_latest_csv(pattern: str) -> str:
    files = glob.glob(pattern, recursive=True)
    if not files:
        raise FileNotFoundError(f"No CSV files matching pattern: {pattern}")
    latest = max(files, key=os.path.getmtime)
    return latest


def _step_metrics(sig, t, band=0.05):
    s = sig.to_numpy() if hasattr(sig, 'to_numpy') else np.asarray(sig)
    tt = t.to_numpy() if hasattr(t, 'to_numpy') else np.asarray(t)
    if len(s) < 3:
        return {'overshoot_pct': float('nan'), 'peak_time': float('nan'), 'settling_time': float('nan'), 'rise_time_10_90': float('nan')}
    y0 = float(s[0])
    # estimate final value as mean of last 10% samples
    tail = max(1, len(s) // 10)
    yfin = float(np.mean(s[-tail:]))
    A = yfin - y0
    if abs(A) < 1e-9:
        return {'overshoot_pct': float('nan'), 'peak_time': float('nan'), 'settling_time': float('nan'), 'rise_time_10_90': float('nan')}
    # Overshoot
    peak_idx = int(np.argmax(s)) if A >= 0 else int(np.argmin(s))
    peak_val = float(s[peak_idx])
    peak_time = float(tt[peak_idx])
    OS = max(0.0, (peak_val - yfin) / abs(A) if A >= 0 else (yfin - peak_val) / abs(A)) * 100.0
    # Rise 10–90%
    y10, y90 = y0 + 0.1 * A, y0 + 0.9 * A
    try:
        idx10 = np.where((s - y10) * np.sign(A) >= 0)[0][0]
        idx90 = np.where((s - y90) * np.sign(A) >= 0)[0][0]
        Tr = float(tt[idx90] - tt[idx10])
    except Exception:
        Tr = float('nan')
    # Settling time (±band)
    lo, hi = yfin - band * abs(A), yfin + band * abs(A)
    Ts = float('nan')
    for k in range(len(s) - 1, -1, -1):
        if s[k] < lo or s[k] > hi:
            Ts = float(tt[k + 1]) if k + 1 < len(s) else float('nan')
            break
    return {'overshoot_pct': OS, 'peak_time': peak_time, 'settling_time': Ts, 'rise_time_10_90': Tr}


def _prepare_dataframe(csv_path: str):
    """Load and prepare dataframe with numeric conversion and time ordering."""
    df = pd.read_csv(csv_path)
    if 'time' not in df.columns:
        raise ValueError('time column missing')
    for c in df.columns:
        try:
            df[c] = pd.to_numeric(df[c], errors='coerce')
        except Exception:
            pass
    df = df.dropna(subset=['time']).sort_values('time').reset_index(drop=True)
    return df


def _plot_position(ax, df, target_x, target_y):
    """Plot position data with targets."""
    ax.plot(df["time"], df["x"], label="x [m]")
    ax.plot(df["time"], df["y"], label="y [m]")
    if target_x is not None:
        ax.axhline(target_x, color="tab:blue", ls="--", alpha=0.5, label="x target")
    if target_y is not None:
        ax.axhline(target_y, color="tab:orange", ls="--", alpha=0.5, label="y target")
    ax.set_title("Position")
    ax.set_xlabel("t [s]")
    ax.set_ylabel("pos [m]")
    ax.grid(True)
    ax.legend()


def _plot_velocity(ax, df):
    """Plot velocity data."""
    ax.plot(df["time"], df["vx"], label="vx [m/s]")
    ax.plot(df["time"], df["vy"], label="vy [m/s]")
    ax.set_title("Velocity")
    ax.set_xlabel("t [s]")
    ax.set_ylabel("vel [m/s]")
    ax.grid(True)
    ax.legend()


def _plot_control(ax, df):
    """Plot control commands (raw vs filtered)."""
    if "ax_cmd_f" in df.columns:
        ax.plot(df["time"], df["ax_cmd_f"], label="ax_cmd_f [m/s^2]")
        ax.plot(df["time"], df["ay_cmd_f"], label="ay_cmd_f [m/s^2]")
    ax.plot(df["time"], df["ax_cmd"], ls=":", alpha=0.7, label="ax_cmd [m/s^2]")
    ax.plot(df["time"], df["ay_cmd"], ls=":", alpha=0.7, label="ay_cmd [m/s^2]")
    # Optional: hybrid contributions if present
    if {"ax_pid","ay_pid","ax_fuzzy","ay_fuzzy"}.issubset(set(df.columns)):
        ax.plot(df["time"], df["ax_pid"], color="tab:green", alpha=0.6, label="ax_pid [m/s^2]")
        ax.plot(df["time"], df["ay_pid"], color="tab:olive", alpha=0.6, label="ay_pid [m/s^2]")
        ax.plot(df["time"], df["ax_fuzzy"], color="tab:red", alpha=0.6, label="ax_fuzzy [m/s^2]")
        ax.plot(df["time"], df["ay_fuzzy"], color="tab:pink", alpha=0.6, label="ay_fuzzy [m/s^2]")
    ax.set_title("Control (raw vs filtered)")
    ax.set_xlabel("t [s]")
    ax.set_ylabel("a_cmd [m/s^2]")
    ax.grid(True)
    ax.legend()


def _plot_drag(ax, df):
    """Plot drag and relative airspeed."""
    if "ax_drag" in df.columns and "ay_drag" in df.columns:
        a_drag_norm = np.hypot(df["ax_drag"].to_numpy(), df["ay_drag"].to_numpy())
        ax.plot(df["time"], a_drag_norm, label="|a_drag| [m/s^2]")
    if "vrel_norm" in df.columns:
        ax2 = ax.twinx()
        ax2.plot(df["time"], df["vrel_norm"], color="gray", alpha=0.6, label="|v_rel| [m/s]")
        ax2.set_ylabel("|v_rel| [m/s]")
    ax.set_title("Drag & |v_rel|")
    ax.set_xlabel("t [s]")
    ax.set_ylabel("drag [m/s^2]")
    ax.grid(True)


def _plot_error_x(ax, df, target_x):
    """Plot X error with bands and metrics."""
    if target_x is None:
        return

    err_x = df["x"] - target_x
    ax.plot(df["time"], err_x, label="error x [m]")

    # bands for 2% and 5% around the final estimate
    s = df["x"].to_numpy()
    tail = max(1, len(s) // 10)
    yfin = float(np.mean(s[-tail:]))
    A = yfin - float(s[0])
    band2 = 0.02 * abs(A)
    band5 = 0.05 * abs(A)
    ax.axhline(yfin + band2, color="gray", ls=":", alpha=0.6)
    ax.axhline(yfin - band2, color="gray", ls=":", alpha=0.6, label="2% band")
    ax.axhline(yfin + band5, color="silver", ls="-.", alpha=0.6)
    ax.axhline(yfin - band5, color="silver", ls="-.", alpha=0.6, label="5% band")

    m = _step_metrics(df["x"], df["time"])
    # summary metrics
    dt = float(np.median(np.diff(df["time"].to_numpy())))
    rmse_x = float(np.sqrt(np.mean((df["x"].to_numpy() - target_x) ** 2)))
    iae_x = float(np.sum(np.abs(df["x"].to_numpy() - target_x)) * dt)
    itae_x = float(np.sum(df["time"].to_numpy() * np.abs(df["x"].to_numpy() - target_x)) * dt)

    if "ax_cmd_f" in df.columns and "ay_cmd_f" in df.columns:
        u_norm = float(np.sum(np.hypot(df["ax_cmd_f"].to_numpy(), df["ay_cmd_f"].to_numpy())) * dt)
    else:
        u_norm = float('nan')

    txt = (
        f"Overshoot: {m['overshoot_pct']:.1f}%\n"
        f"Peak time: {m['peak_time']:.2f}s\n"
        f"Settling(±5%): {m['settling_time']:.2f}s\n"
        f"Rise 10-90%: {m['rise_time_10_90']:.2f}s\n"
        f"RMSE_x: {rmse_x:.3f}, IAE_x: {iae_x:.3f}\nITAE_x: {itae_x:.3f}, |u|_1: {u_norm:.3f}"
    )
    ax.text(0.02, 0.98, txt, transform=ax.transAxes, va="top", ha="left",
            bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.8})
    ax.set_title("Step Error X & Metrics")
    ax.set_xlabel("t [s]")
    ax.set_ylabel("error x [m]")
    ax.grid(True)


def _plot_error_y(ax, df, target_y):
    """Plot Y error with bands."""
    if target_y is not None:
        err_y = df["y"] - target_y
        ax.plot(df["time"], err_y, label="error y [m]")
        step_amp_y = abs(target_y - df["y"].iloc[0])
        ax.axhline(0.02 * step_amp_y, color="gray", ls=":", alpha=0.6)
        ax.axhline(-0.02 * step_amp_y, color="gray", ls=":", alpha=0.6, label="2% band")
        ax.axhline(0.05 * step_amp_y, color="silver", ls="-.", alpha=0.6)
        ax.axhline(-0.05 * step_amp_y, color="silver", ls="-.", alpha=0.6, label="5% band")
    ax.set_title("Step Error Y")
    ax.set_xlabel("t [s]")
    ax.set_ylabel("error y [m]")
    ax.grid(True)


def plot_csv(csv_path: str, out_path: str | None = None, t_end: float | None = None):
    """Main plotting function."""
    df = _prepare_dataframe(csv_path)
    if t_end is not None:
        try:
            t_end_val = float(t_end)
            df = df[df["time"] <= t_end_val].reset_index(drop=True)
        except Exception:
            pass
    target_x = df["target_x"].iloc[0] if "target_x" in df.columns else None
    target_y = df["target_y"].iloc[0] if "target_y" in df.columns else None

    fig, axes = plt.subplots(3, 2, figsize=(12, 10), constrained_layout=True)

    _plot_position(axes[0, 0], df, target_x, target_y)
    _plot_velocity(axes[0, 1], df)
    _plot_control(axes[1, 0], df)
    _plot_drag(axes[1, 1], df)
    _plot_error_x(axes[2, 0], df, target_x)
    _plot_error_y(axes[2, 1], df, target_y)

    # Meta text
    meta_lines = []
    for k in ["kp", "ki", "kd", "vx_wind", "vy_wind", "cd_lin", "cd_quad", "v_thr", "tau_up", "tau_down", "a_max", "dt"]:
        if k in df.columns:
            meta_lines.append(f"{k}={df[k].iloc[0]}")
    meta_text = "  ".join(meta_lines)
    fig.suptitle(f"Dynamics 2D Test\n{os.path.basename(csv_path)}\n{meta_text}", fontsize=10)

    if out_path is None:
        base_dir = os.path.dirname(csv_path)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        out_path = os.path.join(base_dir, f"plot_{stamp}.png")

    fig.savefig(out_path, dpi=140)
    print(f"Saved plot: {out_path}")


def main():
    parser = argparse.ArgumentParser(description="Plot 2D dynamics CSV output")
    parser.add_argument("--csv", type=str, default=None, help="Path to CSV file. If omitted, picks latest in outputs/simulations/dynamics2d")
    parser.add_argument("--out", type=str, default=None, help="Output PNG path")
    parser.add_argument("--t_end", type=float, default=None, help="Optional time window end [s] (e.g., 20.0)")
    args = parser.parse_args()

    patterns = [
        os.path.join("outputs", "simulations", "dynamics2d", "**", "*.csv"),
        os.path.join("outputs", "simulations", "dynamics2d", "*.csv"),
        os.path.join("simulation_outputs", "dynamics_2d_test_*.csv"),
    ]

    csv_path = args.csv
    if not csv_path:
        last = None
        for pat in patterns:
            try:
                last = find_latest_csv(pat)
                break
            except FileNotFoundError:
                continue
        if not last:
            raise FileNotFoundError("No CSV found in outputs/simulations/dynamics2d or legacy simulation_outputs")
        csv_path = last

    plot_csv(csv_path, args.out, args.t_end)


if __name__ == "__main__":
    main()
