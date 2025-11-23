import argparse
import glob
import os
from datetime import datetime
from pathlib import Path
from typing import List, Tuple

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def load_csv(csv_path: str) -> pd.DataFrame:
    df = pd.read_csv(csv_path)
    for c in df.columns:
        try:
            df[c] = pd.to_numeric(df[c], errors='coerce')
        except Exception:
            pass
    df = df.dropna(subset=['time']).sort_values('time').reset_index(drop=True)
    return df


def pick_files(patterns: List[str]) -> List[str]:
    files: List[str] = []
    for p in patterns:
        if os.path.isdir(p):
            files.extend(glob.glob(os.path.join(p, "**", "*.csv"), recursive=True))
        else:
            files.extend(glob.glob(p))
    files = sorted(set(files), key=lambda s: (os.path.basename(os.path.dirname(s)), os.path.basename(s)))
    if not files:
        raise FileNotFoundError("No CSVs found for given patterns")
    return files


def plot_compare(csv_files: List[str], labels: List[str] | None, t_end: float | None, zoom: float | None, out_path: str | None):
    if labels and len(labels) != len(csv_files):
        raise ValueError("labels length must match csv_files length")
    series = []
    for i, f in enumerate(csv_files):
        df = load_csv(f)
        if t_end is not None:
            df = df[df['time'] <= t_end].reset_index(drop=True)
        tx = df['target_x'].iloc[0] if 'target_x' in df.columns else np.nan
        ty = df['target_y'].iloc[0] if 'target_y' in df.columns else np.nan
        lab = labels[i] if labels else os.path.basename(os.path.dirname(f))
        series.append((lab, df, tx, ty))

    fig, axes = plt.subplots(2, 2, figsize=(12, 9), constrained_layout=True)
    axx, axy, axerr, axxy = axes[0,0], axes[0,1], axes[1,0], axes[1,1]

    # X and Y positions vs targets
    for lab, df, tx, ty in series:
        axx.plot(df['time'], df['x'], label=f"{lab} x")
        axy.plot(df['time'], df['y'], label=f"{lab} y")
    # targets
    if not np.isnan(series[0][2]):
        axx.axhline(series[0][2], color='k', ls='--', alpha=0.4, label='x target')
    if not np.isnan(series[0][3]):
        axy.axhline(series[0][3], color='k', ls='--', alpha=0.4, label='y target')
    axx.set_title('Position X'); axx.set_xlabel('t [s]'); axx.set_ylabel('x [m]'); axx.grid(True); axx.legend(ncol=2)
    axy.set_title('Position Y'); axy.set_xlabel('t [s]'); axy.set_ylabel('y [m]'); axy.grid(True); axy.legend(ncol=2)

    # Error norm
    for lab, df, tx, ty in series:
        if np.isnan(tx) or np.isnan(ty):
            continue
        en = np.hypot(df['x']-tx, df['y']-ty)
        axerr.plot(df['time'], en, label=f"{lab} |e|")
    axerr.set_title('Error Norm'); axerr.set_xlabel('t [s]'); axerr.set_ylabel('|e| [m]'); axerr.grid(True); axerr.legend(ncol=2)

    # XY trajectory
    for lab, df, tx, ty in series:
        axxy.plot(df['x'], df['y'], label=lab)
    if not np.isnan(series[0][2]) and not np.isnan(series[0][3]):
        axxy.scatter([series[0][2]], [series[0][3]], c='k', marker='x', label='target')
    axxy.set_title('XY Trajectory'); axxy.set_xlabel('x [m]'); axxy.set_ylabel('y [m]'); axxy.axis('equal'); axxy.grid(True); axxy.legend(ncol=2)

    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    if out_path is None:
        out_path = f"compare_positions_{stamp}.png"
    fig.suptitle(f"Compare Positions (t_end={t_end if t_end else 'full'})")
    fig.savefig(out_path, dpi=140)
    print(f"Saved compare plot: {out_path}")

    if zoom is not None:
        # zoom figure
        fig2, axes2 = plt.subplots(1, 2, figsize=(12, 4), constrained_layout=True)
        axz1, axz2 = axes2
        for lab, df, tx, ty in series:
            dff = df[df['time'] <= zoom]
            axz1.plot(dff['time'], dff['x'], label=f"{lab} x")
            axz2.plot(dff['time'], np.hypot(dff['x']-tx, dff['y']-ty), label=f"{lab} |e|")
        if not np.isnan(series[0][2]):
            axz1.axhline(series[0][2], color='k', ls='--', alpha=0.4, label='x target')
        axz1.set_title(f'X (0-{zoom}s)'); axz1.set_xlabel('t [s]'); axz1.set_ylabel('x [m]'); axz1.grid(True); axz1.legend(ncol=2)
        axz2.set_title(f'|e| (0-{zoom}s)'); axz2.set_xlabel('t [s]'); axz2.set_ylabel('|e| [m]'); axz2.grid(True); axz2.legend(ncol=2)
        out2 = f"compare_positions_zoom_{zoom:.0f}s_{stamp}.png"
        fig2.savefig(out2, dpi=140)
        print(f"Saved zoom plot: {out2}")


def main():
    ap = argparse.ArgumentParser(description='Compare multiple CSV runs (position-focused)')
    ap.add_argument('--csv', nargs='+', help='CSV paths or glob patterns or run directories')
    ap.add_argument('--day', default='', help='If given, picks all CSVs under this day folder')
    ap.add_argument('--contains', default='', help='Filter run labels containing this substring')
    ap.add_argument('--labels', nargs='+', default=None, help='Custom labels (same length as files)')
    ap.add_argument('--t_end', type=float, default=None, help='End time [s] (e.g., 20)')
    ap.add_argument('--zoom', type=float, default=20.0, help='Zoom window [s] for mini-scale figure')
    ap.add_argument('--out', type=str, default=None, help='Output PNG path for main figure')
    args = ap.parse_args()

    files: List[str] = []
    if args.day:
        root = Path('outputs/simulations/dynamics2d') / args.day
        files = glob.glob(str(root / 'run_*' / 'run_*.csv'))
        if args.contains:
            files = [f for f in files if args.contains in os.path.basename(os.path.dirname(f))]
    elif args.csv:
        files = pick_files(args.csv)
    else:
        raise SystemExit('Provide --day or --csv patterns')

    if not files:
        raise SystemExit('No CSV files matched')

    plot_compare(files, args.labels, args.t_end, args.zoom, args.out)


if __name__ == '__main__':
    main()

