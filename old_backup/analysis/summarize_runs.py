import argparse
import glob
import os
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np
import pandas as pd


def metrics_from_csv(csv_path: Path) -> Dict[str, float]:
    df = pd.read_csv(csv_path)
    df = df.dropna(subset=["time"]).sort_values("time").reset_index(drop=True)
    t = df["time"].to_numpy()
    x = df["x"].to_numpy()
    tx = float(df.get("target_x", pd.Series([x[-1]])).iloc[0])
    A = tx - float(x[0])
    tail = max(1, len(x) // 10)
    yfin = float(np.mean(x[-tail:]))

    if abs(yfin - x[0]) > 1e-12:
        peak_idx = int(np.argmax(x)) if A >= 0 else int(np.argmin(x))
        peak_val = float(x[peak_idx])
        tp = float(t[peak_idx])
        OS = max(0.0, (peak_val - yfin) / abs(yfin - x[0]) if A >= 0 else (yfin - peak_val) / abs(yfin - x[0])) * 100.0
        lo, hi = yfin - 0.05 * abs(yfin - x[0]), yfin + 0.05 * abs(yfin - x[0])
        Ts5 = float("nan")
        for k in range(len(x) - 1, -1, -1):
            if x[k] < lo or x[k] > hi:
                Ts5 = float(t[k + 1]) if k + 1 < len(x) else float("nan")
                break
    else:
        OS = float("nan"); tp = float("nan"); Ts5 = float("nan")

    dtm = float(np.median(np.diff(t))) if len(t) > 1 else 0.0
    rmse = float(np.sqrt(np.mean((x - tx) ** 2)))
    iae = float(np.sum(np.abs(x - tx)) * dtm)
    return dict(OS=OS, tp=tp, Ts5=Ts5, RMSE=rmse, IAE=iae)


def parse_label_from_path(p: Path) -> Tuple[str, str, int]:
    day = p.parents[1].name  # YYYYMMDD
    run_dir = p.parent.name  # run_###__label or run_###
    label = run_dir
    run_idx = -1
    if run_dir.startswith("run_"):
        rest = run_dir[4:]
        digits = []
        for ch in rest:
            if ch.isdigit():
                digits.append(ch)
            else:
                break
        if digits:
            run_idx = int("".join(digits))
        if "__" in run_dir:
            label = run_dir.split("__", 1)[1]
    return day, label, run_idx


def main():
    ap = argparse.ArgumentParser(description="Summarize Dynamics2D runs and rank the best ones")
    ap.add_argument("--root", default=str(Path("outputs/simulations/dynamics2d")))
    ap.add_argument("--day", default="", help="YYYYMMDD; empty means all days")
    ap.add_argument("--contains", default="", help="Filter labels containing this substring")
    ap.add_argument("--os-max", type=float, default=20.0)
    ap.add_argument("--ts-max", type=float, default=8.0)
    ap.add_argument("--top", type=int, default=10)
    args = ap.parse_args()

    root = Path(args.root)
    if args.day:
        pattern = str(root / args.day / "run_*" / "run_*.csv")
    else:
        pattern = str(root / "*" / "run_*" / "run_*.csv")

    files = [Path(p) for p in glob.glob(pattern)]
    rows: List[Dict] = []
    for f in files:
        day, label, run_idx = parse_label_from_path(f)
        if args.contains and args.contains not in label:
            continue
        try:
            mets = metrics_from_csv(f)
        except Exception:
            continue
        rows.append({
            "day": day,
            "run_idx": run_idx,
            "label": label,
            "path": str(f),
            **mets,
        })

    if not rows:
        print("No runs found matching criteria.")
        return

    df = pd.DataFrame(rows)
    # Ranking: meet constraints first, then Ts5, IAE, OS
    ok = (df["OS"] <= args.os_max) & (df["Ts5"].replace([np.inf, -np.inf], np.nan).notna()) & (df["Ts5"] <= args.ts_max)
    df["rank_key"] = list(zip(~ok, df["Ts5"].fillna(1e9), df["IAE"], df["OS"]))
    df_sorted = df.sort_values("rank_key").drop(columns=["rank_key"]) 

    day = args.day if args.day else "all"
    out_csv = root / f"{day}_summary_all.csv"
    df_sorted.to_csv(out_csv, index=False)
    print(f"Wrote summary: {out_csv}")

    top_df = df_sorted.head(args.top)
    print("\nTop candidates:")
    for _, r in top_df.iterrows():
        print(f"{r['label']} (OS={r['OS']:.1f}% Ts5={r['Ts5']:.2f}s IAE={r['IAE']:.3f}) -> {r['path']}")


if __name__ == "__main__":
    main()

