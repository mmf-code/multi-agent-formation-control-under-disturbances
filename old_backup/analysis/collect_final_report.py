import argparse
import glob
import os
import shutil
from pathlib import Path
from typing import Dict, List

import numpy as np
import pandas as pd


def metrics_from_csv(csv_path: Path) -> Dict[str, float]:
    df = pd.read_csv(csv_path)
    df = df.dropna(subset=["time"]).sort_values("time").reset_index(drop=True)
    t = df["time"].to_numpy()
    x = df["x"].to_numpy()
    tx = float(df.get("target_x", pd.Series([x[-1]])).iloc[0])
    A = tx - float(x[0])
    tail = max(1, len(x)//10)
    yfin = float(np.mean(x[-tail:]))
    if abs(yfin - x[0]) > 1e-12:
        peak_idx = int(np.argmax(x)) if A >= 0 else int(np.argmin(x))
        peak_val = float(x[peak_idx])
        tp = float(t[peak_idx])
        OS = max(0.0, (peak_val - yfin)/abs(yfin - x[0]) if A>=0 else (yfin - peak_val)/abs(yfin - x[0])) * 100.0
        lo, hi = yfin - 0.05*abs(yfin - x[0]), yfin + 0.05*abs(yfin - x[0])
        Ts = float('nan')
        for k in range(len(x)-1, -1, -1):
            if x[k] < lo or x[k] > hi:
                Ts = float(t[k+1]) if k+1 < len(x) else float('nan')
                break
    else:
        OS = float('nan'); tp = float('nan'); Ts = float('nan')
    dt = float(np.median(np.diff(t))) if len(t)>1 else 0.0
    rmse = float(np.sqrt(np.mean((x - tx)**2)))
    iae = float(np.sum(np.abs(x - tx)) * dt)
    return dict(OS=OS, tp=tp, Ts5=Ts, RMSE=rmse, IAE=iae)


def main():
    ap = argparse.ArgumentParser(description="Collect top runs into a final report folder")
    ap.add_argument("--day", required=True, help="YYYYMMDD")
    ap.add_argument("--labels", default="pd_vel,pid_vel,pidf_", help="Comma-separated substrings to filter labels")
    ap.add_argument("--root", default=str(Path("outputs/simulations/dynamics2d")))
    ap.add_argument("--out", default="outputs/final_report")
    ap.add_argument("--top", type=int, default=1, help="How many per label to copy")
    args = ap.parse_args()

    root = Path(args.root)
    labels = [s.strip() for s in args.labels.split(",") if s.strip()]
    pattern = str(root / args.day / "run_*" / "run_*.csv")
    files = [Path(p) for p in glob.glob(pattern)]

    rows: List[Dict] = []
    for f in files:
        run_dir = f.parent.name
        label = run_dir
        if run_dir.startswith("run_") and "__" in run_dir:
            label = run_dir.split("__", 1)[1]
        if labels and not any(sub in label for sub in labels):
            continue
        try:
            mets = metrics_from_csv(f)
        except Exception:
            continue
        rows.append({"label": label, "csv": str(f), "png": str(f.with_name(f.stem.replace('.csv','') + ".png")), **mets})

    if not rows:
        print("No runs matched.")
        return

    df = pd.DataFrame(rows)
    # rank within each label: Ts5 asc, IAE asc, OS asc
    collected = []
    for lab, g in df.groupby("label"):
        gg = g.sort_values(by=["Ts5","IAE","OS"], na_position="last").head(args.top)
        collected.append(gg)
    cat = pd.concat(collected, ignore_index=True)

    out_dir = Path(args.out) / args.day
    out_dir.mkdir(parents=True, exist_ok=True)
    summary_csv = out_dir / "final_summary.csv"
    cat.to_csv(summary_csv, index=False)

    # copy assets
    for _, r in cat.iterrows():
        src_csv = Path(r["csv"])
        src_png = Path(r["png"]) if Path(r["png"]).exists() else None
        dst_dir = out_dir / r["label"]
        dst_dir.mkdir(exist_ok=True)
        shutil.copy2(src_csv, dst_dir / src_csv.name)
        if src_png is not None and src_png.exists():
            shutil.copy2(src_png, dst_dir / src_png.name)

    print(f"Collected {len(cat)} runs into {out_dir}")


if __name__ == "__main__":
    main()

