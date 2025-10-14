import argparse
import csv
import datetime as dt
import os
import re
import shutil
import subprocess
from pathlib import Path
from typing import List, Tuple

import numpy as np
import pandas as pd
import yaml


# --- Shared formulas (kept here to avoid import hassles) ---
def zeta_from_overshoot(Mp: float) -> float:
    Mp = max(1e-6, min(0.999, Mp))
    lnMp = np.log(Mp)
    return float(-lnMp / np.sqrt(np.pi**2 + lnMp**2))


def wn_from_Ts5(zeta: float, Ts5: float) -> float:
    return float(3.0 / max(1e-9, (zeta * Ts5)))


def pid_map_with_actuator(zeta: float, wn: float, tau: float, beta: float) -> Tuple[float, float, float]:
    inv_tau = 1.0 / max(1e-9, tau)
    p_i = beta * wn
    p_a = inv_tau - 2.0 * zeta * wn - p_i
    if p_a <= 0:
        p_a = 0.1 * inv_tau
    Kd = tau * (wn ** 2 + 2.0 * zeta * wn * (p_a + p_i) + p_a * p_i)
    Kp = tau * (wn ** 2 * (p_a + p_i) + 2.0 * zeta * wn * p_a * p_i)
    Ki = tau * (wn ** 2 * p_a * p_i)
    return float(Kp), float(Ki), float(Kd)


def evaluate_csv(csv_path: Path):
    df = pd.read_csv(csv_path)
    df = df.dropna(subset=["time"]).sort_values("time").reset_index(drop=True)
    t = df["time"].to_numpy()
    x = df["x"].to_numpy()
    tx = float(df.get("target_x", pd.Series([x[-1]])).iloc[0])
    # Step size
    A = tx - float(x[0])
    # Overshoot, peak time, Ts(5%)
    tail = max(1, len(x)//10)
    yfin = float(np.mean(x[-tail:]))
    if abs(yfin - x[0]) > 1e-12:
        peak_idx = int(np.argmax(x)) if A >= 0 else int(np.argmin(x))
        peak_val = float(x[peak_idx])
        peak_time = float(t[peak_idx])
        OS = max(0.0, (peak_val - yfin)/abs(yfin - x[0]) if A >= 0 else (yfin - peak_val)/abs(yfin - x[0])) * 100.0
        lo, hi = yfin - 0.05*abs(yfin - x[0]), yfin + 0.05*abs(yfin - x[0])
        Ts = float('nan')
        for k in range(len(x)-1, -1, -1):
            if x[k] < lo or x[k] > hi:
                Ts = float(t[k+1]) if k+1 < len(x) else float('nan'); break
    else:
        OS = float('nan'); peak_time = float('nan'); Ts = float('nan')
    # Error metrics
    dtm = float(np.median(np.diff(t))) if len(t) > 1 else 0.0
    rmse = float(np.sqrt(np.mean((x - tx)**2)))
    iae = float(np.sum(np.abs(x - tx)) * dtm)
    # Oscillation: count prominent peaks above 2% band
    e = x - tx
    thr = 0.02 * abs(A) if abs(A) > 1e-9 else 0.01
    peaks = 0
    for i in range(1, len(e)-1):
        if e[i] > e[i-1] and e[i] > e[i+1] and abs(e[i]) > thr:
            peaks += 1
    return dict(OS=OS, tp=peak_time, Ts5=Ts, RMSE=rmse, IAE=iae, peaks=peaks)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--base-config", required=True)
    ap.add_argument("--mp", default="0.08,0.10")
    ap.add_argument("--ts5", default="10,12")
    ap.add_argument("--beta", default="3.0,3.5")
    ap.add_argument("--tau-modes", default="avg,down")
    ap.add_argument("--exe", default=str(Path("build/Release/dynamics_2d_tester.exe")))
    ap.add_argument("--out-dir", default="agent_control_pkg/config/experiments/auto")
    args = ap.parse_args()

    base = Path(args.base_config)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    cfg = yaml.safe_load(base.read_text())
    tau_up = cfg.get("physics", {}).get("tau_up", 0.1)
    tau_down = cfg.get("physics", {}).get("tau_down", 0.1)

    Mp_list = [float(s) for s in args.mp.split(",")]
    Ts_list = [float(s) for s in args.ts5.split(",")]
    B_list = [float(s) for s in args.beta.split(",")]
    tau_modes = [s.strip() for s in args.tau_modes.split(",")]

    combos = []
    for Mp in Mp_list:
        for Ts5 in Ts_list:
            for B in B_list:
                for tm in tau_modes:
                    combos.append((Mp, Ts5, B, tm))

    results = []
    exe_path = Path(args.exe)
    wrote_re = re.compile(r"Wrote:\s+(.*\.csv)")

    for idx, (Mp, Ts5, B, tm) in enumerate(combos, start=1):
        tau = {"avg": 0.5*(tau_up + tau_down), "up": tau_up, "down": tau_down}[tm]
        zeta = zeta_from_overshoot(Mp)
        wn = wn_from_Ts5(zeta, Ts5)
        kp, ki, kd = pid_map_with_actuator(zeta, wn, tau, B)

        cfg_i = yaml.safe_load(base.read_text())
        cfg_i.setdefault("controller_settings", {}).setdefault("pid", {})
        cfg_i["controller_settings"]["pid"].update({"kp": float(kp), "ki": float(ki), "kd": float(kd)})

        yml_path = out_dir / f"pid_auto_{idx:03d}.yaml"
        yml_path.write_text(yaml.safe_dump(cfg_i, sort_keys=False))

        cp = subprocess.run([str(exe_path), str(yml_path)], capture_output=True, text=True)
        out = (cp.stdout or "") + "\n" + (cp.stderr or "")
        m = wrote_re.search(out)
        if not m:
            print(f"[WARN] Could not parse output CSV path for {yml_path}. Skipping.\n{out}")
            continue
        csv_path = Path(m.group(1).strip())
        mets = evaluate_csv(csv_path)
        results.append({
            "idx": idx, "csv": str(csv_path), "yaml": str(yml_path),
            "Mp": Mp, "Ts5_spec": Ts5, "beta": B, "tau_mode": tm,
            **mets,
        })
        print(f"[{idx}/{len(combos)}] {tm} Mp={Mp} Ts5={Ts5} B={B} -> OS={mets['OS']:.1f}% Ts5={mets['Ts5']:.2f}s IAE={mets['IAE']:.3f} peaks={mets['peaks']}")

    if not results:
        print("No successful runs.")
        return

    # Lexicographic selection: minimize Ts5 (valid), then IAE, then OS, then peaks
    def keyfun(r):
        ts = r["Ts5"] if np.isfinite(r["Ts5"]) else 1e9
        return (ts, r["IAE"], r["OS"], r["peaks"])

    results.sort(key=keyfun)
    best = results[0]

    # Write summary CSV next to latest day folder
    day = dt.datetime.now().strftime("%Y%m%d")
    out_summary = Path("outputs/simulations/dynamics2d") / day / "auto_tune_summary.csv"
    out_summary.parent.mkdir(parents=True, exist_ok=True)
    with out_summary.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(results[0].keys()))
        w.writeheader(); w.writerows(results)

    # Copy best yaml to a stable path
    best_yaml = Path(best["yaml"])
    target_yaml = Path("agent_control_pkg/config/experiments/pid_nowind_best.yaml")
    shutil.copyfile(best_yaml, target_yaml)

    print("\nBest candidate:")
    print(best)
    print(f"Summary: {out_summary}")
    print(f"Best YAML copied to: {target_yaml}")


if __name__ == "__main__":
    main()

