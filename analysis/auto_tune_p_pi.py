import argparse
import csv
import os
import re
import subprocess
from pathlib import Path

import numpy as np
import pandas as pd
import yaml


def eval_csv(csv_path: Path):
    df = pd.read_csv(csv_path)
    df = df.dropna(subset=["time"]).sort_values("time").reset_index(drop=True)
    t = df["time"].to_numpy(); x = df["x"].to_numpy()
    tx = float(df.get("target_x", pd.Series([x[-1]])).iloc[0])
    A = tx - float(x[0])
    tail = max(1, len(x)//10)
    yfin = float(np.mean(x[-tail:]))
    if abs(yfin - x[0]) > 1e-12:
        peak_idx = int(np.argmax(x)) if A>=0 else int(np.argmin(x))
        peak_val = float(x[peak_idx]); tp = float(t[peak_idx])
        OS = max(0.0, (peak_val - yfin)/abs(yfin - x[0]) if A>=0 else (yfin - peak_val)/abs(yfin - x[0])) * 100.0
        lo, hi = yfin - 0.05*abs(yfin - x[0]), yfin + 0.05*abs(yfin - x[0])
        Ts = float('nan')
        for k in range(len(x)-1, -1, -1):
            if x[k] < lo or x[k] > hi:
                Ts = float(t[k+1]) if k+1 < len(x) else float('nan'); break
    else:
        OS = float('nan'); tp = float('nan'); Ts = float('nan')
    dt = float(np.median(np.diff(t))) if len(t)>1 else 0.0
    rmse = float(np.sqrt(np.mean((x - tx)**2)))
    iae = float(np.sum(np.abs(x - tx)) * dt)
    return dict(OS=OS, tp=tp, Ts5=Ts, RMSE=rmse, IAE=iae)


def run_case(exe: Path, base_cfg: Path, ctrl_type: str, kp: float, ki: float = 0.0, label: str = ""):
    cfg = yaml.safe_load(base_cfg.read_text())
    cfg.setdefault("controller_settings", {}).setdefault("pid", {})
    cfg["controller_settings"]["pid"]["kp"] = float(kp)
    cfg["controller_settings"]["pid"]["ki"] = float(ki)
    # Keep kd from base cfg but PI/P set it to 0 when building
    cfg["controller_settings"]["type"] = ctrl_type
    # Ensure autoplot off for speed
    out_settings = cfg.setdefault("output_settings", {})
    if label:
        out_settings["run_label"] = label
    out_settings["auto_plot"] = False
    tmp_yaml = base_cfg.parent / f"auto_{ctrl_type}_kp{kp:.3f}_ki{ki:.3f}.yaml"
    tmp_yaml.write_text(yaml.safe_dump(cfg, sort_keys=False))
    cp = subprocess.run([str(exe), str(tmp_yaml)], capture_output=True, text=True)
    out_text = (cp.stdout or "") + "\n" + (cp.stderr or "")
    m = re.search(r"Wrote:\s+(.*\.csv)", out_text)
    if not m:
        return None, None
    csv_path = Path(m.group(1).strip())
    mets = eval_csv(csv_path)
    return csv_path, mets


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--base-config", required=True, help="Use physics/target from here (e.g., pid_vel_2p5.yaml)")
    ap.add_argument("--exe", default=str(Path("build/Release/dynamics_2d_tester.exe")))
    ap.add_argument("--os-max", type=float, default=15.0)
    ap.add_argument("--ts-max", type=float, default=6.0)
    ap.add_argument("--grid-p", default="0.05,0.1,0.2,0.3,0.4,0.6,0.8,1.0,1.2,1.5,2.0")
    ap.add_argument("--grid-pi-kp", default="0.2,0.3,0.4,0.6,0.8,1.0,1.2,1.5,2.0,2.5,3.0")
    ap.add_argument("--grid-pi-ki", default="0.02,0.05,0.08,0.12,0.16,0.2,0.3,0.4,0.6,0.8,1.0")
    args = ap.parse_args()

    base_cfg = Path(args.base_config)
    exe = Path(args.exe)
    os_max = float(args.os_max)
    ts_max = float(args.ts_max)

    # Search P
    p_vals = [float(x) for x in args.grid_p.split(",")]
    best_p = None
    cand_p = []
    for kp in p_vals:
        csvp, mp = run_case(exe, base_cfg, "p", kp, 0.0, label=f"p_tune")
        if mp is None: continue
        cand_p.append((kp, mp))
    # Select: prioritize constraints, then Ts5, then IAE, then OS
    def score(m):
        ts = m["Ts5"] if np.isfinite(m["Ts5"]) else 1e9
        return ( (m["OS"] <= os_max) and (ts <= ts_max), ts, m["IAE"], m["OS"])  # True>False
    if cand_p:
        cand_p.sort(key=lambda t: (not (t[1]["OS"] <= os_max and (np.isfinite(t[1]["Ts5"]) and t[1]["Ts5"] <= ts_max)),
                                   t[1]["Ts5"] if np.isfinite(t[1]["Ts5"]) else 1e9,
                                   t[1]["IAE"], t[1]["OS"]))
        best_p = cand_p[0]

    # Search PI
    kp_vals = [float(x) for x in args.grid_pi_kp.split(",")]
    ki_vals = [float(x) for x in args.grid_pi_ki.split(",")]
    best_pi = None
    cand_pi = []
    for kp in kp_vals:
        for ki in ki_vals:
            csvi, mi = run_case(exe, base_cfg, "pi", kp, ki, label=f"pi_tune")
            if mi is None: continue
            cand_pi.append((kp, ki, mi))
    if cand_pi:
        cand_pi.sort(key=lambda t: (not (t[2]["OS"] <= os_max and (np.isfinite(t[2]["Ts5"]) and t[2]["Ts5"] <= ts_max)),
                                    t[2]["Ts5"] if np.isfinite(t[2]["Ts5"]) else 1e9,
                                    t[2]["IAE"], t[2]["OS"]))
        best_pi = cand_pi[0]

    # Print summary
    print("\nP candidates (top 5):")
    for kp, m in cand_p[:5]:
        print(f"kp={kp:.3f} -> OS={m['OS']:.1f}% Ts5={m['Ts5']:.2f}s IAE={m['IAE']:.3f}")
    if best_p:
        kp, mp = best_p
        print(f"Best P: kp={kp:.3f} -> OS={mp['OS']:.1f}% Ts5={mp['Ts5']:.2f}s IAE={mp['IAE']:.3f}")

    print("\nPI candidates (top 5):")
    for kp, ki, m in cand_pi[:5]:
        print(f"kp={kp:.3f} ki={ki:.3f} -> OS={m['OS']:.1f}% Ts5={m['Ts5']:.2f}s IAE={m['IAE']:.3f}")
    if best_pi:
        kp, ki, mi = best_pi
        print(f"Best PI: kp={kp:.3f} ki={ki:.3f} -> OS={mi['OS']:.1f}% Ts5={mi['Ts5']:.2f}s IAE={mi['IAE']:.3f}")

    # Write tuned YAMLs with labels + autoplot
    if best_p:
        kp, _ = best_p
        cfgp = yaml.safe_load(base_cfg.read_text())
        cfgp["controller_settings"]["type"] = "p"
        cfgp["controller_settings"]["pid"]["kp"] = float(kp)
        cfgp["controller_settings"]["pid"]["ki"] = 0.0
        outp = base_cfg.parent / "p_vel_tuned.yaml"
        cfgp.setdefault("output_settings", {})
        cfgp["output_settings"]["run_label"] = "p_vel_tuned"
        cfgp["output_settings"]["auto_plot"] = True
        outp.write_text(yaml.safe_dump(cfgp, sort_keys=False))
        print(f"Wrote {outp}")
    if best_pi:
        kp, ki, _ = best_pi
        cfgi = yaml.safe_load(base_cfg.read_text())
        cfgi["controller_settings"]["type"] = "pi"
        cfgi["controller_settings"]["pid"]["kp"] = float(kp)
        cfgi["controller_settings"]["pid"]["ki"] = float(ki)
        outi = base_cfg.parent / "pi_vel_tuned.yaml"
        cfgi.setdefault("output_settings", {})
        cfgi["output_settings"]["run_label"] = "pi_vel_tuned"
        cfgi["output_settings"]["auto_plot"] = True
        outi.write_text(yaml.safe_dump(cfgi, sort_keys=False))
        print(f"Wrote {outi}")


if __name__ == "__main__":
    main()

