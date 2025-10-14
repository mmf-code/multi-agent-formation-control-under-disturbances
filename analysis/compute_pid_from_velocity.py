import argparse
import math
import os
import re
import subprocess
from pathlib import Path
from typing import Dict

import numpy as np
import pandas as pd
import yaml


def zeta_from_overshoot(Mp: float) -> float:
    """Overshoot fraction (e.g., 0.08 for 8%)."""
    Mp = max(1e-6, min(0.999, float(Mp)))
    lnMp = math.log(Mp)
    return -lnMp / math.sqrt(math.pi**2 + lnMp**2)


def wn_from_ts5(zeta: float, Ts5: float) -> float:
    return 3.0 / max(1e-9, (zeta * Ts5))


def wn_from_bandwidth(zeta: float, omega_bw: float) -> float:
    """Solve approximate relation between 2nd-order bandwidth and wn.
    omega_bw ≈ wn * sqrt(1 - 2*zeta^2 + sqrt(4*zeta^4 - 4*zeta^2 + 2)).
    """
    term = 1.0 - 2.0 * zeta * zeta + math.sqrt(max(0.0, 4.0 * zeta ** 4 - 4.0 * zeta * zeta + 2.0))
    denom = math.sqrt(max(1e-12, term))
    return omega_bw / denom


def actuator_aware_pid(zeta: float, wn: float, tau: float, beta: float) -> Dict[str, float]:
    inv_tau = 1.0 / max(1e-9, tau)
    p_i = beta * wn
    p_a = inv_tau - 2.0 * zeta * wn - p_i
    if p_a <= 0.0:
        p_a = 0.1 * inv_tau
    kd = tau * (wn ** 2 + 2.0 * zeta * wn * (p_a + p_i) + p_a * p_i)
    kp = tau * (wn ** 2 * (p_a + p_i) + 2.0 * zeta * wn * p_a * p_i)
    ki = tau * (wn ** 2 * p_a * p_i)
    return {"kp": kp, "ki": ki, "kd": kd, "p_a": p_a, "p_i": p_i}


def eval_metrics(csv_path: Path) -> Dict[str, float]:
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
        peak_time = float(t[peak_idx])
        OS = max(0.0, (peak_val - yfin) / abs(yfin - x[0]) if A >= 0 else (yfin - peak_val) / abs(yfin - x[0])) * 100.0
        lo, hi = yfin - 0.05 * abs(yfin - x[0]), yfin + 0.05 * abs(yfin - x[0])
        Ts = float('nan')
        for k in range(len(x) - 1, -1, -1):
            if x[k] < lo or x[k] > hi:
                Ts = float(t[k + 1]) if k + 1 < len(x) else float('nan')
                break
    else:
        OS = float('nan'); peak_time = float('nan'); Ts = float('nan')
    dtm = float(np.median(np.diff(t))) if len(t) > 1 else 0.0
    rmse = float(np.sqrt(np.mean((x - tx) ** 2)))
    iae = float(np.sum(np.abs(x - tx)) * dtm)
    return {"OS": OS, "tp": peak_time, "Ts5": Ts, "RMSE": rmse, "IAE": iae}


def main():
    ap = argparse.ArgumentParser(description="Velocity-based PID tuning (actuator-aware)")
    ap.add_argument("--base-config", required=True, help="YAML to read physics (tau_up/down, a_max, etc.)")
    ap.add_argument("--distance", type=float, required=True, help="Step distance [m] (e.g., sqrt(5^2+5^2)=7.07)")
    ap.add_argument("--v-cruise", type=float, required=True, help="Target cruise velocity [m/s]")
    ap.add_argument("--overshoot", type=float, default=0.10, help="Max overshoot as fraction (0.10=10%)")
    ap.add_argument("--beta", type=float, default=4.0, help="Integral pole factor (p_i=beta*wn)")
    ap.add_argument("--tau-mode", choices=["avg", "up", "down"], default="down")
    ap.add_argument("--method", choices=["scaling", "bw", "time_opt"], default="time_opt")
    ap.add_argument("--scaling", type=float, default=2.5, help="Scaling factor for method=scaling")
    ap.add_argument("--time-opt-factor", type=float, default=1.8, help="Multiplier from time-optimal to Ts5")
    ap.add_argument("--label", type=str, default="pid_vel_tuned")
    ap.add_argument("--out-yaml", type=str, default="agent_control_pkg/config/experiments/pid_vel_tuned.yaml")
    ap.add_argument("--run-sim", action="store_true")
    ap.add_argument("--exe", type=str, default=str(Path("build/Release/dynamics_2d_tester.exe")))
    args = ap.parse_args()

    cfg = yaml.safe_load(Path(args.base_config).read_text())
    tau_up = cfg.get("physics", {}).get("tau_up", 0.1)
    tau_down = cfg.get("physics", {}).get("tau_down", 0.1)
    a_max = cfg.get("physics", {}).get("a_max", 12.0)
    tau = {"avg": 0.5 * (tau_up + tau_down), "up": tau_up, "down": tau_down}[args.tau_mode]

    zeta = zeta_from_overshoot(args.overshoot)

    if args.method == "scaling":
        wn = (args.v_cruise / max(1e-9, args.distance)) * args.scaling
        Ts5_target = 3.0 / (zeta * wn)
    elif args.method == "bw":
        omega_bw = args.v_cruise / max(1e-9, args.distance)
        wn = wn_from_bandwidth(zeta, omega_bw)
        Ts5_target = 3.0 / (zeta * wn)
    else:  # time_opt
        t_accel = args.v_cruise / max(1e-9, a_max)
        d_accel = 0.5 * a_max * t_accel * t_accel
        if args.distance > 2.0 * d_accel:
            t_total = 2.0 * t_accel + (args.distance - 2.0 * d_accel) / args.v_cruise
        else:
            t_total = 2.0 * math.sqrt(args.distance / max(1e-9, a_max))
        Ts5_target = args.time_opt_factor * t_total
        wn = wn_from_ts5(zeta, Ts5_target)

    gains = actuator_aware_pid(zeta, wn, tau, args.beta)

    # write YAML
    cfg.setdefault("controller_settings", {}).setdefault("pid", {})
    cfg["controller_settings"]["pid"].update({
        "kp": float(gains["kp"]), "ki": float(gains["ki"]), "kd": float(gains["kd"])
    })
    # attach output settings for nice naming & autoplot
    out_settings = cfg.setdefault("output_settings", {})
    out_settings.setdefault("run_label", args.label)
    out_settings.setdefault("auto_plot", True)
    out = Path(args.out_yaml)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(yaml.safe_dump(cfg, sort_keys=False))

    print(f"distance={args.distance:.3f} v_cruise={args.v_cruise:.3f} overshoot={args.overshoot*100:.1f}% tau={tau:.3f}")
    print(f"zeta={zeta:.3f} wn={wn:.3f} rad/s  Ts5_target~{Ts5_target:.2f}s  beta={args.beta:.2f}")
    print(f"PID: kp={gains['kp']:.3f} ki={gains['ki']:.3f} kd={gains['kd']:.3f}")
    print(f"Wrote YAML: {out}")

    if args.run_sim:
        exe = Path(args.exe)
        cp = subprocess.run([str(exe), str(out)], capture_output=True, text=True)
        out_text = (cp.stdout or "") + "\n" + (cp.stderr or "")
        print(out_text)
        m = re.search(r"Wrote:\s+(.*\.csv)", out_text)
        if m:
            csv_path = Path(m.group(1).strip())
            mets = eval_metrics(csv_path)
            print(f"Metrics: OS={mets['OS']:.1f}%  Ts5={mets['Ts5']:.2f}s  tp={mets['tp']:.2f}s  RMSE={mets['RMSE']:.3f}  IAE={mets['IAE']:.3f}")
        else:
            print("[WARN] Could not locate output CSV to evaluate metrics.")


if __name__ == "__main__":
    main()











