import argparse
import math
import yaml
from pathlib import Path


def zeta_from_overshoot(Mp: float) -> float:
    """Mp in [0,1]; uses Mp = exp(-zeta*pi/sqrt(1-zeta^2))."""
    Mp = max(1e-6, min(0.999, Mp))
    lnMp = math.log(Mp)
    return -lnMp / math.sqrt(math.pi**2 + lnMp**2)


def wn_from_Ts5(zeta: float, Ts5: float) -> float:
    # 5% settling time approximation for 2nd order
    return 3.0 / max(1e-9, (zeta * Ts5))


def pid_for_plant_with_actuator(zeta: float, wn: float, tau: float, beta: float = 2.5):
    """
    Plant: P(s) = 1/(s^2 (tau s + 1))
    Controller (D on measurement): u = Kp e + Ki * int(e) - Kd * y_dot

    Desired characteristic: (s^2 + 2 zeta wn s + wn^2)(s + p_a)(s + p_i)
    with p_a + 2 zeta wn + p_i = 1/tau (matching s^3 coefficient/ tau-normalized form).

    Matching yields (after normalizing by tau):
      Kd = tau * [ wn^2 + 2 zeta wn (p_a + p_i) + p_a p_i ]
      Kp = tau * [ wn^2 (p_a + p_i) + 2 zeta wn p_a p_i ]
      Ki = tau * [ wn^2 p_a p_i ]
    """
    inv_tau = 1.0 / max(1e-9, tau)
    p_i = beta * wn
    p_a = inv_tau - 2.0 * zeta * wn - p_i
    if p_a <= 0:
        # if it goes non-positive, clip to small positive to keep stability
        p_a = 0.1 * inv_tau
    Kd = tau * (wn ** 2 + 2.0 * zeta * wn * (p_a + p_i) + p_a * p_i)
    Kp = tau * (wn ** 2 * (p_a + p_i) + 2.0 * zeta * wn * p_a * p_i)
    Ki = tau * (wn ** 2 * p_a * p_i)
    return Kp, Ki, Kd, p_a, p_i


def pd_for_plant_with_actuator(zeta: float, wn: float, tau: float):
    """
    For PD only: desired (s^2 + 2 zeta wn s + wn^2)(s + p_a) and plant char eq
    tau s^3 + s^2 + Kd s + Kp = 0 -> normalized by tau and matched to desired.
    Coefficient match gives:
      1/tau = 2 zeta wn + p_a
      Kd/tau = wn^2 + 2 zeta wn p_a
      Kp/tau = wn^2 p_a
    """
    inv_tau = 1.0 / max(1e-9, tau)
    p_a = inv_tau - 2.0 * zeta * wn
    if p_a <= 0:
        p_a = 0.1 * inv_tau
    Kd = tau * (wn ** 2 + 2.0 * zeta * wn * p_a)
    Kp = tau * (wn ** 2 * p_a)
    return Kp, Kd, p_a


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True, help="YAML to read tau_up/tau_down, defaults, etc.")
    ap.add_argument("--mp", type=float, default=0.15, help="Target overshoot (e.g., 0.15 for 15%)")
    ap.add_argument("--ts5", type=float, default=15.0, help="Target 5% settling time [s]")
    ap.add_argument("--beta", type=float, default=2.5, help="Integral pole factor for PID (p_i=beta*wn)")
    ap.add_argument("--which", choices=["pid","pd"], default="pid")
    ap.add_argument("--tau_mode", choices=["avg","up","down"], default="avg", help="Which actuator tau to use")
    args = ap.parse_args()

    cfg = yaml.safe_load(Path(args.config).read_text())
    tau_up = cfg.get("physics",{}).get("tau_up", 0.1)
    tau_down = cfg.get("physics",{}).get("tau_down", 0.1)
    # Use a small-signal effective tau (average)
    if args.tau_mode == "up":
        tau = tau_up
    elif args.tau_mode == "down":
        tau = tau_down
    else:
        tau = 0.5 * (tau_up + tau_down)

    zeta = zeta_from_overshoot(args.mp)
    wn = wn_from_Ts5(zeta, args.ts5)

    if args.which == "pid":
        Kp, Ki, Kd, p_a, p_i = pid_for_plant_with_actuator(zeta, wn, tau, args.beta)
        print(f"tau_eff={tau:.4f}, zeta={zeta:.3f}, wn={wn:.3f}, p_a={p_a:.3f}, p_i={p_i:.3f}")
        print(f"PID: kp={Kp:.3f}, ki={Ki:.3f}, kd={Kd:.3f}")
    else:
        Kp, Kd, p_a = pd_for_plant_with_actuator(zeta, wn, tau)
        print(f"tau_eff={tau:.4f}, zeta={zeta:.3f}, wn={wn:.3f}, p_a={p_a:.3f}")
        print(f"PD: kp={Kp:.3f}, kd={Kd:.3f}")


if __name__ == "__main__":
    main()
