import math
from pathlib import Path

import numpy as np
import pandas as pd


def step_metrics(t, s, target):
    t = np.asarray(t)
    s = np.asarray(s)
    A = target - s[0]
    tail = max(1, len(s) // 10)
    yfin = float(np.mean(s[-tail:]))
    # overshoot
    if A >= 0:
        pk_idx = int(np.argmax(s))
        peak = float(s[pk_idx])
        pk_t = float(t[pk_idx])
        overshoot = max(0.0, (peak - yfin) / abs(yfin - s[0]) * 100.0)
    else:
        pk_idx = int(np.argmin(s))
        peak = float(s[pk_idx])
        pk_t = float(t[pk_idx])
        overshoot = max(0.0, (yfin - peak) / abs(yfin - s[0]) * 100.0)
    # rise 10-90
    y10 = s[0] + 0.1 * A
    y90 = s[0] + 0.9 * A

    def first_cross(val):
        idx = np.where((s - val) * np.sign(A) >= 0)[0]
        return float(t[idx[0]]) if len(idx) > 0 else float('nan')
    a = first_cross(y10)
    b = first_cross(y90)
    tr = float(b - a) if not math.isnan(a) and not math.isnan(b) else float('nan')
    # settling 5%
    band = 0.05 * abs(A)
    lo, hi = yfin - band, yfin + band
    Ts = float('nan')
    for k in range(len(s) - 1, -1, -1):
        if s[k] < lo or s[k] > hi:
            Ts = float(t[k + 1]) if k + 1 < len(s) else float('nan')
            break
    return yfin, peak, pk_t, overshoot, tr, Ts


def main():
    p = Path('outputs/simulations/dynamics2d/20251012/run_012/run_012.csv')
    df = pd.read_csv(p)
    t = df['time'].to_numpy()
    dt = float(np.median(np.diff(t)))
    tx = float(df['target_x'].iloc[0])
    ty = float(df['target_y'].iloc[0])
    yfin_x, peak_x, pk_t_x, OS_x, Tr_x, Ts_x = step_metrics(t, df['x'].to_numpy(), tx)
    yfin_y, peak_y, pk_t_y, OS_y, Tr_y, Ts_y = step_metrics(t, df['y'].to_numpy(), ty)
    rmse_x = float(np.sqrt(np.mean((df['x'].to_numpy() - tx) ** 2)))
    rmse_y = float(np.sqrt(np.mean((df['y'].to_numpy() - ty) ** 2)))
    iae_x = float(np.sum(np.abs(df['x'].to_numpy() - tx)) * dt)
    iae_y = float(np.sum(np.abs(df['y'].to_numpy() - ty)) * dt)
    itae_x = float(np.sum(t * np.abs(df['x'].to_numpy() - tx)) * dt)
    itae_y = float(np.sum(t * np.abs(df['y'].to_numpy() - ty)) * dt)
    if 'ax_cmd_f' in df.columns and 'ay_cmd_f' in df.columns:
        u_norm = float(np.sum(np.hypot(df['ax_cmd_f'].to_numpy(), df['ay_cmd_f'].to_numpy())) * dt)
    else:
        u_norm = float('nan')
    print('dt =', dt, 'T =', float(t[-1]))
    print('X: yfin', yfin_x, 'peak', peak_x, 'tp', pk_t_x, 'OS%', OS_x, 'Tr', Tr_x, 'Ts', Ts_x, 'RMSE', rmse_x, 'IAE', iae_x, 'ITAE', itae_x)
    print('Y: yfin', yfin_y, 'peak', peak_y, 'tp', pk_t_y, 'OS%', OS_y, 'Tr', Tr_y, 'Ts', Ts_y, 'RMSE', rmse_y, 'IAE', iae_y, 'ITAE', itae_y)
    print('|u|_1 =', u_norm)
    if 'vrel_norm' in df.columns:
        print('avg |v_rel| =', float(df['vrel_norm'].mean()))


if __name__ == '__main__':
    main()
