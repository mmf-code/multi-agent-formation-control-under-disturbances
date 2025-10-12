# 2D Dynamics — Quickstart, Validation, and Outputs

This guide shows how to run the new 2D dynamics simulation, what changed in the code, where outputs go, and how to read the plots and metrics.

## What’s Implemented

- Relative‑wind model and drag blend
  - Quadratic drag uses speed norm |v| (cross‑coupled): agent_control_pkg/src/drone_dynamics_2d.cpp:1
  - Linear→quadratic blend weight also uses |v| (threshold): agent_control_pkg/src/drone_dynamics_2d.cpp:1
- Asymmetric actuator lag (up/down taus) and diagnostics
  - agent_control_pkg/include/agent_control_pkg/drone_dynamics_2d.hpp:1
  - agent_control_pkg/src/drone_dynamics_2d.cpp:1
  - Diagnostics getters for filtered command, drag accel, |v_rel|
- Controller interface (PID adapter) for future plug‑ins
  - Base: agent_control_pkg/include/agent_control_pkg/controllers/controller_base.hpp:1
  - PID adapter: agent_control_pkg/include/agent_control_pkg/controllers/pid_adapter.hpp:1
  - Wired into tester: agent_control_pkg/src/dynamics_2d_test_main.cpp:1
- YAML‑driven configuration
  - Example: agent_control_pkg/config/dynamics_2d_test.yaml:1
  - Loads: dt/total_time, physics (mass, cd_lin, cd_quad, v_thr, a_max, tau_up/down), wind (vx, vy), target (x, y), and PID gains
- Outputs and plots
  - CSV+PNG under: outputs/simulations/dynamics2d/YYYYMMDD/run_###/
  - Plot panels include position, velocity, control (raw vs filtered), drag & |v_rel|, step errors with bands/metrics

## Run It

1) Build (VS generator example)
```
cmake -S agent_control_pkg -B build -G "Visual Studio 17 2022"
cmake --build build --config Debug --target dynamics_2d_tester
```

2) Run with example YAML
```
build/Debug/dynamics_2d_tester.exe agent_control_pkg/config/dynamics_2d_test.yaml
```

3) Plot latest CSV
```
agent_control_pkg/.venv/Scripts/python.exe analysis/plot_dynamics_2d.py
```
The plot (.png) is saved next to the CSV in the run folder.

## Output Structure
- CSV path: `outputs/simulations/dynamics2d/YYYYMMDD/run_###/run_###.csv`
- PNG path: `outputs/simulations/dynamics2d/YYYYMMDD/run_###/plot_<timestamp>.png`

CSV columns (subset)
- time, x, y, vx, vy
- ax_cmd, ay_cmd (raw control), ax_cmd_f, ay_cmd_f (filtered)
- ax_drag, ay_drag (drag accelerations), vrel_norm (|v_rel|)
- ax_est, ay_est (estimated accels from Δv)
- target_x, target_y
- e_x, |e_x|, e_y, |e_y|
- kp, ki, kd, vx_wind, vy_wind, cd_lin, cd_quad, v_thr, tau_up, tau_down, a_max, dt

## Plot Reading
- Position: x/y vs time with target lines
- Velocity: vx/vy vs time
- Control: raw vs filtered accelerations (ivme komutları)
- Drag & |v_rel|: drag accelerations with relative airspeed magnitude
- Step Error X: error curve with 2%/5% bands and metrics box (Overshoot %, Peak time, Settling 2%/5%, Rise 10–90%)
- Step Error Y: error curve with bands (no metrics box by default)

## Tuning Tips
- Start without wind (wind.vx=0, vy=0), set Ki small (e.g., 0.1–0.2) to remove bias; use Kd for overshoot control.
- With wind on, adjust Ki upwards carefully; large Ki with actuator lag can cause overshoot.
- Blend threshold v_thr ≈ 1–3 m/s; cd_quad small at first (0.02–0.04) then increase if high‑speed drag is under‑modeled.

## Next Up
- Controller plug‑ins (PI/PID/FLS) selectable via YAML `controller.type`
- Post‑hoc summary: RMSE, MAE, IAE, ITAE, ∫|a_cmd|dt for fair comparisons
- Ziegler–Nichols (classic/no‑overshoot) and safe relay autotune modules

