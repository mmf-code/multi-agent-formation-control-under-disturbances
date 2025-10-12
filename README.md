# Multi-Agent Formation Control Under Disturbances - Quickstart

This repo contains a 2D single-drone dynamics testbed (extensible to multi-drone) with selectable controllers (PID, P/PI/PD, GT2 Fuzzy, and hybrid PID+Fuzzy), YAML-driven configuration, CSV logging, and Python plotting.

## Highlights
- 2D dynamics with ambient wind velocity, linear->quadratic drag blend, asymmetric actuator lag (up/down), semi-implicit Euler
- Controller selection from YAML: pid | p | pi | pd | fuzzy | pid_fuzzy
- CSV outputs under outputs/simulations/dynamics2d/YYYYMMDD/run_###/
- Python plot script with step metrics (OS, tp, Tr10-90, Ts+-5%, RMSE/IAE/ITAE, |u|_1)

## Repo Map
- Dynamics core: agent_control_pkg/include/agent_control_pkg/drone_dynamics_2d.hpp, agent_control_pkg/src/drone_dynamics_2d.cpp
- Controllers
  - PID core: agent_control_pkg/include/agent_control_pkg/pid_controller.hpp, agent_control_pkg/src/pid_controller.cpp
  - Interface: agent_control_pkg/include/agent_control_pkg/controllers/controller_base.hpp
  - PID adapter: agent_control_pkg/include/agent_control_pkg/controllers/pid_adapter.hpp
  - GT2 fuzzy: agent_control_pkg/include/agent_control_pkg/gt2_fuzzy_logic_system.hpp, agent_control_pkg/src/gt2_fuzzy_logic_system.cpp
  - Fuzzy adapter: agent_control_pkg/include/agent_control_pkg/controllers/fuzzy_gt2_adapter.hpp, agent_control_pkg/src/controllers/fuzzy_gt2_adapter.cpp
  - Hybrid PID+Fuzzy: agent_control_pkg/include/agent_control_pkg/controllers/combined_pid_fuzzy_adapter.hpp, agent_control_pkg/src/controllers/combined_pid_fuzzy_adapter.cpp
- Tester: agent_control_pkg/src/dynamics_2d_test_main.cpp
- Configs: agent_control_pkg/config/dynamics_2d_test.yaml, agent_control_pkg/config/dynamics_2d_wind.yaml, agent_control_pkg/config/fuzzy_params.yaml
- Plotting: analysis/plot_dynamics_2d.py
- Docs: docs/dynamics/quickstart_dynamics2d.md, docs/dynamics/dynamics2d_system_and_control.md

## Build (Windows, VS 2022)
```
cmake -S agent_control_pkg -B build -G "Visual Studio 17 2022"
cmake --build build --config Debug --target dynamics_2d_tester
```

## Run
```
# No-wind
build/Debug/dynamics_2d_tester.exe agent_control_pkg/config/dynamics_2d_test.yaml

# Wind
build/Debug/dynamics_2d_tester.exe agent_control_pkg/config/dynamics_2d_wind.yaml
```

CSV path: outputs/simulations/dynamics2d/YYYYMMDD/run_###/run_###.csv

## Plot
```
python analysis/plot_dynamics_2d.py --csv outputs/simulations/dynamics2d/YYYYMMDD/run_###/run_###.csv
```
The plot PNG is saved next to the CSV.

## Controller Selection (YAML)
```
controller_settings:
  type: pid_fuzzy   # pid | p | pi | pd | fuzzy | pid_fuzzy
  pid:
    kp: 0.538
    ki: 0.145
    kd: 1.368
    enable_derivative_filter: true
  fls:
    enable: true
    params_file: fuzzy_params.yaml
  mix:
    k_pid: 1.0
    k_fuzzy: 1.0
```
Notes
- Fuzzy params define IT2 triangular FOUs for variables error, dError, optional wind, and an output variable (e.g., output or correction). Rules are [error_set, dError_set, wind_set, output_set].
- Hybrid sums u_pid and u_fuzzy, then clamps to +-a_max.

## Dynamics (per axis, summary)
- Actuator (1st order, asym up/down): a_cmd_f[k] = a_cmd_f[k-1] + alpha*(a_cmd[k]-a_cmd_f[k-1]), alpha = clamp(dt/tau, 0,1)
- Relative airspeed: v_rel = v - v_wind, |v_rel| = hypot(v_rel,x, v_rel,y)
- Drag blend: a_drag = (1-w)*a_lin + w*a_quad; a_lin = -(c_lin/m)*v_rel, a_quad = -(c_quad/m)*|v_rel|*v_rel; w = clamp((|v_rel|-v_thr)/max(v_thr,1e-6),0,1)
- Semi-implicit Euler: v[k] = v[k-1] + a*dt, x[k] = x[k-1] + v[k]*dt

## Troubleshooting Fuzzy / Hybrid
If pid_fuzzy looks identical to pid:
- Scale output sets to [m/s^2] (e.g., peaks +-6..10). Too small -> u_fuzzy ~ 0.
- Ensure input sets cover the step range (e up to ~5 m; de/dt up to ~0.5..1.0 m/s).
- Increase k_fuzzy (e.g., 0.3..1.0).
- Make sure rule consequents match output set names exactly.

## Outputs & Hygiene
- outputs/ is git-ignored. CSV/plots live per-run under dated folders.
- Configs under agent_control_pkg/config. Docs under docs/.

## See Also
- docs/dynamics/quickstart_dynamics2d.md
- docs/dynamics/dynamics2d_system_and_control.md

