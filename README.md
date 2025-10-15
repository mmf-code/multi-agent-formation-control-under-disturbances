# Multi-Agent Formation Control Under Disturbances - Quickstart

This repo contains a 2D single-drone dynamics testbed (extensible to multi-drone) with selectable controllers (PID, P/PI/PD, GT2 Fuzzy, and hybrid PID+Fuzzy), YAML-driven configuration, CSV logging, and Python plotting.

## Highlights
- 2D dynamics with ambient wind velocity, linear->quadratic drag blend, asymmetric actuator lag (up/down), semi-implicit Euler
- Optional constant wind acceleration bias (wind.ax, wind.ay) for integral-action testing
- Controller selection from YAML: `pid | p | pi | pd | fuzzy | pid_fuzzy`
- Hybrid PID+Fuzzy (GT2) with mix gains and separate contribution logging
- Model-based feed-forward (drag + wind bias cancellation)
- Target prefilter (2x first-order, critical-damped approximation) to reduce overshoot
- CSV outputs under `outputs/simulations/dynamics2d/YYYYMMDD/run_###__/`
- Python tooling for metrics, plotting, batch summaries and final reports

## What’s New (2025-10)
- Output management
  - `output_settings.run_label` → run folder named as `run_###__<label>`
  - `output_settings.auto_plot` → PNG is saved automatically after each run
  - `output_settings.max_runs_per_day` → keep last N runs per day (older are pruned)
- Wind options
  - YAML `wind.vx`, `wind.vy` (ambient velocity)
  - NEW: `wind.ax`, `wind.ay` (constant acceleration bias)
- CSV additions
  - CSV header includes `ax_wind, ay_wind` and hybrid contributions `ax_pid, ay_pid, ax_fuzzy, ay_fuzzy`, plus `ax_ff, ay_ff`
- Analysis utilities (under `analysis/`)
  - `plot_dynamics_2d.py --t_end <T>`: plot a single CSV with optional time window
  - `plot_compare_positions.py`: compare multiple runs (position focus, with zoom 0–20 s)
  - `compute_pid_from_specs.py`: actuator-aware closed-form PID from (Mp, Ts5)
  - `compute_pid_from_velocity.py`: velocity/distance-based PID design (time-optimal inspired)
  - `auto_tune_pid.py`: small grid search for PID specs (Mp, Ts5, beta, tau_mode)
  - `auto_tune_p_pi.py`: grid search for P / PI (for baseline evidence)
  - `summarize_runs.py`: rank all runs (per day) by constraints (OS, Ts5) and metrics
  - `collect_final_report.py`: copy best runs (CSV+PNG) to `outputs/final_report/<day>`
  - `cleanup_runs.py`: move clutter runs to `outputs/simulations/dynamics2d/_trash/<day>`

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

## ROS2 Build & Launch (Linux, ROS 2 Humble)
- `source /opt/ros/humble/setup.bash`
- `colcon build --packages-select my_custom_interfaces_pkg formation_coordinator_pkg agent_control_pkg`
- `source install/setup.bash`
- `ros2 launch agent_control_pkg single_agent_test.launch.py` for a single-agent loop (controller + coordinator)
- `ros2 launch agent_control_pkg multi_agent_formation.launch.py` to spawn three agents under `/agent_i` namespaces
- Controller params live in `agent_control_pkg/config/ros2/agent_controller_default.yaml`; formation params in `other_packages/formation_coordinator_pkg/config/formation_config.yaml`

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

### Position Comparison Plots
```
# 3 runs side-by-side (PD, PID, PID+FLS) up to t=20 s
python analysis/plot_compare_positions.py \
  --csv \
    outputs/simulations/dynamics2d/20251014/run_004__pd_vel_2p5/run_004__pd_vel_2p5.csv \
    outputs/simulations/dynamics2d/20251014/run_005__pid_vel_2p5/run_005__pid_vel_2p5.csv \
    outputs/simulations/dynamics2d/20251014/run_008__pidf_kf10_vel_2p5/run_008__pidf_kf10_vel_2p5.csv \
  --labels PD PID PIDF_k1 \
  --t_end 20 \
  --out outputs/simulations/dynamics2d/20251014/compare_positions_20s.png
```

### Summarize and Collect
```
# Rank runs for a day
python analysis/summarize_runs.py --day 20251014 --os-max 20 --ts-max 8 --top 10

# Collect best runs to a tidy folder
python analysis/collect_final_report.py --day 20251014 --labels pd_vel,pid_vel,pidf_ --top 1
```

## Example Scenarios (Thesis‑ready)
- **No-wind (reference)**: PD often excels (plant is 1/s^2, no integral action needed)
- **Persistent bias** (constant acceleration ax/ay): PID better than PD on IAE; PID+FLS better than PID
- **Time-varying wind** (sinusoidal, step, gust): Tests disturbance rejection under dynamic conditions
- **Robustness test** (wind + bias): PID+FLS > PID > PD on IAE (demonstrates hybrid advantage)

Use provided YAMLs under `agent_control_pkg/config/experiments/`:
- `*_wbias.yaml` → adds constant bias (`wind.ax/ay`) for integral action testing
- `*_wind.yaml` → adds ambient wind velocity (steady-state, absorbed by drag model)
- `*_wind_step.yaml` → step wind disturbance (t=5s → 8 m/s)
- `*_wind_tv.yaml` → sinusoidal wind (5±3 m/s @ 0.5 Hz)
- `*_wind_gust.yaml` → stochastic gust (turbulence model)
- Pick labels and autoplot via `output_settings` in YAML

## Physics and Configuration
- Core files
  - Dynamics: `agent_control_pkg/include/agent_control_pkg/drone_dynamics_2d.hpp`, `agent_control_pkg/src/drone_dynamics_2d.cpp`
  - Controllers: PID, GT2 FLS adapters under `include/agent_control_pkg/controllers/` and `src/controllers/`
  - Tuning: `analysis/compute_pid_from_specs.py`, `analysis/compute_pid_from_velocity.py`, `analysis/auto_tune_pid.py`
- Model: position double-integrator with first-order actuator lag (separate up/down), drag blend (linear->quadratic by speed threshold)
- Feed-forward: drag + wind bias cancellation (`feedforward` section in YAML)
- Target prefilter: two cascaded first-order filters (`target_prefilter: {enabled, tau}`)
- Output management: `output_settings: {run_label, auto_plot, max_runs_per_day}`

See also: `docs/dynamics/advanced_control_notes.md` for prefilter/FF details and closed-form tuning formulas.

## Tuning Guide (Overshoot and Settling)
- Closed-form actuator-aware mapping from specs (Mp, Ts5) to PID (use `compute_pid_from_specs.py`)
- Velocity-based design from distance and desired cruise speed (use `compute_pid_from_velocity.py`)
- Practical tips
  - Increase damping (zeta) first to reduce OS, then adjust wn to hit Ts5
  - Setpoint weighting (2-DOF PID) and prefilter reduce OS without hurting disturbance rejection
  - Use FF to reduce controller workload in wind/bias cases
  - For hybrid, scan `k_fuzzy` in 0.2..1.2, ensure output MF peaks are near +/- 6..10 m/s^2

## ROS2 Roadmap (next phase)
- Architecture
  - Node `agent_controller_node`: subscribes to position setpoints, publishes attitude/thrust commands
  - Node `formation_coordinator_node`: generates waypoints/centers (already present as stubs)
  - Messages: positions (geometry_msgs/PoseStamped), accelerations or attitude setpoints
- Mapping accel->attitude/thrust
  - theta_cmd ~= ax_des/g, phi_cmd ~= -ay_des/g (yaw fixed), T_cmd = m*(g+az_des)
  - enforce angle/rate limits and slew-rate on commands
- Integration path
  - Extract controlloop core (PID, FLS, prefilter, FF) into a reusable library
  - Wrap as ROS2 node with YAML params mirroring this repo
  - Optional SITL: PX4 + Gazebo/Ignition for 3D validation

## Roadmap / What’s Next
- Add 2-DOF PID setpoint weighting to PIDAdapter and YAML key `pid.setpoint_weight_b`
- k_fuzzy fine scan and MF scaling audit under wind+bias
- Wind time-window disturbance scenarios and multi-drone formation runs
- Final figure pack and LaTeX-ready tables from `summarize_runs.py`


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

## Roadmap / What’s Next
- Fuzzy MF/output scaling and `k_fuzzy` fine scan (0.2–1.2) under wind+bias
- Windowed wind disturbances (time windows) and multi‑drone formation tests
- Pack final figures (compare PNGs + summary CSVs) for the thesis results section
- Optional: script to auto‑export LaTeX‑ready tables from `summarize_runs.py`

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


## Time-varying Wind Model
Add `wind_model` section to YAML config for dynamic wind disturbances:

```yaml
wind_model:
  type: sinusoid  # Options: sinusoid | step | gust
  base_vx: 5.0    # Base wind velocity (m/s)
  base_vy: 0.0
  amp_vx: 3.0     # Amplitude for sinusoid/gust (m/s)
  amp_vy: 0.0
  frequency_hz: 0.5  # For sinusoid mode
  step_time: 5.0     # For step mode (seconds)
  step_vx: 8.0       # Step target velocity
  gust_std: 2.0      # For gust mode (standard deviation)
  gust_alpha: 0.95   # For gust mode (LPF smoothing)
```

Wind velocity is updated each simulation step and logged in CSV columns `vx_wind, vy_wind`.

## Thesis Deliverables (Results Section)

All comparison plots are available in `outputs/simulations/dynamics2d/20251015/`:

### Key Figures for Thesis
1. **Figure 1: No-wind baseline** - `compare_positions_20s_nowind.png`
   - Shows PD optimal performance (no persistent disturbance)
   - Reference for evaluating integral action necessity

2. **Figure 2: Persistent bias** - `compare_positions_20s_nowind_bias.png`
   - Demonstrates PID superiority over PD (31-37% IAE improvement)
   - Shows PID+Fuzzy further improves tracking (3-7% over PID)

3. **Figure 3: Step wind disturbance** - `compare_positions_20s_wind_step.png`
   - 8 m/s step at t=5s tests transient response
   - Validates controller robustness to sudden disturbances

4. **Figure 4: Stochastic gust** - `compare_positions_20s_wind_gust.png`
   - Turbulence model tests steady-state disturbance rejection
   - Shows hybrid controller advantage in noisy environments

5. **Figure 5: Time-varying sinusoidal wind** - `compare_positions_20s_wind_tv.png`
   - 5±3 m/s @ 0.5 Hz tests periodic disturbances
   - Demonstrates tracking performance under oscillatory conditions

6. **Figure 6: Wind + bias robustness** - `compare_positions_20s_wind_bias.png`
   - Combined persistent and dynamic disturbances
   - Validates PID+FLS > PID > PD ranking (robustness case)

### Metrics Summary
Use `analysis/summarize_runs.py` to generate LaTeX-ready tables:
```bash
python analysis/summarize_runs.py --day 20251015 --os-max 20 --ts-max 8 --top 20
```

Key metrics: Overshoot (%), Settling time (s), IAE, ITAE, RMSE
