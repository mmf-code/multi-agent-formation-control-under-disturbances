# Dynamics2D System and Controller Notes

This document summarizes the implemented 2D drone dynamics, the exact P/PI/PID controller equations used by the tester, the discrete‑time realizations, parameter mappings (YAML → code), and the step‑response metrics we report. Content mirrors the current C++ implementation so it can serve as a design/validation reference.

## File Map (source of truth)
- Dynamics core: `agent_control_pkg/include/agent_control_pkg/drone_dynamics_2d.hpp`, `agent_control_pkg/src/drone_dynamics_2d.cpp`
- PID controller: `agent_control_pkg/include/agent_control_pkg/pid_controller.hpp`, `agent_control_pkg/src/pid_controller.cpp`
- Controller interface and adapter: `agent_control_pkg/include/agent_control_pkg/controllers/controller_base.hpp`, `agent_control_pkg/include/agent_control_pkg/controllers/pid_adapter.hpp`
- Tester (single‑drone step runner): `agent_control_pkg/src/dynamics_2d_test_main.cpp`
- Example configs: `agent_control_pkg/config/dynamics_2d_test.yaml`, `agent_control_pkg/config/dynamics_2d_wind.yaml`
- Plotter: `analysis/plot_dynamics_2d.py`

## State, Inputs, Disturbances
- State per timestep k: `x_k, y_k, v_xk, v_yk`
- Raw control (from controller): `a_cmd_x[k], a_cmd_y[k]` (desired translational accelerations)
- Actuator‑filtered control (after first‑order actuator model): `a_cmd_fx[k], a_cmd_fy[k]`
- Ambient wind velocity (disturbance): `v_wx, v_wy` (m/s)
- Optional constant wind acceleration bias: `a_wx, a_wy` (default zero; kept for legacy)

## Dynamics (per axis; see `drone_dynamics_2d.cpp`)
Notation: `dt > 0`, `m = mass`, `c_lin = drag_coeff_lin`, `c_quad = drag_coeff_quad`, `v_thr = drag_speed_threshold`, `a_max = max_accel`.

1) Actuator (first‑order, asymmetric tau up/down)
   - Select time constant depending on command direction:
     - `tau_x = (a_cmd_x > a_cmd_fx) ? (tau_up > 0 ? tau_up : tau) : (tau_down > 0 ? tau_down : tau)`
     - same for y
   - Discrete update (exponential smoothing):
     - `alpha = clamp(dt / max(tau, 1e-6), 0, 1)`
     - `a_cmd_f[k] = a_cmd_f[k-1] + alpha * (a_cmd[k] - a_cmd_f[k-1])`
   - Per‑axis saturation:
     - `a_cmd_f[k] = clamp(a_cmd_f[k], -a_max, +a_max)`

2) Relative airspeed (ambient wind velocity)
   - `v_rel = (v_x - v_wx, v_y - v_wy)`, `|v_rel| = sqrt(v_rel_x^2 + v_rel_y^2)`

3) Aerodynamic drag (blend linear → quadratic based on |v_rel|)
   - Linear component per axis: `a_lin = -(c_lin/m) * v_rel_axis`
   - Quadratic component per axis: `a_quad = -(c_quad/m) * |v_rel| * v_rel_axis`
   - Blend weight: `w = clamp((|v_rel| - v_thr) / max(v_thr, 1e-6), 0, 1)`
   - Axis drag: `a_drag_axis = (1 - w) * a_lin + w * a_quad`

4) Total acceleration and semi‑implicit Euler integration
   - `a_x = a_cmd_fx + a_drag_x + a_wx`
   - `a_y = a_cmd_fy + a_drag_y + a_wy`
   - Update:
     - `v_x[k] = v_x[k-1] + a_x * dt`, `v_y[k] = v_y[k-1] + a_y * dt`
     - `x[k]   = x[k-1]   + v_x[k] * dt`, `y[k]   = y[k-1]   + v_y[k] * dt`

## Controller Forms (per axis)
The tester drives position → acceleration via the `IController1D` interface. Current adapter is a PID with derivative on measurement and anti‑windup (see `pid_controller.cpp`). Let `e[k] = r[k] - y[k]` where `r` is the position target (e.g., 5 m).

Baseline continuous forms:
- P: `u = Kp * e`
- PI: `u = Kp * e + Ki ∫ e dt`
- PID (D on measurement): `u = Kp * e - Kd * d(y)/dt + Ki ∫ e dt`

Discrete‑time realization used in code (per step dt):
- Error: `e[k] = r[k] - y[k]`
- Integral state: `I[k] = I[k-1] + e[k] * dt`
- Derivative on measurement with optional 1st‑order LPF:
  - Raw: `y_dot_raw[k] = (y[k] - y[k-1]) / dt`
  - Filter: `y_dot_f[k] = α * y_dot_raw[k] + (1-α) * y_dot_f[k-1]`, `α ∈ (0,1]`
  - D‑term: `D = -Kd * y_dot_f[k]`
- Output before clamp: `u* = Kp * e[k] + Ki * I[k] + D`
- Output clamp: `u = clamp(u*, u_min, u_max)`

Anti‑windup strategy (as implemented):
1) Predict `u*`. If `u*` would saturate and `e[k]` increases saturation, undo the just‑applied integral increment (`I[k] -= e[k] * dt`).
2) Clamp `I[k]` magnitude so `Ki*I` cannot dominate (proportional to output range).
3) After clamping `u`, apply partial back‑calculation: `I[k] ← I[k] - 0.5 * (u* - u)/Ki` when `Ki ≠ 0`.

Switching controller type without code changes:
- P: set `Ki = 0`, `Kd = 0`
- PI: set `Kd = 0`
- PD: set `Ki = 0`
- PID: all non‑zero

Planned (optional): add `controller.type: p|pi|pd|pid` in YAML and construct the appropriate `IController1D` implementation in the tester.

## YAML → Code Parameter Map
- `simulation_settings.dt`, `simulation_settings.total_time` → integrator step and horizon
- `physics.mass` → `m`
- `physics.cd_lin`, `physics.cd_quad`, `physics.v_thr` → drag coefficients and blend threshold
- `physics.a_max` → `a_max` (acceleration clamp)
- `physics.tau_up`, `physics.tau_down` (and `actuator_tau` fallback) → actuator first‑order time constants
- `wind.vx`, `wind.vy` → ambient wind velocity `(v_wx, v_wy)`
- `controller_settings.pid.{kp,ki,kd, enable_derivative_filter}` → PID gains and derivative LPF

## Reported Metrics (plot/analysis)
All versus time `t[k]` sampled at `dt`.
- Overshoot OS%: `(peak − final)/|final − initial| × 100` (sign‑aware)
- Peak time: `t` at peak
- Rise time 10–90%: `t(90%) − t(10%)`
- Settling time (±5%): last time where signal exits the final ±5% band
- RMSE: `sqrt(mean((y − r)^2))`
- IAE: `∑ |y − r| dt`
- ITAE: `∑ t |y − r| dt`
- Control effort L1 (if filtered commands present): `∫ ||[a_cmd_fx, a_cmd_fy]|| dt`

## Example Results (50 s runs)
Rüzgâr açık (kp=0.9, ki=0.2, kd=0.45):
- X: OS ≈ 74.4%, tp ≈ 3.39 s, Tr ≈ 1.15 s, Ts(±5%) ≈ 23.80 s, RMSE ≈ 1.093 m, IAE ≈ 26.99, ITAE ≈ 201.72
- Y: OS ≈ 70.8%, tp ≈ 3.41 s, Tr ≈ 1.17 s, Ts(±5%) ≈ 21.12 s, RMSE ≈ 1.066 m, IAE ≈ 26.19, ITAE ≈ 194.47
- |u|₁ ≈ 34.17, E[|v_rel|] ≈ 1.50 m/s

Rüzgâr kapalı (kp=1.0, ki=0.10, kd=0.35):
- X≈Y (simetrik hedef): OS ≈ 68.1%, tp ≈ 3.20 s, Tr ≈ 1.11 s, Ts(±5%) ≈ 22.24 s, RMSE ≈ 1.017 m, IAE ≈ 24.14, ITAE ≈ 168.24

## Usage Recap (tester and plot)
- Configure (examples above), build target `dynamics_2d_tester`, run with a YAML, then plot:
  - `cmake --build build --config Debug --target dynamics_2d_tester`
  - `build/Debug/dynamics_2d_tester.exe agent_control_pkg/config/dynamics_2d_wind.yaml`
  - `python analysis/plot_dynamics_2d.py --csv outputs/simulations/dynamics2d/YYYYMMDD/run_###/run_###.csv`

## Tuning Hints
- Overshoot ↓ / Ts ↓: artır `Kd` veya azalt `Kp` (integratör rüzgâr altında gerekli ama OS’i yükseltir → `Ki`yi küçük adımlarla artır).
- Aerodinamik sönümleme için `cd_quad` ve `v_thr` küçük dokunuşlarla pikleri yumuşatır; önce kontrol tarafını ayarla, sonra fizik dokunuşu yap.

## Closed‑Form Initial PID Tuning (from OS, Ts5)
Use these mappings as an engineer’s starting point. They come from matching the closed‑loop characteristic to a desired second/third‑order form for a double‑integrator plant controlled in acceleration (our case). Assumes setpoint is a step and D is on measurement (equivalent to D on error for steps).

1) Pick target overshoot `M_p` and 5% settling time `Ts5` for the dominant response. Compute damping ratio `ζ` and natural frequency `ω_n`:

   - `ζ = sqrt( (ln M_p)^2 / (π^2 + (ln M_p)^2) )`
   - `ω_n ≈ 3 / (ζ · Ts5)`  (5% criterion; use `4/(ζ·Ts2)` for 2% if needed)

2) Controller mappings (per axis)
   - PD (no integral): `Kp = ω_n^2`, `Kd = 2 ζ ω_n`
   - PID (third order with a fast real pole `p_i = α ω_n`, α ∈ [2, 5]):
     - `Kd = 2 ζ ω_n + p_i`
     - `Kp = ω_n^2 + 2 ζ ω_n p_i`
     - `Ki = ω_n^2 p_i`

   Notes: Units are 1/s² for `Kp`, 1/s for `Kd`, and 1/s³ for `Ki`. Our code applies these directly as acceleration commands; actuator filtering and drag will slightly change effective dynamics, therefore treat results as high‑quality initials and fine‑tune on simulation.

3) Ready‑to‑try sets (computed examples)

   Baseline (no wind): target `M_p = 15%`, `Ts5 = 15 s`, choose `α = 2.5`
   - `ζ ≈ 0.516`, `ω_n ≈ 0.388 rad/s`, `p_i ≈ 0.969`
   - PID: `Kp ≈ 0.538`, `Ki ≈ 0.145`, `Kd ≈ 1.368`
   - PD:  `Kp ≈ 0.150`, `Kd ≈ 0.399`  (set `Ki = 0`)

   Wind scenario: target `M_p = 20%`, `Ts5 = 18 s`, choose `α = 2.5`
   - `ζ ≈ 0.456`, `ω_n ≈ 0.366 rad/s`, `p_i ≈ 0.914`
   - PID: `Kp ≈ 0.437`, `Ki ≈ 0.122`, `Kd ≈ 1.247`
   - PD:  `Kp ≈ 0.134`, `Kd ≈ 0.333`

   Faster/low‑overshoot option: `M_p = 10%`, `Ts5 = 12 s`, `α = 2.5`
   - `ζ ≈ 0.591`, `ω_n ≈ 0.423 rad/s`, `p_i ≈ 1.058`
   - PID: `Kp ≈ 0.708`, `Ki ≈ 0.189`, `Kd ≈ 1.558`
   - PD:  `Kp ≈ 0.179`, `Kd ≈ 0.500`

Implementation tips
- Start with PD using the PD formulas; confirm OS and Ts trends. Then enable `Ki` from the corresponding PID line to remove steady‑state bias (especially with wind).
- If overshoot is still high, increase `Kd` by 10–20% or reduce `Kp` slightly. If steady‑state is slow, increase `Ki` carefully (watch OS).
- Derivative filter `alpha` trades noise vs. phase lag (typical 0.05–0.2). Lower `alpha` → more smoothing, slower D.

## Controller Selection and Hybrid (PID+Fuzzy)

- YAML key `controller_settings.type` selects the controller per axis:
  - `pid` (default), `p`, `pi`, `pd`
  - `fuzzy` (GT2 IT2 FLS with triangular FOUs and Karnik–Mendel)
  - `pid_fuzzy` (hybrid): `u = k_pid · u_pid + k_fuzzy · u_fuzzy`, then saturate to ±`a_max`.
  - Mix gains from YAML: `controller_settings.mix.k_pid`, `controller_settings.mix.k_fuzzy`.
- Fuzzy params are loaded from `controller_settings.fls.params_file` and can define variables `error`, `dError`, optional `wind`, and the output variable (e.g., `output` or `correction`). The adapter auto‑detects the output variable name if it is not `output`.

### Why PID and PID+Fuzzy may look identical now
- `u_fuzzy` magnitude may be near zero if output membership sets are not scaled in [m/s²] realistically; or mix `k_fuzzy` is too small.
- Input memberships must cover the step range (e.g., `e` up to ≈ 5 m and `de/dt` up to ≈ 0.5–1.0 m/s initially). Too narrow sets ⇒ weak firing.
- Action items:
  - Ensure output set peaks cover a plausible command range (e.g., ±6…10 m/s²).
  - Verify rule consequents match the output variable’s set names exactly.
  - Optionally log `u_pid` and `u_fuzzy` separately for diagnostics.

### GT2 Fuzzy pipeline (summary)
- Fuzzify inputs with IT2 triangular FOUs (lower/upper triangles).
- Rule firing via min t‑norm on interval memberships.
- Karnik–Mendel type reduction (left/right) and defuzzify by averaging.
- Units: output is interpreted as acceleration [m/s²] and then clamped.

## Tuning Methods: ZN vs Better Options

- Ziegler–Nichols (ZN): Fast to get going, often 20–40% overshoot on nonlinear plants like ours. Good for a first pass and for reporting, but not the final word.
- Tyreus–Luyben (TL): More conservative than ZN; typically reduces overshoot and oscillation for similar plants without long derivations.
- Cohen–Coon (CC): Step‑response based; good when process dynamics are asymmetric. Can be aggressive on noisy data.
- Åström–Hägglund Relay Auto‑Tune: Automatically finds Ku, Pu (ultimate gain/period) without manual sweeps; apply ZN/TL formulas afterward.
- IMC/Lambda Tuning: Choose a closed‑loop time constant λ and shape a robust response; generally more robust to disturbances and modeling error.
- Direct Pole Placement (Recommended here): Use the 2nd/3rd‑order target forms already listed above (ζ, ω_n, and an integral pole p_i) and back‑solve PD/PID. This aligns with our double‑integrator + lag + drag structure and gives clear control over overshoot and settling time.

Practical thesis guidance
- Report ZN (Classic) as a baseline, then TL as a “less‑overshoot” alternative. Conclude with pole‑placement (our formulas) as the preferred, traceable design that best meets specs under wind.
- When wind bias causes steady‑state error with PD, enable the integral pole (PID) using the 3rd‑order mapping above.

## Experiment Plan (Comparable Conditions)

- Controllers: p, pi, pd, pid, fuzzy (GT2), pid_fuzzy (hybrid).
- Conditions: same `dt`, `total_time`, target (step to 5 m), physics, actuator limits; repeat with (vx, vy) = (0,0) and a mild wind, e.g. (1.0, −0.5) m/s.
- Metrics: OS, peak time, rise 10–90%, Ts±5%, RMSE, IAE, ITAE, and |u|₁ (already computed by the plotting script).
- Hybrid diagnostics: CSV now includes `ax_pid, ay_pid, ax_fuzzy, ay_fuzzy` so you can see contributions.

Procedure
1) Start with PD using the PD line above to hit an overshoot target; confirm step metrics with no wind.
2) Switch to PID using the mapped `p_i` to remove steady‑state bias; verify against wind.
3) Run ZN (Classic) from Ku/Pu (either manual sweep or relay auto‑tune) and TL; compare.
4) Enable fuzzy (use current MFs/rules); confirm output membership peaks are scaled in [m/s²] ~ ±6…10; plot contributions.
5) Try hybrid with `k_pid≈1.0, k_fuzzy in [0.3, 1.0]`; verify that `u_fuzzy` materially contributes in transients and in rejecting wind.

Notes on Fuzzy Design
- Input ranges: ensure `error` covers ± step amplitude (≈±5 m) and `dError` covers early transient slopes (~±0.5…1.0 m/s). Too narrow → weak firing.
- Output scaling: set peaks of output sets to physically plausible accelerations (±6…10 m/s²) to avoid `u_fuzzy ≈ 0`.
- Wind usage: include the `wind` input when running wind scenarios. If its effect is weak, widen wind MFs and re‑balance rules so strong wind biases map to stronger corrective sets.
