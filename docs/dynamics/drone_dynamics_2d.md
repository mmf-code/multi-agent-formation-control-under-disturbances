# Drone Dynamics (2D) — Model and Upgrade Plan

This note documents the current 2D drone dynamics model used in the project, practical upgrades to increase realism (without changing the controller yet), and how the test harness produces step-response metrics and plots. The design is ROS 2–ready: the 2D core stays a clean C++ library that higher layers (standalone runner or ROS 2 nodes) can reuse unchanged.

## Current Model (Implemented)

- Files
  - Code: `agent_control_pkg/include/agent_control_pkg/drone_dynamics_2d.hpp`, `agent_control_pkg/src/drone_dynamics_2d.cpp`
  - Tester: `agent_control_pkg/src/dynamics_2d_test_main.cpp`
  - Plot: `analysis/plot_dynamics_2d.py`
  - CMake targets: `dynamics_2d_lib`, `dynamics_2d_tester`

- State and inputs
  - State: position and velocity in a plane `S = {x, y, vx, vy}`
  - Inputs: commanded accelerations `a_cmd = {ax_cmd, ay_cmd}`
  - Disturbance: constant wind acceleration `{ax_w, ay_w}` (will be upgraded to an ambient wind velocity model)

- Parameters (per `DroneDynamics2D::Params`)
  - `mass` [kg]
  - `drag_coeff_lin` [kg/s] — linear aerodynamic damping
  - `max_accel` [m/s^2] — per-axis saturation on commanded acceleration
  - `actuator_tau` [s] — first-order actuator (motor/ESC) lag on acceleration commands

- Equations (per-axis, semi-implicit Euler integration)
  1) Actuator lag (first-order filter)
     - `a_cmd_f(k+1) = a_cmd_f(k) + alpha * (a_cmd(k) - a_cmd_f(k))`, with `alpha = clamp(dt / tau, 0, 1)`
  2) Saturation
     - `a_cmd_f = clamp(a_cmd_f, -a_max, a_max)`
  3) Linear drag (on body velocity)
     - `a_drag = -(c_lin / m) * v`
  4) Total acceleration
     - `a = a_cmd_f + a_drag + a_wind`
  5) Integration (semi-implicit)
     - `v(k+1) = v(k) + a * dt`, `p(k+1) = p(k) + v(k+1) * dt`

- Test harness and outputs
  - Tester drives a planar step from `(0,0)` to `(5,5)` with two position P–D loops creating acceleration commands.
  - CSV columns: `time,x,y,vx,vy,ax_cmd,ay_cmd,ax_est,ay_est,target_x,target_y` (estimated accels from velocity diffs)
  - Plot panels: position (with targets), velocity, commanded acceleration, step error(X) with metrics box (overshoot, peak time, settling 2/5%, rise 10–90%).

## Near-Term Physics Upgrades (Controller unchanged)

1) Aerodynamic drag: piecewise linear → quadratic at higher speeds
- Motivation: Low-speed flow on small rotors/frames often appears nearly linear in speed; at higher speeds the drag grows ~quadratically.
- Model (per axis, using relative airspeed — see item 2):
  - Linear + quadratic: `a_drag = -(c_lin/m) * v_rel - (c_quad/m) * |v_rel| * v_rel`
  - Piecewise (threshold `v_thr`):
    - if `|v_rel| <= v_thr`: `a_drag = -(c_lin/m) * v_rel`
    - else: `a_drag = -(c_lin/m) * v_thr * sign(v_rel) - (c_quad/m) * (|v_rel|-v_thr) * v_rel`
- Parameters to add: `drag_coeff_quad`, `drag_speed_threshold`

2) Wind as ambient air velocity (flow), not a constant acceleration
- Replace constant `a_wind` by an ambient wind velocity `v_wind = {vx_w, vy_w}` and compute relative airspeed `v_rel = v - v_wind`.
- Drag uses `v_rel`. Under steady wind, the craft drifts to an equilibrium where drag balances the control, rather than accelerating indefinitely.
- Optionally support time-varying wind `v_wind(t)` (sinusoidal gusts or filtered noise) for disturbance-rejection tests.

3) Actuator (thrust) dynamics — still 2D, lightweight
- Keep the external API (acceleration commands), but allow asymmetric time constants or a simple deadzone:
  - `tau_up`, `tau_down` (faster spin-down than spin-up or vice versa)
  - Optional nonlinearity (soft-limiting) before saturation to mimic diminishing returns near max command
- Leave full `u → ω → T∝ω²` mapping for the future 3D model to avoid scope creep now.

4) Numerical details
- Keep semi-implicit Euler (symplectic) with sufficiently small `dt` (e.g., 0.01–0.02 s). RK4 can be added later for 3D attitude.
- Apply saturations before integration; piecewise drag should be computed on the current (or predicted) velocity consistently.

## Implemented Upgrades (Now in code)

- Ambient wind as air velocity (relative airflow)
  - API: `setWindVelocity(vx_wind, vy_wind)` implemented at `agent_control_pkg/src/drone_dynamics_2d.cpp:20`
  - Header declaration at `agent_control_pkg/include/agent_control_pkg/drone_dynamics_2d.hpp:36`
  - Used in tester at `agent_control_pkg/src/dynamics_2d_test_main.cpp:32`
  - Effect: drag is computed from relative airspeed `v_rel = v - v_wind` (`agent_control_pkg/src/drone_dynamics_2d.cpp:46`)

- Piecewise linear→quadratic drag (smooth blend)
  - Parameters added to `Params` (header): `drag_coeff_quad`, `drag_speed_threshold` at `agent_control_pkg/include/agent_control_pkg/drone_dynamics_2d.hpp:11`
  - Implementation: `drag_axis` lambda blends linear and quadratic terms at `agent_control_pkg/src/drone_dynamics_2d.cpp:51`
  - Total acceleration uses `ax_drag/ay_drag` with blend at `agent_control_pkg/src/drone_dynamics_2d.cpp:69`

- Asymmetric actuator lag (up/down taus)
  - Parameters: `actuator_tau_up`, `actuator_tau_down` at `agent_control_pkg/include/agent_control_pkg/drone_dynamics_2d.hpp:13`
  - Implementation chooses tau per sign of command change and computes `alpha_x/alpha_y` at `agent_control_pkg/src/drone_dynamics_2d.cpp:36`

- YAML support for new physics parameters
  - Parser reads `drag_coeff_quad|cd_quad`, `drag_speed_threshold|drag_v_threshold|v_thr`, and `actuator_tau_up|tau_up`, `actuator_tau_down|tau_down` at `agent_control_pkg/src/config_reader.cpp:198`

### Code Path Explanation

1) In each step, the actuator filter applies potentially different time constants for increasing vs. decreasing commands (asymmetric lag) and updates the filtered commands (`alpha_x/alpha_y`). See `agent_control_pkg/src/drone_dynamics_2d.cpp:36`.

2) The model computes the relative airspeed by subtracting ambient wind velocity from the body velocity (`vx_rel`, `vy_rel`). See `agent_control_pkg/src/drone_dynamics_2d.cpp:46`.

3) Drag per axis is a smooth blend between linear and quadratic laws. Below a threshold speed, the linear term dominates; above it, the quadratic term gradually takes over. See `agent_control_pkg/src/drone_dynamics_2d.cpp:51`.

4) Total acceleration sums filtered command, drag, and (optional) constant acceleration bias for legacy wind. See `agent_control_pkg/src/drone_dynamics_2d.cpp:69`.

5) Tester constructs a planar step to (5,5), sets ambient wind velocity, logs CSV, and the plotting script produces step-response metrics and bands. See `agent_control_pkg/src/dynamics_2d_test_main.cpp:32` and `analysis/plot_dynamics_2d.py:1`.

## Pseudocode (proposed step with drag + wind-velocity)

```cpp
// Inputs: ax_cmd, ay_cmd, dt
// Params: mass m, c_lin, c_quad, v_thr, tau_up, tau_down, v_wind={vxw, vyw}

// 1) Actuator lag (asymmetric)
double tau = ( (ax_cmd > ax_cmd_f_) ? tau_up : tau_down );
double alpha = std::clamp(dt / std::max(tau, 1e-6), 0.0, 1.0);
ax_cmd_f_ += alpha * (ax_cmd - ax_cmd_f_);
ay_cmd_f_ += alpha * (ay_cmd - ay_cmd_f_);

// 2) Saturate command
ax_cmd_f_ = std::clamp(ax_cmd_f_, -max_accel, max_accel);
ay_cmd_f_ = std::clamp(ay_cmd_f_, -max_accel, max_accel);

// 3) Relative airspeed (wind as ambient air velocity)
double vx_rel = state_.vx - vx_wind_;
double vy_rel = state_.vy - vy_wind_;

auto drag_axis = [&](double v_rel){
  double a_lin = -(c_lin / m) * v_rel;
  double a_quad = -(c_quad / m) * std::abs(v_rel) * v_rel;
  if (std::abs(v_rel) <= v_thr) return a_lin;          // linear region
  // smooth piecewise or blended:
  double w = std::clamp((std::abs(v_rel) - v_thr) / v_thr, 0.0, 1.0);
  return (1.0 - w) * a_lin + w * a_quad;               // simple blend
};

double ax_drag = drag_axis(vx_rel);
double ay_drag = drag_axis(vy_rel);

// 4) Total acceleration and integrate (semi-implicit)
double ax = ax_cmd_f_ + ax_drag; // wind enters via v_rel
double ay = ay_cmd_f_ + ay_drag;
state_.vx += ax * dt;
state_.vy += ay * dt;
state_.x  += state_.vx * dt;
state_.y  += state_.vy * dt;
```

## Test Harness and Metrics (ready today)
- Build and run the tester target: `dynamics_2d_tester`
- CSV saved to `simulation_outputs/dynamics_2d_test_<timestamp>.csv`
- Plot generator `analysis/plot_dynamics_2d.py` automatically finds the newest CSV and produces a 4-panel PNG with step-response metrics:
  - Overshoot [%], peak time [s], settling times at 2%/5% bands [s], rise time 10–90% [s]
- These metrics will remain compatible after the drag/wind upgrades since they operate on position/target time series.

## ROS 2 Readiness
- `dynamics_2d_lib` is a pure C++ library with stable state/step API; ROS 2 nodes can reuse it directly.
- When 3D is added, keep this class as the planar specialization and introduce a new `Quad6DOF` class for full translational + rotational dynamics; controllers can select the model at runtime.

## Roadmap (Physics only; controller unchanged)
1) Implement ambient wind velocity and piecewise linear–quadratic drag (configurable via YAML).
2) Add optional asymmetric actuator lag and soft-limiting near max accel (config).
3) Validate parameters against simple tests (drift under steady wind, max speed, step response).
4) Prepare 3D extension skeleton (gravity + thrust + attitude states), no controller changes yet.

---

## Türkçe Özet
Mevcut 2B model; kütle–sürükleme–itki (ivme komutu), birinci mertebe aktüatör gecikmesi, doyumlar ve basit rüzgâr bozucusunu içeriyor. Bu, üst-seviye takip ve kontrol karşılaştırmaları için kabul gören bir noktacık (kütle–sönüm) yaklaşımıdır. Bir sonraki adımda, gerçekçiliği artırmak için:
- Sürüklemeyi düşük hızda lineer, yüksek hızda kuadratik davranacak şekilde parça-bazlı (veya harmanlanmış) modele çekmek,
- Rüzgârı ivme yerine ortam hava hızı olarak tanımlayıp sürüklemeyi bağıl hız üzerinden hesaplamak,
- Aktüatör gecikmesini asimetrik (tau_up/tau_down) ve opsiyonel yumuşak doyumla zenginleştirmek,
- Sayısal entegrasyonu koruyup (semi-implicit Euler) küçük adım aralığıyla çalışmak,
önerilmektedir. Bu değişiklikler denetleyiciye dokunmadan fizik katmanını güçlendirir. Test hedefi/pozisyon adım cevabı grafik ve metrikleri (aşma, tepe zamanı, oturma süreleri, yükselme süresi) mevcut betiklerle aynı kalıp sonuçları karşılaştırmaya elverişli olacaktır. 3B’ye geçişte bu 2B sınıf korunarak yeni bir `Quad6DOF` sınıfı eklenmesi tavsiye edilir.
