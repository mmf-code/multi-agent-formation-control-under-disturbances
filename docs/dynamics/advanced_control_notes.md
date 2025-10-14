# Advanced Control Notes (Prefilter, Feed-forward, Closed-form Tuning)

This note complements `docs/dynamics/dynamics2d_system_and_control.md` with the latest additions used in experiments.

## Target Prefilter
- Two cascaded first-order filters towards the target position.
- YAML: `target_prefilter: {enabled: true, tau: 0.8}`
- Effect: reduces overshoot for step-like references with minimal impact on disturbance rejection.

## Model-based Feed-forward
- Cancels predicted drag and constant wind-acceleration bias.
- YAML:
```
feedforward:
  enable: true
  cancel_drag: true
  cancel_wind: true
  k_drag: 1.0
  k_wind: 1.0
```
- Effect: reduces controller workload and improves transients in wind/bias cases.

## Actuator-aware Closed-form PID (summary)
Per-axis plant: `P(s) = 1 / ( s^2 (tau s + 1) )`.
From specs (overshoot `Mp`, settling `Ts5`) compute `zeta` and `wn`:
`zeta = -ln(Mp)/sqrt(pi^2 + ln(Mp)^2)`, `wn ≈ 3/(zeta*Ts5)`.

Target polynomials and coefficient match (normalized by tau):
- PD target `(s^2 + 2*zeta*wn*s + wn^2)*(s + p_a)`:
  - `1/tau = 2*zeta*wn + p_a`
  - `Kd = tau*(wn^2 + 2*zeta*wn*p_a)`, `Kp = tau*(wn^2*p_a)`
- PID target `(s^2 + 2*zeta*wn*s + wn^2)*(s + p_a)*(s + p_i)`, with `p_i = beta*wn (beta~2..5)`:
  - `1/tau = 2*zeta*wn + p_a + p_i`
  - `Kd = tau*(wn^2 + 2*zeta*wn*(p_a + p_i) + p_a*p_i)`
  - `Kp = tau*(wn^2*(p_a + p_i) + 2*zeta*wn*p_a*p_i)`
  - `Ki = tau*(wn^2*p_a*p_i)`

Scripts implementing these mappings:
- `analysis/compute_pid_from_specs.py`
- `analysis/compute_pid_from_velocity.py`

## Scenario Matrix (for thesis)
- No-wind (reference)
- No-wind + bias (ax/ay): shows PID > PD and PID+FLS > PID clearly
- Wind (vx/vy)
- Wind + bias (robustness case): PID+FLS > PID > PD by IAE

Metrics: OS%, peak time, Ts5, RMSE, IAE/ITAE, |u|_1; for multi-drone also formation error metrics.

