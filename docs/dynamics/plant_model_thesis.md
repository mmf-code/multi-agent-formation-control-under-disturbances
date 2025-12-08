# Plant Model Documentation (Thesis Format)

This document provides the formal mathematical description of the drone plant model used in this thesis, suitable for direct inclusion in academic writing.

## 1. System Overview

The multi-agent formation control system operates on a fleet of quadrotor drones performing 2D planar position control. Each drone is modeled as a point mass with translational dynamics, where the control input is desired acceleration and the output is position.

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                        CONTROL SYSTEM HIERARCHY                              │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│   Formation      ┌─────────────┐     ┌─────────────┐     ┌─────────────┐   │
│   Coordinator    │   Target    │     │   Agent     │     │   Drone     │   │
│   (PSO)     ───▶ │   Poses     │ ──▶ │ Controller  │ ──▶ │   Plant     │   │
│                  └─────────────┘     │ (PD/PID/    │     │ (Double     │   │
│                                      │  Fuzzy)     │     │ Integrator) │   │
│                                      └─────────────┘     └─────────────┘   │
│                                            │                    │           │
│                                            │   a_cmd           │  p, v     │
│                                            ▼                    ▼           │
│                                      ┌─────────────────────────────────┐   │
│                                      │        Feedback Loop            │   │
│                                      └─────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────────┘
```

## 2. Translational Dynamics (Double Integrator)

### 2.1 Continuous-Time Model

The drone position dynamics are modeled as a double-integrator with disturbance:

$$
\ddot{p}(t) = a_{cmd}(t) + d(t)
$$

where:
- $p(t) \in \mathbb{R}^2$ is the position vector $[x, y]^T$
- $a_{cmd}(t) \in \mathbb{R}^2$ is the commanded acceleration
- $d(t) \in \mathbb{R}^2$ is the disturbance (wind, modeling error)

In state-space form with state $\mathbf{x} = [p, \dot{p}]^T = [x, y, v_x, v_y]^T$:

$$
\dot{\mathbf{x}} = \begin{bmatrix} 0 & I_2 \\ 0 & 0 \end{bmatrix} \mathbf{x} + \begin{bmatrix} 0 \\ I_2 \end{bmatrix} (a_{cmd} + d)
$$

### 2.2 Transfer Function Representation

From acceleration input to position output (per axis):

$$
G_p(s) = \frac{P(s)}{A_{cmd}(s)} = \frac{1}{s^2}
$$

**System Type Analysis:**
- This is a **Type-2 system** (two poles at the origin)
- Zero steady-state error for step position input with P control
- Zero steady-state error for ramp input with PI control
- **Integral term required for constant disturbance rejection**

### 2.3 Physical Interpretation

The double-integrator model assumes:

1. **Fast Inner Loop:** The attitude/thrust control (inner loop) is sufficiently fast that we can directly command acceleration
2. **Negligible Rotational Dynamics:** Roll/pitch transients settle faster than position dynamics
3. **Hover Approximation:** Small angle assumption is valid for moderate accelerations

For the Crazyflie with 500 Hz attitude control, this approximation holds well for position control bandwidth < 5 Hz.

## 3. Enhanced Plant Model (Gazebo Simulation)

### 3.1 Complete Dynamics with Drag and Actuator Lag

The Gazebo simulation uses an enhanced model:

$$
\ddot{p} = a_{cmd,f} + a_{drag} + a_{wind}
$$

where:

**Actuator Dynamics (First-Order Lag):**
$$
\dot{a}_{cmd,f} = \frac{1}{\tau_{act}} (a_{cmd} - a_{cmd,f})
$$

**Aerodynamic Drag (Piecewise Linear-Quadratic):**
$$
a_{drag} = \begin{cases}
-\frac{c_{lin}}{m} v_{rel} & |v_{rel}| \leq v_{thr} \\
-\frac{c_{lin}}{m} v_{thr} \cdot \text{sign}(v_{rel}) - \frac{c_{quad}}{m} (|v_{rel}| - v_{thr}) v_{rel} & |v_{rel}| > v_{thr}
\end{cases}
$$

where $v_{rel} = v - v_{wind}$ is the relative airspeed.

### 3.2 Transfer Function with Actuator Lag

Including the first-order actuator dynamics:

$$
G_{eff}(s) = \frac{1}{s^2} \cdot \frac{1}{\tau_{act} s + 1} = \frac{1}{s^2 (\tau_{act} s + 1)}
$$

With $\tau_{act} \approx 0.02$ s (20 ms), the actuator pole is at $s = -50$ rad/s, which is much faster than the position control bandwidth ($\omega_n \approx 0.4$ rad/s), so the simpler double-integrator model is valid for control design.

## 4. Disturbance Model

### 4.1 Wind Disturbance

Wind is modeled as an ambient air velocity field:

$$
v_{wind}(t) = [v_{wx}(t), v_{wy}(t)]^T
$$

For constant wind bias:
$$
d_{wind}(t) = -\frac{c_{lin}}{m} v_{wind} = \text{constant}
$$

This creates a **constant disturbance** that requires integral action to reject.

### 4.2 Disturbance Categories

| Disturbance Type | Model | Rejection Requirement |
|------------------|-------|----------------------|
| Constant wind | $d(t) = d_0$ | Integral action (PID) |
| Gust (step) | $d(t) = d_0 \cdot u(t-t_0)$ | Fast response + integral |
| Turbulence | $d(t) = \text{filtered noise}$ | Robust controller design |

## 5. System Parameters

### 5.1 Gazebo Custom Drone

| Parameter | Symbol | Value | Unit |
|-----------|--------|-------|------|
| Mass | $m$ | 1.5 | kg |
| Linear drag coefficient | $c_{lin}$ | 0.12 | kg/s |
| Quadratic drag coefficient | $c_{quad}$ | 0.05 | kg/m |
| Drag speed threshold | $v_{thr}$ | 1.0 | m/s |
| Actuator time constant | $\tau_{act}$ | 0.02 | s |
| Max acceleration | $a_{max}$ | 10.0 | m/s² |

### 5.2 Crazyflie 2.1

| Parameter | Symbol | Value | Unit |
|-----------|--------|-------|------|
| Mass | $m$ | 0.027 | kg |
| Attitude loop bandwidth | - | ~25 | Hz |
| Attitude loop settling | $\tau_{att}$ | ~0.02 | s |
| Max thrust | $T_{max}$ | 0.6 | N |
| Max acceleration (theoretical) | $a_{max}$ | 20.3 | m/s² |
| Max acceleration (safe limit) | $a_{safe}$ | 5.0 | m/s² |

## 6. Effective Plant for Controller Design

### 6.1 Simplified Model

For controller design, we use the simplified double-integrator:

$$
G(s) = \frac{1}{s^2}
$$

This is justified because:
1. Actuator dynamics are 10x faster than position bandwidth
2. Drag provides natural damping (improves stability margin)
3. Controller already includes derivative term for damping

### 6.2 Design Implications

For a double-integrator plant $G(s) = 1/s^2$:

**With PD Controller** $C_{PD}(s) = K_p + K_d s$:
$$
T_{PD}(s) = \frac{K_d s + K_p}{s^2 + K_d s + K_p}
$$

Characteristic equation: $s^2 + 2\zeta\omega_n s + \omega_n^2 = 0$

Parameter mapping:
$$
\omega_n = \sqrt{K_p}, \quad \zeta = \frac{K_d}{2\sqrt{K_p}}
$$

**With PID Controller** $C_{PID}(s) = K_p + K_i/s + K_d s$:
$$
T_{PID}(s) = \frac{K_d s^2 + K_p s + K_i}{s^3 + K_d s^2 + K_p s + K_i}
$$

This is a 3rd-order system with one additional pole from integral action.

## 7. Frequency Domain Analysis

### 7.1 Open-Loop Bode Plot

For the double-integrator:
- Magnitude: $|G(j\omega)| = 1/\omega^2$ → -40 dB/decade slope
- Phase: $\angle G(j\omega) = -180°$ (constant)

This means the plant starts with -180° phase, so the controller must add phase lead (derivative action) for stability.

### 7.2 Crossover Frequency Design

Target crossover frequency: $\omega_c \approx 0.5$ rad/s

At crossover, we want:
- $|G(j\omega_c) \cdot C(j\omega_c)| = 1$ (0 dB)
- Phase margin > 45° for robustness

The PD controller provides up to +90° phase lead, which combined with the -180° plant phase gives adequate margin.

## 8. Summary Table

| Property | Value/Expression |
|----------|------------------|
| Plant transfer function | $G(s) = 1/s^2$ |
| System type | Type-2 |
| Steady-state error (step, P ctrl) | 0 |
| Steady-state error (const. dist., PD) | Non-zero |
| Steady-state error (const. dist., PID) | 0 |
| Stability requirement | $K_d > 0$ (phase lead) |
| Design DOF (PD) | 2 ($\omega_n$, $\zeta$) |
| Design DOF (PID) | 3 ($\omega_n$, $\zeta$, $p_i$) |

## 9. References

1. Franklin, G.F., Powell, J.D., & Emami-Naeini, A. (2015). *Feedback Control of Dynamic Systems* (7th ed.). Pearson.
2. Ogata, K. (2010). *Modern Control Engineering* (5th ed.). Prentice Hall.
3. Astrom, K.J., & Murray, R.M. (2008). *Feedback Systems: An Introduction for Scientists and Engineers*. Princeton University Press.

---

**Document Version:** 1.0
**Last Updated:** 2024-12-09
**Author:** Control Systems Research Team
