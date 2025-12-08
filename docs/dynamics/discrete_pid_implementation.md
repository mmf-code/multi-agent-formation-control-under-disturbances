# Discrete PID Controller Implementation

This document describes the discrete-time PID controller implementation used in the multi-agent formation control system. The formulation follows standard control theory conventions suitable for thesis documentation.

## 1. Continuous-Time PID Controller

The ideal continuous-time PID controller is defined as:

$$
u(t) = K_p e(t) + K_i \int_0^t e(\tau) d\tau + K_d \frac{de(t)}{dt}
$$

Where:
- $e(t) = r(t) - y(t)$ is the error (reference - measurement)
- $K_p$ is the proportional gain
- $K_i$ is the integral gain
- $K_d$ is the derivative gain
- $u(t)$ is the control output

## 2. Discrete-Time Implementation

### 2.1 Sampling and Discretization

For digital implementation with sampling period $T_s$ (dt in code), we discretize each term:

**Proportional Term:**
$$
P[k] = K_p \cdot e[k]
$$

**Integral Term (Backward Euler / Rectangular Rule):**
$$
I[k] = I[k-1] + K_i \cdot e[k] \cdot T_s
$$

**Derivative Term (on Measurement):**
To avoid derivative kick when reference changes, we compute derivative on measurement:
$$
D[k] = -K_d \cdot \frac{y[k] - y[k-1]}{T_s}
$$

The negative sign accounts for $\frac{de}{dt} = \frac{dr}{dt} - \frac{dy}{dt}$, and assuming $\frac{dr}{dt} = 0$ (constant setpoint).

### 2.2 Complete Discrete PID Algorithm

```
e[k] = r[k] - y[k]                           // Error
P[k] = Kp * e[k]                              // Proportional
I[k] = I[k-1] + Ki * e[k] * Ts                // Integral (accumulated)
D[k] = -Kd * (y[k] - y[k-1]) / Ts             // Derivative on measurement
u[k] = P[k] + I[k] + D[k]                     // Total output
u[k] = clamp(u[k], u_min, u_max)              // Saturation
```

## 3. Derivative Filtering

High-frequency noise is amplified by the derivative term. A first-order low-pass filter is applied:

$$
D_f[k] = \alpha \cdot D_{raw}[k] + (1 - \alpha) \cdot D_f[k-1]
$$

Where:
- $\alpha \in (0, 1]$ is the filter coefficient
- $\alpha = 1$ means no filtering
- Smaller $\alpha$ provides more smoothing but adds phase lag
- Typical value: $\alpha = 0.1$

In the frequency domain, this corresponds to:
$$
H(z) = \frac{\alpha}{1 - (1-\alpha)z^{-1}}
$$

With cutoff frequency approximately:
$$
f_c \approx \frac{\alpha \cdot f_s}{2\pi}
$$

## 4. Anti-Windup Mechanisms

Integral windup occurs when the controller output saturates while the integrator continues to accumulate error. Three anti-windup strategies are implemented:

### 4.1 Conditional Integration

Stop integration when output is saturated and the error would increase saturation:

```
if (u_preliminary > u_max AND e > 0) OR (u_preliminary < u_min AND e < 0):
    do_not_integrate = true
```

### 4.2 Back-Calculation

Feed the saturation error back to the integrator with tracking time constant $T_t$:

$$
e_s = u_{sat} - u_{unsat}
$$

$$
I[k] = I[k-1] + \frac{T_s}{T_t} \cdot e_s
$$

Where:
- $e_s$ is the saturation error (clamped - unclamped output)
- $T_t$ is the tracking time constant

**Typical choice for $T_t$:**
$$
T_t = \sqrt{T_i \cdot T_d}
$$

Where $T_i = K_p / K_i$ (integral time) and $T_d = K_d / K_p$ (derivative time).

If $K_d = 0$ (PI controller), use $T_t = T_i$.

### 4.3 Combined (Default)

Both conditional integration and back-calculation are applied:
1. First, conditional integration prevents new error from accumulating during saturation
2. Then, back-calculation corrects any residual windup

## 5. Implementation Parameters

| Parameter | Symbol | Default | Description |
|-----------|--------|---------|-------------|
| Proportional Gain | $K_p$ | 0.538 | Position error gain |
| Integral Gain | $K_i$ | 0.145 | Steady-state error elimination |
| Derivative Gain | $K_d$ | 1.368 | Damping / rate response |
| Sample Time | $T_s$ | 0.005 s | 200 Hz control loop |
| Filter Alpha | $\alpha$ | 0.1 | Derivative filter coefficient |
| Output Min | $u_{min}$ | -10.0 m/s² | Minimum acceleration |
| Output Max | $u_{max}$ | 10.0 m/s² | Maximum acceleration |

## 6. Closed-Loop Analysis

For a double-integrator plant (position control):
$$
G_p(s) = \frac{1}{s^2}
$$

With PID controller (assuming ideal, no filtering):
$$
G_c(s) = K_p + \frac{K_i}{s} + K_d s
$$

The closed-loop transfer function becomes:
$$
G_{cl}(s) = \frac{K_d s^2 + K_p s + K_i}{s^3 + K_d s^2 + K_p s + K_i}
$$

**Design considerations:**
- PD control ($K_i = 0$) gives 2nd-order closed-loop (no steady-state error for step disturbance, but has for constant disturbance bias)
- PID control gives 3rd-order closed-loop with zero steady-state error
- Dominant poles determine settling time and overshoot

## 7. Code Reference

The implementation is in:
- Header: `agent_control_pkg/include/agent_control_pkg/pid_controller.hpp`
- Source: `agent_control_pkg/src/pid_controller.cpp`
- Adapter: `agent_control_pkg/include/agent_control_pkg/controllers/pid_adapter.hpp`

Key method: `PIDController::calculate_with_terms(double current_value, double dt)`

## 8. References

1. Astrom, K.J., & Hagglund, T. (2006). *Advanced PID Control*. ISA.
2. Franklin, G.F., Powell, J.D., & Emami-Naeini, A. (2015). *Feedback Control of Dynamic Systems* (7th ed.). Pearson.
3. Ogata, K. (2010). *Modern Control Engineering* (5th ed.). Prentice Hall.
