# Closed-Loop Analysis for Double-Integrator Plant

This document provides detailed closed-loop analysis for P, PD, PI, and PID controllers applied to the double-integrator drone plant model. All derivations are suitable for thesis inclusion.

## 1. Plant Model

The drone position dynamics (per axis) are represented as:

$$
G_p(s) = \frac{1}{s^2}
$$

This is a **Type-2 system** with two poles at the origin.

## 2. P Controller Analysis

### 2.1 Controller and Closed-Loop

**Controller:**
$$
C_P(s) = K_p
$$

**Open-Loop Transfer Function:**
$$
L(s) = K_p \cdot \frac{1}{s^2} = \frac{K_p}{s^2}
$$

**Closed-Loop Transfer Function:**
$$
T(s) = \frac{L(s)}{1 + L(s)} = \frac{K_p}{s^2 + K_p}
$$

### 2.2 Characteristic Equation

$$
s^2 + K_p = 0
$$

Roots: $s_{1,2} = \pm j\sqrt{K_p}$

**Result:** Pure imaginary poles → **marginally stable** (sustained oscillation)

### 2.3 Conclusion

P control alone on a double-integrator is **not acceptable** - the system will oscillate indefinitely without damping.

---

## 3. PD Controller Analysis

### 3.1 Controller and Closed-Loop

**Controller:**
$$
C_{PD}(s) = K_p + K_d s
$$

**Open-Loop Transfer Function:**
$$
L(s) = (K_p + K_d s) \cdot \frac{1}{s^2} = \frac{K_d s + K_p}{s^2}
$$

**Closed-Loop Transfer Function:**
$$
T(s) = \frac{K_d s + K_p}{s^2 + K_d s + K_p}
$$

### 3.2 Characteristic Equation

$$
s^2 + K_d s + K_p = 0
$$

This matches the standard second-order form:
$$
s^2 + 2\zeta\omega_n s + \omega_n^2 = 0
$$

### 3.3 Parameter Mapping

$$
\boxed{\omega_n = \sqrt{K_p}}
$$

$$
\boxed{\zeta = \frac{K_d}{2\sqrt{K_p}} = \frac{K_d}{2\omega_n}}
$$

**Inverse mapping (design → gains):**
$$
K_p = \omega_n^2
$$
$$
K_d = 2\zeta\omega_n
$$

### 3.4 Stability Condition

For stability (poles in LHP): $K_p > 0$ and $K_d > 0$

For desired damping:
- Underdamped ($\zeta < 1$): Complex conjugate poles, oscillatory response
- Critically damped ($\zeta = 1$): Fastest non-oscillatory response
- Overdamped ($\zeta > 1$): Slower, no oscillation

### 3.5 Time-Domain Specifications

For underdamped response ($\zeta < 1$):

**Peak Time:**
$$
t_p = \frac{\pi}{\omega_n\sqrt{1-\zeta^2}} = \frac{\pi}{\omega_d}
$$

**Percent Overshoot:**
$$
M_p = e^{-\frac{\zeta\pi}{\sqrt{1-\zeta^2}}} \times 100\%
$$

**Settling Time (2%):**
$$
t_s \approx \frac{4}{\zeta\omega_n}
$$

**Settling Time (5%):**
$$
t_s \approx \frac{3}{\zeta\omega_n}
$$

**Rise Time (10-90%):**
$$
t_r \approx \frac{1.8}{\omega_n}
$$

### 3.6 Design from Specifications

Given desired overshoot $M_p$ and settling time $t_s$:

**Step 1: Calculate damping ratio from overshoot**
$$
\zeta = \sqrt{\frac{(\ln M_p)^2}{\pi^2 + (\ln M_p)^2}}
$$

**Step 2: Calculate natural frequency from settling time**
$$
\omega_n = \frac{3}{\zeta \cdot t_s} \quad \text{(5\% criterion)}
$$
$$
\omega_n = \frac{4}{\zeta \cdot t_s} \quad \text{(2\% criterion)}
$$

**Step 3: Calculate controller gains**
$$
K_p = \omega_n^2, \quad K_d = 2\zeta\omega_n
$$

### 3.7 Disturbance Rejection (PD Limitation)

For a constant disturbance $D(s) = d_0/s$:

$$
E(s) = \frac{1}{1 + L(s)} \cdot \frac{d_0}{s} = \frac{s^2}{s^2 + K_d s + K_p} \cdot \frac{d_0}{s}
$$

Steady-state error (Final Value Theorem):
$$
e_{ss} = \lim_{s \to 0} s \cdot E(s) = \lim_{s \to 0} \frac{s \cdot d_0}{s^2 + K_d s + K_p} = \frac{d_0}{K_p}
$$

**Result:** PD control has **non-zero steady-state error** for constant disturbances (wind).

---

## 4. PI Controller Analysis

### 4.1 Controller and Closed-Loop

**Controller:**
$$
C_{PI}(s) = K_p + \frac{K_i}{s} = \frac{K_p s + K_i}{s}
$$

**Open-Loop Transfer Function:**
$$
L(s) = \frac{K_p s + K_i}{s} \cdot \frac{1}{s^2} = \frac{K_p s + K_i}{s^3}
$$

**Closed-Loop Transfer Function:**
$$
T(s) = \frac{K_p s + K_i}{s^3 + K_p s + K_i}
$$

### 4.2 Characteristic Equation

$$
s^3 + K_p s + K_i = 0
$$

### 4.3 Stability Analysis (Routh-Hurwitz)

Routh array for $s^3 + 0 \cdot s^2 + K_p s + K_i$:

| $s^3$ | 1 | $K_p$ |
|-------|---|-------|
| $s^2$ | 0 | $K_i$ |
| $s^1$ | ? | 0 |
| $s^0$ | $K_i$ | |

The zero in the $s^2$ row indicates that the system is **unstable** or marginally stable regardless of $K_p$ and $K_i$ values.

**Result:** PI control alone on a double-integrator is **unstable** - derivative action is required.

---

## 5. PID Controller Analysis

### 5.1 Controller and Closed-Loop

**Controller:**
$$
C_{PID}(s) = K_p + \frac{K_i}{s} + K_d s = \frac{K_d s^2 + K_p s + K_i}{s}
$$

**Open-Loop Transfer Function:**
$$
L(s) = \frac{K_d s^2 + K_p s + K_i}{s} \cdot \frac{1}{s^2} = \frac{K_d s^2 + K_p s + K_i}{s^3}
$$

**Closed-Loop Transfer Function:**
$$
T(s) = \frac{K_d s^2 + K_p s + K_i}{s^3 + K_d s^2 + K_p s + K_i}
$$

### 5.2 Characteristic Equation

$$
s^3 + K_d s^2 + K_p s + K_i = 0
$$

### 5.3 Stability Analysis (Routh-Hurwitz)

Routh array:

| $s^3$ | 1 | $K_p$ |
|-------|---|-------|
| $s^2$ | $K_d$ | $K_i$ |
| $s^1$ | $\frac{K_d K_p - K_i}{K_d}$ | 0 |
| $s^0$ | $K_i$ | |

**Stability conditions:**
1. $K_d > 0$
2. $K_i > 0$
3. $K_d K_p > K_i$ (most restrictive)

### 5.4 Pole Placement Design

Design the closed-loop to have:
- A dominant second-order response with $\omega_n$ and $\zeta$
- A fast real pole at $s = -p_i$ (integral pole)

Target characteristic polynomial:
$$
(s^2 + 2\zeta\omega_n s + \omega_n^2)(s + p_i) = 0
$$

Expanding:
$$
s^3 + (2\zeta\omega_n + p_i)s^2 + (\omega_n^2 + 2\zeta\omega_n p_i)s + \omega_n^2 p_i = 0
$$

Matching coefficients with $s^3 + K_d s^2 + K_p s + K_i = 0$:

$$
\boxed{K_d = 2\zeta\omega_n + p_i}
$$

$$
\boxed{K_p = \omega_n^2 + 2\zeta\omega_n p_i}
$$

$$
\boxed{K_i = \omega_n^2 p_i}
$$

### 5.5 Integral Pole Selection

The integral pole $p_i$ affects:
- **Larger $p_i$:** Faster disturbance rejection, but more aggressive integral action (risk of overshoot/windup)
- **Smaller $p_i$:** Slower rejection, gentler response

**Recommended:** $p_i = \alpha \cdot \omega_n$ with $\alpha \in [2, 5]$

For $\alpha = 2.5$: The integral pole is 2.5 times faster than the dominant poles, ensuring it doesn't significantly affect the step response shape.

### 5.6 Disturbance Rejection (PID Advantage)

For a constant disturbance $D(s) = d_0/s$:

$$
E(s) = \frac{1}{1 + L(s)} \cdot \frac{d_0}{s} = \frac{s^3}{s^3 + K_d s^2 + K_p s + K_i} \cdot \frac{d_0}{s}
$$

Steady-state error:
$$
e_{ss} = \lim_{s \to 0} s \cdot E(s) = \lim_{s \to 0} \frac{s^2 \cdot d_0}{s^3 + K_d s^2 + K_p s + K_i} = 0
$$

**Result:** PID control provides **zero steady-state error** for constant disturbances.

---

## 6. Summary: Controller Comparison

| Controller | Char. Eq. Order | Stability | SS Error (step) | SS Error (dist.) |
|------------|-----------------|-----------|-----------------|------------------|
| P | 2nd | Marginal | 0 | $\infty$ |
| PD | 2nd | Stable ($K_p, K_d > 0$) | 0 | $d_0/K_p$ |
| PI | 3rd | Unstable | - | - |
| PID | 3rd | Stable (conditions) | 0 | 0 |

## 7. Design Example: Current System Parameters

### 7.1 Specifications

- Target overshoot: $M_p = 15\%$
- Target settling time (5%): $t_s = 15$ s
- Integral pole factor: $\alpha = 2.5$

### 7.2 Calculations

**Step 1: Damping ratio**
$$
\zeta = \sqrt{\frac{(\ln 0.15)^2}{\pi^2 + (\ln 0.15)^2}} = \sqrt{\frac{3.59}{9.87 + 3.59}} = 0.516
$$

**Step 2: Natural frequency**
$$
\omega_n = \frac{3}{\zeta \cdot t_s} = \frac{3}{0.516 \times 15} = 0.388 \text{ rad/s}
$$

**Step 3: Integral pole**
$$
p_i = \alpha \cdot \omega_n = 2.5 \times 0.388 = 0.969 \text{ rad/s}
$$

**Step 4: PID gains**
$$
K_d = 2 \times 0.516 \times 0.388 + 0.969 = 1.368
$$
$$
K_p = 0.388^2 + 2 \times 0.516 \times 0.388 \times 0.969 = 0.538
$$
$$
K_i = 0.388^2 \times 0.969 = 0.145
$$

### 7.3 Summary

| Parameter | Value | Unit |
|-----------|-------|------|
| $\zeta$ | 0.516 | - |
| $\omega_n$ | 0.388 | rad/s |
| $p_i$ | 0.969 | rad/s |
| $K_p$ | 0.538 | - |
| $K_i$ | 0.145 | - |
| $K_d$ | 1.368 | - |

These are the values currently configured in `agent_controller_default.yaml`.

---

## 8. PD vs PID Selection Criteria

| Criterion | PD | PID |
|-----------|-----|------|
| Steady-state error (no disturbance) | Zero | Zero |
| Steady-state error (const. disturbance) | Non-zero | Zero |
| Overshoot control | Good | Moderate |
| Implementation complexity | Low | Medium |
| Tuning difficulty | Easy (2 params) | Medium (3 params) |
| Anti-windup needed | No | Yes |

**Recommendation:**
- Use **PD** for calm conditions or when overshoot is critical
- Use **PID** when constant disturbances (wind) must be rejected
- Use **PID+Fuzzy** for adaptive disturbance compensation

---

## 9. Frequency Domain Verification

### 9.1 Open-Loop Bode (PID on Double-Integrator)

At low frequency ($\omega \to 0$):
- Plant contributes -40 dB/decade, -180° phase
- PID integral adds -20 dB/decade, -90° phase
- Net: -60 dB/decade slope at low frequency

At crossover frequency ($\omega_c \approx \omega_n$):
- PID zero at $z = K_i/K_p$ adds phase lead
- Derivative action $K_d s$ adds up to +90° phase

**Phase margin:** Typically 45-60° for the designed parameters.

### 9.2 Gain Margin

The system has infinite gain margin because the phase never crosses -180° at high frequencies (derivative action).

---

## 10. Implementation Notes

1. **Derivative on Measurement:** Compute $D = -K_d \dot{y}$ instead of $D = K_d \dot{e}$ to avoid derivative kick on setpoint changes.

2. **Derivative Filter:** Apply low-pass filter to reduce noise amplification:
   $$D_f[k] = \alpha \cdot D_{raw}[k] + (1-\alpha) \cdot D_f[k-1]$$

3. **Anti-Windup:** Essential for PID to prevent integral saturation. Implemented strategies:
   - Conditional integration
   - Back-calculation
   - Combined (both)

4. **Discrete-Time:** See `discrete_pid_implementation.md` for detailed discretization.

---

## 11. References

1. Franklin, G.F., Powell, J.D., & Emami-Naeini, A. (2015). *Feedback Control of Dynamic Systems* (7th ed.). Pearson.
2. Ogata, K. (2010). *Modern Control Engineering* (5th ed.). Prentice Hall.
3. Astrom, K.J., & Hagglund, T. (2006). *Advanced PID Control*. ISA.
4. Dorf, R.C., & Bishop, R.H. (2017). *Modern Control Systems* (13th ed.). Pearson.

---

**Document Version:** 1.0
**Last Updated:** 2024-12-09
**Author:** Control Systems Research Team
