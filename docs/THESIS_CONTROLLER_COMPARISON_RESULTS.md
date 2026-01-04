# Multi-Agent Formation Control: Controller Comparison Under Wind Disturbances

## Executive Summary

This document presents comprehensive experimental results comparing four controller types for multi-agent drone formation control under various wind disturbance scenarios. The study uses 12 Crazyflie 2.1 drones in a Gazebo simulation environment with ROS2 Humble.

### Controllers Tested
| Controller | Type | Description |
|------------|------|-------------|
| **PD** | Proportional-Derivative | Baseline controller without integral action |
| **PID** | Proportional-Integral-Derivative | Classical PID with anti-windup |
| **IT2-FLS** | Interval Type-2 Fuzzy Logic System | PID + IT2 fuzzy (additive: 100% PID + 50% fuzzy) |
| **GT2-FLS** | General Type-2 Fuzzy Logic System | PID + GT2 with 5 alpha-planes (additive mode) |

## Summary: Controller Performance Ranking (Latest Run: 2026-01-04)

| Phase | Best Overall | Best RMSE | Steady-State | Notes |
|-------|--------------|-----------|--------------|-------|
| 1. BASELINE | GT2 | 0.53m | - | No wind, GT2 precision advantage |
| 2. STEADY_WIND | IT2 | 0.95m | - | DC rejection |
| 3. TURBULENCE | IT2 | 0.93m | - | von Karman TI=0.15 |
| 4. GUST | **IT2** | **0.71m** | - | Best fuzzy advantage (22% vs PID) |
| 5. COMBINED | PID≈IT2≈GT2 | 0.99m | 0.71-0.73m | Marginal difference (~3%) |

**Key Findings**:
- **IT2-FLS** consistently outperforms in **structured wind** (Phase 2-4), best in gusts (22% improvement)
- **GT2-FLS** excels in **precision** (Phase 1) but over-amplifies noise in stochastic conditions
- **Phase 5**: In steady-state, all integral-based controllers perform **equivalently** (~0.72m)
- **Sensor noise** (10cm pos, 15cm/s vel) already included in tests

---

## Phase 1: BASELINE

**Objective**: No wind (reference performance)

**Conditions**:
- Wind Profile: constant
- Duration: 60 seconds

### Results (N=1 run) - Latest: 2026-01-04

| Controller | RMSE (m) | Std Dev | Peak Error | Last Violation | MAE |
|------------|----------|---------|------------|----------------|-----|
| **PD** | 1.073 | 0.102 | 3.029 | 60.1s | 1.008 |
| **PID** | 0.578 | 0.168 | 2.934 | 60.2s | 0.428 |
| **IT2** | 0.618 | 0.130 | 3.097 | 60.1s | 0.460 |
| **GT2** | 0.528 | 0.175 | 2.051 | 60.1s | 0.408 |

### Winner: GT2 (0.53m RMSE)

---

## Phase 2: STEADY_WIND

**Objective**: Constant 3 m/s @ 45 deg (DC rejection)

**Conditions**:
- Wind Profile: constant
- Wind Magnitude: 3.0 m/s
- Wind Direction: 45.0°
- Duration: 60 seconds

### Results (N=1 run) - Latest: 2026-01-04

| Controller | RMSE (m) | Std Dev | Peak Error | Last Violation | MAE |
|------------|----------|---------|------------|----------------|-----|
| **PD** | 1.824 | 0.036 | 4.079 | 60.7s | 1.786 |
| **PID** | 0.995 | 0.182 | 3.742 | 60.7s | 0.843 |
| **IT2** | 0.945 | 0.102 | 3.640 | 60.7s | 0.821 |
| **GT2** | 1.002 | 0.167 | 3.711 | 60.7s | 0.851 |

### Winner: IT2 (0.95m RMSE)

---

## Phase 3: TURBULENCE

**Objective**: Von Karman turbulence (stochastic wind variations)

**Conditions**:
- Wind Profile: vonkarman
- Wind Magnitude: 2.5 m/s
- Turbulence Intensity: 0.15
- Duration: 60 seconds

### Results (N=1 run) - Latest: 2026-01-04

| Controller | RMSE (m) | Std Dev | Peak Error | Last Violation | MAE |
|------------|----------|---------|------------|----------------|-----|
| **PD** | 1.827 | 0.015 | 3.134 | 60.2s | 1.804 |
| **PID** | 0.959 | 0.180 | 3.109 | 60.1s | 0.847 |
| **IT2** | 0.933 | 0.138 | 3.192 | 60.2s | 0.834 |
| **GT2** | 0.946 | 0.166 | 3.111 | 60.1s | 0.838 |

### Winner: IT2 (0.93m RMSE)

---

## Phase 4: GUST

**Objective**: Periodic gusts (transient response)

**Conditions**:
- Wind Profile: gust
- Wind Magnitude: 5.0 m/s
- Duration: 60 seconds

### Results (N=1 run) - Latest: 2026-01-04

| Controller | RMSE (m) | Std Dev | Peak Error | Last Violation | MAE |
|------------|----------|---------|------------|----------------|-----|
| **PD** | 1.187 | 0.032 | 2.943 | 60.3s | 1.120 |
| **PID** | 0.905 | 0.351 | 3.059 | 60.3s | 0.630 |
| **IT2** | 0.709 | 0.097 | 2.472 | 60.2s | 0.538 |
| **GT2** | 0.860 | 0.311 | 3.145 | 60.3s | 0.599 |

### Winner: IT2 (0.71m RMSE)

---

## Phase 5: COMBINED

**Objective**: Stochastic: turbulence + gusts + direction wander

**Conditions**:
- Wind Profile: stochastic
- Base Magnitude: 2.5 m/s (continuous)
- Gust Multiplier: 1.5x-2.0x
- Direction Wander: ±30° from base
- Duration: 60 seconds

### Results (N=1 run) - Latest: 2026-01-04

| Controller | RMSE (m) | Std Dev | Peak Error | Last Violation | MAE |
|------------|----------|---------|------------|----------------|-----|
| **PD** | 1.743 | 0.068 | 3.950 | 60.9s | 1.686 |
| **PID** | 0.994 | 0.070 | 3.716 | 60.9s | 0.816 |
| **IT2** | 1.007 | 0.112 | 3.846 | 60.9s | 0.812 |
| **GT2** | 1.087 | 0.049 | 4.372 | 60.9s | 0.854 |

### Winner: PID (marginal - see analysis)

### Steady-State Performance (Last 30s)

| Controller | Mean Error | Std Dev |
|------------|-----------|---------|
| **PID** | 0.712m | 0.391m |
| **IT2** | 0.733m | 0.417m |
| **GT2** | 0.734m | 0.408m |
| PD | 1.671m | 0.349m |

**Key Finding**: In steady-state, PID ≈ IT2 ≈ GT2 (difference only ~3%). The startup transient (~2.4m for all PID/IT2/GT2) inflates overall RMSE.

**Analysis**:
1. **Stochastic ≠ Structured Uncertainty**: The stochastic profile generates random noise, not structured uncertainty. Fuzzy controllers excel when uncertainty has patterns (model mismatch, sensor bias, etc.), not pure randomness.
2. **Integral Action Advantage**: PID's integral term effectively integrates out random disturbances over time.
3. **GT2 Sensitivity**: GT2's 43 large error events (>3m) vs PID's 21 suggests the secondary MF may over-amplify noise.
4. **Practical Equivalence**: For stochastic wind, any integral-based controller (PID, IT2, GT2) performs similarly.

**Scenarios Where Fuzzy Would Excel**:
- Sensor data dropout/corruption
- Model parameter uncertainty (mass changes, aerodynamic variations)
- Sudden step changes in disturbance
- Non-Gaussian disturbance distributions

---

## Experimental Setup

### Platform
- **Drones**: 12x Crazyflie 2.1 (27g, 92mm)
- **Simulation**: Gazebo 11 + ROS2 Humble
- **Control Rate**: 200 Hz
- **Physics Step**: 1 ms

### Controller Parameters
```yaml
PID Gains (all controllers):
  Kp: 3.501
  Ki: 1.946 (0 for PD)
  Kd: 3.608

Fuzzy Mixing (Additive Mode):
  k_pid: 1.0    # Full PID contribution
  k_fuzzy: 0.5  # Fuzzy adds on top
  # Formula: u = 1.0*u_pid + 0.5*u_fuzzy

Wind Input:
  fuzzy.include_wind: true
  fuzzy.wind_scalar: 1.0  # CRITICAL: Must be non-zero!

GT2 Specific:
  Alpha Levels: 5
  Secondary Shape: Triangular
  Secondary Spread: 0.3

Sensor Noise (enabled in world file):
  position_noise_std: 0.10 m   # 10cm position noise
  velocity_noise_std: 0.15 m/s # 15cm/s velocity noise
```

### Formation Configuration
- Shape: Triangle (3 drones per group)
- Spacing: 3.0 meters
- Groups: 4 (PD, PID, IT2, GT2)
- Y-Lane Separation: 8 meters

### Metric Definitions
| Metric | Definition | Formula |
|--------|------------|---------|
| RMSE | Root Mean Squared Error | sqrt(mean(e²)) |
| Peak Error | Maximum error magnitude | max(\|e\|) over mission |
| Last Violation | Last time error exceeded 0.1m threshold | max(t) where \|e\| > 0.1m |
| MAE | Mean Absolute Error | mean(\|e\|) |
| Control Effort | Integral of control magnitude | ∫\|u\| dt (IAE) |

**Note on Last Violation**: This metric uses a strict 0.1m threshold. Values near mission duration (~60s) indicate the controller never achieved sustained sub-threshold performance. This is NOT settling time in the classical control sense—it measures violation persistence, not transient response characteristics.


---

## Key Conclusions

### 1. IT2-FLS Shows Consistent Advantage in Wind Scenarios
- **Phases 2-4**: IT2 consistently outperforms PID and GT2
- Best improvement in **Phase 4 (GUST)**: IT2 achieves 0.71m vs PID's 0.91m (22% improvement)

### 2. GT2-FLS Excels in Precision, Not Robustness
- **Phase 1 (BASELINE)**: GT2 achieves best RMSE (0.53m) and lowest peak error (2.05m)
- **Phase 5 (COMBINED)**: GT2 has most large error events (43 vs PID's 21)
- GT2's secondary MF may over-amplify noise in stochastic conditions

### 3. Stochastic Wind ≠ Structured Uncertainty
- In **steady state**, PID ≈ IT2 ≈ GT2 (difference ~3%)
- Fuzzy excels with **structured uncertainty** (patterns, bias, model mismatch)
- Random noise is effectively filtered by PID's integral action

### 4. Sensor Noise Already Included
- Tests include 10cm position noise and 15cm/s velocity noise
- This level of noise doesn't differentiate fuzzy from PID

### Future Work
1. **Higher sensor noise** (position_noise_std > 0.3m) - simulate GPS degradation
2. **Sensor dropout** - simulate packet loss or communication failures
3. **Model uncertainty** - vary mass/drag parameters during flight
4. **Non-Gaussian disturbances** - bimodal wind patterns, sudden reversals

---

## Run History

| Date | Phases | Notes |
|------|--------|-------|
| 2026-01-04 19:00 | 5 | Deep data analysis - steady state comparison |
| 2026-01-04 18:40 | 5 | Phase 5 re-run with clean_sim.sh (corrected results) |
| 2026-01-04 18:30 | 1-4 | Latest run with clean_sim.sh |
| 2026-01-04 16:29 | 1-5 | Initial results (post wind_scalar fix) - Phase 5 had errors |

---

*Report Updated: 2026-01-04 19:10*
*Source: results/phase_X/run_1*
*Framework: 6-Phase Thesis Test Framework v1.0*
