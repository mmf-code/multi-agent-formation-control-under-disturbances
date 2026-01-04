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

## Summary: Controller Performance Ranking

| Phase | Best Overall | Best RMSE | Notes |
|-------|--------------|-----------|-------|
| 1. BASELINE | GT2 | 0.58m | N=1 |
| 2. STEADY_WIND | PID | 0.89m | N=1 |
| 3. TURBULENCE | IT2 | 1.40m | N=1 |
| 4. GUST | IT2 | 0.81m | N=1 |
| 5. COMBINED | PID | 8.10m | N=1 |

---

## Phase 1: BASELINE

**Objective**: No wind (reference performance)

**Conditions**:
- Wind Profile: constant
- Duration: 60 seconds

### Results (N=1 run)

| Controller | RMSE (m) | Std Dev | Peak Error | Last Violation | MAE |
|------------|----------|---------|------------|----------------|-----|
| **PD** | 1.050 | 0.000 | 3.005 | 60.7s | 0.992 |
| **PID** | 0.680 | 0.000 | 3.117 | 60.7s | 0.480 |
| **IT2** | 0.621 | 0.000 | 3.060 | 60.8s | 0.455 |
| **GT2** | 0.578 | 0.000 | 2.969 | 60.7s | 0.449 |

### Winner: GT2 (0.58m RMSE)

---

## Phase 2: STEADY_WIND

**Objective**: Constant 3 m/s @ 45 deg (DC rejection)

**Conditions**:
- Wind Profile: constant
- Wind Magnitude: 3.0 m/s
- Wind Direction: 45.0°
- Duration: 60 seconds

### Results (N=1 run)

| Controller | RMSE (m) | Std Dev | Peak Error | Last Violation | MAE |
|------------|----------|---------|------------|----------------|-----|
| **PD** | 1.813 | 0.000 | 3.676 | 60.8s | 1.776 |
| **PID** | 0.892 | 0.000 | 3.222 | 60.8s | 0.804 |
| **IT2** | 1.003 | 0.000 | 3.771 | 60.8s | 0.850 |
| **GT2** | 1.012 | 0.000 | 4.058 | 60.8s | 0.834 |

### Winner: PID (0.89m RMSE)

---

## Phase 3: TURBULENCE

**Objective**: Von Karman turbulence (stochastic wind variations)

**Conditions**:
- Wind Profile: vonkarman
- Wind Magnitude: 2.5 m/s
- Turbulence Intensity: 0.15
- Duration: 60 seconds

### Results (N=1 run)

| Controller | RMSE (m) | Std Dev | Peak Error | Last Violation | MAE |
|------------|----------|---------|------------|----------------|-----|
| **PD** | 1.771 | 0.000 | 3.735 | 60.7s | 1.789 |
| **PID** | 1.462 | 0.000 | 4.448 | 60.7s | 1.361 |
| **IT2** | 1.398 | 0.000 | 4.297 | 60.7s | 1.407 |
| **GT2** | 1.410 | 0.000 | 4.493 | 60.7s | 1.350 |

### Winner: IT2 (1.40m RMSE)

---

## Phase 4: GUST

**Objective**: Periodic gusts (transient response)

**Conditions**:
- Wind Profile: gust
- Wind Magnitude: 5.0 m/s
- Duration: 60 seconds

### Results (N=1 run)

| Controller | RMSE (m) | Std Dev | Peak Error | Last Violation | MAE |
|------------|----------|---------|------------|----------------|-----|
| **PD** | 1.456 | 0.000 | 3.045 | 60.2s | 1.448 |
| **PID** | 0.821 | 0.000 | 3.079 | 60.2s | 0.723 |
| **IT2** | 0.811 | 0.000 | 3.018 | 60.2s | 0.701 |
| **GT2** | 0.838 | 0.000 | 3.024 | 60.2s | 0.708 |

### Winner: IT2 (0.81m RMSE)

---

## Phase 5: COMBINED

**Objective**: Stochastic: turbulence + gusts + direction wander

**Conditions**:
- Wind Profile: stochastic
- Wind Magnitude: 2.5 m/s
- Duration: 60 seconds

### Results (N=1 run)

| Controller | RMSE (m) | Std Dev | Peak Error | Last Violation | MAE |
|------------|----------|---------|------------|----------------|-----|
| **PD** | 9.083 | 0.000 | 13.342 | 60.1s | 7.051 |
| **PID** | 8.103 | 0.000 | 14.147 | 60.1s | 6.105 |
| **IT2** | 10.566 | 0.000 | 14.925 | 51.3s | 7.533 |
| **GT2** | 10.928 | 0.000 | 17.784 | 60.1s | 6.696 |

### Winner: PID (8.10m RMSE)

**Note**: The high RMSE values (8-11m) in this phase represent genuine agent dispersion under combined stochastic disturbances, not measurement error. The stochastic wind profile combines turbulence, gusts, and direction wander simultaneously, creating conditions that exceed the controllers' rejection bandwidth. This highlights a fundamental limitation of the current control architecture under extreme disturbance scenarios.

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

*Report Generated: 2026-01-04 14:46:44*
*Git Commit: c61e571*
*Source: results/new_metrics_test*
*Framework: 6-Phase Thesis Test Framework v1.0*
