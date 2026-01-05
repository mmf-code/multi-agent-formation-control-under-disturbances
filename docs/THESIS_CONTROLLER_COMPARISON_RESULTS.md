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
| 2. STEADY_WIND | PID | 0.88m | N=1 |
| 3. TURBULENCE | IT2 | 0.88m | N=1 |

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
| **PD** | 1.683 | 0.000 | 3.105 | 60.2s | 1.657 |
| **PID** | 0.876 | 0.000 | 3.027 | 60.2s | 0.783 |
| **IT2** | 0.882 | 0.000 | 3.013 | 60.2s | 0.802 |
| **GT2** | 0.878 | 0.000 | 3.056 | 60.1s | 0.793 |

### Winner: PID (0.88m RMSE)

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
| **PD** | 1.773 | 0.000 | 3.258 | 60.3s | 1.730 |
| **PID** | 0.926 | 0.000 | 2.961 | 60.3s | 0.829 |
| **IT2** | 0.880 | 0.000 | 2.884 | 60.3s | 0.788 |
| **GT2** | 0.912 | 0.000 | 3.060 | 60.3s | 0.820 |

### Winner: IT2 (0.88m RMSE)

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

*Report Generated: 2026-01-05 15:02:32*
*Git Commit: b05ba94*
*Source: results/etc_fix_test*
*Framework: 6-Phase Thesis Test Framework v1.0*
