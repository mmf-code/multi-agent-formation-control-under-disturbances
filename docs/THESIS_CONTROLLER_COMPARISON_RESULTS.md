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

### Key Findings (Updated 2026-01-04 - Post Wind Scalar Fix)

> **BUG FIX APPLIED (2026-01-04)**: `fuzzy.wind_scalar=1.0` now enabled.
> Results below reflect fixed configuration with wind input properly reaching fuzzy controllers.

- **Baseline**: PID/GT2 achieve best RMSE (0.60m) - all integral controllers perform similarly
- **Steady Wind**: IT2-FLS (0.94m) slightly outperforms GT2 (0.95m) and PID (0.95m)
- **Turbulence**: GT2-FLS (0.94m) provides best uncertainty handling
- **Gusts**: PD (1.83m) outperforms fuzzy controllers - fuzzy overcompensates on transients
- **Combined**: (Pending re-test)

---

## Phase 1: BASELINE (No Wind)

**Objective**: Establish reference performance without disturbances

**Conditions**:
- Wind: None (0 m/s)
- Duration: 60 seconds
- Formation: Triangle (3 drones per group)
- Trajectory: Waypoint-based (x: -5 → 0 → 5 → 5m)

### Results (Updated 2026-01-04 - Post Wind Scalar Fix)

| Controller | RMSE (m) | Std Dev | Notes |
|------------|----------|---------|-------|
| **PD** | 1.036 | 0.060 | Higher error without integral action |
| **PID** | **0.599** | 0.175 | Excellent tracking |
| **IT2-FLS** | 0.606 | 0.176 | Matches PID performance |
| **GT2-FLS** | **0.599** | 0.188 | Matches PID performance |

### Analysis
- **All integral controllers achieve similar excellent tracking** (~0.60m RMSE)
- PD shows 73% higher error (1.04m) without integral action to eliminate steady-state error
- Fuzzy controllers now perform identically to PID in calm conditions
- With wind_scalar=1.0, fuzzy systems are properly configured

### Winner: PID/GT2 (0.60m RMSE) - All integral controllers equal

---

## Phase 2: STEADY WIND (DC Disturbance Rejection)

**Objective**: Test constant wind disturbance rejection capability

**Conditions**:
- Wind: 1.5 m/s constant
- Duration: 60 seconds
- Challenge: Controllers must compensate for persistent force offset

### Results (Updated 2026-01-04 - Post Wind Scalar Fix)

| Controller | RMSE (m) | Std Dev | Notes |
|------------|----------|---------|-------|
| **PD** | 1.760 | 0.041 | 86% higher error than integral controllers |
| **PID** | 0.946 | 0.212 | Excellent disturbance rejection |
| **IT2-FLS** | **0.943** | 0.198 | Best - FOU adapts to steady offset |
| **GT2-FLS** | 0.947 | 0.212 | Slightly behind IT2 |

### Analysis
- **IT2-FLS achieves best performance** (0.943m) under steady wind
- All integral controllers show similar excellent performance (~0.94-0.95m)
- PD shows clear limitation without integral action for DC offset rejection
- Fuzzy wind input (wind_scalar=1.0) allows proper disturbance feedforward

### Winner: IT2-FLS (0.94m) - All integral controllers nearly equal

---

## Phase 3: TURBULENCE (Stochastic Wind Variations)

**Objective**: Test performance under stochastic wind turbulence

**Conditions**:
- Base Wind: 1.5 m/s with Gaussian variations
- Turbulence: Magnitude std = 0.5 m/s, Direction wander = 10°/s
- Duration: 60 seconds

### Results (Updated 2026-01-04 - Post Wind Scalar Fix)

| Controller | RMSE (m) | Std Dev | Notes |
|------------|----------|---------|-------|
| **PD** | 1.814 | 0.099 | 94% higher error than fuzzy |
| **PID** | 0.998 | 0.328 | Good but higher variance |
| **IT2-FLS** | 0.936 | 0.241 | FOU bounds handle stochastic uncertainty |
| **GT2-FLS** | **0.935** | 0.238 | Best - secondary MF smooths noise |

### Analysis
- **GT2-FLS achieves best performance** (0.935m) under turbulence
- IT2-FLS very close (0.936m) - both fuzzy controllers excel
- Fuzzy controllers show ~6% improvement over PID (0.998m)
- PD shows clear limitation (1.81m) without integral action
- Lower std dev for fuzzy indicates more consistent tracking

### Winner: GT2-FLS (0.94m) - Fuzzy controllers outperform PID

---

## Phase 4: GUST (Transient Response)

**Objective**: Test response to sudden wind gusts

**Conditions**:
- Base Wind: 1.5 m/s
- Gust Profile: Random gusts (5-15% probability)
- Gust Magnitude: 1.5-2.0x multiplier
- Gust Duration: 1-4 seconds

### Results (Updated 2026-01-04 - Post Wind Scalar Fix)

| Controller | RMSE (m) | Std Dev | Notes |
|------------|----------|---------|-------|
| **PD** | **1.828** | 0.555 | Best - no fuzzy overcompensation |
| **GT2-FLS** | 2.164 | 0.367 | Overcompensates during gusts |
| **PID** | 2.473 | 0.847 | Integral action amplifies gusts |
| **IT2-FLS** | 3.410 | 1.415 | Worst - fuzzy wind input overreacts |

### Analysis
- **CRITICAL FINDING**: Fuzzy controllers UNDERPERFORM during gusts
- **PD achieves best performance** (1.83m) - simpler is better for transients
- IT2-FLS shows 87% higher error than PD (3.41m vs 1.83m)
- GT2-FLS performs better than IT2 (2.16m) due to smoothing from secondary MF
- `fuzzy.wind_scalar=1.0` causes overcompensation during sudden gusts
- **Recommendation**: Reduce wind_scalar for gust-heavy environments, or add gust detection

### Winner: PD (1.83m) - Fuzzy overcompensation hurts transient response

---

## Phase 5: COMBINED (Stochastic Multi-Mode Disturbance)

**Objective**: Test under combined realistic conditions

**Conditions**:
- Base Wind: 1.5 m/s with direction wander (15°/s)
- Turbulence: Magnitude std = 0.5 m/s
- Random Gusts: 5-15% probability, 1.5-2.0x magnitude

### Results (2026-01-04)

> **DATA COLLECTION ISSUE**: Phase 5 failed to capture metrics data due to Gazebo
> simulation timing issues. Results pending re-test.

| Controller | RMSE (m) | Std Dev | Notes |
|------------|----------|---------|-------|
| **PD** | - | - | Pending |
| **PID** | - | - | Pending |
| **IT2-FLS** | - | - | Pending |
| **GT2-FLS** | - | - | Pending |

### Analysis
- Phase 5 test needs to be re-run with extended timeout
- Based on Phase 2-4 patterns, expected behavior:
  - Fuzzy controllers should excel in steady/turbulent components
  - PD may outperform during gust components due to overcompensation issue

### Winner: (Pending re-test)

---

## Summary: Controller Performance Ranking (Updated 2026-01-04)

| Phase | Best Overall | Best Fuzzy | RMSE (Best/Fuzzy) |
|-------|--------------|------------|-------------------|
| 1. BASELINE | PID/GT2 | GT2-FLS | 0.60 / 0.60 |
| 2. STEADY_WIND | IT2-FLS | IT2-FLS | 0.94 / 0.94 |
| 3. TURBULENCE | GT2-FLS | GT2-FLS | 0.94 / 0.94 |
| 4. GUST | **PD** | GT2-FLS | 1.83 / 2.16 |
| 5. COMBINED | (Pending) | (Pending) | - / - |

## Recommendations for Thesis

### Primary Conclusions (Updated 2026-01-04)
1. **Fuzzy controllers excel in steady-state and turbulent conditions** (Phases 2-3)
2. **GT2-FLS provides best turbulence handling** with secondary MF smoothing
3. **IT2-FLS slightly edges GT2 in steady wind** conditions
4. **CRITICAL: Fuzzy controllers UNDERPERFORM during gusts** due to wind_scalar overcompensation
5. **PD wins in gust conditions** - simpler control avoids overcompensation

### Controller Selection Guide
| Environment | Recommended Controller | Rationale |
|-------------|----------------------|-----------|
| Indoor/Calm | PID/IT2/GT2 | All perform equally well (~0.60m) |
| Outdoor/Steady Wind | IT2-FLS | Best steady-state compensation |
| Outdoor/Turbulence | GT2-FLS | Secondary MF handles stochastic noise |
| Outdoor/Gusty | **PD** | Fuzzy overcompensates - avoid |
| Mixed/Unknown | GT2-FLS (with reduced wind_scalar) | Best compromise |

### Known Issues
- **Gust Overcompensation**: `fuzzy.wind_scalar=1.0` causes fuzzy controllers to overreact to gusts
- **Potential Fix**: Implement adaptive wind_scalar or gust detection to reduce gain during transients

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

---

*Report Generated: 2026-01-03*
*Last Updated: 2026-01-04 (Post wind_scalar fix - Phase 1-4 results validated)*
*Framework: 6-Phase Thesis Test Framework v1.0*
*Note: Phase 5 pending re-test due to data collection issues*
