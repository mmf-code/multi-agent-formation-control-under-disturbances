# Phased Test Framework Results

*Generated: 2026-01-03 16:46:37*

## Overview

This report summarizes the 6-phase controller comparison test results.

**Controllers Tested:**
- **PD**: Proportional-Derivative (baseline)
- **PID**: Proportional-Integral-Derivative
- **IT2**: PID + Interval Type-2 Fuzzy Logic System
- **GT2**: PID + General Type-2 Fuzzy Logic System

## Test Phases

| Phase | Name | Purpose | Duration |
|-------|------|---------|----------|
| 1 | BASELINE | Reference (no wind) | 60s |
| 2 | STEADY_WIND | DC rejection (3 m/s @ 45 deg) | 60s |
| 3 | TURBULENCE | Von Karman TI sweep [0.15, 0.25, 0.35] | 60s |
| 4 | GUST | Periodic gusts (5 m/s, 1.5s, 10s interval) | 60s |
| 5 | COMBINED | Stochastic: turbulence + gusts + wander | 60s |
| 6 | ENDURANCE | Long-run stability | 300s |

## Results by Phase

### Phase 1: BASELINE

| Controller | RMSE (m) | ITAE | Settling (s) | Overshoot (m) | vs PD |
|------------|----------|------|--------------|---------------|-------|
| PD | 1.2546 +- 0.0000 | 209.90 +- 0.00 | 11.20 | 1.9481 | baseline |
| PID | 0.7922 +- 0.0000 | 142.03 +- 0.00 | 11.20 | 1.7602 | +36.9% |
| IT2 | 1.0997 +- 0.0000 | 150.10 +- 0.00 | 11.20 | 2.1572 | +12.3% |
| GT2 | 0.9867 +- 0.0000 | 130.65 +- 0.00 | 11.20 | 1.6246 | +21.4% |

### Phase 2: STEADY_WIND

| Controller | RMSE (m) | ITAE | Settling (s) | Overshoot (m) | vs PD |
|------------|----------|------|--------------|---------------|-------|
| PD | 21.5792 +- 0.0000 | 255.60 +- 0.00 | 3.50 | 20.1508 | baseline |
| PID | 2.3683 +- 0.0000 | 0.00 +- 0.00 | 0.00 | 0.0000 | +89.0% |
| IT2 | 3.1185 +- 0.0000 | 0.03 +- 0.00 | 0.05 | 0.0000 | +85.5% |
| GT2 | 2.3348 +- 0.0000 | 0.02 +- 0.00 | 0.05 | 0.0000 | +89.2% |

### Phase 3: TURBULENCE

| Controller | RMSE (m) | ITAE | Settling (s) | Overshoot (m) | vs PD |
|------------|----------|------|--------------|---------------|-------|
| PD | 12.2320 +- 0.0000 | 2636.95 +- 0.00 | 18.30 | 11.8927 | baseline |
| PID | 7.2895 +- 0.0000 | 0.10 +- 0.00 | 0.10 | 1.8107 | +40.4% |
| IT2 | 1.1984 +- 0.0000 | 0.00 +- 0.00 | 0.00 | 0.0000 | +90.2% |
| GT2 | 9.4421 +- 0.0000 | 0.07 +- 0.00 | 0.07 | 0.0000 | +22.8% |

### Phase 4: GUST

| Controller | RMSE (m) | ITAE | Settling (s) | Overshoot (m) | vs PD |
|------------|----------|------|--------------|---------------|-------|
| PD | 9.5507 +- 0.0000 | 0.01 +- 0.00 | 0.03 | 0.0000 | baseline |
| PID | 5.2871 +- 0.0000 | 17.85 +- 0.00 | 2.13 | 6.4856 | +44.6% |
| IT2 | 13.3014 +- 0.0000 | 0.51 +- 0.00 | 0.10 | 0.2333 | -39.3% |
| GT2 | 31.7502 +- 0.0000 | 0.00 +- 0.00 | 0.00 | 22.8471 | -232.4% |

### Phase 5: COMBINED

| Controller | RMSE (m) | ITAE | Settling (s) | Overshoot (m) | vs PD |
|------------|----------|------|--------------|---------------|-------|
| PD | 131.5495 +- 0.0000 | 0.00 +- 0.00 | 0.00 | 70.9768 | baseline |
| PID | 64.9325 +- 0.0000 | 3376.02 +- 0.00 | 3.73 | 70.3300 | +50.6% |
| IT2 | 245.4277 +- 0.0000 | 45.33 +- 0.00 | 0.43 | 193.9261 | -86.6% |
| GT2 | 101.0190 +- 0.0000 | 1534.92 +- 0.00 | 2.40 | 102.3414 | +23.2% |

## Overall Performance Summary

### Best Performer by Phase

| Phase | Best RMSE | Best ITAE | Best Settling |
|-------|-----------|-----------|---------------|
| 1 | PID (0.7922) | GT2 (130.65) | PD (11.20s) |
| 2 | GT2 (2.3348) | PID (0.00) | PID (0.00s) |
| 3 | IT2 (1.1984) | IT2 (0.00) | IT2 (0.00s) |
| 4 | PID (5.2871) | GT2 (0.00) | GT2 (0.00s) |
| 5 | PID (64.9325) | PD (0.00) | PD (0.00s) |

## Conclusions

*TODO: Add analysis conclusions based on results*

### Key Findings

1. **Baseline Performance**: [Analysis of Phase 1 results]
2. **Steady Wind**: [IT2/GT2 advantage over PID/PD]
3. **Turbulence**: [Effect of TI on controller ranking]
4. **Gust Recovery**: [Transient response comparison]
5. **Combined Disturbance**: [Realistic scenario results]
6. **Endurance**: [Long-term stability assessment]

## Limitations

- Simulation-based results (Gazebo ODE physics)
- Single random seed per run (consider multiple seeds)
- Fixed PID gains across all controllers

## Validation Checks

- [ ] Wind topic active with expected dynamics
- [ ] GT2 controllers receiving normalized wind [0,1]
- [ ] IT2 controllers receiving signed wind [-10,10] m/s
- [ ] Metrics windows aligned with phase boundaries
