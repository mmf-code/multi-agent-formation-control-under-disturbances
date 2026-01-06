# Thesis Plot Report

**Generated:** 2026-01-06 23:35:26

## Plot Configuration

| Plot | Status | Description |
|------|--------|-------------|
| rmse_evolution | ACTIVE | RMSE time series with std dev shading |
| xyz_error_decomposition | ACTIVE | Error breakdown by X/Y/Z axis |
| ss_error_boxplot | ACTIVE | Steady-state error distribution |
| min_inter_agent_distance | ACTIVE | Minimum inter-agent distance over time |
| collision_risk_histogram | ACTIVE | Distribution of inter-agent distances |
| control_effort | ACTIVE | Control effort (IAE) comparison |
| jerk_analysis | ACTIVE | Jerk (smoothness) analysis |
| trajectory_2d | ACTIVE | Top-view XY trajectory |
| altitude_hold | ACTIVE | Altitude (Z) over time |
| wind_profile | ACTIVE | Wind disturbance time series (X/Y/magnitude) |
| wind_correlation | ACTIVE | Wind magnitude vs error correlation |
| phase_comparison_heatmap | ACTIVE | Cross-phase RMSE heatmap |
| controller_ranking | ACTIVE | Controller win count |

## Phase Summary

### Phase 1: BASELINE

| Controller | RMSE | SS-RMSE | MAE | Control Effort |
|------------|------|---------|-----|----------------|
| PD | 1.1912 | 1.1218 | 1.0755 | 521.00 |
| PID | 0.6662 | 0.3931 | 0.4855 | 519.35 |
| IT2 | 0.6491 | 0.3761 | 0.4720 | 521.21 |
| GT2 | 0.6515 | 0.3862 | 0.4751 | 520.97 |

**Winner:** IT2

### Phase 2: STEADY_WIND

| Controller | RMSE | SS-RMSE | MAE | Control Effort |
|------------|------|---------|-----|----------------|
| PD | 1.8116 | 1.8295 | 1.7117 | 550.11 |
| PID | 0.9165 | 0.7050 | 0.8028 | 552.24 |
| IT2 | 0.9342 | 0.6805 | 0.8050 | 549.98 |
| GT2 | 0.8874 | 0.6737 | 0.7630 | 548.85 |

**Winner:** GT2

### Phase 3: TURBULENCE

| Controller | RMSE | SS-RMSE | MAE | Control Effort |
|------------|------|---------|-----|----------------|
| PD | 1.9732 | 1.9338 | 1.8298 | 538.04 |
| PID | 1.1607 | 0.7386 | 0.9523 | 537.55 |
| IT2 | 1.0219 | 0.7451 | 0.8762 | 539.49 |
| GT2 | 1.0057 | 0.7575 | 0.8683 | 544.58 |

**Winner:** GT2

### Phase 4: GUST

| Controller | RMSE | SS-RMSE | MAE | Control Effort |
|------------|------|---------|-----|----------------|
| PD | 1.3620 | 1.3197 | 1.2129 | 534.84 |
| PID | 0.8510 | 0.6690 | 0.6169 | 531.78 |
| IT2 | 0.8637 | 0.6563 | 0.6182 | 532.39 |
| GT2 | 0.8449 | 0.6527 | 0.6098 | 532.06 |

**Winner:** GT2

### Phase 5: COMBINED

| Controller | RMSE | SS-RMSE | MAE | Control Effort |
|------------|------|---------|-----|----------------|
| PD | 1.7743 | 1.7488 | 1.6847 | 535.60 |
| PID | 1.0596 | 0.7617 | 0.8286 | 540.92 |
| IT2 | 1.1242 | 0.7392 | 0.8471 | 532.39 |
| GT2 | 0.9964 | 0.7403 | 0.7999 | 540.02 |

**Winner:** GT2

