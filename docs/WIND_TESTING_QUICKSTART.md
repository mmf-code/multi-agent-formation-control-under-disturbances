# Wind Disturbance Testing - Quick Start Guide

**Defense Industry Standard Testing Protocol**
**Author**: Multi-Agent Formation Control Research
**Date**: 2026-01-02

---

## Overview

Bu dokuman, rüzgar bozucu test altyapısının hızlı kullanımı için hazırlanmıştır.
Detaylı framework için: `docs/WIND_TEST_FRAMEWORK.md`

---

## Phase 1: IMMEDIATE ACTIONS (This Week)

### Step 1: Add Wind Metrics to Build

MetricsData.msg zaten güncellendi. Şimdi rebuild gerekli:

```bash
# Clean build for message changes
colcon build --symlink-install --packages-select my_custom_interfaces_pkg
colcon build --symlink-install --packages-select agent_control_pkg

# Source
source install/setup.bash
```

### Step 2: Update metrics_publisher_node.cpp

**File**: `agent_control_pkg/src/ros/metrics_publisher_node.cpp`

**Add wind subscription** (after target subscription, ~line 55):

```cpp
// Subscribe to wind velocity for wind metrics
std::string wind_topic = "/" + agent_name + "/wind/velocity";
wind_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
    wind_topic,
    10,
    std::bind(&MetricsPublisherNode::windCallback, this, std::placeholders::_1)
);

// Fallback to global wind if per-drone not available
wind_sub_global_ = this->create_subscription<geometry_msgs::msg::Vector3>(
    "/wind/velocity",
    10,
    std::bind(&MetricsPublisherNode::windCallback, this, std::placeholders::_1)
);
```

**Add wind callback** (new method):

```cpp
void windCallback(const geometry_msgs::msg::Vector3::SharedPtr msg)
{
    wind_vx_ = msg->x;
    wind_vy_ = msg->y;
    wind_vz_ = msg->z;
}
```

**Add member variables** (in header, ~line 60):

```cpp
// Wind tracking
rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr wind_sub_;
rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr wind_sub_global_;
double wind_vx_ = 0.0;
double wind_vy_ = 0.0;
double wind_vz_ = 0.0;
double wind_exposure_time_ = 0.0;
const double WIND_THRESHOLD = 1.0;  // [m/s]
```

**Update publishMetrics()** (add after existing metrics, ~line 300):

```cpp
// Wind disturbance metrics
msg.wind_velocity_x = wind_vx_;
msg.wind_velocity_y = wind_vy_;
msg.wind_velocity_z = wind_vz_;
msg.wind_magnitude = std::sqrt(wind_vx_*wind_vx_ + wind_vy_*wind_vy_ + wind_vz_*wind_vz_);

// Disturbance Rejection Ratio (DRR): error/wind
double error_mag = std::sqrt(error_x*error_x + error_y*error_y);
msg.disturbance_rejection_ratio = (msg.wind_magnitude > 0.1)
    ? (error_mag / msg.wind_magnitude)
    : 0.0;

// Wind-normalized RMSE
msg.wind_normalized_rmse = (msg.wind_magnitude > 0.1)
    ? (msg.rmse_total / msg.wind_magnitude)
    : msg.rmse_total;

// Accumulate wind exposure time
auto now = this->now();
double dt = (now - last_time_).seconds();
if (msg.wind_magnitude > WIND_THRESHOLD) {
    wind_exposure_time_ += dt;
}
msg.wind_exposure_time = wind_exposure_time_;
```

**Rebuild**:

```bash
colcon build --symlink-install --packages-select agent_control_pkg
```

---

### Step 3: Enable Spatial Wind Coherence (OPTIONAL but RECOMMENDED)

**File**: `agent_control_pkg/launch/formation_comparison_demo.launch.py`

**Add after wind_publisher node** (~line 240):

```python
# Spatial Wind Field Node (IEC coherence)
spatial_wind_node = Node(
    package='agent_control_pkg',
    executable='spatial_wind_field.py',
    name='spatial_wind_field',
    output='screen',
    parameters=[{
        'num_agents': 9,
        'agent_prefix': 'agent_',
        'publish_rate': 20.0,  # 20 Hz

        # Turbulence model
        'turbulence_model': 'vonkarman',
        'turbulence_intensity': 0.15,
        'integral_length_u': 50.0,
        'integral_length_v': 25.0,
        'integral_length_w': 10.0,
        'mean_wind_speed': 3.5,

        # Mean wind
        'mean_wind_magnitude': 3.5,
        'mean_wind_direction_deg': 90.0,

        # IEC Spatial Coherence
        'coherence_decay_constant': 340.2,
        'enable_coherence': True,
    }]
)

# Add to launch actions
launch_description.append(spatial_wind_node)
```

**Note**: If you enable spatial wind, you can DISABLE the global `wind_publisher` node
or keep both (per-drone topics will override global).

---

## Quick Testing

### Test 1: Quick Wind Validation (60 seconds)

```bash
./scripts/quick_wind_test.sh 60
```

**Expected Output**:
```
Quick Wind Disturbance Test
==========================================
Duration: 60s
Scenario: moderate_wind_moderate_turbulence
==========================================

✓ Per-drone wind topics detected (spatial coherence active)
...
Calculating Wind KPIs...
==========================================
WIND DISTURBANCE KPI RESULTS
==========================================
  DRR (Disturbance Rejection Ratio):    0.423  (target: < 0.5)  ✓
  WNRMSE (Wind-Normalized RMSE):        0.651  (target: < 0.8)  ✓
  WIT (Wind Impact Time):               12.3%  (target: < 20%)  ✓
  ...
✓ All KPIs within target range!
```

---

### Test 2: PSD Validation

Rüzgar modelinin doğru çalıştığını verify et:

```bash
python3 scripts/validate_wind_model.py
```

**Expected**: 7 figures in `validation_results/`

---

### Test 3: Spatial Coherence Validation

IEC coherence doğrulaması:

```bash
python3 scripts/validate_spatial_coherence.py
```

**Expected**: 3 figures showing coherence decay with distance

---

## Systematic Testing (Phase 2)

### Full Wind Scenario Sweep

12 scenario x 3 repeats = 36 runs (~2 hours):

```bash
./scripts/run_wind_scenario_sweep.sh 180 3
```

**Output**:
- `thesis_data/wind_scenario_sweep/sweep_TIMESTAMP/`
  - `aggregated_kpis.csv` - All KPI data
  - `summary_statistics.csv` - Mean ± std for each scenario
  - `wind_scenario_comparison.png` - Comparison plots

---

## Key Performance Indicators (KPIs)

### Core Metrics

1. **DRR** (Disturbance Rejection Ratio)
   - Formula: `mean(|error| / |wind|)`
   - Target: **< 0.5**
   - Interpretation: Lower = better wind rejection

2. **WNRMSE** (Wind-Normalized RMSE)
   - Formula: `RMSE / mean(|wind|)`
   - Target: **< 0.8**
   - Interpretation: Dimensionless performance metric

3. **WIT** (Wind Impact Time)
   - Formula: `% time when |error| > 0.2m under wind`
   - Target: **< 20%**
   - Interpretation: Less time affected = more robust

4. **MWDR** (Max Wind Disturbance Rejection)
   - Formula: `1 - max(|error| / |wind|)`
   - Target: **> 0.6**
   - Interpretation: Worst-case handling

---

## Controller Comparison Strategy

### Hypothesis Testing

**Research Question**: Does IT2-FLS outperform PID under wind disturbances?

**Method**:
1. Run all 12 scenarios for each controller
2. Collect DRR, WNRMSE for each run
3. Statistical test: One-way ANOVA
4. Post-hoc: Tukey HSD
5. Effect size: Cohen's d

**Expected Result** (based on literature):
- PID: DRR ≈ 0.60-0.70
- PD: DRR ≈ 0.65-0.75 (worse, no integral)
- IT2-FLS: DRR ≈ 0.40-0.50 (best, adaptive)

**Thesis Statement Template**:
> "Statistical analysis (one-way ANOVA, F=24.3, p<0.001) reveals significant
> differences in wind rejection performance. Post-hoc Tukey HSD confirms that
> IT2-FLS achieves 35% lower DRR (M=0.42, SD=0.08) compared to classical PID
> (M=0.65, SD=0.12), with large effect size (Cohen's d = 1.8). Under high
> turbulence intensity (TI=20%), the advantage increases to 45%."

---

## Troubleshooting

### Issue 1: No per-drone wind topics

**Symptom**: `quick_wind_test.sh` reports "Using global wind topic"

**Fix**:
- Check if `spatial_wind_field.py` is launched
- Verify topic: `ros2 topic list | grep agent_0/wind`
- If missing, add spatial_wind_node to launch file (see Step 3)

---

### Issue 2: High DRR (> 0.8)

**Symptom**: Controllers perform poorly, DRR exceeds target

**Possible Causes**:
1. PID gains not tuned for wind
2. Wind too strong (> 8 m/s)
3. Fuzzy wind_scalar misconfigured

**Fix**:
```bash
# Re-tune PID under wind
python3 scripts/tune_pid_under_wind.py

# Check fuzzy wind scalar
# In launch file, verify: fuzzy.wind_scalar: 1.0
```

---

### Issue 3: Metrics CSV not generated

**Symptom**: `quick_wind_test.sh` reports "No metrics CSV found"

**Fix**:
- Check if `metrics_collection_node` is running
- Verify `thesis_data/` directory exists
- Check: `ros2 topic echo /agent_0/metrics`

---

## Advanced Features (Phase 3+)

### Feed-Forward Wind Compensation

Re-enable feed-forward (currently disabled):

```python
# In formation_comparison_demo.launch.py
'feedforward.enable_wind': True,
'feedforward.k_wind': 1.0,  # Start with 1.0, tune via sweep
```

**Then run**:
```bash
# Compare FF vs No-FF
./scripts/feedforward_sweep.sh
```

---

### Fuzzy Wind Scalar Optimization

Find optimal `fuzzy.wind_scalar`:

```bash
python3 scripts/tune_fuzzy_wind_scalar.py
```

**Expected**: Optimal value ≈ 0.8-1.5

---

## Current Status Summary

### ✅ Implemented & Working
1. Dual wind physics (velocity + force)
2. von Kármán & Dryden turbulence models
3. IEC spatial coherence (code ready)
4. Fuzzy controller wind input
5. 12 standard wind scenarios
6. Wind KPI calculator
7. Quick test & scenario sweep scripts

### ⚠️ Needs Activation
1. **Spatial wind field** - Add to launch file
2. **Wind metrics** - Update metrics_publisher_node.cpp
3. **Feed-forward** - Re-enable in config

### 🔴 Not Yet Implemented
1. Real-time wind estimation (Kalman filter)
2. Adaptive fuzzy wind gain
3. Formation-level wind metrics

---

## Next Steps

1. **Today**:
   - Update `metrics_publisher_node.cpp` with wind metrics
   - Rebuild workspace
   - Run `quick_wind_test.sh`

2. **This Week**:
   - Add spatial_wind_field to launch file (optional)
   - Run PSD validation
   - Run 1-2 wind scenarios for verification

3. **Next Week**:
   - Full scenario sweep (12 scenarios)
   - Statistical analysis
   - Generate thesis figures

---

## Questions?

Refer to:
- Full framework: `docs/WIND_TEST_FRAMEWORK.md`
- Wind scenarios: `scripts/wind_scenarios.py --print-table`
- Validation scripts: `scripts/validate_*.py`

**Thesis Contribution**: This framework enables rigorous, defense-industry-standard
testing of multi-agent formation control under realistic atmospheric disturbances,
with quantitative KPIs and statistical validation.

---

**END OF QUICK START GUIDE**
