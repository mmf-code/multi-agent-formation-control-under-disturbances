# Event-Triggered Communication (ETC) Implementation Report

**Date:** 2025-01-05
**Branch:** `feature/event-triggered-communication`
**Status:** Implemented and Verified

---

## 1. Overview

Event-Triggered Communication (ETC) was implemented for the formation coordinator to reduce communication bandwidth while maintaining control performance. Instead of publishing target poses at a fixed rate (10 Hz), ETC only publishes when necessary based on configurable conditions.

## 2. Implementation Details

### 2.1 Files Modified

| File | Changes |
|------|---------|
| `formation_coordinator_node.hpp` | Added `ETCState`, `ETCMetrics` structs; ETC parameters and methods |
| `formation_coordinator_node.cpp` | Implemented `shouldTriggerEvent()`, `poseDistance()`, `publishETCMetrics()` |
| `formation_12drone_pd.yaml` | Added ETC configuration parameters |
| `formation_12drone_pid_group.yaml` | Added ETC configuration parameters |
| `formation_12drone_it2_group.yaml` | Added ETC configuration parameters |
| `formation_12drone_gt2_group.yaml` | Added ETC configuration parameters |

### 2.2 ETC Parameters

```yaml
etc.enable: true           # true=event-triggered, false=time-triggered (original)
etc.epsilon_pos: 0.05      # Position threshold [m] - trigger if target moved > 5cm
etc.min_period_sec: 0.02   # Anti-chattering minimum interval [s]
etc.max_period_sec: 0.5    # Heartbeat maximum interval [s] (must be < 2.0s stale threshold)
```

### 2.3 Trigger Conditions

ETC publishes a new target pose when ANY of these conditions are met:
1. **First message** - Always publish on initialization
2. **Force publish** - On waypoint or shape transitions
3. **Position change** - Target moved more than `epsilon_pos` (0.05m)
4. **Heartbeat** - Maximum time since last publish exceeded (`max_period_sec`)

### 2.4 Anti-chattering

Minimum period (`min_period_sec = 0.02s`) prevents excessive publishing even when conditions are met rapidly.

## 3. Bug Fix

### 3.1 Problem
Initial implementation caused catastrophic failures with some agents showing RMSE of 200-300m instead of ~1-2m. The bug occurred when clock time went backwards (simulation reset, time sync issues).

### 3.2 Root Cause
```cpp
// BUG: Negative time_since_last caused anti-chattering to always return false
if (time_since_last < etc_min_period_sec_) {  // -1.0 < 0.02 = TRUE!
    return false;  // Stuck here forever, heartbeat never checked
}
```

### 3.3 Fix Applied
```cpp
// Handle clock going backwards
if (time_since_last < 0.0) {
    RCLCPP_WARN_THROTTLE(..., "Clock went backwards");
    state.last_sent_time = current_time;
    return true;  // Force publish to recover
}

// Handle excessive time gap (>10s indicates something wrong)
if (time_since_last > 10.0) {
    RCLCPP_WARN_THROTTLE(..., "Excessive time gap");
    return true;  // Force publish
}
```

Also fixed `ETCState` initialization:
```cpp
struct ETCState {
    rclcpp::Time last_sent_time{0, 0, RCL_ROS_TIME};  // Explicit init
    // ...
};
```

## 4. Test Results

### 4.1 Configuration
- **k_fuzzy:** 0.8
- **Wind seed:** 42
- **Simulation duration:** ~84 seconds

### 4.2 Phase 4 (Turbulent Wind) Results

| Group | ETC OFF (RMSE) | ETC ON (RMSE) | Difference |
|-------|----------------|---------------|------------|
| PD | 1.98m | 1.93m | -2% |
| PID | 1.25m | 1.28m | +2% |
| PID+IT2 | 1.22m | 1.26m | +3% |
| PID+GT2 | 1.22m | 1.19m | -2% |
| **OVERALL** | **1.42m** | **1.42m** | **0%** |

### 4.3 Phase 5 (Combined Wind) Results

| Group | ETC OFF (RMSE) | ETC ON (RMSE) | Difference |
|-------|----------------|---------------|------------|
| PD | 1.92m | 1.98m | +3% |
| PID | 1.23m | 1.22m | -1% |
| PID+IT2 | 1.19m | 1.19m | 0% |
| PID+GT2 | 1.21m | 1.22m | +1% |
| **OVERALL** | **1.39m** | **1.40m** | **+1%** |

### 4.4 Bandwidth Analysis

| Metric | Time-Triggered | Event-Triggered |
|--------|----------------|-----------------|
| Publish rate | 10 Hz continuous | ~2 Hz (heartbeat max) |
| Messages/sec/agent | 10 | ~2-3 |
| **Bandwidth reduction** | - | **~70-80%** |

## 5. Commits

```
c253124 config: Add ETC parameters to formation coordinator YAML configs
a9eacef feat: Add Event-Triggered Communication (ETC) to formation coordinator
```

## 6. Current Configuration

- **ETC:** Enabled (`etc.enable: true`)
- **k_fuzzy:** 0.8
- **Heartbeat:** 0.5s (safely below 2.0s stale threshold)

## 7. Usage

### Enable ETC (event-triggered)
```yaml
etc.enable: true
```

### Disable ETC (time-triggered, original behavior)
```yaml
etc.enable: false
```

### Monitor ETC metrics
```bash
ros2 topic echo /formation_X/etc_metrics
# Data: [etc_enable, total_events, total_cycles, event_rate, avg_inter_event_time, bandwidth_reduction]
```

## 8. Conclusions

1. **ETC maintains control performance** - Less than ±3% difference in RMSE across all groups
2. **Significant bandwidth reduction** - ~70-80% fewer messages published
3. **Robust to timing issues** - Bug fix handles clock backwards and excessive gaps
4. **Thesis-ready** - Can compare ETC ON vs OFF with same seed for fair comparison

## 9. Notes for Future Work

- The Mission RMSE includes initial transient (agents moving from spawn to formation positions)
- Steady-state waypoint errors are ~0.5-0.6m, consistent with previous results
- For cleaner metrics, consider starting recording after t>15s (formation settled)

---

*Report generated: 2025-01-05*
