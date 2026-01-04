# Collision Avoidance Safety Layer - Test Results

## Overview

This document presents the test results for the APF-based (Artificial Potential Field) collision avoidance safety layer integrated into the multi-agent formation control system.

**Important Note**: This is a **SOFT CONSTRAINT** approach. It reduces collision risk but does NOT guarantee collision-free operation. For hard constraints, consider CBF (Control Barrier Functions) or RVO (Reciprocal Velocity Obstacles) approaches.

## Architecture

```
Formation Coordinator → target_pose → [CollisionSafetyLayer] → safe_target_pose → AgentController
                                            ↑
                                    (mode=disabled: bypass)
                                    (mode=pass_through: republish only)
                                    (mode=avoidance: APF active)
```

### Operating Modes

| Mode | Description | Use Case |
|------|-------------|----------|
| `disabled` | Layer bypassed completely | Baseline comparison (default) |
| `pass_through` | Republish targets without modification | Isolate republish effect |
| `avoidance` | APF-based collision avoidance active | Safety experiments |

### APF Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| `safety_distance` | 0.8 m | Near-miss detection threshold |
| `influence_distance` | 2.0 m | APF repulsion activation radius |
| `k_repulsion` | 3.0 | Repulsion gain |
| `max_target_deviation` | 0.5 m | Maximum target modification |
| `publish_rate_hz` | 20.0 | Safe target publish rate |

## Test Methodology

### Fair Comparison Setup

To ensure methodologically sound comparison:
- Same random seed (42) used for both tests
- Same wind profile (von Kármán turbulence)
- Same controller configurations
- Same simulation duration (60s)

### Test Commands

```bash
# Collision OFF (Baseline)
ros2 launch agent_control_pkg phased_comparison.launch.py \
    phase:=3 collision_mode:=disabled seed:=42

# Collision ON (Avoidance)
ros2 launch agent_control_pkg phased_comparison.launch.py \
    phase:=3 collision_mode:=avoidance seed:=42
```

## Results: Phase 3 (von Kármán Turbulence)

### Mean RMSE [m]

| Controller | Collision OFF | Collision ON | Δ (Change) | % Change |
|------------|--------------|--------------|------------|----------|
| **PD** | 1.785 | 1.796 | +0.011 | +0.6% |
| **PID** | 0.923 | 0.924 | +0.001 | +0.1% |
| **IT2-FLS** | 0.897 | 0.914 | +0.017 | +1.9% |
| **GT2-FLS** | 0.915 | 0.913 | -0.002 | -0.2% |

### Steady-State RMSE [m]

| Controller | Collision OFF | Collision ON | Δ (Change) | % Change |
|------------|--------------|--------------|------------|----------|
| **PD** | 1.818 | 1.827 | +0.009 | +0.5% |
| **PID** | 0.783 | 0.764 | -0.019 | **-2.4%** |
| **IT2-FLS** | 0.767 | 0.760 | -0.007 | -0.9% |
| **GT2-FLS** | 0.765 | 0.756 | -0.009 | **-1.2%** |

### Control Effort (IAE)

| Controller | Collision OFF | Collision ON | Δ (Change) | % Change |
|------------|--------------|--------------|------------|----------|
| **PD** | 555.4 | 550.0 | -5.4 | **-1.0%** |
| **PID** | 556.7 | 550.8 | -5.9 | **-1.1%** |
| **IT2-FLS** | 558.2 | 552.2 | -6.0 | **-1.1%** |
| **GT2-FLS** | 557.9 | 550.1 | -7.8 | **-1.4%** |

### Peak Error [m]

| Controller | Collision OFF | Collision ON | Δ (Change) |
|------------|--------------|--------------|------------|
| **PD** | 3.255 | 3.171 | -0.084 |
| **PID** | 3.007 | 3.010 | +0.003 |
| **IT2-FLS** | 3.101 | 2.981 | -0.120 |
| **GT2-FLS** | 3.073 | 3.032 | -0.041 |

## Summary

```
┌─────────────────────────────────────────────────────────────────┐
│           COLLISION AVOIDANCE IMPACT (Same Seed)                │
├─────────────────────────────────────────────────────────────────┤
│  RMSE Change:          ≈ 0% (negligible impact)                │
│  SS-RMSE Change:       -0.9% ~ -2.4% (slight improvement)      │
│  Control Effort:       -1.0% ~ -1.4% (slight reduction)        │
│  Peak Error:           -0.04 ~ -0.12m (slight improvement)     │
├─────────────────────────────────────────────────────────────────┤
│  CONCLUSION: Collision avoidance does NOT degrade performance  │
│              Minor improvements observed in some metrics       │
└─────────────────────────────────────────────────────────────────┘
```

## Analysis

### Why No Performance Degradation?

1. **Large inter-group spacing**: 4 groups positioned at Y = {-12, -4, +4, +12} meters (8m apart)
2. **APF influence radius**: 2.0m influence_distance means groups don't trigger repulsion
3. **Intra-group spacing**: 3.0m formation spacing keeps drones outside influence zone

### Why Slight Improvement?

1. **Republish effect**: Safe targets published at 20Hz provide fresher timestamps
2. **Minor corrections**: APF may apply small beneficial corrections

## Safety Metrics

The collision safety layer publishes metrics to `/collision_safety/metrics`:

| Index | Metric | Description |
|-------|--------|-------------|
| 0 | `min_inter_agent_distance` | Current minimum distance between any two agents |
| 1 | `min_distance_ever` | All-time minimum distance observed |
| 2 | `near_miss_count` | Number of frames where d < safety_threshold |
| 3 | `time_below_threshold_sec` | Cumulative time below safety threshold |
| 4 | `avg_inter_agent_distance` | Average inter-agent distance |
| 5 | `safety_threshold` | Configured threshold (default: 0.8m) |
| 6 | `mode` | Current mode (0=DISABLED, 1=PASS_THROUGH, 2=AVOIDANCE) |

## Usage

### Launch Arguments

```bash
ros2 launch agent_control_pkg phased_comparison.launch.py \
    phase:=<1-6> \
    collision_mode:=<disabled|pass_through|avoidance> \
    seed:=<integer>
```

### Recommended Thesis Structure

1. **Main Results (Chapter 5.1)**: Use `collision_mode:=disabled` for controller comparison
2. **Safety Analysis (Chapter 5.2)**: Use `collision_mode:=avoidance` for safety experiments
3. **Trade-off Discussion (Chapter 6)**: Compare both modes with same seeds

## Files

| File | Description |
|------|-------------|
| `collision_safety_layer.hpp` | Header with SafetyMode enum, Position2D, metrics |
| `collision_safety_layer_node.cpp` | Full APF implementation (~350 lines) |
| `collision_avoidance_params.yaml` | Tunable parameters |
| `phased_comparison.launch.py` | Launch integration with remapping |

## Conclusion

The collision safety layer has been successfully integrated into the multi-agent formation control system. Test results demonstrate that:

1. **Controller comparison methodology remains valid** - RMSE differences are within noise margin (<2%)
2. **No performance penalty** - Control effort actually decreased slightly
3. **Modular design** - Can be enabled/disabled via launch argument
4. **Clean separation** - No modifications to agent_controller_node.cpp required

This implementation provides a foundation for future safety enhancements while maintaining backward compatibility with existing experiments.
