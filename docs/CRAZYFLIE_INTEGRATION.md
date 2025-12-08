# Crazyflie Integration Plan

**Branch:** `feature/crazyflie-integration`
**Created:** 2024-12-08
**Status:** In Progress

---

## Objective

Integrate Crazyswarm2 simulator with existing formation control system to demonstrate controller portability across different drone platforms.

## Architecture

```
                           ┌──────────────────────────────┐
                           │   agent_controller_node      │
                           │   (PID + IT2-FLS logic)      │
                           └──────────────┬───────────────┘
                                          │
                    ┌─────────────────────┼─────────────────────┐
                    │                     │                     │
                    ▼                     ▼                     │
        ┌───────────────────┐   ┌───────────────────┐          │
        │ /agent_X/cmd_accel│   │/cfX/cmd_full_state│          │
        │ (Vector3)         │   │ (FullState)       │          │
        │ [EXISTING]        │   │ [NEW]             │          │
        └─────────┬─────────┘   └─────────┬─────────┘          │
                  │                       │                     │
                  ▼                       ▼                     │
        ┌───────────────────┐   ┌───────────────────┐          │
        │simple_drone_plugin│   │ crazyflie_server  │          │
        │ (Custom Gazebo)   │   │ (Crazyswarm2 Sim) │          │
        └───────────────────┘   └───────────────────┘          │
```

## Phases

### Phase 1: Environment Setup (Week 1)
- [ ] Install `ros-humble-crazyflie-sim` package
- [ ] Test single Crazyflie hover in simulation
- [ ] Document Crazyswarm2 topic structure
- [ ] Verify compatibility with existing workspace

### Phase 2: Code Integration (Week 2)
- [ ] Add `ENABLE_CRAZYFLIE` CMake option
- [ ] Implement dual-output in `agent_controller_node.cpp`
- [ ] Create `crazyflie_formation.launch.py`
- [ ] Test 3-drone formation with Crazyflie sim

### Phase 3: Tuning & Validation (Week 3)
- [ ] Calculate initial Crazyflie gains (mass ratio)
- [ ] Tune PID parameters for Crazyflie dynamics
- [ ] Tune Fuzzy scaling factors
- [ ] Collect comparison data (5 runs each)
- [ ] Generate comparison plots

## Technical Details

### Gain Adaptation Formula

```
m_custom = 1.5 kg
m_crazyflie = 0.027 kg
ratio = m_cf / m_custom = 0.018

Initial Crazyflie gains:
  Kp_cf ≈ Kp_custom × ratio × correction
       ≈ 3.5 × 0.018 × 5 = 0.315
  Kd_cf ≈ Kd_custom × ratio × correction
       ≈ 3.6 × 0.018 × 3 = 0.194
```

### CMake Flag Usage

```bash
# Build WITHOUT Crazyflie (default, thesis data safe)
colcon build --symlink-install

# Build WITH Crazyflie support
colcon build --symlink-install --cmake-args -DENABLE_CRAZYFLIE=ON
```

### File Changes Required

| File | Change Type | Description |
|------|-------------|-------------|
| `CMakeLists.txt` | MODIFY | Add ENABLE_CRAZYFLIE option |
| `package.xml` | MODIFY | Add crazyflie_interfaces dependency |
| `agent_controller_node.cpp` | MODIFY | Add dual-output publisher |
| `crazyflie_formation.launch.py` | NEW | Launch file for CF sim |
| `config/crazyflie_gains.yaml` | NEW | CF-specific parameters |

## Safety Rules

1. **NEVER modify existing publishers** - only ADD new ones
2. **Feature flag required** - changes must be behind `#ifdef CRAZYFLIE_SUPPORT`
3. **Separate config files** - CF gains in separate YAML
4. **Test mevcut sistem first** - verify `./scripts/run_full_demo.sh` still works

## Success Criteria

- [ ] Existing thesis simulation unchanged (regression test)
- [ ] Crazyflie sim runs 3-drone formation
- [ ] RMSE < 0.5m for Crazyflie formation
- [ ] Side-by-side comparison video ready
- [ ] Thesis section 5.2 draft complete

## References

- [Crazyswarm2 Documentation](https://imrclab.github.io/crazyswarm2/)
- [ROS2 Humble Crazyflie Packages](https://index.ros.org/r/crazyflie/)
- [Crazyflie Interfaces](https://github.com/IMRCLab/crazyswarm2/tree/main/crazyflie_interfaces)

---

## Progress Log

### 2024-12-08
- Created feature branch `feature/crazyflie-integration`
- Initial planning document created
