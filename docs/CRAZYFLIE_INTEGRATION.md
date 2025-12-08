# Crazyflie Integration Plan

**Branch:** `main` (merged from `feature/crazyflie-integration`)
**Created:** 2024-12-08
**Last Updated:** 2025-12-09
**Status:** Phase 2 Complete - Ready for Testing

---

## Quick Start

```bash
# Build with Crazyflie support
colcon build --symlink-install --cmake-args -DENABLE_CRAZYFLIE=ON

# Run demo (1 drone)
./scripts/run_crazyflie_demo.sh

# Run demo (3 drones) - requires crazyflies.yaml config update
./scripts/run_crazyflie_demo.sh 3

# Run with GUI
./scripts/run_crazyflie_demo.sh 1 --gui
```

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

        ┌───────────────────┐
        │ tf_to_odom_bridge │  <-- NEW: Converts TF to Odometry
        │ (Python node)     │      for controller compatibility
        └───────────────────┘
```

## Phases

### Phase 1: Environment Setup - COMPLETE
- [x] Install `ros-humble-crazyflie-sim` package (apt)
- [x] Document Crazyswarm2 topic structure (FullState msg)
- [x] Verify compatibility with existing workspace
- [x] Create TF→Odom bridge (`scripts/tf_to_odom_bridge.py`)

### Phase 2: Code Integration - COMPLETE
- [x] Add `ENABLE_CRAZYFLIE` CMake option
- [x] Implement dual-output in `agent_controller_node.cpp` (`#ifdef CRAZYFLIE_SUPPORT`)
- [x] Create `crazyflie_formation.launch.py` with conditional node launch
- [x] Create demo script (`scripts/run_crazyflie_demo.sh`)
- [x] Test 1-drone and 3-drone controller startup

### Phase 3: Tuning & Validation - PENDING
- [x] Calculate initial Crazyflie gains (mass ratio) - see `config/crazyflie_params.yaml`
- [ ] Add cf232, cf233 to Crazyswarm2 config (for 3-drone formation)
- [ ] Tune PID parameters for Crazyflie dynamics
- [ ] Tune Fuzzy scaling factors
- [ ] Collect comparison data (5 runs each)
- [ ] Generate comparison plots

---

## Files Created/Modified

| File | Status | Description |
|------|--------|-------------|
| `CMakeLists.txt` | MODIFIED | Added `ENABLE_CRAZYFLIE` option |
| `package.xml` | MODIFIED | Added `crazyflie_interfaces` exec_depend |
| `include/.../agent_controller_node.hpp` | MODIFIED | Added `#ifdef CRAZYFLIE_SUPPORT` members |
| `src/ros/agent_controller_node.cpp` | MODIFIED | Added dual-output publisher |
| `launch/crazyflie_formation.launch.py` | NEW | Launch file for CF sim |
| `config/crazyflie_params.yaml` | NEW | CF-specific PID parameters |
| `scripts/tf_to_odom_bridge.py` | NEW | TF→Odometry converter |
| `scripts/run_crazyflie_demo.sh` | NEW | All-in-one demo script |
| `external/COLCON_IGNORE` | NEW | Excludes crazyflie-firmware from colcon |
| `external/crazyflie-firmware/COLCON_IGNORE` | NEW | Same |

---

## Technical Details

### Gain Adaptation Formula

```
m_custom = 1.5 kg
m_crazyflie = 0.027 kg
ratio = m_cf / m_custom = 0.018

Initial Crazyflie gains (in config/crazyflie_params.yaml):
  Kp_cf = 0.315  (Kp_custom × ratio × correction)
  Ki_cf = 0.050
  Kd_cf = 0.194  (Kd_custom × ratio × correction)
```

### CMake Flag Usage

```bash
# Build WITHOUT Crazyflie (default, thesis data safe)
colcon build --symlink-install

# Build WITH Crazyflie support
colcon build --symlink-install --cmake-args -DENABLE_CRAZYFLIE=ON
```

### Topic Mapping

| Controller Topic | Crazyswarm2 Topic | Bridge |
|-----------------|-------------------|--------|
| `/cf0/odom` | `/cf231/odom` | tf_to_odom_bridge.py |
| `/cf0/target_pose` | `/cf231/target_pose` | Direct (remapped) |
| `/cf0/cmd_full_state` | `/cf231/cmd_full_state` | Direct (remapped) |

---

## Known Issues & Solutions

### Issue 1: Crazyswarm2 doesn't publish `/odom`
**Solution:** Use `tf_to_odom_bridge.py` to convert TF broadcasts to Odometry messages.

### Issue 2: Only cf231 defined in default config
**Solution:** Edit `/opt/ros/humble/share/crazyflie/config/crazyflies.yaml` to add cf232, cf233:
```yaml
robots:
  cf231:
    enabled: true
    uri: radio://0/80/2M/E7E7E7E7E7
    initial_position: [0.0, 0.0, 0.0]
    type: cf21
  cf232:
    enabled: true
    uri: radio://0/80/2M/E7E7E7E732
    initial_position: [0.5, 0.0, 0.0]
    type: cf21
  cf233:
    enabled: true
    uri: radio://0/80/2M/E7E7E7E733
    initial_position: [1.0, 0.0, 0.0]
    type: cf21
```

### Issue 3: RViz crashes with wayland error
**Solution:** Run without GUI flag or use X11 instead of Wayland.

---

## Next Steps (for next session)

1. **3-Drone Config:** Update crazyflies.yaml with cf232, cf233
2. **Visual Test:** Run with `--gui` and verify drone movement
3. **PID Tuning:** Adjust gains based on observed behavior
4. **Data Collection:** Record position tracking performance
5. **Comparison:** Side-by-side with custom Gazebo simulation

---

## Success Criteria

- [x] Existing thesis simulation unchanged (regression test passed)
- [x] Crazyflie controllers start successfully (1 and 3 drones)
- [ ] Crazyflie sim runs 3-drone formation visually
- [ ] RMSE < 0.5m for Crazyflie formation
- [ ] Side-by-side comparison video ready
- [ ] Thesis section 5.2 draft complete

---

## References

- [Crazyswarm2 Documentation](https://imrclab.github.io/crazyswarm2/)
- [ROS2 Humble Crazyflie Packages](https://index.ros.org/r/crazyflie/)
- [Crazyflie Interfaces](https://github.com/IMRCLab/crazyswarm2/tree/main/crazyflie_interfaces)

---

## Progress Log

### 2025-12-09 (Session Complete)
- **TF→Odom Bridge Created:**
  - `scripts/tf_to_odom_bridge.py` converts Crazyswarm2 TF to Odometry
  - Publishes at 100Hz for smooth control
- **Demo Script Created:**
  - `scripts/run_crazyflie_demo.sh` runs entire stack with one command
  - Supports 1-3 drones and optional GUI
- **Integration Tested:**
  - 1 drone: Controllers start, dual-output enabled
  - 3 drones: All controllers start successfully
  - Ctrl+C cleanup works correctly
- **Status:** Ready for visual testing and tuning

### 2025-12-08 (Initial Implementation)
- **Phase 1 Complete:**
  - Verified ros-humble-crazyflie packages already installed via apt
  - Added COLCON_IGNORE to external/crazyflie-firmware
- **Phase 2 Complete:**
  - Added `ENABLE_CRAZYFLIE` CMake option in CMakeLists.txt
  - Added `crazyflie_interfaces` exec_depend in package.xml
  - Implemented dual-output publisher with `#ifdef CRAZYFLIE_SUPPORT`
  - Created conditional launch file `crazyflie_formation.launch.py`
  - Build tested: Default and WITH Crazyflie both succeed
- **Regression Test Passed:** Existing thesis simulation unchanged

### 2024-12-08
- Created feature branch `feature/crazyflie-integration`
- Initial planning document created
