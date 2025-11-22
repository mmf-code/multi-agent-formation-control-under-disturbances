# Multi-Agent Formation Control - Debugging Session Report

**Date:** 2025-11-22  
**Session Duration:** ~2 hours  
**Status:** ✅ **RESOLVED - Simulation Working**

---

## 🎯 Original Problem

**User Report:** Drones spawn in Gazebo simulation but do not move.

**Symptoms:**
- Drones visible in Gazebo
- Controllers logging "No target received yet. Awaiting target_pose."
- No movement despite all nodes appearing to run

---

## 🔍 Root Causes Identified

### 1. **DDS Shared Memory Exhaustion**
**Problem:** 30+ ROS 2 nodes exhausted system shared memory segments  
**Evidence:** `[RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_port7417`  
**Impact:** Inter-node communication failures

### 2. **Formation Coordinator Node Collision**
**Problem:** Launch file spawned 3 coordinators but all registered as `/formation_coordinator_node`  
**Evidence:** `ros2 node list` showed only 1 coordinator instead of 3  
**Impact:** Only 1 group received targets, others starved

### 3. **QoS Policy Mismatch**
**Problem:** Publishers used BEST_EFFORT, subscribers used RELIABLE (or vice versa)  
**Evidence:** `New subscription discovered... incompatible QoS. Last incompatible policy: RELIABILITY_QOS_POLICY`  
**Impact:** Zero messages delivered despite nodes running

### 4. **Gazebo Server Crash on Launch**
**Problem:** Gazebo crashed (exit 255) when starting with GUI + 30+ nodes  
**Evidence:** `[ERROR] [gzserver-1]: process has died [pid X, exit code 255]`  
**Impact:** No physics simulation, drones frozen

---

## ✅ Solutions Implemented

### 1. FastDDS UDP-Only Configuration

**File Created:** `fastdds_profiles.xml`

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>udp_transport</transport_id>
      <type>UDPv4</type>
    </transport_descriptor>
  </transport_descriptors>
  <participant profile_name="udp_only" is_default_profile="true">
    <rtps>
      <userTransports>
        <transport_id>udp_transport</transport_id>
      </userTransports>
      <useBuiltinTransports>false</useBuiltinTransports>
    </rtps>
  </participant>
</profiles>
```

**Environment Variables:**
```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export RMW_FASTRTPS_USE_SHM=0
export FASTRTPS_DEFAULT_PROFILES_FILE=$(pwd)/fastdds_profiles.xml
```

### 2. Launch File Namespace Fix

**File:** `agent_control_pkg/launch/formation_comparison_demo.launch.py`

**Changes (lines 357-403):**
- Wrapped each `formation_coordinator_node` in `GroupAction` with `PushRosNamespace`
- Added explicit `arguments=['--ros-args', '-r', '__node:=formation_coordinator_groupX', '-r', '__ns:=/formation_X']`

**Result:** 3 distinct nodes:
- `/formation_0/formation_coordinator_group0`
- `/formation_1/formation_coordinator_group1`
- `/formation_2/formation_coordinator_group2`

### 3. QoS Policy Standardization

**Changed to RELIABLE QoS (10) for all `target_pose` topics:**

**Files Modified:**
1. `agent_control_pkg/src/ros/agent_controller_node.cpp` (line 46)
2. `agent_control_pkg/src/ros/metrics_publisher_node.cpp` (line 77)
3. `formation_coordinator_pkg/src/formation_coordinator_node.cpp` (line 115)

**Before:**
```cpp
create_publisher<PoseStamped>("target_pose", rclcpp::SensorDataQoS())
create_subscription<PoseStamped>("target_pose", rclcpp::SensorDataQoS(), ...)
```

**After:**
```cpp
create_publisher<PoseStamped>("target_pose", 10)
create_subscription<PoseStamped>("target_pose", 10, ...)
```

### 4. Headless Launch Mode

**File:** `fix_and_launch.sh` (line 18)

**Change:**
```bash
# Before
ros2 launch agent_control_pkg formation_comparison_demo.launch.py

# After
ros2 launch agent_control_pkg formation_comparison_demo.launch.py gazebo_gui:=false rviz:=false
```

**Workaround:** Launch Gazebo without GUI to prevent crash, then attach `gzclient` manually if needed.

---

## 📊 Current Simulation Details

### Simulation Type
**Formation Comparison Demo** - Waypoint-based trajectory tracking with 3 controller types

### Configuration
- **9 Drones** in 3 groups of 3
- **3 Altitudes:** 1m, 4m, 7m (vertical separation)
- **3 Y-lanes:** y=-4, y=0, y=4 (horizontal separation)
- **8 Waypoints** per group, ~110 second duration

### Controller Types
| Group | Agents | Controller | Altitude | Y-Position | Expected Performance |
|-------|--------|------------|----------|------------|---------------------|
| 0 | 0-2 | PID+Fuzzy | 1m | -4m | Best disturbance rejection |
| 1 | 3-5 | PD | 4m | 0m | Fastest settling time |
| 2 | 6-8 | PID | 7m | 4m | Balanced performance |

### Trajectory Phases
1. **0-22s:** Takeoff and hover at start position
2. **22-27s:** Brief hover
3. **27-49s:** Move to waypoint 2 (x=4)
4. **49-54s:** Hover
5. **54-76s:** Move to waypoint 4 (x=12)
6. **76-81s:** Hover
7. **81-103s:** Move to final waypoint (x=20/22)
8. **103-110s:** Final hover

---

## 🎮 How to Run

### Standard Launch (Recommended)
```bash
./fix_and_launch.sh
```

### With GUI (After Headless Launch)
```bash
# In separate terminal
gzclient &
```

### With RViz
```bash
source install/setup.bash
rviz2 -d install/agent_control_pkg/share/agent_control_pkg/rviz/formation_demo.rviz
```

---

## 📈 Dashboard Integration

### Topics Published
- `/agent_X/odom` - Position/velocity (10 Hz)
- `/agent_X/target_pose` - Target position (10 Hz)
- `/agent_X/cmd_accel` - Control commands (200 Hz)
- `/agent_X/metrics` - Performance metrics (10 Hz)
- `/agent_X/path` - Trajectory history
- `/metrics/group_X` - Aggregate group metrics (10 Hz)

### Expected Dashboard Behavior
✅ All 9 drones visible and moving  
✅ Real-time position tracking  
✅ Error metrics decreasing over time  
✅ Formation shapes maintained  
✅ No QoS warnings  
✅ Smooth trajectory following

---

## 🗑️ Cleanup

### Temporary Files Created (Safe to Delete)
```bash
rm -f launch_debug.log
rm -f verification_log.txt
```

### Files to Keep
- `fastdds_profiles.xml` - **REQUIRED** for DDS config
- `fix_and_launch.sh` - **REQUIRED** for stable launch
- `clean_build.sh` - Useful for rebuilds

---

## ⚠️ Critical: DO NOT MODIFY

### QoS Settings
**ALL `target_pose` publishers and subscribers MUST use RELIABLE QoS (10)**

Changing to `SensorDataQoS()` will break communication.

### Environment Variables
**MUST set before every launch:**
```bash
export RMW_FASTRTPS_USE_SHM=0
```

### Launch Mode
**Use headless mode to prevent Gazebo crash**

---

## 📝 Build Instructions

### After QoS Changes
```bash
source /opt/ros/humble/setup.bash
rm -rf build/ install/ log/
colcon build --symlink-install
source install/setup.bash
```

### Quick Rebuild
```bash
./clean_build.sh
```

---

## 🎓 For Thesis Documentation

### Key Achievements
- ✅ Stable multi-agent simulation with 9 drones
- ✅ 30+ ROS 2 nodes communicating reliably
- ✅ Real-time performance metrics collection
- ✅ Comparison of 3 controller types
- ✅ Waypoint-based trajectory tracking

### Metrics to Report
- **Formation Error (RMSE):** Measure of formation maintenance
- **Settling Time:** Time to reach target within threshold
- **IAE/ITAE:** Integral error metrics
- **Controller Comparison:** PID+Fuzzy vs PD vs PID

### Simulation Parameters
- Update rate: 200 Hz (dt=0.005s)
- Control frequency: 200 Hz
- Target update: 10 Hz
- Metrics logging: 10 Hz
- Total agents: 9
- Total nodes: 30+

---

## 📌 Version Info

- **ROS 2:** Humble
- **Gazebo:** Classic 11.10.2
- **FastDDS:** UDP-only transport
- **OS:** Linux (Ubuntu 22.04 assumed)
- **Date Fixed:** 2025-11-22

---

## 🎉 Final Status

**✅ ALL ISSUES RESOLVED**

- ✅ Gazebo running stably
- ✅ All 9 drones moving
- ✅ All coordinators publishing targets
- ✅ All controllers receiving targets
- ✅ No QoS warnings
- ✅ No DDS errors
- ✅ Metrics being logged

**Simulation is production-ready for thesis data collection.**

---

**Report Generated:** 2025-11-22 17:15  
**Session Type:** Emergency Debugging → Full Resolution  
**Outcome:** Success 🎉
