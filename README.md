# Multi-Agent Formation Control Under Disturbances - Quickstart

This repo contains a 2D single-drone dynamics testbed (extensible to multi-drone) with selectable controllers (PID, P/PI/PD, GT2 Fuzzy, and hybrid PID+Fuzzy), YAML-driven configuration, CSV logging, and Python plotting.

---

## 🎓 **FOR THESIS INSTRUCTOR: Quick Demo Guide**

### **System Status: ✅ READY**
- 9 drones in 3 formation groups (PID+Fuzzy, PD, PID)
- Wind disturbance testing (4.0N mean, 1.5N variance)
- Real-time metrics & CSV data logging
- Optimized controller parameters (Ts5 < 3s, OS < 0.2%)

### **Quick Commands for Demo**

#### **⭐ ONE COMMAND - COMPLETE DEMO (EASIEST!)**
```bash
cd /home/mmf/Documents/GitHub/multi-agent-formation-control-under-disturbances
./scripts/run_full_demo.sh              # Everything: GUI + CSV (60s)
./scripts/run_full_demo.sh 90           # 90 seconds
```

**What it does:** Full automated demo from start to finish:
1. ✅ Setup ROS2 + workspace
2. ✅ Launch Gazebo + RViz (visible GUI)
3. ✅ Record CSV data (synchronized from t=0)
4. ✅ Stop and display results

---

#### **1. Visual Demo (Alternative Methods)**
```bash
./scripts/quick_test.sh                  # 15s system validation (optional)
./scripts/run_formation_demo.sh          # Full GUI demo (watch only)
./scripts/demo_with_recording.sh 60      # Full GUI + CSV recording
```

**What you'll see:**
- 9 drones in 3 groups, each group maintaining triangle formation
- **Group 0 (Magenta)**: PID+Fuzzy controllers - Best wind rejection
- **Group 1 (Cyan)**: PD controllers - Fastest settling time
- **Group 2 (Yellow)**: PID controllers - Balanced performance
- All groups move from x=-10 to x=+5 under random wind forces

#### **2. Record Thesis Data (CSV Output Only - Headless)**
```bash
./scripts/record_thesis_data.sh      # 60s recording (default)
./scripts/record_thesis_data.sh 120  # 120s recording
```

**Output:** `thesis_data/YYYY-MM-DD/HH-MM-SS_formation/`
- `agent_0_pidfuzzy.csv` - PID+Fuzzy controller metrics
- `agent_3_pd.csv` - PD controller metrics
- `agent_6_pid.csv` - PID controller metrics

**CSV Columns:** timestamp, elapsed_time, error_x/y/z, RMSE, IAE, ITAE, settling_time, is_settled

#### **3. Expected Results (Thesis Hypothesis)**
Under wind disturbance (4.0N mean):
- **PID+Fuzzy (k_fuzzy=0.7)**: Lowest RMSE & IAE (best disturbance rejection)
- **PID (Ki=1.946)**: Good steady-state, moderate disturbance rejection
- **PD (no integral)**: Fast settling but higher steady-state error under wind

### **Controller Parameters (Optimized)**
| Controller | Kp    | Ki    | Kd    | k_fuzzy | Notes                          |
|-----------|-------|-------|-------|---------|--------------------------------|
| PID+Fuzzy | 3.501 | 1.946 | 3.608 | 0.7     | Wind-aware fuzzy + PID hybrid  |
| PID       | 3.501 | 1.946 | 3.608 | -       | Standard 3-term PID            |
| PD        | 3.5   | 0.0   | 3.61  | -       | No integral, fast response     |

**Design specs:** Ts5 ≈ 2.7s, Overshoot < 0.2%, Settling threshold = 5cm

### **Troubleshooting**
If demo doesn't start:
```bash
# Kill old processes
pkill -9 gzserver; pkill -9 gzclient

# Rebuild workspace
source /opt/ros/humble/setup.bash
colcon build --packages-select my_custom_interfaces_pkg formation_coordinator_pkg agent_control_pkg
source install/setup.bash

# Run quick test to validate
./scripts/quick_test.sh
```

---

## 🚀 **NEW: Long Scenario Demo (5-Minute Zigzag Trajectory)**

### **Enhanced Thesis Demo - Advanced Features**

**NEW in this version:**
- ✅ **Waypoint-based zigzag trajectory** (5 phases, 300 seconds)
- ✅ **3D motion** with altitude changes (1m → 3m → 1.5m → 2.5m → 2m)
- ✅ **Checkpoint system** (automatic saves every 60s)
- ✅ **Enhanced metrics logging** (IAE, ITAE, ISE, step info)
- ✅ **Phase-by-phase analysis** tools
- ✅ **Real-time dashboard** with progress indicators

### **Quick Start - Long Scenario**

```bash
cd /home/mmf/Documents/GitHub/multi-agent-formation-control-under-disturbances

# Full 5-minute demo (all features)
./scripts/run_long_thesis_demo.sh

# Quick 1-minute test
./scripts/run_long_thesis_demo.sh --test-mode

# Headless mode (maximum FPS)
./scripts/run_long_thesis_demo.sh --headless
```

### **What's Different?**

**Trajectory:**
- **Phase 1 (0-60s)**:   Start → WP1 | Forward 10m, lateral 6m, climb 1m | Light wind (2N)
- **Phase 2 (60-120s)**: WP1 → WP2   | Forward 10m, zigzag 4m, climb 1m  | Medium wind (4N)
- **Phase 3 (120-180s)**: WP2 → WP3   | Forward 5m, zigzag 5m, descend 1.5m | Heavy gusts (6N)
- **Phase 4 (180-240s)**: WP3 → WP4   | **Backward 10m**, return 3m, climb 1m | Variable wind
- **Phase 5 (240-300s)**: WP4 → Final | Forward 5m, stabilize | Stress test

**Output Structure:**
```
thesis_data/YYYY-MM-DD/HH-MM-SS_long_scenario/
├── raw_data/                    # Full 300s CSV files
│   ├── agent_0_pidfuzzy_full.csv
│   ├── agent_3_pd_full.csv
│   └── agent_6_pid_full.csv
├── checkpoints/                 # Phase-by-phase snapshots
│   ├── phase1_60s/
│   ├── phase2_120s/
│   ├── phase3_180s/
│   ├── phase4_240s/
│   └── phase5_300s/
└── final_results/               # Summary + analysis
    ├── summary.txt
    └── *.csv (complete datasets)
```

### **Analysis Tools**

After running the demo:

```bash
# 1. Checkpoint analysis (phase-by-phase metrics)
python3 analysis/analyze_checkpoint_data.py thesis_data/YYYY-MM-DD/HH-MM-SS_long_scenario

# 2. Generate plots (trajectory, error, comparison)
python3 analysis/plot_long_scenario.py thesis_data/YYYY-MM-DD/HH-MM-SS_long_scenario

# 3. Thesis report (LaTeX tables + markdown summary)
python3 analysis/generate_thesis_report.py thesis_data/YYYY-MM-DD/HH-MM-SS_long_scenario

# 4. Controller comparison (all phases)
python3 analysis/compare_phases.py thesis_data/YYYY-MM-DD/HH-MM-SS_long_scenario
```

### **Technical Details**

**Waypoint System:**
- Linear interpolation between waypoints
- Smooth transitions (no discontinuities)
- 3D trajectory support (X, Y, Z)
- Configurable via YAML (`waypoints.times`, `waypoints.x`, `waypoints.y`, `waypoints.z`)

**Enhanced Metrics:**
- **IAE** (Integral Absolute Error) - accumulated tracking error
- **ITAE** (Integral Time-weighted Absolute Error) - penalizes late errors
- **ISE** (Integral Square Error) - penalizes large errors
- **Step Info** - overshoot, settling time, rise time

**Configuration Files:**
```
other_packages/formation_coordinator_pkg/config/
├── long_scenario_group0_fuzzy.yaml   # PID+Fuzzy waypoints
├── long_scenario_group1_pd.yaml      # PD waypoints
└── long_scenario_group2_pid.yaml     # PID waypoints
```

### **Expected Results**

With the long scenario, you'll be able to:
- **Compare controllers** across different trajectory segments
- **Analyze phase transitions** (how each controller adapts)
- **Measure disturbance rejection** under varying wind conditions
- **Generate publication-ready figures** for thesis

**Hypothesis:**
- PID+Fuzzy should outperform in **heavy wind phases** (Phase 3, 5)
- PD should have **fastest response** in clean trajectory segments
- PID should provide **best overall balance** across all phases

---

## Highlights
- 2D dynamics with ambient wind velocity, linear->quadratic drag blend, asymmetric actuator lag (up/down), semi-implicit Euler
- Optional constant wind acceleration bias (wind.ax, wind.ay) for integral-action testing
- Controller selection from YAML: `pid | p | pi | pd | fuzzy | pid_fuzzy`
- Hybrid PID+Fuzzy (GT2) with mix gains and separate contribution logging
- Model-based feed-forward (drag + wind bias cancellation)
- Target prefilter (2x first-order, critical-damped approximation) to reduce overshoot
- CSV outputs under `outputs/simulations/dynamics2d/YYYYMMDD/run_###__/`
- Python tooling for metrics, plotting, batch summaries and final reports

## What’s New (2025-10)
- Output management
  - `output_settings.run_label` → run folder named as `run_###__<label>`
  - `output_settings.auto_plot` → PNG is saved automatically after each run
  - `output_settings.max_runs_per_day` → keep last N runs per day (older are pruned)
- Wind options
  - YAML `wind.vx`, `wind.vy` (ambient velocity)
  - NEW: `wind.ax`, `wind.ay` (constant acceleration bias)
- CSV additions
  - CSV header includes `ax_wind, ay_wind` and hybrid contributions `ax_pid, ay_pid, ax_fuzzy, ay_fuzzy`, plus `ax_ff, ay_ff`
- Analysis utilities (under `analysis/`)
  - `plot_dynamics_2d.py --t_end <T>`: plot a single CSV with optional time window
  - `plot_compare_positions.py`: compare multiple runs (position focus, with zoom 0–20 s)
  - `compute_pid_from_specs.py`: actuator-aware closed-form PID from (Mp, Ts5)
  - `compute_pid_from_velocity.py`: velocity/distance-based PID design (time-optimal inspired)
  - `auto_tune_pid.py`: small grid search for PID specs (Mp, Ts5, beta, tau_mode)
  - `auto_tune_p_pi.py`: grid search for P / PI (for baseline evidence)
  - `summarize_runs.py`: rank all runs (per day) by constraints (OS, Ts5) and metrics
  - `collect_final_report.py`: copy best runs (CSV+PNG) to `outputs/final_report/<day>`
  - `cleanup_runs.py`: move clutter runs to `outputs/simulations/dynamics2d/_trash/<day>`

## Repo Map
- Dynamics core: agent_control_pkg/include/agent_control_pkg/drone_dynamics_2d.hpp, agent_control_pkg/src/drone_dynamics_2d.cpp
- Controllers
  - PID core: agent_control_pkg/include/agent_control_pkg/pid_controller.hpp, agent_control_pkg/src/pid_controller.cpp
  - Interface: agent_control_pkg/include/agent_control_pkg/controllers/controller_base.hpp
  - PID adapter: agent_control_pkg/include/agent_control_pkg/controllers/pid_adapter.hpp
  - GT2 fuzzy: agent_control_pkg/include/agent_control_pkg/gt2_fuzzy_logic_system.hpp, agent_control_pkg/src/gt2_fuzzy_logic_system.cpp
  - Fuzzy adapter: agent_control_pkg/include/agent_control_pkg/controllers/fuzzy_gt2_adapter.hpp, agent_control_pkg/src/controllers/fuzzy_gt2_adapter.cpp
  - Hybrid PID+Fuzzy: agent_control_pkg/include/agent_control_pkg/controllers/combined_pid_fuzzy_adapter.hpp, agent_control_pkg/src/controllers/combined_pid_fuzzy_adapter.cpp
- Tester: agent_control_pkg/src/dynamics_2d_test_main.cpp
- Configs: agent_control_pkg/config/dynamics_2d_test.yaml, agent_control_pkg/config/dynamics_2d_wind.yaml, agent_control_pkg/config/fuzzy_params.yaml
- Plotting: analysis/plot_dynamics_2d.py
- Docs: docs/dynamics/quickstart_dynamics2d.md, docs/dynamics/dynamics2d_system_and_control.md

## Build (Windows, VS 2022)
```
cmake -S agent_control_pkg -B build -G "Visual Studio 17 2022"
cmake --build build --config Debug --target dynamics_2d_tester
```

## ROS2 Build & Launch (Linux, ROS 2 Humble)

### Build from Source
```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Navigate to workspace
cd /home/mmf/Documents/GitHub/multi-agent-formation-control-under-disturbances

# Build all required packages
colcon build --packages-select my_custom_interfaces_pkg formation_coordinator_pkg agent_control_pkg

# Source workspace
source install/setup.bash
```

### Launch Options

**Recommended: Full Gazebo Demo** (ROS2 + Gazebo + RViz2):
```bash
./scripts/run_demo.sh                    # Full GUI demo with PID controller
./scripts/run_demo.sh --controller pd    # PD controller
./scripts/run_demo.sh --headless         # Headless mode (no Gazebo GUI)
```

**Alternative: ROS2-Only Tests** (No Gazebo):
```bash
# Single agent test (controller + coordinator, no physics simulation)
ros2 launch agent_control_pkg single_agent_test.launch.py

# Multi-agent formation (3 agents under /agent_i namespaces)
ros2 launch agent_control_pkg multi_agent_formation.launch.py
```

**Configuration Files:**
- Controller params: `agent_control_pkg/config/ros2/agent_controller_*.yaml`
- Formation params: `other_packages/formation_coordinator_pkg/config/formation_config.yaml`

## Gazebo Simulation (ROS2 + 3D Visualization)

### 🎓 Thesis Multi-Controller Comparison Demo (NEW!)

**Compare All 5 Controllers Simultaneously:**
```bash
source /opt/ros/humble/setup.bash
cd /home/mmf/Documents/GitHub/multi-agent-formation-control-under-disturbances
source install/setup.bash
./scripts/run_thesis_demo.sh
```

**Features:**
- ✅ **5 drones** running simultaneously, each with different controller
- ✅ **Color-coded drones**: Red (P), Green (PI), Blue (PD), Yellow (PID), Magenta (PID+Fuzzy)
- ✅ **Triangle formation** with first 3 drones
- ✅ **High FPS** optimization for smooth performance
- ✅ **Real-time metrics** comparison
- ✅ **Individual trajectory** visualization for each controller

**Performance Modes:**
```bash
./scripts/run_thesis_demo.sh                # Full demo (GUI + RViz)
./scripts/run_thesis_demo.sh --headless     # No Gazebo GUI (higher FPS)
./scripts/run_thesis_demo.sh --no-rviz      # No RViz2 (save resources)
./scripts/run_thesis_demo.sh --performance  # Headless + No RViz (maximum FPS)
./scripts/run_thesis_demo.sh --triangle     # Only 3 drones (triangle formation)
```

---

### 🚀 Quick Start - Single Drone Demo (30 Seconds)

**One-Command Launch:**
```bash
source /opt/ros/humble/setup.bash
cd /home/mmf/Documents/GitHub/multi-agent-formation-control-under-disturbances
source install/setup.bash
./scripts/run_demo.sh
```

**Or Step-by-Step:**
```bash
# 1. Setup environment
source /opt/ros/humble/setup.bash
cd /home/mmf/Documents/GitHub/multi-agent-formation-control-under-disturbances
source install/setup.bash

# 2. Clean old processes (automatic in run_demo.sh)
pkill -9 gzserver gzclient 2>/dev/null || true

# 3. Launch simulation
ros2 launch agent_control_pkg demo_presentation.launch.py
```

### 🎛️ Launch Options

**Different Modes:**
```bash
# Headless mode (faster, no Gazebo GUI)
./scripts/run_demo.sh --headless

# No RViz2 (terminal only)
./scripts/run_demo.sh --no-rviz
```

**Different Controllers:**
```bash
./scripts/run_demo.sh --controller pid        # PID (default, Kp=0.538, Ki=0.145, Kd=1.368)
./scripts/run_demo.sh --controller p          # P-only controller
./scripts/run_demo.sh --controller pi         # PI controller
./scripts/run_demo.sh --controller pd         # PD controller
./scripts/run_demo.sh --controller pid_fuzzy  # Hybrid PID+Fuzzy (GT2)

# Or directly with ros2 launch:
ros2 launch agent_control_pkg demo_presentation.launch.py controller_config:=pid
ros2 launch agent_control_pkg demo_presentation.launch.py controller_config:=p_only
ros2 launch agent_control_pkg demo_presentation.launch.py controller_config:=pd
ros2 launch agent_control_pkg demo_presentation.launch.py controller_config:=pi
ros2 launch agent_control_pkg demo_presentation.launch.py controller_config:=pid_fuzzy
```

**Different Worlds:**
```bash
# Minimal world (fast startup, default)
ros2 launch agent_control_pkg demo_presentation.launch.py

# Professional world (visual aids, grid, axes, scale markers)
ros2 launch agent_control_pkg demo_presentation.launch.py world_file:=demo_presentation.world
```

### 📊 What's Included

- ✅ **RViz2 visualization** with real-time trajectory path tracking
- ✅ **Real-time metrics publisher** (error, RMSE, IAE, ITAE, settling time)
- ✅ **Multiple tuned controllers** (P, PI, PD, PID, PID+Fuzzy)
  - Default PID gains: Kp=0.538, Ki=0.145, Kd=1.368 (~10% overshoot, 4s settling)
- ✅ **Enhanced quadrotor model** with improved physics and visual indicators
- ✅ **SimpleDronePlugin** with force-based dynamics, damping, altitude PD control
- ✅ **Two world options:**
  - `minimal_test.world` → Fast startup, simple visuals (default)
  - `demo_presentation.world` → Professional presentation mode with grid/axes
- ✅ **Path visualizer node** for trajectory recording
- ✅ **Metrics publisher node** for performance monitoring

### 📈 Monitor Real-Time Metrics

**Terminal 2 (Optional):**
```bash
# Performance metrics (error, RMSE, IAE, ITAE, settling time)
ros2 topic echo /agent_0/metrics

# Trajectory path
ros2 topic echo /agent_0/path

# Controller diagnostics (PID/Fuzzy contributions)
ros2 topic echo /agent_0/diagnostics

# Drone position
ros2 topic echo /agent_0/odom --field pose.pose.position

# Target position
ros2 topic echo /agent_0/target_pose --field pose.position
```

### System Architecture (3 Layers)

The simulation uses a hierarchical control architecture:

1. **Formation Coordinator Node** (`formation_coordinator_pkg`)
   - Generates reference trajectories
   - Publishes `/agent_0/target_pose` (geometry_msgs/PoseStamped)
   - Configurable via `formation_coordinator_pkg/config/formation_config.yaml`

2. **Agent Controller Node** (`agent_control_pkg`)
   - Reads odometry: `/agent_0/odom` (nav_msgs/Odometry)
   - Reads target: `/agent_0/target_pose` (geometry_msgs/PoseStamped)
   - Computes control using PID/Fuzzy/Hybrid controllers
   - Publishes acceleration command: `/agent_0/cmd_accel` (geometry_msgs/Accel)
   - Publishes diagnostics: `/agent_0/diagnostics` (custom diagnostics message)

3. **Gazebo Physics Simulation** + **SimpleDronePlugin**
   - Applies force-based dynamics (F=ma)
   - Reads `/agent_0/cmd_accel` and applies forces to drone model
   - Publishes state feedback: `/agent_0/odom` (nav_msgs/Odometry)
   - Includes damping, altitude control, and realistic physics

4. **Visualization & Metrics** (Optional)
   - **Path Visualizer Node**: Records trajectory → `/agent_0/path` (nav_msgs/Path)
   - **Metrics Publisher Node**: Computes real-time performance → `/agent_0/metrics`
   - **RViz2**: 3D visualization with custom display configuration

### Configuration Files

**Controller Configurations** (`agent_control_pkg/config/ros2/`):
- `agent_controller_pid.yaml` → PID controller (Kp=0.538, Ki=0.145, Kd=1.368)
- `agent_controller_p_only.yaml` → Proportional-only controller
- `agent_controller_pi.yaml` → PI controller (no derivative)
- `agent_controller_pd.yaml` → PD controller (no integral)
- `agent_controller_pid_fuzzy.yaml` → Hybrid PID+Fuzzy (GT2) controller
- `agent_controller_default.yaml` → Default configuration (alias for PID)

**World Files** (`agent_control_pkg/worlds/`):
- `minimal_test.world` → Fast startup, basic visuals, single drone at origin
- `demo_presentation.world` → Professional mode with grid, axes, scale markers

**Key Components**:
- **Physics Plugin**: [simple_drone_plugin.cpp](agent_control_pkg/plugins/simple_drone_plugin.cpp)
  - Force-based dynamics (F=ma)
  - Linear and attitude damping
  - Altitude PD control (Kp≈8, Kd≈3) for level flight
- **Launch File**: [demo_presentation.launch.py](agent_control_pkg/launch/demo_presentation.launch.py)
  - Configurable controller selection via `controller_config` argument
  - Gazebo server/client management
  - ROS2 node orchestration with timed startup
  - RViz2 integration with custom config

### Monitoring & Debugging
```bash
# Check topics
ros2 topic list | grep agent_0

# Monitor real-time data
ros2 topic echo /agent_0/target_pose  # Formation reference
ros2 topic echo /agent_0/odom         # Drone state from Gazebo
ros2 topic echo /agent_0/cmd_accel    # Controller output
ros2 topic echo /agent_0/metrics      # Performance metrics (RMSE, IAE, ITAE)
ros2 topic echo /agent_0/path         # Trajectory path visualization

# Verify PID parameters loaded correctly
ros2 param get /agent_0/agent_controller pid.kp  # Should return 0.538
ros2 param list | grep agent_0        # List all parameters

# Check controller diagnostics
ros2 topic echo /agent_0/diagnostics  # PID/Fuzzy contributions

# Node information
ros2 node list                        # List all active nodes
ros2 node info /agent_0/agent_controller  # Controller node details
```

### CSV Data Analysis
The logging script captures synchronized data for post-processing:
```bash
# Columns: stamp, pos_xyz, vel_xyz, cmd_accel_xyz, target_xyz
python3 scripts/log_simulation_csv.py --output outputs/logs/test_run.csv

# Plot with existing analysis tools
python analysis/plot_dynamics_2d.py --csv outputs/logs/test_run.csv
```

### Troubleshooting

**Common Issues:**
- **Oscillation/Instability**:
  - Check if PID gains loaded correctly: `ros2 param get /agent_0/agent_controller pid.kp`
  - Try different controller config: `./scripts/run_demo.sh --controller pd`
  - Verify target is reachable: `ros2 topic echo /agent_0/target_pose`

- **Gazebo frozen/crashed**:
  - Kill processes: `pkill -9 gzserver gzclient`
  - Remove lock files: `rm -rf /tmp/.gazebo*`
  - Restart launch: `./scripts/run_demo.sh`

- **No movement/drone not responding**:
  - Verify topics are publishing: `ros2 topic hz /agent_0/cmd_accel`
  - Check if plugin loaded: Look for "SimpleDronePlugin: Loaded" in terminal
  - Verify odom is publishing: `ros2 topic hz /agent_0/odom`

- **Config changes not applied**:
  - Rebuild package: `colcon build --packages-select agent_control_pkg`
  - Re-source workspace: `source install/setup.bash`
  - Clear CMake cache if needed: `rm -rf build/agent_control_pkg`

- **RViz2 not showing trajectory**:
  - Check path topic: `ros2 topic hz /agent_0/path`
  - Verify path_visualizer_node is running: `ros2 node list | grep path_visualizer`
  - Reload RViz config: File → Open Config → `agent_control_pkg/rviz/formation_demo.rviz`

## Formation Comparison Demo (9 drones, 3 controller groups)

Compare PID+Fuzzy, PD, and PID groups moving as triangle formations along x under wind.

- World: `agent_control_pkg/worlds/formation_comparison.world`
- Launch: `agent_control_pkg/launch/formation_comparison_demo.launch.py`
- Coordinators and configs:
  - Group 0 (PID+Fuzzy): `other_packages/formation_coordinator_pkg/config/formation_group0_fuzzy.yaml` (lane y≈−4)
  - Group 1 (PD):        `other_packages/formation_coordinator_pkg/config/formation_group1_pd.yaml` (lane y≈0)
  - Group 2 (PID):       `other_packages/formation_coordinator_pkg/config/formation_group2_pid.yaml` (lane y≈+4)

Dynamic center motion
- Each group’s formation center moves from x≈−10 → +5 at 0.8 m/s (see `motion.*` parameters in YAMLs).
- Controllers per group:
  - PID+Fuzzy (mix.k_fuzzy=0.7, wind input enabled)
  - PD (derivative filter enabled)
  - PID (stronger derivative filter alpha)

Run
```
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to formation_coordinator_pkg agent_control_pkg
source install/setup.bash
ros2 launch agent_control_pkg formation_comparison_demo.launch.py
```

Headless
```
ros2 launch agent_control_pkg formation_comparison_demo.launch.py gazebo_gui:=false rviz:=false
```

Verify targets
```
ros2 param get /formation_0/formation_coordinator_group0 motion.enable  # true
ros2 topic echo /agent_0/target_pose -n 1            # PoseStamped with increasing x
ros2 topic info /agent_0/target_pose -v              # Publisher from formation_0, Best Effort
```

Notes
- QoS: Coordinators publish Best Effort; controllers subscribe with SensorData QoS. RViz may warn about reliability mismatch; it’s benign.
- Moving targets: “settled=NO” in metrics is expected; use RMSE/IAE trends to compare tracking.
- PID+Fuzzy advantage appears under stronger wind bias/gusts; tune `mix.k_fuzzy` (0.5→1.0) and fuzzy output scaling (±6..10 m/s²) for more impact.

## Run
```
# No-wind
build/Debug/dynamics_2d_tester.exe agent_control_pkg/config/dynamics_2d_test.yaml

# Wind
build/Debug/dynamics_2d_tester.exe agent_control_pkg/config/dynamics_2d_wind.yaml
```

CSV path: outputs/simulations/dynamics2d/YYYYMMDD/run_###/run_###.csv

## Plot
```
python analysis/plot_dynamics_2d.py --csv outputs/simulations/dynamics2d/YYYYMMDD/run_###/run_###.csv
```
The plot PNG is saved next to the CSV.

### Position Comparison Plots
```
# 3 runs side-by-side (PD, PID, PID+FLS) up to t=20 s
python analysis/plot_compare_positions.py \
  --csv \
    outputs/simulations/dynamics2d/20251014/run_004__pd_vel_2p5/run_004__pd_vel_2p5.csv \
    outputs/simulations/dynamics2d/20251014/run_005__pid_vel_2p5/run_005__pid_vel_2p5.csv \
    outputs/simulations/dynamics2d/20251014/run_008__pidf_kf10_vel_2p5/run_008__pidf_kf10_vel_2p5.csv \
  --labels PD PID PIDF_k1 \
  --t_end 20 \
  --out outputs/simulations/dynamics2d/20251014/compare_positions_20s.png
```

### Summarize and Collect
```
# Rank runs for a day
python analysis/summarize_runs.py --day 20251014 --os-max 20 --ts-max 8 --top 10

# Collect best runs to a tidy folder
python analysis/collect_final_report.py --day 20251014 --labels pd_vel,pid_vel,pidf_ --top 1
```

## Example Scenarios (Thesis‑ready)
- **No-wind (reference)**: PD often excels (plant is 1/s^2, no integral action needed)
- **Persistent bias** (constant acceleration ax/ay): PID better than PD on IAE; PID+FLS better than PID
- **Time-varying wind** (sinusoidal, step, gust): Tests disturbance rejection under dynamic conditions
- **Robustness test** (wind + bias): PID+FLS > PID > PD on IAE (demonstrates hybrid advantage)

Use provided YAMLs under `agent_control_pkg/config/experiments/`:
- `*_wbias.yaml` → adds constant bias (`wind.ax/ay`) for integral action testing
- `*_wind.yaml` → adds ambient wind velocity (steady-state, absorbed by drag model)
- `*_wind_step.yaml` → step wind disturbance (t=5s → 8 m/s)
- `*_wind_tv.yaml` → sinusoidal wind (5±3 m/s @ 0.5 Hz)
- `*_wind_gust.yaml` → stochastic gust (turbulence model)
- Pick labels and autoplot via `output_settings` in YAML

## Physics and Configuration
- Core files
  - Dynamics: `agent_control_pkg/include/agent_control_pkg/drone_dynamics_2d.hpp`, `agent_control_pkg/src/drone_dynamics_2d.cpp`
  - Controllers: PID, GT2 FLS adapters under `include/agent_control_pkg/controllers/` and `src/controllers/`
  - Tuning: `analysis/compute_pid_from_specs.py`, `analysis/compute_pid_from_velocity.py`, `analysis/auto_tune_pid.py`
- Model: position double-integrator with first-order actuator lag (separate up/down), drag blend (linear->quadratic by speed threshold)
- Feed-forward: drag + wind bias cancellation (`feedforward` section in YAML)
- Target prefilter: two cascaded first-order filters (`target_prefilter: {enabled, tau}`)
- Output management: `output_settings: {run_label, auto_plot, max_runs_per_day}`

See also: `docs/dynamics/advanced_control_notes.md` for prefilter/FF details and closed-form tuning formulas.

## Tuning Guide (Overshoot and Settling)
- Closed-form actuator-aware mapping from specs (Mp, Ts5) to PID (use `compute_pid_from_specs.py`)
- Velocity-based design from distance and desired cruise speed (use `compute_pid_from_velocity.py`)
- Practical tips
  - Increase damping (zeta) first to reduce OS, then adjust wn to hit Ts5
  - Setpoint weighting (2-DOF PID) and prefilter reduce OS without hurting disturbance rejection
  - Use FF to reduce controller workload in wind/bias cases
  - For hybrid, scan `k_fuzzy` in 0.2..1.2, ensure output MF peaks are near +/- 6..10 m/s^2

## ROS2 Roadmap (next phase)
- Architecture
  - Node `agent_controller_node`: subscribes to position setpoints, publishes attitude/thrust commands
  - Node `formation_coordinator_node`: generates waypoints/centers (already present as stubs)
  - Messages: positions (geometry_msgs/PoseStamped), accelerations or attitude setpoints
- Mapping accel->attitude/thrust
  - theta_cmd ~= ax_des/g, phi_cmd ~= -ay_des/g (yaw fixed), T_cmd = m*(g+az_des)
  - enforce angle/rate limits and slew-rate on commands
- Integration path
  - Extract controlloop core (PID, FLS, prefilter, FF) into a reusable library
  - Wrap as ROS2 node with YAML params mirroring this repo
  - Optional SITL: PX4 + Gazebo/Ignition for 3D validation

## Roadmap / What’s Next
- Add 2-DOF PID setpoint weighting to PIDAdapter and YAML key `pid.setpoint_weight_b`
- k_fuzzy fine scan and MF scaling audit under wind+bias
- Wind time-window disturbance scenarios and multi-drone formation runs
- Final figure pack and LaTeX-ready tables from `summarize_runs.py`


## Controller Selection (YAML)
```
controller_settings:
  type: pid_fuzzy   # pid | p | pi | pd | fuzzy | pid_fuzzy
  pid:
    kp: 0.538
    ki: 0.145
    kd: 1.368
    enable_derivative_filter: true
  fls:
    enable: true
    params_file: fuzzy_params.yaml
  mix:
    k_pid: 1.0
    k_fuzzy: 1.0
```
Notes
- Fuzzy params define IT2 triangular FOUs for variables error, dError, optional wind, and an output variable (e.g., output or correction). Rules are [error_set, dError_set, wind_set, output_set].
- Hybrid sums u_pid and u_fuzzy, then clamps to +-a_max.

## Roadmap / What’s Next
- Fuzzy MF/output scaling and `k_fuzzy` fine scan (0.2–1.2) under wind+bias
- Windowed wind disturbances (time windows) and multi‑drone formation tests
- Pack final figures (compare PNGs + summary CSVs) for the thesis results section
- Optional: script to auto‑export LaTeX‑ready tables from `summarize_runs.py`

## Dynamics (per axis, summary)
- Actuator (1st order, asym up/down): a_cmd_f[k] = a_cmd_f[k-1] + alpha*(a_cmd[k]-a_cmd_f[k-1]), alpha = clamp(dt/tau, 0,1)
- Relative airspeed: v_rel = v - v_wind, |v_rel| = hypot(v_rel,x, v_rel,y)
- Drag blend: a_drag = (1-w)*a_lin + w*a_quad; a_lin = -(c_lin/m)*v_rel, a_quad = -(c_quad/m)*|v_rel|*v_rel; w = clamp((|v_rel|-v_thr)/max(v_thr,1e-6),0,1)
- Semi-implicit Euler: v[k] = v[k-1] + a*dt, x[k] = x[k-1] + v[k]*dt

## Troubleshooting Fuzzy / Hybrid
If pid_fuzzy looks identical to pid:
- Scale output sets to [m/s^2] (e.g., peaks +-6..10). Too small -> u_fuzzy ~ 0.
- Ensure input sets cover the step range (e up to ~5 m; de/dt up to ~0.5..1.0 m/s).
- Increase k_fuzzy (e.g., 0.3..1.0).
- Make sure rule consequents match output set names exactly.

## Outputs & Hygiene
- outputs/ is git-ignored. CSV/plots live per-run under dated folders.
- Configs under agent_control_pkg/config. Docs under docs/.

## See Also
- docs/dynamics/quickstart_dynamics2d.md
- docs/dynamics/dynamics2d_system_and_control.md


## Time-varying Wind Model
Add `wind_model` section to YAML config for dynamic wind disturbances:

```yaml
wind_model:
  type: sinusoid  # Options: sinusoid | step | gust
  base_vx: 5.0    # Base wind velocity (m/s)
  base_vy: 0.0
  amp_vx: 3.0     # Amplitude for sinusoid/gust (m/s)
  amp_vy: 0.0
  frequency_hz: 0.5  # For sinusoid mode
  step_time: 5.0     # For step mode (seconds)
  step_vx: 8.0       # Step target velocity
  gust_std: 2.0      # For gust mode (standard deviation)
  gust_alpha: 0.95   # For gust mode (LPF smoothing)
```

Wind velocity is updated each simulation step and logged in CSV columns `vx_wind, vy_wind`.

## Thesis Deliverables (Results Section)

All comparison plots are available in `outputs/simulations/dynamics2d/20251015/`:

### Key Figures for Thesis
1. **Figure 1: No-wind baseline** - `compare_positions_20s_nowind.png`
   - Shows PD optimal performance (no persistent disturbance)
   - Reference for evaluating integral action necessity

2. **Figure 2: Persistent bias** - `compare_positions_20s_nowind_bias.png`
   - Demonstrates PID superiority over PD (31-37% IAE improvement)
   - Shows PID+Fuzzy further improves tracking (3-7% over PID)

3. **Figure 3: Step wind disturbance** - `compare_positions_20s_wind_step.png`
   - 8 m/s step at t=5s tests transient response
   - Validates controller robustness to sudden disturbances

4. **Figure 4: Stochastic gust** - `compare_positions_20s_wind_gust.png`
   - Turbulence model tests steady-state disturbance rejection
   - Shows hybrid controller advantage in noisy environments

5. **Figure 5: Time-varying sinusoidal wind** - `compare_positions_20s_wind_tv.png`
   - 5±3 m/s @ 0.5 Hz tests periodic disturbances
   - Demonstrates tracking performance under oscillatory conditions

6. **Figure 6: Wind + bias robustness** - `compare_positions_20s_wind_bias.png`
   - Combined persistent and dynamic disturbances
   - Validates PID+FLS > PID > PD ranking (robustness case)

### Metrics Summary
Use `analysis/summarize_runs.py` to generate LaTeX-ready tables:
```bash
python analysis/summarize_runs.py --day 20251015 --os-max 20 --ts-max 8 --top 20
```

Key metrics: Overshoot (%), Settling time (s), IAE, ITAE, RMSE
