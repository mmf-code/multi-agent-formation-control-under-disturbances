#!/usr/bin/env python3
"""
Automated Controller Tuning Script
Runs simulations with different parameters and collects metrics.
"""

import subprocess
import time
import os
import yaml
import json
import signal
from datetime import datetime
from pathlib import Path

# Paths
BASE_DIR = Path("/home/mmf/Documents/GitHub/multi-agent-formation-control-under-disturbances")
CONFIG_DIR = BASE_DIR / "agent_control_pkg/config"
RESULTS_DIR = BASE_DIR / "tuning_results"
RESULTS_DIR.mkdir(exist_ok=True)

# Default parameters (baseline)
BASELINE_PARAMS = {
    "pid": {"kp": 3.501, "ki": 1.946, "kd": 3.608},
    "pd": {"kp": 3.500, "ki": 0.0, "kd": 3.610},
    "fuzzy": {"k_fuzzy": 0.6, "wind_scalar": 1.5}
}

def kill_simulation():
    """Kill any running simulation processes."""
    subprocess.run("killall -9 gzserver agent_controller_node metrics_publisher_node formation_coordinator_node 2>/dev/null",
                   shell=True, capture_output=True)
    time.sleep(2)

def update_config(params):
    """Update the YAML config file with new parameters."""
    config_file = CONFIG_DIR / "crazyflie_9drone_params.yaml"

    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)

    # Update PID parameters for each group
    for i in range(9):
        agent_key = f"agent_{i}"
        if agent_key in config:
            group = i // 3
            if group == 0:  # PID+Fuzzy
                config[agent_key]["pid"]["kp"] = params["pid"]["kp"]
                config[agent_key]["pid"]["ki"] = params["pid"]["ki"]
                config[agent_key]["pid"]["kd"] = params["pid"]["kd"]
                if "fuzzy" in params:
                    config[agent_key]["mix"]["k_fuzzy"] = params["fuzzy"]["k_fuzzy"]
                    config[agent_key]["fuzzy"]["wind_scalar"] = params["fuzzy"]["wind_scalar"]
            elif group == 1:  # PD
                config[agent_key]["pid"]["kp"] = params["pd"]["kp"]
                config[agent_key]["pid"]["ki"] = 0.0
                config[agent_key]["pid"]["kd"] = params["pd"]["kd"]
            else:  # PID
                config[agent_key]["pid"]["kp"] = params["pid"]["kp"]
                config[agent_key]["pid"]["ki"] = params["pid"]["ki"]
                config[agent_key]["pid"]["kd"] = params["pid"]["kd"]

    with open(config_file, 'w') as f:
        yaml.dump(config, f, default_flow_style=False)

def run_simulation(duration=30):
    """Run simulation and collect metrics."""
    # Source and launch
    cmd = f"""
    source /opt/ros/humble/setup.bash && \
    source {BASE_DIR}/install/setup.bash && \
    timeout {duration + 10} ros2 launch agent_control_pkg formation_comparison_demo.launch.py \
        gazebo_gui:=false rviz:=false 2>&1
    """

    proc = subprocess.Popen(cmd, shell=True, executable='/bin/bash',
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                           preexec_fn=os.setsid)

    # Wait for simulation to start
    time.sleep(15)

    # Collect metrics for the duration
    metrics = collect_metrics(duration - 10)

    # Kill simulation
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    except:
        pass
    kill_simulation()

    return metrics

def collect_metrics(duration):
    """Collect metrics from ROS topics."""
    metrics = {0: [], 1: [], 2: []}  # Group 0, 1, 2

    cmd = f"""
    source /opt/ros/humble/setup.bash && \
    source {BASE_DIR}/install/setup.bash && \
    timeout {duration} ros2 topic echo /metrics/group_0 --field rmse 2>/dev/null &
    timeout {duration} ros2 topic echo /metrics/group_1 --field rmse 2>/dev/null &
    timeout {duration} ros2 topic echo /metrics/group_2 --field rmse 2>/dev/null
    """

    # Simpler approach: just record for duration and parse output
    start_time = time.time()
    while time.time() - start_time < duration:
        for group in range(3):
            try:
                result = subprocess.run(
                    f"source /opt/ros/humble/setup.bash && source {BASE_DIR}/install/setup.bash && "
                    f"timeout 1 ros2 topic echo /agent_{group*3}/metrics --once 2>/dev/null | grep rmse",
                    shell=True, executable='/bin/bash', capture_output=True, text=True
                )
                if result.stdout:
                    try:
                        rmse = float(result.stdout.strip().split(':')[-1])
                        metrics[group].append(rmse)
                    except:
                        pass
            except:
                pass
        time.sleep(2)

    # Calculate averages
    avg_metrics = {}
    for group, values in metrics.items():
        if values:
            avg_metrics[group] = {
                "avg_rmse": sum(values) / len(values),
                "min_rmse": min(values),
                "max_rmse": max(values),
                "samples": len(values)
            }
        else:
            avg_metrics[group] = {"avg_rmse": float('inf'), "samples": 0}

    return avg_metrics

def evaluate_params(params, duration=30):
    """Run simulation with params and return metrics."""
    print(f"\n{'='*60}")
    print(f"Testing: PID[Kp={params['pid']['kp']:.2f}, Ki={params['pid']['ki']:.2f}, Kd={params['pid']['kd']:.2f}]")
    print(f"         PD[Kp={params['pd']['kp']:.2f}, Kd={params['pd']['kd']:.2f}]")
    if 'fuzzy' in params:
        print(f"         Fuzzy[k={params['fuzzy']['k_fuzzy']:.2f}, wind_scalar={params['fuzzy']['wind_scalar']:.2f}]")
    print(f"{'='*60}")

    kill_simulation()
    update_config(params)

    # Rebuild to apply config changes
    subprocess.run(f"cd {BASE_DIR} && colcon build --symlink-install --packages-select agent_control_pkg 2>/dev/null",
                   shell=True, capture_output=True)

    metrics = run_simulation(duration)

    print(f"\nResults:")
    print(f"  Group 0 (PID+Fuzzy): RMSE={metrics[0].get('avg_rmse', 'N/A'):.3f}m")
    print(f"  Group 1 (PD):        RMSE={metrics[1].get('avg_rmse', 'N/A'):.3f}m")
    print(f"  Group 2 (PID):       RMSE={metrics[2].get('avg_rmse', 'N/A'):.3f}m")

    return metrics

def run_tuning_sweep():
    """Run parameter sweep for tuning."""
    results = []

    # Parameter variations to test
    test_configs = [
        # Baseline
        {"name": "baseline", "pid": {"kp": 3.5, "ki": 1.95, "kd": 3.6},
         "pd": {"kp": 3.5, "ki": 0.0, "kd": 3.6}, "fuzzy": {"k_fuzzy": 0.6, "wind_scalar": 1.5}},

        # Higher gains
        {"name": "high_gains", "pid": {"kp": 5.0, "ki": 3.5, "kd": 4.5},
         "pd": {"kp": 6.0, "ki": 0.0, "kd": 5.5}, "fuzzy": {"k_fuzzy": 0.8, "wind_scalar": 2.0}},

        # Medium gains
        {"name": "medium_gains", "pid": {"kp": 4.2, "ki": 2.8, "kd": 4.0},
         "pd": {"kp": 4.5, "ki": 0.0, "kd": 4.2}, "fuzzy": {"k_fuzzy": 0.7, "wind_scalar": 1.8}},

        # Higher Ki for wind rejection
        {"name": "high_ki", "pid": {"kp": 4.0, "ki": 4.0, "kd": 4.0},
         "pd": {"kp": 5.0, "ki": 0.0, "kd": 5.0}, "fuzzy": {"k_fuzzy": 0.8, "wind_scalar": 2.0}},

        # Higher Kd for damping
        {"name": "high_kd", "pid": {"kp": 4.5, "ki": 2.5, "kd": 5.5},
         "pd": {"kp": 5.0, "ki": 0.0, "kd": 6.0}, "fuzzy": {"k_fuzzy": 0.7, "wind_scalar": 1.5}},
    ]

    for config in test_configs:
        name = config.pop("name")
        metrics = evaluate_params(config, duration=40)

        result = {
            "name": name,
            "params": config,
            "metrics": metrics,
            "timestamp": datetime.now().isoformat()
        }
        results.append(result)

        # Save intermediate results
        with open(RESULTS_DIR / "tuning_results.json", 'w') as f:
            json.dump(results, f, indent=2, default=str)

    # Find best configuration
    print("\n" + "="*60)
    print("TUNING COMPLETE - Summary")
    print("="*60)

    for r in results:
        g0 = r['metrics'][0].get('avg_rmse', float('inf'))
        g1 = r['metrics'][1].get('avg_rmse', float('inf'))
        g2 = r['metrics'][2].get('avg_rmse', float('inf'))
        print(f"{r['name']:15s}: G0={g0:.3f}m, G1={g1:.3f}m, G2={g2:.3f}m")

    # Check thesis requirements: PID > PD, PID+Fuzzy > PID (under wind)
    print("\nThesis Validation:")
    for r in results:
        g0 = r['metrics'][0].get('avg_rmse', float('inf'))  # PID+Fuzzy
        g1 = r['metrics'][1].get('avg_rmse', float('inf'))  # PD
        g2 = r['metrics'][2].get('avg_rmse', float('inf'))  # PID

        pid_better_than_pd = g2 < g1
        fuzzy_better_than_pid = g0 < g2

        status = "PASS" if (pid_better_than_pd and fuzzy_better_than_pid) else "FAIL"
        print(f"  {r['name']}: PID<PD={pid_better_than_pd}, Fuzzy<PID={fuzzy_better_than_pid} [{status}]")

    return results

if __name__ == "__main__":
    print("="*60)
    print("AUTOMATED CONTROLLER TUNING")
    print("="*60)
    run_tuning_sweep()
