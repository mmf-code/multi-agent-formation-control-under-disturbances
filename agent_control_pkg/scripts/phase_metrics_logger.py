#!/usr/bin/env python3
"""
Phase Metrics Logger for 6-Phase Thesis Test Framework

Subscribes to all agent metrics topics and exports:
- Per-agent CSV files with full time series
- Group summary CSV with aggregated metrics
- Phase metadata JSON

Output structure:
    results/
    ├── phase_{id}/
    │   ├── run_{k}/
    │   │   ├── agent_0_pd.csv
    │   │   ├── agent_3_pid.csv
    │   │   ├── agent_6_it2.csv
    │   │   ├── agent_9_gt2.csv
    │   │   ├── group_summary.csv
    │   │   ├── wind_data.csv
    │   │   └── phase_metadata.json
    │   └── summary.csv (across all runs)
    └── final_report.md

Usage:
    ros2 run agent_control_pkg phase_metrics_logger.py --ros-args \
        -p phase_id:=3 -p run_index:=1 -p output_dir:='results'
"""

import os
import sys
import json
import csv
import time
from datetime import datetime
from dataclasses import dataclass, field, asdict
from typing import Dict, List, Optional, Any, Tuple
from collections import defaultdict

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Vector3
from my_custom_interfaces_pkg.msg import MetricsData


@dataclass
class AgentMetrics:
    """Accumulated metrics for one agent."""
    agent_id: str
    controller_type: str
    group_id: int
    samples: int = 0

    # Time series storage
    timestamps: List[float] = field(default_factory=list)
    errors_x: List[float] = field(default_factory=list)
    errors_y: List[float] = field(default_factory=list)
    errors_mag: List[float] = field(default_factory=list)
    rmse_x: List[float] = field(default_factory=list)
    rmse_y: List[float] = field(default_factory=list)
    rmse_total: List[float] = field(default_factory=list)  # Per-waypoint RMSE (for CSV)
    iae_x: List[float] = field(default_factory=list)
    iae_y: List[float] = field(default_factory=list)
    itae_x: List[float] = field(default_factory=list)
    itae_y: List[float] = field(default_factory=list)
    settling_times: List[float] = field(default_factory=list)
    max_overshoots_x: List[float] = field(default_factory=list)
    max_overshoots_y: List[float] = field(default_factory=list)
    control_efforts: List[float] = field(default_factory=list)

    # Mission-wide metrics (accumulated across all waypoints)
    mission_rmse: List[float] = field(default_factory=list)
    mission_itae_x: List[float] = field(default_factory=list)
    mission_itae_y: List[float] = field(default_factory=list)

    # Final summary metrics
    final_rmse: float = 0.0  # Will use mission_rmse, not per-waypoint rmse_total
    final_itae: float = 0.0
    final_settling_time: float = 0.0  # Uses last_violation_time (OFFLINE)
    final_max_overshoot: float = 0.0  # Uses peak_error (OFFLINE) - NOT true overshoot
    control_effort_iae: float = 0.0   # Integral of |u| dt

    # OFFLINE computed metrics (from error time-series, not from publisher)
    peak_error: float = 0.0           # max(|e|) over mission (NOT overshoot!)
    last_violation_time: float = 0.0  # last t where |e| > threshold (settling-like)
    mean_absolute_error: float = 0.0  # MAE over mission

    # STEADY-STATE metrics (computed for t > steady_state_start_sec)
    # These exclude startup transient for fair controller comparison
    ss_rmse: float = 0.0              # RMSE for t > t_ss
    ss_mae: float = 0.0               # MAE for t > t_ss
    ss_peak_error: float = 0.0        # Peak error for t > t_ss
    ss_large_errors: int = 0          # Count of errors > 2m in steady-state


@dataclass
class GroupSummary:
    """Summary metrics for a controller group."""
    group_id: int
    controller_type: str
    agent_count: int = 0
    mean_rmse: float = 0.0
    std_rmse: float = 0.0
    mean_itae: float = 0.0
    std_itae: float = 0.0
    mean_settling_time: float = 0.0      # Uses last_violation_time
    mean_max_overshoot: float = 0.0      # Uses peak_error (NOT true overshoot)
    mean_control_effort_iae: float = 0.0  # Integral of |u| dt
    agents_settled: int = 0
    settled_ratio: float = 0.0
    # OFFLINE computed metrics (aggregated from agents)
    mean_peak_error: float = 0.0          # max(|e|) over mission
    mean_last_violation: float = 0.0      # last t where |e| > threshold
    mean_mae: float = 0.0                 # Mean Absolute Error
    # STEADY-STATE metrics (t > steady_state_start_sec, excludes startup transient)
    mean_ss_rmse: float = 0.0             # Steady-state RMSE
    std_ss_rmse: float = 0.0              # Std dev of SS-RMSE
    mean_ss_mae: float = 0.0              # Steady-state MAE
    mean_ss_peak: float = 0.0             # Steady-state peak error
    total_ss_large_errors: int = 0        # Total large errors (>2m) in steady-state


# Agent to controller type mapping for 12-drone 4-group setup
AGENT_CONTROLLER_MAP = {
    "agent_0": ("PD", 0), "agent_1": ("PD", 0), "agent_2": ("PD", 0),
    "agent_3": ("PID", 1), "agent_4": ("PID", 1), "agent_5": ("PID", 1),
    "agent_6": ("IT2", 2), "agent_7": ("IT2", 2), "agent_8": ("IT2", 2),
    "agent_9": ("GT2", 3), "agent_10": ("GT2", 3), "agent_11": ("GT2", 3),
}


class PhaseMetricsLogger(Node):
    """ROS2 node for logging phase metrics to CSV."""

    def __init__(self):
        super().__init__("phase_metrics_logger")

        # Declare parameters
        self.declare_parameter("phase_id", 1)
        self.declare_parameter("run_index", 1)
        self.declare_parameter("output_dir", "results")
        self.declare_parameter("duration_sec", 60)
        self.declare_parameter("num_agents", 12)
        self.declare_parameter("steady_state_start_sec", 15.0)  # Exclude startup transient

        # Get parameters
        self.phase_id = self.get_parameter("phase_id").value
        self.run_index = self.get_parameter("run_index").value
        self.output_dir = self.get_parameter("output_dir").value
        self.duration_sec = self.get_parameter("duration_sec").value
        self.num_agents = self.get_parameter("num_agents").value
        self.steady_state_start_sec = self.get_parameter("steady_state_start_sec").value

        # Setup output directory
        self.run_dir = os.path.join(
            self.output_dir,
            f"phase_{self.phase_id}",
            f"run_{self.run_index}"
        )
        os.makedirs(self.run_dir, exist_ok=True)

        # Initialize storage
        self.agent_metrics: Dict[str, AgentMetrics] = {}
        self.wind_data: List[Dict[str, float]] = []
        self.start_time_ns: Optional[int] = None  # Will be set on first message
        self.end_time: Optional[float] = None
        self.first_message_received: bool = False  # Phase 5 fix: start on first msg

        # Control effort tracking (subscribe to cmd_accel topics)
        self.cmd_data: Dict[str, List[Tuple[float, float]]] = defaultdict(list)
        self.cmd_subs = []

        # Check if using simulation time (already declared by ROS2)
        try:
            self.use_sim_time = self.get_parameter("use_sim_time").value
        except Exception:
            self.use_sim_time = True

        # QoS profile for reliable data
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribe to all agent metrics
        self.metrics_subs = []
        for i in range(self.num_agents):
            agent_id = f"agent_{i}"
            ctrl_type, group_id = AGENT_CONTROLLER_MAP.get(
                agent_id, ("UNKNOWN", -1)
            )

            # Initialize agent metrics
            self.agent_metrics[agent_id] = AgentMetrics(
                agent_id=agent_id,
                controller_type=ctrl_type,
                group_id=group_id
            )

            # Subscribe to metrics topic
            sub = self.create_subscription(
                MetricsData,
                f"/{agent_id}/metrics",
                lambda msg, aid=agent_id: self.metrics_callback(msg, aid),
                qos
            )
            self.metrics_subs.append(sub)

        # Subscribe to wind data
        self.wind_sub = self.create_subscription(
            Vector3,
            "/wind/velocity",
            self.wind_callback,
            qos
        )

        # Subscribe to cmd_accel for control effort tracking
        for i in range(self.num_agents):
            agent_id = f"agent_{i}"
            cmd_sub = self.create_subscription(
                Vector3,
                f"/{agent_id}/cmd_accel",
                lambda msg, aid=agent_id: self.cmd_callback(msg, aid),
                qos
            )
            self.cmd_subs.append(cmd_sub)

        # Status timer (1 Hz)
        self.status_timer = self.create_timer(1.0, self.status_callback)

        # Note: start_time_ns will be set on first metrics message (Phase 5 fix)
        self.get_logger().info(
            f"Phase {self.phase_id} Run {self.run_index} - "
            f"Logging {self.num_agents} agents for {self.duration_sec}s"
        )
        self.get_logger().info(f"Output directory: {self.run_dir}")
        self.get_logger().info("Waiting for first metrics message to start timer...")

    def _get_elapsed_sim_time(self) -> float:
        """Get elapsed simulation time in seconds."""
        if self.start_time_ns is None:
            return 0.0
        current_ns = self.get_clock().now().nanoseconds
        return (current_ns - self.start_time_ns) / 1e9

    def metrics_callback(self, msg: MetricsData, agent_id: str):
        """Handle incoming metrics data."""
        if agent_id not in self.agent_metrics:
            return

        # Phase 5 fix: Start timer on first message from ANY agent
        if not self.first_message_received:
            self.start_time_ns = self.get_clock().now().nanoseconds
            self.first_message_received = True
            self.get_logger().info(
                f"First metrics received from {agent_id}, starting timer"
            )

        am = self.agent_metrics[agent_id]
        elapsed = self._get_elapsed_sim_time()

        # Store time series data
        am.timestamps.append(elapsed)
        am.errors_x.append(msg.error_x)
        am.errors_y.append(msg.error_y)
        am.errors_mag.append(msg.error_magnitude)
        am.rmse_x.append(msg.rmse_x)
        am.rmse_y.append(msg.rmse_y)
        am.rmse_total.append(msg.rmse_total)
        am.iae_x.append(msg.iae_x)
        am.iae_y.append(msg.iae_y)
        am.itae_x.append(msg.itae_x)
        am.itae_y.append(msg.itae_y)
        am.settling_times.append(msg.settling_time)
        am.max_overshoots_x.append(msg.max_overshoot_x)
        am.max_overshoots_y.append(msg.max_overshoot_y)

        # Store mission-wide metrics (these accumulate across waypoints)
        am.mission_rmse.append(msg.mission_rmse)
        am.mission_itae_x.append(msg.mission_itae_x)
        am.mission_itae_y.append(msg.mission_itae_y)

        am.samples += 1

    def wind_callback(self, msg: Vector3):
        """Handle wind velocity data."""
        elapsed = self._get_elapsed_sim_time()
        magnitude = (msg.x**2 + msg.y**2 + msg.z**2)**0.5
        self.wind_data.append({
            "timestamp": elapsed,
            "wind_x": msg.x,
            "wind_y": msg.y,
            "wind_z": msg.z,
            "wind_magnitude": magnitude,
        })

    def cmd_callback(self, msg: Vector3, agent_id: str):
        """Handle control command data for control effort tracking."""
        elapsed = self._get_elapsed_sim_time()
        import math
        mag = math.sqrt(msg.x**2 + msg.y**2)  # XY control effort
        self.cmd_data[agent_id].append((elapsed, mag))

    def status_callback(self):
        """Print status and check for completion."""
        if self.start_time_ns is None:
            return

        elapsed = self._get_elapsed_sim_time()
        remaining = max(0, self.duration_sec - elapsed)

        # Count samples per group
        group_samples = defaultdict(int)
        for am in self.agent_metrics.values():
            group_samples[am.controller_type] += am.samples

        status = " | ".join(
            f"{ct}: {samples}" for ct, samples in sorted(group_samples.items())
        )
        self.get_logger().info(
            f"[{elapsed:.0f}s/{self.duration_sec}s] Samples: {status}"
        )

        # Check for completion
        if elapsed >= self.duration_sec:
            self.get_logger().info("Duration complete - exporting data...")
            self.export_all_data()
            self.get_logger().info("Data export complete. Shutting down.")
            rclpy.shutdown()

    def compute_agent_summary(self, am: AgentMetrics):
        """Compute final summary metrics for an agent."""
        if am.samples == 0:
            return

        import math

        # OFFLINE computation from stored error time-series
        # 1. Compute error magnitudes
        error_mags = [
            math.sqrt(ex**2 + ey**2)
            for ex, ey in zip(am.errors_x, am.errors_y)
        ]

        # 2. Peak Error (max error magnitude over mission)
        am.peak_error = max(error_mags) if error_mags else 0.0

        # 3. Last Violation Time (settling-like: last time error exceeded threshold)
        THRESHOLD = 0.1  # 10cm threshold for "settled"
        violations = [
            (t, e) for t, e in zip(am.timestamps, error_mags) if e > THRESHOLD
        ]
        am.last_violation_time = violations[-1][0] if violations else 0.0

        # 4. Mean Absolute Error (MAE)
        am.mean_absolute_error = (
            sum(error_mags) / len(error_mags) if error_mags else 0.0
        )

        # 5. RMSE (from time-series, not publisher's per-waypoint value)
        if am.mission_rmse and am.mission_rmse[-1] > 0:
            am.final_rmse = am.mission_rmse[-1]
        elif error_mags:
            sum_sq = sum(ex**2 + ey**2 for ex, ey in zip(am.errors_x, am.errors_y))
            am.final_rmse = math.sqrt(sum_sq / am.samples)
        else:
            am.final_rmse = am.rmse_total[-1] if am.rmse_total else 0.0

        # Use mission-wide ITAE if available, otherwise per-waypoint
        if am.mission_itae_x and am.mission_itae_y:
            am.final_itae = am.mission_itae_x[-1] + am.mission_itae_y[-1]
        else:
            am.final_itae = (
                (am.itae_x[-1] if am.itae_x else 0.0) +
                (am.itae_y[-1] if am.itae_y else 0.0)
            )

        # Use OFFLINE computed metrics instead of publisher's (which are 0)
        am.final_settling_time = am.last_violation_time  # last_violation_time as settling proxy
        am.final_max_overshoot = am.peak_error  # peak_error as overshoot proxy (NOT true overshoot)

        # 6. Control Effort IAE (integral of |u| dt)
        am.control_effort_iae = self._compute_control_effort_iae(am.agent_id)

        # 7. STEADY-STATE metrics (exclude startup transient t < steady_state_start_sec)
        # This provides fair controller comparison by excluding formation acquisition phase
        ss_indices = [
            i for i, t in enumerate(am.timestamps)
            if t >= self.steady_state_start_sec
        ]

        if ss_indices:
            ss_errors_x = [am.errors_x[i] for i in ss_indices]
            ss_errors_y = [am.errors_y[i] for i in ss_indices]
            ss_error_mags = [
                math.sqrt(ex**2 + ey**2)
                for ex, ey in zip(ss_errors_x, ss_errors_y)
            ]

            # Steady-state RMSE
            ss_sum_sq = sum(ex**2 + ey**2 for ex, ey in zip(ss_errors_x, ss_errors_y))
            am.ss_rmse = math.sqrt(ss_sum_sq / len(ss_indices))

            # Steady-state MAE
            am.ss_mae = sum(ss_error_mags) / len(ss_error_mags)

            # Steady-state Peak Error
            am.ss_peak_error = max(ss_error_mags)

            # Large errors (>2m) in steady-state - robustness metric
            LARGE_ERROR_THRESHOLD = 2.0
            am.ss_large_errors = sum(1 for e in ss_error_mags if e > LARGE_ERROR_THRESHOLD)
        else:
            # Not enough data for steady-state analysis
            am.ss_rmse = am.final_rmse
            am.ss_mae = am.mean_absolute_error
            am.ss_peak_error = am.peak_error
            am.ss_large_errors = 0

    def _compute_control_effort_iae(self, agent_id: str) -> float:
        """Compute IAE of control effort (integral of |u| dt)."""
        data = self.cmd_data.get(agent_id, [])
        if len(data) < 2:
            return 0.0

        effort = 0.0
        for i in range(1, len(data)):
            dt = data[i][0] - data[i - 1][0]
            if dt > 0:
                effort += data[i][1] * dt

        return effort

    def compute_group_summaries(self) -> Dict[str, GroupSummary]:
        """Compute summary metrics per controller group."""
        groups: Dict[str, List[AgentMetrics]] = defaultdict(list)

        for am in self.agent_metrics.values():
            if am.samples > 0:
                self.compute_agent_summary(am)
                groups[am.controller_type].append(am)

        summaries = {}
        for ctrl_type, agents in groups.items():
            if not agents:
                continue

            n = len(agents)
            rmses = [a.final_rmse for a in agents]
            itaes = [a.final_itae for a in agents]
            settlings = [a.final_settling_time for a in agents]
            overshoots = [a.final_max_overshoot for a in agents]

            # OFFLINE computed metrics
            peak_errors = [a.peak_error for a in agents]
            last_violations = [a.last_violation_time for a in agents]
            maes = [a.mean_absolute_error for a in agents]
            efforts = [a.control_effort_iae for a in agents]

            # STEADY-STATE metrics
            ss_rmses = [a.ss_rmse for a in agents]
            ss_maes = [a.ss_mae for a in agents]
            ss_peaks = [a.ss_peak_error for a in agents]
            ss_large = [a.ss_large_errors for a in agents]

            import statistics
            summaries[ctrl_type] = GroupSummary(
                group_id=agents[0].group_id,
                controller_type=ctrl_type,
                agent_count=n,
                mean_rmse=statistics.mean(rmses) if rmses else 0.0,
                std_rmse=statistics.stdev(rmses) if len(rmses) > 1 else 0.0,
                mean_itae=statistics.mean(itaes) if itaes else 0.0,
                std_itae=statistics.stdev(itaes) if len(itaes) > 1 else 0.0,
                mean_settling_time=statistics.mean(settlings) if settlings else 0.0,
                mean_max_overshoot=statistics.mean(overshoots) if overshoots else 0.0,
                mean_control_effort_iae=statistics.mean(efforts) if efforts else 0.0,
                agents_settled=sum(1 for s in settlings if s < self.duration_sec * 0.8),
                # OFFLINE metrics
                mean_peak_error=statistics.mean(peak_errors) if peak_errors else 0.0,
                mean_last_violation=statistics.mean(last_violations) if last_violations else 0.0,
                mean_mae=statistics.mean(maes) if maes else 0.0,
                # STEADY-STATE metrics (t > steady_state_start_sec)
                mean_ss_rmse=statistics.mean(ss_rmses) if ss_rmses else 0.0,
                std_ss_rmse=statistics.stdev(ss_rmses) if len(ss_rmses) > 1 else 0.0,
                mean_ss_mae=statistics.mean(ss_maes) if ss_maes else 0.0,
                mean_ss_peak=statistics.mean(ss_peaks) if ss_peaks else 0.0,
                total_ss_large_errors=sum(ss_large),
            )
            summaries[ctrl_type].settled_ratio = (
                summaries[ctrl_type].agents_settled / n if n > 0 else 0.0
            )

        return summaries

    def export_agent_csv(self, am: AgentMetrics):
        """Export time series data for one agent."""
        filename = os.path.join(
            self.run_dir,
            f"{am.agent_id}_{am.controller_type.lower()}.csv"
        )

        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                "timestamp", "error_x", "error_y", "error_magnitude",
                "rmse_x", "rmse_y", "rmse_total",
                "iae_x", "iae_y", "itae_x", "itae_y",
                "settling_time", "max_overshoot_x", "max_overshoot_y"
            ])

            for i in range(am.samples):
                writer.writerow([
                    f"{am.timestamps[i]:.3f}",
                    f"{am.errors_x[i]:.6f}",
                    f"{am.errors_y[i]:.6f}",
                    f"{am.errors_mag[i]:.6f}",
                    f"{am.rmse_x[i]:.6f}",
                    f"{am.rmse_y[i]:.6f}",
                    f"{am.rmse_total[i]:.6f}",
                    f"{am.iae_x[i]:.6f}",
                    f"{am.iae_y[i]:.6f}",
                    f"{am.itae_x[i]:.6f}",
                    f"{am.itae_y[i]:.6f}",
                    f"{am.settling_times[i]:.3f}",
                    f"{am.max_overshoots_x[i]:.6f}",
                    f"{am.max_overshoots_y[i]:.6f}",
                ])

        self.get_logger().info(f"Exported {am.samples} samples to {filename}")

    def export_wind_csv(self):
        """Export wind data to CSV."""
        filename = os.path.join(self.run_dir, "wind_data.csv")

        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["timestamp", "wind_x", "wind_y", "wind_z", "wind_magnitude"])

            for wd in self.wind_data:
                writer.writerow([
                    f"{wd['timestamp']:.3f}",
                    f"{wd['wind_x']:.4f}",
                    f"{wd['wind_y']:.4f}",
                    f"{wd['wind_z']:.4f}",
                    f"{wd['wind_magnitude']:.4f}",
                ])

        self.get_logger().info(f"Exported {len(self.wind_data)} wind samples")

    def export_group_summary(self, summaries: Dict[str, GroupSummary]):
        """Export group summary to CSV."""
        filename = os.path.join(self.run_dir, "group_summary.csv")

        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                "controller_type", "group_id", "agent_count",
                "mean_rmse", "std_rmse",
                "mean_itae", "std_itae",
                "mean_settling_time", "mean_max_overshoot",
                "control_effort_iae",  # Integral of |u| dt
                "agents_settled", "settled_ratio",
                # OFFLINE computed metrics
                "mean_peak_error", "mean_last_violation", "mean_mae",
                # STEADY-STATE metrics (t > steady_state_start_sec)
                "ss_rmse", "std_ss_rmse", "ss_mae", "ss_peak", "ss_large_errors"
            ])

            for ctrl_type in ["PD", "PID", "IT2", "GT2"]:
                if ctrl_type in summaries:
                    s = summaries[ctrl_type]
                    writer.writerow([
                        s.controller_type, s.group_id, s.agent_count,
                        f"{s.mean_rmse:.6f}", f"{s.std_rmse:.6f}",
                        f"{s.mean_itae:.6f}", f"{s.std_itae:.6f}",
                        f"{s.mean_settling_time:.3f}", f"{s.mean_max_overshoot:.6f}",
                        f"{s.mean_control_effort_iae:.6f}",
                        s.agents_settled, f"{s.settled_ratio:.2f}",
                        # OFFLINE computed metrics
                        f"{s.mean_peak_error:.6f}", f"{s.mean_last_violation:.3f}",
                        f"{s.mean_mae:.6f}",
                        # STEADY-STATE metrics
                        f"{s.mean_ss_rmse:.6f}", f"{s.std_ss_rmse:.6f}",
                        f"{s.mean_ss_mae:.6f}", f"{s.mean_ss_peak:.6f}",
                        s.total_ss_large_errors
                    ])

        self.get_logger().info(f"Exported group summary to {filename}")

    def _get_git_commit(self) -> str:
        """Get current git commit hash (short form)."""
        import subprocess
        try:
            result = subprocess.run(
                ["git", "rev-parse", "--short", "HEAD"],
                capture_output=True,
                text=True,
                cwd=os.path.dirname(os.path.abspath(__file__)),
            )
            return result.stdout.strip() if result.returncode == 0 else "unknown"
        except Exception:
            return "unknown"

    def export_metadata(self, summaries: Dict[str, GroupSummary]):
        """Export phase metadata to JSON."""
        filename = os.path.join(self.run_dir, "phase_metadata.json")

        # Calculate actual sim time elapsed
        actual_sim_duration = self._get_elapsed_sim_time()

        metadata = {
            "phase_id": self.phase_id,
            "run_index": self.run_index,
            "duration_sec": self.duration_sec,
            "actual_sim_duration_sec": actual_sim_duration,
            "start_time": datetime.now().isoformat(),  # Wall clock for reference
            "end_time": datetime.now().isoformat(),
            "num_agents": self.num_agents,
            "total_samples": sum(am.samples for am in self.agent_metrics.values()),
            "wind_samples": len(self.wind_data),
            "timing_source": "simulation_time",
            "git_commit": self._get_git_commit(),
            "config_snapshot": {
                "wind_scalar": 1.0,  # From launch file
                "k_fuzzy": 0.5,      # Fuzzy contribution factor
                "k_pid": 1.0,        # PID contribution factor
                "settling_threshold_m": 0.1,  # 10cm for last_violation_time
                "steady_state_start_sec": self.steady_state_start_sec,  # Transient exclusion
                "large_error_threshold_m": 2.0,  # For ss_large_errors count
            },
            "metric_definitions": {
                "peak_error": "max(|e|) over mission - maximum error magnitude",
                "last_violation_time": "last t where |e| > 0.1m - settling-like metric",
                "mean_absolute_error": "mean(|e|) over mission - MAE",
                "control_effort_iae": "integral(|u|) dt - control effort IAE",
                "rmse": "sqrt(mean(e^2)) - root mean squared error",
                "itae": "integral(t * |e|) dt - integral of time-weighted absolute error",
                # STEADY-STATE metrics (exclude startup transient)
                "ss_rmse": f"RMSE for t > {self.steady_state_start_sec}s - excludes formation acquisition",
                "ss_mae": f"MAE for t > {self.steady_state_start_sec}s - steady-state mean error",
                "ss_peak": f"Peak error for t > {self.steady_state_start_sec}s - max steady-state error",
                "ss_large_errors": "Count of errors > 2m in steady-state - robustness metric",
            },
            "group_summaries": {
                k: asdict(v) for k, v in summaries.items()
            },
        }

        with open(filename, 'w') as f:
            json.dump(metadata, f, indent=2)

        self.get_logger().info(f"Exported metadata to {filename}")

    def export_all_data(self):
        """Export all collected data."""
        self.end_time = time.time()

        # Export per-agent CSVs
        for am in self.agent_metrics.values():
            if am.samples > 0:
                self.export_agent_csv(am)

        # Export wind data
        self.export_wind_csv()

        # Compute and export group summaries
        summaries = self.compute_group_summaries()
        self.export_group_summary(summaries)

        # Export metadata
        self.export_metadata(summaries)

        # Print summary to console
        print("\n" + "=" * 60)
        print(f"Phase {self.phase_id} Run {self.run_index} - RESULTS SUMMARY")
        print("=" * 60)

        for ctrl_type in ["PD", "PID", "IT2", "GT2"]:
            if ctrl_type in summaries:
                s = summaries[ctrl_type]
                print(f"\n{ctrl_type} (Group {s.group_id}):")
                print(f"  RMSE:     {s.mean_rmse:.4f} ± {s.std_rmse:.4f} m")
                print(f"  ITAE:     {s.mean_itae:.4f} ± {s.std_itae:.4f}")
                print(f"  Settling: {s.mean_settling_time:.2f} s")
                print(f"  Settled:  {s.agents_settled}/{s.agent_count}")

        print("\n" + "=" * 60)


def main(args=None):
    rclpy.init(args=args)

    try:
        node = PhaseMetricsLogger()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
