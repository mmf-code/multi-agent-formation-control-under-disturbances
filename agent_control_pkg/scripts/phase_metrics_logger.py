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
from typing import Dict, List, Optional, Any
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
    rmse_total: List[float] = field(default_factory=list)
    iae_x: List[float] = field(default_factory=list)
    iae_y: List[float] = field(default_factory=list)
    itae_x: List[float] = field(default_factory=list)
    itae_y: List[float] = field(default_factory=list)
    settling_times: List[float] = field(default_factory=list)
    max_overshoots_x: List[float] = field(default_factory=list)
    max_overshoots_y: List[float] = field(default_factory=list)
    control_efforts: List[float] = field(default_factory=list)

    # Final summary metrics
    final_rmse: float = 0.0
    final_itae: float = 0.0
    final_settling_time: float = 0.0
    final_max_overshoot: float = 0.0
    mean_control_effort: float = 0.0


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
    mean_settling_time: float = 0.0
    mean_max_overshoot: float = 0.0
    mean_control_effort: float = 0.0
    agents_settled: int = 0
    settled_ratio: float = 0.0


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

        # Get parameters
        self.phase_id = self.get_parameter("phase_id").value
        self.run_index = self.get_parameter("run_index").value
        self.output_dir = self.get_parameter("output_dir").value
        self.duration_sec = self.get_parameter("duration_sec").value
        self.num_agents = self.get_parameter("num_agents").value

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
        self.start_time_ns: Optional[int] = None  # Simulation time in nanoseconds
        self.end_time: Optional[float] = None

        # Check if using simulation time (already declared by ROS2)
        # Use get_parameter_or to avoid duplicate declaration
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

        # Status timer (1 Hz)
        self.status_timer = self.create_timer(1.0, self.status_callback)

        # Record start time using simulation time
        self.start_time_ns = self.get_clock().now().nanoseconds

        self.get_logger().info(
            f"Phase {self.phase_id} Run {self.run_index} - "
            f"Logging {self.num_agents} agents for {self.duration_sec}s"
        )
        self.get_logger().info(f"Output directory: {self.run_dir}")

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

        # Use final values from time series
        am.final_rmse = am.rmse_total[-1] if am.rmse_total else 0.0
        am.final_itae = (
            (am.itae_x[-1] if am.itae_x else 0.0) +
            (am.itae_y[-1] if am.itae_y else 0.0)
        )
        am.final_settling_time = am.settling_times[-1] if am.settling_times else 0.0
        am.final_max_overshoot = max(
            max(am.max_overshoots_x) if am.max_overshoots_x else 0.0,
            max(am.max_overshoots_y) if am.max_overshoots_y else 0.0,
        )

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
                agents_settled=sum(1 for s in settlings if s < self.duration_sec * 0.8),
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
                "agents_settled", "settled_ratio"
            ])

            for ctrl_type in ["PD", "PID", "IT2", "GT2"]:
                if ctrl_type in summaries:
                    s = summaries[ctrl_type]
                    writer.writerow([
                        s.controller_type, s.group_id, s.agent_count,
                        f"{s.mean_rmse:.6f}", f"{s.std_rmse:.6f}",
                        f"{s.mean_itae:.6f}", f"{s.std_itae:.6f}",
                        f"{s.mean_settling_time:.3f}", f"{s.mean_max_overshoot:.6f}",
                        s.agents_settled, f"{s.settled_ratio:.2f}"
                    ])

        self.get_logger().info(f"Exported group summary to {filename}")

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
