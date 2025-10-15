#!/usr/bin/env python3
"""
Gazebo Simulation Data Logger for Formation Control

This ROS2 node subscribes to key topics from the Gazebo simulation and logs
all data to CSV for post-experiment analysis. It captures:
  - Drone state (position, velocity) from odometry
  - Target positions from formation coordinator
  - Commanded accelerations from agent controller

The CSV file can be used for:
  - Performance analysis (settling time, overshoot, tracking error)
  - Controller tuning validation
  - Comparison with standalone C++ simulations
  - Plotting trajectories and control signals

Usage:
    # Terminal 1: Launch Gazebo simulation
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    ros2 launch agent_control_pkg gazebo_single_agent.launch.py

    # Terminal 2: Start logging
    python3 scripts/log_simulation_csv.py \
        --namespace agent_0 \
        --output outputs/logs/run_001.csv

    # Stop logging: Ctrl+C

CSV Format:
    stamp_sec, stamp_nanosec, pos_x, pos_y, pos_z, vel_x, vel_y, vel_z,
    cmd_ax, cmd_ay, cmd_az, target_x, target_y, target_z

Author: Multi-Agent Formation Control Team
Date: 2025
"""

import argparse
import csv
import os
from pathlib import Path
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class SimulationLogger(Node):
    """
    ROS2 node for logging simulation data to CSV.

    This node subscribes to three key topics and synchronizes their data:
      1. /agent_X/odom - Full state from Gazebo (triggers CSV write)
      2. /agent_X/cmd_accel - Controller output
      3. /agent_X/target_pose - Formation coordinator reference

    Data is written row-by-row whenever new odometry arrives, using the
    most recent command and target values. This ensures all three data
    streams are captured even if they arrive at different rates.

    Attributes:
        namespace (str): ROS2 namespace for topic subscription (e.g., "agent_0")
        output_path (Path): Absolute path to output CSV file
        csv_file (file): Open file handle for writing
        writer (csv.writer): CSV writer object
        last_cmd (Vector3): Most recent acceleration command (latched)
        last_target (PoseStamped): Most recent target pose (latched)
    """

    def __init__(self, namespace: str, output_path: Path):
        super().__init__("simulation_logger")

        self.namespace = namespace.strip("/")
        self.output_path = output_path

        # Use RELIABLE QoS to ensure no data loss
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Subscribe to controller output (cmd_accel)
        self.cmd_sub = self.create_subscription(
            Vector3,
            f"/{self.namespace}/cmd_accel",
            self._cmd_callback,
            qos,
        )

        # Subscribe to formation targets
        self.target_sub = self.create_subscription(
            PoseStamped,
            f"/{self.namespace}/target_pose",
            self._target_callback,
            qos,
        )

        # Subscribe to odometry (state feedback from Gazebo)
        # This is the primary trigger for CSV writes
        self.odom_sub = self.create_subscription(
            Odometry,
            f"/{self.namespace}/odom",
            self._odom_callback,
            qos,
        )

        # Latch last received values from cmd and target topics
        self.last_cmd: Optional[Vector3] = None
        self.last_target: Optional[PoseStamped] = None

        # Prepare CSV file with header
        os.makedirs(self.output_path.parent, exist_ok=True)
        self.csv_file = open(self.output_path, "w", newline="")
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(
            [
                "stamp_sec",       # ROS timestamp (seconds)
                "stamp_nanosec",   # ROS timestamp (nanoseconds)
                "pos_x",           # Position X (m)
                "pos_y",           # Position Y (m)
                "pos_z",           # Position Z (m)
                "vel_x",           # Velocity X (m/s)
                "vel_y",           # Velocity Y (m/s)
                "vel_z",           # Velocity Z (m/s)
                "cmd_ax",          # Commanded acceleration X (m/s²)
                "cmd_ay",          # Commanded acceleration Y (m/s²)
                "cmd_az",          # Commanded acceleration Z (m/s²)
                "target_x",        # Target position X (m)
                "target_y",        # Target position Y (m)
                "target_z",        # Target position Z (m)
            ]
        )
        self.get_logger().info(f"Logging to {self.output_path}")

    def _cmd_callback(self, msg: Vector3) -> None:
        """Store latest acceleration command (latched)"""
        self.last_cmd = msg

    def _target_callback(self, msg: PoseStamped) -> None:
        """Store latest target pose (latched)"""
        self.last_target = msg

    def _odom_callback(self, msg: Odometry) -> None:
        """
        Main callback: triggered by odometry from Gazebo.

        Writes one CSV row containing:
          - Current timestamp from odometry header
          - Position and velocity from odometry
          - Most recent cmd_accel (controller output)
          - Most recent target_pose (formation reference)

        This callback typically fires at ~1kHz (Gazebo physics rate).
        """
        if self.writer is None:
            return

        # Use last received cmd/target, or zeros if none received yet
        cmd = self.last_cmd or Vector3()
        target = self.last_target.pose if self.last_target else PoseStamped().pose

        # Write synchronized row to CSV
        self.writer.writerow(
            [
                msg.header.stamp.sec,
                msg.header.stamp.nanosec,
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z,
                cmd.x,
                cmd.y,
                cmd.z,
                target.position.x,
                target.position.y,
                target.position.z,
            ]
        )

    def destroy_node(self):
        """Clean shutdown: flush and close CSV file"""
        try:
            if hasattr(self, "csv_file") and not self.csv_file.closed:
                self.csv_file.flush()
                self.csv_file.close()
                self.get_logger().info("CSV file closed.")
        finally:
            super().destroy_node()


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments for logger configuration"""
    parser = argparse.ArgumentParser(
        description="Record simulation topics to CSV for post-analysis."
    )
    parser.add_argument(
        "--namespace",
        "-n",
        default="agent_0",
        help="ROS2 namespace of the agent (default: agent_0)",
    )
    parser.add_argument(
        "--output",
        "-o",
        default="outputs/logs/simulation_log.csv",
        help="Output CSV file path (default: outputs/logs/simulation_log.csv)",
    )
    return parser.parse_args()


def main() -> None:
    """
    Main entry point for the simulation logger.

    Initializes ROS2, creates the logger node, and spins until Ctrl+C.
    Ensures clean shutdown and file closure on exit.
    """
    args = parse_args()
    rclpy.init()
    output_path = Path(args.output).expanduser().resolve()

    node = SimulationLogger(args.namespace, output_path)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Logging interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
