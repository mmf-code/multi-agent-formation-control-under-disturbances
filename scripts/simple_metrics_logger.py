#!/usr/bin/env python3
"""
Simple Metrics Logger - Records formation demo metrics to CSV
Works by subscribing directly to ROS2 topics (proper way)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import csv
import sys
import os
from datetime import datetime

# Import the actual message type
try:
    from my_custom_interfaces_pkg.msg import MetricsData
    HAS_METRICS_MSG = True
except ImportError:
    HAS_METRICS_MSG = False
    print("WARNING: MetricsData message not found, using generic approach")

class SimpleMetricsLogger(Node):
    def __init__(self, output_dir, duration=60):
        super().__init__('simple_metrics_logger')

        self.output_dir = output_dir
        self.duration = duration
        self.start_time = self.get_clock().now()

        # QoS profile matching the publishers
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # CSV files
        self.csv_files = {}
        self.csv_writers = {}

        agents = {
            'agent_0': 'pidfuzzy',
            'agent_3': 'pd',
            'agent_6': 'pid'
        }

        for agent_id, controller_type in agents.items():
            csv_path = os.path.join(output_dir, f'{agent_id}_{controller_type}.csv')
            f = open(csv_path, 'w', newline='')
            writer = csv.writer(f)
            writer.writerow([
                'timestamp', 'elapsed_time',
                'error_x', 'error_y', 'error_z', 'error_magnitude',
                'rmse_x', 'rmse_y', 'rmse_z', 'rmse_total',
                'iae_x', 'iae_y', 'settling_time', 'is_settled'
            ])
            f.flush()
            self.csv_files[agent_id] = f
            self.csv_writers[agent_id] = writer

            # Subscribe to metrics topic
            if HAS_METRICS_MSG:
                self.create_subscription(
                    MetricsData,
                    f'/{agent_id}/metrics',
                    lambda msg, aid=agent_id: self.metrics_callback(msg, aid),
                    qos
                )
            else:
                self.get_logger().warn(f'Cannot subscribe to {agent_id}/metrics - message type not available')

        self.get_logger().info(f'Logger initialized. Recording to: {output_dir}')
        self.get_logger().info(f'Duration: {duration}s')

        # Timer to check if we should stop
        self.timer = self.create_timer(1.0, self.check_duration)
        self.sample_counts = {aid: 0 for aid in agents.keys()}

    def metrics_callback(self, msg, agent_id):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        if elapsed > self.duration:
            return

        timestamp = datetime.now().timestamp()

        row = [
            timestamp, elapsed,
            msg.error_x, msg.error_y, msg.error_z, msg.error_magnitude,
            msg.rmse_x, msg.rmse_y, msg.rmse_z, msg.rmse_total,
            msg.iae_x, msg.iae_y, msg.settling_time,
            1 if msg.is_settled else 0
        ]

        self.csv_writers[agent_id].writerow(row)
        self.csv_files[agent_id].flush()
        self.sample_counts[agent_id] += 1

    def check_duration(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        if elapsed >= self.duration:
            total = sum(self.sample_counts.values())
            self.get_logger().info(f'Recording complete! Total samples: {total}')
            for agent_id, count in self.sample_counts.items():
                self.get_logger().info(f'  {agent_id}: {count} samples')
            self.cleanup()
            rclpy.shutdown()
        else:
            progress = int(elapsed / self.duration * 100)
            self.get_logger().info(f'Progress: {progress}% ({int(self.duration - elapsed)}s remaining)')

    def cleanup(self):
        for f in self.csv_files.values():
            f.close()

def main():
    import argparse
    parser = argparse.ArgumentParser(description='Record ROS2 metrics to CSV')
    parser.add_argument('--output-dir', '-o', required=True, help='Output directory for CSV files')
    parser.add_argument('--duration', '-d', type=int, default=60, help='Recording duration in seconds (default: 60)')
    parser.add_argument('--agents', '-a', nargs='+', default=['0', '3', '6'], help='Agent IDs to record (default: 0 3 6)')

    args = parser.parse_args()

    output_dir = args.output_dir
    duration = args.duration

    os.makedirs(output_dir, exist_ok=True)

    print(f"Recording metrics for agents: {', '.join(args.agents)}")
    print(f"Output directory: {output_dir}")
    print(f"Duration: {duration}s")

    rclpy.init()
    logger = SimpleMetricsLogger(output_dir, duration)

    try:
        rclpy.spin(logger)
    except KeyboardInterrupt:
        pass
    finally:
        logger.cleanup()
        logger.destroy_node()

if __name__ == '__main__':
    main()
