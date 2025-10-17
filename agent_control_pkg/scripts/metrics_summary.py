#!/usr/bin/env python3
"""
Metrics Summary Node

This node collects metrics from all agents and prints a final summary
when the simulation is terminated (Ctrl+C).

Subscribes to:
  /agent_X/metrics (std_msgs/Float64MultiArray)

Metrics array format (from metrics_publisher_node.cpp):
  [0] error_magnitude
  [1] rmse
  [2] iae_x
  [3] iae_y
  [4] settling_time
  [5] settled (1.0 = yes, 0.0 = no)

Author: Multi-Agent Formation Control Research Team
Date: 2025-10-17
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import signal
import sys
from collections import defaultdict


class MetricsSummaryNode(Node):
    def __init__(self):
        super().__init__('metrics_summary')

        # Storage for latest metrics from each agent
        self.agent_metrics = {}

        # Group definitions
        self.groups = {
            'Group 0 (PID+Fuzzy)': [0, 1, 2],
            'Group 1 (PD)': [3, 4, 5],
            'Group 2 (PID)': [6, 7, 8],
        }

        # Subscribe to all agent metrics
        self.subscribers = []
        for agent_id in range(9):
            topic = f'/agent_{agent_id}/metrics'
            sub = self.create_subscription(
                Float64MultiArray,
                topic,
                lambda msg, aid=agent_id: self.metrics_callback(msg, aid),
                10
            )
            self.subscribers.append(sub)
            self.agent_metrics[agent_id] = None

        # Register signal handler for Ctrl+C
        signal.signal(signal.SIGINT, self.signal_handler)

        self.get_logger().info('Metrics Summary Node initialized. Collecting data from 9 agents...')
        self.get_logger().info('Press Ctrl+C to see final summary.')

    def metrics_callback(self, msg, agent_id):
        """Store latest metrics for this agent"""
        if len(msg.data) >= 6:
            self.agent_metrics[agent_id] = {
                'error': msg.data[0],
                'rmse': msg.data[1],
                'iae_x': msg.data[2],
                'iae_y': msg.data[3],
                'settling_time': msg.data[4],
                'settled': msg.data[5] > 0.5,
            }

    def signal_handler(self, sig, frame):
        """Handle Ctrl+C and print final summary"""
        self.print_final_summary()
        rclpy.shutdown()
        sys.exit(0)

    def print_final_summary(self):
        """Print comprehensive performance summary"""
        print("\n" + "=" * 70)
        print(" " * 15 + "FORMATION COMPARISON - FINAL RESULTS")
        print("=" * 70)
        print()

        group_stats = {}

        for group_name, agent_ids in self.groups.items():
            print(f"{group_name}")
            print("-" * 70)

            settling_times = []
            rmse_values = []
            iae_total_values = []
            settled_count = 0

            for agent_id in agent_ids:
                metrics = self.agent_metrics.get(agent_id)

                if metrics is None:
                    print(f"  agent_{agent_id}: NO DATA")
                    continue

                # Calculate total IAE
                iae_total = metrics['iae_x'] + metrics['iae_y']

                # Collect for averages
                settling_times.append(metrics['settling_time'])
                rmse_values.append(metrics['rmse'])
                iae_total_values.append(iae_total)
                if metrics['settled']:
                    settled_count += 1

                # Print individual agent
                settled_str = "YES" if metrics['settled'] else "NO"
                print(f"  agent_{agent_id}: Ts={metrics['settling_time']:.1f}s, "
                      f"RMSE={metrics['rmse']:.3f}m, "
                      f"IAE={iae_total:.1f}, "
                      f"Settled={settled_str}")

            # Calculate group averages
            if settling_times:
                avg_ts = sum(settling_times) / len(settling_times)
                avg_rmse = sum(rmse_values) / len(rmse_values)
                avg_iae = sum(iae_total_values) / len(iae_total_values)

                print("-" * 70)
                print(f"  Average: Ts={avg_ts:.1f}s, RMSE={avg_rmse:.3f}m, IAE={avg_iae:.1f}")
                print(f"  Settled: {settled_count}/{len(agent_ids)} agents")
                print()

                # Store for final comparison
                group_stats[group_name] = {
                    'avg_ts': avg_ts,
                    'avg_rmse': avg_rmse,
                    'avg_iae': avg_iae,
                    'settled_ratio': settled_count / len(agent_ids)
                }

        # Final comparison
        print("=" * 70)
        print(" " * 20 + "PERFORMANCE COMPARISON")
        print("=" * 70)

        if group_stats:
            # Find best in each category
            fastest_group = min(group_stats.items(), key=lambda x: x[1]['avg_ts'])
            most_accurate_group = min(group_stats.items(), key=lambda x: x[1]['avg_rmse'])
            lowest_iae_group = min(group_stats.items(), key=lambda x: x[1]['avg_iae'])
            best_settled_group = max(group_stats.items(), key=lambda x: x[1]['settled_ratio'])

            print(f"FASTEST:        {fastest_group[0]} - Avg Ts = {fastest_group[1]['avg_ts']:.1f}s")
            print(f"MOST ACCURATE:  {most_accurate_group[0]} - Avg RMSE = {most_accurate_group[1]['avg_rmse']:.3f}m")
            print(f"LOWEST IAE:     {lowest_iae_group[0]} - Avg IAE = {lowest_iae_group[1]['avg_iae']:.1f}")
            print(f"BEST SETTLED:   {best_settled_group[0]} - {int(best_settled_group[1]['settled_ratio']*100)}% settled")

            # Overall winner (balanced score)
            print()
            print("THESIS CONCLUSION:")
            print("-" * 70)
            print("Group 0 (PID+Fuzzy): Expected best wind disturbance rejection")
            print("Group 1 (PD):        Expected fastest response (with overshoot)")
            print("Group 2 (PID):       Expected balanced performance")

        print("=" * 70)
        print()


def main(args=None):
    rclpy.init(args=args)
    node = MetricsSummaryNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
