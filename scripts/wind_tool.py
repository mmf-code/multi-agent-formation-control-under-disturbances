#!/usr/bin/env python3
"""
Wind Tool - Control wind in Gazebo (ROS 2)

Can publish geometry_msgs/Vector3 on:
  - "/wind/force"    (constant disturbance force in Newtons)
  - "/wind/velocity" (ambient wind velocity in m/s)

Usage examples:
  # Force-only (legacy behaviour)
  python3 scripts/wind_tool.py --x 4.0 --y 1.2 --z 0.0 --duration 60

  # Velocity + force together (recommended for fuzzy + physics)
  python3 scripts/wind_tool.py --x 4.0 --y 1.2 --z 0.0 --duration 60 --mode both
"""

import argparse
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3


class WindTool(Node):
    def __init__(self, x: float, y: float, z: float, rate_hz: float, duration: float, mode: str):
        super().__init__('wind_tool')
        self.mode = mode
        self.force_pub = None
        self.velocity_pub = None

        if mode in ('force', 'both'):
            self.force_pub = self.create_publisher(Vector3, '/wind/force', 10)
        if mode in ('velocity', 'both'):
            self.velocity_pub = self.create_publisher(Vector3, '/wind/velocity', 10)

        self.msg = Vector3(x=x, y=y, z=z)
        self.rate = 1.0 / max(rate_hz, 0.1)
        self.end_time = self.get_clock().now() + rclpy.duration.Duration(seconds=float(duration))
        self.timer = self.create_timer(self.rate, self.tick)

        self.get_logger().info(
            f"Publishing wind (mode={mode}): x={x:.2f}, y={y:.2f}, z={z:.2f} @ {rate_hz:.1f} Hz for {duration:.1f}s"
        )

    def tick(self):
        if self.get_clock().now() >= self.end_time:
            self.get_logger().info('Done. Stopping wind publisher.')
            rclpy.shutdown()
            return
        if self.force_pub is not None:
            self.force_pub.publish(self.msg)
        if self.velocity_pub is not None:
            self.velocity_pub.publish(self.msg)


def main():
    parser = argparse.ArgumentParser(description='Publish wind force to Gazebo')
    parser.add_argument('--x', type=float, default=4.0)
    parser.add_argument('--y', type=float, default=1.2)
    parser.add_argument('--z', type=float, default=0.0)
    parser.add_argument('--rate', type=float, default=10.0, help='Publish rate in Hz')
    parser.add_argument('--duration', type=float, default=60.0, help='Duration in seconds')
    parser.add_argument(
        '--mode',
        type=str,
        choices=['force', 'velocity', 'both'],
        default='force',
        help='What to publish: "force", "velocity", or "both" on /wind/force and /wind/velocity.'
    )
    args = parser.parse_args()

    rclpy.init()
    node = WindTool(args.x, args.y, args.z, args.rate, args.duration, args.mode)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
