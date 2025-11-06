#!/usr/bin/env python3
"""
Wind Tool - Control wind force vector in Gazebo (ROS 2)

Publishes geometry_msgs/Vector3 on "/wind/force" so you can:
  - Set constant wind (x,y,z in Newtons)
  - Sweep or adjust interactively via repeated calls

Usage examples:
  python3 scripts/wind_tool.py --x 4.0 --y 1.2 --z 0.0 --duration 60
  python3 scripts/wind_tool.py --x 0 --y 0 --z 0 --duration 5  # calm
"""

import argparse
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3


class WindTool(Node):
    def __init__(self, x: float, y: float, z: float, rate_hz: float, duration: float):
        super().__init__('wind_tool')
        self.pub = self.create_publisher(Vector3, '/wind/force', 10)
        self.msg = Vector3(x=x, y=y, z=z)
        self.rate = 1.0 / max(rate_hz, 0.1)
        self.end_time = self.get_clock().now() + rclpy.duration.Duration(seconds=float(duration))
        self.timer = self.create_timer(self.rate, self.tick)

        self.get_logger().info(f"Publishing wind: x={x:.2f} N, y={y:.2f} N, z={z:.2f} N @ {rate_hz:.1f} Hz for {duration:.1f}s")

    def tick(self):
        if self.get_clock().now() >= self.end_time:
            self.get_logger().info('Done. Stopping wind publisher.')
            rclpy.shutdown()
            return
        self.pub.publish(self.msg)


def main():
    parser = argparse.ArgumentParser(description='Publish wind force to Gazebo')
    parser.add_argument('--x', type=float, default=4.0)
    parser.add_argument('--y', type=float, default=1.2)
    parser.add_argument('--z', type=float, default=0.0)
    parser.add_argument('--rate', type=float, default=10.0, help='Publish rate in Hz')
    parser.add_argument('--duration', type=float, default=60.0, help='Duration in seconds')
    args = parser.parse_args()

    rclpy.init()
    node = WindTool(args.x, args.y, args.z, args.rate, args.duration)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()

