#!/usr/bin/env python3
"""
Simple test odometry publisher for ROS2 agent controller testing.
Publishes fake odometry data to test the controller node.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import math

class TestOdomPublisher(Node):
    def __init__(self):
        super().__init__('test_odom_publisher')

        # Parameters
        self.declare_parameter('agent_name', 'agent_0')
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('circular_motion', False)

        agent_name = self.get_parameter('agent_name').value
        rate_hz = self.get_parameter('rate_hz').value
        self.circular_motion = self.get_parameter('circular_motion').value

        # Publisher
        topic_name = f'/{agent_name}/odom'
        self.publisher_ = self.create_publisher(Odometry, topic_name, 10)

        # Timer
        timer_period = 1.0 / rate_hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # State
        self.x = 0.0
        self.y = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.t = 0.0

        self.get_logger().info(f'Publishing test odometry to {topic_name} at {rate_hz} Hz')

    def timer_callback(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'

        if self.circular_motion:
            # Circular motion for visual testing
            radius = 2.0
            omega = 0.5  # rad/s
            self.x = radius * math.cos(omega * self.t)
            self.y = radius * math.sin(omega * self.t)
            self.vx = -radius * omega * math.sin(omega * self.t)
            self.vy = radius * omega * math.cos(omega * self.t)
        else:
            # Stationary at origin (waiting for commands)
            self.x = 0.0
            self.y = 0.0
            self.vx = 0.0
            self.vy = 0.0

        # Position
        msg.pose.pose.position = Point(x=self.x, y=self.y, z=0.0)
        msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        # Velocity
        msg.twist.twist.linear = Vector3(x=self.vx, y=self.vy, z=0.0)
        msg.twist.twist.angular = Vector3(x=0.0, y=0.0, z=0.0)

        self.publisher_.publish(msg)
        self.t += 0.02  # Approximate dt

def main(args=None):
    rclpy.init(args=args)
    node = TestOdomPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
