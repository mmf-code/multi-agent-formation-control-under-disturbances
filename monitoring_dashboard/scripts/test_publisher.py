#!/usr/bin/env python3
"""
Test ROS2 Publisher for Dashboard Testing
Publishes dummy data to test the monitoring dashboard without running full simulation
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Vector3
from std_msgs.msg import Float64MultiArray
import math
import time


class TestPublisher(Node):
    def __init__(self):
        super().__init__('dashboard_test_publisher')

        # Number of test agents
        self.num_agents = 3
        self.t = 0.0

        # Create publishers for each agent
        self.odom_pubs = []
        self.target_pubs = []
        self.diag_pubs = []

        for i in range(self.num_agents):
            agent_id = f'agent_{i}'

            # Odometry
            odom_pub = self.create_publisher(Odometry, f'/{agent_id}/odom', 10)
            self.odom_pubs.append(odom_pub)

            # Target pose
            target_pub = self.create_publisher(PoseStamped, f'/{agent_id}/target_pose', 10)
            self.target_pubs.append(target_pub)

            # Diagnostics
            diag_pub = self.create_publisher(Float64MultiArray, f'/{agent_id}/diagnostics', 10)
            self.diag_pubs.append(diag_pub)

        # Wind publishers
        self.wind_vel_pub = self.create_publisher(Vector3, '/wind/velocity', 10)
        self.wind_force_pub = self.create_publisher(Vector3, '/wind/force', 10)

        # Timer for publishing
        self.timer = self.create_timer(0.1, self.publish_test_data)  # 10 Hz

        self.get_logger().info('Test publisher started - simulating 3 agents')
        self.get_logger().info('Publishing to:')
        for i in range(self.num_agents):
            self.get_logger().info(f'  - /agent_{i}/odom')
            self.get_logger().info(f'  - /agent_{i}/target_pose')
            self.get_logger().info(f'  - /agent_{i}/diagnostics')
        self.get_logger().info('  - /wind/velocity')
        self.get_logger().info('  - /wind/force')

    def publish_test_data(self):
        """Publish test data for all agents"""
        self.t += 0.1

        # Simulate agents in formation (triangle)
        for i in range(self.num_agents):
            angle = (i * 2 * math.pi / self.num_agents) + (self.t * 0.1)
            radius = 2.0 + 0.5 * math.sin(self.t * 0.5)

            # Calculate position (circular motion)
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            z = 1.0 + 0.2 * math.sin(self.t)

            # Odometry
            odom = Odometry()
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.header.frame_id = 'world'
            odom.child_frame_id = f'agent_{i}'

            odom.pose.pose.position.x = x
            odom.pose.pose.position.y = y
            odom.pose.pose.position.z = z

            odom.twist.twist.linear.x = -radius * math.sin(angle) * 0.1
            odom.twist.twist.linear.y = radius * math.cos(angle) * 0.1
            odom.twist.twist.linear.z = 0.2 * math.cos(self.t)

            self.odom_pubs[i].publish(odom)

            # Target pose (slightly ahead)
            target = PoseStamped()
            target.header.stamp = self.get_clock().now().to_msg()
            target.header.frame_id = 'world'

            target_angle = angle + 0.2
            target.pose.position.x = radius * math.cos(target_angle)
            target.pose.position.y = radius * math.sin(target_angle)
            target.pose.position.z = z + 0.1

            self.target_pubs[i].publish(target)

            # Diagnostics (dummy controller outputs)
            diag = Float64MultiArray()
            diag.data = [
                1.5 + 0.3 * math.sin(self.t + i),  # x_total
                1.0 + 0.2 * math.sin(self.t + i),  # x_pid
                0.5 + 0.1 * math.sin(self.t + i),  # x_fuzzy
                1.2 + 0.3 * math.cos(self.t + i),  # y_total
                0.8 + 0.2 * math.cos(self.t + i),  # y_pid
                0.4 + 0.1 * math.cos(self.t + i),  # y_fuzzy
            ]
            self.diag_pubs[i].publish(diag)

        # Wind data (varying)
        wind_vel = Vector3()
        wind_vel.x = 2.0 * math.sin(self.t * 0.3)
        wind_vel.y = 1.5 * math.cos(self.t * 0.4)
        wind_vel.z = 0.5 * math.sin(self.t * 0.2)
        self.wind_vel_pub.publish(wind_vel)

        wind_force = Vector3()
        wind_force.x = wind_vel.x * 0.5
        wind_force.y = wind_vel.y * 0.5
        wind_force.z = wind_vel.z * 0.5
        self.wind_force_pub.publish(wind_force)


def main(args=None):
    rclpy.init(args=args)
    node = TestPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Test publisher stopped')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
