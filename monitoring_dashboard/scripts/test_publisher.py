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
from my_custom_interfaces_pkg.msg import ControllerParams
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
        self.params_pubs = []

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

            # Controller parameters
            params_pub = self.create_publisher(ControllerParams, f'/{agent_id}/controller_params', 10)
            self.params_pubs.append(params_pub)

        # Wind publishers
        self.wind_vel_pub = self.create_publisher(Vector3, '/wind/velocity', 10)
        self.wind_force_pub = self.create_publisher(Vector3, '/wind/force', 10)

        # Timer for publishing
        self.timer = self.create_timer(0.1, self.publish_test_data)  # 10 Hz

        # Timer for controller params (less frequent - every 2 seconds)
        self.params_timer = self.create_timer(2.0, self.publish_controller_params)

        self.get_logger().info('Test publisher started - simulating 3 agents')
        self.get_logger().info('Publishing to:')
        for i in range(self.num_agents):
            self.get_logger().info(f'  - /agent_{i}/odom')
            self.get_logger().info(f'  - /agent_{i}/target_pose')
            self.get_logger().info(f'  - /agent_{i}/diagnostics')
            self.get_logger().info(f'  - /agent_{i}/controller_params')
        self.get_logger().info('  - /wind/velocity')
        self.get_logger().info('  - /wind/force')

        # Publish initial controller params immediately
        self.publish_controller_params()

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

    def publish_controller_params(self):
        """Publish controller parameters for all agents"""
        # Different controller configs for variety
        configs = [
            # Agent 0: Pure PID
            {
                'type': 'pid',
                'kp': 5.0, 'ki': 2.5, 'kd': 5.0,
                'fuzzy_enable': False, 'fuzzy_wind': 0.0,
                'mix_pid': 1.0, 'mix_fuzzy': 0.0,
                'ff_drag': False, 'ff_wind': False,
                'k_drag': 0.8, 'k_wind': 1.0
            },
            # Agent 1: Hybrid (PID + Fuzzy)
            {
                'type': 'hybrid',
                'kp': 3.5, 'ki': 1.8, 'kd': 4.0,
                'fuzzy_enable': True, 'fuzzy_wind': 0.5,
                'mix_pid': 0.7, 'mix_fuzzy': 0.3,
                'ff_drag': True, 'ff_wind': False,
                'k_drag': 0.8, 'k_wind': 1.0
            },
            # Agent 2: Full Hybrid with FF
            {
                'type': 'hybrid',
                'kp': 4.0, 'ki': 2.0, 'kd': 4.5,
                'fuzzy_enable': True, 'fuzzy_wind': 0.8,
                'mix_pid': 0.6, 'mix_fuzzy': 0.4,
                'ff_drag': True, 'ff_wind': True,
                'k_drag': 0.8, 'k_wind': 1.0
            }
        ]

        for i in range(self.num_agents):
            config = configs[i]

            params = ControllerParams()
            params.controller_type = config['type']

            # PID params
            params.pid_kp = config['kp']
            params.pid_ki = config['ki']
            params.pid_kd = config['kd']

            # Fuzzy params
            params.fuzzy_enable = config['fuzzy_enable']
            params.fuzzy_wind_scalar = config['fuzzy_wind']

            # Hybrid mixing
            params.mix_k_pid = config['mix_pid']
            params.mix_k_fuzzy = config['mix_fuzzy']

            # Feed-forward
            params.feedforward_enable_drag = config['ff_drag']
            params.feedforward_enable_wind = config['ff_wind']
            params.feedforward_k_drag = config['k_drag']
            params.feedforward_k_wind = config['k_wind']

            # Output limits
            params.output_limit_x_min = -10.0
            params.output_limit_x_max = 10.0
            params.output_limit_y_min = -8.0
            params.output_limit_y_max = 12.0

            # Timing
            params.control_frequency_hz = 200.0
            params.dt = 0.005

            self.params_pubs[i].publish(params)


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
