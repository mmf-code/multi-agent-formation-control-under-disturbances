#!/usr/bin/env python3
"""
TF to Odometry Bridge for Crazyswarm2 Integration

Crazyswarm2 publishes drone poses via TF (world -> cfXXX frames).
Our controllers expect nav_msgs/Odometry messages on /cfXXX/odom topics.
This bridge converts TF broadcasts to Odometry messages.

Supports 9 drones: cf231-cf239 (matching crazyflies_9drone.yaml)

Usage:
    ros2 run agent_control_pkg tf_to_odom_bridge.py
    ros2 run agent_control_pkg tf_to_odom_bridge.py --ros-args -p num_drones:=3

Author: Multi-Agent Formation Control Team
Date: 2025-01
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.duration import Duration
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformListener, Buffer
import numpy as np


class TfToOdomBridge(Node):
    """Converts TF transforms to Odometry messages for Crazyswarm2 drones."""

    def __init__(self):
        super().__init__('tf_to_odom_bridge')

        # Parameters
        self.declare_parameter('num_drones', 9)
        self.declare_parameter('base_cf_id', 231)  # cf231 is first drone
        self.declare_parameter('publish_rate', 100.0)  # Hz
        self.declare_parameter('world_frame', 'world')
        # NOTE: use_sim_time is auto-declared by Node, don't redeclare it!

        self.num_drones = self.get_parameter('num_drones').value
        self.base_cf_id = self.get_parameter('base_cf_id').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.world_frame = self.get_parameter('world_frame').value

        # TF buffer and listener
        # Increased cache time to 30s to handle delayed TF data from simulation
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # QoS for odometry (sensor data)
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # Create publishers for each drone
        self.odom_publishers = {}
        self.drone_frames = {}
        self.last_positions = {}
        self.last_times = {}

        for i in range(self.num_drones):
            cf_id = self.base_cf_id + i
            cf_name = f'cf{cf_id}'

            # Publisher: /cfXXX/odom
            topic = f'/{cf_name}/odom'
            self.odom_publishers[cf_name] = self.create_publisher(
                Odometry, topic, odom_qos)

            # Frame name in TF tree
            self.drone_frames[cf_name] = cf_name

            # For velocity estimation
            self.last_positions[cf_name] = None
            self.last_times[cf_name] = None

            self.get_logger().info(f'Bridge: TF({self.world_frame}->{cf_name}) -> {topic}')

        # Timer for publishing
        period = 1.0 / self.publish_rate
        self.timer = self.create_timer(period, self.publish_odom)

        self.get_logger().info(
            f'TF->Odom bridge started: {self.num_drones} drones, {self.publish_rate}Hz')

    def publish_odom(self):
        """Look up TF for each drone and publish odometry."""
        now = self.get_clock().now()

        for cf_name, frame in self.drone_frames.items():
            try:
                # Look up transform: world -> cfXXX
                # Use longer timeout to avoid missing TF data in simulation
                transform = self.tf_buffer.lookup_transform(
                    self.world_frame,
                    frame,
                    rclpy.time.Time(),  # Latest available
                    timeout=Duration(seconds=0.1)
                )

                # Create odometry message
                odom = self.transform_to_odom(cf_name, transform, now)

                # Publish
                self.odom_publishers[cf_name].publish(odom)

            except Exception as e:
                # TF not available yet - this is normal during startup
                pass

    def transform_to_odom(self, cf_name: str, transform: TransformStamped,
                          now: rclpy.time.Time) -> Odometry:
        """Convert TF transform to Odometry message with velocity estimation."""
        odom = Odometry()
        # Use TF timestamp (sim_time) instead of wall clock!
        odom.header.stamp = transform.header.stamp
        odom.header.frame_id = self.world_frame
        odom.child_frame_id = cf_name

        # Position from TF
        t = transform.transform.translation
        odom.pose.pose.position.x = t.x
        odom.pose.pose.position.y = t.y
        odom.pose.pose.position.z = t.z

        # Orientation from TF
        r = transform.transform.rotation
        odom.pose.pose.orientation.x = r.x
        odom.pose.pose.orientation.y = r.y
        odom.pose.pose.orientation.z = r.z
        odom.pose.pose.orientation.w = r.w

        # Velocity estimation (finite difference) using TF timestamp
        current_pos = np.array([t.x, t.y, t.z])
        tf_stamp = transform.header.stamp
        current_time = tf_stamp.sec + tf_stamp.nanosec / 1e9

        if self.last_positions[cf_name] is not None and self.last_times[cf_name] is not None:
            dt = current_time - self.last_times[cf_name]
            if dt > 0.001:  # Avoid division by very small dt
                velocity = (current_pos - self.last_positions[cf_name]) / dt
                odom.twist.twist.linear.x = velocity[0]
                odom.twist.twist.linear.y = velocity[1]
                odom.twist.twist.linear.z = velocity[2]

        # Store for next iteration
        self.last_positions[cf_name] = current_pos
        self.last_times[cf_name] = current_time

        # Covariance (approximate values for motion capture)
        # Position covariance: ~1mm accuracy
        pos_var = 0.001 ** 2
        odom.pose.covariance[0] = pos_var   # x
        odom.pose.covariance[7] = pos_var   # y
        odom.pose.covariance[14] = pos_var  # z

        # Velocity covariance: estimated from finite difference
        vel_var = 0.01 ** 2
        odom.twist.covariance[0] = vel_var  # vx
        odom.twist.covariance[7] = vel_var  # vy
        odom.twist.covariance[14] = vel_var  # vz

        return odom


def main(args=None):
    rclpy.init(args=args)
    node = TfToOdomBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
