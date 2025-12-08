#!/usr/bin/env python3
"""
TF to Odometry Bridge for Crazyswarm2

Subscribes to TF and publishes nav_msgs/Odometry for each Crazyflie.
This bridges Crazyswarm2's TF-based pose to our controller's odom input.

Usage:
  ros2 run agent_control_pkg tf_to_odom_bridge.py
  # or
  python3 scripts/tf_to_odom_bridge.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
from tf2_ros import TransformException


class TfToOdomBridge(Node):
    def __init__(self):
        super().__init__('tf_to_odom_bridge')

        # Parameters
        self.declare_parameter('cf_ids', ['cf231', 'cf232', 'cf233'])
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('publish_rate', 100.0)  # Hz

        cf_ids = self.get_parameter('cf_ids').value
        self.world_frame = self.get_parameter('world_frame').value
        publish_rate = self.get_parameter('publish_rate').value

        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Create publishers for each Crazyflie
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.odom_pubs = {}
        self.last_transforms = {}

        for cf_id in cf_ids:
            topic = f'/{cf_id}/odom'
            self.odom_pubs[cf_id] = self.create_publisher(Odometry, topic, qos)
            self.last_transforms[cf_id] = None
            self.get_logger().info(f'Publishing odom on {topic}')

        self.cf_ids = cf_ids

        # Timer for publishing
        period = 1.0 / publish_rate
        self.timer = self.create_timer(period, self.publish_odom)

        self.get_logger().info(f'TF to Odom bridge started for {cf_ids}')

    def publish_odom(self):
        now = self.get_clock().now()

        for cf_id in self.cf_ids:
            try:
                # Get transform from world to crazyflie
                transform = self.tf_buffer.lookup_transform(
                    self.world_frame,
                    cf_id,
                    rclpy.time.Time(),  # Get latest
                    timeout=rclpy.duration.Duration(seconds=0.01)
                )

                # Create odometry message
                odom = Odometry()
                odom.header.stamp = now.to_msg()
                odom.header.frame_id = self.world_frame
                odom.child_frame_id = cf_id

                # Position from TF
                odom.pose.pose.position.x = transform.transform.translation.x
                odom.pose.pose.position.y = transform.transform.translation.y
                odom.pose.pose.position.z = transform.transform.translation.z
                odom.pose.pose.orientation = transform.transform.rotation

                # Estimate velocity from consecutive transforms
                if self.last_transforms[cf_id] is not None:
                    last_tf, last_time = self.last_transforms[cf_id]
                    dt = (now - last_time).nanoseconds / 1e9
                    if dt > 0.001:  # Avoid division by zero
                        odom.twist.twist.linear.x = (
                            transform.transform.translation.x - last_tf.transform.translation.x
                        ) / dt
                        odom.twist.twist.linear.y = (
                            transform.transform.translation.y - last_tf.transform.translation.y
                        ) / dt
                        odom.twist.twist.linear.z = (
                            transform.transform.translation.z - last_tf.transform.translation.z
                        ) / dt

                self.last_transforms[cf_id] = (transform, now)

                # Publish
                self.odom_pubs[cf_id].publish(odom)

            except TransformException as ex:
                # TF not available yet - this is normal at startup
                pass


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
