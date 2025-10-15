#!/usr/bin/env python3
"""
Standalone ROS2 simulation with Python physics (like old C++ standalone).
No Gazebo needed - faster and with plots!
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, PoseStamped
from nav_msgs.msg import Odometry
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time

class SimpleDronePhysics:
    """Simple 2D drone physics model"""
    def __init__(self, mass=1.5, dt=0.001):
        self.mass = mass
        self.dt = dt

        # State
        self.x = 0.0
        self.y = 0.0
        self.z = 0.5
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

        # Command
        self.cmd_ax = 0.0
        self.cmd_ay = 0.0

    def set_command(self, ax, ay):
        """Set commanded acceleration"""
        self.cmd_ax = ax
        self.cmd_ay = ay

    def step(self):
        """Simulate one physics step"""
        # Apply acceleration
        self.vx += self.cmd_ax * self.dt
        self.vy += self.cmd_ay * self.dt

        # Simple drag
        drag = 0.1
        self.vx *= (1.0 - drag * self.dt)
        self.vy *= (1.0 - drag * self.dt)

        # Update position
        self.x += self.vx * self.dt
        self.y += self.vy * self.dt

        # Keep Z constant
        self.z = 0.5
        self.vz = 0.0


class ROS2SimulationNode(Node):
    """ROS2 node that simulates drone physics and publishes odom"""
    def __init__(self):
        super().__init__('ros2_simulation')

        # Physics
        self.drone = SimpleDronePhysics()

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/agent_0/odom', 10)

        # Subscribers
        self.cmd_sub = self.create_subscription(
            Vector3, '/agent_0/cmd_accel',
            self.cmd_callback, 10)

        # Fast physics timer (1000 Hz)
        self.physics_timer = self.create_timer(0.001, self.physics_step)

        # Slower odom publisher (50 Hz)
        self.odom_timer = self.create_timer(0.02, self.publish_odom)

        # Data logging
        self.time_data = []
        self.x_data = []
        self.y_data = []
        self.vx_data = []
        self.vy_data = []
        self.target_x = []
        self.target_y = []
        self.cmd_ax = []
        self.cmd_ay = []

        self.start_time = time.time()

        self.get_logger().info('Standalone ROS2 simulation started!')

    def cmd_callback(self, msg):
        """Receive acceleration commands from controller"""
        self.drone.set_command(msg.x, msg.y)

    def physics_step(self):
        """Fast physics update"""
        self.drone.step()

        # Log data
        t = time.time() - self.start_time
        self.time_data.append(t)
        self.x_data.append(self.drone.x)
        self.y_data.append(self.drone.y)
        self.vx_data.append(self.drone.vx)
        self.vy_data.append(self.drone.vy)
        self.cmd_ax.append(self.drone.cmd_ax)
        self.cmd_ay.append(self.drone.cmd_ay)

    def publish_odom(self):
        """Publish odometry"""
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'

        msg.pose.pose.position.x = self.drone.x
        msg.pose.pose.position.y = self.drone.y
        msg.pose.pose.position.z = self.drone.z
        msg.pose.pose.orientation.w = 1.0

        msg.twist.twist.linear.x = self.drone.vx
        msg.twist.twist.linear.y = self.drone.vy
        msg.twist.twist.linear.z = 0.0

        self.odom_pub.publish(msg)

    def plot_results(self):
        """Generate plots like old standalone simulation"""
        fig, axes = plt.subplots(2, 2, figsize=(12, 8))

        t = np.array(self.time_data)

        # Position
        axes[0, 0].plot(t, self.x_data, 'b-', label='X position')
        axes[0, 0].plot(t, self.y_data, 'r-', label='Y position')
        axes[0, 0].axhline(y=2.309, color='g', linestyle='--', label='Target Y')
        axes[0, 0].set_xlabel('Time (s)')
        axes[0, 0].set_ylabel('Position (m)')
        axes[0, 0].set_title('Position vs Time')
        axes[0, 0].legend()
        axes[0, 0].grid(True)

        # Velocity
        axes[0, 1].plot(t, self.vx_data, 'b-', label='Vx')
        axes[0, 1].plot(t, self.vy_data, 'r-', label='Vy')
        axes[0, 1].set_xlabel('Time (s)')
        axes[0, 1].set_ylabel('Velocity (m/s)')
        axes[0, 1].set_title('Velocity vs Time')
        axes[0, 1].legend()
        axes[0, 1].grid(True)

        # Commands
        axes[1, 0].plot(t, self.cmd_ax, 'b-', label='Ax command')
        axes[1, 0].plot(t, self.cmd_ay, 'r-', label='Ay command')
        axes[1, 0].set_xlabel('Time (s)')
        axes[1, 0].set_ylabel('Acceleration (m/s²)')
        axes[1, 0].set_title('Controller Commands')
        axes[1, 0].legend()
        axes[1, 0].grid(True)

        # Trajectory
        axes[1, 1].plot(self.x_data, self.y_data, 'b-', linewidth=2, label='Trajectory')
        axes[1, 1].plot(0, 0, 'go', markersize=10, label='Start')
        axes[1, 1].plot(0, 2.309, 'r*', markersize=15, label='Target')
        if len(self.x_data) > 0:
            axes[1, 1].plot(self.x_data[-1], self.y_data[-1], 'bs',
                          markersize=10, label='Current')
        axes[1, 1].set_xlabel('X (m)')
        axes[1, 1].set_ylabel('Y (m)')
        axes[1, 1].set_title('2D Trajectory')
        axes[1, 1].legend()
        axes[1, 1].grid(True)
        axes[1, 1].axis('equal')

        plt.tight_layout()
        plt.savefig('ros2_simulation_results.png', dpi=150)
        print(f"\n✅ Plot saved: ros2_simulation_results.png")
        plt.show()


def main():
    rclpy.init()

    sim_node = ROS2SimulationNode()

    print("\n" + "="*70)
    print("  ROS2 STANDALONE SIMULATION (No Gazebo!)")
    print("="*70)
    print("\nSimulation running... Press Ctrl+C to stop and see plots\n")

    try:
        rclpy.spin(sim_node)
    except KeyboardInterrupt:
        print("\n\n⏹️  Simulation stopped!")
        print(f"📊 Total time: {sim_node.time_data[-1]:.2f} seconds")
        print(f"📍 Final position: ({sim_node.drone.x:.3f}, {sim_node.drone.y:.3f})")
        print(f"🎯 Target: (0.000, 2.309)")
        error = abs(sim_node.drone.y - 2.309)
        print(f"📏 Error: {error:.4f} m")

        print("\n🎨 Generating plots...")
        sim_node.plot_results()
    finally:
        sim_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
