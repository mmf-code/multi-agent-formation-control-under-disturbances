#!/usr/bin/env python3
"""
Wind Publisher Node for Formation Control Simulation

Publishes wind disturbance to /wind/velocity for use by simple_drone_plugin.
Supports multiple wind profiles: constant, sinusoidal, step, gust, stochastic.

Usage:
    ros2 run agent_control_pkg wind_publisher.py --ros-args -p profile:=sinusoidal -p magnitude:=0.5

Parameters:
    profile (str): Wind profile type - constant, sinusoidal, step, gust, stochastic (default: sinusoidal)
    magnitude (float): Base wind magnitude in m/s (default: 0.3)
    direction (float): Wind direction in degrees (0 = +X axis) (default: 0.0)
    frequency (float): Frequency for sinusoidal profile in Hz (default: 0.1)
    step_time (float): Time for step change in seconds (default: 30.0)
    gust_duration (float): Gust duration in seconds (default: 2.0)
    gust_interval (float): Time between gusts in seconds (default: 15.0)

    # Stochastic profile parameters (for thesis - uncertainty handling)
    stochastic_mag_std (float): Std dev of magnitude variation (default: 1.0 m/s)
    stochastic_dir_rate (float): Direction change rate in deg/s (default: 10.0)
    stochastic_gust_prob (float): Probability of random gust per second (default: 0.1)
    stochastic_gust_mag (float): Random gust magnitude multiplier (default: 2.0)
    stochastic_turbulence (float): High-freq turbulence intensity (default: 0.3)

Topics Published:
    /wind/velocity (geometry_msgs/Vector3): Wind velocity in m/s

Author: Rosie (ROS2 Systems Engineer Agent)
Date: 2025-12
"""

import math
import random
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3


class WindPublisher(Node):
    """Publishes configurable wind disturbance for drone simulation."""

    def __init__(self):
        super().__init__('wind_publisher')

        # Declare parameters
        self.declare_parameter('profile', 'sinusoidal')
        self.declare_parameter('magnitude', 0.3)
        self.declare_parameter('direction', 0.0)  # degrees
        self.declare_parameter('frequency', 0.1)  # Hz for sinusoidal
        self.declare_parameter('step_time', 30.0)  # seconds
        self.declare_parameter('gust_duration', 2.0)  # seconds
        self.declare_parameter('gust_interval', 15.0)  # seconds
        self.declare_parameter('publish_rate', 10.0)  # Hz

        # Stochastic profile parameters
        self.declare_parameter('stochastic_mag_std', 1.0)  # magnitude std dev
        self.declare_parameter('stochastic_dir_rate', 10.0)  # direction change rate deg/s
        self.declare_parameter('stochastic_gust_prob', 0.1)  # gust probability per second
        self.declare_parameter('stochastic_gust_mag', 2.0)  # gust magnitude multiplier
        self.declare_parameter('stochastic_turbulence', 0.3)  # turbulence intensity
        self.declare_parameter('random_seed', -1)  # -1 for random seed, else fixed

        # Get parameters
        self.profile = self.get_parameter('profile').get_parameter_value().string_value
        self.magnitude = self.get_parameter('magnitude').get_parameter_value().double_value
        self.direction_deg = self.get_parameter('direction').get_parameter_value().double_value
        self.frequency = self.get_parameter('frequency').get_parameter_value().double_value
        self.step_time = self.get_parameter('step_time').get_parameter_value().double_value
        self.gust_duration = self.get_parameter('gust_duration').get_parameter_value().double_value
        self.gust_interval = self.get_parameter('gust_interval').get_parameter_value().double_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        # Get stochastic parameters
        self.stoch_mag_std = self.get_parameter('stochastic_mag_std').get_parameter_value().double_value
        self.stoch_dir_rate = self.get_parameter('stochastic_dir_rate').get_parameter_value().double_value
        self.stoch_gust_prob = self.get_parameter('stochastic_gust_prob').get_parameter_value().double_value
        self.stoch_gust_mag = self.get_parameter('stochastic_gust_mag').get_parameter_value().double_value
        self.stoch_turbulence = self.get_parameter('stochastic_turbulence').get_parameter_value().double_value
        random_seed = self.get_parameter('random_seed').get_parameter_value().integer_value

        # Initialize random seed (for reproducibility in experiments)
        if random_seed >= 0:
            random.seed(random_seed)
            self.get_logger().info(f'Using fixed random seed: {random_seed}')

        # Convert direction to radians
        self.direction_rad = math.radians(self.direction_deg)

        # Stochastic state variables
        self.stoch_current_dir = self.direction_rad  # current wandering direction
        self.stoch_current_mag = self.magnitude  # smoothed magnitude
        self.stoch_gust_active = False
        self.stoch_gust_remaining = 0.0  # seconds of gust remaining
        self.stoch_gust_intensity = 0.0  # current gust intensity

        # Publisher
        self.wind_pub = self.create_publisher(Vector3, '/wind/velocity', 10)

        # Timer
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_wind)

        # Start time for time-varying profiles
        self.start_time = self.get_clock().now()
        self.last_gust_time = self.start_time

        self.get_logger().info(
            f'Wind Publisher started: profile={self.profile}, '
            f'magnitude={self.magnitude:.2f} m/s, direction={self.direction_deg:.1f} deg'
        )

    def publish_wind(self):
        """Compute and publish wind velocity based on selected profile."""
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9  # seconds

        # Compute magnitude based on profile
        if self.profile == 'constant':
            current_mag = self.magnitude

        elif self.profile == 'sinusoidal':
            # Sinusoidal variation: magnitude * (0.5 + 0.5 * sin(2*pi*f*t))
            # Range: [0, magnitude]
            current_mag = self.magnitude * (0.5 + 0.5 * math.sin(2 * math.pi * self.frequency * elapsed))

        elif self.profile == 'step':
            # Step change at step_time
            current_mag = self.magnitude if elapsed >= self.step_time else 0.0

        elif self.profile == 'gust':
            # Periodic gusts
            time_since_last_gust = (now - self.last_gust_time).nanoseconds / 1e9
            if time_since_last_gust >= self.gust_interval:
                self.last_gust_time = now
                time_since_last_gust = 0.0

            if time_since_last_gust < self.gust_duration:
                # Gust active: smooth ramp up/down
                gust_progress = time_since_last_gust / self.gust_duration
                current_mag = self.magnitude * math.sin(math.pi * gust_progress)
            else:
                current_mag = 0.0

        elif self.profile == 'ramp':
            # Linear ramp over step_time seconds
            if elapsed < self.step_time:
                current_mag = self.magnitude * (elapsed / self.step_time)
            else:
                current_mag = self.magnitude

        elif self.profile == 'stochastic':
            # Stochastic wind: CONTINUOUS base wind + random gust intensifications
            # This is the THESIS profile for testing uncertainty handling
            #
            # Key design:
            # - ALWAYS blowing at base magnitude (continuous disturbance)
            # - Occasional random gusts MULTIPLY the base (1.5x - 2.5x stronger)
            # - Small direction wandering around base direction
            # - Small turbulence overlay
            dt = 1.0 / 10.0  # Assume 10 Hz publish rate

            # 1. Small direction wandering (±15° from base direction)
            # Ornstein-Uhlenbeck process with stronger mean reversion
            dir_noise = random.gauss(0, math.radians(self.stoch_dir_rate) * dt * 0.5)
            mean_reversion = 0.3 * (self.direction_rad - self.stoch_current_dir) * dt
            self.stoch_current_dir += dir_noise + mean_reversion
            # Keep within ±30° of base direction
            max_wander = math.radians(30)
            dir_diff = self.stoch_current_dir - self.direction_rad
            if abs(dir_diff) > max_wander:
                self.stoch_current_dir = self.direction_rad + max_wander * (1 if dir_diff > 0 else -1)

            # 2. CONTINUOUS base magnitude (always blowing!)
            # Small variations around base (±20%)
            base_variation = random.gauss(1.0, 0.1)  # 10% std dev
            base_variation = max(0.8, min(1.2, base_variation))  # clamp to ±20%
            continuous_mag = self.magnitude * base_variation

            # 3. Random gust events (Poisson process) - MULTIPLY base wind
            if not self.stoch_gust_active:
                # Check for new gust (probability per timestep)
                if random.random() < self.stoch_gust_prob * dt:
                    self.stoch_gust_active = True
                    self.stoch_gust_remaining = random.uniform(1.0, 4.0)  # longer gusts
                    self.stoch_gust_intensity = random.uniform(1.5, self.stoch_gust_mag)
                    self.get_logger().info(
                        f'⚡ GUST! {self.stoch_gust_intensity:.1f}x stronger for {self.stoch_gust_remaining:.1f}s'
                    )
            else:
                self.stoch_gust_remaining -= dt
                if self.stoch_gust_remaining <= 0:
                    self.stoch_gust_active = False
                    self.stoch_gust_intensity = 1.0
                    self.get_logger().info('💨 Gust ended, back to base wind')

            # 4. Compute current magnitude (base * gust_factor)
            gust_factor = self.stoch_gust_intensity if self.stoch_gust_active else 1.0
            current_mag = continuous_mag * gust_factor

            # 5. Small high-frequency turbulence
            turbulence_x = random.gauss(0, self.stoch_turbulence * 0.5)
            turbulence_y = random.gauss(0, self.stoch_turbulence * 0.5)

            # Use wandering direction
            msg = Vector3()
            msg.x = current_mag * math.cos(self.stoch_current_dir) + turbulence_x
            msg.y = current_mag * math.sin(self.stoch_current_dir) + turbulence_y
            msg.z = 0.0

            self.wind_pub.publish(msg)
            return  # Early return since we already published

        elif self.profile == 'aggressive':
            # AGGRESSIVE OUTDOOR wind: Sharp gusts + direction shifts
            # Designed to stress-test controllers and show fuzzy advantages
            #
            # Key differences from 'stochastic':
            # - SHORTER, SHARPER gusts (0.3-0.8s instead of 1-4s)
            # - HIGHER intensity (3x-5x instead of 1.5-2.5x)
            # - MORE FREQUENT gusts (higher probability)
            # - SUDDEN direction shifts (not just wandering)
            dt = 1.0 / 10.0

            # 1. Direction: Occasional SUDDEN shifts (not gradual)
            if random.random() < 0.02 * dt:  # 2% chance per second of sudden shift
                shift = random.choice([-1, 1]) * random.uniform(30, 60)  # 30-60 degree jump
                self.stoch_current_dir += math.radians(shift)
                self.get_logger().info(f'🔄 Direction shift: {shift:+.0f}°')
            else:
                # Small wandering
                dir_noise = random.gauss(0, math.radians(5) * dt)
                self.stoch_current_dir += dir_noise

            # Keep within ±90° of base
            max_wander = math.radians(90)
            dir_diff = self.stoch_current_dir - self.direction_rad
            if abs(dir_diff) > max_wander:
                self.stoch_current_dir = self.direction_rad + max_wander * (1 if dir_diff > 0 else -1)

            # 2. Base magnitude with more variation
            base_variation = random.gauss(1.0, 0.15)
            base_variation = max(0.7, min(1.3, base_variation))
            continuous_mag = self.magnitude * base_variation

            # 3. SHARP, INTENSE gusts - more frequent
            if not self.stoch_gust_active:
                # Higher probability: 25% per second (vs 10%)
                if random.random() < 0.25 * dt:
                    self.stoch_gust_active = True
                    self.stoch_gust_remaining = random.uniform(0.3, 0.8)  # SHORT: 0.3-0.8s
                    self.stoch_gust_intensity = random.uniform(3.0, 5.0)  # HIGH: 3x-5x
                    self.get_logger().info(
                        f'💥 SHARP GUST! {self.stoch_gust_intensity:.1f}x for {self.stoch_gust_remaining:.2f}s'
                    )
            else:
                self.stoch_gust_remaining -= dt
                if self.stoch_gust_remaining <= 0:
                    self.stoch_gust_active = False
                    self.stoch_gust_intensity = 1.0

            # 4. Compute magnitude
            gust_factor = self.stoch_gust_intensity if self.stoch_gust_active else 1.0
            current_mag = continuous_mag * gust_factor

            # 5. Higher turbulence
            turbulence_x = random.gauss(0, self.stoch_turbulence)
            turbulence_y = random.gauss(0, self.stoch_turbulence)

            msg = Vector3()
            msg.x = current_mag * math.cos(self.stoch_current_dir) + turbulence_x
            msg.y = current_mag * math.sin(self.stoch_current_dir) + turbulence_y
            msg.z = 0.0

            self.wind_pub.publish(msg)
            return

        else:
            self.get_logger().warn(f'Unknown wind profile: {self.profile}, using constant')
            current_mag = self.magnitude

        # Convert magnitude and direction to velocity components
        msg = Vector3()
        msg.x = current_mag * math.cos(self.direction_rad)
        msg.y = current_mag * math.sin(self.direction_rad)
        msg.z = 0.0

        self.wind_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WindPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
