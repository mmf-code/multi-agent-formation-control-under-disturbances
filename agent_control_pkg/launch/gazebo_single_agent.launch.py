#!/usr/bin/env python3
"""
ROS2 Launch File: Single-Agent Gazebo Simulation with Formation Control

This launch file orchestrates the complete ROS2-Gazebo simulation environment:
  1. Gazebo Server (gzserver) - Physics simulation engine
  2. Gazebo Client (gzclient) - 3D visualization GUI
  3. Agent Controller Node - PID/Fuzzy controller for drone
  4. Formation Coordinator Node - Publishes target positions

System Architecture (3 layers):
  ┌────────────────────────────────────────┐
  │  Formation Coordinator                 │  <-- High-level planning
  │  - Publishes target poses              │
  └──────────────┬─────────────────────────┘
                 │ /agent_X/target_pose
  ┌──────────────▼─────────────────────────┐
  │  Agent Controller Node                 │  <-- Mid-level control
  │  - PID/Fuzzy controller                │
  │  - Reads: odom, target_pose            │
  │  - Writes: cmd_accel                   │
  └──────────────┬─────────────────────────┘
                 │ /agent_X/cmd_accel
  ┌──────────────▼─────────────────────────┐
  │  Gazebo + SimpleDronePlugin            │  <-- Low-level physics
  │  - Applies forces (F = m*a)            │
  │  - Publishes: odom                     │
  └────────────────────────────────────────┘

Usage:
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    ros2 launch agent_control_pkg gazebo_single_agent.launch.py

Optional arguments:
    gui:=false      # Launch without Gazebo GUI (headless)
    verbose:=true   # Enable verbose Gazebo output

Author: Multi-Agent Formation Control Team
Date: 2025
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch import conditions
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Generate launch description for single-agent Gazebo simulation.

    Returns:
        LaunchDescription: Complete launch configuration with all nodes
    """
    # ===== Package Directories =====
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_agent_control = get_package_share_directory('agent_control_pkg')

    # ===== File Paths =====
    # World file: contains drone model, physics settings, and plugin configuration
    world_file = os.path.join(pkg_agent_control, 'worlds', 'minimal_test.world')

    # Controller config: PID gains, output limits, controller type
    config_file = os.path.join(pkg_agent_control, 'config', 'ros2', 'agent_controller_default.yaml')

    # ===== Gazebo Environment Setup =====
    # Set paths for Gazebo to find models and plugins
    model_path = os.path.join(pkg_agent_control, 'models')
    plugin_path = os.path.join(pkg_agent_control, '..', '..', 'lib')  # install/agent_control_pkg/lib

    # Append to existing paths (don't overwrite)
    gazebo_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    gazebo_plugin_path = os.environ.get('GAZEBO_PLUGIN_PATH', '')

    os.environ['GAZEBO_MODEL_PATH'] = f"{model_path}:{gazebo_model_path}"
    os.environ['GAZEBO_PLUGIN_PATH'] = f"{plugin_path}:{gazebo_plugin_path}"

    # ===== Launch Arguments =====
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui = LaunchConfiguration('gui', default='true')
    verbose = LaunchConfiguration('verbose', default='false')

    # ===== Gazebo Server (Physics Engine) =====
    # Runs headless physics simulation at ~1kHz
    # Plugins: gazebo_ros_init (ROS2 integration), gazebo_ros_factory (model spawning)
    gzserver_cmd = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_file],
        output='screen',
        additional_env={
            'GAZEBO_PLUGIN_PATH': os.environ.get('GAZEBO_PLUGIN_PATH', ''),
            'GAZEBO_MODEL_PATH': os.environ.get('GAZEBO_MODEL_PATH', ''),
            'LD_LIBRARY_PATH': os.environ.get('LD_LIBRARY_PATH', '')
        }
    )

    # ===== Gazebo Client (3D Visualization) =====
    # Optional GUI for visualization (can be disabled with gui:=false)
    gzclient_cmd = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        condition=conditions.IfCondition(gui)
    )

    # ===== Agent Controller Node =====
    # Mid-level control: implements PID/Fuzzy/Hybrid controller
    # Input:  /agent_0/odom (state feedback)
    #         /agent_0/target_pose (reference from formation coordinator)
    # Output: /agent_0/cmd_accel (acceleration commands)
    #         /agent_0/diagnostics (controller metrics)
    agent_controller_node = Node(
        package='agent_control_pkg',
        executable='agent_controller_node',
        name='agent_controller',
        namespace='agent_0',
        output='screen',
        parameters=[
            config_file,  # Load PID gains and controller config
            {'use_sim_time': use_sim_time}  # Sync with Gazebo time
        ],
        remappings=[
            # Explicit topic remappings (for clarity and multi-agent compatibility)
            ('target_pose', '/agent_0/target_pose'),
            ('odom', '/agent_0/odom'),
            ('cmd_accel', '/agent_0/cmd_accel'),
            ('diagnostics', '/agent_0/diagnostics'),
        ]
    )

    # ===== Formation Coordinator Node =====
    # High-level planning: publishes target positions for agents
    # Output: /agent_X/target_pose (one per agent)
    # Uses triangular formation with configurable spacing
    formation_config = os.path.join(
        get_package_share_directory('formation_coordinator_pkg'),
        'config', 'formation_config.yaml'
    )

    formation_coordinator_node = Node(
        package='formation_coordinator_pkg',
        executable='formation_coordinator_node',
        name='formation_coordinator',
        output='screen',
        parameters=[
            formation_config,
            {'use_sim_time': use_sim_time}
        ]
    )

    # ===== Launch Description =====
    # Defines startup sequence and configuration
    return LaunchDescription([
        # Declare command-line arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation time (true) or wall clock time (false)'
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Launch Gazebo GUI (gzclient) for visualization'
        ),
        DeclareLaunchArgument(
            'verbose',
            default_value='false',
            description='Enable verbose Gazebo logging output'
        ),

        # Launch nodes in order
        # Note: Gazebo starts first, then controllers connect to it
        gzserver_cmd,              # Physics simulation
        gzclient_cmd,              # 3D visualization (optional)
        agent_controller_node,     # PID/Fuzzy controller
        formation_coordinator_node,  # Target pose publisher
    ])
