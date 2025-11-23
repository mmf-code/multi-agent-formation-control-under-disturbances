#!/usr/bin/env python3
"""
ROS2 Launch File: Single-Agent Gazebo Simulation with Formation Control

This launch file orchestrates the complete ROS2-Gazebo simulation environment:
  1. Gazebo Server (gzserver) - Physics simulation engine
  2. Gazebo Client (gzclient) - 3D visualization GUI
  3. Agent Controller Node - PID/Fuzzy controller for drone
  4. Formation Coordinator Node - Publishes target positions

Usage:
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    ros2 launch agent_control_pkg gazebo_single_agent.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch import conditions
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_agent_control = get_package_share_directory('agent_control_pkg')

    world_file = os.path.join(pkg_agent_control, 'worlds', 'minimal_test.world')
    config_file = os.path.join(pkg_agent_control, 'config', 'ros2', 'agent_controller_default.yaml')

    model_path = os.path.join(pkg_agent_control, 'models')
    plugin_path = os.path.join(pkg_agent_control, '..', '..', 'lib')

    os.environ['GAZEBO_MODEL_PATH'] = f"{model_path}:{os.environ.get('GAZEBO_MODEL_PATH', '')}"
    os.environ['GAZEBO_PLUGIN_PATH'] = f"{plugin_path}:{os.environ.get('GAZEBO_PLUGIN_PATH', '')}"

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui = LaunchConfiguration('gui', default='true')

    gzserver_cmd = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_file],
        output='screen',
        additional_env={
            'GAZEBO_PLUGIN_PATH': os.environ.get('GAZEBO_PLUGIN_PATH', ''),
            'GAZEBO_MODEL_PATH': os.environ.get('GAZEBO_MODEL_PATH', ''),
            'LD_LIBRARY_PATH': os.environ.get('LD_LIBRARY_PATH', '')
        }
    )

    gzclient_cmd = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        condition=conditions.IfCondition(gui)
    )

    agent_controller_node = Node(
        package='agent_control_pkg',
        executable='agent_controller_node',
        name='agent_controller',
        namespace='agent_0',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('target_pose', '/agent_0/target_pose'),
            ('odom', '/agent_0/odom'),
            ('cmd_accel', '/agent_0/cmd_accel'),
            ('diagnostics', '/agent_0/diagnostics'),
        ]
    )

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

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),
        DeclareLaunchArgument('gui', default_value='true', description='Launch Gazebo GUI'),
        gzserver_cmd,
        gzclient_cmd,
        agent_controller_node,
        formation_coordinator_node,
    ])
