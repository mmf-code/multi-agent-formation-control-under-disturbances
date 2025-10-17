#!/usr/bin/env python3
"""
Professional Formation Control Demonstration Launch File

This launch file provides a complete, presentation-ready ROS2-Gazebo simulation
environment for multi-agent formation control with PID/Fuzzy controllers.

Features:
  - Professional Gazebo world with visual aids (grid, axes, scale markers)
  - RViz2 visualization with real-time metrics and trajectory tracking
  - Agent controller with tuned PID parameters (Kp=0.538, Ki=0.145, Kd=1.368)
  - Formation coordinator for target generation
  - Automatic trajectory path visualization
  - Recording-ready configuration for presentations

Usage:
    # Full demo with GUI (Gazebo + RViz2)
    ros2 launch agent_control_pkg demo_presentation.launch.py

    # Headless mode (no Gazebo GUI)
    ros2 launch agent_control_pkg demo_presentation.launch.py gazebo_gui:=false

    # Without RViz2
    ros2 launch agent_control_pkg demo_presentation.launch.py rviz:=false

    # Different controller scenario (select config file)
    ros2 launch agent_control_pkg demo_presentation.launch.py controller_config:=pd

    # Available scenarios: p_only, pd, pi, pid, pid_fuzzy

Launch Arguments:
    gazebo_gui (bool): Launch Gazebo client GUI (default: true)
    rviz (bool): Launch RViz2 visualization (default: true)
    use_sim_time (bool): Use Gazebo simulation time (default: true)
    controller_config (string): Controller config file name (default: pid)
    world_file (string): Gazebo world file (default: minimal_test.world)

Author: Multi-Agent Formation Control Team
Date: 2025-10-17
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch import conditions
from launch.actions import ExecuteProcess, DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for formation control demonstration"""

    # Package directories
    pkg_agent_control = get_package_share_directory('agent_control_pkg')

    # File paths
    world_file_arg = LaunchConfiguration('world_file', default='minimal_test.world')
    controller_config_arg = LaunchConfiguration('controller_config', default='pid')

    # Use a substitution so overrides via world_file:=... are honored
    world_file = PathJoinSubstitution([FindPackageShare('agent_control_pkg'), 'worlds', world_file_arg])

    # Dynamically select controller config file based on scenario
    # Maps controller_config argument to actual YAML file
    # Build filename: agent_controller_{config}.yaml
    config_file = PathJoinSubstitution([
        FindPackageShare('agent_control_pkg'),
        'config',
        'ros2',
        PythonExpression(['"agent_controller_" + "', controller_config_arg, '" + ".yaml"'])
    ])

    rviz_config = os.path.join(pkg_agent_control, 'rviz', 'formation_demo.rviz')

    # Gazebo paths
    model_path = os.path.join(pkg_agent_control, 'models')
    plugin_path = os.path.join(pkg_agent_control, '..', '..', 'lib')

    # Set environment variables for Gazebo
    os.environ['GAZEBO_MODEL_PATH'] = f"{model_path}:{os.environ.get('GAZEBO_MODEL_PATH', '')}"
    os.environ['GAZEBO_PLUGIN_PATH'] = f"{plugin_path}:{os.environ.get('GAZEBO_PLUGIN_PATH', '')}"

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gazebo_gui = LaunchConfiguration('gazebo_gui', default='true')
    rviz_enabled = LaunchConfiguration('rviz', default='true')

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time from Gazebo'
    )

    declare_gazebo_gui = DeclareLaunchArgument(
        'gazebo_gui',
        default_value='true',
        description='Launch Gazebo client GUI'
    )

    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2 visualization'
    )

    declare_controller_config = DeclareLaunchArgument(
        'controller_config',
        default_value='pid',
        description='Controller scenario: p_only, pd, pi, pid, pid_fuzzy'
    )

    declare_world_file = DeclareLaunchArgument(
        'world_file',
        default_value='minimal_test.world',
        description='Gazebo world file name (minimal_test.world or demo_presentation.world)'
    )

    # ===== Gazebo Simulation =====

    # Gazebo server (physics simulation)
    gzserver = ExecuteProcess(
        cmd=[
            'gzserver',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            world_file
        ],
        output='screen',
        additional_env={
            'GAZEBO_PLUGIN_PATH': os.environ.get('GAZEBO_PLUGIN_PATH', ''),
            'GAZEBO_MODEL_PATH': os.environ.get('GAZEBO_MODEL_PATH', ''),
            'LD_LIBRARY_PATH': os.environ.get('LD_LIBRARY_PATH', '')
        }
    )

    # Gazebo client (3D visualization)
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        condition=conditions.IfCondition(gazebo_gui)
    )

    # ===== ROS2 Controller Nodes =====

    # Agent controller node (PID/Fuzzy control)
    # Controller parameters now loaded from scenario-specific config file
    agent_controller = Node(
        package='agent_control_pkg',
        executable='agent_controller_node',
        name='agent_controller',
        namespace='agent_0',
        output='screen',
        parameters=[
            config_file,  # Dynamically selected based on controller_config argument
            {
                'use_sim_time': use_sim_time,
                # All PID/Fuzzy parameters loaded from config file
                # No hardcoded overrides - use config files for tuning
            }
        ],
        remappings=[
            ('target_pose', '/agent_0/target_pose'),
            ('odom', '/agent_0/odom'),
            ('cmd_accel', '/agent_0/cmd_accel'),
            ('diagnostics', '/agent_0/diagnostics'),
        ]
    )

    # Formation coordinator node (target generation)
    pkg_formation_coordinator = get_package_share_directory('formation_coordinator_pkg')
    formation_config = os.path.join(pkg_formation_coordinator, 'config', 'formation_config.yaml')

    formation_coordinator = Node(
        package='formation_coordinator_pkg',
        executable='formation_coordinator_node',
        name='formation_coordinator',
        output='screen',
        parameters=[
            formation_config,
            {'use_sim_time': use_sim_time}
        ]
    )

    # ===== Visualization Tools =====

    # RViz2 (3D visualization and diagnostics)
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=conditions.IfCondition(rviz_enabled)
    )

    # Path visualization node (trajectory tracking)
    path_visualizer = Node(
        package='agent_control_pkg',
        executable='path_visualizer_node',
        name='path_visualizer',
        namespace='agent_0',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('odom', '/agent_0/odom'),
            ('path', '/agent_0/path'),
        ],
        condition=conditions.IfCondition(rviz_enabled)
    )

    # Metrics publisher node (real-time performance metrics)
    metrics_publisher = Node(
        package='agent_control_pkg',
        executable='metrics_publisher_node',
        name='metrics_publisher',
        namespace='agent_0',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'publish_rate_hz': 10.0},
            {'settling_threshold': 0.05},
            {'settling_time_window': 2.0},
        ],
        remappings=[
            ('odom', '/agent_0/odom'),
            ('target_pose', '/agent_0/target_pose'),
            ('metrics', '/agent_0/metrics'),
        ]
    )

    # ===== Launch Sequence =====

    # Start Gazebo immediately
    gazebo_start = [gzserver, gzclient]

    # Wait 3 seconds for Gazebo to initialize, then start ROS2 nodes
    ros2_nodes_delayed = TimerAction(
        period=3.0,
        actions=[
            agent_controller,
            formation_coordinator,
            metrics_publisher,
        ]
    )

    # Start RViz2 after controllers are running (5 seconds total)
    rviz_delayed = TimerAction(
        period=5.0,
        actions=[rviz2, path_visualizer]
    )

    # ===== Launch Description =====

    return LaunchDescription([
        # Launch arguments
        declare_use_sim_time,
        declare_gazebo_gui,
        declare_rviz,
        declare_controller_config,  # Updated: controller_config instead of controller_type
        declare_world_file,

        # Start Gazebo
        *gazebo_start,

        # Start ROS2 nodes (delayed)
        ros2_nodes_delayed,

        # Start visualization (delayed)
        rviz_delayed,
    ])
