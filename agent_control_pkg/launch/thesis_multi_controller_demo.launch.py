#!/usr/bin/env python3
"""
Thesis Multi-Controller Comparison Demo - Launch File

This launch file creates a comprehensive thesis demonstration with:
  - 5 drones running simultaneously, each with different controller
  - High-performance Gazebo simulation (optimized for FPS)
  - Triangle formation coordination (first 3 drones)
  - Individual path visualization for each drone
  - Metrics comparison dashboard

Controller Assignment:
  - agent_0: P-only (Red)
  - agent_1: PI (Green)
  - agent_2: PD (Blue)
  - agent_3: PID (Yellow)
  - agent_4: PID+Fuzzy (Magenta)

Usage:
    # Full demo with all visualizations
    ros2 launch agent_control_pkg thesis_multi_controller_demo.launch.py

    # Headless mode (maximum FPS)
    ros2 launch agent_control_pkg thesis_multi_controller_demo.launch.py gazebo_gui:=false rviz:=false

    # Triangle formation with first 3 drones only
    ros2 launch agent_control_pkg thesis_multi_controller_demo.launch.py num_agents:=3

Author: Multi-Agent Formation Control Research Team
Date: 2025-10-17
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch import conditions
from launch.actions import ExecuteProcess, DeclareLaunchArgument, TimerAction, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for multi-controller thesis demonstration"""

    # Package directories
    pkg_agent_control = get_package_share_directory('agent_control_pkg')
    pkg_formation_coordinator = get_package_share_directory('formation_coordinator_pkg')

    # World file - optimized for multiple drones
    world_file = PathJoinSubstitution([
        FindPackageShare('agent_control_pkg'),
        'worlds',
        'thesis_multi_drone.world'
    ])

    # RViz config
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
    num_agents = LaunchConfiguration('num_agents', default='5')

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

    declare_num_agents = DeclareLaunchArgument(
        'num_agents',
        default_value='5',
        description='Number of agents to spawn (3 or 5)'
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

    # ===== Controller Configuration Paths =====
    controller_configs = {
        '0': PathJoinSubstitution([FindPackageShare('agent_control_pkg'), 'config', 'ros2', 'agent_controller_p_only.yaml']),
        '1': PathJoinSubstitution([FindPackageShare('agent_control_pkg'), 'config', 'ros2', 'agent_controller_pi.yaml']),
        '2': PathJoinSubstitution([FindPackageShare('agent_control_pkg'), 'config', 'ros2', 'agent_controller_pd.yaml']),
        '3': PathJoinSubstitution([FindPackageShare('agent_control_pkg'), 'config', 'ros2', 'agent_controller_pid.yaml']),
        '4': PathJoinSubstitution([FindPackageShare('agent_control_pkg'), 'config', 'ros2', 'agent_controller_pid_fuzzy.yaml']),
    }

    # ===== Create Agent Controller Nodes =====
    agent_nodes = []

    for i in range(5):
        agent_namespace = f'agent_{i}'

        # Agent controller node
        agent_controller = Node(
            package='agent_control_pkg',
            executable='agent_controller_node',
            name='agent_controller',
            namespace=agent_namespace,
            output='screen',
            parameters=[
                controller_configs[str(i)],
                {'use_sim_time': use_sim_time}
            ],
            remappings=[
                ('target_pose', f'/{agent_namespace}/target_pose'),
                ('odom', f'/{agent_namespace}/odom'),
                ('cmd_accel', f'/{agent_namespace}/cmd_accel'),
                ('diagnostics', f'/{agent_namespace}/diagnostics'),
            ]
        )

        # Path visualizer node for trajectory tracking
        path_visualizer = Node(
            package='agent_control_pkg',
            executable='path_visualizer_node',
            name='path_visualizer',
            namespace=agent_namespace,
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[
                ('odom', f'/{agent_namespace}/odom'),
                ('path', f'/{agent_namespace}/path'),
            ],
            condition=conditions.IfCondition(rviz_enabled)
        )

        # Metrics publisher node for performance monitoring
        metrics_publisher = Node(
            package='agent_control_pkg',
            executable='metrics_publisher_node',
            name='metrics_publisher',
            namespace=agent_namespace,
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'publish_rate_hz': 10.0},
                {'settling_threshold': 0.05},
                {'settling_time_window': 2.0},
            ],
            remappings=[
                ('odom', f'/{agent_namespace}/odom'),
                ('target_pose', f'/{agent_namespace}/target_pose'),
                ('metrics', f'/{agent_namespace}/metrics'),
            ]
        )

        agent_nodes.append(agent_controller)
        agent_nodes.append(path_visualizer)
        agent_nodes.append(metrics_publisher)

    # ===== Formation Coordinator Node =====
    formation_config = os.path.join(pkg_formation_coordinator, 'config', 'formation_config.yaml')

    formation_coordinator = Node(
        package='formation_coordinator_pkg',
        executable='formation_coordinator_node',
        name='formation_coordinator',
        output='screen',
        parameters=[
            formation_config,
            {'use_sim_time': use_sim_time},
            {
                'agent_ids': ['agent_0', 'agent_1', 'agent_2', 'agent_3', 'agent_4'],
                'formation.shape': 'triangle',
                'formation.spacing': 3.0,
                'formation.center.x': 5.0,
                'formation.center.y': 4.0,
                'formation.center.z': 0.5,
            }
        ]
    )

    # ===== RViz2 Visualization =====
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=conditions.IfCondition(rviz_enabled)
    )

    # ===== Launch Sequence =====

    # Start Gazebo immediately
    gazebo_start = [gzserver, gzclient]

    # Wait 3 seconds for Gazebo to initialize, then start ROS2 nodes
    ros2_nodes_delayed = TimerAction(
        period=3.0,
        actions=[
            *agent_nodes,
            formation_coordinator,
        ]
    )

    # Start RViz2 after controllers are running (5 seconds total)
    rviz_delayed = TimerAction(
        period=5.0,
        actions=[rviz2]
    )

    # ===== Launch Description =====

    return LaunchDescription([
        # Launch arguments
        declare_use_sim_time,
        declare_gazebo_gui,
        declare_rviz,
        declare_num_agents,

        # Start Gazebo
        *gazebo_start,

        # Start ROS2 nodes (delayed)
        ros2_nodes_delayed,

        # Start visualization (delayed)
        rviz_delayed,
    ])
