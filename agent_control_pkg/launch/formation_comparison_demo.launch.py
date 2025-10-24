#!/usr/bin/env python3
"""
Formation Comparison Demo - Launch File

This launch file creates a comparison demonstration with 3 formation groups:
  - Group 0: 3x PID+Fuzzy controllers (agent_0,1,2) - Best wind handling expected
  - Group 1: 3x PD controllers (agent_3,4,5) - Fastest settling expected
  - Group 2: 3x PID controllers (agent_6,7,8) - Balanced performance expected

Each group maintains triangle formation while moving from (-10, Y, 0) to (5, Y, 0)
Wind disturbance is active to test controller robustness.

Formation Targets:
  - Group 0 (PID+Fuzzy): (5, -4, 0) - Bottom lane
  - Group 1 (PD):        (5,  0, 0) - Middle lane
  - Group 2 (PID):       (5,  4, 0) - Top lane

Usage:
    # Full demo with visualizations
    ros2 launch agent_control_pkg formation_comparison_demo.launch.py

    # Headless mode (maximum FPS)
    ros2 launch agent_control_pkg formation_comparison_demo.launch.py gazebo_gui:=false rviz:=false

Author: Multi-Agent Formation Control Research Team
Date: 2025-10-17
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch import conditions
from launch.actions import ExecuteProcess, DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    """Generate launch description for formation comparison demonstration"""

    # Package directories
    pkg_agent_control = get_package_share_directory('agent_control_pkg')
    pkg_formation_coordinator = get_package_share_directory('formation_coordinator_pkg')

    # World file - 9 drones in 3 groups
    world_file = PathJoinSubstitution([
        FindPackageShare('agent_control_pkg'),
        'worlds',
        'formation_comparison.world'
    ])

    # RViz config (reuse from existing demo)
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
        description='Launch Gazebo GUI'
    )

    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz for visualization'
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

    # ===== Controller Configurations =====
    # Define controller parameters for each group

    # GROUP 0: PID+Fuzzy (agent_0, agent_1, agent_2)
    # Updated with optimal parameters from C++ run_088
    fuzzy_controller_params = {
        'controller_type': 'pid_fuzzy',
        'dt': 0.005,
        'pid.kp': 3.501,    # UPDATED from velocity tuning
        'pid.ki': 1.946,    # UPDATED for wind rejection
        'pid.kd': 3.608,    # UPDATED for damping
        'fuzzy.enable': True,
        'fuzzy.include_wind': True,
        'fuzzy.wind_scalar': 1.0,  # UPDATED: full wind sensitivity
        'fuzzy.params_file': 'fuzzy_params.yaml',
        'mix.k_pid': 1.0,
        'mix.k_fuzzy': 0.7,  # Strong fuzzy influence for disturbance rejection
        'use_sim_time': use_sim_time,
        'output_limits.x.min': -10.0,
        'output_limits.x.max': 10.0,
        'output_limits.y.min': -10.0,
        'output_limits.y.max': 10.0,
    }

    # GROUP 1: PD (agent_3, agent_4, agent_5)
    pd_controller_params = {
        'controller_type': 'pd',
        'dt': 0.005,
        'pid.kp': 3.50,
        'pid.ki': 0.0,
        'pid.kd': 3.61,
        'pid.enable_derivative_filter': True,
        'fuzzy.enable': False,
        'use_sim_time': use_sim_time,
        'output_limits.x.min': -10.0,
        'output_limits.x.max': 10.0,
        'output_limits.y.min': -10.0,
        'output_limits.y.max': 10.0,
    }

    # GROUP 2: PID (agent_6, agent_7, agent_8)
    # Updated with optimal parameters from C++ run_088
    pid_controller_params = {
        'controller_type': 'pid',
        'dt': 0.005,
        'pid.kp': 3.501,    # UPDATED from velocity tuning
        'pid.ki': 1.946,    # UPDATED for wind rejection
        'pid.kd': 3.608,    # UPDATED for damping
        'pid.enable_derivative_filter': True,
        'pid.derivative_filter_alpha': 0.1,
        'fuzzy.enable': False,
        'use_sim_time': use_sim_time,
        'output_limits.x.min': -10.0,
        'output_limits.x.max': 10.0,
        'output_limits.y.min': -10.0,
        'output_limits.y.max': 10.0,
    }

    # Map agent IDs to controller params
    agent_controller_map = {
        0: fuzzy_controller_params,
        1: fuzzy_controller_params,
        2: fuzzy_controller_params,
        3: pd_controller_params,
        4: pd_controller_params,
        5: pd_controller_params,
        6: pid_controller_params,
        7: pid_controller_params,
        8: pid_controller_params,
    }

    # ===== Create Agent Controller Nodes =====
    agent_nodes = []

    for i in range(9):
        agent_namespace = f'agent_{i}'

        # Get controller params for this agent
        controller_params = agent_controller_map[i].copy()

        # Agent controller node
        agent_controller = GroupAction([
            PushRosNamespace(agent_namespace),
            Node(
                package='agent_control_pkg',
                executable='agent_controller_node',
                name='agent_controller',
                output='screen',
                parameters=[controller_params]
            ),
            Node(
                package='agent_control_pkg',
                executable='path_visualizer_node',
                name='path_visualizer',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'max_path_length': 1000,
                    'publish_rate_hz': 10.0,
                }]
            ),
            Node(
                package='agent_control_pkg',
                executable='metrics_publisher_node',
                name='metrics_publisher',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'publish_rate_hz': 10.0,
                    'settling_threshold': 0.05,  # 5cm
                    'settling_time_window': 2.0,
                }]
            ),
        ])

        agent_nodes.append(agent_controller)

    # ===== Formation Coordinator Nodes =====
    # 3 separate formation coordinators, one per group

    # Formation config files
    formation_config_group0 = os.path.join(
        pkg_formation_coordinator,
        'config',
        'formation_group0_fuzzy.yaml'
    )
    formation_config_group1 = os.path.join(
        pkg_formation_coordinator,
        'config',
        'formation_group1_pd.yaml'
    )
    formation_config_group2 = os.path.join(
        pkg_formation_coordinator,
        'config',
        'formation_group2_pid.yaml'
    )

    # Group 0 Formation Coordinator (PID+Fuzzy)
    formation_coordinator_group0 = Node(
        package='formation_coordinator_pkg',
        executable='formation_coordinator_node',
        name='formation_coordinator_group0',
        namespace='formation_0',
        output='screen',
        parameters=[
            formation_config_group0,
            {'use_sim_time': use_sim_time}
        ]
    )

    # Group 1 Formation Coordinator (PD)
    formation_coordinator_group1 = Node(
        package='formation_coordinator_pkg',
        executable='formation_coordinator_node',
        name='formation_coordinator_group1',
        namespace='formation_1',
        output='screen',
        parameters=[
            formation_config_group1,
            {'use_sim_time': use_sim_time}
        ]
    )

    # Group 2 Formation Coordinator (PID)
    formation_coordinator_group2 = Node(
        package='formation_coordinator_pkg',
        executable='formation_coordinator_node',
        name='formation_coordinator_group2',
        namespace='formation_2',
        output='screen',
        parameters=[
            formation_config_group2,
            {'use_sim_time': use_sim_time}
        ]
    )

    # ===== RViz Visualization =====
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=conditions.IfCondition(rviz_enabled),
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ===== Metrics Summary Node (Optional) =====
    # This node will aggregate metrics from all agents and print summary on Ctrl+C
    metrics_summary = Node(
        package='agent_control_pkg',
        executable='metrics_summary.py',
        name='metrics_summary',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ===== Launch Description =====
    return LaunchDescription([
        # Arguments
        declare_use_sim_time,
        declare_gazebo_gui,
        declare_rviz,

        # Gazebo
        gzserver,
        gzclient,

        # Agent controllers (9 agents)
        *agent_nodes,

        # Formation coordinators (3 groups)
        formation_coordinator_group0,
        formation_coordinator_group1,
        formation_coordinator_group2,

        # Visualization
        rviz_node,

        # Metrics summary (optional - comment out if not needed)
        # metrics_summary,
    ])
