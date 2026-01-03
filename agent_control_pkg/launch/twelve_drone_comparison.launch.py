#!/usr/bin/env python3
"""
12-Drone Controller Comparison Launch File

Compares IT2-FLS vs GT2-FLS vs Pure PID with 12 drones:
  - Group 0: 4x PID+IT2-Fuzzy controllers (agent_0,1,2,3)
  - Group 1: 4x PID+GT2-Fuzzy controllers (agent_4,5,6,7)
  - Group 2: 4x Pure PID controllers (agent_8,9,10,11)

Usage:
    # Headless mode (maximum FPS)
    ros2 launch agent_control_pkg twelve_drone_comparison.launch.py

    # With GUI (for debugging)
    ros2 launch agent_control_pkg twelve_drone_comparison.launch.py gazebo_gui:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch import conditions
from launch.actions import ExecuteProcess, DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    """Generate launch description for 12-drone comparison"""

    # Package directories
    pkg_agent_control = get_package_share_directory('agent_control_pkg')
    pkg_formation_coordinator = get_package_share_directory('formation_coordinator_pkg')

    # World file
    world_file = PathJoinSubstitution([
        FindPackageShare('agent_control_pkg'),
        'worlds',
        'crazyflie_12drone_comparison.world'
    ])

    # Gazebo paths
    model_path = os.path.join(pkg_agent_control, 'models')
    pkg_root = os.path.dirname(os.path.dirname(pkg_agent_control))
    plugin_path_pkg = os.path.join(pkg_root, 'lib')
    plugin_path_std = os.path.join(os.path.dirname(pkg_root), 'lib')

    merged_model_path = ":".join([p for p in [model_path, os.environ.get('GAZEBO_MODEL_PATH', '')] if p])
    merged_plugin_path = ":".join([p for p in [plugin_path_pkg, plugin_path_std, os.environ.get('GAZEBO_PLUGIN_PATH', '')] if p])
    os.environ['GAZEBO_MODEL_PATH'] = merged_model_path
    os.environ['GAZEBO_PLUGIN_PATH'] = merged_plugin_path

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gazebo_gui = LaunchConfiguration('gazebo_gui', default='false')  # Default headless

    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_gazebo_gui = DeclareLaunchArgument('gazebo_gui', default_value='false')

    # Gazebo server
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

    # Gazebo client (optional)
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        condition=conditions.IfCondition(gazebo_gui)
    )

    # Wind publisher
    wind_publisher = Node(
        package='agent_control_pkg',
        executable='wind_publisher.py',
        name='wind_publisher',
        output='screen',
        parameters=[{
            'profile': 'gust',
            'magnitude': 3.5,
            'direction': 90.0,
            'gust_duration': 1.5,
            'gust_interval': 8.0,
        }]
    )

    # ===== Controller Configurations =====

    # Common PID gains (tuned for Crazyflie)
    common_pid = {
        'pid.kp': 3.501,
        'pid.ki': 1.946,
        'pid.kd': 3.608,
    }

    # IT2 Fuzzy controller params (Group 0: agents 0-3)
    it2_controller_params = {
        'controller_type': 'pid_fuzzy',
        'dt': 0.005,
        **common_pid,
        'fuzzy.enable': True,
        'fuzzy.include_wind': True,
        'fuzzy.wind_scalar': 1.0,
        'fuzzy.params_file': 'fuzzy_params_crazyflie.yaml',
        'mix.k_pid': 1.0,
        'mix.k_fuzzy': 0.35,
        'wind_source_topic': '/wind/velocity',
        'wind_source_type': 'velocity',
        'feedforward.enable_wind': False,
        'feedforward.k_wind': 0.0,
        'use_sim_time': use_sim_time,
        'output_limits.x.min': -10.0,
        'output_limits.x.max': 10.0,
        'output_limits.y.min': -10.0,
        'output_limits.y.max': 10.0,
    }

    # GT2 Fuzzy controller params (Group 1: agents 4-7)
    gt2_controller_params = {
        'controller_type': 'pid_gt2_fuzzy',
        'dt': 0.005,
        **common_pid,
        'fuzzy.enable': True,
        'fuzzy.include_wind': True,
        'fuzzy.wind_scalar': 1.0,
        'mix.k_pid': 1.0,
        'mix.k_fuzzy': 0.35,
        'gt2.num_alpha_levels': 5,
        'gt2.secondary_shape': 'triangular',
        'gt2.secondary_spread': 0.3,
        'gt2.params_file': 'gt2_fuzzy_params_crazyflie.yaml',
        'wind_source_topic': '/wind/velocity',
        'wind_source_type': 'velocity',
        'feedforward.enable_wind': False,
        'feedforward.k_wind': 0.0,
        'use_sim_time': use_sim_time,
        'output_limits.x.min': -10.0,
        'output_limits.x.max': 10.0,
        'output_limits.y.min': -10.0,
        'output_limits.y.max': 10.0,
    }

    # Pure PID controller params (Group 2: agents 8-11)
    pid_controller_params = {
        'controller_type': 'pid',
        'dt': 0.005,
        **common_pid,
        'fuzzy.enable': False,
        'wind_source_topic': '/wind/velocity',
        'wind_source_type': 'velocity',
        'feedforward.enable_wind': False,
        'feedforward.k_wind': 0.0,
        'use_sim_time': use_sim_time,
        'output_limits.x.min': -10.0,
        'output_limits.x.max': 10.0,
        'output_limits.y.min': -10.0,
        'output_limits.y.max': 10.0,
    }

    # Agent controller map: agent_id -> params
    agent_controller_map = {
        # Group 0: IT2 Fuzzy (agents 0-3)
        0: it2_controller_params,
        1: it2_controller_params,
        2: it2_controller_params,
        3: it2_controller_params,
        # Group 1: GT2 Fuzzy (agents 4-7)
        4: gt2_controller_params,
        5: gt2_controller_params,
        6: gt2_controller_params,
        7: gt2_controller_params,
        # Group 2: Pure PID (agents 8-11)
        8: pid_controller_params,
        9: pid_controller_params,
        10: pid_controller_params,
        11: pid_controller_params,
    }

    # Create agent nodes
    agent_nodes = []
    for i in range(12):
        agent_namespace = f'agent_{i}'
        controller_params = agent_controller_map[i].copy()

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
                executable='metrics_publisher_node',
                name='metrics_publisher',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'publish_rate_hz': 10.0,
                    'settled_pos_threshold': 0.10,
                    'settled_time_window': 1.0,
                    'metrics_window_sec': 30.0,
                    'enable_group_metrics': True,
                }]
            ),
        ])
        agent_nodes.append(agent_controller)

    # Formation Coordinators for each group (12-drone configs)
    formation_config_it2 = os.path.join(pkg_formation_coordinator, 'config', 'formation_12drone_it2.yaml')
    formation_config_gt2 = os.path.join(pkg_formation_coordinator, 'config', 'formation_12drone_gt2.yaml')
    formation_config_pid = os.path.join(pkg_formation_coordinator, 'config', 'formation_12drone_pid.yaml')

    # IT2 Group formation (agents 0-3)
    formation_coordinator_it2 = GroupAction([
        PushRosNamespace('formation_0'),
        Node(
            package='formation_coordinator_pkg',
            executable='formation_coordinator_node',
            name='formation_coordinator_it2',
            output='screen',
            arguments=['--ros-args', '-r', '__node:=formation_coordinator_it2', '-r', '__ns:=/formation_0'],
            parameters=[formation_config_it2, {'use_sim_time': use_sim_time}]
        )
    ])

    # GT2 Group formation (agents 4-7)
    formation_coordinator_gt2 = GroupAction([
        PushRosNamespace('formation_1'),
        Node(
            package='formation_coordinator_pkg',
            executable='formation_coordinator_node',
            name='formation_coordinator_gt2',
            output='screen',
            arguments=['--ros-args', '-r', '__node:=formation_coordinator_gt2', '-r', '__ns:=/formation_1'],
            parameters=[formation_config_gt2, {'use_sim_time': use_sim_time}]
        )
    ])

    # PID Group formation (agents 8-11)
    formation_coordinator_pid = GroupAction([
        PushRosNamespace('formation_2'),
        Node(
            package='formation_coordinator_pkg',
            executable='formation_coordinator_node',
            name='formation_coordinator_pid',
            output='screen',
            arguments=['--ros-args', '-r', '__node:=formation_coordinator_pid', '-r', '__ns:=/formation_2'],
            parameters=[formation_config_pid, {'use_sim_time': use_sim_time}]
        )
    ])

    return LaunchDescription([
        declare_use_sim_time,
        declare_gazebo_gui,
        gzserver,
        gzclient,
        wind_publisher,
        *agent_nodes,
        formation_coordinator_it2,
        formation_coordinator_gt2,
        formation_coordinator_pid,
    ])
