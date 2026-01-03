#!/usr/bin/env python3
"""
12-Drone 4-Group Controller Comparison Launch File

Compares PD vs PID vs IT2-FLS vs GT2-FLS with 12 drones (3 per group):
  - Group 0: 3x PD controllers (agent_0,1,2)
  - Group 1: 3x PID controllers (agent_3,4,5)
  - Group 2: 3x PID+IT2-Fuzzy controllers (agent_6,7,8)
  - Group 3: 3x PID+GT2-Fuzzy controllers (agent_9,10,11)

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
    """Generate launch description for 12-drone 4-group comparison"""

    # Package directories
    pkg_agent_control = get_package_share_directory('agent_control_pkg')
    pkg_formation_coordinator = get_package_share_directory('formation_coordinator_pkg')

    # World file (4-group 12-drone)
    world_file = PathJoinSubstitution([
        FindPackageShare('agent_control_pkg'),
        'worlds',
        'crazyflie_12drone_4group.world'
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

    # Wind publisher - von Karman turbulence for realistic atmospheric wind
    wind_publisher = Node(
        package='agent_control_pkg',
        executable='wind_publisher.py',
        name='wind_publisher',
        output='screen',
        parameters=[{
            'profile': 'vonkarman',  # Scientifically-grounded turbulence model
            'magnitude': 2.5,         # Mean wind speed [m/s]
            'direction': 45.0,        # Wind direction [deg] - diagonal for X+Y disturbance
            'turbulence_intensity': 0.20,  # 20% TI - moderate turbulence
            'mean_wind_speed': 2.5,   # For turbulence scaling
            'integral_length_u': 30.0,  # Longitudinal scale [m]
            'integral_length_v': 15.0,  # Lateral scale [m]
            'integral_length_w': 5.0,   # Vertical scale [m]
            'publish_rate': 20.0,     # Higher rate for smoother turbulence
        }]
    )

    # ===== Controller Configurations =====

    # Common PID gains (tuned for Crazyflie)
    common_pid = {
        'pid.kp': 3.501,
        'pid.ki': 1.946,
        'pid.kd': 3.608,
    }

    # PD controller params (Group 0: agents 0-2) - No integral term
    pd_controller_params = {
        'controller_type': 'pd',
        'dt': 0.005,
        'pid.kp': 3.501,
        'pid.ki': 0.0,  # No integral for PD
        'pid.kd': 3.608,
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

    # PID controller params (Group 1: agents 3-5) - Full PID
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

    # IT2 Fuzzy controller params (Group 2: agents 6-8)
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

    # GT2 Fuzzy controller params (Group 3: agents 9-11)
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

    # Agent controller map: agent_id -> params
    agent_controller_map = {
        # Group 0: PD (agents 0-2)
        0: pd_controller_params,
        1: pd_controller_params,
        2: pd_controller_params,
        # Group 1: PID (agents 3-5)
        3: pid_controller_params,
        4: pid_controller_params,
        5: pid_controller_params,
        # Group 2: IT2 Fuzzy (agents 6-8)
        6: it2_controller_params,
        7: it2_controller_params,
        8: it2_controller_params,
        # Group 3: GT2 Fuzzy (agents 9-11)
        9: gt2_controller_params,
        10: gt2_controller_params,
        11: gt2_controller_params,
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

    # Formation Coordinators for each group (4-group configs)
    formation_config_pd = os.path.join(pkg_formation_coordinator, 'config', 'formation_12drone_pd.yaml')
    formation_config_pid = os.path.join(pkg_formation_coordinator, 'config', 'formation_12drone_pid_group.yaml')
    formation_config_it2 = os.path.join(pkg_formation_coordinator, 'config', 'formation_12drone_it2_group.yaml')
    formation_config_gt2 = os.path.join(pkg_formation_coordinator, 'config', 'formation_12drone_gt2_group.yaml')

    # PD Group formation (agents 0-2)
    formation_coordinator_pd = GroupAction([
        PushRosNamespace('formation_0'),
        Node(
            package='formation_coordinator_pkg',
            executable='formation_coordinator_node',
            name='formation_coordinator_pd',
            output='screen',
            arguments=['--ros-args', '-r', '__node:=formation_coordinator_pd', '-r', '__ns:=/formation_0'],
            parameters=[formation_config_pd, {'use_sim_time': use_sim_time}]
        )
    ])

    # PID Group formation (agents 3-5)
    formation_coordinator_pid = GroupAction([
        PushRosNamespace('formation_1'),
        Node(
            package='formation_coordinator_pkg',
            executable='formation_coordinator_node',
            name='formation_coordinator_pid',
            output='screen',
            arguments=['--ros-args', '-r', '__node:=formation_coordinator_pid', '-r', '__ns:=/formation_1'],
            parameters=[formation_config_pid, {'use_sim_time': use_sim_time}]
        )
    ])

    # IT2 Group formation (agents 6-8)
    formation_coordinator_it2 = GroupAction([
        PushRosNamespace('formation_2'),
        Node(
            package='formation_coordinator_pkg',
            executable='formation_coordinator_node',
            name='formation_coordinator_it2',
            output='screen',
            arguments=['--ros-args', '-r', '__node:=formation_coordinator_it2', '-r', '__ns:=/formation_2'],
            parameters=[formation_config_it2, {'use_sim_time': use_sim_time}]
        )
    ])

    # GT2 Group formation (agents 9-11)
    formation_coordinator_gt2 = GroupAction([
        PushRosNamespace('formation_3'),
        Node(
            package='formation_coordinator_pkg',
            executable='formation_coordinator_node',
            name='formation_coordinator_gt2',
            output='screen',
            arguments=['--ros-args', '-r', '__node:=formation_coordinator_gt2', '-r', '__ns:=/formation_3'],
            parameters=[formation_config_gt2, {'use_sim_time': use_sim_time}]
        )
    ])

    return LaunchDescription([
        declare_use_sim_time,
        declare_gazebo_gui,
        gzserver,
        gzclient,
        wind_publisher,
        *agent_nodes,
        formation_coordinator_pd,
        formation_coordinator_pid,
        formation_coordinator_it2,
        formation_coordinator_gt2,
    ])
