#!/usr/bin/env python3
"""
Crazyflie 9-Drone Formation Control Launch File

This launch file creates a formation comparison with 9 Crazyflie drones in 3 groups:
  - Group 0: 3x PID+IT2-FLS (cf231,232,233) - Best disturbance rejection
  - Group 1: 3x PD (cf234,235,236) - Fastest settling
  - Group 2: 3x PID (cf237,238,239) - Balanced performance

Works with:
  - Crazyswarm2 Simulation (use_sim_time:=true)
  - Real Crazyflies (use_sim_time:=false)

Usage:
    # Simulation
    ros2 launch agent_control_pkg crazyflie_9drone_formation.launch.py

    # Real hardware
    ros2 launch agent_control_pkg crazyflie_9drone_formation.launch.py use_sim_time:=false

Prerequisites:
    - Build with: colcon build --cmake-args -DENABLE_CRAZYFLIE=ON
    - For sim: ros2 launch crazyflie launch.py backend:=sim

Author: Multi-Agent Formation Control Team
Date: 2025-01
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    """Generate launch description for 9-drone Crazyflie formation."""

    pkg_share = get_package_share_directory('agent_control_pkg')

    # Configuration files
    crazyflie_config = os.path.join(pkg_share, 'config', 'crazyflie_9drone_params.yaml')

    # Launch arguments
    # Crazyswarm2 sim backend publishes /clock, so use_sim_time=true for sim
    # For real hardware, use use_sim_time=false
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    launch_bridge = LaunchConfiguration('launch_bridge', default='true')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time (true for Crazyswarm2 sim, false for real hardware)'
    )

    declare_launch_bridge = DeclareLaunchArgument(
        'launch_bridge',
        default_value='true',
        description='Launch TF->Odom bridge (disable if using alternative odometry source)'
    )

    # ===== Drone Mapping =====
    # cf231-cf239 -> agent_0-agent_8
    # Group 0 (PID+Fuzzy): cf231,232,233 -> agent_0,1,2
    # Group 1 (PD):        cf234,235,236 -> agent_3,4,5
    # Group 2 (PID):       cf237,238,239 -> agent_6,7,8

    drone_mapping = [
        # (cf_id, agent_id, controller_type, y_lane)
        (231, 0, 'pid_fuzzy', -4.0),
        (232, 1, 'pid_fuzzy', -4.0),
        (233, 2, 'pid_fuzzy', -4.0),
        (234, 3, 'pd', 0.0),
        (235, 4, 'pd', 0.0),
        (236, 5, 'pd', 0.0),
        (237, 6, 'pid', 4.0),
        (238, 7, 'pid', 4.0),
        (239, 8, 'pid', 4.0),
    ]

    # ===== TF to Odom Bridge =====
    tf_odom_bridge = Node(
        package='agent_control_pkg',
        executable='tf_to_odom_bridge.py',
        name='tf_to_odom_bridge',
        output='screen',
        parameters=[{
            'num_drones': 9,
            'base_cf_id': 231,
            'publish_rate': 100.0,
            'world_frame': 'world',
            'use_sim_time': use_sim_time,  # Must match controller setting!
        }],
        condition=IfCondition(launch_bridge)
    )

    # ===== Controller Nodes =====
    controller_nodes = []

    for cf_id, agent_id, controller_type, y_lane in drone_mapping:
        cf_name = f'cf{cf_id}'
        agent_ns = f'agent_{agent_id}'

        # Namespace config from YAML + runtime overrides
        controller_params = {
            'use_sim_time': use_sim_time,
            'controller_type': controller_type,
            'dt': 0.005,
            'control_frequency_hz': 200.0,
            # Crazyflie output enabled
            'crazyflie.enable': True,
            'crazyflie.cmd_topic': 'cmd_full_state',
            'crazyflie.mass': 0.027,
            'crazyflie.max_thrust': 0.6,
            'crazyflie.max_roll_rad': 0.5236,
            'crazyflie.max_pitch_rad': 0.5236,
        }

        # Add controller-specific parameters
        if controller_type == 'pid_fuzzy':
            controller_params.update({
                'pid.kp': 3.501,
                'pid.ki': 1.946,
                'pid.kd': 3.608,
                'fuzzy.enable': True,
                'fuzzy.include_wind': True,
                'fuzzy.wind_scalar': 1.5,
                'fuzzy.params_file': 'fuzzy_params_crazyflie.yaml',
                'mix.k_pid': 1.0,
                'mix.k_fuzzy': 0.6,
            })
        elif controller_type == 'pd':
            controller_params.update({
                'pid.kp': 3.50,
                'pid.ki': 0.0,
                'pid.kd': 3.61,
                'pid.enable_derivative_filter': True,
                'fuzzy.enable': False,
            })
        else:  # pid
            controller_params.update({
                'pid.kp': 3.501,
                'pid.ki': 1.946,
                'pid.kd': 3.608,
                'pid.enable_derivative_filter': True,
                'pid.derivative_filter_alpha': 0.1,
                'fuzzy.enable': False,
            })

        # Output limits
        controller_params.update({
            'output_limits.x.min': -10.0,
            'output_limits.x.max': 10.0,
            'output_limits.y.min': -10.0,
            'output_limits.y.max': 10.0,
        })

        # Controller node with remappings for Crazyswarm2 topics
        controller = Node(
            package='agent_control_pkg',
            executable='agent_controller_node',
            name='agent_controller',
            namespace=agent_ns,
            output='screen',
            parameters=[controller_params],
            remappings=[
                # Crazyswarm2 topic mappings
                ('odom', f'/{cf_name}/odom'),
                ('target_pose', f'/{agent_ns}/target_pose'),  # Keep agent namespace for targets
                ('cmd_full_state', f'/{cf_name}/cmd_full_state'),
                ('cmd_accel', f'/{agent_ns}/cmd_accel'),  # For monitoring
            ]
        )

        # Metrics publisher for this agent
        metrics = Node(
            package='agent_control_pkg',
            executable='metrics_publisher_node',
            name='metrics_publisher',
            namespace=agent_ns,
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'publish_rate_hz': 10.0,
                'settled_pos_threshold': 0.10,
                'settled_time_window': 1.0,
                'metrics_window_sec': 30.0,
                'enable_group_metrics': True,
            }],
            remappings=[
                ('odom', f'/{cf_name}/odom'),
            ]
        )

        controller_nodes.append(controller)
        controller_nodes.append(metrics)

    # ===== Formation Coordinators =====
    pkg_formation = get_package_share_directory('formation_coordinator_pkg')

    formation_configs = [
        ('formation_0', 'formation_group0_fuzzy.yaml'),
        ('formation_1', 'formation_group1_pd.yaml'),
        ('formation_2', 'formation_group2_pid.yaml'),
    ]

    formation_nodes = []
    for ns, config_file in formation_configs:
        config_path = os.path.join(pkg_formation, 'config', config_file)
        # Extract group number (formation_0 -> group0)
        group_num = ns.split('_')[1]
        node_name = f'formation_coordinator_group{group_num}'

        coordinator = GroupAction([
            PushRosNamespace(ns),
            Node(
                package='formation_coordinator_pkg',
                executable='formation_coordinator_node',
                name=node_name,
                output='screen',
                parameters=[
                    config_path,
                    {'use_sim_time': use_sim_time}
                ]
            )
        ])
        formation_nodes.append(coordinator)

    # ===== Launch Description =====
    return LaunchDescription([
        # Arguments
        declare_use_sim_time,
        declare_launch_bridge,

        # TF to Odom bridge
        tf_odom_bridge,

        # Controller nodes (9 agents)
        *controller_nodes,

        # Formation coordinators (3 groups)
        *formation_nodes,
    ])
