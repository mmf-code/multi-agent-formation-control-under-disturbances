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
    # Full demo with realistic physics (default - sim_cf2 calibrated)
    ros2 launch agent_control_pkg formation_comparison_demo.launch.py

    # With simple physics (original model)
    ros2 launch agent_control_pkg formation_comparison_demo.launch.py physics_model:=simple world_file:=crazyflie_formation.world

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

    # Physics model selection: 'realistic' (sim_cf2 calibrated) or 'simple' (original)
    physics_model = LaunchConfiguration('physics_model', default='realistic')

    # World file - auto-select based on physics_model, or override with world_file:=<name>
    # realistic -> crazyflie_arena.world (new professional arena with realistic physics)
    # simple -> crazyflie_formation.world (original simple physics)
    world_file_arg = LaunchConfiguration('world_file', default='crazyflie_arena.world')
    world_file = PathJoinSubstitution([
        FindPackageShare('agent_control_pkg'),
        'worlds',
        world_file_arg
    ])

    # RViz config (reuse from existing demo)
    rviz_config = os.path.join(pkg_agent_control, 'rviz', 'formation_demo.rviz')

    # Gazebo paths
    model_path = os.path.join(pkg_agent_control, 'models')
    # Resolve both common install layouts for the plugin:
    #  - install/agent_control_pkg/lib (observed in this workspace via symlink)
    #  - install/lib (standard ament install path)
    pkg_root = os.path.dirname(os.path.dirname(pkg_agent_control))            # .../install/agent_control_pkg
    plugin_path_pkg = os.path.join(pkg_root, 'lib')                           # .../install/agent_control_pkg/lib
    plugin_path_std = os.path.join(os.path.dirname(pkg_root), 'lib')          # .../install/lib

    # Set environment variables for Gazebo
    merged_model_path = ":".join([p for p in [model_path, os.environ.get('GAZEBO_MODEL_PATH', '')] if p])
    merged_plugin_path = ":".join([p for p in [plugin_path_pkg, plugin_path_std, os.environ.get('GAZEBO_PLUGIN_PATH', '')] if p])
    os.environ['GAZEBO_MODEL_PATH'] = merged_model_path
    os.environ['GAZEBO_PLUGIN_PATH'] = merged_plugin_path

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gazebo_gui = LaunchConfiguration('gazebo_gui', default='true')
    rviz_enabled = LaunchConfiguration('rviz', default='true')

    # Controller tuning arguments (PID vs PID+Fuzzy)
    # NOTE: Defaults preserve previous behaviour.
    pid_group_kp = LaunchConfiguration('pid_group_kp')
    pid_group_ki = LaunchConfiguration('pid_group_ki')
    pid_group_kd = LaunchConfiguration('pid_group_kd')
    fuzzy_group_kp = LaunchConfiguration('fuzzy_group_kp')
    fuzzy_group_ki = LaunchConfiguration('fuzzy_group_ki')
    fuzzy_group_kd = LaunchConfiguration('fuzzy_group_kd')
    fuzzy_group_k_fuzzy = LaunchConfiguration('fuzzy_group_k_fuzzy')

    # Wind + metrics arguments
    wind_source_topic = LaunchConfiguration('wind_source_topic')
    wind_source_type = LaunchConfiguration('wind_source_type')
    settled_pos_threshold = LaunchConfiguration('settled_pos_threshold')
    settled_time_window = LaunchConfiguration('settled_time_window')
    metrics_window_sec = LaunchConfiguration('metrics_window_sec')

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

    declare_physics_model = DeclareLaunchArgument(
        'physics_model',
        default_value='realistic',
        description='Physics model: realistic (sim_cf2 calibrated motors) or simple (original)'
    )

    declare_world_file = DeclareLaunchArgument(
        'world_file',
        default_value='crazyflie_arena.world',
        description='Gazebo world file: crazyflie_arena.world (realistic) or crazyflie_formation.world (simple)'
    )

    # Wind profile argument - allows switching between deterministic and stochastic wind
    wind_profile = LaunchConfiguration('wind_profile', default='gust')
    declare_wind_profile = DeclareLaunchArgument(
        'wind_profile',
        default_value='gust',
        description='Wind profile: gust (deterministic), stochastic (random/uncertain)'
    )

    # Controller tuning launch arguments
    declare_pid_group_kp = DeclareLaunchArgument(
        'pid_group_kp',
        default_value='3.501',
        description='Kp for pure PID group (Group 2: agents 6–8).'
    )
    declare_pid_group_ki = DeclareLaunchArgument(
        'pid_group_ki',
        default_value='1.946',
        description='Ki for pure PID group (Group 2: agents 6–8).'
    )
    declare_pid_group_kd = DeclareLaunchArgument(
        'pid_group_kd',
        default_value='3.608',
        description='Kd for pure PID group (Group 2: agents 6–8).'
    )

    declare_fuzzy_group_kp = DeclareLaunchArgument(
        'fuzzy_group_kp',
        default_value='3.501',
        description='Kp for PID+Fuzzy group (Group 0: agents 0–2).'
    )
    declare_fuzzy_group_ki = DeclareLaunchArgument(
        'fuzzy_group_ki',
        default_value='1.946',
        description='Ki for PID+Fuzzy group (Group 0: agents 0–2).'
    )
    declare_fuzzy_group_kd = DeclareLaunchArgument(
        'fuzzy_group_kd',
        default_value='3.608',
        description='Kd for PID+Fuzzy group (Group 0: agents 0–2).'
    )
    declare_fuzzy_group_k_fuzzy = DeclareLaunchArgument(
        'fuzzy_group_k_fuzzy',
        default_value='0.6',
        description='Mixing gain k_fuzzy for PID+Fuzzy group (Group 0). Tuned for Crazyflie scaled MFs.'
    )

    # Wind + metrics launch arguments
    declare_wind_source_topic = DeclareLaunchArgument(
        'wind_source_topic',
        default_value='/wind/velocity',
        description='Topic supplying wind data to agent controllers.'
    )
    declare_wind_source_type = DeclareLaunchArgument(
        'wind_source_type',
        default_value='velocity',
        description='Wind data interpretation: "velocity" or "force".'
    )
    declare_settled_pos_threshold = DeclareLaunchArgument(
        'settled_pos_threshold',
        default_value='0.10',
        description='Position error threshold (m) for settled detection.'
    )
    declare_settled_time_window = DeclareLaunchArgument(
        'settled_time_window',
        default_value='1.0',
        description='Time window (s) for settled detection.'
    )
    declare_metrics_window_sec = DeclareLaunchArgument(
        'metrics_window_sec',
        default_value='30.0',
        description='Window length (s) for group-level RMSE/IAE metrics.'
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

    # Wind publisher node (provides /wind/velocity for disturbance testing)
    # THESIS CONFIG: Aggressive wind to differentiate controller performance
    # Real-world outdoor conditions for nano quadrotors: 2-5 m/s gusts
    # Use wind_profile:=stochastic for uncertainty/robustness testing
    wind_publisher = Node(
        package='agent_control_pkg',
        executable='wind_publisher.py',
        name='wind_publisher',
        output='screen',
        parameters=[{
            'profile': wind_profile,  # gust (deterministic) or stochastic (random)
            'magnitude': 3.5,         # m/s - base wind magnitude
            'direction': 90.0,        # degrees - CROSS-WIND (perpendicular to +X motion)
            'gust_duration': 1.5,     # seconds - for gust profile
            'gust_interval': 8.0,     # seconds - for gust profile
            # Stochastic profile params (active when profile:=stochastic)
            'stochastic_mag_std': 1.5,       # magnitude std dev (m/s)
            'stochastic_dir_rate': 15.0,     # direction change rate (deg/s)
            'stochastic_gust_prob': 0.15,    # random gust probability per second
            'stochastic_gust_mag': 2.5,      # max gust multiplier
            'stochastic_turbulence': 0.5,    # high-freq turbulence intensity
        }]
    )

    # ===== Controller Configurations =====
    # Define controller parameters for each group

    # GROUP 0: PID+Fuzzy (agent_0, agent_1, agent_2)
    # Updated with optimal parameters from C++ run_088
    fuzzy_controller_params = {
        'controller_type': 'pid_fuzzy',
        'dt': 0.005,
        # PID baseline for hybrid controller
        'pid.kp': fuzzy_group_kp,
        'pid.ki': fuzzy_group_ki,
        'pid.kd': fuzzy_group_kd,
        'fuzzy.enable': True,
        'fuzzy.include_wind': True,
        # Wind scalar for fuzzy controller. Reverted to 1.5 (was 1.0)
        'fuzzy.wind_scalar': 1.5,
        'fuzzy.params_file': 'fuzzy_params_crazyflie.yaml',  # Crazyflie-scaled MFs
        'mix.k_pid': 1.0,
        'mix.k_fuzzy': fuzzy_group_k_fuzzy,  # Fuzzy mix factor
        # Wind handling (can be overridden per launch)
        'wind_source_topic': wind_source_topic,
        'wind_source_type': wind_source_type,
        'feedforward.enable_wind': False, # Reverted to False
        'feedforward.k_wind': 0.0,        # Reverted to 0.0
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
        'wind_source_topic': wind_source_topic,
        'wind_source_type': wind_source_type,
        'feedforward.enable_wind': False, # Reverted to False
        'feedforward.k_wind': 0.0,        # Reverted to 0.0
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
        # Pure PID group parameters (no fuzzy)
        'pid.kp': pid_group_kp,
        'pid.ki': pid_group_ki,
        'pid.kd': pid_group_kd,
        'pid.enable_derivative_filter': True,
        'pid.derivative_filter_alpha': 0.1,
        'fuzzy.enable': False,
        'mix.k_pid': 1.0,
        'mix.k_fuzzy': 0.0,
        'wind_source_topic': wind_source_topic,
        'wind_source_type': wind_source_type,
        'feedforward.enable_wind': False, # Reverted to False
        'feedforward.k_wind': 0.0,        # Reverted to 0.0
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
                    # Looser, configurable settled detection for dashboard
                    'settled_pos_threshold': settled_pos_threshold,
                    'settled_time_window': settled_time_window,
                    # Group-level comparison window (RMSE / IAE)
                    'metrics_window_sec': metrics_window_sec,
                    'enable_group_metrics': True,
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
    formation_coordinator_group0 = GroupAction([
        PushRosNamespace('formation_0'),
        Node(
            package='formation_coordinator_pkg',
            executable='formation_coordinator_node',
            name='formation_coordinator_group0',
            output='screen',
            # Force node name and namespace using explicit arguments
            arguments=['--ros-args', '-r', '__node:=formation_coordinator_group0', '-r', '__ns:=/formation_0'],
            parameters=[
                formation_config_group0,
                {'use_sim_time': use_sim_time}
            ]
        )
    ])

    # Group 1 Formation Coordinator (PD)
    formation_coordinator_group1 = GroupAction([
        PushRosNamespace('formation_1'),
        Node(
            package='formation_coordinator_pkg',
            executable='formation_coordinator_node',
            name='formation_coordinator_group1',
            output='screen',
            # Force node name and namespace using explicit arguments
            arguments=['--ros-args', '-r', '__node:=formation_coordinator_group1', '-r', '__ns:=/formation_1'],
            parameters=[
                formation_config_group1,
                {'use_sim_time': use_sim_time}
            ]
        )
    ])

    # Group 2 Formation Coordinator (PID)
    formation_coordinator_group2 = GroupAction([
        PushRosNamespace('formation_2'),
        Node(
            package='formation_coordinator_pkg',
            executable='formation_coordinator_node',
            name='formation_coordinator_group2',
            output='screen',
            # Force node name and namespace using explicit arguments
            arguments=['--ros-args', '-r', '__node:=formation_coordinator_group2', '-r', '__ns:=/formation_2'],
            parameters=[
                formation_config_group2,
                {'use_sim_time': use_sim_time}
            ]
        )
    ])

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
        declare_physics_model,
        declare_world_file,
        declare_wind_profile,
        declare_pid_group_kp,
        declare_pid_group_ki,
        declare_pid_group_kd,
        declare_fuzzy_group_kp,
        declare_fuzzy_group_ki,
        declare_fuzzy_group_kd,
        declare_fuzzy_group_k_fuzzy,
        declare_wind_source_topic,
        declare_wind_source_type,
        declare_settled_pos_threshold,
        declare_settled_time_window,
        declare_metrics_window_sec,

        # Gazebo
        gzserver,
        gzclient,

        # Wind disturbance
        wind_publisher,

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
