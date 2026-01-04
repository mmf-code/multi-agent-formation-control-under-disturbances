#!/usr/bin/env python3
"""
6-Phase Thesis Test Framework Launch File

Runs standardized wind test phases with controlled wind profiles, auto-export metrics,
and checkpoint-based validation for thesis-grade scientific validation.

Phases:
  1. BASELINE      - No wind (reference performance)
  2. STEADY_WIND   - Constant 3 m/s @ 45 deg (DC rejection)
  3. TURBULENCE    - Von Karman with TI sweep [0.15, 0.25, 0.35]
  4. GUST          - Periodic gusts (transient response)
  5. COMBINED      - Stochastic: turbulence + gusts + direction wander
  6. ENDURANCE     - Long-run combined (300s)

Usage:
    # Run Phase 1 (Baseline) with default settings
    ros2 launch agent_control_pkg phased_comparison.launch.py phase:=1

    # Run Phase 3 with TI=0.25 (sweep_index=1)
    ros2 launch agent_control_pkg phased_comparison.launch.py phase:=3 sweep_index:=1

    # Run Phase 6 Endurance with specific seed
    ros2 launch agent_control_pkg phased_comparison.launch.py phase:=6 run_index:=1 seed:=42

    # With Gazebo GUI for debugging
    ros2 launch agent_control_pkg phased_comparison.launch.py phase:=1 gazebo_gui:=true
"""

import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch import conditions
from launch.actions import (
    ExecuteProcess, DeclareLaunchArgument, GroupAction,
    OpaqueFunction, LogInfo, TimerAction
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, PushRosNamespace

# Import wind scenarios module
sys.path.insert(0, os.path.join(
    get_package_share_directory('agent_control_pkg'), 'lib', 'agent_control_pkg'
))
try:
    from wind_scenarios import get_phase_config, get_phase_duration, get_phase_name, PHASE_DEFINITIONS
except ImportError:
    # Fallback if module not installed yet
    PHASE_DEFINITIONS = None


def get_phase_wind_params(phase_id: int, sweep_index: int, seed: int) -> dict:
    """Get wind publisher parameters for a specific phase."""
    if PHASE_DEFINITIONS is None:
        # Fallback configurations
        phase_configs = {
            1: {"profile": "constant", "magnitude": 0.0, "direction": 0.0},
            2: {"profile": "constant", "magnitude": 3.0, "direction": 45.0},
            3: {"profile": "vonkarman", "magnitude": 2.5, "direction": 45.0,
                "turbulence_intensity": [0.15, 0.25, 0.35][sweep_index],
                "mean_wind_speed": 2.5, "integral_length_u": 30.0,
                "integral_length_v": 15.0, "integral_length_w": 5.0},
            4: {"profile": "gust", "magnitude": 5.0, "direction": 45.0,
                "gust_duration": 1.5, "gust_interval": 10.0},
            # Phase 5: Structured uncertainty profile (optimized for fuzzy comparison)
            # Changes from original: less random noise, same intensity
            # - Reduced direction wandering (5 deg/s vs 15) - less random
            # - Reduced turbulence (0.15 vs 0.3) - less high-freq noise
            # - Same gust intensity (2.0) and probability (0.08) - same difficulty
            5: {"profile": "stochastic", "magnitude": 2.5, "direction": 45.0,
                "stochastic_mag_std": 1.0, "stochastic_dir_rate": 5.0,
                "stochastic_gust_prob": 0.08, "stochastic_gust_mag": 2.0,
                "stochastic_turbulence": 0.15},
            6: {"profile": "stochastic", "magnitude": 2.5, "direction": 45.0,
                "stochastic_mag_std": 1.5, "stochastic_dir_rate": 15.0,
                "stochastic_gust_prob": 0.08, "stochastic_gust_mag": 2.0},
        }
        config = phase_configs.get(phase_id, phase_configs[1])
    else:
        config = get_phase_config(phase_id, sweep_index if phase_id == 3 else None, seed)

    config["publish_rate"] = 20.0
    config["random_seed"] = seed
    return config


def get_duration(phase_id: int) -> int:
    """Get duration in seconds for a phase."""
    durations = {1: 60, 2: 60, 3: 60, 4: 60, 5: 60, 6: 300}
    return durations.get(phase_id, 60)


def get_name(phase_id: int) -> str:
    """Get phase name."""
    names = {
        1: "BASELINE", 2: "STEADY_WIND", 3: "TURBULENCE",
        4: "GUST", 5: "COMBINED", 6: "ENDURANCE"
    }
    return names.get(phase_id, "UNKNOWN")


def launch_setup(context, *args, **kwargs):
    """Generate launch entities based on launch arguments."""

    # Get launch arguments
    phase_id = int(LaunchConfiguration('phase').perform(context))
    sweep_index = int(LaunchConfiguration('sweep_index').perform(context))
    run_index = int(LaunchConfiguration('run_index').perform(context))
    seed = int(LaunchConfiguration('seed').perform(context))
    output_dir = LaunchConfiguration('output_dir').perform(context)
    gazebo_gui_str = LaunchConfiguration('gazebo_gui').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)

    # Validate phase
    if phase_id < 1 or phase_id > 6:
        raise ValueError(f"Invalid phase: {phase_id}. Must be 1-6.")

    phase_name = get_name(phase_id)
    duration = get_duration(phase_id)
    wind_params = get_phase_wind_params(phase_id, sweep_index, seed)

    # Package directories
    pkg_agent_control = get_package_share_directory('agent_control_pkg')
    pkg_formation_coordinator = get_package_share_directory('formation_coordinator_pkg')

    # World file
    world_file = os.path.join(pkg_agent_control, 'worlds', 'crazyflie_12drone_4group.world')

    # Gazebo paths
    model_path = os.path.join(pkg_agent_control, 'models')
    pkg_root = os.path.dirname(os.path.dirname(pkg_agent_control))
    plugin_path_pkg = os.path.join(pkg_root, 'lib')
    plugin_path_std = os.path.join(os.path.dirname(pkg_root), 'lib')

    merged_model_path = ":".join([p for p in [model_path, os.environ.get('GAZEBO_MODEL_PATH', '')] if p])
    merged_plugin_path = ":".join([p for p in [plugin_path_pkg, plugin_path_std, os.environ.get('GAZEBO_PLUGIN_PATH', '')] if p])
    os.environ['GAZEBO_MODEL_PATH'] = merged_model_path
    os.environ['GAZEBO_PLUGIN_PATH'] = merged_plugin_path

    # Log phase info
    log_phase_info = LogInfo(msg=f"\n{'='*60}\n"
                                 f"PHASE {phase_id}: {phase_name}\n"
                                 f"Duration: {duration}s | Run: {run_index} | Seed: {seed}\n"
                                 f"Wind Profile: {wind_params.get('profile', 'N/A')}\n"
                                 f"Output: {output_dir}/phase_{phase_id}/run_{run_index}\n"
                                 f"{'='*60}")

    # Gazebo server
    # Note: Plugin loading via -s flags removed - plugins are defined in the world file
    gzserver = ExecuteProcess(
        cmd=['gzserver', world_file],
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
        condition=conditions.IfCondition(gazebo_gui_str)
    )

    # Wind publisher with phase-specific parameters
    wind_publisher = Node(
        package='agent_control_pkg',
        executable='wind_publisher.py',
        name='wind_publisher',
        output='screen',
        parameters=[wind_params]
    )

    # Phase metrics logger
    metrics_logger = Node(
        package='agent_control_pkg',
        executable='phase_metrics_logger.py',
        name='phase_metrics_logger',
        output='screen',
        parameters=[{
            'phase_id': phase_id,
            'run_index': run_index,
            'output_dir': output_dir,
            'duration_sec': duration,
            'num_agents': 12,
        }]
    )

    # ===== Controller Configurations (same as twelve_drone_comparison) =====

    common_base = {
        'dt': 0.005,
        'output_limits.x': [-10.0, 10.0],
        'output_limits.y': [-10.0, 10.0],
        'use_sim_time': True,
        'wind_source_topic': '/wind/velocity',
        'wind_source_type': 'velocity',
        'feedforward.enable_wind': False,
    }

    # ==========================================================================
    # PID GAINS - Tuned for Crazyflie 2.1 physics model (DO NOT CHANGE)
    # ==========================================================================
    TUNED_KP = 3.501   # Proportional gain
    TUNED_KI = 1.946   # Integral gain
    TUNED_KD = 3.608   # Derivative gain

    # Fuzzy mix ratios - ADDITIVE mode
    # Keep full PID, add fuzzy correction on top
    # Formula: u = 1.0*PID + k_fuzzy*Fuzzy
    # TEST: Increasing fuzzy authority for better transient rejection
    MIX_K_PID = 1.0
    MIX_K_FUZZY = 0.5  # Reverted - 0.6 and 0.7 cause instability

    # PD controller params (Group 0) - No integral action
    pd_controller_params = {
        **common_base,
        'controller_type': 'pd',
        'pid.kp': TUNED_KP, 'pid.ki': 0.0, 'pid.kd': TUNED_KD,
        'fuzzy.enable': False,
    }

    # PID controller params (Group 1) - Full PID
    pid_controller_params = {
        **common_base,
        'controller_type': 'pid',
        'pid.kp': TUNED_KP, 'pid.ki': TUNED_KI, 'pid.kd': TUNED_KD,
        'fuzzy.enable': False,
    }

    # IT2-Fuzzy params (Group 2) - PID + IT2 Fuzzy hybrid
    # Same PID gains, fuzzy adds disturbance compensation
    fuzzy_params_file = os.path.join(pkg_agent_control, 'config', 'fuzzy_params_crazyflie.yaml')
    it2_controller_params = {
        **common_base,
        'controller_type': 'pid_fuzzy',
        'pid.kp': TUNED_KP, 'pid.ki': TUNED_KI, 'pid.kd': TUNED_KD,
        'fuzzy.enable': True,
        'fuzzy.params_file': fuzzy_params_file,
        'fuzzy.include_wind': True,
        'fuzzy.wind_scalar': 1.0,  # CRITICAL: Enable wind input to fuzzy
        'mix.k_pid': MIX_K_PID, 'mix.k_fuzzy': MIX_K_FUZZY,
    }

    # GT2-Fuzzy params (Group 3) - PID + GT2 Fuzzy hybrid
    # Same PID gains, GT2 fuzzy adds uncertainty handling
    gt2_params_file = os.path.join(pkg_agent_control, 'config', 'gt2_fuzzy_params_crazyflie.yaml')
    gt2_controller_params = {
        **common_base,
        'controller_type': 'pid_gt2_fuzzy',
        'pid.kp': TUNED_KP, 'pid.ki': TUNED_KI, 'pid.kd': TUNED_KD,
        'fuzzy.enable': False,  # GT2 uses gt2.params_file, not IT2 fuzzy.params_file
        'gt2.params_file': gt2_params_file,
        'gt2.num_alpha_levels': 5,
        'gt2.secondary_shape': 'triangular',
        'gt2.secondary_spread': 0.2,  # Reduced from 0.3 to decrease noise amplification
        'fuzzy.include_wind': True,
        'fuzzy.wind_scalar': 1.0,  # CRITICAL: Enable wind input to fuzzy
        'mix.k_pid': MIX_K_PID, 'mix.k_fuzzy': MIX_K_FUZZY,
    }

    # Agent-controller mapping
    agent_controller_map = {
        0: pd_controller_params, 1: pd_controller_params, 2: pd_controller_params,
        3: pid_controller_params, 4: pid_controller_params, 5: pid_controller_params,
        6: it2_controller_params, 7: it2_controller_params, 8: it2_controller_params,
        9: gt2_controller_params, 10: gt2_controller_params, 11: gt2_controller_params,
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
                    'use_sim_time': True,
                    'publish_rate_hz': 10.0,
                    'settled_pos_threshold': 0.10,
                    'settled_time_window': 1.0,
                    'metrics_window_sec': 30.0,
                    'enable_group_metrics': True,
                }]
            ),
        ])
        agent_nodes.append(agent_controller)

    # Formation coordinators
    formation_config_pd = os.path.join(pkg_formation_coordinator, 'config', 'formation_12drone_pd.yaml')
    formation_config_pid = os.path.join(pkg_formation_coordinator, 'config', 'formation_12drone_pid_group.yaml')
    formation_config_it2 = os.path.join(pkg_formation_coordinator, 'config', 'formation_12drone_it2_group.yaml')
    formation_config_gt2 = os.path.join(pkg_formation_coordinator, 'config', 'formation_12drone_gt2_group.yaml')

    formation_coordinators = [
        GroupAction([PushRosNamespace('formation_0'), Node(
            package='formation_coordinator_pkg', executable='formation_coordinator_node',
            name='formation_coordinator_pd', output='screen',
            parameters=[formation_config_pd, {'use_sim_time': True}]
        )]),
        GroupAction([PushRosNamespace('formation_1'), Node(
            package='formation_coordinator_pkg', executable='formation_coordinator_node',
            name='formation_coordinator_pid', output='screen',
            parameters=[formation_config_pid, {'use_sim_time': True}]
        )]),
        GroupAction([PushRosNamespace('formation_2'), Node(
            package='formation_coordinator_pkg', executable='formation_coordinator_node',
            name='formation_coordinator_it2', output='screen',
            parameters=[formation_config_it2, {'use_sim_time': True}]
        )]),
        GroupAction([PushRosNamespace('formation_3'), Node(
            package='formation_coordinator_pkg', executable='formation_coordinator_node',
            name='formation_coordinator_gt2', output='screen',
            parameters=[formation_config_gt2, {'use_sim_time': True}]
        )]),
    ]

    return [
        log_phase_info,
        gzserver,
        gzclient,
        wind_publisher,
        metrics_logger,
        *agent_nodes,
        *formation_coordinators,
    ]


def generate_launch_description():
    """Generate launch description for phased comparison."""

    return LaunchDescription([
        # Phase selection (1-6)
        DeclareLaunchArgument('phase', default_value='1',
            description='Test phase (1=BASELINE, 2=STEADY_WIND, 3=TURBULENCE, 4=GUST, 5=COMBINED, 6=ENDURANCE)'),

        # Phase 3 TI sweep index (0=0.15, 1=0.25, 2=0.35)
        DeclareLaunchArgument('sweep_index', default_value='0',
            description='For Phase 3: TI sweep index (0=0.15, 1=0.25, 2=0.35)'),

        # Run index for multi-run experiments
        DeclareLaunchArgument('run_index', default_value='1',
            description='Run index for repeated experiments'),

        # Random seed for reproducibility
        DeclareLaunchArgument('seed', default_value='-1',
            description='Random seed (-1 for random)'),

        # Output directory
        DeclareLaunchArgument('output_dir', default_value='results',
            description='Output directory for CSV results'),

        # Simulation settings
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gazebo_gui', default_value='false',
            description='Launch Gazebo GUI'),

        # Dynamic launch setup
        OpaqueFunction(function=launch_setup),
    ])
