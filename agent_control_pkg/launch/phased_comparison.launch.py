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

Collision Avoidance Modes:
  - disabled:     No collision avoidance (baseline comparison) [DEFAULT]
  - pass_through: Republish targets without modification (isolate republish effect)
  - avoidance:    APF-based collision avoidance active

Usage:
    # Run Phase 1 (Baseline) with default settings
    ros2 launch agent_control_pkg phased_comparison.launch.py phase:=1

    # Run Phase 3 with TI=0.25 (sweep_index=1)
    ros2 launch agent_control_pkg phased_comparison.launch.py phase:=3 sweep_index:=1

    # Run Phase 6 Endurance with specific seed
    ros2 launch agent_control_pkg phased_comparison.launch.py phase:=6 run_index:=1 seed:=42

    # With Gazebo GUI for debugging
    ros2 launch agent_control_pkg phased_comparison.launch.py phase:=1 gazebo_gui:=true

    # With collision avoidance enabled
    ros2 launch agent_control_pkg phased_comparison.launch.py phase:=5 collision_mode:=avoidance
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
            # CALIBRATED DEMO scenarios - PD fails, Fuzzy survives
            # Phase 7: VIDEO DEMO - Lower base wind + stronger gusts for visible difference
            # Key: Lower constant (2.2 m/s) so drones can track, but frequent gusts (0.15 prob)
            # where fuzzy shows smoother recovery. Gusts 2.5 m/s = dramatic visible jolts
            7: {"profile": "stochastic", "magnitude": 2.2, "direction": 45.0,
                "stochastic_mag_std": 0.4, "stochastic_dir_rate": 6.0,
                "stochastic_gust_prob": 0.15, "stochastic_gust_mag": 2.5,
                "stochastic_turbulence": 0.15, "start_delay_sec": 8.0},
            8: {"profile": "aggressive", "magnitude": 5.5, "direction": 90.0,
                "stochastic_mag_std": 1.2, "stochastic_dir_rate": 20.0,
                "stochastic_gust_prob": 0.12, "stochastic_gust_mag": 1.6,
                "stochastic_turbulence": 1.0, "start_delay_sec": 15.0},
        }
        config = phase_configs.get(phase_id, phase_configs[1])
    else:
        config = get_phase_config(phase_id, sweep_index if phase_id == 3 else None, seed)

    config["publish_rate"] = 20.0
    config["random_seed"] = seed
    # Add wind delay for formation stabilization (Phase 1 has no wind, others get delay)
    # This allows drones to reach formation before wind disturbance begins
    # Use config value if provided, else default: 0s for phase 1, 10s for others
    if "start_delay_sec" not in config:
        config["start_delay_sec"] = 0.0 if phase_id == 1 else 10.0
    return config


def get_duration(phase_id: int) -> int:
    """Get duration in seconds for a phase."""
    durations = {1: 60, 2: 60, 3: 60, 4: 60, 5: 60, 6: 300, 7: 90, 8: 90}
    return durations.get(phase_id, 60)


def get_name(phase_id: int) -> str:
    """Get phase name."""
    names = {
        1: "BASELINE", 2: "STEADY_WIND", 3: "TURBULENCE",
        4: "GUST", 5: "COMBINED", 6: "ENDURANCE",
        7: "EXTREME_DEMO", 8: "STRONG_DEMO"
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
    collision_mode = LaunchConfiguration('collision_mode').perform(context).lower()
    paused = LaunchConfiguration('paused').perform(context).lower() == 'true'
    video_demo = LaunchConfiguration('video_demo').perform(context).lower() == 'true'

    # Validate collision mode
    valid_collision_modes = ['disabled', 'pass_through', 'passthrough', 'avoidance']
    if collision_mode not in valid_collision_modes:
        raise ValueError(f"Invalid collision_mode: {collision_mode}. Must be one of {valid_collision_modes}")

    # Normalize collision mode
    if collision_mode == 'passthrough':
        collision_mode = 'pass_through'

    # Determine if collision avoidance is active (for remapping)
    collision_active = collision_mode != 'disabled'

    # Validate phase
    if phase_id < 1 or phase_id > 8:
        raise ValueError(f"Invalid phase: {phase_id}. Must be 1-8.")

    phase_name = get_name(phase_id)
    duration = get_duration(phase_id)
    wind_params = get_phase_wind_params(phase_id, sweep_index, seed)

    # Package directories
    pkg_agent_control = get_package_share_directory('agent_control_pkg')
    pkg_formation_coordinator = get_package_share_directory('formation_coordinator_pkg')

    # World file - use professional world for video demo
    if video_demo:
        world_file = os.path.join(pkg_agent_control, 'worlds', 'crazyflie_12drone_4group_professional.world')
    else:
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
    collision_info = f"Collision Mode: {collision_mode.upper()}" if collision_active else "Collision Mode: DISABLED (baseline)"
    log_phase_info = LogInfo(msg=f"\n{'='*60}\n"
                                 f"PHASE {phase_id}: {phase_name}\n"
                                 f"Duration: {duration}s | Run: {run_index} | Seed: {seed}\n"
                                 f"Wind Profile: {wind_params.get('profile', 'N/A')}\n"
                                 f"{collision_info}\n"
                                 f"Output: {output_dir}/phase_{phase_id}/run_{run_index}\n"
                                 f"{'='*60}")

    # Gazebo server
    # Note: Plugin loading via -s flags removed - plugins are defined in the world file
    gzserver_cmd = ['gzserver', world_file]
    if paused:
        gzserver_cmd.append('--pause')

    gzserver = ExecuteProcess(
        cmd=gzserver_cmd,
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
    # steady_state_start_sec excludes startup transient from SS metrics
    # Formation stabilization + wind delay = ~15s before steady-state analysis
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
            'steady_state_start_sec': 15.0,  # Exclude first 15s (formation + wind ramp)
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
    # PID GAINS - Tuned for Crazyflie 2.1 physics model
    # FAIR COMPARISON: All controllers use SAME base Kp, Kd values
    # Differences come from:
    #   - PD: No integral (Ki=0) → cannot eliminate steady-state error
    #   - PID: Full PID → integral eliminates steady-state error slowly
    #   - IT2/GT2: PID + Fuzzy rules → intelligent disturbance compensation
    # ==========================================================================
    TUNED_KP = 3.501   # Proportional gain (SAME for all controllers)
    TUNED_KI = 1.946   # Integral gain (only PID, IT2, GT2 use this)
    TUNED_KD = 3.608   # Derivative gain (SAME for all controllers)

    # Fuzzy mix ratios - ADDITIVE mode
    # Keep full PID, add fuzzy correction on top
    # Formula: u = 1.0*PID + k_fuzzy*Fuzzy
    MIX_K_PID = 1.0
    MIX_K_FUZZY = 0.5  # Balanced fuzzy authority (1.0 causes oscillation/divergence)

    # PD controller params (Group 0) - SAME Kp/Kd as others, but Ki=0
    # Structural difference: no integral action → cannot eliminate steady-state error
    pd_controller_params = {
        **common_base,
        'controller_type': 'pd',
        'pid.kp': TUNED_KP, 'pid.ki': 0.0, 'pid.kd': TUNED_KD,  # SAME Kp/Kd, no Ki
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

        # Remapping for collision avoidance: when active, listen to safe_target_pose
        # When disabled, listen to original target_pose (no remapping needed)
        controller_remappings = []
        if collision_active:
            controller_remappings = [('target_pose', 'safe_target_pose')]

        agent_controller = GroupAction([
            PushRosNamespace(agent_namespace),
            Node(
                package='agent_control_pkg',
                executable='agent_controller_node',
                name='agent_controller',
                output='screen',
                parameters=[controller_params],
                remappings=controller_remappings
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

    # Formation coordinators (select video demo or standard configs)
    if video_demo:
        # Video demo: circular trajectories with shape transitions
        config_subdir = 'video_demo'
        formation_config_pd = os.path.join(pkg_formation_coordinator, 'config', config_subdir, 'formation_video_pd.yaml')
        formation_config_pid = os.path.join(pkg_formation_coordinator, 'config', config_subdir, 'formation_video_pid.yaml')
        formation_config_it2 = os.path.join(pkg_formation_coordinator, 'config', config_subdir, 'formation_video_it2.yaml')
        formation_config_gt2 = os.path.join(pkg_formation_coordinator, 'config', config_subdir, 'formation_video_gt2.yaml')
    else:
        # Standard: linear waypoint trajectories
        formation_config_pd = os.path.join(pkg_formation_coordinator, 'config', 'formation_12drone_pd.yaml')
        formation_config_pid = os.path.join(pkg_formation_coordinator, 'config', 'formation_12drone_pid_group.yaml')
        formation_config_it2 = os.path.join(pkg_formation_coordinator, 'config', 'formation_12drone_it2_group.yaml')
        formation_config_gt2 = os.path.join(pkg_formation_coordinator, 'config', 'formation_12drone_gt2_group.yaml')

    formation_coordinators = [
        GroupAction([PushRosNamespace('formation_0'), Node(
            package='formation_coordinator_pkg', executable='formation_coordinator_node',
            name='formation_coordinator_pd', output='screen',
            parameters=[formation_config_pd, {'use_sim_time': False}]
        )]),
        GroupAction([PushRosNamespace('formation_1'), Node(
            package='formation_coordinator_pkg', executable='formation_coordinator_node',
            name='formation_coordinator_pid', output='screen',
            parameters=[formation_config_pid, {'use_sim_time': False}]
        )]),
        GroupAction([PushRosNamespace('formation_2'), Node(
            package='formation_coordinator_pkg', executable='formation_coordinator_node',
            name='formation_coordinator_it2', output='screen',
            parameters=[formation_config_it2, {'use_sim_time': False}]
        )]),
        GroupAction([PushRosNamespace('formation_3'), Node(
            package='formation_coordinator_pkg', executable='formation_coordinator_node',
            name='formation_coordinator_gt2', output='screen',
            parameters=[formation_config_gt2, {'use_sim_time': False}]
        )]),
    ]

    # Collision Safety Layer (only when collision_mode != 'disabled')
    # This node subscribes to all agent odometry and target_pose,
    # then publishes safe_target_pose with APF-based collision avoidance
    collision_safety_nodes = []
    if collision_active:
        collision_params_file = os.path.join(pkg_agent_control, 'config', 'collision_avoidance_params.yaml')
        collision_safety_layer = Node(
            package='agent_control_pkg',
            executable='collision_safety_layer_node',
            name='collision_safety_layer',
            output='screen',
            parameters=[
                collision_params_file,
                {
                    'mode': collision_mode,
                    'num_agents': 12,
                    'use_sim_time': True,
                }
            ]
        )
        collision_safety_nodes.append(collision_safety_layer)

    return [
        log_phase_info,
        gzserver,
        gzclient,
        wind_publisher,
        metrics_logger,
        *collision_safety_nodes,
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

        # Start paused for video recording
        DeclareLaunchArgument('paused', default_value='false',
            description='Start Gazebo paused for video recording'),

        # Video demo mode (use enhanced circular trajectories)
        DeclareLaunchArgument('video_demo', default_value='false',
            description='Use video demo trajectories (circular with shape transitions)'),

        # Collision avoidance mode
        # 'disabled' (default): No collision avoidance - baseline comparison
        # 'pass_through': Republish targets without modification (isolate republish effect)
        # 'avoidance': APF-based collision avoidance active
        DeclareLaunchArgument('collision_mode', default_value='disabled',
            description='Collision avoidance mode: disabled, pass_through, or avoidance'),

        # Dynamic launch setup
        OpaqueFunction(function=launch_setup),
    ])
