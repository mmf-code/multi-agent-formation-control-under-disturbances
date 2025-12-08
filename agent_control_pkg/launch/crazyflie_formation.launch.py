"""
Crazyflie Formation Control Launch File

Launches agent_controller_node(s) configured for Crazyflie/Crazyswarm2 simulation.
Requires: colcon build --cmake-args -DENABLE_CRAZYFLIE=ON

Usage:
  ros2 launch agent_control_pkg crazyflie_formation.launch.py
  ros2 launch agent_control_pkg crazyflie_formation.launch.py num_agents:=3
  ros2 launch agent_control_pkg crazyflie_formation.launch.py use_sim_time:=false  # For real hardware
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('agent_control_pkg')
    config_file = os.path.join(pkg_share, 'config', 'crazyflie_params.yaml')

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time (true for Gazebo sim, false for real hardware)'
    )

    num_agents_arg = DeclareLaunchArgument(
        'num_agents',
        default_value='1',
        description='Number of Crazyflie agents (1-3)'
    )

    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    num_agents = LaunchConfiguration('num_agents')

    # Crazyflie 0 controller (cf0 -> cf231 in Crazyswarm2 sim)
    cf0_controller = Node(
        package='agent_control_pkg',
        executable='agent_controller_node',
        name='agent_controller',
        namespace='cf0',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('odom', '/cf231/odom'),
            ('target_pose', '/cf231/target_pose'),
            ('cmd_full_state', '/cf231/cmd_full_state'),
        ],
        output='screen'
    )

    # Crazyflie 1 controller (cf1 -> cf232 in Crazyswarm2 sim)
    cf1_controller = Node(
        package='agent_control_pkg',
        executable='agent_controller_node',
        name='agent_controller',
        namespace='cf1',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('odom', '/cf232/odom'),
            ('target_pose', '/cf232/target_pose'),
            ('cmd_full_state', '/cf232/cmd_full_state'),
        ],
        output='screen',
        condition=IfCondition(PythonExpression([num_agents, ' >= 2']))
    )

    # Crazyflie 2 controller (cf2 -> cf233 in Crazyswarm2 sim)
    cf2_controller = Node(
        package='agent_control_pkg',
        executable='agent_controller_node',
        name='agent_controller',
        namespace='cf2',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('odom', '/cf233/odom'),
            ('target_pose', '/cf233/target_pose'),
            ('cmd_full_state', '/cf233/cmd_full_state'),
        ],
        output='screen',
        condition=IfCondition(PythonExpression([num_agents, ' >= 3']))
    )

    return LaunchDescription([
        use_sim_time_arg,
        num_agents_arg,
        cf0_controller,
        cf1_controller,
        cf2_controller,
    ])
