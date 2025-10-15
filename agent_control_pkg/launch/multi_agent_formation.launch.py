import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    controller_params = LaunchConfiguration('controller_params')
    formation_params = LaunchConfiguration('formation_params')

    controller_params_default = os.path.join(
        get_package_share_directory('agent_control_pkg'),
        'config', 'ros2', 'agent_controller_default.yaml')
    formation_params_default = os.path.join(
        get_package_share_directory('formation_coordinator_pkg'),
        'config', 'formation_config.yaml')

    controller_arg = DeclareLaunchArgument(
        'controller_params',
        default_value=controller_params_default,
        description='YAML parameter file for each agent controller node')

    formation_arg = DeclareLaunchArgument(
        'formation_params',
        default_value=formation_params_default,
        description='YAML parameter file for the formation coordinator node')

    agent_ids = ['agent_0', 'agent_1', 'agent_2']
    controller_groups = []
    for agent_id in agent_ids:
        controller_groups.append(
            GroupAction([
                PushRosNamespace(agent_id),
                Node(
                    package='agent_control_pkg',
                    executable='agent_controller_node',
                    name='agent_controller',
                    output='screen',
                    parameters=[controller_params]
                )
            ])
        )

    formation_node = Node(
        package='formation_coordinator_pkg',
        executable='formation_coordinator_node',
        name='formation_coordinator',
        output='screen',
        parameters=[formation_params]
    )

    return LaunchDescription([
        controller_arg,
        formation_arg,
        formation_node,
        *controller_groups,
    ])
