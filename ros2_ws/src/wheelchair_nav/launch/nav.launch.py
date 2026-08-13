"""Nav2スタック起動(マップなし・odom基準)"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('wheelchair_nav'),
        'config', 'nav2_params.yaml')

    nodes = []
    for pkg, exe, name in [
        ('nav2_controller', 'controller_server', 'controller_server'),
        ('nav2_planner', 'planner_server', 'planner_server'),
        ('nav2_behaviors', 'behavior_server', 'behavior_server'),
        ('nav2_bt_navigator', 'bt_navigator', 'bt_navigator'),
    ]:
        nodes.append(Node(
            package=pkg,
            executable=exe,
            name=name,
            output='screen',
            parameters=[params],
        ))

    lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['controller_server',
                           'planner_server',
                           'behavior_server',
                           'bt_navigator'],
        }],
    )

    return LaunchDescription(nodes + [lifecycle])
