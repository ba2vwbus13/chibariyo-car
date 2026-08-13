"""Nav2統合デモ: Gazebo + 車椅子 + 人の移動 + Nav2 + Nav2追従ノード

車椅子はNav2の経路計画・障害物回避を使って人を追従する。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo = get_package_share_directory('wheelchair_gazebo')
    pkg_nav = get_package_share_directory('wheelchair_nav')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'sim.launch.py')))

    person_mover = Node(
        package='wheelchair_follower',
        executable='person_mover',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'nav.launch.py')))

    nav_follower = Node(
        package='wheelchair_follower',
        executable='nav_follower',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        sim,
        # Gazebo起動が落ち着いてからNav2と人の移動を開始
        TimerAction(period=8.0, actions=[person_mover, nav2]),
        # Nav2のライフサイクル起動完了を待ってから追従開始
        TimerAction(period=16.0, actions=[nav_follower]),
    ])
