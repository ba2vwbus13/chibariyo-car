"""踊り物真似デモ: Gazebo + 車椅子 + 人の踊り + 車椅子の物真似

人がその場で踊り(person_dancer)、車椅子が少し遅れて同じ動きを真似る(dance_mimic)。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo = get_package_share_directory('wheelchair_gazebo')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'sim.launch.py')))

    person_dancer = Node(
        package='wheelchair_follower',
        executable='person_dancer',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    dance_mimic = Node(
        package='wheelchair_follower',
        executable='dance_mimic',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # Gazebo起動が落ち着いてから踊りを開始
    delayed = TimerAction(period=8.0, actions=[person_dancer, dance_mimic])

    return LaunchDescription([sim, delayed])
