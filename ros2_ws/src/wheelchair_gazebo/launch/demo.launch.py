"""全部入り: Gazebo + 車椅子 + 人の移動 + 追従ノード

追従方式の切り替え:
  ros2 launch wheelchair_gazebo demo.launch.py                  # LiDAR追従(既定)
  ros2 launch wheelchair_gazebo demo.launch.py method:=camera   # カメラ追従
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo = get_package_share_directory('wheelchair_gazebo')

    method_arg = DeclareLaunchArgument(
        'method', default_value='lidar',
        description='追従方式: lidar / camera / yolo')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'sim.launch.py')))

    person_mover = Node(
        package='wheelchair_follower',
        executable='person_mover',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    lidar_follower = Node(
        package='wheelchair_follower',
        executable='follower',
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=LaunchConfigurationEquals('method', 'lidar'),
    )

    camera_follower = Node(
        package='wheelchair_follower',
        executable='camera_follower',
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=LaunchConfigurationEquals('method', 'camera'),
    )

    yolo_follower = Node(
        package='wheelchair_follower',
        executable='yolo_follower',
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=LaunchConfigurationEquals('method', 'yolo'),
    )

    # Gazebo起動が落ち着いてからノードを開始
    delayed = TimerAction(
        period=8.0,
        actions=[person_mover, lidar_follower, camera_follower, yolo_follower])

    return LaunchDescription([method_arg, sim, delayed])
