"""Nav2統合デモ: Gazebo + 車椅子 + 人の移動 + Nav2 + Nav2追従ノード

車椅子はNav2の経路計画・障害物回避を使って人を追従する。

ワールドの切り替え:
  ros2 launch wheelchair_gazebo nav_demo.launch.py world:=facility.world
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    pkg_gazebo = get_package_share_directory('wheelchair_gazebo')
    pkg_nav = get_package_share_directory('wheelchair_nav')

    world_name = LaunchConfiguration('world').perform(context)
    spawn_x = float(LaunchConfiguration('spawn_x').perform(context))
    spawn_y = float(LaunchConfiguration('spawn_y').perform(context))

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'sim.launch.py')),
        launch_arguments={'world': world_name,
                          'spawn_x': str(spawn_x),
                          'spawn_y': str(spawn_y)}.items(),
    )

    wp_yaml = os.path.join(
        pkg_gazebo, 'config',
        f'person_waypoints_{os.path.splitext(world_name)[0]}.yaml')
    person_params = []
    if os.path.exists(wp_yaml):
        person_params.append(wp_yaml)
    person_params.append({'use_sim_time': True,
                          'robot_world_offset': [spawn_x, spawn_y]})

    person_mover = Node(
        package='wheelchair_follower',
        executable='person_mover',
        parameters=person_params,
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

    return [
        sim,
        # Gazebo起動が落ち着いてからNav2と人の移動を開始
        TimerAction(period=8.0, actions=[person_mover, nav2]),
        # Nav2のライフサイクル起動完了を待ってから追従開始
        TimerAction(period=16.0, actions=[nav_follower]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='follow_test.world',
                              description='ワールドファイル名 (例: facility.world)'),
        DeclareLaunchArgument('spawn_x', default_value='-1.0',
                              description='車椅子のスポーンX座標'),
        DeclareLaunchArgument('spawn_y', default_value='0.0',
                              description='車椅子のスポーンY座標'),
        OpaqueFunction(function=launch_setup),
    ])
