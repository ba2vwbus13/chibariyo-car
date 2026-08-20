"""全部入り: Gazebo + 車椅子 + 人の移動 + 追従ノード

追従方式の切り替え:
  ros2 launch wheelchair_gazebo demo.launch.py                  # LiDAR追従(既定)
  ros2 launch wheelchair_gazebo demo.launch.py method:=camera   # カメラ追従
  ros2 launch wheelchair_gazebo demo.launch.py method:=yolo     # YOLO追従

ワールドの切り替え:
  ros2 launch wheelchair_gazebo demo.launch.py world:=facility.world
  → 人の巡回ルートは config/person_waypoints_<ワールド名>.yaml が自動で使われる
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

FOLLOWER_EXECUTABLES = {
    'lidar': 'follower',
    'camera': 'camera_follower',
    'yolo': 'yolo_follower',
}


def launch_setup(context, *args, **kwargs):
    pkg_gazebo = get_package_share_directory('wheelchair_gazebo')

    world_name = LaunchConfiguration('world').perform(context)
    method = LaunchConfiguration('method').perform(context)
    spawn_x = float(LaunchConfiguration('spawn_x').perform(context))
    spawn_y = float(LaunchConfiguration('spawn_y').perform(context))

    if method not in FOLLOWER_EXECUTABLES:
        raise RuntimeError(
            f'method は {list(FOLLOWER_EXECUTABLES)} のいずれかです: {method}')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'sim.launch.py')),
        launch_arguments={'world': world_name,
                          'spawn_x': str(spawn_x),
                          'spawn_y': str(spawn_y)}.items(),
    )

    # ワールドに対応する巡回ルートがあれば読み込む(無ければノードの既定値)
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

    follower = Node(
        package='wheelchair_follower',
        executable=FOLLOWER_EXECUTABLES[method],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # Gazebo起動が落ち着いてからノードを開始
    return [sim, TimerAction(period=8.0, actions=[person_mover, follower])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('method', default_value='lidar',
                              description='追従方式: lidar / camera / yolo'),
        DeclareLaunchArgument('world', default_value='follow_test.world',
                              description='ワールドファイル名 (例: facility.world)'),
        DeclareLaunchArgument('spawn_x', default_value='-1.0',
                              description='車椅子のスポーンX座標'),
        DeclareLaunchArgument('spawn_y', default_value='0.0',
                              description='車椅子のスポーンY座標'),
        OpaqueFunction(function=launch_setup),
    ])
