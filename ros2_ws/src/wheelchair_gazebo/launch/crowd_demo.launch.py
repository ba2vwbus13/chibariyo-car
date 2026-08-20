"""人混み(通行人あり)での追尾・Re-ID評価デモ

  Gazebo(crowd_test.world) + 車椅子 + 本人(person) + 通行人(person2)
  + 追従ノード + 計測ノード(metrics_logger) を一括起動する。

使い方:
  ros2 launch wheelchair_gazebo crowd_demo.launch.py                  # Re-ID方式(既定)
  ros2 launch wheelchair_gazebo crowd_demo.launch.py method:=yolo     # 比較: 最近傍YOLO
  ros2 launch wheelchair_gazebo crowd_demo.launch.py method:=lidar    # 比較: LiDAR

  計測結果は ~/ros2_ws/metrics/ に CSV と summary_*.txt で保存される。
  60〜120秒ほど走らせて Ctrl+C で終了すると summary が確定する。

シナリオ:
  本人は部屋を周回し、通行人(赤い帯)は部屋の中央を南北に往復する。
  車椅子と本人の間を通行人が繰り返し横切るため、
  「別人を追いかけないか(ID維持率)」「見失いから何秒で復帰するか」を測れる。
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
    'yolo': 'yolo_follower',
    'reid': 'yolo_reid_follower',
}

SPAWN_X = -1.0
SPAWN_Y = 0.0

# 本人の周回コース(障害物の間を縫う既定コースの短縮版)
TARGET_WAYPOINTS = [2.0, 0.0,
                    4.5, 2.0,
                    4.5, 4.5,
                    0.0, 4.5,
                    -4.0, 4.5,
                    -4.5, 0.0,
                    -4.0, -4.5,
                    0.0, -4.5,
                    4.0, -4.5,
                    4.5, -1.5,
                    2.0, -2.5,
                    -1.0, -2.0,
                    -2.0, 1.5,
                    0.0, 3.0,
                    2.5, 2.5]

# 通行人: 部屋の中央(x=0.5)を南北に往復し、車椅子と本人の間を横切る
DISTRACTOR_WAYPOINTS = [0.5, -4.5,
                        0.5, 4.5]


def launch_setup(context, *args, **kwargs):
    pkg_gazebo = get_package_share_directory('wheelchair_gazebo')
    method = LaunchConfiguration('method').perform(context)
    if method not in FOLLOWER_EXECUTABLES:
        raise RuntimeError(
            f'method は {list(FOLLOWER_EXECUTABLES)} のいずれかです: {method}')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'sim.launch.py')),
        launch_arguments={'world': 'crowd_test.world',
                          'spawn_x': str(SPAWN_X),
                          'spawn_y': str(SPAWN_Y)}.items(),
    )

    # person2用の追加ブリッジ(sim.launch.pyはperson1のみブリッジするため)
    bridge2 = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='person2_bridge',
        arguments=[
            '/model/person2/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/model/person2/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
        ],
        remappings=[
            ('/model/person2/cmd_vel', '/person2/cmd_vel'),
            ('/model/person2/odometry', '/person2/odom'),
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    person_mover = Node(
        package='wheelchair_follower',
        executable='person_mover',
        name='person_mover',
        parameters=[{'use_sim_time': True,
                     'waypoints': TARGET_WAYPOINTS,
                     'speed': 0.4,
                     'robot_world_offset': [SPAWN_X, SPAWN_Y]}],
        output='screen',
    )

    # 通行人: トピックをperson2系に付け替えた2つ目のperson_mover
    person2_mover = Node(
        package='wheelchair_follower',
        executable='person_mover',
        name='person2_mover',
        remappings=[
            ('/person/cmd_vel', '/person2/cmd_vel'),
            ('/person/odom', '/person2/odom'),
        ],
        parameters=[{'use_sim_time': True,
                     'waypoints': DISTRACTOR_WAYPOINTS,
                     'speed': 0.6,
                     'robot_stop_radius': 0.35,  # 車椅子の近くでも歩き続けて横切る
                     'robot_world_offset': [SPAWN_X, SPAWN_Y]}],
        output='screen',
    )

    follower = Node(
        package='wheelchair_follower',
        executable=FOLLOWER_EXECUTABLES[method],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    metrics = Node(
        package='wheelchair_follower',
        executable='metrics_logger',
        parameters=[{'use_sim_time': True,
                     'label': method,
                     'robot_world_offset': [SPAWN_X, SPAWN_Y]}],
        output='screen',
    )

    # Gazebo起動が落ち着いてからノードを開始
    return [sim, bridge2,
            TimerAction(period=8.0,
                        actions=[person_mover, person2_mover,
                                 follower, metrics])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('method', default_value='reid',
                              description='追従方式: reid / yolo / lidar'),
        OpaqueFunction(function=launch_setup),
    ])
