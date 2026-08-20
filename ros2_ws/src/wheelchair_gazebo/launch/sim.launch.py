"""Gazebo Fortress起動 + 車椅子スポーン + ROS-GZブリッジ

ワールドの切り替え:
  ros2 launch wheelchair_gazebo sim.launch.py                       # 12m四方の部屋(既定)
  ros2 launch wheelchair_gazebo sim.launch.py world:=facility.world # 屋内施設(廊下+4部屋)

車椅子のスポーン位置も引数で変えられる:
  ros2 launch wheelchair_gazebo sim.launch.py world:=facility.world spawn_x:=-1.0 spawn_y:=0.0
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    pkg_gazebo = get_package_share_directory('wheelchair_gazebo')
    pkg_desc = get_package_share_directory('wheelchair_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_name = LaunchConfiguration('world').perform(context)
    spawn_x = LaunchConfiguration('spawn_x').perform(context)
    spawn_y = LaunchConfiguration('spawn_y').perform(context)

    world = os.path.join(pkg_gazebo, 'worlds', world_name)
    if not os.path.exists(world):
        raise RuntimeError(f'ワールドファイルが見つかりません: {world}')

    xacro_file = os.path.join(pkg_desc, 'urdf', 'wheelchair.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]), value_type=str)

    # Gazebo Fortress (-r: 即時再生開始)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world}'}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}],
        output='screen',
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'wheelchair',
                   '-x', spawn_x, '-y', spawn_y, '-z', '0.1'],
        output='screen',
    )

    # ROS 2 <-> Gazebo トピックブリッジ
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/model/person/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/model/person/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
        ],
        remappings=[
            ('/camera/image', '/camera/image_raw'),
            ('/camera/depth_image', '/camera/depth/image_raw'),
            ('/model/person/cmd_vel', '/person/cmd_vel'),
            ('/model/person/odometry', '/person/odom'),
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return [gz_sim, robot_state_publisher, spawn, bridge]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='follow_test.world',
            description='worlds/ 以下のワールドファイル名 (例: facility.world)'),
        DeclareLaunchArgument(
            'spawn_x', default_value='-1.0',
            description='車椅子のスポーンX座標'),
        DeclareLaunchArgument(
            'spawn_y', default_value='0.0',
            description='車椅子のスポーンY座標'),
        OpaqueFunction(function=launch_setup),
    ])
