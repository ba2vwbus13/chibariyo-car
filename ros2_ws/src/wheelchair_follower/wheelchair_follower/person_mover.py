#!/usr/bin/env python3
"""Gazebo内の人モデル(person)をウェイポイント巡回させるノード。

Gazebo Fortress対応: worldのVelocityControl/OdometryPublisherプラグインを利用し、
/person/odom(現在位置)を見ながら /person/cmd_vel(速度指令)で閉ループ制御する。
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class PersonMover(Node):

    def __init__(self):
        super().__init__('person_mover')

        self.declare_parameter('speed', 0.4)      # 歩行速度 [m/s]
        self.declare_parameter('k_yaw', 2.0)      # 旋回ゲイン
        self.declare_parameter('goal_tolerance', 0.15)  # 到達判定 [m]
        self.declare_parameter('robot_stop_radius', 0.7)  # 車椅子がこの距離内なら立ち止まる [m]
        # 車椅子odom原点のworld座標(sim.launch.pyのスポーン位置に合わせる)
        self.declare_parameter('robot_world_offset', [-1.0, 0.0])
        # 巡回ウェイポイント [x1,y1, x2,y2, ...]
        # 部屋の外周(壁から1.5m)を一周 → 障害物の間を縫って中央へ → 戻る長めのコース
        self.declare_parameter('waypoints',
                               [2.0, 0.0,
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
                                2.5, 2.5])

        wp = self.get_parameter('waypoints').value
        self.waypoints = [(wp[i], wp[i + 1]) for i in range(0, len(wp), 2)]
        self.wp_index = 1
        self.pose = None  # (x, y, yaw)

        self.robot_pos = None  # 車椅子のworld座標

        self.cmd_pub = self.create_publisher(Twist, '/person/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/person/odom', self.odom_callback, 10)
        self.robot_odom_sub = self.create_subscription(
            Odometry, '/odom', self.robot_odom_callback, 10)
        self.timer = self.create_timer(0.05, self.step)

        self.get_logger().info('人モデル移動ノード起動(/person/odom 待機中)')

    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (p.x, p.y, yaw)

    def robot_odom_callback(self, msg: Odometry):
        off = self.get_parameter('robot_world_offset').value
        p = msg.pose.pose.position
        self.robot_pos = (p.x + off[0], p.y + off[1])

    def step(self):
        if self.pose is None:
            return

        x, y, yaw = self.pose

        # 車椅子が目の前にいるときは立ち止まる(衝突防止)
        stop_r = self.get_parameter('robot_stop_radius').value
        if self.robot_pos is not None:
            if math.hypot(self.robot_pos[0] - x,
                          self.robot_pos[1] - y) < stop_r:
                self.cmd_pub.publish(Twist())
                return
        gx, gy = self.waypoints[self.wp_index]
        dist = math.hypot(gx - x, gy - y)

        if dist < self.get_parameter('goal_tolerance').value:
            self.wp_index = (self.wp_index + 1) % len(self.waypoints)
            return

        # 目標方向への旋回 + 前進(体正面方向に移動)
        target_yaw = math.atan2(gy - y, gx - x)
        yaw_err = math.atan2(math.sin(target_yaw - yaw),
                             math.cos(target_yaw - yaw))

        k_yaw = self.get_parameter('k_yaw').value
        speed = self.get_parameter('speed').value

        cmd = Twist()
        cmd.angular.z = max(-1.5, min(1.5, k_yaw * yaw_err))
        cmd.linear.x = speed if abs(yaw_err) < 0.6 else 0.0
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PersonMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
