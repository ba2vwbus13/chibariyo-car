#!/usr/bin/env python3
"""人モデルにその場で踊らせる振り付けノード。

/person/cmd_vel に速度指令(Twist)の並びを出力する。
各振り付けは (継続時間[s], 前後速度[m/s], 旋回速度[rad/s]) で定義。
車椅子側の dance_mimic ノードがこの動きを少し遅れて真似る。
"""
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist


# 振り付け: (duration [s], linear.x [m/s], angular.z [rad/s])
CHOREOGRAPHY = [
    (1.0, 0.0,  0.0),   # 静止(構え)
    (2.0, 0.0,  1.6),   # 左スピン
    (2.0, 0.0, -1.6),   # 右スピン
    (0.5, 0.35, 0.0),   # 前
    (0.5, -0.35, 0.0),  # 後
    (0.5, 0.35, 0.0),   # 前
    (0.5, -0.35, 0.0),  # 後
    (0.4, 0.0,  1.8),   # ウィグル(左右に小刻み)
    (0.4, 0.0, -1.8),
    (0.4, 0.0,  1.8),
    (0.4, 0.0, -1.8),
    (1.5, 0.0,  2.2),   # 決めの高速スピン
    (1.0, 0.0,  0.0),   # ポーズ
]


class PersonDancer(Node):

    def __init__(self):
        super().__init__('person_dancer')
        self.declare_parameter('loop', True)   # 振り付けを繰り返すか
        self.declare_parameter('speed_scale', 1.0)

        self.pub = self.create_publisher(Twist, '/person/cmd_vel', 10)
        self.idx = 0
        self.move_end = None       # 現在の振り付けの終了時刻
        self.cur = (0.0, 0.0)      # 現在の(linear, angular)
        self.finished = False
        self.timer = self.create_timer(0.05, self.step)
        self.get_logger().info('踊り開始')

    def next_move(self, now):
        """次の振り付けへ進む。全て終わってloop=Falseなら終了を返す"""
        if self.idx >= len(CHOREOGRAPHY):
            if self.get_parameter('loop').value:
                self.idx = 0
            else:
                self.finished = True
                self.get_logger().info('踊り終了')
                return
        dur, lin, ang = CHOREOGRAPHY[self.idx]
        self.cur = (lin, ang)
        self.move_end = now + Duration(seconds=dur)
        self.idx += 1

    def step(self):
        if self.finished:
            self.pub.publish(Twist())
            return

        now = self.get_clock().now()
        if self.move_end is None or now >= self.move_end:
            self.next_move(now)
            if self.finished:
                return

        scale = self.get_parameter('speed_scale').value
        cmd = Twist()
        cmd.linear.x = self.cur[0] * scale
        cmd.angular.z = self.cur[1] * scale
        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PersonDancer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
