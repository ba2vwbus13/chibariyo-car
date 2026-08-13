#!/usr/bin/env python3
"""車椅子が人の踊りの動きを真似るノード(コール&レスポンス)。

人モデルの動き(/person/odom のツイスト = 前後速度・旋回速度)を
遅延バッファに貯め、delay 秒前の動きを自分の /cmd_vel として再生する。
これにより「人が踊る → 少し遅れて車椅子が同じ動きを真似る」ようになる。

差動二輪の車椅子は横移動できないため、人の前後(linear.x)・旋回(angular.z)
成分のみを真似る(踊りの振り付けもこの2成分で構成している)。
"""
import collections

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class DanceMimic(Node):

    def __init__(self):
        super().__init__('dance_mimic')

        self.declare_parameter('delay', 1.2)          # 真似るまでの遅れ [s]
        self.declare_parameter('linear_scale', 1.0)   # 前後動作の倍率
        self.declare_parameter('angular_scale', 1.0)  # 旋回動作の倍率
        self.declare_parameter('deadband', 0.02)      # 微小値は0とみなす

        self.buffer = collections.deque()  # (t_sec, linear, angular)

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/person/odom', self.odom_callback, 10)
        self.timer = self.create_timer(0.05, self.replay)

        self.get_logger().info('物真似ダンス開始(人の動きを待機中)')

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def odom_callback(self, msg: Odometry):
        lin = msg.twist.twist.linear.x
        ang = msg.twist.twist.angular.z
        db = self.get_parameter('deadband').value
        if abs(lin) < db:
            lin = 0.0
        if abs(ang) < db:
            ang = 0.0
        self.buffer.append((self.now_sec(), lin, ang))

    def replay(self):
        if not self.buffer:
            return
        delay = self.get_parameter('delay').value
        target_t = self.now_sec() - delay

        # target_t 以前で最も新しいサンプルを取り出す
        sample = None
        while self.buffer and self.buffer[0][0] <= target_t:
            sample = self.buffer.popleft()
        if sample is None:
            return

        cmd = Twist()
        cmd.linear.x = sample[1] * self.get_parameter('linear_scale').value
        cmd.angular.z = sample[2] * self.get_parameter('angular_scale').value
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = DanceMimic()
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
