#!/usr/bin/env python3
"""2D LiDARスキャンから人(脚)を検出して追従するノード。

アルゴリズム:
  1. LaserScanを距離の不連続(ギャップ)でクラスタに分割
  2. 人らしいクラスタ(幅0.05〜0.7m、検出範囲内)を候補にする
  3. 前回のターゲット位置に最も近い候補を選択(トラッキング)
  4. P制御で cmd_vel を出力(目標距離を保ち、正面に捉える)
  5. 一定時間見失ったら停止
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class FollowerNode(Node):

    def __init__(self):
        super().__init__('follower')

        # パラメータ
        self.declare_parameter('target_distance', 1.2)   # 保つ距離 [m]
        self.declare_parameter('stop_distance', 0.8)     # これ以下なら前進しない [m]
        self.declare_parameter('max_linear', 0.7)        # 最大並進速度 [m/s]
        self.declare_parameter('max_angular', 1.2)       # 最大旋回速度 [rad/s]
        self.declare_parameter('k_linear', 0.8)
        self.declare_parameter('k_angular', 1.8)
        self.declare_parameter('detect_range_max', 6.0)  # 探索最大距離 [m]
        self.declare_parameter('detect_angle', 2.0)      # 探索角度 ±[rad]
        self.declare_parameter('cluster_gap', 0.35)      # クラスタ分割ギャップ [m]
        self.declare_parameter('cluster_width_min', 0.05)
        self.declare_parameter('cluster_width_max', 0.7)
        self.declare_parameter('track_radius', 0.8)      # 前回位置からの許容移動 [m]
        self.declare_parameter('lost_timeout', 1.0)      # 見失い許容時間 [s]

        self.target_pos = None      # (x, y) LiDAR座標系
        self.last_seen_time = None

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        self.timer = self.create_timer(0.1, self.watchdog)

        self.get_logger().info('人追従ノード起動')

    # ---------- クラスタリング ----------
    def extract_clusters(self, scan: LaserScan):
        """スキャンを (x, y) 点群クラスタのリストに分割"""
        gap = self.get_parameter('cluster_gap').value
        rmax = self.get_parameter('detect_range_max').value
        amax = self.get_parameter('detect_angle').value

        points = []
        for i, r in enumerate(scan.ranges):
            angle = scan.angle_min + i * scan.angle_increment
            if not math.isfinite(r):
                continue
            if r < scan.range_min or r > rmax:
                continue
            if abs(angle) > amax:
                continue
            points.append((r * math.cos(angle), r * math.sin(angle), r, angle))

        clusters = []
        current = []
        for p in points:
            if current:
                px, py = current[-1][0], current[-1][1]
                if math.hypot(p[0] - px, p[1] - py) > gap:
                    clusters.append(current)
                    current = []
            current.append(p)
        if current:
            clusters.append(current)
        return clusters

    def person_candidates(self, clusters):
        """人らしいクラスタの重心リストを返す"""
        wmin = self.get_parameter('cluster_width_min').value
        wmax = self.get_parameter('cluster_width_max').value
        rmax = self.get_parameter('detect_range_max').value
        amax = self.get_parameter('detect_angle').value
        margin = 0.05

        candidates = []
        for c in clusters:
            if len(c) < 3:
                continue
            # 視野境界・距離境界で切れたクラスタは幅を正しく測れない
            # (大きな物体の一部が"人らしい幅"に見える)ため除外
            if c[0][3] <= -amax + margin or c[-1][3] >= amax - margin:
                continue
            if c[0][2] >= rmax - 0.15 or c[-1][2] >= rmax - 0.15:
                continue
            x0, y0 = c[0][0], c[0][1]
            x1, y1 = c[-1][0], c[-1][1]
            width = math.hypot(x1 - x0, y1 - y0)
            if not (wmin <= width <= wmax):
                continue
            cx = sum(p[0] for p in c) / len(c)
            cy = sum(p[1] for p in c) / len(c)
            candidates.append((cx, cy))
        return candidates

    # ---------- メイン処理 ----------
    def scan_callback(self, scan: LaserScan):
        candidates = self.person_candidates(self.extract_clusters(scan))
        if not candidates:
            return

        track_r = self.get_parameter('track_radius').value

        if self.target_pos is not None:
            # 前回位置に近い候補を優先(トラッキング)
            tx, ty = self.target_pos
            near = [(math.hypot(cx - tx, cy - ty), (cx, cy))
                    for cx, cy in candidates]
            near.sort(key=lambda e: e[0])
            if near[0][0] <= track_r:
                self.target_pos = near[0][1]
                self.last_seen_time = self.get_clock().now()
                self.publish_cmd()
                return
            # 追跡失敗 → タイムアウトまでは最後の位置を保持
            if not self.is_lost():
                return
            self.target_pos = None

        # 新規ターゲット: 最も近い候補
        candidates.sort(key=lambda c: math.hypot(c[0], c[1]))
        self.target_pos = candidates[0]
        self.last_seen_time = self.get_clock().now()
        self.publish_cmd()

    def publish_cmd(self):
        tx, ty = self.target_pos
        dist = math.hypot(tx, ty)
        bearing = math.atan2(ty, tx)

        target_d = self.get_parameter('target_distance').value
        stop_d = self.get_parameter('stop_distance').value
        k_lin = self.get_parameter('k_linear').value
        k_ang = self.get_parameter('k_angular').value
        max_lin = self.get_parameter('max_linear').value
        max_ang = self.get_parameter('max_angular').value

        cmd = Twist()
        cmd.angular.z = max(-max_ang, min(max_ang, k_ang * bearing))

        if dist > stop_d:
            v = k_lin * (dist - target_d)
            # 大きく横を向いているときは前進を抑える
            v *= max(0.0, math.cos(bearing))
            cmd.linear.x = max(0.0, min(max_lin, v))

        self.cmd_pub.publish(cmd)

    # ---------- 見失い処理 ----------
    def is_lost(self):
        if self.last_seen_time is None:
            return True
        timeout = self.get_parameter('lost_timeout').value
        elapsed = (self.get_clock().now() - self.last_seen_time).nanoseconds * 1e-9
        return elapsed > timeout

    def watchdog(self):
        if self.target_pos is None or self.is_lost():
            self.target_pos = None
            self.cmd_pub.publish(Twist())  # 停止


def main(args=None):
    rclpy.init(args=args)
    node = FollowerNode()
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
