#!/usr/bin/env python3
"""Nav2を使って障害物回避しながら人を追従するノード。

アルゴリズム:
  1. LiDARスキャンから人らしいクラスタを検出(follower_nodeと同じ方式)
  2. TFで人の位置をodom座標系に変換
  3. 「人の手前 standoff [m]」をゴールとしてNav2(NavigateToPose)に送信
  4. 人が動いてゴールが古くなったら新しいゴールで置き換え(自動プリエンプト)
  5. 経路計画・障害物回避・速度指令はすべてNav2が担当

直接 /cmd_vel を出す他方式と違い、このノードはゴールを出すだけ。
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time
from rclpy.duration import Duration

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

import tf2_ros


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class NavFollowerNode(Node):

    def __init__(self):
        super().__init__('nav_follower')

        self.declare_parameter('standoff', 1.0)          # 人の手前で止まる距離 [m]
        self.declare_parameter('goal_update_dist', 0.4)  # 人がこれだけ動いたらゴール更新 [m]
        self.declare_parameter('goal_update_period', 1.0)  # ゴール更新の最短間隔 [s]
        self.declare_parameter('detect_range_max', 6.0)
        self.declare_parameter('detect_angle', 2.2)      # 探索角度 ±[rad]
        self.declare_parameter('cluster_gap', 0.35)
        self.declare_parameter('cluster_width_min', 0.05)
        self.declare_parameter('cluster_width_max', 0.7)
        self.declare_parameter('track_radius', 0.8)
        self.declare_parameter('person_timeout', 3.0)  # 検出が途絶えたらリセット [s]

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.person_odom = None      # (x, y) odom座標系の人位置
        self.last_detect_time = None
        self.last_goal_pos = None    # 最後に送ったゴールの元になった人位置
        self.last_goal_time = None

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        self.timer = self.create_timer(0.2, self.update_goal)

        self.get_logger().info('Nav2追従ノード起動(navigate_to_pose 待機中)')

    # ---------- 人検出(LiDARクラスタ) ----------
    def detect_person(self, scan: LaserScan):
        """人らしいクラスタの重心(LiDAR座標系)を返す。無ければNone"""
        gap = self.get_parameter('cluster_gap').value
        rmax = self.get_parameter('detect_range_max').value
        amax = self.get_parameter('detect_angle').value
        wmin = self.get_parameter('cluster_width_min').value
        wmax = self.get_parameter('cluster_width_max').value

        points = []
        for i, r in enumerate(scan.ranges):
            angle = scan.angle_min + i * scan.angle_increment
            if not math.isfinite(r) or r < scan.range_min or r > rmax:
                continue
            if abs(angle) > amax:
                continue
            points.append((r * math.cos(angle), r * math.sin(angle), r, angle))

        clusters, current = [], []
        for p in points:
            if current and math.hypot(p[0] - current[-1][0],
                                      p[1] - current[-1][1]) > gap:
                clusters.append(current)
                current = []
            current.append(p)
        if current:
            clusters.append(current)

        margin = 0.05  # 視野・距離境界の除外マージン [rad]/[m]
        candidates = []
        for c in clusters:
            if len(c) < 3:
                continue
            # 視野境界で切れたクラスタは幅を正しく測れない(大きな物体の
            # 一部が"人らしい幅"に見える)ため除外
            if c[0][3] <= -amax + margin or c[-1][3] >= amax - margin:
                continue
            # 探索距離の上限付近で切れたクラスタも同様に除外
            if c[0][2] >= rmax - 0.15 or c[-1][2] >= rmax - 0.15:
                continue
            width = math.hypot(c[-1][0] - c[0][0], c[-1][1] - c[0][1])
            if not (wmin <= width <= wmax):
                continue
            cx = sum(p[0] for p in c) / len(c)
            cy = sum(p[1] for p in c) / len(c)
            candidates.append((cx, cy))

        if not candidates:
            return None
        candidates.sort(key=lambda c: math.hypot(c[0], c[1]))
        return candidates[0]

    # ---------- TF変換 ----------
    def to_odom(self, x, y, frame):
        """frame座標系の点をodom座標系へ変換"""
        try:
            t = self.tf_buffer.lookup_transform(
                'odom', frame, Time(), timeout=Duration(seconds=0.2))
        except tf2_ros.TransformException:
            return None
        yaw = yaw_from_quat(t.transform.rotation)
        tx, ty = t.transform.translation.x, t.transform.translation.y
        return (tx + x * math.cos(yaw) - y * math.sin(yaw),
                ty + x * math.sin(yaw) + y * math.cos(yaw))

    def robot_pos(self):
        try:
            t = self.tf_buffer.lookup_transform(
                'odom', 'base_footprint', Time(), timeout=Duration(seconds=0.2))
        except tf2_ros.TransformException:
            return None
        return (t.transform.translation.x, t.transform.translation.y)

    # ---------- コールバック ----------
    def scan_callback(self, scan: LaserScan):
        det = self.detect_person(scan)
        if det is None:
            return
        pos = self.to_odom(det[0], det[1], scan.header.frame_id)
        if pos is None:
            return

        # トラッキング: 前回位置から大きく飛んだ検出は無視
        track_r = self.get_parameter('track_radius').value
        if self.person_odom is not None:
            if math.hypot(pos[0] - self.person_odom[0],
                          pos[1] - self.person_odom[1]) > track_r:
                return
        self.person_odom = pos
        self.last_detect_time = self.get_clock().now()

    # ---------- ゴール送信 ----------
    def update_goal(self):
        if self.person_odom is None:
            return
        if not self.nav_client.server_is_ready():
            return

        now = self.get_clock().now()

        # 一定時間検出がなければ見失ったとみなしリセット(誤トラッキング対策)
        timeout = self.get_parameter('person_timeout').value
        if (self.last_detect_time is not None and
                (now - self.last_detect_time).nanoseconds * 1e-9 > timeout):
            self.get_logger().warn('人を見失いました。再探索します')
            self.person_odom = None
            return
        period = self.get_parameter('goal_update_period').value
        if (self.last_goal_time is not None and
                (now - self.last_goal_time).nanoseconds * 1e-9 < period):
            return

        # 人が前回ゴール時点から十分動いていなければ更新しない
        upd = self.get_parameter('goal_update_dist').value
        px, py = self.person_odom
        if (self.last_goal_pos is not None and
                math.hypot(px - self.last_goal_pos[0],
                           py - self.last_goal_pos[1]) < upd):
            return

        robot = self.robot_pos()
        if robot is None:
            return

        # ゴール = 人からロボット方向にstandoffだけ手前、向きは人の方
        standoff = self.get_parameter('standoff').value
        dx, dy = px - robot[0], py - robot[1]
        dist = math.hypot(dx, dy)
        if dist < standoff * 0.9:
            return  # すでに十分近い(ゴールがロボットの後ろになるのを防ぐ)
        gx = px - dx / dist * standoff
        gy = py - dy / dist * standoff
        yaw = math.atan2(dy, dx)

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'odom'
        goal.pose.header.stamp = now.to_msg()
        goal.pose.pose.position.x = gx
        goal.pose.pose.position.y = gy
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)

        self.nav_client.send_goal_async(goal)  # 新ゴールで前のゴールは自動置換
        self.last_goal_pos = (px, py)
        self.last_goal_time = now
        self.get_logger().info(
            f'ゴール更新: ({gx:.2f}, {gy:.2f}) 人まで{dist:.2f}m')


def main(args=None):
    rclpy.init(args=args)
    node = NavFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
