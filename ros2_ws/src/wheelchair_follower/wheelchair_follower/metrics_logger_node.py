#!/usr/bin/env python3
"""追尾性能の定量評価ノード。

シミュレーションの真値(人・車椅子のodom)と追従ノードの状態(/follower/status)から、
SCORE!等の書類・発表に使える数値を自動計測する:

  - ID維持率       : 追従中、正しく「本人」を追っていた時間の割合 [%]
  - ID取り違え回数 : 本人→別人(person2)に乗り移った回数
  - 見失い率       : 全時間に占める見失い状態の割合 [%]
  - 平均再発見時間 : 見失ってから本人を再発見するまでの平均秒数
  - 距離維持誤差   : 本人追従中の |実距離 - 目標距離| の平均・標準偏差 [m]

出力: out_dir(既定 ~/ros2_ws/metrics)に
  - metrics_<開始時刻>.csv     : 0.1秒ごとの生データ
  - summary_<開始時刻>.txt     : 上記指標のまとめ(終了時とCtrl+C時に書き出し)

/follower/status は yolo_reid_follower が配信する。LiDAR方式・YOLO方式で
比較測定する場合、statusが無くても真値ベースの指標(距離・最接近など)は記録される。
"""
import csv
import json
import math
import os
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class MetricsLogger(Node):

    def __init__(self):
        super().__init__('metrics_logger')

        self.declare_parameter('target_distance', 1.2)   # 目標距離 [m]
        self.declare_parameter('robot_world_offset', [-1.0, 0.0])  # 車椅子odom原点のworld座標
        self.declare_parameter('id_gate', 1.2)     # 推定位置と真値の対応付け許容 [m]
        self.declare_parameter('status_fresh', 0.5)  # statusをこの秒数まで有効とみなす
        self.declare_parameter('out_dir', '~/ros2_ws/metrics')
        self.declare_parameter('label', 'reid')    # ファイル名に入れる実験ラベル

        self.person = None    # 本人 (x, y) world
        self.person2 = None   # 別人 (x, y) world
        self.robot = None     # 車椅子 (x, y, yaw) world
        self.status = None    # 直近の /follower/status (dict)
        self.status_stamp = None

        self.rows = []
        self.followed_prev = None   # 'person' / 'person2' / None
        self.id_switches = 0
        self.lost_since = None      # 見失い開始時刻(秒)
        self.reacq_times = []       # 再発見までの秒数
        self.lost_total = 0.0
        self.correct_total = 0.0
        self.wrong_total = 0.0
        self.dist_errs = []
        self.t_prev = None
        self.t0 = None

        stamp = time.strftime('%Y%m%d_%H%M%S')
        label = self.get_parameter('label').value
        out_dir = os.path.expanduser(self.get_parameter('out_dir').value)
        os.makedirs(out_dir, exist_ok=True)
        self.csv_path = os.path.join(out_dir, f'metrics_{label}_{stamp}.csv')
        self.summary_path = os.path.join(out_dir, f'summary_{label}_{stamp}.txt')
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(
            ['t', 'robot_x', 'robot_y', 'person_x', 'person_y',
             'person2_x', 'person2_y', 'dist_to_person', 'dist_to_person2',
             'state', 'followed', 'est_x', 'est_y'])

        self.create_subscription(Odometry, '/person/odom', self.person_cb, 10)
        self.create_subscription(Odometry, '/person2/odom', self.person2_cb, 10)
        self.create_subscription(Odometry, '/odom', self.robot_cb, 10)
        self.create_subscription(String, '/follower/status', self.status_cb, 10)
        self.create_timer(0.1, self.sample)
        self.create_timer(10.0, self.report)

        self.get_logger().info(f'計測開始: {self.csv_path}')

    # ---------- コールバック ----------
    def person_cb(self, msg):
        p = msg.pose.pose.position
        self.person = (p.x, p.y)

    def person2_cb(self, msg):
        p = msg.pose.pose.position
        self.person2 = (p.x, p.y)

    def robot_cb(self, msg):
        off = self.get_parameter('robot_world_offset').value
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.robot = (p.x + off[0], p.y + off[1], yaw)

    def status_cb(self, msg):
        try:
            self.status = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.status_stamp = self.now_sec()

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ---------- 0.1秒ごとの記録 ----------
    def sample(self):
        if self.person is None or self.robot is None:
            return
        t = self.now_sec()
        if self.t0 is None:
            self.t0 = t
            self.t_prev = t
            return
        dt = t - self.t_prev
        self.t_prev = t
        if dt <= 0.0 or dt > 1.0:  # 一時停止などの異常間隔は捨てる
            return

        rx, ry, ryaw = self.robot
        px, py = self.person
        d_person = math.hypot(px - rx, py - ry)
        d_person2 = None
        if self.person2 is not None:
            d_person2 = math.hypot(self.person2[0] - rx, self.person2[1] - ry)

        # 追従ノードの推定位置をworld座標に変換し、どちらを追っているか判定
        state = 'none'
        followed = None
        est = (None, None)
        fresh = self.get_parameter('status_fresh').value
        if self.status is not None and self.status_stamp is not None \
                and t - self.status_stamp < fresh:
            state = self.status.get('state', 'none')
            if state == 'tracking' and self.status.get('distance') is not None:
                d = self.status['distance']
                b = self.status['bearing']
                ex = rx + d * math.cos(ryaw + b)
                ey = ry + d * math.sin(ryaw + b)
                est = (ex, ey)
                gate = self.get_parameter('id_gate').value
                dp = math.hypot(ex - px, ey - py)
                d2 = (math.hypot(ex - self.person2[0], ey - self.person2[1])
                      if self.person2 is not None else float('inf'))
                if min(dp, d2) < gate:
                    followed = 'person' if dp <= d2 else 'person2'

        # 集計
        if followed == 'person':
            self.correct_total += dt
            self.dist_errs.append(
                abs(d_person - self.get_parameter('target_distance').value))
            if self.lost_since is not None:
                self.reacq_times.append(t - self.lost_since)
                self.lost_since = None
        elif followed == 'person2':
            self.wrong_total += dt
            if self.followed_prev == 'person':
                self.id_switches += 1
        else:
            self.lost_total += dt
            if self.lost_since is None:
                self.lost_since = t
        if followed is not None:
            self.followed_prev = followed

        self.writer.writerow(
            [f'{t - self.t0:.2f}', f'{rx:.3f}', f'{ry:.3f}',
             f'{px:.3f}', f'{py:.3f}',
             f'{self.person2[0]:.3f}' if self.person2 else '',
             f'{self.person2[1]:.3f}' if self.person2 else '',
             f'{d_person:.3f}',
             f'{d_person2:.3f}' if d_person2 is not None else '',
             state, followed or '',
             f'{est[0]:.3f}' if est[0] is not None else '',
             f'{est[1]:.3f}' if est[1] is not None else ''])

    # ---------- まとめ ----------
    def summary_text(self):
        total = self.correct_total + self.wrong_total + self.lost_total
        if total <= 0.0:
            return '計測データなし'
        lines = []
        lines.append(f'計測時間: {total:.1f} 秒')
        lines.append(f'ID維持率(本人を追従): {100.0 * self.correct_total / total:.1f} %')
        lines.append(f'別人追従: {100.0 * self.wrong_total / total:.1f} % '
                     f'(取り違え {self.id_switches} 回)')
        lines.append(f'見失い率: {100.0 * self.lost_total / total:.1f} %')
        if self.reacq_times:
            mean_r = sum(self.reacq_times) / len(self.reacq_times)
            lines.append(f'平均再発見時間: {mean_r:.2f} 秒 '
                         f'(見失い→再発見 {len(self.reacq_times)} 回)')
        if self.dist_errs:
            n = len(self.dist_errs)
            mean_e = sum(self.dist_errs) / n
            var = sum((e - mean_e) ** 2 for e in self.dist_errs) / n
            lines.append(f'距離維持誤差(本人追従中): 平均 {mean_e:.2f} m / '
                         f'標準偏差 {math.sqrt(var):.2f} m '
                         f'(目標 {self.get_parameter("target_distance").value} m)')
        return '\n'.join(lines)

    def report(self):
        self.get_logger().info('\n===== 中間集計 =====\n' + self.summary_text())
        self.csv_file.flush()

    def finish(self):
        with open(self.summary_path, 'w') as f:
            f.write(self.summary_text() + '\n')
        self.csv_file.close()
        self.get_logger().info(
            f'まとめを書き出しました: {self.summary_path}\n' + self.summary_text())


def main(args=None):
    rclpy.init(args=args)
    node = MetricsLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.finish()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
