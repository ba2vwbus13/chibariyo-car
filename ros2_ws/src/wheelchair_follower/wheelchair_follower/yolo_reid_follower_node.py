#!/usr/bin/env python3
"""YOLO + 色ヒストグラムRe-IDで「利用者本人」を追従するノード。

yolo_follower(最も近い人を追う)との違い:
  - ロックオン時に対象の服の色ヒストグラム(HSV)を記憶する
  - 以降は「色の類似度 + 位置の連続性」で本人を選ぶ(近いだけの別人は追わない)
  - 見失っても記憶は保持し、再登場した本人を色で再発見(Re-ID)する
  - 追従状態を /follower/status (JSON文字列)で配信し、metrics_loggerが記録する

アルゴリズム:
  1. RGB画像に対しYOLO(ultralytics)で人(COCOクラス0)を検出
  2. 各候補について 距離(深度中央値)・方位角・上半身の色ヒストグラム を計算
  3. 未ロックなら: 最も近い候補(lock_max_dist以内)を本人としてロックオン
  4. ロック済みなら: score = w_hist×色類似度 + w_pos×位置連続性 が最大の候補を選ぶ
     (score が accept_threshold 未満なら「本人不在」として見失い扱い)
  5. P制御で cmd_vel を出力。lost_timeout の間見失ったら停止(記憶は保持)
"""
import json
import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class YoloReidFollowerNode(Node):

    def __init__(self):
        super().__init__('yolo_reid_follower')

        # 制御パラメータ(他方式と共通の考え方)
        self.declare_parameter('target_distance', 1.2)
        self.declare_parameter('stop_distance', 0.8)
        self.declare_parameter('max_linear', 0.7)
        self.declare_parameter('max_angular', 1.2)
        self.declare_parameter('k_linear', 0.8)
        self.declare_parameter('k_angular', 1.8)
        self.declare_parameter('lost_timeout', 1.5)
        self.declare_parameter('hfov', 1.36)          # 水平FOV [rad]
        # YOLOパラメータ
        self.declare_parameter('model', 'yolov8n.pt')
        self.declare_parameter('confidence', 0.4)
        self.declare_parameter('imgsz', 320)
        # Re-IDパラメータ
        self.declare_parameter('lock_max_dist', 4.0)   # ロックオン許容距離 [m]
        self.declare_parameter('w_hist', 0.6)          # 色類似度の重み
        self.declare_parameter('w_pos', 0.4)           # 位置連続性の重み
        self.declare_parameter('accept_threshold', 0.45)  # 本人と認める最小score
        self.declare_parameter('hist_min_sim', 0.25)   # 色類似度の下限(これ未満は別人)
        self.declare_parameter('hist_update_sim', 0.6) # これ以上似ていたらヒストグラム更新
        self.declare_parameter('hist_ema', 0.9)        # ヒストグラム更新の慣性(旧:新=0.9:0.1)
        self.declare_parameter('pos_angle_sigma', 0.35)  # 位置連続性: 方位のゆるさ [rad]
        self.declare_parameter('pos_dist_sigma', 0.8)    # 位置連続性: 距離のゆるさ [m]

        self.get_logger().info('YOLOモデルを読み込み中...')
        from ultralytics import YOLO  # import に時間がかかるためここで
        self.model = YOLO(self.get_parameter('model').value)
        self.get_logger().info('YOLOモデル読み込み完了')

        self.depth_image = None
        self.busy = False           # 推論中は新しいフレームをスキップ
        self.target_hist = None     # 本人の色ヒストグラム(ロックオン後に保持)
        self.last_seen_time = None  # 最後に本人を見た時刻
        self.last_dist = None       # 最後に見た距離 [m]
        self.last_bearing = None    # 最後に見た方位 [rad]

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/follower/status', 10)
        self.rgb_sub = self.create_subscription(
            Image, '/camera/image_raw', self.rgb_callback, 1)
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_callback, 1)
        self.timer = self.create_timer(0.1, self.watchdog)

        self.get_logger().info('YOLO Re-ID追従ノード起動')

    # ---------- 画像変換(cv_bridge非依存) ----------
    @staticmethod
    def image_to_bgr(msg: Image):
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        img = buf.reshape(msg.height, msg.width, -1)
        if msg.encoding == 'rgb8':
            return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        if msg.encoding == 'bgr8':
            return img.copy()
        return None

    @staticmethod
    def image_to_depth(msg: Image):
        if msg.encoding == '32FC1':
            return np.frombuffer(msg.data, dtype=np.float32).reshape(
                msg.height, msg.width)
        if msg.encoding == '16UC1':
            raw = np.frombuffer(msg.data, dtype=np.uint16).reshape(
                msg.height, msg.width)
            return raw.astype(np.float32) * 0.001
        return None

    # ---------- 色ヒストグラム ----------
    @staticmethod
    def bbox_histogram(bgr, x0, y0, x1, y1):
        """ボックス上半身中央領域のHSVヒストグラム(H30×S32ビン、正規化済み)"""
        h_img, w_img = bgr.shape[:2]
        # 上半身: 縦は上から15%〜55%、横は中央60%
        bw = x1 - x0
        bh = y1 - y0
        rx0 = int(max(0, x0 + 0.2 * bw))
        rx1 = int(min(w_img, x1 - 0.2 * bw))
        ry0 = int(max(0, y0 + 0.15 * bh))
        ry1 = int(min(h_img, y0 + 0.55 * bh))
        if rx1 - rx0 < 4 or ry1 - ry0 < 4:
            return None
        patch = bgr[ry0:ry1, rx0:rx1]
        hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
        hist = cv2.calcHist([hsv], [0, 1], None, [30, 32],
                            [0, 180, 0, 256])
        cv2.normalize(hist, hist, alpha=1.0, norm_type=cv2.NORM_L1)
        return hist

    @staticmethod
    def hist_similarity(h1, h2):
        """バタチャリヤ距離に基づく類似度(1=同一, 0=全く別)"""
        d = cv2.compareHist(h1, h2, cv2.HISTCMP_BHATTACHARYYA)
        return max(0.0, 1.0 - float(d))

    def pos_score(self, dist, bearing, t_lost):
        """位置連続性スコア。見失い時間が長いほどゲートを緩める"""
        if self.last_dist is None:
            return 0.0
        relax = 1.0 + min(3.0, t_lost)  # 最大4倍まで緩和
        s_ang = self.get_parameter('pos_angle_sigma').value * relax
        s_dst = self.get_parameter('pos_dist_sigma').value * relax
        da = (bearing - self.last_bearing) / s_ang
        dd = (dist - self.last_dist) / s_dst
        return math.exp(-(da * da + dd * dd))

    # ---------- コールバック ----------
    def depth_callback(self, msg: Image):
        self.depth_image = self.image_to_depth(msg)

    def rgb_callback(self, msg: Image):
        if self.busy or self.depth_image is None:
            return
        bgr = self.image_to_bgr(msg)
        if bgr is None:
            return
        if self.depth_image.shape[:2] != (msg.height, msg.width):
            return

        self.busy = True
        try:
            results = self.model.predict(
                bgr,
                classes=[0],  # person のみ
                conf=self.get_parameter('confidence').value,
                imgsz=self.get_parameter('imgsz').value,
                verbose=False,
            )
            boxes = results[0].boxes
            if boxes is None or len(boxes) == 0:
                return

            hfov = self.get_parameter('hfov').value
            candidates = []  # (dist, bearing, hist)
            for xyxy in boxes.xyxy.cpu().numpy():
                x0, y0, x1, y1 = xyxy
                dist = self.bbox_depth(x0, y0, x1, y1, msg.width, msg.height)
                if dist is None:
                    continue
                hist = self.bbox_histogram(bgr, x0, y0, x1, y1)
                if hist is None:
                    continue
                bearing = (0.5 - (x0 + x1) / 2 / msg.width) * hfov
                candidates.append((dist, bearing, hist))
            if not candidates:
                return

            if self.target_hist is None:
                self.lock_on(candidates)
                return

            self.track(candidates)
        finally:
            self.busy = False

    def lock_on(self, candidates):
        """未ロック時: 最も近い候補を本人として記憶"""
        dist, bearing, hist = min(candidates, key=lambda c: c[0])
        if dist > self.get_parameter('lock_max_dist').value:
            return
        self.target_hist = hist
        self.mark_seen(dist, bearing, sim=1.0)
        self.get_logger().info(f'ロックオン: 距離{dist:.2f}m の人物を本人として記憶')

    def track(self, candidates):
        """ロック済み: 色類似度+位置連続性で本人を選ぶ"""
        now = self.get_clock().now()
        t_lost = 0.0
        if self.last_seen_time is not None:
            t_lost = (now - self.last_seen_time).nanoseconds * 1e-9

        w_h = self.get_parameter('w_hist').value
        w_p = self.get_parameter('w_pos').value
        best = None  # (score, sim, dist, bearing, hist)
        for dist, bearing, hist in candidates:
            sim = self.hist_similarity(self.target_hist, hist)
            if sim < self.get_parameter('hist_min_sim').value:
                continue  # 色が違いすぎる → 別人
            score = w_h * sim + w_p * self.pos_score(dist, bearing, t_lost)
            if best is None or score > best[0]:
                best = (score, sim, dist, bearing, hist)

        if best is None or best[0] < self.get_parameter('accept_threshold').value:
            return  # 本人候補なし(見失い継続。watchdogが停止を担当)

        score, sim, dist, bearing, hist = best
        # 十分似ているときだけ記憶を少しずつ更新(照明変化に追随、別人には染まらない)
        if sim > self.get_parameter('hist_update_sim').value:
            ema = self.get_parameter('hist_ema').value
            self.target_hist = ema * self.target_hist + (1.0 - ema) * hist
            cv2.normalize(self.target_hist, self.target_hist,
                          alpha=1.0, norm_type=cv2.NORM_L1)
        self.mark_seen(dist, bearing, sim)

    def mark_seen(self, dist, bearing, sim):
        self.last_seen_time = self.get_clock().now()
        self.last_dist = dist
        self.last_bearing = bearing
        self.publish_cmd(dist, bearing)
        self.publish_status('tracking', dist, bearing, sim)

    def bbox_depth(self, x0, y0, x1, y1, w, h):
        """ボックス中央1/3領域の深度中央値 [m]"""
        cx0 = int(max(0, x0 + (x1 - x0) / 3))
        cx1 = int(min(w, x1 - (x1 - x0) / 3))
        cy0 = int(max(0, y0 + (y1 - y0) / 3))
        cy1 = int(min(h, y1 - (y1 - y0) / 3))
        if cx1 <= cx0 or cy1 <= cy0:
            return None
        patch = self.depth_image[cy0:cy1, cx0:cx1]
        valid = patch[np.isfinite(patch) & (patch > 0.05)]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    # ---------- 制御 ----------
    def publish_cmd(self, dist, bearing):
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
            v *= max(0.0, math.cos(bearing))
            cmd.linear.x = max(0.0, min(max_lin, v))
        self.cmd_pub.publish(cmd)

    def publish_status(self, state, dist=None, bearing=None, sim=None):
        msg = String()
        msg.data = json.dumps({
            'state': state,
            'distance': dist,
            'bearing': bearing,
            'similarity': sim,
        })
        self.status_pub.publish(msg)

    def watchdog(self):
        if self.last_seen_time is None:
            return
        timeout = self.get_parameter('lost_timeout').value
        elapsed = (self.get_clock().now() - self.last_seen_time).nanoseconds * 1e-9
        if elapsed > timeout:
            self.cmd_pub.publish(Twist())  # 停止(記憶は保持し再発見を待つ)
            self.publish_status('lost')


def main(args=None):
    rclpy.init(args=args)
    node = YoloReidFollowerNode()
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
