#!/usr/bin/env python3
"""YOLOによる人検出で追従するノード。

アルゴリズム:
  1. RGB画像に対しYOLO(ultralytics)で人(COCOクラス0)を検出
  2. 最も近い(深度が小さい)人のバウンディングボックスを選択
  3. ボックス中心の画素位置 → 方位角(水平FOVから換算)
  4. 深度画像のボックス中央領域の中央値 → 距離
  5. P制御で cmd_vel を出力。一定時間見失ったら停止

初回起動時にモデル(yolov8n.pt、約6MB)を自動ダウンロードする。
"""
import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


class YoloFollowerNode(Node):

    def __init__(self):
        super().__init__('yolo_follower')

        # 制御パラメータ(他方式と共通の考え方)
        self.declare_parameter('target_distance', 1.2)
        self.declare_parameter('stop_distance', 0.8)
        self.declare_parameter('max_linear', 0.7)
        self.declare_parameter('max_angular', 1.2)
        self.declare_parameter('k_linear', 0.8)
        self.declare_parameter('k_angular', 1.8)
        self.declare_parameter('lost_timeout', 1.5)
        self.declare_parameter('hfov', 1.36)         # 水平FOV [rad]
        # YOLOパラメータ
        self.declare_parameter('model', 'yolov8n.pt')
        self.declare_parameter('confidence', 0.4)
        self.declare_parameter('imgsz', 320)         # 推論解像度(小さいほど高速)

        self.get_logger().info('YOLOモデルを読み込み中...')
        from ultralytics import YOLO  # import に時間がかかるためここで
        self.model = YOLO(self.get_parameter('model').value)
        self.get_logger().info('YOLOモデル読み込み完了')

        self.depth_image = None
        self.last_seen_time = None
        self.busy = False  # 推論中は新しいフレームをスキップ

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.rgb_sub = self.create_subscription(
            Image, '/camera/image_raw', self.rgb_callback, 1)
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_callback, 1)
        self.timer = self.create_timer(0.1, self.watchdog)

        self.get_logger().info('YOLO追従ノード起動')

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

            # 各検出の距離を求め、最も近い人を選択
            best = None  # (dist, cx)
            for xyxy in boxes.xyxy.cpu().numpy():
                x0, y0, x1, y1 = xyxy
                dist = self.bbox_depth(x0, y0, x1, y1, msg.width, msg.height)
                if dist is None:
                    continue
                if best is None or dist < best[0]:
                    best = (dist, (x0 + x1) / 2)
            if best is None:
                return

            dist, cx = best
            hfov = self.get_parameter('hfov').value
            bearing = (0.5 - cx / msg.width) * hfov

            self.last_seen_time = self.get_clock().now()
            self.publish_cmd(dist, bearing)
        finally:
            self.busy = False

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

    def watchdog(self):
        if self.last_seen_time is None:
            return
        timeout = self.get_parameter('lost_timeout').value
        elapsed = (self.get_clock().now() - self.last_seen_time).nanoseconds * 1e-9
        if elapsed > timeout:
            self.cmd_pub.publish(Twist())  # 停止


def main(args=None):
    rclpy.init(args=args)
    node = YoloFollowerNode()
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
