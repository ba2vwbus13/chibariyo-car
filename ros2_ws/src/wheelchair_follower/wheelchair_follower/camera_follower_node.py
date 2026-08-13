#!/usr/bin/env python3
"""RGB-Dカメラで人を検出して追従するノード。

アルゴリズム:
  1. RGB画像をHSV変換し、人モデルの色(オレンジ)をマスク抽出
  2. 最大輪郭の重心を求める
  3. 重心の画素位置 → 方位角(水平FOVから換算)
  4. 深度画像の重心周辺の中央値 → 距離
  5. P制御で cmd_vel を出力。一定時間見失ったら停止

※ 実機ではこの色検出部をYOLO等の人検出器に置き換える想定。
"""
import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


class CameraFollowerNode(Node):

    def __init__(self):
        super().__init__('camera_follower')

        # 制御パラメータ(LiDAR版と共通の考え方)
        self.declare_parameter('target_distance', 1.2)
        self.declare_parameter('stop_distance', 0.8)
        self.declare_parameter('max_linear', 0.7)
        self.declare_parameter('max_angular', 1.2)
        self.declare_parameter('k_linear', 0.8)
        self.declare_parameter('k_angular', 1.8)
        self.declare_parameter('lost_timeout', 1.0)
        # カメラパラメータ
        self.declare_parameter('hfov', 1.36)             # 水平FOV [rad]
        # 色検出パラメータ(HSV、OpenCV基準 H:0-179)
        self.declare_parameter('hsv_lower', [5, 100, 60])    # オレンジ下限
        self.declare_parameter('hsv_upper', [25, 255, 255])  # オレンジ上限
        self.declare_parameter('min_area', 300)          # 最小検出面積 [px]

        self.depth_image = None
        self.last_seen_time = None

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.rgb_sub = self.create_subscription(
            Image, '/camera/image_raw', self.rgb_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_callback, 10)
        self.timer = self.create_timer(0.1, self.watchdog)

        self.get_logger().info('カメラ追従ノード起動')

    # ---------- 画像変換(cv_bridge非依存) ----------
    @staticmethod
    def image_to_bgr(msg: Image):
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        img = buf.reshape(msg.height, msg.width, -1)
        if msg.encoding in ('rgb8',):
            return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        if msg.encoding in ('bgr8',):
            return img
        return None

    @staticmethod
    def image_to_depth(msg: Image):
        """深度画像を [m] 単位のfloat配列に変換"""
        if msg.encoding == '32FC1':
            return np.frombuffer(msg.data, dtype=np.float32).reshape(
                msg.height, msg.width)
        if msg.encoding == '16UC1':
            raw = np.frombuffer(msg.data, dtype=np.uint16).reshape(
                msg.height, msg.width)
            return raw.astype(np.float32) * 0.001  # mm → m
        return None

    # ---------- コールバック ----------
    def depth_callback(self, msg: Image):
        self.depth_image = self.image_to_depth(msg)

    def rgb_callback(self, msg: Image):
        bgr = self.image_to_bgr(msg)
        if bgr is None or self.depth_image is None:
            return
        if self.depth_image.shape[:2] != (msg.height, msg.width):
            return

        # 1. 色マスク
        lower = np.array(self.get_parameter('hsv_lower').value, dtype=np.uint8)
        upper = np.array(self.get_parameter('hsv_upper').value, dtype=np.uint8)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                                np.ones((5, 5), np.uint8))

        # 2. 最大輪郭
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < self.get_parameter('min_area').value:
            return
        m = cv2.moments(largest)
        if m['m00'] == 0:
            return
        cx = m['m10'] / m['m00']
        cy = m['m01'] / m['m00']

        # 3. 方位角(画像中心からのずれ → 角度)
        hfov = self.get_parameter('hfov').value
        bearing = (0.5 - cx / msg.width) * hfov

        # 4. 距離(重心周辺11x11pxの深度中央値)
        r = 5
        y0, y1 = max(0, int(cy) - r), min(msg.height, int(cy) + r + 1)
        x0, x1 = max(0, int(cx) - r), min(msg.width, int(cx) + r + 1)
        patch = self.depth_image[y0:y1, x0:x1]
        valid = patch[np.isfinite(patch) & (patch > 0.05)]
        if valid.size == 0:
            return
        dist = float(np.median(valid))

        self.last_seen_time = self.get_clock().now()
        self.publish_cmd(dist, bearing)

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
    node = CameraFollowerNode()
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
