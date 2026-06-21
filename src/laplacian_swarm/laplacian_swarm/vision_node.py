#!/usr/bin/env python3
"""Per-drone vision (runs inside /uavN). Subscribes the bridged downward camera,
decodes QR codes (pyzbar) and detects red/blue zones (HSV), publishing
QrDetection and ColorDetection on the namespaced vision/* topics."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
try:
    from pyzbar.pyzbar import decode as qr_decode
    HAVE_ZBAR = True
except Exception:
    HAVE_ZBAR = False

from laplacian_interfaces.msg import QrDetection, ColorDetection


class Vision(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.declare_parameter('drone_id', 0)
        self.declare_parameter('image_topic', 'down_cam/image')
        self.declare_parameter('process_every', 3)        # decode every Nth frame
        self.id = int(self.get_parameter('drone_id').value)
        self.every = max(1, int(self.get_parameter('process_every').value))
        self._n = 0
        topic = self.get_parameter('image_topic').value
        self.bridge = CvBridge()
        self.create_subscription(Image, topic, self.cb, 10)
        self.p_qr = self.create_publisher(QrDetection, 'vision/qr', 10)
        self.p_color = self.create_publisher(ColorDetection, 'vision/color', 10)
        if not HAVE_ZBAR:
            self.get_logger().warn('pyzbar missing: QR decode OFF. '
                                   'sudo apt install libzbar0 ; pip install pyzbar')
        self.get_logger().info(f'vision uav{self.id} on {topic}')

    def cb(self, msg):
        self._n += 1
        if self._n % self.every:
            return
        try:
            img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            return
        h, w = img.shape[:2]

        # ---- QR ----
        q = QrDetection(); q.header = msg.header; q.drone_id = self.id
        q.valid = False; q.payload = ''; q.offset_x = 0.0; q.offset_y = 0.0
        if HAVE_ZBAR:
            for d in qr_decode(img):
                try:
                    q.payload = d.data.decode('utf-8'); q.valid = True
                    rx, ry, rw, rh = d.rect
                    cx, cy = rx + rw / 2.0, ry + rh / 2.0
                    q.offset_x = float((cx - w / 2.0) / (w / 2.0))
                    q.offset_y = float((h / 2.0 - cy) / (h / 2.0))
                    break
                except Exception:
                    pass
        self.p_qr.publish(q)

        # ---- red / blue zones ----
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        for name, mask in (('KIRMIZI', self._red(hsv)), ('MAVI', self._blue(hsv))):
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            c = ColorDetection(); c.header = msg.header; c.drone_id = self.id; c.color = name
            c.detected = False; c.offset_x = 0.0; c.offset_y = 0.0; c.area_frac = 0.0
            if cnts:
                big = max(cnts, key=cv2.contourArea)
                a = cv2.contourArea(big)
                if a > 0.001 * w * h:
                    m = cv2.moments(big)
                    if m['m00'] > 0:
                        cx, cy = m['m10'] / m['m00'], m['m01'] / m['m00']
                        c.detected = True
                        c.offset_x = float((cx - w / 2.0) / (w / 2.0))
                        c.offset_y = float((h / 2.0 - cy) / (h / 2.0))
                        c.area_frac = float(a / (w * h))
            self.p_color.publish(c)

    @staticmethod
    def _red(hsv):
        return cv2.inRange(hsv, (0, 120, 70), (10, 255, 255)) | \
               cv2.inRange(hsv, (170, 120, 70), (180, 255, 255))

    @staticmethod
    def _blue(hsv):
        return cv2.inRange(hsv, (100, 120, 70), (130, 255, 255))


def main(args=None):
    rclpy.init(args=args)
    node = Vision()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()