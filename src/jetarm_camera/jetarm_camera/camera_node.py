#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""腕部相机节点：/dev/video20 → /camera/image_raw/compressed。

相机本身出 MJPG，这里用 V4L2 的 CONVERT_RGB=0 直接拿压缩帧，
不解码、不重编码，带宽约 50KB/帧（1280x720，5~10fps）。
需要原始图时把 publish_raw 设 true。

话题：
    /camera/image_raw/compressed   sensor_msgs/CompressedImage（format=jpeg）
    /camera/image_raw              sensor_msgs/Image（可选，默认关）
    /camera/camera_info            sensor_msgs/CameraInfo（TRANSIENT_LOCAL）

内参来自 handeye.json（1280x720，2026-10-28 标定）。
"""

import json
import os
import threading
import time

import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy, qos_profile_sensor_data)
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

DEFAULT_DEVICE = ('/dev/v4l/by-id/'
                  'usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_'
                  'AY25B5F0034-video-index0')


class CameraNode(Node):

    def __init__(self):
        super().__init__('camera_node')

        share = get_package_share_directory('jetarm_camera')
        self.declare_parameter('device', DEFAULT_DEVICE)
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('fps', 10)
        self.declare_parameter('frame_id', 'wrist_camera')
        self.declare_parameter('camera_info_file',
                               os.path.join(share, 'config', 'handeye.json'))
        self.declare_parameter('publish_raw', False)
        self.declare_parameter('reconnect_period_s', 2.0)

        self.device = str(self.get_parameter('device').value)
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.fps = int(self.get_parameter('fps').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.publish_raw = bool(self.get_parameter('publish_raw').value)
        self.reconnect_period = float(
            self.get_parameter('reconnect_period_s').value)

        self.info = self._load_camera_info(
            str(self.get_parameter('camera_info_file').value))

        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        info_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.pub_compressed = self.create_publisher(
            CompressedImage, '/camera/image_raw/compressed', image_qos)
        self.pub_raw = self.create_publisher(Image, '/camera/image_raw',
                                             image_qos)
        self.pub_info = self.create_publisher(CameraInfo, '/camera/camera_info',
                                              info_qos)

        self.cap = None
        self.capture = None
        self._last_sub = time.monotonic()
        self.frames = 0
        self.drops = 0
        self.started = time.monotonic()
        self.error = ''
        self._stop = threading.Event()
        self.create_timer(1.0, self.publish_info)
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()
        self.get_logger().info('相机节点启动：%s' % self.device)

    # ---------- 内参 ----------
    def _load_camera_info(self, path):
        info = CameraInfo()
        info.header.frame_id = self.frame_id
        info.width = self.width
        info.height = self.height
        try:
            with open(path, encoding='utf-8') as f:
                doc = json.load(f)
            k = doc['camera_matrix']
            info.distortion_model = 'plumb_bob'
            info.d = [float(v) for v in doc['dist_coeffs']]
            info.k = [float(k[0][0]), 0.0, float(k[0][2]),
                      0.0, float(k[1][1]), float(k[1][2]),
                      0.0, 0.0, 1.0]
            info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            info.p = [float(k[0][0]), 0.0, float(k[0][2]), 0.0,
                      0.0, float(k[1][1]), float(k[1][2]), 0.0,
                      0.0, 0.0, 1.0, 0.0]
            size = doc.get('image_size')
            if size:
                info.width, info.height = int(size[0]), int(size[1])
            self.get_logger().info('相机内参已加载：%s（%dx%d）'
                                   % (path, info.width, info.height))
        except Exception as exc:  # noqa: BLE001
            self.error = 'camera_info: %s' % exc
            self.get_logger().error('读取内参失败：%s' % exc)
        return info

    def publish_info(self):
        if self.info is None:
            return
        self.info.header.stamp = self.get_clock().now().to_msg()
        self.pub_info.publish(self.info)

    # ---------- 采集 ----------
    def _open(self):
        if self.capture is not None:
            return True
        cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            self.error = 'camera_open_failed'
            return False
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        # 关键：拿到未解码的 MJPG 字节，直接转发
        cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
        self.capture = cap
        self.error = ''
        self.get_logger().info('相机已打开：%s（MJPG 直通）' % self.device)
        return True

    def _idle(self):
        """没有订阅者就关闭设备休眠，省 CPU；有人订阅自动再开。"""
        subs = self.pub_compressed.get_subscription_count()
        if self.publish_raw:
            subs += self.pub_raw.get_subscription_count()
        now = time.monotonic()
        if subs > 0:
            self._last_sub = now
            return False
        if now - self._last_sub < 2.0:
            return False
        if self.capture is not None:
            self.capture.release()
            self.capture = None
            self.get_logger().info('无订阅者，相机休眠')
        return True

    def _loop(self):
        while not self._stop.is_set():
            if self._idle():
                self._stop.wait(0.5)
                continue
            if not self._open():
                self._stop.wait(self.reconnect_period)
                continue
            ok, frame = self.capture.read()
            if not ok or frame is None:
                self.drops += 1
                if self.drops % 20 == 1:
                    self.get_logger().warn('取帧失败，累计 %d 次' % self.drops)
                time.sleep(0.1)
                if self.drops % 50 == 0:
                    self.capture.release()
                    self.capture = None
                continue
            self.frames += 1
            self._publish(frame)
            elapsed = time.monotonic() - self.started
            if elapsed >= 10.0:
                fps = self.frames / elapsed
                if fps < self.fps * 0.5:
                    self.get_logger().warning('实际帧率 %.1f fps（请求 %d）'
                                              % (fps, self.fps))
                else:
                    self.get_logger().info('实际帧率 %.1f fps' % fps)
                self.started = time.monotonic()
                self.frames = 0

    def _publish(self, frame):
        stamp = self.get_clock().now().to_msg()
        if frame.ndim >= 2 and (frame.ndim == 2 or frame.shape[2] == 1):
            data = frame.tobytes()
            fmt = 'jpeg'
        else:
            ok, buf = cv2.imencode('.jpg', frame,
                                   [int(cv2.IMWRITE_JPEG_QUALITY), 90])
            if not ok:
                return
            data = buf.tobytes()
            fmt = 'jpeg'

        msg = CompressedImage()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.format = fmt
        msg.data = data
        self.pub_compressed.publish(msg)

        if self.publish_raw and frame.ndim == 3:
            raw = Image()
            raw.header.stamp = stamp
            raw.header.frame_id = self.frame_id
            raw.height, raw.width = frame.shape[:2]
            raw.encoding = 'bgr8'
            raw.step = frame.shape[1] * 3
            raw.data = frame.tobytes()
            self.pub_raw.publish(raw)

    def shutdown(self):
        self._stop.set()
        if self.thread.is_alive():
            self.thread.join(timeout=1.0)
        if self.capture is not None:
            self.capture.release()
            self.capture = None


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()