#!/usr/bin/env python3
"""欢创 D2 雷达驱动节点：串口 115200 8N1 -> sensor_msgs/LaserScan。

协议（实测确认）：
    帧长 28 字节，帧头 0x55 0xAA，8 点/帧
    [3]    点数 N
    [6:8]  FSA 起始角，小端，单位 1/64 度
    [8:8+2N]  采样点，小端 2 字节，距离 = 原始值/dist_divisor (mm)，0x8000 表示无效
    [8+2N:10+2N] LSA 结束角，小端，单位 1/64 度
    每帧约 5.76°，6Hz 转一圈约 500 点，按 0.72° 分箱发布。

发布 /scan（整圈，约 6Hz，供 SLAM/nav2/避障使用），
以及 /scan_fast（当前扇区，每帧一条，约 375Hz，供低延迟反应层使用），
并发布 base_link -> laser 静态 TF（默认 285mm 安装高度）。
"""

import glob
import math
import os
import termios
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

HDR = b"\x55\xaa"
ANGLE_UNIT = 1.0 / 64.0


class D2ScanNode(Node):

    def __init__(self):
        super().__init__('d2_scan')
        self.declare_parameter('port', 'auto')
        self.declare_parameter('uart_port', '/dev/ttyS10')
        self.declare_parameter('frame_id', 'laser')
        self.declare_parameter('parent_frame', 'base_link')
        self.declare_parameter('mount_height', 0.285)
        self.declare_parameter('x_offset_m', 0.0)
        self.declare_parameter('y_offset_m', 0.0)
        self.declare_parameter('yaw_offset_deg', 0.0)
        self.declare_parameter('mirror', False)
        self.declare_parameter('bin_deg', 0.72)
        # 0.30 用于过滤车体自身回波（车角离雷达约 0.20~0.25m）
        self.declare_parameter('range_min', 0.30)
        self.declare_parameter('range_max', 8.0)
        # 实测：本雷达采样值按 mm 解释（dist_divisor=1）；手册写的 /4 会整体缩小 4 倍
        self.declare_parameter('dist_divisor', 1.0)
        self.declare_parameter('scan_hz', 6.0)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('publish_fast', True)

        self.port = self.get_parameter('port').value
        self.uart_port = str(self.get_parameter('uart_port').value)
        self.auto_mode = (self.port == 'auto')
        self.frame_id = self.get_parameter('frame_id').value
        self.parent_frame = self.get_parameter('parent_frame').value
        self.mount_height = float(self.get_parameter('mount_height').value)
        self.x_offset_m = float(self.get_parameter('x_offset_m').value)
        self.y_offset_m = float(self.get_parameter('y_offset_m').value)
        self.yaw_offset_deg = float(self.get_parameter('yaw_offset_deg').value)
        self.mirror = bool(self.get_parameter('mirror').value)
        self.bin_deg = float(self.get_parameter('bin_deg').value)
        self.range_min = float(self.get_parameter('range_min').value)
        self.range_max = float(self.get_parameter('range_max').value)
        self.dist_divisor = max(0.01, float(self.get_parameter('dist_divisor').value))
        self.scan_hz = float(self.get_parameter('scan_hz').value)
        self.publish_fast = bool(self.get_parameter('publish_fast').value)

        self.bins = int(round(360.0 / self.bin_deg))
        self.ranges = [float('inf')] * self.bins
        self.last_angle = None
        self.buf = bytearray()
        self.fd = None
        self.last_open_try = 0.0
        self.last_data = 0.0
        self.pub = self.create_publisher(LaserScan, 'scan', 10)
        self.pub_fast = None
        if self.publish_fast:
            self.pub_fast = self.create_publisher(LaserScan, 'scan_fast', 10)
        self.tf_pub = None
        if bool(self.get_parameter('publish_tf').value):
            self._publish_static_tf()

        self.create_timer(0.01, self._tick)
        self.get_logger().info('D2 雷达：%s -> /scan（%d 箱，%s -> %s，位置 x=%.3f y=%.3f z=%.3f，偏角 %.1f°）'
                               % (self.port, self.bins, self.parent_frame, self.frame_id,
                                  self.x_offset_m, self.y_offset_m, self.mount_height,
                                  self.yaw_offset_deg))

    def _publish_static_tf(self):
        import tf2_ros
        yaw = math.radians(self.yaw_offset_deg)
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.parent_frame
        tf.child_frame_id = self.frame_id
        tf.transform.translation.x = self.x_offset_m
        tf.transform.translation.y = self.y_offset_m
        tf.transform.translation.z = self.mount_height
        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = 0.0
        tf.transform.rotation.z = math.sin(yaw / 2.0)
        tf.transform.rotation.w = math.cos(yaw / 2.0)
        self.tf_pub = tf2_ros.StaticTransformBroadcaster(self)
        self.tf_pub.sendTransform(tf)

    def _open_port(self, path=None):
        fd = os.open(path or self.port, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
        attrs = termios.tcgetattr(fd)
        attrs[0] &= ~(termios.IGNBRK | termios.BRKINT | termios.PARMRK | termios.ISTRIP
                      | termios.INLCR | termios.IGNCR | termios.ICRNL | termios.IXON)
        attrs[1] &= ~termios.OPOST
        attrs[2] &= ~(termios.CSIZE | termios.PARENB)
        attrs[2] |= termios.CS8 | termios.CREAD | termios.CLOCAL
        attrs[3] &= ~(termios.ECHO | termios.ECHONL | termios.ICANON | termios.ISIG
                      | termios.IEXTEN)
        attrs[4] = termios.B115200
        attrs[5] = termios.B115200
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        return fd

    def _detect_port(self):
        """自动识别雷达串口：优先板载 UART，其次 /dev/ttyUSB*。"""
        candidates = []
        if self.uart_port and os.path.exists(self.uart_port):
            candidates.append(self.uart_port)
        candidates.extend(sorted(glob.glob('/dev/ttyUSB*')))
        for path in candidates:
            try:
                fd = self._open_port(path)
            except Exception as e:
                self.get_logger().debug('跳过 %s：%s' % (path, e))
                continue
            frames = 0
            deadline = time.monotonic() + 0.6
            while time.monotonic() < deadline:
                try:
                    chunk = os.read(fd, 4096)
                except BlockingIOError:
                    chunk = b''
                if chunk:
                    frames += chunk.count(HDR)
                else:
                    time.sleep(0.01)
            if frames >= 5:
                self.get_logger().info('自动识别雷达串口：%s（0.6s 内 %d 个帧头）'
                                       % (path, frames))
                return fd, path
            os.close(fd)
        return None, None

    def _tick(self):
        if self.fd is None:
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self.last_open_try < 1.0:
                return
            self.last_open_try = now
            if self.port == 'auto':
                fd, path = self._detect_port()
                if fd is None:
                    self.get_logger().warn('未找到输出 55AA 帧的串口，1 秒后重试')
                    return
                self.fd = fd
                self.port = path
            else:
                try:
                    self.fd = self._open_port(self.port)
                    self.get_logger().info('串口已打开：%s' % self.port)
                except Exception as e:
                    self.get_logger().warn('打开 %s 失败：%s（1 秒后重试）' % (self.port, e))
                    return
            self.buf = bytearray()
            self.last_data = time.monotonic()
            return

        try:
            chunk = os.read(self.fd, 8192)
        except BlockingIOError:
            chunk = b''
        except OSError as e:
            self.get_logger().error('串口读取失败：%s，等待重连' % e)
            try:
                os.close(self.fd)
            except Exception:
                pass
            self.fd = None
            if self.auto_mode:
                self.port = 'auto'
            return
        if chunk:
            self.buf += chunk
            self.last_data = time.monotonic()
        elif time.monotonic() - self.last_data > 2.0:
            # USB 设备被拔掉或重新枚举时，旧句柄读不到任何数据，
            # 也不会报错，只能靠"长时间无数据"主动重开。
            self.get_logger().warn('雷达 2 秒无数据，重新扫描串口')
            try:
                os.close(self.fd)
            except Exception:
                pass
            self.fd = None
            if self.auto_mode:
                self.port = 'auto'
            return
        self._parse()

    def _parse(self):
        buf = self.buf
        while True:
            idx = buf.find(HDR)
            if idx < 0:
                del buf[:max(0, len(buf) - 1)]
                return
            if idx:
                del buf[:idx]
            if len(buf) < 4:
                return
            num = buf[3]
            if num < 1 or num > 32:
                del buf[:1]
                continue
            flen = 12 + 2 * num
            if len(buf) < flen:
                return
            frame = bytes(buf[:flen])
            del buf[:flen]
            self._handle(frame, num)

    def _handle(self, frame, num):
        n = len(frame)
        fsa = (frame[6] | (frame[7] << 8)) * ANGLE_UNIT
        lsa = (frame[n - 4] | (frame[n - 3] << 8)) * ANGLE_UNIT
        span = (lsa - fsa) % 360.0
        if not 4.0 <= span <= 8.0:
            return

        if self.last_angle is not None and fsa < self.last_angle - 180.0:
            self._publish()
        self.last_angle = fsa

        step = span / (num - 1) if num > 1 else self.bin_deg
        distances = []
        for k in range(num):
            raw = frame[8 + 2 * k] | (frame[9 + 2 * k] << 8)
            dist = raw / self.dist_divisor / 1000.0
            if dist < self.range_min or dist > self.range_max:
                distances.append(float('inf'))
                continue
            distances.append(dist)
            idx = int(((fsa + k * step) % 360.0) / self.bin_deg) % self.bins
            self.ranges[idx] = dist

        if self.pub_fast is not None and self.pub_fast.get_subscription_count() > 0:
            self._publish_fast(fsa, lsa, span, num, distances)

    def _publish_fast(self, fsa, lsa, span, num, distances):
        """把当前扇区单独发布为一条 LaserScan，供低延迟反应层使用。"""
        frame_dur = span / 360.0 / self.scan_hz if self.scan_hz > 0 else 0.0
        if self.mirror:
            ranges = list(reversed(distances))
            angle_min = (360.0 - lsa) % 360.0
        else:
            ranges = distances
            angle_min = fsa

        step_deg = span / (num - 1) if num > 1 else self.bin_deg
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.angle_min = math.radians(angle_min)
        msg.angle_increment = math.radians(step_deg)
        msg.angle_max = msg.angle_min + math.radians(step_deg) * (num - 1)
        msg.time_increment = frame_dur / num if num > 0 else 0.0
        msg.scan_time = frame_dur
        msg.range_min = self.range_min
        msg.range_max = self.range_max
        msg.ranges = ranges
        self.pub_fast.publish(msg)

    def _publish(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.angle_min = 0.0
        msg.angle_increment = math.radians(self.bin_deg)
        msg.angle_max = msg.angle_min + math.radians(self.bin_deg) * self.bins
        msg.time_increment = 0.0
        msg.scan_time = 1.0 / self.scan_hz if self.scan_hz > 0 else 0.0
        msg.range_min = self.range_min
        msg.range_max = self.range_max
        if self.mirror:   # 传感器顺时针输出，按角度取反做镜像
            src = self.ranges
            msg.ranges = [src[(-i) % self.bins] for i in range(self.bins)]
        else:
            msg.ranges = self.ranges
        self.pub.publish(msg)
        self.ranges = [float('inf')] * self.bins


def main():
    rclpy.init()
    node = D2ScanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.fd is not None:
            try:
                os.close(node.fd)
            except Exception:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
