#!/usr/bin/env python3
"""激光安装偏角测量工具。

用法：
    1. 把车推到平整墙面前，车头正对墙
    2. 用卷尺把车体左右两个前角到墙的距离调成相等（差 1mm 约等于 0.2°）
    3. 运行 ros2 run mecanum_nav laser_yaw_check
    4. 工具每秒打印前墙在 base_link 下的角度，并给出建议的 yaw_offset_deg

判断标准：
    偏差 < 0.3° 可以认为已对齐（D2 角分辨率 0.72°/点，再往下没有意义）。
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


def _normalize_deg(a):
    """把角度折算到 (-90, 90]，墙面是无向直线，差 180° 等价。"""
    while a > 90.0:
        a -= 180.0
    while a <= -90.0:
        a += 180.0
    return a


class LaserYawCheck(Node):

    def __init__(self):
        super().__init__('laser_yaw_check')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('scan_frame', 'laser')
        self.declare_parameter('max_range', 1.5)
        self.declare_parameter('half_width', 0.8)
        self.declare_parameter('min_points', 10)

        self.base_frame = self.get_parameter('base_frame').value
        self.scan_frame = self.get_parameter('scan_frame').value
        self.max_range = float(self.get_parameter('max_range').value)
        self.half_width = float(self.get_parameter('half_width').value)
        self.min_points = int(self.get_parameter('min_points').value)

        self.scan = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(LaserScan, 'scan', self.on_scan, 10)
        self.create_timer(1.0, self.report)
        self.get_logger().info('等待 /scan …（车头正对平整墙面时读数才有效）')

    def on_scan(self, msg):
        self.scan = msg

    def report(self):
        if self.scan is None:
            return
        try:
            tr = self.tf_buffer.lookup_transform(
                self.base_frame, self.scan_frame, rclpy.time.Time(),
                rclpy.time.Duration(seconds=0.5))
        except Exception:
            self.get_logger().warn('暂时拿不到 %s -> %s 的 TF'
                                   % (self.base_frame, self.scan_frame))
            return

        t = tr.transform.translation
        q = tr.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        scan = self.scan
        pts = []
        for i, r in enumerate(scan.ranges):
            if not (scan.range_min <= r <= scan.range_max):
                continue
            if r > self.max_range:
                continue
            a = scan.angle_min + i * scan.angle_increment + yaw
            x = t.x + r * math.cos(a)
            y = t.y + r * math.sin(a)
            if 0.1 < x and abs(y) < self.half_width:
                pts.append((x, y))

        if len(pts) < self.min_points:
            self.get_logger().warn('前方有效点太少（%d 个），确认车头正对墙面、距离 0.2~1.2m'
                                   % len(pts))
            return

        mx = sum(p[0] for p in pts) / len(pts)
        my = sum(p[1] for p in pts) / len(pts)
        sxx = sum((p[0] - mx) ** 2 for p in pts)
        syy = sum((p[1] - my) ** 2 for p in pts)
        sxy = sum((p[0] - mx) * (p[1] - my) for p in pts)
        theta = 0.5 * math.atan2(2.0 * sxy, sxx - syy)

        nx, ny = -math.sin(theta), math.cos(theta)
        rms = (sum(((p[0] - mx) * nx + (p[1] - my) * ny) ** 2
                   for p in pts) / len(pts)) ** 0.5

        theta_deg = _normalize_deg(math.degrees(theta))
        dev = _normalize_deg(theta_deg - 90.0)
        current = math.degrees(yaw)
        suggested = current - dev

        self.get_logger().info(
            '前墙 %+.2f°  偏差 %+.2f°  残差 %.1fcm  点数 %d  当前 yaw_offset %.1f° -> 建议 %.1f°'
            % (theta_deg, dev, rms * 100.0, len(pts), current, suggested))
        if rms > 0.05:
            self.get_logger().warn('拟合残差偏大（%.1fcm），前方可能不是平整墙面'
                                   % (rms * 100.0))


def main():
    rclpy.init()
    node = LaserYawCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
