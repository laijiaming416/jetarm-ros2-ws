#!/usr/bin/env python3
"""雷达避障节点：订阅 /scan，持续发布 /cmd_vel 给 mecanum_base 底盘。

安全约定：
    armed 默认 false —— 只打印决策、始终发布零速，车不会动。
    -p armed:=true 才真正输出非零速度。
    底盘看门狗 200ms，本节点 20Hz 发布，不会因超时自停。

策略（连续速度，允许混轴，麦轮可横移）：
    前方 max(safe) 以上        -> 巡航前进
    前方 stop..safe            -> 减速贴近
    前方 stop 以内             -> 停前进，优先横移绕障，其次后退，最后原地转
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

SECTORS = {
    'front': (335.0, 25.0),
    'fl': (25.0, 80.0),
    'fr': (280.0, 335.0),
    'left': (80.0, 110.0),
    'right': (250.0, 280.0),
    'rear': (150.0, 210.0),
}


def in_sector(angle_deg, lo, hi):
    if lo <= hi:
        return lo <= angle_deg <= hi
    return angle_deg >= lo or angle_deg <= hi


def sector_min(ranges, amin_deg, inc_deg, lo, hi):
    best = float('inf')
    for i, r in enumerate(ranges):
        if r <= 0.0 or math.isinf(r) or math.isnan(r):
            continue
        if in_sector((amin_deg + i * inc_deg) % 360.0, lo, hi) and r < best:
            best = r
    return best


class D2AvoidNode(Node):

    def __init__(self):
        super().__init__('d2_avoid')
        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('cmd_topic', 'cmd_vel')
        self.declare_parameter('armed', False)
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('scan_timeout', 0.5)
        self.declare_parameter('cruise_speed', 0.15)
        self.declare_parameter('creep_speed', 0.06)
        self.declare_parameter('strafe_speed', 0.10)
        self.declare_parameter('back_speed', 0.10)
        self.declare_parameter('rotate_speed', 0.50)
        self.declare_parameter('safe_dist', 0.80)
        self.declare_parameter('stop_dist', 0.45)
        self.declare_parameter('open_dist', 0.70)
        self.declare_parameter('accel_limit', 1.0)   # m/s^2
        self.declare_parameter('yaw_accel_limit', 2.0)  # rad/s^2

        self.armed = bool(self.get_parameter('armed').value)
        self.rate = max(5.0, float(self.get_parameter('rate_hz').value))
        self.dt = 1.0 / self.rate
        self.scan_timeout = float(self.get_parameter('scan_timeout').value)
        self.cruise = float(self.get_parameter('cruise_speed').value)
        self.creep = float(self.get_parameter('creep_speed').value)
        self.strafe = float(self.get_parameter('strafe_speed').value)
        self.back = float(self.get_parameter('back_speed').value)
        self.rotate = float(self.get_parameter('rotate_speed').value)
        self.safe_dist = float(self.get_parameter('safe_dist').value)
        self.stop_dist = float(self.get_parameter('stop_dist').value)
        self.open_dist = float(self.get_parameter('open_dist').value)
        self.accel = float(self.get_parameter('accel_limit').value)
        self.yaw_accel = float(self.get_parameter('yaw_accel_limit').value)

        self.scan = None
        self.scan_t = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.last_reason = None

        self.create_subscription(LaserScan, self.get_parameter('scan_topic').value,
                                 self._on_scan, 10)
        self.pub = self.create_publisher(Twist, self.get_parameter('cmd_topic').value, 10)
        self.create_timer(self.dt, self._tick)
        self.get_logger().info('雷达避障已启动，armed=%s，输出 %s'
                               % (self.armed, self.get_parameter('cmd_topic').value))

    def _on_scan(self, msg):
        self.scan = msg
        self.scan_t = self.get_clock().now().nanoseconds / 1e9

    def _sectors(self):
        msg = self.scan
        amin = math.degrees(msg.angle_min)
        inc = math.degrees(msg.angle_increment)
        return {name: sector_min(msg.ranges, amin, inc, lo, hi)
                for name, (lo, hi) in SECTORS.items()}

    def _target(self):
        """返回 (vx, vy, wz, reason)。"""
        if not self.armed:
            return 0.0, 0.0, 0.0, 'not armed'
        if self.scan is None:
            return 0.0, 0.0, 0.0, 'no scan'
        age = self.get_clock().now().nanoseconds / 1e9 - self.scan_t
        if age > self.scan_timeout:
            return 0.0, 0.0, 0.0, 'scan stale %.2fs' % age

        s = self._sectors()
        if all(math.isinf(v) for v in s.values()):
            return 0.0, 0.0, 0.0, 'no lidar returns'

        front = s['front']
        left_clear = min(s['fl'], s['left'])
        right_clear = min(s['fr'], s['right'])

        if front >= self.safe_dist:
            return self.cruise, 0.0, 0.0, 'cruise front %.2fm' % front
        if front >= self.stop_dist:
            return self.creep, 0.0, 0.0, 'slow front %.2fm' % front
        if left_clear >= self.open_dist and left_clear >= right_clear:
            return 0.0, self.strafe, 0.0, 'strafe left %.2fm' % left_clear
        if right_clear >= self.open_dist:
            return 0.0, -self.strafe, 0.0, 'strafe right %.2fm' % right_clear
        if s['rear'] >= self.open_dist:
            return -self.back, 0.0, 0.0, 'back off %.2fm' % s['rear']
        return 0.0, 0.0, self.rotate, 'rotate in place front %.2fm' % front

    def _tick(self):
        tx, ty, tz, reason = self._target()

        max_dv = self.accel * self.dt
        max_dw = self.yaw_accel * self.dt
        self.vx += max(-max_dv, min(max_dv, tx - self.vx))
        self.vy += max(-max_dv, min(max_dv, ty - self.vy))
        self.wz += max(-max_dw, min(max_dw, tz - self.wz))

        if reason != self.last_reason:
            self.get_logger().info('决策：%s' % reason)
            self.last_reason = reason

        msg = Twist()
        msg.linear.x = self.vx
        msg.linear.y = self.vy
        msg.angular.z = self.wz
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = D2AvoidNode()
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
