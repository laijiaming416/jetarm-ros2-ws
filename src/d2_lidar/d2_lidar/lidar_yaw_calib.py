#!/usr/bin/env python3
"""贴墙标定：拟合墙面直线，推算 base_link 与雷达 0° 的偏航差。

原理：
    /scan 的角度属于雷达坐标系 laser，车身坐标系是 base_link。
    把车身侧面贴紧直墙后，墙法线在 base_link 下的方向是已知的：
        墙在左侧 -> +90°，墙在右侧 -> -90°，车头正对墙 -> 0°
    拟合得到的墙法线角 φ 是 laser 下的角度，则
        yaw_offset_deg = ψ_base - φ_laser

用法：
    ros2 run d2_lidar lidar_yaw_calib --duration 10
    ros2 run d2_lidar lidar_yaw_calib --duration 10 --sector 300 60
"""

import argparse
import math
import random
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


def norm180(deg):
    while deg > 180.0:
        deg -= 360.0
    while deg <= -180.0:
        deg += 360.0
    return deg


def in_sector(deg, lo, hi):
    if lo <= hi:
        return lo <= deg <= hi
    return deg >= lo or deg <= hi


def fit_line(points, thresh=0.02, iters=300, seed=7):
    """RANSAC + 最小二乘拟合，返回 (方向角, 法线候选, 内点数, RMS, 质心)。"""
    n = len(points)
    if n < 20:
        return None
    rnd = random.Random(seed)
    best = []
    for _ in range(iters):
        i = rnd.randrange(n)
        j = rnd.randrange(n)
        if i == j:
            continue
        x1, y1 = points[i]
        x2, y2 = points[j]
        dx, dy = x2 - x1, y2 - y1
        length = math.hypot(dx, dy)
        if length < 0.3:
            continue
        ux, uy = dx / length, dy / length
        inliers = []
        for x, y in points:
            if abs((x - x1) * uy - (y - y1) * ux) <= thresh:
                inliers.append((x, y))
        if len(inliers) > len(best):
            best = inliers

    if len(best) < 20:
        return None

    cx = sum(p[0] for p in best) / len(best)
    cy = sum(p[1] for p in best) / len(best)
    sxx = sum((p[0] - cx) ** 2 for p in best)
    syy = sum((p[1] - cy) ** 2 for p in best)
    sxy = sum((p[0] - cx) * (p[1] - cy) for p in best)
    theta = 0.5 * math.atan2(2.0 * sxy, sxx - syy)  # 直线方向
    ux, uy = math.cos(theta), math.sin(theta)
    rms = math.sqrt(sum(((p[0] - cx) * uy - (p[1] - cy) * ux) ** 2
                        for p in best) / len(best))

    dir_deg = math.degrees(theta)
    n1 = norm180(dir_deg + 90.0)
    n2 = norm180(dir_deg - 90.0)
    bearing = math.degrees(math.atan2(cy, cx))
    toward = n1 if abs(norm180(n1 - bearing)) <= abs(norm180(n2 - bearing)) else n2

    return dir_deg, n1, n2, toward, len(best), rms, (cx, cy)


class Calib(Node):

    def __init__(self, args):
        super().__init__('lidar_yaw_calib')
        self.args = args
        self.points = []
        self.scans = 0
        self.create_subscription(LaserScan, 'scan', self.on_scan, 10)

    def on_scan(self, msg):
        self.scans += 1
        amin = msg.angle_min
        inc = msg.angle_increment
        lo, hi = (self.args.sector if self.args.sector else (0.0, 360.0))
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r):
                continue
            if r < self.args.min_range or r > self.args.max_range:
                continue
            deg = math.degrees(amin + i * inc) % 360.0
            if not in_sector(deg, lo, hi):
                continue
            a = math.radians(deg)
            self.points.append((r * math.cos(a), r * math.sin(a)))

    def report(self):
        if not self.points:
            print('没有采集到有效点：确认雷达节点在运行、/scan 有数据')
            return 1

        pts = self.points
        if len(pts) > 4000:
            step = len(pts) // 4000 + 1
            pts = pts[::step]

        result = fit_line(pts, self.args.threshold)
        if result is None:
            print('拟合失败：点太少或墙面不够平直，换个位置或延长采集时间后重试')
            return 1

        dir_deg, n1, n2, toward, inliers, rms, c = result
        print('采集 %d 帧 / %d 点（拟合用 %d 点）'
              % (self.scans, len(self.points), len(pts)))
        print('墙面：内点 %d，RMS %.3f m，墙面距离约 %.2f m'
              % (inliers, rms, math.hypot(c[0], c[1])))
        print('直线方向 %.1f°，法线候选 %.1f° / %.1f°'
              % (dir_deg, n1, n2))
        print('指向墙面的法线 φ = %.1f°' % toward)
        print('')
        print('若墙在车的左侧（base_link 法线 +90°） -> yaw_offset_deg = %.1f'
              % norm180(90.0 - toward))
        print('若墙在车的右侧（base_link 法线 -90°） -> yaw_offset_deg = %.1f'
              % norm180(-90.0 - toward))
        print('若车头正对墙面（base_link 法线 0°） -> yaw_offset_deg = %.1f'
              % norm180(0.0 - toward))
        return 0


def main():
    ap = argparse.ArgumentParser(description='贴墙标定雷达偏航角')
    ap.add_argument('--duration', type=float, default=10.0, help='采集秒数')
    ap.add_argument('--sector', nargs=2, type=float, default=None,
                    metavar=('LO', 'HI'), help='只取该角度范围内的点（度，可跨 0°）')
    ap.add_argument('--threshold', type=float, default=0.02, help='RANSAC 内点阈值 m')
    ap.add_argument('--min-range', type=float, default=0.15)
    ap.add_argument('--max-range', type=float, default=8.0)
    args, _ = ap.parse_known_args()

    rclpy.init()
    node = Calib(args)
    print('开始采集 %.0f 秒，请保持车辆静止……' % args.duration)
    start = time.monotonic()
    try:
        while time.monotonic() - start < args.duration:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    code = node.report()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
