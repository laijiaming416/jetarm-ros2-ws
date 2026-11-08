#!/usr/bin/env python3
"""AMCL 初始位姿自动恢复 + 全局重定位兜底。

启动流程：
    1. 等待 AMCL 就绪（startup_delay）
    2. 读取 ~/.ros/last_pose_<地图名>.yaml，向 /initialpose 发布 3 次
    3. 若没有存档，或 fallback_delay 后定位协方差仍然过大，
       调用 /reinitialize_global_localization 触发全局重定位
运行期间：
    每 save_period 秒通过 TF 读取 map->base_link，位姿变化超过阈值就落盘
手动兜底：
    服务 /relocalize（std_srvs/Trigger）可随时触发一次全局重定位
"""
import math
import os
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Empty, Trigger
from tf2_ros import Buffer, TransformListener


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class PoseRestorer(Node):

    def __init__(self):
        super().__init__('pose_restorer')

        self.declare_parameter('map_file', 'map.yaml')
        self.declare_parameter('startup_delay', 5.0)
        self.declare_parameter('amcl_wait', 30.0)
        self.declare_parameter('publish_initial', True)
        self.declare_parameter('global_fallback', True)
        self.declare_parameter('check_amcl', True)
        self.declare_parameter('cov_save_threshold', 0.5)
        self.declare_parameter('fallback_delay', 20.0)
        self.declare_parameter('save_period', 2.0)
        self.declare_parameter('move_threshold', 0.03)

        map_file = str(self.get_parameter('map_file').value)
        map_name = os.path.splitext(os.path.basename(map_file))[0] or 'map'
        self.pose_file = os.path.expanduser(
            '~/.ros/last_pose_%s.yaml' % map_name)
        self.startup_delay = float(self.get_parameter('startup_delay').value)
        self.amcl_wait = float(self.get_parameter('amcl_wait').value)
        self.publish_initial = bool(self.get_parameter('publish_initial').value)
        self.global_fallback = bool(
            self.get_parameter('global_fallback').value)
        self.check_amcl = bool(self.get_parameter('check_amcl').value)
        self.cov_save_threshold = float(
            self.get_parameter('cov_save_threshold').value)
        self.fallback_delay = float(self.get_parameter('fallback_delay').value)
        self.save_period = float(self.get_parameter('save_period').value)
        self.move_threshold = float(self.get_parameter('move_threshold').value)

        self.saved = self._load_pose()
        self.init_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.global_cli = self.create_client(
            Empty, '/reinitialize_global_localization')
        self.create_service(Trigger, '/relocalize', self.on_relocalize)

        self.start = time.monotonic()
        self.amcl_ready = False
        self.init_sent = 3 if not self.publish_initial else 0
        self.last_init = 0.0
        self.fallback_tried = not self.global_fallback
        self.amcl_stamp = 0.0
        self.amcl_cov = float('inf')
        self.last_save_pos = None
        self.last_save_time = 0.0

        self.create_timer(0.5, self.tick)
        self.get_logger().info('位姿存档: %s（%s）'
                               % (self.pose_file,
                                  '已找到' if self.saved else '无'))

    # ---------- 位姿文件 ----------
    def _load_pose(self):
        if not os.path.exists(self.pose_file):
            return None
        try:
            with open(self.pose_file, 'r') as f:
                data = yaml.safe_load(f) or {}
            return (float(data['x']), float(data['y']), float(data['yaw']))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn('读取位姿存档失败: %s' % exc)
            return None

    def _save_pose(self, x, y, yaw):
        try:
            os.makedirs(os.path.dirname(self.pose_file), exist_ok=True)
            with open(self.pose_file, 'w') as f:
                yaml.safe_dump({'x': round(x, 4), 'y': round(y, 4),
                                'yaw': round(yaw, 4)}, f)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn('保存位姿失败: %s' % exc)

    # ---------- 回调 ----------
    def on_amcl(self, msg):
        self.amcl_stamp = time.monotonic()
        cov = msg.pose.covariance
        self.amcl_cov = max(cov[0], cov[7])

    def on_relocalize(self, request, response):
        ok = self._call_global()
        response.success = ok
        response.message = '已触发全局重定位，请缓慢移动车辆' if ok \
            else '全局重定位服务不可用'
        return response

    def _call_global(self):
        if not self.global_cli.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn(
                '/reinitialize_global_localization 服务不可用')
            return False
        self.global_cli.call_async(Empty.Request())
        self.get_logger().info('已触发全局重定位，请缓慢移动车辆让粒子收敛')
        return True

    def _publish_initial(self, x, y, yaw):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        cov = [0.0] * 36
        # 有存档时相信存档（标准差 0.25m）；无存档时给大协方差让滤波器自己找
        cov_xy = 0.0625 if self.saved else 4.0
        cov_yaw = 0.017 if self.saved else 1.0
        cov[0] = cov_xy
        cov[7] = cov_xy
        cov[35] = cov_yaw
        msg.pose.covariance = cov
        self.init_pub.publish(msg)

    # ---------- 主循环 ----------
    def tick(self):
        now = time.monotonic()
        elapsed = now - self.start

        # 等 AMCL 真正起来（全局重定位服务存在）再发初始位姿，
        # 避免 nav2 启动慢时消息丢失。
        if not self.amcl_ready:
            if self.global_cli.service_is_ready():
                self.amcl_ready = True
                self.get_logger().info('AMCL 已就绪')
            elif elapsed > self.amcl_wait:
                self.amcl_ready = True
                self.get_logger().warn('等待 AMCL 超时，继续发布初始位姿')
            else:
                return

        if elapsed >= self.startup_delay and self.init_sent < 3:
            if now - self.last_init >= 0.5:
                x, y, yaw = self.saved if self.saved else (0.0, 0.0, 0.0)
                self._publish_initial(x, y, yaw)
                self.init_sent += 1
                self.last_init = now
                if self.init_sent == 1:
                    self.get_logger().info(
                        '发布初始位姿: x=%.2f y=%.2f yaw=%.2f'
                        % (x, y, yaw))
            return

        if (self.init_sent >= 3 and not self.fallback_tried
                and now - self.last_init >= self.fallback_delay):
            self.fallback_tried = True
            if self.saved is None:
                self.get_logger().warn('无位姿存档，触发全局重定位')
                self._call_global()
            else:
                self.get_logger().info(
                    '有存档位姿，跳过自动全局重定位（对不上请用 /relocalize）')

        if now - self.last_save_time >= self.save_period:
            self.last_save_time = now
            # 定位不健康时不落盘，避免把错误位姿覆盖掉正确的存档
            if self.check_amcl:
                fresh = now - self.amcl_stamp < 2.0
                if not fresh or self.amcl_cov > self.cov_save_threshold:
                    return
            self.save_current()

    def save_current(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_link', Time(), timeout=Duration(seconds=0.1))
        except Exception:  # noqa: BLE001
            return
        t = tf.transform.translation
        yaw = yaw_from_quat(tf.transform.rotation)
        pos = (t.x, t.y, yaw)
        if self.last_save_pos is None:
            moved = True
        else:
            dx = abs(pos[0] - self.last_save_pos[0])
            dy = abs(pos[1] - self.last_save_pos[1])
            da = abs((pos[2] - self.last_save_pos[2] + math.pi)
                     % (2 * math.pi) - math.pi)
            moved = max(dx, dy) > self.move_threshold or \
                da > self.move_threshold
        if moved:
            self._save_pose(*pos)
            self.last_save_pos = pos


def main():
    rclpy.init()
    node = PoseRestorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.save_current()
        except Exception:  # noqa: BLE001
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
