#!/usr/bin/env python3
"""底盘速度指令仲裁节点。

订阅：
    /cmd_vel_joy  手柄遥控输出（仅 START 使能后才发布）
    /cmd_vel_nav  nav2 控制器输出（navigation_launch 默认重映射）
发布：
    /cmd_vel_final  给底盘节点

规则：
    1. 手柄 0.3s 内有**非零**指令 -> 手柄独占（人工接管）
       注意：joy_teleop 以 20Hz 持续发布零速保活，不能只看"有没有消息"，
       否则导航指令会被永远压在零速下面（历史 bug）。
    2. 否则导航 0.5s 内有消息 -> 使用导航指令
    3. 两者都没有 -> 发布零速
"""
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdMux(Node):

    def __init__(self):
        super().__init__('cmd_mux')

        self.declare_parameter('joy_topic', '/cmd_vel_joy')
        self.declare_parameter('nav_topic', '/cmd_vel_nav')
        self.declare_parameter('out_topic', '/cmd_vel_final')
        self.declare_parameter('joy_timeout', 0.3)
        self.declare_parameter('nav_timeout', 0.5)
        self.declare_parameter('rate_hz', 20.0)

        joy_topic = str(self.get_parameter('joy_topic').value)
        nav_topic = str(self.get_parameter('nav_topic').value)
        out_topic = str(self.get_parameter('out_topic').value)
        self.joy_timeout = float(self.get_parameter('joy_timeout').value)
        self.nav_timeout = float(self.get_parameter('nav_timeout').value)
        rate = max(1.0, float(self.get_parameter('rate_hz').value))

        self.joy_msg = None
        self.joy_stamp = 0.0
        # 只有手柄给出非零指令时才延长这个时刻；持续发布的零速不算接管
        self.joy_hold_until = 0.0
        self.nav_msg = None
        self.nav_stamp = 0.0
        self.source = None

        self.pub = self.create_publisher(Twist, out_topic, 10)
        self.create_subscription(Twist, joy_topic, self.on_joy, 10)
        self.create_subscription(Twist, nav_topic, self.on_nav, 10)
        self.create_timer(1.0 / rate, self.tick)

        self.get_logger().info(
            '指令仲裁：手柄 %s > 导航 %s -> %s'
            % (joy_topic, nav_topic, out_topic))

    def on_joy(self, msg):
        self.joy_msg = msg
        self.joy_stamp = time.monotonic()
        if (abs(msg.linear.x) + abs(msg.linear.y) + abs(msg.angular.z)) > 1e-6:
            # 非零指令 = 人工接管，从此刻起独占 joy_timeout
            self.joy_hold_until = self.joy_stamp + self.joy_timeout

    def on_nav(self, msg):
        self.nav_msg = msg
        self.nav_stamp = time.monotonic()

    def tick(self):
        now = time.monotonic()

        if self.joy_msg is not None and now < self.joy_hold_until:
            source, msg = 'joy', self.joy_msg
        elif self.nav_msg is not None and now - self.nav_stamp < self.nav_timeout:
            source, msg = 'nav', self.nav_msg
        else:
            source, msg = 'zero', Twist()

        if source != self.source:
            self.get_logger().info('指令源切换: %s -> %s'
                                   % (self.source or '-', source))
            self.source = source

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = CmdMux()
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
