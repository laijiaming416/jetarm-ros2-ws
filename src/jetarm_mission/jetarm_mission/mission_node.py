#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""取物任务编排节点。

流程：
    网页选点  → /mission/target (PoseStamped, TRANSIENT_LOCAL)
    说「你好去拿」→ /voice/intent(action=mission_start)
        → 记录起点 map->base_link
        → nav2 去取物点
        → /grasp（视觉抓取 + 收拢到运输位）
        → nav2 回起点
        → 播报「已取回」

任何时刻「停止/取消」→ 取消导航 + /arm/stop + 状态 canceled。

话题：
    /mission/target   PoseStamped   输入（网页发布）
    /voice/intent     Intent        输入
    /mode/state       String        输入
    /mission/state    MissionState  输出
    /voice/reply      String        输出（交给 TTS）
服务：
    /mission/start    Trigger
    /mission/cancel   Trigger
"""

import math
import os
import threading
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy)
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from jetarm_interfaces.msg import Intent, MissionState
from jetarm_interfaces.srv import Grasp


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class MissionNode(Node):

    def __init__(self):
        super().__init__('mission_node')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('home_file', os.path.expanduser(
            '~/.ros/mission_home.yaml'))
        self.declare_parameter('skip_nav', False)
        self.declare_parameter('skip_grasp', False)
        self.declare_parameter('nav_timeout_s', 300.0)
        self.declare_parameter('grasp_timeout_s', 300.0)
        self.declare_parameter('require_navigation_mode', True)
        # nav2 在 CPU 抖动时可能报失败，但车其实已经到点；用位姿兜底
        self.declare_parameter('arrive_tolerance_m', 0.35)

        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.home_file = str(self.get_parameter('home_file').value)
        self.skip_nav = bool(self.get_parameter('skip_nav').value)
        self.skip_grasp = bool(self.get_parameter('skip_grasp').value)
        self.nav_timeout = float(self.get_parameter('nav_timeout_s').value)
        self.grasp_timeout = float(self.get_parameter('grasp_timeout_s').value)
        self.require_nav_mode = bool(
            self.get_parameter('require_navigation_mode').value)
        self.arrive_tol = float(
            self.get_parameter('arrive_tolerance_m').value)

        self.target = None
        self.home = None
        self.mode = 'unknown'
        self.state = 'idle'
        self.detail = ''
        self.busy = False
        self.cancel_flag = threading.Event()
        self.goal_handle = None
        self.distance = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        group = ReentrantCallbackGroup()
        self.create_subscription(
            PoseStamped, '/mission/target', self.on_target,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL),
            callback_group=group)
        self.create_subscription(Intent, '/voice/intent', self.on_intent, 10,
                                 callback_group=group)
        self.create_subscription(String, '/mode/state', self.on_mode, 10,
                                 callback_group=group)

        self.state_pub = self.create_publisher(MissionState, '/mission/state', 10)
        self.reply_pub = self.create_publisher(String, '/voice/reply', 10)
        self.create_service(Trigger, '/mission/start', self.on_start,
                            callback_group=group)
        self.create_service(Trigger, '/mission/cancel', self.on_cancel,
                            callback_group=group)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.grasp_cli = self.create_client(Grasp, '/grasp')
        self.arm_stop_cli = self.create_client(Trigger, '/arm/stop')

        self.create_timer(1.0, self.publish_state)
        self.get_logger().info('mission_node 启动（skip_nav=%s skip_grasp=%s）'
                               % (self.skip_nav, self.skip_grasp))

    # ---------- 输入 ----------
    def on_target(self, msg):
        self.target = msg
        self.get_logger().info('取物点: %.2f, %.2f'
                               % (msg.pose.position.x, msg.pose.position.y))
        if not self.busy and self.state in ('idle', 'armed', 'done', 'canceled',
                                            'error'):
            self._set_state('armed', '取物点已选')

    def on_mode(self, msg):
        self.mode = str(msg.data)

    def on_intent(self, msg):
        if msg.action == 'stop':
            # 空闲且没有待执行目标时不播报，避免 TTS 声音被麦克风录回去
            self._cancel('voice', speak=(self.busy or self.target is not None))
        elif msg.action == 'mission_start':
            ok, text = self._start_mission()
            if not ok:
                self._reply(text)
        elif msg.action == 'mission_home':
            self._reply('请说去拿东西开始任务。')

    def on_start(self, req, res):
        del req
        ok, text = self._start_mission()
        res.success, res.message = ok, text
        return res

    def on_cancel(self, req, res):
        del req
        ok, text = self._cancel('manual')
        res.success, res.message = ok, text
        return res

    # ---------- 任务 ----------
    def _start_mission(self):
        if self.busy:
            return False, '任务正在执行'
        if self.target is None:
            return False, '还没有在图上选取物点'
        if self.require_nav_mode and self.mode != 'navigation':
            return False, '请先在网页切到导航模式'
        home = self._capture_pose()
        if home is None and not self.skip_nav:
            return False, '定位还没就绪'
        self.home = home if home is not None else (0.0, 0.0, 0.0)
        self._save_home(self.home)
        self.busy = True
        self.cancel_flag.clear()
        self._set_state('armed', '任务开始')
        threading.Thread(target=self._run, daemon=True).start()
        return True, '任务已开始'

    def _run(self):
        try:
            self._reply('出发。')
            self._set_state('to_target', '')
            tx = self.target.pose.position.x
            ty = self.target.pose.position.y
            ok, text = self._navigate(
                tx, ty, yaw_from_quat(self.target.pose.orientation))
            if self.cancel_flag.is_set():
                return
            if not ok:
                dist = self._distance_to(tx, ty)
                if dist is not None and dist <= self.arrive_tol:
                    self.get_logger().warn(
                        '导航未报成功（%s），但距目标 %.2fm，按到达处理' % (text, dist))
                    time.sleep(1.0)
                else:
                    self._fail('没走到取物点：' + text)
                    return

            self._set_state('grasping', '')
            self._reply('已到达，开始抓取。')
            ok, text = self._grasp()
            if self.cancel_flag.is_set():
                return
            self._reply('抓到了，正在返回。' if ok else '没抓到，正在返回。')

            self._set_state('to_home', '')
            ok2, text2 = self._navigate(*self.home)
            if self.cancel_flag.is_set():
                return
            if not ok2:
                dist = self._distance_to(self.home[0], self.home[1])
                if dist is not None and dist <= self.arrive_tol:
                    self.get_logger().warn(
                        '返航未报成功（%s），但距起点 %.2fm，按到达处理'
                        % (text2, dist))
                    time.sleep(1.0)
                else:
                    self._fail('回起点失败：' + text2)
                    return
            self._set_state('done', '')
            # 「已取回。」单独一句合成出来有气声/沙哑，加半句后模型音质明显更好
            self._reply('已取回，请查收。')
        except Exception as exc:  # noqa: BLE001
            if not self.cancel_flag.is_set():
                self._fail('任务异常：%s' % exc)
        finally:
            self.busy = False

    def _navigate(self, x, y, yaw):
        if self.skip_nav:
            self.get_logger().warn('skip_nav=true，模拟到达 (%.2f, %.2f)'
                                   % (x, y))
            time.sleep(1.0)
            return True, '(dry-run)'
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            return False, '导航 action 未就绪'
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
        goal.pose.pose.orientation.w = math.cos(float(yaw) / 2.0)
        send_future = self.nav_client.send_goal_async(
            goal, feedback_callback=self._on_nav_feedback)
        handle = self._wait_future(send_future, 15.0)
        if handle is None:
            return False, '目标发送超时'
        if not handle.accepted:
            return False, '目标被拒绝'
        self.goal_handle = handle
        result = self._wait_future(handle.get_result_async(),
                                   self.nav_timeout)
        self.goal_handle = None
        if result is None:
            handle.cancel_goal_async()
            return False, '导航超时'
        if result.status == 4:
            return True, ''
        return False, '导航状态 %d' % result.status

    def _on_nav_feedback(self, msg):
        try:
            self.distance = float(msg.feedback.distance_remaining)
        except Exception:  # noqa: BLE001
            pass

    def _grasp(self):
        if self.skip_grasp:
            self.get_logger().warn('skip_grasp=true，模拟抓取成功')
            time.sleep(1.0)
            return True, '(dry-run)'
        if not self.grasp_cli.wait_for_service(timeout_sec=10.0):
            return False, '抓取服务未就绪'
        req = Grasp.Request()
        req.hue = 0
        req.height_mm = 0.0
        req.tool_deg = 0.0
        req.z_try_mm = 0.0
        req.min_area = 0.0
        req.lift_after = True
        future = self.grasp_cli.call_async(req)
        res = self._wait_future(future, self.grasp_timeout)
        if res is None:
            return False, '抓取超时（%d 秒）' % int(self.grasp_timeout)
        return bool(res.ok), str(res.message)

    def _cancel(self, source, speak=True):
        self.cancel_flag.set()
        if self.goal_handle is not None:
            try:
                self.goal_handle.cancel_goal_async()
            except Exception:  # noqa: BLE001
                pass
        if self.arm_stop_cli.wait_for_service(timeout_sec=1.0):
            self.arm_stop_cli.call_async(Trigger.Request())
        self.busy = False
        self.target = None
        self._set_state('idle', '已取消：%s' % source)
        if speak:
            self._reply('已停止。')
        return True, '任务已取消'

    def _fail(self, text):
        self.get_logger().error(text)
        self._set_state('error', text)
        self._reply(text)

    # ---------- 工具 ----------
    def _wait_future(self, future, timeout):
        deadline = time.monotonic() + float(timeout)
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        return future.result() if future.done() else None

    def _distance_to(self, x, y):
        pose = self._capture_pose()
        if pose is None:
            return None
        return math.hypot(pose[0] - float(x), pose[1] - float(y))

    def _capture_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time(),
                timeout=Duration(seconds=0.5))
        except Exception:  # noqa: BLE001
            return None
        t = tf.transform.translation
        return (t.x, t.y, yaw_from_quat(tf.transform.rotation))

    def _save_home(self, pose):
        try:
            os.makedirs(os.path.dirname(self.home_file), exist_ok=True)
            with open(self.home_file, 'w') as f:
                yaml.safe_dump({'x': round(pose[0], 4),
                                'y': round(pose[1], 4),
                                'yaw': round(pose[2], 4)}, f)
        except Exception:  # noqa: BLE001
            pass

    def _reply(self, text):
        msg = String()
        msg.data = text
        self.reply_pub.publish(msg)

    def _set_state(self, state, detail):
        self.state = state
        self.detail = detail
        self.publish_state()
        self.get_logger().info('任务状态=%s %s' % (state, detail))

    def publish_state(self):
        msg = MissionState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = self.state
        msg.detail = self.detail
        if self.target is not None:
            msg.target = self.target
        msg.home.header.frame_id = self.map_frame
        msg.home.header.stamp = msg.header.stamp
        if self.home is not None:
            msg.home.pose.position.x = float(self.home[0])
            msg.home.pose.position.y = float(self.home[1])
            msg.home.pose.orientation.z = math.sin(float(self.home[2]) / 2.0)
            msg.home.pose.orientation.w = math.cos(float(self.home[2]) / 2.0)
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()