#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""视觉抓取节点：调用 camera + arm，不直接碰相机设备或串口。

流程（沿用 ~/jetarm/grid_grasp.py 已验证的做法）：
    1. /arm/goto_pose photo
    2. 取最新帧，detect(hue) → 像素 → 底座坐标（height_mm 平面）
    3. 先张开夹爪，然后按 3x3 ±20mm + 外圈 ±40mm 网格逐个尝试：
       下探到 z_try → 合爪（堵转停位判定）→ 抬到 z=160 再读夹爪
       抬起后停位变化 ≤8 刻度才算夹稳
    4. 抓稳且 lift_after=true 时调用 /arm/carry 收拢到运输位

机械臂状态一律通过服务读取（/arm/read_state 或各运动服务的返回值），
不订阅 /arm/state：相机 10Hz 图像会让状态话题回调严重延迟。

服务：
    /grasp          抓取（会移动机械臂）
    /grasp/preview  只识别与解算，不移动
"""

import json
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_srvs.srv import Trigger

from jetarm_interfaces.srv import (Grasp, GraspPreview, Grip, MovePose,
                                   NamedPose, ReadState)

import jetarm_arm.driver as _driver_pkg  # 复用同一份 kinematics.py

DRIVER_DIR = os.path.dirname(os.path.abspath(_driver_pkg.__file__))
if DRIVER_DIR not in sys.path:
    sys.path.insert(0, DRIVER_DIR)

from jetarm_grasp.vision import (annotate, cam2base,  # noqa: E402
                                 detect, pixel_to_plane)

JOINT_ORDER = (1, 2, 3, 4, 5, 10)
GRID = [(0, 0), (20, 0), (-20, 0), (0, 20), (0, -20),
        (20, 20), (20, -20), (-20, 20), (-20, -20),
        (40, 0), (-40, 0), (0, 40), (0, -40)]


class GraspNode(Node):

    def __init__(self):
        super().__init__('grasp_node')

        arm_share = get_package_share_directory('jetarm_arm')
        cam_share = get_package_share_directory('jetarm_camera')
        self.declare_parameter('robot_file',
                               os.path.join(arm_share, 'config', 'robot.json'))
        self.declare_parameter('handeye_file',
                               os.path.join(cam_share, 'config', 'handeye.json'))
        self.declare_parameter('hue', 118)
        self.declare_parameter('hue_tol', 20)
        self.declare_parameter('height_mm', 70.0)
        self.declare_parameter('tool_deg', 170.0)
        self.declare_parameter('z_try_mm', 30.0)
        self.declare_parameter('z_lift_mm', 160.0)
        self.declare_parameter('min_area', 8000.0)
        self.declare_parameter('lift_delta_tick', 8.0)
        self.declare_parameter('photo_duration_ms', 2500)
        self.declare_parameter('frame_max_age_s', 1.0)

        self.robot = self._load_json(
            str(self.get_parameter('robot_file').value))
        self.handeye = self._load_json(
            str(self.get_parameter('handeye_file').value))
        self.k = np.array(self.handeye['camera_matrix'])
        self.dist = np.array(self.handeye['dist_coeffs'])
        self.open_pos = int(self.robot['joints']['10']['open'])
        self.close_pos = int(self.robot['joints']['10']['close'])

        self.frame = None
        self.frame_t = 0.0

        group = ReentrantCallbackGroup()
        self.group = group

        self.goto_cli = self.create_client(NamedPose, '/arm/goto_pose')
        self.move_cli = self.create_client(MovePose, '/arm/move_pose')
        self.grip_cli = self.create_client(Grip, '/arm/grip')
        self.carry_cli = self.create_client(Trigger, '/arm/carry')
        self.state_cli = self.create_client(ReadState, '/arm/read_state')

        self.pub_debug = self.create_publisher(
            CompressedImage, '/grasp/debug/compressed', 2)
        self.create_service(Grasp, '/grasp',
                            lambda rq, rs: self.on_grasp(rq, rs),
                            callback_group=group)
        self.create_service(GraspPreview, '/grasp/preview',
                            lambda rq, rs: self.on_preview(rq, rs),
                            callback_group=group)

        self.get_logger().info('grasp_node 启动（hue=%d，min_area=%.0f）'
                               % (self.get_parameter('hue').value,
                                  self.get_parameter('min_area').value))

    # ---------- 基础 ----------
    @staticmethod
    def _load_json(path):
        with open(path, encoding='utf-8') as f:
            return json.load(f)

    def _subscribe_image(self):
        self.frame = None
        self.frame_t = 0.0
        return self.create_subscription(
            CompressedImage, '/camera/image_raw/compressed',
            self.on_image, qos_profile_sensor_data, callback_group=self.group)

    def _unsubscribe_image(self, sub):
        if sub is not None:
            try:
                self.destroy_subscription(sub)
            except Exception:  # noqa: BLE001
                pass
        self.frame = None
        self.frame_t = 0.0

    def on_image(self, msg):
        self.frame = msg
        self.frame_t = time.monotonic()

    def _wait_frame(self, since, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.frame is not None and self.frame_t >= since:
                return self.frame
            time.sleep(0.05)
        return None

    def _call(self, client, request, wait=5.0):
        if not client.wait_for_service(timeout_sec=wait):
            raise RuntimeError('服务未就绪: %s' % client.srv_name)
        return client.call(request)

    @staticmethod
    def _ticks_of(state):
        ticks = list(state.tick)
        if len(ticks) != 6 or any(not np.isfinite(t) for t in ticks):
            return None
        return {sid: int(round(ticks[i])) for i, sid in enumerate(JOINT_ORDER)}

    def _read_ticks(self):
        last = '未知'
        for _ in range(3):
            res = self._call(self.state_cli, ReadState.Request())
            if res.ok:
                pose = self._ticks_of(res.state)
                if pose is not None:
                    return pose
                last = '关节读数缺失'
            else:
                last = res.message
            time.sleep(0.2)
        raise RuntimeError('读取机械臂状态失败: %s' % last)

    def _goto_photo(self):
        req = NamedPose.Request()
        req.name = 'photo'
        req.duration_ms = int(self.get_parameter('photo_duration_ms').value)
        res = self._call(self.goto_cli, req)
        if not res.ok:
            raise RuntimeError('到拍照位失败: %s' % res.message)
        time.sleep(0.6)
        return res

    def _move_pose(self, x, y, z, tool_deg, duration_ms=0, elbow='up'):
        req = MovePose.Request()
        req.x_mm, req.y_mm, req.z_mm = float(x), float(y), float(z)
        req.tool_deg = float(tool_deg)
        req.elbow = elbow
        req.duration_ms = int(duration_ms)
        return self._call(self.move_cli, req)

    def _grip(self, close, verified=True, target=0.0):
        req = Grip.Request()
        req.close = bool(close)
        req.verified = bool(verified)
        req.target_tick = float(target)
        return self._call(self.grip_cli, req)

    # ---------- 识别 ----------
    def _solve(self, frame, hue, height_mm, min_area, pose):
        """返回 (u, v, area, base_xy) 或 (None, 错误)。"""
        d = detect(frame, hue, tol=self.get_parameter('hue_tol').value,
                   min_area=min_area)
        if d is None:
            return None, '未检出目标'
        u, v, area = d
        r, t = cam2base(self.robot, self.handeye, pose)
        p = pixel_to_plane(u, v, self.k, self.dist, r, t, float(height_mm))
        if p is None:
            return None, '目标平面投影无效'
        return (u, v, area, (float(p[0]), float(p[1]))), ''

    def _annotated(self, frame, u, v, target_mm):
        data = annotate(frame, getattr(detect, 'contour', None), u, v, target_mm)
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'wrist_camera'
        msg.format = 'jpeg'
        msg.data = data
        self.pub_debug.publish(msg)
        return msg

    def _decode(self, frame_msg):
        return cv2.imdecode(np.frombuffer(bytes(frame_msg.data), np.uint8),
                            cv2.IMREAD_COLOR)

    # ---------- 服务：preview ----------
    def on_preview(self, req, res):
        sub = self._subscribe_image()
        try:
            frame_msg = self._wait_frame(time.monotonic() - 2.0, timeout=6.0)
            if frame_msg is None:
                res.ok, res.message = False, '没有相机图像'
                return res
            frame = self._decode(frame_msg)
            if frame is None:
                res.ok, res.message = False, '图像解码失败'
                return res
            hue = int(req.hue) if req.hue else int(self.get_parameter('hue').value)
            height = float(req.height_mm) if req.height_mm else float(
                self.get_parameter('height_mm').value)
            min_area = float(req.min_area) if req.min_area else float(
                self.get_parameter('min_area').value)
            pose = self._read_ticks()
            solved, err = self._solve(frame, hue, height, min_area, pose)
            if solved is None:
                res.ok, res.message = False, err
                return res
            u, v, area, target = solved
            res.ok = True
            res.message = '像素(%.0f,%.0f) 面积%.0f → 底座(%.1f,%.1f)mm' % (
                u, v, area, target[0], target[1])
            res.pixel_u, res.pixel_v, res.area = float(u), float(v), float(area)
            res.target_mm.x = float(target[0])
            res.target_mm.y = float(target[1])
            res.target_mm.z = float(height)
            res.annotated = self._annotated(frame, u, v, target)
            self.get_logger().info('preview: ' + res.message)
        except Exception as exc:  # noqa: BLE001
            res.ok, res.message = False, '预览异常: %s' % exc
        finally:
            self._unsubscribe_image(sub)
        return res

    # ---------- 服务：抓取 ----------
    def on_grasp(self, req, res):
        res.ok, res.message = False, ''
        res.attempts = 0
        sub = self._subscribe_image()
        try:
            hue = int(req.hue) if req.hue else int(self.get_parameter('hue').value)
            height = float(req.height_mm) if req.height_mm else float(
                self.get_parameter('height_mm').value)
            tool = float(req.tool_deg) if req.tool_deg else float(
                self.get_parameter('tool_deg').value)
            z_try = float(req.z_try_mm) if req.z_try_mm else float(
                self.get_parameter('z_try_mm').value)
            z_lift = float(self.get_parameter('z_lift_mm').value)
            min_area = float(req.min_area) if req.min_area else float(
                self.get_parameter('min_area').value)
            delta_tick = float(self.get_parameter('lift_delta_tick').value)

            self.get_logger().info('抓取开始：hue=%d height=%.0f tool=%.0f z=%.0f'
                                   % (hue, height, tool, z_try))
            photo = self._goto_photo()
            pose = self._ticks_of(photo.state)
            if pose is None:
                pose = self._read_ticks()

            since = time.monotonic()
            frame_msg = self._wait_frame(since)
            if frame_msg is None:
                res.message = '拍照位没有新图像'
                return res
            frame = self._decode(frame_msg)
            if frame is None:
                res.message = '图像解码失败'
                return res
            solved, err = self._solve(frame, hue, height, min_area, pose)
            if solved is None:
                res.message = err
                return res
            u, v, area, (mx, my) = solved
            self._annotated(frame, u, v, (mx, my))
            self.get_logger().info('视觉估计：像素(%.0f,%.0f) 面积%.0f → 底座(%.1f,%.1f)mm'
                                   % (u, v, area, mx, my))

            opened = self._grip(close=False, verified=True, target=self.open_pos)
            if not opened.ok:
                res.message = '夹爪张开失败: %s' % opened.message
                return res

            hit = None
            for index, (dx, dy) in enumerate(GRID):
                res.attempts = index + 1
                x, y = mx + dx, my + dy
                hover = self._move_pose(x, y, 120.0, tool)
                if not hover.ok:
                    self.get_logger().warn('偏移(%+d,%+d) 上方不可达: %s'
                                           % (dx, dy, hover.message))
                    continue
                down = self._move_pose(x, y, z_try, tool)
                if not down.ok:
                    self.get_logger().warn('偏移(%+d,%+d) 下探失败: %s'
                                           % (dx, dy, down.message))
                    continue
                grip = self._grip(close=True, verified=True, target=self.close_pos)
                contact = float(grip.final_tick)
                if not grip.ok or not grip.contact_candidate:
                    self.get_logger().info('偏移(%+d,%+d) 未检出接触（%s）'
                                           % (dx, dy, grip.message))
                    self._grip(close=False, verified=True, target=self.open_pos)
                    continue

                lift = self._move_pose(x, y, z_lift, tool)
                if not lift.ok:
                    self.get_logger().warn('抬升失败: %s' % lift.message)
                    self._grip(close=False, verified=True, target=self.open_pos)
                    continue
                after = int(round(list(lift.state.tick)[5]))
                if abs(after - contact) <= delta_tick:
                    hit = (x, y, contact, after)
                    self.get_logger().info('命中 偏移(%+d,%+d)：合爪 %d，抬升后 %d'
                                           % (dx, dy, contact, after))
                    break
                self.get_logger().info('偏移(%+d,%+d) 抬升后 %d（合爪 %d）→ 滑脱'
                                       % (dx, dy, after, contact))
                self._grip(close=False, verified=True, target=self.open_pos)

            if hit is None:
                res.message = '网格内都没有夹稳'
                self._goto_photo()
                return res

            res.target_mm.x, res.target_mm.y = float(hit[0]), float(hit[1])
            res.target_mm.z = float(z_try)
            res.final_tick = float(hit[3])
            if req.lift_after:
                carry = self._call(self.carry_cli, Trigger.Request(), wait=5.0)
                if not carry.success:
                    res.ok = False
                    res.message = '已夹住但收拢失败: %s' % carry.message
                    return res
                res.message = ('抓取成功：偏移(%+.0f,%+.0f)，合爪 %d，抬升后 %d，'
                               '已到运输位'
                               % (hit[0] - mx, hit[1] - my, hit[2], hit[3]))
            else:
                res.message = ('抓取成功：偏移(%+.0f,%+.0f)，合爪 %d，抬升后 %d'
                               % (hit[0] - mx, hit[1] - my, hit[2], hit[3]))
            res.ok = True
            self.get_logger().info(res.message)
        except Exception as exc:  # noqa: BLE001
            res.ok, res.message = False, '抓取异常: %s' % exc
            self.get_logger().error(res.message)
        finally:
            self._unsubscribe_image(sub)
        return res


def main(args=None):
    rclpy.init(args=args)
    node = GraspNode()
    executor = MultiThreadedExecutor(num_threads=3)
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