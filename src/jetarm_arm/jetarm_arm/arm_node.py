#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""JetArm 机械臂 ROS 2 节点。

唯一打开机械臂串口的进程；其他节点通过服务使用机械臂。

硬件：
    控制器板 CH340（1 Mbaud），默认自动探测：
    只在 /dev/serial/by-path/* 与 /dev/ttyUSB* 上发送**读位置**帧，
    哪个口能读到舵机就用哪个（当前实测是 /dev/ttyUSB1）。

坐标与刻度：
    舵机刻度 0~1000 对应 0~240°；命名姿态用 robot.json 的机械限位校验，
    actions.json 的 safe_limits 只用于动作组。

服务：
    /arm/goto_pose     NamedPose   命名姿态 home / photo / transport
    /arm/home          Trigger     回 home
    /arm/photo         Trigger     到拍照位
    /arm/carry         Trigger     分段抬升 + 收拢到运输位（带反馈校验）
    /arm/move_joints   MoveJoints  直接给 6 个关节刻度
    /arm/move_pose     MovePose    逆运动学到 TCP
    /arm/grip          Grip        夹爪（可带堵转停位判定）
    /arm/play_action   PlayAction  播放 actions.json 动作组
    /arm/stop          Trigger     停止所有关节
    /arm/load_all      Trigger     全部锁力
    /arm/unload_all    Trigger     全部卸力
    /arm/beep          Trigger     蜂鸣

话题：
    /arm/state         ArmState    5Hz
    /arm/joint_state   JointState  5Hz（弧度，给通用工具用）
"""

import glob
import json
import math
import os
import sys
import threading
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from jetarm_interfaces.msg import ArmState
from jetarm_interfaces.srv import (Grip, MoveJoints, MovePose, NamedPose,
                                   PlayAction, ReadState)

# 原驱动目录：保持 ~/jetarm 的源码原样，不改动
DRIVER_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'driver')
if DRIVER_DIR not in sys.path:
    sys.path.insert(0, DRIVER_DIR)

import actions as ACT        # noqa: E402
import gripper_hold as GH    # noqa: E402
import jetarm as JA          # noqa: E402
import kinematics as KIN     # noqa: E402

JOINT_ORDER = (1, 2, 3, 4, 5, 10)
JOINT_NAMES = ('base', 'shoulder', 'elbow', 'wrist_pitch', 'wrist_roll',
               'gripper')


class LockedArm:
    """给机械臂对象的每次串口事务加锁。

    gripper_hold / actions 内部会多次调用驱动，逐次加锁既保证串口事务不交叉，
    又不会在一次 4 秒的夹爪动作里一直占着锁（状态发布仍能穿插读）。
    """

    def __init__(self, arm, lock):
        object.__setattr__(self, '_arm', arm)
        object.__setattr__(self, '_lock', lock)

    def __getattr__(self, name):
        attr = getattr(object.__getattribute__(self, '_arm'), name)
        if not callable(attr):
            return attr
        lock = object.__getattribute__(self, '_lock')

        def wrapped(*args, **kwargs):
            with lock:
                return attr(*args, **kwargs)
        return wrapped

    def __setattr__(self, name, value):
        setattr(object.__getattribute__(self, '_arm'), name, value)


class ArmNode(Node):

    def __init__(self):
        super().__init__('arm_node')

        share = get_package_share_directory('jetarm_arm')
        self.declare_parameter('port', 'auto')
        self.declare_parameter('baud', 1000000)
        self.declare_parameter('timeout', 0.4)
        self.declare_parameter('state_rate_hz', 5.0)
        self.declare_parameter('robot_file', os.path.join(share, 'config',
                                                          'robot.json'))
        self.declare_parameter('actions_file', os.path.join(share, 'config',
                                                            'actions.json'))
        self.declare_parameter('poses_file', os.path.join(share, 'config',
                                                          'arm_poses.yaml'))
        self.declare_parameter('max_tick_delta', 700.0)
        self.declare_parameter('verify_tolerance', 25.0)
        self.declare_parameter('named_pose_margin', 40.0)
        self.declare_parameter('ms_per_tick', 6.0)
        self.declare_parameter('min_move_ms', 800)
        self.declare_parameter('max_move_ms', 3500)
        self.declare_parameter('beep_times', 2)
        self.declare_parameter('beep_freq', 2000)

        self.baud = int(self.get_parameter('baud').value)
        self.timeout = float(self.get_parameter('timeout').value)
        self.max_delta = float(self.get_parameter('max_tick_delta').value)
        self.tol = float(self.get_parameter('verify_tolerance').value)
        self.pose_margin = float(self.get_parameter('named_pose_margin').value)
        self.ms_per_tick = float(self.get_parameter('ms_per_tick').value)
        self.min_ms = int(self.get_parameter('min_move_ms').value)
        self.max_ms = int(self.get_parameter('max_move_ms').value)
        self.verify_tol = self.tol

        self.robot = self._load_json(str(self.get_parameter('robot_file').value))
        self.actions_doc = self._load_json(
            str(self.get_parameter('actions_file').value))
        with open(str(self.get_parameter('poses_file').value),
                  encoding='utf-8') as f:
            self.poses = yaml.safe_load(f)
        self._check_poses()

        self.io_lock = threading.RLock()
        # 串行化整段机械臂操作，避免两个服务同时下发运动
        self.op_lock = threading.RLock()
        self.arm = None
        self.locked_arm = None
        self._loaded = False
        self._motion_deadline = 0.0
        self._last_connect_try = 0.0
        self._last_connect_log = 0.0
        self.error = ''

        self.state_pub = self.create_publisher(ArmState, '/arm/state', 10)
        self.joint_pub = self.create_publisher(JointState, '/arm/joint_state', 10)

        group = ReentrantCallbackGroup()
        self.create_service(Trigger, '/arm/home',
                            lambda rq, rs: self._locked(self.on_home, rq, rs),
                            callback_group=group)
        self.create_service(Trigger, '/arm/photo',
                            lambda rq, rs: self._locked(self.on_photo, rq, rs),
                            callback_group=group)
        self.create_service(Trigger, '/arm/carry',
                            lambda rq, rs: self._locked(self.on_carry, rq, rs),
                            callback_group=group)
        self.create_service(Trigger, '/arm/stop',
                            lambda rq, rs: self._locked(self.on_stop, rq, rs),
                            callback_group=group)
        self.create_service(Trigger, '/arm/load_all',
                            lambda rq, rs: self._locked(self.on_load_all, rq, rs),
                            callback_group=group)
        self.create_service(Trigger, '/arm/unload_all',
                            lambda rq, rs: self._locked(self.on_unload_all, rq, rs),
                            callback_group=group)
        self.create_service(Trigger, '/arm/beep',
                            lambda rq, rs: self._locked(self.on_beep, rq, rs),
                            callback_group=group)
        self.create_service(NamedPose, '/arm/goto_pose',
                            lambda rq, rs: self._locked(self.on_goto_pose, rq, rs),
                            callback_group=group)
        self.create_service(MoveJoints, '/arm/move_joints',
                            lambda rq, rs: self._locked(self.on_move_joints, rq, rs),
                            callback_group=group)
        self.create_service(MovePose, '/arm/move_pose',
                            lambda rq, rs: self._locked(self.on_move_pose, rq, rs),
                            callback_group=group)
        self.create_service(Grip, '/arm/grip',
                            lambda rq, rs: self._locked(self.on_grip, rq, rs),
                            callback_group=group)
        self.create_service(PlayAction, '/arm/play_action',
                            lambda rq, rs: self._locked(self.on_play_action, rq, rs),
                            callback_group=group)
        self.create_service(ReadState, '/arm/read_state',
                            lambda rq, rs: self._locked(self.on_read_state, rq, rs),
                            callback_group=group)
        self.create_service(ReadState, '/arm/read_state',
                            lambda rq, rs: self._locked(self.on_read_state, rq, rs),
                            callback_group=group)

        rate = float(self.get_parameter('state_rate_hz').value)
        self.create_timer(1.0 / max(0.5, rate), self.publish_state)

        self.get_logger().info('arm_node 启动，串口参数=%s'
                               % self.get_parameter('port').value)
        if not self._connect():
            self.get_logger().error(
                '机械臂串口连接失败，将在后台每 2 秒重试；'
                '检查控制器电源与 USB 后无需重启节点')

    # ---------- 配置 ----------
    @staticmethod
    def _load_json(path):
        with open(path, encoding='utf-8') as f:
            return json.load(f)

    def _check_poses(self):
        """命名姿态校验。

        photo 是用户 2026-11-04 确认并实际使用的姿态，其中 ID2=646 比
        robot.json 的保守上限 613 高 33 刻度。因此命名姿态允许
        named_pose_margin 的余量：超出严格限位只告警，超出余量才拒绝。
        """
        limits = {int(k): v for k, v in self.robot.get('limits', {}).items()}
        margin = float(self.get_parameter('named_pose_margin').value)

        def check(label, sid, val):
            lo, hi = limits.get(sid, (0, 1000))
            if lo - margin <= int(val) <= hi + margin:
                if not lo <= int(val) <= hi:
                    self.get_logger().warn(
                        '%s 的 ID%d=%d 超出机械限位 %d~%d，'
                        '但在命名姿态余量 %.0f 内，按用户确认值使用'
                        % (label, sid, val, lo, hi, margin))
                return
            raise RuntimeError(
                '%s 的 ID%d=%d 超出机械限位 %d~%d（余量 %.0f）'
                % (label, sid, val, lo, hi, margin))

        for name in ('home', 'photo'):
            pos = self.poses.get(name)
            if not pos:
                raise RuntimeError('arm_poses.yaml 缺少 %s' % name)
            for sid, val in zip(JOINT_ORDER, pos):
                check('命名姿态 %s' % name, sid, val)
        tr = self.poses.get('transport') or {}
        if not tr.get('positions'):
            raise RuntimeError('arm_poses.yaml 缺少 transport.positions')
        for sid, val in zip((1, 2, 3, 4), tr['positions']):
            check('运输位', sid, val)

    def _pose_limits(self):
        """命名姿态用的限位：机械限位 + named_pose_margin。"""
        margin = int(self.pose_margin)
        return {int(k): (v[0] - margin, v[1] + margin)
                for k, v in self.robot['limits'].items()}

    # ---------- 连接 ----------
    def _candidates(self):
        cands = []
        for pat in ('/dev/serial/by-path/*', '/dev/ttyUSB*'):
            cands.extend(sorted(glob.glob(pat)))
        seen, out = set(), []
        for p in cands:
            real = os.path.realpath(p)
            if real in seen:
                continue
            seen.add(real)
            out.append(p)
        return out

    def _connect(self):
        with self.op_lock:
            if self.arm is not None:
                return True
            port = str(self.get_parameter('port').value)
            if port != 'auto':
                return self._open(port)
            for cand in self._candidates():
                if self._open(cand, quiet=True):
                    return True
            return False

    def _open(self, port, quiet=False):
        for attempt in range(3):
            try:
                arm = JA.JetArm(port=port, baud=self.baud, timeout=self.timeout)
            except Exception as exc:  # noqa: BLE001
                if attempt == 2:
                    if not quiet:
                        self.get_logger().error('打开 %s 失败: %s' % (port, exc))
                    return False
                time.sleep(0.3)
                continue
            self.arm = arm
            self.locked_arm = LockedArm(arm, self.io_lock)
            self.get_logger().info('机械臂已连接: %s（%d baud）' % (port, self.baud))
            self._ensure_loaded()
            return True
        return False

    def _need_arm(self):
        if self.arm is None:
            now = time.monotonic()
            if now - self._last_connect_try > 2.0:
                self._last_connect_try = now
                if self._connect():
                    return True
                if now - self._last_connect_log > 10.0:
                    self._last_connect_log = now
                    self.get_logger().warn('机械臂仍未连接，继续重试')
            return False
        return True

    # ---------- 基础操作 ----------
    def _locked(self, fn, req, res):
        with self.op_lock:
            return fn(req, res)

    def _read_positions(self):
        if not self._need_arm():
            return {sid: None for sid in JOINT_ORDER}
        out = {}
        for sid in JOINT_ORDER:
            with self.io_lock:
                out[sid] = self.arm.read_position(sid)
        return out

    def _auto_duration(self, targets, cur):
        deltas = [abs(int(v) - int(cur[s])) for s, v in targets.items()
                  if cur.get(s) is not None]
        if not deltas:
            return self.min_ms
        dur = int(max(deltas) * self.ms_per_tick)
        return max(self.min_ms, min(self.max_ms, dur))

    def _check_limits(self, targets, limits):
        problems = []
        for sid, val in targets.items():
            lo, hi = limits.get(sid, (0, 1000))
            if not lo <= int(val) <= hi:
                problems.append('ID%d=%d 超出 %d~%d' % (sid, val, lo, hi))
        return problems

    def _check_delta(self, targets, cur):
        problems = []
        if self.max_delta <= 0:
            return problems
        for sid, val in targets.items():
            if cur.get(sid) is None:
                continue
            delta = abs(int(val) - int(cur[sid]))
            if delta > self.max_delta:
                problems.append('ID%d 单次位移 %d 超过上限 %d'
                                % (sid, delta, int(self.max_delta)))
        return problems

    def _move(self, targets, duration_ms, check_limits=True, limits=None):
        """下发运动并等待完成，返回 (ok, message, state)。"""
        with self.op_lock:
            return self._move_locked(targets, duration_ms, check_limits, limits)

    def _move_locked(self, targets, duration_ms, check_limits=True, limits=None):
        if not self._need_arm():
            return False, '机械臂未连接', self._make_state(self._empty_positions())
        cur = self._read_positions()
        targets = {int(k): int(v) for k, v in targets.items()}
        if check_limits:
            limits = limits if limits is not None else {
                int(k): v for k, v in self.robot['limits'].items()}
            problems = self._check_limits(targets, limits)
            if problems:
                return False, '拒绝：' + '；'.join(problems), self._make_state(cur)
        problems = self._check_delta(targets, cur)
        if problems:
            return False, '拒绝：' + '；'.join(problems), self._make_state(cur)

        dur = int(duration_ms) if duration_ms else self._auto_duration(targets, cur)
        dur = max(self.min_ms, min(self.max_ms, dur))
        with self.io_lock:
            self.arm.move(targets, dur)
        self._motion_deadline = time.monotonic() + dur / 1000.0
        time.sleep(dur / 1000.0 + 0.15)

        # 固件插值结束到舵机真正停稳还有滞后，最多再等 1 秒
        deadline = time.monotonic() + 1.0
        actual = {}
        while True:
            bad = []
            for sid in targets:
                with self.io_lock:
                    val = self.arm.read_position(sid)
                actual[sid] = val
                if val is None:
                    bad.append('ID%d 读不到' % sid)
                elif abs(int(val) - targets[sid]) > self.verify_tol:
                    bad.append('ID%d 实际 %d 目标 %d' % (sid, val, targets[sid]))
            if not bad or time.monotonic() >= deadline:
                break
            time.sleep(0.1)
        self._motion_deadline = 0.0
        state = self._make_state(self._merge_positions(cur, actual))
        if bad:
            return False, '运动到位校验失败：' + '；'.join(bad), state
        return True, '完成（%d ms）' % dur, state

    @staticmethod
    def _merge_positions(cur, actual):
        out = dict(cur)
        out.update({k: v for k, v in actual.items() if v is not None})
        return out

    @staticmethod
    def _empty_positions():
        return {sid: None for sid in JOINT_ORDER}

    # ---------- 状态 ----------
    def _make_state(self, pos):
        msg = ArmState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'arm_base'
        ticks, degs = [], []
        valid = True
        for sid in JOINT_ORDER:
            val = pos.get(sid)
            if val is None:
                valid = False
                ticks.append(float('nan'))
                degs.append(0.0)
                continue
            ticks.append(float(val))
            try:
                degs.append(float(KIN.pos_to_deg(self.robot, sid, int(val))))
            except Exception:  # noqa: BLE001
                degs.append(0.0)
        msg.tick = ticks
        msg.deg = degs
        if valid and all(pos.get(sid) is not None for sid in (1, 2, 3, 4)):
            try:
                tcp = KIN.fk(self.robot, pos)['tcp']
                msg.tcp_mm = Point(x=float(tcp[0]), y=float(tcp[1]), z=float(tcp[2]))
            except Exception:  # noqa: BLE001
                msg.tcp_mm = Point()
        else:
            msg.tcp_mm = Point()
        msg.loaded = bool(self._loaded)
        msg.moving = time.monotonic() < self._motion_deadline
        msg.error = self.error
        return msg

    def publish_state(self):
        if self.arm is None and not self._need_arm():
            self.state_pub.publish(self._make_state(self._empty_positions()))
            return
        pos = self._read_positions()
        state = self._make_state(pos)
        self.state_pub.publish(state)

        js = JointState()
        js.header = state.header
        js.name = list(JOINT_NAMES)
        js.position = [math.radians(d) for d in state.deg]
        self.joint_pub.publish(js)

    def on_read_state(self, req, res):
        del req
        pos = self._read_positions()
        res.state = self._make_state(pos)
        res.ok = True
        res.message = ''
        if any(pos.get(sid) is None for sid in JOINT_ORDER):
            res.ok = False
            res.message = '部分关节读数缺失'
        return res

    def on_read_state(self, req, res):
        del req
        pos = self._read_positions()
        res.state = self._make_state(pos)
        res.ok = not any(pos.get(sid) is None for sid in JOINT_ORDER)
        res.message = '' if res.ok else '部分关节读数缺失'
        return res

    # ---------- 命名姿态 ----------
    def _named_targets(self, name):
        if name == 'transport':
            tr = self.poses['transport']
            cur = self._read_positions()
            targets = {int(s): int(v) for s, v in
                       zip((1, 2, 3, 4), tr['positions'])}
            for sid in tr.get('preserve', [5, 10]):
                if cur.get(int(sid)) is None:
                    return None, '运输位需要保留 ID%s，但读不到当前位置' % sid
                targets[int(sid)] = int(cur[int(sid)])
            return targets, ''
        pos = self.poses.get(name)
        if not pos:
            return None, '未知命名姿态: %s' % name
        return {sid: int(v) for sid, v in zip(JOINT_ORDER, pos)}, ''

    def _goto_named(self, name, duration_ms=0):
        targets, err = self._named_targets(name)
        if targets is None:
            return False, err, self._make_state(self._read_positions())
        limits = self._pose_limits()
        ok, msg, state = self._move(targets, duration_ms, limits=limits)
        if ok:
            msg = '%s：%s' % (name, msg)
        return ok, msg, state

    def on_goto_pose(self, req, res):
        ok, msg, state = self._goto_named(req.name, req.duration_ms)
        res.ok, res.message, res.state = ok, msg, state
        return res

    def on_home(self, req, res):
        del req
        ok, msg, state = self._goto_named('home')
        res.success, res.message = ok, msg
        self.get_logger().info('home: %s %s' % (ok, msg))
        del state
        return res

    def on_photo(self, req, res):
        del req
        ok, msg, _ = self._goto_named('photo')
        res.success, res.message = ok, msg
        self.get_logger().info('photo: %s %s' % (ok, msg))
        return res

    def on_carry(self, req, res):
        del req
        ok, msg = self._carry()
        res.success, res.message = ok, msg
        self.get_logger().info('carry: %s %s' % (ok, msg))
        return res

    def _carry(self):
        """两段收拢到运输位：先关节空间折半，再完全收拢。

        运输位文件要求先离桌升高、再收拢。原来的做法是按当前 TCP 抬高
        50mm 求 IK，但物体在 25cm 外时抬高后不可达；这里改成关节空间
        两段插值（两端都在限位内，中间点必然可达），每段做反馈校验，
        并监测夹爪停位是否变化（判断是否滑脱）。
        """
        if not self._need_arm():
            return False, '机械臂未连接'
        cur = self._read_positions()
        if any(cur.get(sid) is None for sid in JOINT_ORDER):
            return False, '读取当前姿态失败'
        retract, err = self._named_targets('transport')
        if retract is None:
            return False, err

        deltas = [abs(retract[s] - cur[s]) for s in (1, 2, 3, 4)]
        if max(deltas) < 20:
            return True, '已在运输位'

        limits = self._pose_limits()
        mid = {s: int(round((cur[s] + retract[s]) / 2.0)) for s in (1, 2, 3, 4)}
        mid[5] = int(cur[5])
        mid[10] = int(cur[10])

        hold = int(cur[10])
        ok, msg, state = self._move(mid, 1500, limits=limits)
        if not ok:
            return False, '收拢第一段失败：' + msg
        now = self._read_servo(10)
        if now is None or abs(now - hold) > 8:
            return False, ('收拢第一段后夹爪停位 %s（原 %d）变化过大，'
                           '可能滑脱，已停下' % (now, hold))

        ok, msg, state = self._move(retract, 1800, limits=limits)
        if not ok:
            return False, '收拢第二段失败：' + msg
        now = self._read_servo(10)
        if now is None or abs(now - hold) > 8:
            return False, ('收拢第二段后夹爪停位 %s（原 %d）变化过大，'
                           '可能滑脱，已停下' % (now, hold))

        self._last_carry = {
            'mid_targets': mid,
            'retract_targets': retract,
            'gripper_tick': now,
        }
        return True, '已到运输位（夹爪 %d）' % now

    # ---------- 关节 / 位姿 ----------
    def on_move_joints(self, req, res):
        raw = list(req.tick)
        cur = self._read_positions()
        targets = {}
        for sid, val in zip(JOINT_ORDER, raw):
            if val is None or math.isnan(float(val)):
                continue
            if req.relative:
                if cur.get(sid) is None:
                    res.ok, res.message = False, 'ID%d 当前读数缺失，无法增量' % sid
                    res.state = self._make_state(cur)
                    return res
                targets[sid] = int(cur[sid] + float(val))
            else:
                targets[sid] = int(round(float(val)))
        if not targets:
            res.ok, res.message = False, '没有有效的关节目标'
            res.state = self._make_state(cur)
            return res
        ok, msg, state = self._move(targets, req.duration_ms)
        res.ok, res.message, res.state = ok, msg, state
        return res

    def on_move_pose(self, req, res):
        cur = self._read_positions()
        elbow = (req.elbow or 'auto').lower()
        choices = ('up', 'down') if elbow == 'auto' else (elbow,)
        best = None
        for name in choices:
            sol = KIN.ik(self.robot, float(req.x_mm), float(req.y_mm),
                         float(req.z_mm), tool_deg=float(req.tool_deg),
                         elbow=name)
            if not sol['reachable'] or sol['problems']:
                continue
            cost = sum(abs(sol['pos'][s] - (cur.get(s) or sol['pos'][s]))
                       for s in (1, 2, 3, 4))
            if best is None or cost < best[0]:
                best = (cost, sol, name)
        if best is None:
            res.ok = False
            res.message = ('目标 TCP %.0f/%.0f/%.0f tool=%.0f 不可达或超限'
                           % (req.x_mm, req.y_mm, req.z_mm, req.tool_deg))
            res.state = self._make_state(cur)
            return res
        targets = {int(s): int(v) for s, v in best[1]['pos'].items()}
        ok, msg, state = self._move(targets, req.duration_ms,
                                    limits=self._pose_limits())
        res.ok, res.message, res.state = ok, ('%s 解：%s' % (best[2], msg)), state
        return res

    # ---------- 夹爪 ----------
    def _ensure_loaded(self):
        if self.arm is None:
            return False
        if self._loaded:
            return True
        try:
            for sid in JOINT_ORDER:
                with self.io_lock:
                    self.arm.load(sid)
                time.sleep(0.03)
            self._loaded = True
            return True
        except Exception as exc:  # noqa: BLE001
            self.error = 'load_failed: %s' % exc
            return False

    def on_grip(self, req, res):
        res.ok, res.message = False, ''
        res.final_tick = 0.0
        res.contact_candidate = False
        if not self._need_arm():
            res.message = '机械臂未连接'
            return res
        if not self._ensure_loaded():
            res.message = '锁力失败，无法操作夹爪'
            return res
        joints = self.robot['joints']['10']
        open_pos = int(joints['open'])
        close_pos = int(joints['close'])
        target = int(req.target_tick) if req.target_tick and req.target_tick > 0 \
            else (close_pos if req.close else open_pos)
        try:
            if not req.close:
                if req.verified:
                    final = GH.open_verified(self.locked_arm, target)
                else:
                    with self.io_lock:
                        self.arm.move({10: target}, 1200)
                    time.sleep(1.4)
                    final = self._read_servo(10)
                res.ok, res.message = True, '张开到 %s' % final
            else:
                if req.verified:
                    contact, reason = GH.close_and_hold(
                        self.locked_arm, open_pos, target)
                    res.contact_candidate = reason is None
                    res.ok = True
                    res.message = ('闭合停位 %d，%s'
                                   % (contact, '检出接触候选' if reason is None
                                      else '未检出接触（%s）' % reason))
                    final = contact
                else:
                    with self.io_lock:
                        self.arm.move({10: target}, 1200)
                    time.sleep(1.4)
                    final = self._read_servo(10)
                    res.ok, res.message = True, '闭合到 %s' % final
            res.final_tick = float(final if final is not None else 0)
        except GH.GripFault as exc:
            res.ok, res.message = False, '夹爪故障: %s' % exc
            res.final_tick = float(self._read_servo(10) or 0)
        except Exception as exc:  # noqa: BLE001
            res.ok, res.message = False, '夹爪异常: %s' % exc
        return res

    def _read_servo(self, sid):
        with self.io_lock:
            return self.arm.read_position(sid)

    # ---------- 动作组 ----------
    def on_play_action(self, req, res):
        name = req.name
        doc = self.actions_doc
        if name not in doc.get('actions', {}):
            res.ok, res.message = False, '没有动作 %s' % name
            return res
        problems = ACT.validate(doc, name)
        if problems:
            res.ok, res.message = False, '拒绝播放：' + '；'.join(problems)
            return res
        if not self._ensure_loaded():
            res.ok, res.message = False, '锁力失败，无法播放动作组'
            return res
        frames = doc['actions'][name]['frames']
        limits = {int(k): v for k, v in doc['safe_limits'].items()}
        speed = float(req.speed)
        loops = 3 if req.loop else 1
        try:
            for _ in range(loops):
                for frame in frames:
                    targets = {int(k): int(v) for k, v in frame['pos'].items()}
                    for sid in targets:
                        lo, hi = limits.get(sid, (0, 1000))
                        targets[sid] = max(lo, min(hi, targets[sid]))
                    nominal = int(frame.get('duration', 600))
                    if speed > 0:
                        dur = max(280, int(nominal * speed))
                    else:
                        dur = int(max(280, min(2000, nominal)))
                    with self.io_lock:
                        self.arm.move(targets, dur)
                    self._motion_deadline = time.monotonic() + dur / 1000.0
                    time.sleep(dur / 1000.0 + 0.02)
                if not req.loop:
                    break
            res.ok, res.message = True, '已播放 %s（%d 帧）' % (name, len(frames))
        except Exception as exc:  # noqa: BLE001
            res.ok, res.message = False, '播放失败: %s' % exc
        return res

    # ---------- 其它 ----------
    def on_stop(self, req, res):
        del req
        if not self._need_arm():
            res.success, res.message = False, '机械臂未连接'
            return res
        try:
            with self.io_lock:
                self.arm.stop(list(JOINT_ORDER))
            self._motion_deadline = 0.0
            res.success, res.message = True, '已停止'
        except Exception as exc:  # noqa: BLE001
            res.success, res.message = False, '停止失败: %s' % exc
        return res

    def on_load_all(self, req, res):
        del req
        if not self._need_arm():
            res.success, res.message = False, '机械臂未连接'
            return res
        self._loaded = False
        ok = self._ensure_loaded()
        res.success, res.message = ok, '已全部锁力' if ok else '锁力失败'
        return res

    def on_unload_all(self, req, res):
        del req
        if not self._need_arm():
            res.success, res.message = False, '机械臂未连接'
            return res
        try:
            for sid in JOINT_ORDER:
                with self.io_lock:
                    self.arm.unload(sid)
                time.sleep(0.03)
            self._loaded = False
            res.success, res.message = True, '已全部卸力'
        except Exception as exc:  # noqa: BLE001
            res.success, res.message = False, '卸力失败: %s' % exc
        return res

    def on_beep(self, req, res):
        del req
        if not self._need_arm():
            res.success, res.message = False, '机械臂未连接'
            return res
        try:
            with self.io_lock:
                self.arm.beep(int(self.get_parameter('beep_times').value),
                              int(self.get_parameter('beep_freq').value))
            res.success, res.message = True, '已蜂鸣'
        except Exception as exc:  # noqa: BLE001
            res.success, res.message = False, '蜂鸣失败: %s' % exc
        return res

    # ---------- 退出 ----------
    def shutdown(self):
        if self.arm is None:
            return
        try:
            with self.io_lock:
                self.arm.stop(list(JOINT_ORDER))
        except Exception:  # noqa: BLE001
            pass
        try:
            with self.io_lock:
                self.arm.close()
        except Exception:  # noqa: BLE001
            pass


def main(args=None):
    rclpy.init(args=args)
    node = ArmNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()