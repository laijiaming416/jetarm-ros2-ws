#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ASR 节点：私有 venv 里跑 SenseVoice，不 import 进 ROS 的 Python。

子进程协议（沿用 voice-dialog 的 asr_worker.py）：
    stdin  : LISTEN / RESUME
    事件管道: model_ready / listen_ready / text / transcript_rejected / audio_done

话题：
    /voice/text       VoiceText
    /voice/asr_state  AsrState
服务：
    /asr/listen  /asr/pause  /asr/resume   std_srvs/Trigger

注意：worker 每识别完一句会自己停采，等 RESUME 才继续。
"pause" 的含义是“暂不恢复收音”；"resume" 才真正写 RESUME。
"""

import json
import os
import select
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Empty
from std_srvs.srv import Trigger

from jetarm_interfaces.msg import AsrState, VoiceText

REMOVE_ENV = ('PYTHONPATH', 'PYTHONHOME', 'LD_LIBRARY_PATH', 'LD_PRELOAD',
              'AMENT_PREFIX_PATH', 'CMAKE_PREFIX_PATH', 'COLCON_PREFIX_PATH',
              'RMW_IMPLEMENTATION')


class AsrNode(Node):

    def __init__(self):
        super().__init__('asr_node')

        self.declare_parameter(
            'python', '/home/laijinxuan/file-asr-original-20261024-v1/.venv/bin/python')
        self.declare_parameter(
            'worker', os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                   'workers', 'asr_worker.py'))
        self.declare_parameter('threshold', 400.0)  # 0 = 自动；固定值不再受底噪标定影响
        self.declare_parameter('session_s', 0.0)   # 0 = 不限
        self.declare_parameter('auto_listen', True)
        self.declare_parameter('restart_delay_s', 2.0)
        self.declare_parameter('auto_resume_s', 2.0)

        self.python = str(self.get_parameter('python').value)
        self.worker = str(self.get_parameter('worker').value)
        self.threshold = float(self.get_parameter('threshold').value)
        self.session_s = float(self.get_parameter('session_s').value)
        self.auto_listen = bool(self.get_parameter('auto_listen').value)
        self.restart_delay = float(self.get_parameter('restart_delay_s').value)
        self.auto_resume = float(self.get_parameter('auto_resume_s').value)

        self.text_pub = self.create_publisher(VoiceText, '/voice/text', 10)
        self.state_pub = self.create_publisher(AsrState, '/voice/asr_state', 10)
        self.create_subscription(Empty, '/asr/resume_cmd', self.on_resume_msg, 10)
        self.create_service(Trigger, '/asr/listen', self.on_listen)
        self.create_service(Trigger, '/asr/pause', self.on_pause)
        self.create_service(Trigger, '/asr/resume', self.on_resume)

        self.proc = None
        self.ev_fd = None
        self.state = 'idle'
        self.detail = ''
        self.awaiting_resume = False
        self.hold = False
        self.paused_at = 0.0
        self.listening_sent = False
        self._stop = threading.Event()
        self._lock = threading.Lock()

        self.reader = threading.Thread(target=self._event_loop, daemon=True)
        self.reader.start()
        self.create_timer(1.0, self._publish_state)
        self.get_logger().info('asr_node 启动')

    # ---------- 子进程 ----------
    def _spawn(self):
        ev_r, ev_w = os.pipe()
        env = {k: v for k, v in os.environ.items() if k not in REMOVE_ENV}
        env.update(PYTHONDONTWRITEBYTECODE='1', PYTHONOPTIMIZE='0',
                   OMP_NUM_THREADS='4', OPENBLAS_NUM_THREADS='1',
                   PYTHONIOENCODING='utf-8')
        cmd = [self.python, '-B', self.worker, '--event-fd', str(ev_w),
               '--session-s', str(self.session_s)]
        if self.threshold > 0:
            cmd += ['--threshold', str(self.threshold)]
        self.get_logger().info('启动 ASR 子进程（模型加载约 30~90 秒）')
        proc = subprocess.Popen(cmd, stdin=subprocess.PIPE,
                                stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT,
                                pass_fds=(ev_w,), env=env,
                                start_new_session=True)
        os.close(ev_w)
        with self._lock:
            self.proc = proc
            self.ev_fd = ev_r
            self.awaiting_resume = False
            self.hold = False
            self.listening_sent = False
            self._set_state('loading', '模型加载中')
        threading.Thread(target=self._stdout_loop, args=(proc,),
                         daemon=True).start()

    def _stdout_loop(self, proc):
        try:
            for raw in iter(proc.stdout.readline, b''):
                line = raw.decode('utf-8', 'replace').rstrip()
                if line:
                    self.get_logger().info('[asr] ' + line)
        except Exception:  # noqa: BLE001
            pass

    def _event_loop(self):
        while not self._stop.is_set():
            if self.proc is None:
                self._spawn()
            fd = self.ev_fd
            if fd is None:
                time.sleep(0.2)
                continue
            try:
                ready, _, _ = select.select([fd], [], [], 0.5)
            except (OSError, ValueError):
                ready = []
            if ready:
                try:
                    chunk = os.read(fd, 65536)
                except OSError:
                    chunk = b''
                if not chunk:
                    self._on_exit()
                    continue
                for line in chunk.splitlines():
                    if not line.strip():
                        continue
                    try:
                        row = json.loads(line)
                    except ValueError:
                        continue
                    self._on_event(row)

    def _on_event(self, row):
        event = row.get('event')
        if event == 'model_ready':
            self._set_state('starting', '模型已加载')
            if self.auto_listen:
                self._send_stdin('LISTEN')
            else:
                self._set_state('paused', '等待 /asr/listen')
        elif event == 'listen_ready':
            self._set_state('listening', '')
        elif event == 'text':
            text = str(row.get('text', ''))
            # 必须先置标记再发布：对话节点收到文本后会立刻调 /asr/resume，
            # 先发布会让它看到“没有待恢复”，麦克风就一直停着。
            with self._lock:
                self.awaiting_resume = True
            msg = VoiceText()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.text = text
            self.text_pub.publish(msg)
            self._set_state('paused', '已识别：%s' % text)
            self.get_logger().info('识别：%s' % text)
        elif event == 'transcript_rejected':
            with self._lock:
                self.awaiting_resume = True
            self._set_state('paused', '丢弃片段：%s' % row.get('reason', ''))
        elif event == 'audio_done':
            self._set_state('done', '本轮会话结束')
            self._on_exit(restart=False)

    def _on_exit(self, restart=True):
        with self._lock:
            proc, self.proc = self.proc, None
            if self.ev_fd is not None:
                try:
                    os.close(self.ev_fd)
                except OSError:
                    pass
                self.ev_fd = None
        if proc is not None:
            try:
                if proc.poll() is None:
                    os.killpg(os.getpgid(proc.pid), 15)
            except Exception:  # noqa: BLE001
                pass
            try:
                proc.wait(timeout=3)
            except Exception:  # noqa: BLE001
                pass
        if restart and not self._stop.is_set():
            self._set_state('loading', '子进程退出，%.0f 秒后重启' % self.restart_delay)
            time.sleep(self.restart_delay)

    def _send_stdin(self, line):
        with self._lock:
            proc = self.proc
            if proc is None or proc.poll() is not None:
                return False
            try:
                proc.stdin.write((line + '\n').encode())
                proc.stdin.flush()
            except Exception:  # noqa: BLE001
                return False
            if line == 'LISTEN':
                self.listening_sent = True
            return True

    def _set_state(self, state, detail=''):
        if state == 'paused' and self.state != 'paused':
            self.paused_at = time.monotonic()
        self.state = state
        self.detail = detail
        self._publish_state()
        self.get_logger().info('ASR 状态=%s %s' % (state, detail))

    def _publish_state(self):
        # 看门狗：上游节点没来恢复时避免麦克风一直锁死
        if (self.state == 'paused' and self.auto_resume > 0
                and not self.hold
                and self.paused_at > 0
                and time.monotonic() - self.paused_at > self.auto_resume):
            self.get_logger().warn('暂停超过 %.0f 秒没有恢复，自动恢复收音'
                                   % self.auto_resume)
            self.paused_at = 0.0
            with self._lock:
                pending = self.awaiting_resume
            if pending and self._send_stdin('RESUME'):
                with self._lock:
                    self.awaiting_resume = False
                self.state = 'listening'
                self.detail = ''
        msg = AsrState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = self.state
        msg.detail = self.detail
        self.state_pub.publish(msg)

    # ---------- 服务 ----------
    def on_listen(self, req, res):
        del req
        ok = self._send_stdin('LISTEN')
        res.success = ok
        res.message = '已发送 LISTEN' if ok else 'ASR 子进程未就绪'
        return res

    def on_pause(self, req, res):
        del req
        self.hold = True
        # 真正让 worker 停止采集；等 RESUME 再打开
        self._send_stdin('PAUSE')
        res.success, res.message = True, '已暂停收音'
        return res

    def _do_resume(self):
        self.hold = False
        # 无条件发送 RESUME：既用于“说完一句后恢复”，也用于 TTS 播报后从
        # PAUSE 恢复；worker 在非暂停状态下会忽略多余的 RESUME。
        ok = self._send_stdin('RESUME')
        if ok:
            with self._lock:
                self.awaiting_resume = False
            self._set_state('listening', '')
        return ok, '已恢复收音' if ok else '写入 RESUME 失败'

    def on_resume_msg(self, msg):
        del msg
        ok, message = self._do_resume()
        if ok:
            self.get_logger().info('收到 /asr/resume_cmd：%s' % message)

    def on_resume(self, req, res):
        del req
        ok, message = self._do_resume()
        res.success, res.message = ok, message
        return res

    def shutdown(self):
        self._stop.set()
        self._on_exit(restart=False)


def main(args=None):
    rclpy.init(args=args)
    node = AsrNode()
    executor = MultiThreadedExecutor(num_threads=3)
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