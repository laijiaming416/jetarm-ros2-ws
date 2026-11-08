#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""TTS 节点：Matcha 子进程协议 + aplay 顺序播放。

子进程（prompt_audio.py --worker）协议：
    stdin  : {"op":"speak","text":"..."} / {"op":"shutdown"}
    stdout : {"event":"tts_ready"} / {"event":"tts_chunk","path":...} / {"event":"tts_done"}

话题：
    /voice/reply   std_msgs/String   要播报的文本
    /tts/state     TtsState
    /tts/done      std_msgs/Empty    一段播报完成
服务：
    /tts/cue   播放 80ms 提示音（唤醒用）
    /tts/stop  停止当前播放

播报开始前调用 /asr/pause，结束后（默认 350ms 尾巴）调用 /asr/resume，
避免麦克风把播报内容再听一遍。
"""

import io
import json
import math
import os
import struct
import subprocess
import threading
import time
import wave

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger

from jetarm_interfaces.msg import TtsState

REMOVE_ENV = ('PYTHONPATH', 'PYTHONHOME', 'LD_LIBRARY_PATH', 'LD_PRELOAD',
              'AMENT_PREFIX_PATH', 'CMAKE_PREFIX_PATH', 'COLCON_PREFIX_PATH',
              'RMW_IMPLEMENTATION')


def ready_tone():
    rate = 16000
    count = 1280
    pcm = bytearray()
    for i in range(count):
        fade = min(1.0, i / 160.0, (count - 1 - i) / 160.0)
        pcm.extend(struct.pack('<h', int(9000 * fade *
                                         math.sin(2 * math.pi * 880 * i / rate))))
    out = io.BytesIO()
    with wave.open(out, 'wb') as w:
        w.setparams((1, 2, rate, 0, 'NONE', 'not compressed'))
        w.writeframes(pcm)
    return out.getvalue()


class TtsNode(Node):

    def __init__(self):
        super().__init__('tts_node')

        self.declare_parameter(
            'python', '/home/laijinxuan/file-asr-original-20261024-v1/.venv/bin/python')
        self.declare_parameter('tts_dir', '/home/laijinxuan/jetarm_tts_assets')
        self.declare_parameter('device', 'plughw:CARD=Device,DEV=0')
        self.declare_parameter('cue_after_speech', False)
        self.declare_parameter('resume_delay_s', 0.35)
        self.declare_parameter('startup_timeout_s', 150.0)

        self.python = str(self.get_parameter('python').value)
        self.tts_dir = str(self.get_parameter('tts_dir').value)
        self.device = str(self.get_parameter('device').value)
        self.cue_after_speech = bool(
            self.get_parameter('cue_after_speech').value)
        self.resume_delay = float(self.get_parameter('resume_delay_s').value)
        self.startup_timeout = float(
            self.get_parameter('startup_timeout_s').value)

        self.state_pub = self.create_publisher(TtsState, '/tts/state', 10)
        self.done_pub = self.create_publisher(Empty, '/tts/done', 10)
        self.create_subscription(String, '/voice/reply', self.on_reply, 10)
        self.create_subscription(Empty, '/voice/cue', self.on_cue_msg, 10)
        self.create_service(Trigger, '/tts/cue', self.on_cue)
        self.create_service(Trigger, '/tts/stop', self.on_stop)
        self.asr_pause = self.create_client(Trigger, '/asr/pause')
        self.asr_resume = self.create_client(Trigger, '/asr/resume')

        self.proc = None
        self.ready = False
        self.state = 'loading'
        self.text = ''
        self.events = []
        self.events_lock = threading.Condition()
        self.texts = []
        self.texts_lock = threading.Condition()
        self.play_lock = threading.Lock()
        self.current_play = None
        self._stop = threading.Event()

        threading.Thread(target=self._spawn_and_read, daemon=True).start()
        threading.Thread(target=self._playback_loop, daemon=True).start()
        self.create_timer(1.0, self._publish_state)
        self.get_logger().info('tts_node 启动（模型预热约 31 秒）')

    # ---------- 子进程 ----------
    def _spawn_and_read(self):
        env = {k: v for k, v in os.environ.items() if k not in REMOVE_ENV}
        env.update(PYTHONDONTWRITEBYTECODE='1', PYTHONOPTIMIZE='0',
                   OMP_NUM_THREADS='4', OPENBLAS_NUM_THREADS='1',
                   PYTHONIOENCODING='utf-8')
        script = os.path.join(self.tts_dir, 'prompt_audio.py')
        cmd = [self.python, '-B', script, '--worker']
        while not self._stop.is_set():
            try:
                self.proc = subprocess.Popen(
                    cmd, cwd=self.tts_dir, stdin=subprocess.PIPE,
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    env=env, start_new_session=True)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error('启动 TTS 子进程失败: %s' % exc)
                time.sleep(3.0)
                continue
            self._set_state('loading', '模型预热中')
            for raw in iter(self.proc.stdout.readline, b''):
                if self._stop.is_set():
                    break
                line = raw.decode('utf-8', 'replace').strip()
                if not line:
                    continue
                try:
                    row = json.loads(line)
                except ValueError:
                    self.get_logger().warn('[tts] ' + line[:200])
                    continue
                if row.get('event') == 'tts_ready':
                    self.ready = True
                    self._set_state('idle', '模型就绪 %.1fs' % row.get('load_s', 0))
                self._push_event(row)
            self.ready = False
            code = None if self.proc is None else self.proc.poll()
            self.get_logger().error('TTS 子进程退出（code=%s），3 秒后重启' % code)
            self._set_state('loading', '子进程退出，3 秒后重启')
            time.sleep(3.0)

    def _push_event(self, row):
        with self.events_lock:
            self.events.append(row)
            self.events_lock.notify_all()

    def _next_event(self, timeout):
        deadline = time.monotonic() + timeout
        with self.events_lock:
            while not self.events:
                left = deadline - time.monotonic()
                if left <= 0:
                    return None
                self.events_lock.wait(left)
            return self.events.pop(0)

    # ---------- 状态 ----------
    def _set_state(self, state, text=''):
        self.state = state
        self.text = text
        self._publish_state()

    def _publish_state(self):
        msg = TtsState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = self.state
        msg.text = self.text
        self.state_pub.publish(msg)

    # ---------- 播报 ----------
    def on_reply(self, msg):
        text = str(msg.data).strip()
        if not 1 <= len(text) <= 128:
            self.get_logger().warn('忽略无效播报文本（长度 %d）' % len(text))
            return
        with self.texts_lock:
            self.texts.append(text)
            self.texts_lock.notify_all()

    def _playback_loop(self):
        while not self._stop.is_set():
            with self.texts_lock:
                while not self.texts:
                    self.texts_lock.wait(0.5)
                    if self._stop.is_set():
                        return
                text = self.texts.pop(0)
            try:
                self._speak(text)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error('播报失败: %s' % exc)
                self._set_state('error', str(exc))
                self._call_asr(self.asr_resume)

    def _speak(self, text):
        if not self.ready or self.proc is None:
            self.get_logger().warn('TTS 未就绪，等待模型加载')
            deadline = time.monotonic() + self.startup_timeout
            while time.monotonic() < deadline and not self.ready:
                time.sleep(0.2)
            if not self.ready:
                raise RuntimeError('TTS 模型加载超时')
        with self.play_lock:
            self._call_asr(self.asr_pause)
            self._set_state('speaking', text)
            payload = (json.dumps({'op': 'speak', 'text': text},
                                  ensure_ascii=False) + '\n').encode('utf-8')
            self.proc.stdin.write(payload)
            self.proc.stdin.flush()
            started = time.monotonic()
            chunks = 0
            while True:
                row = self._next_event(1.0)
                if row is None:
                    if time.monotonic() - started > 120:
                        raise TimeoutError('TTS 合成超时')
                    continue
                event = row.get('event')
                if event == 'tts_chunk':
                    path = row.get('path')
                    if path:
                        self._play_file(path)
                        chunks += 1
                elif event == 'tts_done':
                    break
                elif event == 'tts_ready':
                    continue
            if self.cue_after_speech:
                self._play_cue()
            self._set_state('idle', '已播报 %d 段' % chunks)
            time.sleep(self.resume_delay)
            self._call_asr(self.asr_resume)
            self.done_pub.publish(Empty())
            self.get_logger().info('播报完成（%d 段）：%s' % (chunks, text))

    def _play_file(self, path):
        # plughw 直连偶尔会被 PipeWire / 其他客户端瞬时占用（rc=1，0.02s），
        # 失败就换 default，再失败重试一轮；stderr 记下来便于排查。
        last = ''
        for attempt in range(2):
            for device in (self.device, 'default'):
                started = time.monotonic()
                proc = subprocess.Popen(
                    ['aplay', '-q', '-D', device, str(path)],
                    stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
                    start_new_session=True)
                self.current_play = proc
                try:
                    _, err = proc.communicate()
                finally:
                    self.current_play = None
                elapsed = time.monotonic() - started
                if proc.returncode == 0:
                    self.get_logger().info(
                        'aplay %s dev=%s rc=0 %.2fs'
                        % (os.path.basename(str(path)), device, elapsed))
                    return
                last = err.decode('utf-8', 'replace').strip()[-200:]
                self.get_logger().warn(
                    'aplay 失败 dev=%s rc=%s %.2fs: %s'
                    % (device, proc.returncode, elapsed, last))
                time.sleep(0.2)
        raise RuntimeError('aplay 播放失败: %s' % last)

    def on_cue_msg(self, msg):
        del msg
        try:
            self._play_cue()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn('提示音失败: %s' % exc)

    def _play_cue(self):
        if self.current_play is not None:
            return
        path = os.path.join(self.tts_dir, 'cache', 'cue.wav')
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, 'wb') as f:
            f.write(ready_tone())
        self._play_file(path)

    def _call_asr(self, client):
        try:
            if client.wait_for_service(timeout_sec=1.0):
                client.call(Trigger.Request())
        except Exception:  # noqa: BLE001
            pass

    # ---------- 服务 ----------
    def on_cue(self, req, res):
        del req
        try:
            self._play_cue()
            res.success, res.message = True, '已播放提示音'
        except Exception as exc:  # noqa: BLE001
            res.success, res.message = False, '提示音失败: %s' % exc
        return res

    def on_stop(self, req, res):
        del req
        proc = self.current_play
        if proc is not None:
            try:
                os.killpg(os.getpgid(proc.pid), 15)
            except Exception:  # noqa: BLE001
                pass
            res.success, res.message = True, '已停止播放'
        else:
            res.success, res.message = True, '当前没有播放'
        return res

    def shutdown(self):
        self._stop.set()
        proc = self.proc
        if proc is not None:
            try:
                proc.stdin.write(b'{"op":"shutdown"}\n')
                proc.stdin.flush()
            except Exception:  # noqa: BLE001
                pass
            try:
                proc.wait(timeout=3)
            except Exception:  # noqa: BLE001
                try:
                    os.killpg(os.getpgid(proc.pid), 15)
                except Exception:  # noqa: BLE001
                    pass


def main(args=None):
    rclpy.init(args=args)
    node = TtsNode()
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