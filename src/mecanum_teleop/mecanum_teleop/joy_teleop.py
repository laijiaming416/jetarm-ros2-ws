#!/usr/bin/env python3
"""PS2 手柄底盘遥控：START 使能后，左摇杆平移、右摇杆原地转向，输出 /cmd_vel_joy。

板端内核未开启 CONFIG_INPUT_JOYDEV（没有 /dev/input/js0），也没有 pygame/evdev，
因此直接解析 evdev 的 input_event 流，零第三方依赖。

轴映射（原始值 0~255，中心 127）：
    ABS_Y  -> linear.x   前后（前为正）
    ABS_X  -> linear.y   横移（左为正）
    ABS_Z  -> angular.z  转向（逆时针为正）
    ABS_RZ -> 未使用

按键（按本手柄实测键码）：
    START(0x139)    -> 使能 / 停止。使能时机械臂蜂鸣器响 1 声，停止时响 2 声
    TL(0x136)       -> 速度系数 -0.1
    TR(0x137)       -> 速度系数 +0.1
    未使能时只发零速，底盘不动

蜂鸣器走机械臂主控板串口（默认 /dev/ttyUSB0，1M 波特率，AA55 + CRC8 协议）。

用法：
    ros2 run mecanum_teleop joy_teleop
    ros2 run mecanum_teleop joy_teleop --probe        # 只打印原始轴/按键，用于校准
"""

import glob
import math
import os
import struct
import sys
import termios
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

EV_SYN = 0x00
EV_KEY = 0x01
EV_ABS = 0x03

ABS_X = 0x00
ABS_Y = 0x01
ABS_Z = 0x02
ABS_RZ = 0x05

BTN_TL = 0x136
BTN_TR = 0x137
BTN_START = 0x139

AXIS_CENTER = 127.5
AXIS_HALF = 127.5

# riscv64 上 struct input_event = timeval(16) + u16 + u16 + s32 = 24 字节
EVENT_FMT = 'llHHi'
EVENT_SIZE = struct.calcsize(EVENT_FMT)

DEFAULT_DEVICE = '/dev/input/by-id/usb-2563_USB_WirelessGamepad-event-joystick'


def resolve_device(path):
    """优先用参数指定路径；不存在时退回到任意 gamepad 的 by-id 软链。"""
    if os.path.exists(path):
        return path
    candidates = sorted(glob.glob('/dev/input/by-id/*event-joystick'))
    return candidates[0] if candidates else None


def normalize(value, invert=False):
    v = (AXIS_CENTER - value) / AXIS_HALF
    v = -v if invert else v
    return max(-1.0, min(1.0, v))


def crc8(data):
    """CRC-8/MAXIM，与主控板固件的校验表等价。"""
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8C if crc & 1 else crc >> 1
    return crc


def beep_serial(port, freq, on_ms, off_ms, repeat):
    """按 AA55 协议向机械臂主控板发送蜂鸣命令（func=2）。"""
    payload = struct.pack('<HHHH', freq, on_ms, off_ms, repeat)
    buf = bytes([0xAA, 0x55, 0x02, len(payload)]) + payload
    buf += bytes([crc8(buf[2:])])

    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        attrs = termios.tcgetattr(fd)
        speed = termios.B1000000
        attrs[0] = 0
        attrs[1] = 0
        attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
        attrs[3] = 0
        attrs[4] = speed
        attrs[5] = speed
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        os.write(fd, buf)
    finally:
        os.close(fd)


def deadzone(v, dz):
    if abs(v) < dz:
        return 0.0
    return math.copysign((abs(v) - dz) / (1.0 - dz), v)


def probe(path, duration):
    """不启 ROS，只打印轴与按键变化，用于确认物理方向。"""
    dev = resolve_device(path)
    if dev is None:
        print('未找到手柄设备，请确认接收器已插入')
        return 1

    fd = os.open(dev, os.O_RDONLY | os.O_NONBLOCK)
    names = {ABS_X: 'ABS_X', ABS_Y: 'ABS_Y', ABS_Z: 'ABS_Z', ABS_RZ: 'ABS_RZ'}
    keys = {BTN_TL: 'TL', BTN_TR: 'TR'}
    print('探测 %s，%.0f 秒，请依次推动摇杆' % (dev, duration))
    start = time.monotonic()
    try:
        while time.monotonic() - start < duration:
            try:
                data = os.read(fd, EVENT_SIZE * 32)
            except BlockingIOError:
                time.sleep(0.005)
                continue
            for off in range(0, len(data) - EVENT_SIZE + 1, EVENT_SIZE):
                _, _, etype, code, value = struct.unpack(EVENT_FMT, data[off:off + EVENT_SIZE])
                if etype == EV_ABS and code in names:
                    print('[%5.1fs] %-6s %3d  -> %+.2f'
                          % (time.monotonic() - start, names[code], value,
                             normalize(value)))
                elif etype == EV_KEY and code in keys:
                    print('[%5.1fs] %s %s' % (time.monotonic() - start, keys[code],
                                              '按下' if value else '松开'))
                sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        os.close(fd)
    print('探测结束')
    return 0


class JoyTeleop(Node):

    def __init__(self):
        super().__init__('joy_teleop')

        self.declare_parameter('device', DEFAULT_DEVICE)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_joy')
        self.declare_parameter('deadzone', 0.12)
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('max_linear', 0.30)
        self.declare_parameter('max_lateral', 0.30)
        self.declare_parameter('max_angular', 1.0)
        self.declare_parameter('scale_init', 0.5)
        self.declare_parameter('scale_step', 0.1)
        self.declare_parameter('scale_min', 0.2)
        self.declare_parameter('scale_max', 1.0)
        self.declare_parameter('ramp_linear', 1.0)
        self.declare_parameter('ramp_angular', 2.0)
        self.declare_parameter('invert_vx', False)
        self.declare_parameter('invert_vy', False)
        self.declare_parameter('invert_wz', False)
        self.declare_parameter('buzzer_port', '/dev/ttyUSB0')
        self.declare_parameter('buzzer_freq', 2000)
        self.declare_parameter('beep_on_ms', 80)
        self.declare_parameter('beep_off_ms', 120)

        self.device_path = str(self.get_parameter('device').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.deadzone_v = float(self.get_parameter('deadzone').value)
        rate = float(self.get_parameter('rate_hz').value)
        self.max_vx = float(self.get_parameter('max_linear').value)
        self.max_vy = float(self.get_parameter('max_lateral').value)
        self.max_wz = float(self.get_parameter('max_angular').value)
        self.scale = float(self.get_parameter('scale_init').value)
        self.scale_step = float(self.get_parameter('scale_step').value)
        self.scale_min = float(self.get_parameter('scale_min').value)
        self.scale_max = float(self.get_parameter('scale_max').value)
        self.ramp_v = float(self.get_parameter('ramp_linear').value)
        self.ramp_w = float(self.get_parameter('ramp_angular').value)
        self.invert = (bool(self.get_parameter('invert_vx').value),
                       bool(self.get_parameter('invert_vy').value),
                       bool(self.get_parameter('invert_wz').value))
        self.buzzer_port = str(self.get_parameter('buzzer_port').value)
        self.buzzer_freq = int(self.get_parameter('buzzer_freq').value)
        self.beep_on = int(self.get_parameter('beep_on_ms').value)
        self.beep_off = int(self.get_parameter('beep_off_ms').value)

        self.pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.fd = None
        self.raw = {ABS_X: AXIS_CENTER, ABS_Y: AXIS_CENTER, ABS_Z: AXIS_CENTER}
        self.vx = self.vy = self.wz = 0.0
        self.armed = False
        self.last_open_attempt = 0.0

        self.create_timer(1.0 / rate, self.on_timer)
        self.get_logger().info('等待 START 启动（未使能时底盘不动）')

    # ---------- 设备 ----------

    def open_device(self):
        dev = resolve_device(self.device_path)
        if dev is None:
            return False
        try:
            self.fd = os.open(dev, os.O_RDONLY | os.O_NONBLOCK)
        except OSError as e:
            self.get_logger().error('打开手柄失败: %s' % e)
            self.fd = None
            return False
        self.get_logger().info('手柄已连接: %s' % dev)
        return True

    def close_device(self):
        if self.fd is not None:
            try:
                os.close(self.fd)
            except OSError:
                pass
            self.fd = None
            self.raw = {ABS_X: AXIS_CENTER, ABS_Y: AXIS_CENTER, ABS_Z: AXIS_CENTER}
            self.armed = False
            self.get_logger().warn('手柄已断开，等待重连')

    # ---------- 输入 ----------

    def read_events(self):
        try:
            data = os.read(self.fd, EVENT_SIZE * 32)
        except BlockingIOError:
            return
        except OSError as e:
            self.get_logger().error('读取手柄失败: %s' % e)
            self.close_device()
            return

        for off in range(0, len(data) - EVENT_SIZE + 1, EVENT_SIZE):
            _, _, etype, code, value = struct.unpack(EVENT_FMT, data[off:off + EVENT_SIZE])
            if etype == EV_ABS and code in self.raw:
                self.raw[code] = value
            elif etype == EV_KEY and code == BTN_START and value == 1:
                self.toggle_armed()
            elif etype == EV_KEY and code in (BTN_TL, BTN_TR) and value == 1:
                step = self.scale_step if code == BTN_TR else -self.scale_step
                self.scale = round(max(self.scale_min,
                                       min(self.scale_max, self.scale + step)), 2)
                self.get_logger().info('速度系数 %.1f' % self.scale)

    def beep(self, repeat):
        try:
            beep_serial(self.buzzer_port, self.buzzer_freq,
                        self.beep_on, self.beep_off, repeat)
        except OSError as e:
            self.get_logger().warn('蜂鸣器不可用: %s' % e)

    def toggle_armed(self):
        self.armed = not self.armed
        if self.armed:
            self.get_logger().info('已启动：摇杆可控，左肩减速 / 右肩加速，系数 %.1f' % self.scale)
            self.beep(1)
        else:
            self.vx = self.vy = self.wz = 0.0
            self.publish(0.0, 0.0, 0.0)
            self.get_logger().info('已停止输出')
            self.beep(2)

    def target(self):
        if not self.armed:
            return 0.0, 0.0, 0.0
        dz = self.deadzone_v
        tx = deadzone(normalize(self.raw[ABS_Y], self.invert[0]), dz) * self.max_vx
        ty = deadzone(normalize(self.raw[ABS_X], self.invert[1]), dz) * self.max_vy
        tz = deadzone(normalize(self.raw[ABS_Z], self.invert[2]), dz) * self.max_wz
        return tx * self.scale, ty * self.scale, tz * self.scale

    # ---------- 输出 ----------

    def on_timer(self):
        dt = 1.0 / max(1.0, self.get_parameter('rate_hz').value)

        if self.fd is None:
            now = time.monotonic()
            if now - self.last_open_attempt >= 1.0:
                self.last_open_attempt = now
                self.open_device()
        else:
            self.read_events()

        tx, ty, tz = self.target() if self.fd is not None else (0.0, 0.0, 0.0)

        max_dv = self.ramp_v * dt
        max_dw = self.ramp_w * dt
        self.vx += max(-max_dv, min(max_dv, tx - self.vx))
        self.vy += max(-max_dv, min(max_dv, ty - self.vy))
        self.wz += max(-max_dw, min(max_dw, tz - self.wz))

        self.publish(self.vx, self.vy, self.wz)

    def publish(self, vx, vy, wz):
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = wz
        self.pub.publish(msg)

    def stop(self):
        self.vx = self.vy = self.wz = 0.0
        if rclpy.ok():
            self.publish(0.0, 0.0, 0.0)
        self.close_device()


def main():
    if '--probe' in sys.argv:
        dur = 15.0
        if '--duration' in sys.argv:
            dur = float(sys.argv[sys.argv.index('--duration') + 1])
        sys.exit(probe(DEFAULT_DEVICE, dur))

    rclpy.init()
    node = JoyTeleop()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
