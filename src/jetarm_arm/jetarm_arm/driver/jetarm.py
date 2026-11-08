#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
JetArm 机械臂上位机驱动

链路：
    PC --USB(Type-C, CH340C)--> 控制器板 STM32F407 (固件 JetArmF4_Firmware)
       --USART6 半双工, 115200--> 6 个串联总线舵机 (HX-12H/HTS-21H/HTS-35H/HTD-35H)

上位机只跟控制器板对话，帧格式与固件 Hiwonder/Misc/packet.c 严格一致：

    0xAA 0x55 | func | len | data[len] | crc8

    crc8 覆盖 func、len、data 三个部分（不含帧头）。

总线舵机命令的功能号 func = 5 (PACKET_FUNC_BUS_SERVO)，
data[0] 是子命令（0x01 运动 / 0x05 读位置 / 0x12 读 ID ...）。
舵机那一侧真正的 0x55 0x55 帧由板子固件收发，上位机不用管。

舵机规格（四款一致）：
    波特率 115200，ID 0~253（出厂默认 1），位置 0~1000 对应 0~240°，中位 500

用法见 --help。
"""

import argparse
import csv
import glob
import errno
try:
    import fcntl as _locking
except ImportError:
    _locking = None
import json
import os
import struct
import sys
import time

try:
    import serial
    HAVE_PYSERIAL = True
except ImportError:
    HAVE_PYSERIAL = False
    # 没有 pyserial 时退回到 termios 裸串口实现（K1 上默认没有 pyserial）
    import array
    import fcntl
    import select
    import termios

    class SerialException(IOError):
        pass

    _BAUD_TABLE = {
        9600: termios.B9600,
        19200: termios.B19200,
        38400: termios.B38400,
        57600: termios.B57600,
        115200: termios.B115200,
        230400: termios.B230400,
        1000000: getattr(termios, "B1000000", 0o010010),
    }

    class RawSerialPort:
        """最小 pyserial 兼容实现，只覆盖本驱动用到的接口。"""

        def __init__(self, port, baudrate, timeout=0.3):
            self.port = port
            self.baudrate = baudrate
            self.timeout = timeout
            self.is_open = False
            try:
                self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            except OSError as e:
                raise SerialException("could not open port %s: %s" % (port, e))
            try:
                fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except OSError:
                os.close(self.fd)
                raise
            spd = _BAUD_TABLE.get(baudrate)
            if spd is None:
                os.close(self.fd)
                raise SerialException("不支持的波特率 %s" % baudrate)
            try:
                attrs = termios.tcgetattr(self.fd)
                # 8N1 原始模式
                attrs[0] = 0                                     # iflag
                attrs[1] = 0                                     # oflag
                attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
                attrs[3] = 0                                     # lflag
                attrs[4] = spd                                   # ispeed
                attrs[5] = spd                                   # ospeed
                attrs[6][termios.VMIN] = 0
                attrs[6][termios.VTIME] = 0
                termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
                termios.tcflush(self.fd, termios.TCIOFLUSH)
            except Exception:
                os.close(self.fd)
                raise
            self.is_open = True

        @property
        def in_waiting(self):
            buf = array.array("i", [0])
            try:
                fcntl.ioctl(self.fd, termios.TIOCINQ, buf, True)
                return buf[0]
            except OSError:
                return 0

        def read(self, size=1):
            end = time.monotonic() + (self.timeout or 0)
            data = bytearray()
            while len(data) < size:
                left = end - time.monotonic()
                if left <= 0:
                    break
                r, _, _ = select.select([self.fd], [], [], left)
                if not r:
                    break
                try:
                    chunk = os.read(self.fd, size - len(data))
                except BlockingIOError:
                    continue
                if not chunk:
                    break
                data += chunk
            return bytes(data)

        def write(self, data):
            data = bytes(data)
            sent = 0
            deadline = time.monotonic() + max(float(self.timeout or 0), 0.3)
            while sent < len(data):
                left = deadline - time.monotonic()
                if left <= 0:
                    raise SerialException("串口写入超时，已写 %d/%d 字节" % (sent, len(data)))
                _, ready, _ = select.select([], [self.fd], [], left)
                if not ready:
                    raise SerialException("串口写入超时，已写 %d/%d 字节" % (sent, len(data)))
                try:
                    count = os.write(self.fd, data[sent:])
                except (BlockingIOError, InterruptedError):
                    continue
                if count <= 0:
                    raise SerialException("串口写入未前进，已写 %d/%d 字节" % (sent, len(data)))
                sent += count
            return sent

        def flush(self):
            termios.tcdrain(self.fd)

        def reset_input_buffer(self):
            termios.tcflush(self.fd, termios.TCIFLUSH)

        def close(self):
            if self.is_open:
                try:
                    os.close(self.fd)
                except OSError:
                    pass
                self.is_open = False

    class _SerialShim:
        Serial = RawSerialPort
        SerialException = SerialException

    serial = _SerialShim()

# ---------------------------------------------------------------------------
# 协议常量
# ---------------------------------------------------------------------------

START1 = 0xAA
START2 = 0x55

FUNC_BUS_SERVO = 5
FUNC_LED = 1
FUNC_BUZZER = 2
FUNC_MOTOR = 3
FUNC_KEY = 6

# 舵机子命令（见 Hiwonder/System/packet_handle.c 的 packet_serial_servo_handle）
SUB_MOVE = 0x01          # 多舵机运动 {cmd, duration, servo_num, [id, position]*}
SUB_STOP = 0x03          # 停止
SUB_READ_POS = 0x05      # 读位置
SUB_READ_VIN = 0x07      # 读输入电压 mV
SUB_READ_TEMP = 0x09     # 读温度 ℃
SUB_UNLOAD = 0x0B        # 卸载动力
SUB_LOAD = 0x0C          # 加载动力
SUB_READ_LOAD = 0x0D     # 读动力状态
SUB_SET_ID = 0x10        # 写 ID
SUB_READ_ID = 0x12       # 读 ID
SUB_SET_DEV = 0x20       # 偏差调整
SUB_READ_DEV = 0x22      # 偏差读取
SUB_SAVE_DEV = 0x24      # 偏差保存
SUB_SET_ANGLE_LIMIT = 0x30
SUB_READ_ANGLE_LIMIT = 0x32
SUB_SET_VIN_LIMIT = 0x34
SUB_READ_VIN_LIMIT = 0x36
SUB_SET_TEMP_LIMIT = 0x38
SUB_READ_TEMP_LIMIT = 0x3A

# checksum.c 里的 CRC8 查表
CRC8_TABLE = bytes([
    0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
    157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
    35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
    190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
    70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
    219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
    101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
    248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
    140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
    17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
    175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
    50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
    202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
    87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
    233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
    116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53,
])

DEFAULT_BAUD = 1000000   # 固件 usart.c 里 USART1 = 1000000
DEFAULT_PORT = "/dev/ttyUSB0"

# 关节映射（2026-10-27 通过"手动掰动 + 时序分析"实测得出，见 joint_map.csv）
# ID1~ID5 从底座往末端依次排列，夹爪单独用 ID10
JOINTS = {
    1: "底座旋转",
    2: "大臂(肩)",
    3: "小臂(肘)",
    4: "腕部俯仰",
    5: "腕部旋转",
    10: "夹爪",
}


def joint_name(servo_id):
    return JOINTS.get(servo_id, "ID%s" % servo_id)


def crc8(data: bytes) -> int:
    c = 0
    for b in data:
        c = CRC8_TABLE[c ^ b]
    return c


def build_frame(func: int, data: bytes) -> bytes:
    """组一帧上位机 -> 板子的数据帧。"""
    if len(data) > 255:
        raise ValueError("data 段不能超过 255 字节")
    body = bytes([func, len(data)]) + data
    return bytes([START1, START2]) + body + bytes([crc8(body)])


def _trace_request(sub, payload, result, sid, status, elapsed_ms):
    """Best-effort bounded diagnostic; never blocks a command on logging failure."""
    path = os.environ.get("JETARM_TRACE_PATH", "/home/laijinxuan/jetarm/communication-trace.jsonl")
    try:
        flags = os.O_WRONLY | os.O_APPEND | os.O_CREAT | getattr(os, "O_NOFOLLOW", 0)
        fd = os.open(path, flags, 0o600)
        try:
            if os.fstat(fd).st_size >= 1000000:
                return
            row = {"time": time.time(), "sub": sub, "target": payload[0] if payload else None,
                   "result": result, "sid": sid, "status": status, "elapsed_ms": round(elapsed_ms, 1)}
            os.write(fd, (json.dumps(row, separators=(",", ":")) + "\n").encode("utf-8"))
        finally:
            os.close(fd)
    except (OSError, ValueError, TypeError):
        pass


class JetArmError(Exception):
    pass


class JetArm:
    def __init__(self, port=DEFAULT_PORT, baud=DEFAULT_BAUD, timeout=0.3, allow_any_port=False):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        # 机械臂控制器是 CH340 (1a86:7523)。优先按 by-id 找，避免把别的 CH340/串口
        # （例如 ttyUSB1 上的未知设备）当候选口并写入舵机帧。
        cands = [port]
        for p in sorted(glob.glob("/dev/serial/by-id/usb-1a86_USB_Serial*")):
            rp = os.path.realpath(p)
            if rp not in cands:
                cands.append(rp)
        if allow_any_port:
            for pat in ("/dev/ttyUSB*", "/dev/ttyACM*"):
                for p in sorted(glob.glob(pat)):
                    if p not in cands:
                        cands.append(p)
        seen = set()
        last_err = None
        for p in cands:
            if p in seen:
                continue
            seen.add(p)
            try:
                self.ser = serial.Serial(p, baud, timeout=timeout)
            except (serial.SerialException, OSError) as e:
                if getattr(e, "errno", None) in (errno.EAGAIN, errno.EACCES):
                    raise JetArmError("串口已被另一个兼容程序占用: %s" % p) from e
                last_err = e
                continue
            try:
                if _locking is not None:
                    _locking.flock(self.ser.fileno() if hasattr(self.ser, "fileno") else self.ser.fd, _locking.LOCK_EX | _locking.LOCK_NB)
            except (OSError, IOError) as e:
                self.ser.close()
                if getattr(e, "errno", None) in (errno.EAGAIN, errno.EACCES):
                    raise JetArmError("串口已被另一个兼容程序占用: %s" % p) from e
                raise
            self.ser.reset_input_buffer()
            time.sleep(0.05)
            # 同一口先探测两轮，避免一次瞬时 255/超时就把正确的口排除掉。
            for _ in range(2):
                if self._read_args(SUB_READ_POS, 1, attempts=2, min_args=2):
                    self.port = p
                    return
                time.sleep(0.1)
            self.ser.close()
        raise JetArmError("候选串口都读不到舵机（%s）：%s"
                          % (", ".join(sorted(seen)), last_err or "无响应"))

    # -- 底层收发 ---------------------------------------------------------

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def _send(self, func: int, data: bytes):
        frame = build_frame(func, data)
        written = self.ser.write(frame)
        if written != len(frame):
            raise JetArmError("串口帧未完整发送：%s/%d 字节" % (written, len(frame)))
        self.ser.flush()

    def _read_frame(self, expect_func=None, expect_sub=None):
        """收一帧，返回 (func, data)。超时返回 (None, None)。"""
        deadline = time.monotonic() + self.timeout
        buf = bytearray()
        while time.monotonic() < deadline:
            chunk = self.ser.read(self.ser.in_waiting or 1)
            if chunk:
                buf += chunk
            # 在缓冲区里找帧头
            while True:
                i = buf.find(bytes([START1, START2]))
                if i < 0:
                    if len(buf) > 4096:
                        buf.clear()
                    break
                if len(buf) < i + 4:
                    break
                flen = buf[i + 3]
                total = i + 5 + flen
                if len(buf) < total:
                    break
                func = buf[i + 2]
                data = bytes(buf[i + 4:i + 4 + flen])
                crc = buf[i + 4 + flen]
                del buf[:total]
                if crc8(bytes([func, flen]) + data) != crc:
                    continue          # 校验失败，继续找下一帧
                if expect_func is not None and func != expect_func:
                    continue
                # 总线舵机回包的数据段布局是 [servo_id, sub_command, success, args...]，
                # 子命令在 data[1]，data[0] 是舵机 ID
                if expect_sub is not None and (len(data) < 2 or data[1] != expect_sub):
                    continue
                return func, data
        return None, None

    def _request(self, sub: int, payload: bytes = b"", timeout=None):
        """发一条总线舵机命令并等回包，返回 (servo_id, success, args_bytes)。"""
        started = time.monotonic()
        old = self.timeout
        if timeout is not None:
            self.timeout = timeout
        try:
            self.ser.reset_input_buffer()
            self._send(FUNC_BUS_SERVO, bytes([sub]) + payload)
            func, data = self._read_frame(expect_func=FUNC_BUS_SERVO, expect_sub=sub)
        finally:
            self.timeout = old
        stats = getattr(self, "communication_stats", None)
        if stats is None:
            stats = self.communication_stats = {"success": 0, "controller_failure": 0, "timeout": 0, "invalid_response": 0}
        if func is None:
            stats["timeout"] += 1
            _trace_request(sub, payload, "timeout", None, None, (time.monotonic()-started)*1000)
            return None, None, b""
        if len(data) < 3:
            stats["invalid_response"] += 1
        elif data[2] == 0:
            stats["success"] += 1
        else:
            stats["controller_failure"] += 1
        # data = [servo_id, sub_command, success, args...]
        if len(data) < 3:
            _trace_request(sub, payload, "invalid_response", None, None, (time.monotonic()-started)*1000)
            return None, None, b""
        _trace_request(sub, payload, "success" if data[2] == 0 else "controller_failure",
                       data[0], data[2], (time.monotonic()-started)*1000)
        return data[0], data[2], data[3:]

    READ_RETRY_DELAYS = (0.05, 0.1, 0.2)

    def _read_args(self, sub, servo_id, timeout=None, attempts=4, min_args=1):
        """带重试的读命令，只用于读，绝不重发运动帧。成功返回 args，失败返回 b''。"""
        args = b""
        for i in range(attempts):
            sid, ok, args = self._request(sub, bytes([servo_id]), timeout)
            if sid == servo_id and ok == 0 and len(args) >= min_args:
                return args
            if i + 1 < attempts:
                time.sleep(self.READ_RETRY_DELAYS[min(i, len(self.READ_RETRY_DELAYS) - 1)])
        return b""

    # -- 高层 API ---------------------------------------------------------

    def move_auto(self, targets, ms_per_unit=3.0, min_ms=280, max_ms=2000):
        """按最大关节位移自动决定时长（3ms/刻度 ≈ 舵机极限速度）。返回实际时长。"""
        cur = {}
        for s in targets:
            p = self.read_position(s)
            cur[s] = int(targets[s]) if p is None else p
        delta = max(abs(int(targets[s]) - cur[s]) for s in targets)
        ms = int(max(min_ms, min(max_ms, delta * ms_per_unit)))
        self.move(targets, ms)
        return ms

    def move(self, targets, duration_ms=500, timeout=None):
        """一次命令让多个舵机同时走到目标位置（板子固件内部做时间插值）。

        targets: {servo_id: position(0~1000)}
        """
        ids = list(targets.keys())
        if not ids:
            return
        body = struct.pack("<HB", int(duration_ms) & 0xFFFF, len(ids))
        for sid in ids:
            pos = max(0, min(1000, int(targets[sid])))
            body += struct.pack("<BH", int(sid) & 0xFF, pos)
        # 运动指令固件不回包，发完即走
        self._send_only(SUB_MOVE, body)

    def _send_only(self, sub: int, payload: bytes = b""):
        """只发不等回包（用于固件不回包的子命令：0x01 运动、0x0B/0x0C 动力开关）。"""
        self.ser.reset_input_buffer()
        self._send(FUNC_BUS_SERVO, bytes([sub]) + payload)

    def stop(self, servo_ids, timeout=None):
        body = bytes([len(servo_ids)]) + bytes(bytearray(int(i) & 0xFF for i in servo_ids))
        self._request(SUB_STOP, body, timeout)

    def beep(self, times=1, freq=2000, on_ms=120, off_ms=120):
        """蜂鸣器响 times 声（固件 func=2，不回包）。"""
        payload = struct.pack("<HHHH", int(freq) & 0xFFFF, int(on_ms) & 0xFFFF,
                              int(off_ms) & 0xFFFF, max(1, int(times)) & 0xFFFF)
        self.ser.reset_input_buffer()
        self._send(FUNC_BUZZER, payload)

    def read_position(self, servo_id, timeout=None):
        args = self._read_args(SUB_READ_POS, servo_id, timeout, min_args=2)
        if not args:
            return None
        return struct.unpack("<h", args[:2])[0]

    def read_vin(self, servo_id, timeout=None):
        args = self._read_args(SUB_READ_VIN, servo_id, timeout, min_args=2)
        if not args:
            return None
        return struct.unpack("<H", args[:2])[0]

    def read_temp(self, servo_id, timeout=None):
        args = self._read_args(SUB_READ_TEMP, servo_id, timeout, min_args=1)
        if not args:
            return None
        return args[0]

    def read_id(self, servo_id, timeout=None):
        """向 servo_id 发读 ID 命令，返回舵机实际上报的 ID。"""
        args = self._read_args(SUB_READ_ID, servo_id, timeout, min_args=1)
        if not args:
            return None
        return args[0]

    def read_deviation(self, servo_id, timeout=None):
        args = self._read_args(SUB_READ_DEV, servo_id, timeout, min_args=1)
        if not args:
            return None
        v = args[0]
        return v - 256 if v > 127 else v

    def read_load_state(self, servo_id, timeout=None):
        args = self._read_args(SUB_READ_LOAD, servo_id, timeout, min_args=1)
        if not args:
            return None
        return args[0]

    def read_angle_limit(self, servo_id, timeout=None):
        args = self._read_args(SUB_READ_ANGLE_LIMIT, servo_id, timeout, min_args=4)
        if not args:
            return None
        return struct.unpack("<HH", args[:4])

    def set_id(self, old_id, new_id, timeout=None):
        self._request(SUB_SET_ID, bytes([old_id, new_id]), timeout)

    def set_deviation(self, servo_id, dev, timeout=None):
        self._request(SUB_SET_DEV, bytes([servo_id, dev & 0xFF]), timeout)

    def save_deviation(self, servo_id, timeout=None):
        self._request(SUB_SAVE_DEV, bytes([servo_id]), timeout)

    def set_angle_limit(self, servo_id, low, high, timeout=None):
        self._request(SUB_SET_ANGLE_LIMIT,
                      bytes([servo_id]) + struct.pack("<HH", low, high), timeout)

    def set_temp_limit(self, servo_id, limit, timeout=None):
        self._request(SUB_SET_TEMP_LIMIT, bytes([servo_id, limit & 0xFF]), timeout)

    def unload(self, servo_id, timeout=None):
        """卸载动力，舵机变软，可以用手摆姿势（示教）。"""
        self._send_only(SUB_UNLOAD, bytes([servo_id]))

    def load(self, servo_id, timeout=None):
        """加载动力，舵机重新锁位。"""
        self._send_only(SUB_LOAD, bytes([servo_id]))

    def scan(self, id_range=range(1, 21), quiet=False):
        """扫描总线，返回 {id: {'id':..,'pos':..,'vin':..,'temp':..}}。"""
        found = {}
        for sid in id_range:
            actual = self.read_id(sid)
            if actual is None:
                continue
            pos = self.read_position(actual)
            vin = self.read_vin(actual)
            temp = self.read_temp(actual)
            found[actual] = {"id": actual, "pos": pos, "vin": vin, "temp": temp}
            if not quiet:
                print("  找到舵机 ID=%-3d %-9s 位置=%-4s 电压=%-6s 温度=%s"
                      % (actual, joint_name(actual), pos,
                         ("%smV" % vin) if vin else "?",
                         ("%s℃" % temp) if temp else "?"))
        return found


# ---------------------------------------------------------------------------
# 命令行
# ---------------------------------------------------------------------------

def _open(args):
    return JetArm(args.port, args.baud, timeout=args.timeout,
                  allow_any_port=args.allow_any_port)


def cmd_scan(args):
    with _open(args) as arm:
        print("扫描 %s @ %d bps，ID 范围 %d~%d ..." % (args.port, args.baud, args.id_min, args.id_max))
        found = arm.scan(range(args.id_min, args.id_max + 1))
        print("\n共发现 %d 个舵机：%s" % (len(found), sorted(found.keys())))
        if not found:
            print("一个都没响应。检查：舵机是否上电(9~12.6V)、总线线序、板子固件是否为 JetArmF4_Firmware")
        dup = len(found) != len(set(found.keys()))
        if dup:
            print("注意：存在重复 ID，需要先用 set-id 逐个改成唯一 ID")


def cmd_read(args):
    with _open(args) as arm:
        for sid in args.ids:
            pos = arm.read_position(sid)
            vin = arm.read_vin(sid)
            temp = arm.read_temp(sid)
            dev = arm.read_deviation(sid)
            load = arm.read_load_state(sid)
            lim = arm.read_angle_limit(sid)
            print("ID=%-3d %-9s 位置=%-5s 电压=%-7s 温度=%-5s 偏差=%-5s 力矩=%-4s 限位=%s"
                  % (sid, joint_name(sid),
                     pos if pos is not None else "无响应",
                     ("%dmV" % vin) if vin is not None else "-",
                     ("%d℃" % temp) if temp is not None else "-",
                     dev if dev is not None else "-",
                     ("已加载" if load == 1 else "已卸载") if load is not None else "-",
                     "%d~%d" % lim if lim else "-"))


def cmd_watch(args):
    ids = args.ids
    print("实时监控 %s（Ctrl+C 退出）" % ids)
    try:
        with _open(args) as arm:
            while True:
                row = []
                for sid in ids:
                    p = arm.read_position(sid)
                    row.append("%d:%s" % (sid, "----" if p is None else "%4d" % p))
                print("\r" + "  ".join(row), end="", flush=True)
                time.sleep(args.interval)
    except KeyboardInterrupt:
        print("\n已停止")


def cmd_pose(args):
    with _open(args) as arm:
        pose = {}
        for sid in args.ids:
            p = arm.read_position(sid)
            if p is not None:
                pose[sid] = p
        print(json.dumps(pose))
        print("\n可直接粘到动作文件的 frames[].positions 里")


def cmd_move(args):
    targets = {}
    for item in args.targets:
        if "=" not in item:
            sys.exit("目标格式应为 id=position，例如 1=500")
        k, v = item.split("=", 1)
        targets[int(k)] = int(v)
    with _open(args) as arm:
        print("运动 %s，用时 %dms" % (targets, args.duration))
        arm.move(targets, args.duration)
        time.sleep(args.duration / 1000.0 + 0.2)
        for sid in targets:
            print("  ID=%-3d 当前位置=%s" % (sid, arm.read_position(sid)))


def cmd_unload(args):
    with _open(args) as arm:
        for sid in args.ids:
            arm.unload(sid)
        print("已卸载动力，舵机现在可以用手自由摆动：%s" % args.ids)
        print("重新锁位请执行：python3 jetarm.py load %s" % " ".join(str(i) for i in args.ids))


def cmd_load(args):
    with _open(args) as arm:
        for sid in args.ids:
            arm.load(sid)
        print("已加载动力，舵机重新锁位：%s" % args.ids)


def cmd_set_id(args):
    with _open(args) as arm:
        print("把 ID=%d 改成 ID=%d ..." % (args.old, args.new))
        arm.set_id(args.old, args.new)
        time.sleep(0.3)
        print("改完后用新 ID 读回验证：%s" % arm.read_id(args.new))
        print("提示：一次只接一个舵机改 ID 最保险，改完记得断电重上电生效")


def cmd_limits(args):
    with _open(args) as arm:
        for sid in args.ids:
            lim = arm.read_angle_limit(sid)
            print("ID=%-3d 角度限位=%s" % (sid, ("%d~%d" % lim) if lim else "读取失败"))


def cmd_teach(args):
    """卸力示教：用手摆姿势，回车记录一个关键帧，q 结束并保存。"""
    frames = []
    with _open(args) as arm:
        print("卸载动力，用手把机械臂摆到想要的姿势 ...")
        for sid in args.ids:
            arm.unload(sid)
        time.sleep(0.3)
        print("【回车】记录一帧   【s 回车】记录当前帧并指定时长   【q 回车】结束保存")
        last = None
        while True:
            try:
                line = input("帧%-3d > " % (len(frames) + 1)).strip().lower()
            except EOFError:
                break
            if line == "q":
                break
            pose = {}
            for sid in args.ids:
                p = arm.read_position(sid)
                if p is not None:
                    pose[str(sid)] = p
            if not pose:
                print("  读不到位置，跳过")
                continue
            duration = args.duration
            if line.startswith("s"):
                parts = line.split()
                if len(parts) > 1:
                    try:
                        duration = int(parts[1])
                    except ValueError:
                        pass
            frames.append({"duration": duration, "positions": pose})
            last = pose
            print("  已记录 %s" % json.dumps(pose, ensure_ascii=False))
        print("重新锁位 ...")
        for sid in args.ids:
            arm.load(sid)
    if not frames:
        print("没记录任何帧，未保存")
        return
    doc = {"name": args.name, "servos": args.ids, "frames": frames}
    path = os.path.abspath(args.out)
    with open(path, "w", encoding="utf-8") as fp:
        json.dump(doc, fp, ensure_ascii=False, indent=2)
    print("已保存 %d 帧到 %s" % (len(frames), path))
    print("回放：python3 jetarm.py play \"%s\"" % path)


def cmd_record(args):
    """持续记录各舵机位置到 CSV，用于分析关节与 ID 的对应关系、或录制轨迹。"""
    ids = args.ids
    t0 = time.monotonic()
    with open(args.out, "w", newline="", encoding="utf-8") as fp:
        w = csv.writer(fp)
        w.writerow(["t"] + ["id%d" % i for i in ids])
        fp.flush()
        n = 0
        with _open(args) as arm:
            while time.monotonic() - t0 < args.duration:
                row = [round(time.monotonic() - t0, 3)]
                for sid in ids:
                    try:
                        p = arm.read_position(sid)
                    except Exception:
                        p = None
                    row.append("" if p is None else p)
                w.writerow(row)
                fp.flush()
                n += 1
                if args.progress and n % 100 == 0:
                    print("  已记录 %d 行 (t=%.0fs)" % (n, time.monotonic() - t0),
                          flush=True)
                time.sleep(args.interval)
    print("记录完成：%d 行 -> %s" % (n, os.path.abspath(args.out)))


def cmd_play(args):
    with open(args.file, encoding="utf-8") as fp:
        doc = json.load(fp)
    frames = doc.get("frames", [])
    if not frames:
        sys.exit("动作文件里没有 frames")
    print("播放动作 '%s'，共 %d 帧" % (doc.get("name", "?"), len(frames)))
    with _open(args) as arm:
        if args.loop:
            print("循环模式，Ctrl+C 停止")
        try:
            while True:
                for i, fr in enumerate(frames):
                    targets = {int(k): int(v) for k, v in fr["positions"].items()}
                    dur = int(fr.get("duration", 500))
                    print("  第 %d/%d 帧  用时 %dms  %s" % (i + 1, len(frames), dur, targets))
                    arm.move(targets, dur)
                    time.sleep(dur / 1000.0 + args.gap)
                if not args.loop:
                    break
        except KeyboardInterrupt:
            print("\n已停止")


def main():
    p = argparse.ArgumentParser(
        description="JetArm 机械臂上位机驱动（通过 STM32 控制器板转发总线舵机指令）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""常用流程：
  1) 扫描总线，确认 6 个舵机的 ID
       python3 jetarm.py scan
  2) 读取全部状态
       python3 jetarm.py read 1 2 3 4 5 6
  3) 卸力，用手摆姿势，示教录制动作
       python3 jetarm.py teach 1 2 3 4 5 6 -o wave.json
  4) 回放
       python3 jetarm.py play wave.json
""")
    p.add_argument("--port", default=DEFAULT_PORT, help="串口设备，默认 %s" % DEFAULT_PORT)
    p.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="波特率，默认 %d" % DEFAULT_BAUD)
    p.add_argument("--timeout", type=float, default=0.3, help="单次请求超时秒数，默认 0.3")
    p.add_argument("--allow-any-port", action="store_true",
                   help="允许探测所有 ttyUSB/ttyACM；默认只认机械臂 CH340 的 by-id 口")

    sub = p.add_subparsers(dest="cmd", required=True)

    sp = sub.add_parser("scan", help="扫描总线上的舵机")
    sp.add_argument("--id-min", type=int, default=1)
    sp.add_argument("--id-max", type=int, default=20)
    sp.set_defaults(func=cmd_scan)

    sp = sub.add_parser("read", help="读取舵机状态")
    sp.add_argument("ids", type=int, nargs="+")
    sp.set_defaults(func=cmd_read)

    sp = sub.add_parser("watch", help="实时监控位置")
    sp.add_argument("ids", type=int, nargs="+")
    sp.add_argument("--interval", type=float, default=0.2)
    sp.set_defaults(func=cmd_watch)

    sp = sub.add_parser("pose", help="打印当前姿态，便于手抄成关键帧")
    sp.add_argument("ids", type=int, nargs="+")
    sp.set_defaults(func=cmd_pose)

    sp = sub.add_parser("move", help="让若干舵机走到目标位置")
    sp.add_argument("targets", nargs="+", help="id=position，例如 1=500 2=620")
    sp.add_argument("--duration", type=int, default=500, help="运动时间 ms，默认 500")
    sp.set_defaults(func=cmd_move)

    sp = sub.add_parser("unload", help="卸载动力（示教时让机械臂变软）")
    sp.add_argument("ids", type=int, nargs="+")
    sp.set_defaults(func=cmd_unload)

    sp = sub.add_parser("load", help="加载动力（重新锁位）")
    sp.add_argument("ids", type=int, nargs="+")
    sp.set_defaults(func=cmd_load)

    sp = sub.add_parser("set-id", help="修改舵机 ID（一次只接一个舵机最保险）")
    sp.add_argument("old", type=int)
    sp.add_argument("new", type=int)
    sp.set_defaults(func=cmd_set_id)

    sp = sub.add_parser("limits", help="读取角度限位")
    sp.add_argument("ids", type=int, nargs="+")
    sp.set_defaults(func=cmd_limits)

    sp = sub.add_parser("teach", help="示教录制动作，保存为 JSON")
    sp.add_argument("ids", type=int, nargs="+")
    sp.add_argument("-o", "--out", default="action.json")
    sp.add_argument("--name", default="action")
    sp.add_argument("--duration", type=int, default=800, help="每帧默认时长 ms")
    sp.set_defaults(func=cmd_teach)

    sp = sub.add_parser("record", help="持续记录位置到 CSV（分析关节映射 / 录制轨迹）")
    sp.add_argument("ids", type=int, nargs="+")
    sp.add_argument("-o", "--out", default="record.csv")
    sp.add_argument("--duration", type=float, default=240.0, help="记录时长秒数，默认 240")
    sp.add_argument("--interval", type=float, default=0.06, help="采样间隔秒数，默认 0.06")
    sp.add_argument("-q", "--no-progress", dest="progress", action="store_false",
                    help="不打印进度")
    sp.set_defaults(func=cmd_record, progress=True)

    sp = sub.add_parser("play", help="回放动作 JSON")
    sp.add_argument("file")
    sp.add_argument("--loop", action="store_true", help="循环播放")
    sp.add_argument("--gap", type=float, default=0.05, help="帧间额外停顿秒数")
    sp.set_defaults(func=cmd_play)

    args = p.parse_args()
    try:
        args.func(args)
    except JetArmError as e:
        sys.exit("错误：%s" % e)
    except KeyboardInterrupt:
        print()


if __name__ == "__main__":
    main()
