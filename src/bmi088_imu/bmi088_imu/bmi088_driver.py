#!/usr/bin/env python3
"""BMI088 SPI 驱动封装。

对应板端两个设备节点：
    加速度计 CSB1 -> GPIO51（软件片选）-> /dev/bmi088_spi
    陀螺仪   CSB2 -> GPIO50（软件片选）-> /dev/bmi088_gyro

两个字符设备由 spidev3 内核模块提供，接口兼容 spidev 的 SPI_IOC_MESSAGE。
加速度计读时序：地址 + 1 字节 dummy + 数据；数据寄存器 0x12-0x17。
陀螺仪读时序：地址 + 数据；数据寄存器 0x02-0x07。
"""

import ctypes
import fcntl
import math
import os
import struct
import time

_SPI_IOC_MAGIC = ord('k')
_SPI_IOC_WR_MAX_SPEED_HZ = 0x40046B04
_SPI_IOC_RD_MAX_SPEED_HZ = 0x80046B04


def _spi_ioc_message(n):
    return (1 << 30) | ((n * 32) << 16) | (_SPI_IOC_MAGIC << 8) | 0


class _SpiIocTransfer(ctypes.Structure):
    _fields_ = [
        ('tx_buf', ctypes.c_uint64),
        ('rx_buf', ctypes.c_uint64),
        ('len', ctypes.c_uint32),
        ('speed_hz', ctypes.c_uint32),
        ('delay_usecs', ctypes.c_uint16),
        ('bits_per_word', ctypes.c_uint8),
        ('cs_change', ctypes.c_uint8),
        ('tx_nbits', ctypes.c_uint8),
        ('rx_nbits', ctypes.c_uint8),
        ('word_delay_usecs', ctypes.c_uint8),
        ('pad', ctypes.c_uint8),
    ]


# 加速度计寄存器
ACC_CHIP_ID = 0x00
ACC_ERR_REG = 0x02
ACC_STATUS = 0x03
ACC_DATA = 0x12
ACC_CONF = 0x40
ACC_RANGE = 0x41
ACC_PWR_CONF = 0x7C
ACC_PWR_CTRL = 0x7D
ACC_SOFTRESET = 0x7E
ACC_TEMP_LSB = 0x22
ACC_TEMP_MSB = 0x23

# 陀螺仪寄存器
GYRO_CHIP_ID = 0x00
GYRO_DATA = 0x02
GYRO_RANGE = 0x0F
GYRO_BANDWIDTH = 0x10
GYRO_LPM1 = 0x11
GYRO_SOFTRESET = 0x14

ACC_CHIP_ID_VALUE = 0x1E
GYRO_CHIP_ID_VALUE = 0x0F
SOFTRESET_CMD = 0xB6

# 量程 -> (寄存器值, 灵敏度 LSB/单位)
ACC_RANGE_CFG = {
    '3g': (0x00, 10920.0),
    '6g': (0x01, 5460.0),
    '12g': (0x02, 2730.0),
    '24g': (0x03, 1365.0),
}

GYRO_RANGE_CFG = {
    '2000dps': (0x00, 16.384),
    '1000dps': (0x01, 32.768),
    '500dps': (0x02, 65.536),
    '250dps': (0x03, 131.072),
    '125dps': (0x04, 262.144),
}

GRAVITY = 9.80665


class Bmi088Error(RuntimeError):
    """驱动层错误：设备缺失、芯片 ID 不匹配、SPI 读写失败等。"""


class Bmi088Driver:

    def __init__(self,
                 acc_dev='/dev/bmi088_spi',
                 gyro_dev='/dev/bmi088_gyro',
                 spi_speed_hz=2000000):
        self.acc_dev = acc_dev
        self.gyro_dev = gyro_dev
        self.spi_speed_hz = int(spi_speed_hz)
        self._acc_fd = None
        self._gyro_fd = None
        self.acc_lsb_per_g = None
        self.gyro_lsb_per_dps = None
        self.acc_range_name = None
        self.gyro_range_name = None

    # ------------------------------------------------------------------ 设备
    def open(self):
        for path in (self.acc_dev, self.gyro_dev):
            if not os.path.exists(path):
                raise Bmi088Error(
                    f'设备 {path} 不存在，请确认 spidev3 模块已加载')
        try:
            self._acc_fd = os.open(self.acc_dev, os.O_RDWR)
            self._gyro_fd = os.open(self.gyro_dev, os.O_RDWR)
        except OSError as exc:
            self.close()
            raise Bmi088Error(f'打开设备失败: {exc}') from exc

        for fd, name in ((self._acc_fd, self.acc_dev),
                         (self._gyro_fd, self.gyro_dev)):
            speed = ctypes.c_uint32(self.spi_speed_hz)
            try:
                fcntl.ioctl(fd, _SPI_IOC_WR_MAX_SPEED_HZ, speed)
            except OSError as exc:
                self.close()
                raise Bmi088Error(
                    f'设置 {name} SPI 速度为 {self.spi_speed_hz}Hz 失败: {exc}'
                ) from exc

    def close(self):
        if self._acc_fd is not None:
            os.close(self._acc_fd)
            self._acc_fd = None
        if self._gyro_fd is not None:
            os.close(self._gyro_fd)
            self._gyro_fd = None

    # ------------------------------------------------------------- 底层传输
    def _xfer(self, fd, tx):
        n = len(tx)
        txb = (ctypes.c_uint8 * n)(*tx)
        rxb = (ctypes.c_uint8 * n)()
        tr = _SpiIocTransfer(
            tx_buf=ctypes.addressof(txb),
            rx_buf=ctypes.addressof(rxb),
            len=n,
            speed_hz=0,  # 0 表示使用 spi_setup 里的 max_speed_hz
        )
        try:
            fcntl.ioctl(fd, _spi_ioc_message(1), tr)
        except OSError as exc:
            raise Bmi088Error(f'SPI 传输失败: {exc}') from exc
        return bytes(rxb)

    def _read_acc(self, reg, n):
        # 加速度计：地址 + 1 dummy，数据从第 3 字节开始
        tx = [0x80 | reg, 0x00] + [0x00] * n
        rx = self._xfer(self._acc_fd, tx)
        return rx[2:2 + n]

    def _write_acc(self, reg, val):
        self._xfer(self._acc_fd, [reg & 0x7F, val])

    def _read_gyro(self, reg, n):
        # 陀螺仪：地址后直接输出数据
        tx = [0x80 | reg] + [0x00] * n
        rx = self._xfer(self._gyro_fd, tx)
        return rx[1:1 + n]

    def _write_gyro(self, reg, val):
        self._xfer(self._gyro_fd, [reg & 0x7F, val])

    def _wait_chip_id(self, read_fn, reg, expected, name, retries=10):
        """软复位后首次读可能无效，重试直到读到正确 CHIP_ID。"""
        last = None
        for _ in range(retries):
            last = read_fn(reg, 1)[0]
            if last == expected:
                return
            time.sleep(0.01)
        raise Bmi088Error(f'{name} CHIP_ID 异常: 0x{last:02X}')

    # --------------------------------------------------------------- 初始化
    def init(self, acc_range='6g', gyro_range='2000dps'):
        if acc_range not in ACC_RANGE_CFG:
            raise Bmi088Error(f'不支持的加速度量程: {acc_range}')
        if gyro_range not in GYRO_RANGE_CFG:
            raise Bmi088Error(f'不支持的陀螺仪量程: {gyro_range}')

        acc_code, self.acc_lsb_per_g = ACC_RANGE_CFG[acc_range]
        gyro_code, self.gyro_lsb_per_dps = GYRO_RANGE_CFG[gyro_range]
        self.acc_range_name = acc_range
        self.gyro_range_name = gyro_range

        # 加速度计：上电默认 suspend，需要先复位再使能
        self._write_acc(ACC_SOFTRESET, SOFTRESET_CMD)
        time.sleep(0.01)
        self._wait_chip_id(self._read_acc, ACC_CHIP_ID, ACC_CHIP_ID_VALUE,
                           '加速度计')
        self._write_acc(ACC_RANGE, acc_code)
        self._write_acc(ACC_PWR_CONF, 0x00)
        self._write_acc(ACC_PWR_CTRL, 0x04)
        time.sleep(0.05)
        if self._read_acc(ACC_ERR_REG, 1)[0] != 0x00:
            raise Bmi088Error('加速度计初始化后 ERR_REG 非 0')

        # 陀螺仪：上电默认 normal mode，复位后重新配置量程
        self._write_gyro(GYRO_SOFTRESET, SOFTRESET_CMD)
        time.sleep(0.03)
        self._wait_chip_id(self._read_gyro, GYRO_CHIP_ID, GYRO_CHIP_ID_VALUE,
                           '陀螺仪')
        self._write_gyro(GYRO_RANGE, gyro_code)
        # 带宽默认 532Hz ODR，100Hz 采样会发生混叠，表现为十几 dps 的"噪声"；
        # 0x03 = 47Hz，低于奈奎斯特频率。
        self._write_gyro(GYRO_BANDWIDTH, 0x03)
        self._write_gyro(GYRO_LPM1, 0x00)
        time.sleep(0.03)

    # ----------------------------------------------------------------- 读数
    def read_raw(self):
        acc = struct.unpack('<hhh', self._read_acc(ACC_DATA, 6))
        gyro = struct.unpack('<hhh', self._read_gyro(GYRO_DATA, 6))
        return acc, gyro

    def read_scaled(self):
        """返回 ((ax, ay, az) m/s^2, (gx, gy, gz) rad/s)。"""
        acc, gyro = self.read_raw()
        accel = tuple(v / self.acc_lsb_per_g * GRAVITY for v in acc)
        gyro_r = tuple(math.radians(v / self.gyro_lsb_per_dps) for v in gyro)
        return accel, gyro_r

    def read_temperature(self):
        """加速度计温度，单位摄氏度。"""
        d = self._read_acc(ACC_TEMP_LSB, 2)
        raw = (d[1] << 8) | d[0]
        return (raw >> 5) * 0.125

    def read_chip_ids(self):
        return (self._read_acc(ACC_CHIP_ID, 1)[0],
                self._read_gyro(GYRO_CHIP_ID, 1)[0])
