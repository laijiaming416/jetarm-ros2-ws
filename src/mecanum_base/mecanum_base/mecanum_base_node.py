#!/usr/bin/env python3
"""麦轮底盘 ROS 2 驱动节点。

硬件（实测确认）：
    设备 /dev/i2c-4，地址 0x34
    0x00  电池电压 2 字节 mV
    0x14  电机类型 写 3
    0x15  极性     写 0
    0x33  四轮速度 4 个有符号字节，单位 = 每 10ms 的编码器计数
    0x3c  四轮计数 4 × int32

实测要点：
    1. 初始化必须带延时（0.1 / 0.5 / 0.3 s），否则速度写入不生效；
    2. MCU 不锁存速度：单次写入后 100ms 内即停，必须持续高频刷新
       （50Hz 只能跑出 68% 且产生断续抖动；200Hz 约 91%，500Hz 约 93%）；
    3. 计数寄存器偶发返回脏值（约 3~10%，跳变达数十万），必须双读校验；
    4. 标定：3960 counts/圈、轮径 96.5mm，1 raw ≈ 0.00766 m/s；
    5. MCU 内部更新周期 10ms；
    6. 寄存器顺序实测：索引0=左前 1=左后 2=右前 3=右后；轮速组合与运动关系实测：
         前进 = (+,-,-,+)   后退 = (-,+,+,-)
         右移 = (+,+,+,+)   左移 = (-,-,-,-)
         旋转 = (+,-,+,-)（右旋） 或 (-,+,-,+)（左旋）
       节点内实测（前进/左移/左旋三段），里程计换算符号用 BASE_SIGNS=(1,-1,-1,1)。

接口：
    订阅 /cmd_vel_final（geometry_msgs/Twist，m/s 与 rad/s，符合 ROS 标准，由 cmd_mux 仲裁输出）
    发布 /odom（nav_msgs/Odometry，30Hz）
    发布 TF odom -> base_link
    服务 /read_battery（std_srvs/Trigger，仅停车时可读）

电池保护：
    只在停车时读 0x00 电压：正常每 battery_period_s（默认 10s）一次，
    低于 battery_min_voltage（10.5V）或连续 battery_fail_max（3）次读失败时锁定零速，
    锁定期间改为每 battery_low_period_s（默认 3s）读一次并打印，
    回升到 battery_recover_voltage（10.7V）以上自动解除。行进中不读电压。

偏航融合（use_imu_yaw，默认启用）：
    订阅 /imu/data_raw，取陀螺 z 轴角速度，启动时静止标定零偏后积分成 yaw 增量；
    里程计 x/y 仍由轮速计数计算，yaw 改用 IMU 增量；IMU 超时或未启动自动回退轮速 yaw。
"""

import fcntl
import math
import os
import struct
import threading
import time

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

I2C_SLAVE = 0x0703
REG_BATTERY = 0x00
REG_MOTOR_TYPE = 0x14
REG_POLARITY = 0x15
REG_SPEED = 0x33
REG_COUNTS = 0x3c

# 电压读数合理性范围：3S 锂电正常区间，超出即视为脏值丢弃
BATTERY_PLAUSIBLE_MIN = 6.0
BATTERY_PLAUSIBLE_MAX = 30.0

# 里程计把原始计数换算回车体坐标系时使用的轮组符号。
# 2026-11-01 单节点实测（+vx / +vy / +wz 三段，物理方向分别为前 / 左 / 左旋），
# 用 (1,-1,-1,1) 时三段 odom 增量与物理运动同号。
BASE_SIGNS = (1, -1, -1, 1)


class I2CBus:
    """单 fd 的 I2C 访问，带互斥锁，写零与读计数可并发安全调用。"""

    def __init__(self, path, address):
        self.fd = os.open(path, os.O_RDWR)
        fcntl.ioctl(self.fd, I2C_SLAVE, address)
        self.lock = threading.Lock()

    def _write(self, data):
        os.write(self.fd, bytes(data))

    def write_reg(self, reg, payload):
        with self.lock:
            self._write([reg] + list(payload))

    def read_reg(self, reg, size):
        with self.lock:
            self._write([reg])
            return os.read(self.fd, size)

    def set_raw(self, values):
        self.write_reg(REG_SPEED, struct.pack('4b', *[int(v) for v in values]))

    def counts(self):
        data = self.read_reg(REG_COUNTS, 16)
        if len(data) != 16:
            raise OSError('counts short read: %d' % len(data))
        return struct.unpack('<4i', data)

    def counts_validated(self, tolerance=100, retries=4):
        """取三次读数的中位数并校验一致性，规避固件偶发脏值（含成对脏值）。"""
        for _ in range(retries):
            samples = [self.counts() for _ in range(3)]
            median = tuple(sorted(s[i] for s in samples)[1] for i in range(4))
            if all(abs(s[i] - median[i]) <= tolerance for s in samples for i in range(4)):
                return median
        return None

    def battery(self):
        data = self.read_reg(REG_BATTERY, 2)
        if len(data) != 2:
            raise OSError('battery short read: %d' % len(data))
        return int.from_bytes(data, 'little') / 1000.0

    def initialize(self):
        """实测必须带延时的初始化序列，缺延时速度写入不生效。"""
        self.set_raw((0, 0, 0, 0))
        time.sleep(0.1)
        self.write_reg(REG_MOTOR_TYPE, (3,))
        time.sleep(0.5)
        self.write_reg(REG_POLARITY, (0,))
        time.sleep(0.3)
        self.set_raw((0, 0, 0, 0))
        time.sleep(0.1)

    def close(self):
        try:
            os.close(self.fd)
        except OSError:
            pass


class MecanumBase(Node):

    def __init__(self):
        super().__init__('mecanum_base')

        self.declare_parameter('i2c_bus', '/dev/i2c-4')
        self.declare_parameter('i2c_address', 0x34)
        self.declare_parameter('wheel_diameter', 0.0965)
        self.declare_parameter('counts_per_rev', 3960.0)
        self.declare_parameter('turn_lever', 0.200)
        self.declare_parameter('wheel_signs', [1, 1, 1, 1])
        self.declare_parameter('max_raw', 72)
        self.declare_parameter('max_linear', 0.30)
        self.declare_parameter('max_angular', 1.00)
        self.declare_parameter('cmd_rate_hz', 200.0)
        self.declare_parameter('watchdog_ms', 200.0)
        self.declare_parameter('odom_rate_hz', 30.0)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('battery_period_s', 10.0)
        self.declare_parameter('battery_low_period_s', 3.0)
        self.declare_parameter('battery_min_voltage', 10.5)
        self.declare_parameter('battery_recover_voltage', 10.7)
        self.declare_parameter('battery_fail_max', 3)
        self.declare_parameter('use_imu_yaw', True)
        self.declare_parameter('imu_topic', '/imu/data_raw')
        self.declare_parameter('imu_timeout_ms', 200.0)
        self.declare_parameter('imu_yaw_sign', 1)
        self.declare_parameter('gyro_bias_samples', 200)
        self.declare_parameter('gyro_lpf_alpha', 0.3)

        self.bus_path = self.get_parameter('i2c_bus').value
        self.bus_addr = int(self.get_parameter('i2c_address').value)
        self.wheel_d = float(self.get_parameter('wheel_diameter').value)
        self.counts_per_rev = float(self.get_parameter('counts_per_rev').value)
        self.turn_lever = float(self.get_parameter('turn_lever').value)
        self.signs = [int(v) for v in self.get_parameter('wheel_signs').value]
        if len(self.signs) != 4 or any(v not in (-1, 1) for v in self.signs):
            raise ValueError('wheel_signs 必须是 4 个 ±1')
        self.max_raw = int(self.get_parameter('max_raw').value)
        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_angular = float(self.get_parameter('max_angular').value)
        self.cmd_period = 1.0 / max(1.0, float(self.get_parameter('cmd_rate_hz').value))
        self.watchdog_s = max(0.05, float(self.get_parameter('watchdog_ms').value) / 1000.0)
        self.odom_period = 1.0 / max(1.0, float(self.get_parameter('odom_rate_hz').value))
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.battery_period = max(1.0, float(self.get_parameter('battery_period_s').value))
        self.battery_low_period = max(0.5, float(self.get_parameter('battery_low_period_s').value))
        self.battery_min = float(self.get_parameter('battery_min_voltage').value)
        self.battery_recover = float(self.get_parameter('battery_recover_voltage').value)
        self.battery_fail_max = max(1, int(self.get_parameter('battery_fail_max').value))
        self.use_imu_yaw = bool(self.get_parameter('use_imu_yaw').value)
        self.imu_topic = str(self.get_parameter('imu_topic').value)
        self.imu_timeout = max(0.05, float(self.get_parameter('imu_timeout_ms').value) / 1000.0)
        self.imu_yaw_sign = -1.0 if int(self.get_parameter('imu_yaw_sign').value) < 0 else 1.0
        self.gyro_bias_samples = max(10, int(self.get_parameter('gyro_bias_samples').value))
        self.gyro_lpf_alpha = min(1.0, max(0.01, float(self.get_parameter('gyro_lpf_alpha').value)))

        # 电池状态：电压过低或读不到时锁定底盘
        self.battery_voltage = None
        self.battery_fail_count = 0
        self.battery_locked = False
        # 必须在首次 update_battery() 之前创建发布器
        self.battery_pub = self.create_publisher(Float32, 'battery_voltage', 10)

        # IMU 偏航融合状态
        self.imu_lock = threading.Lock()
        self.imu_yaw = 0.0
        self.imu_yaw_ref = 0.0
        self.imu_bias = 0.0
        self.imu_bias_sum = 0.0
        self.imu_bias_count = 0
        self.imu_bias_ready = False
        self.imu_last_time = None
        self.imu_last_recv = 0.0
        self.imu_filtered = None
        self.imu_active = False
        self.wheel_still_since = None

        # 运动学换算
        self.metres_per_count = math.pi * self.wheel_d / self.counts_per_rev
        self.raw_per_mps = self.counts_per_rev / (math.pi * self.wheel_d) * 0.01

        self.bus = I2CBus(self.bus_path, self.bus_addr)
        self.bus.initialize()

        # 启动时先确认电池状态：读不到或电压过低都不允许运动
        for _ in range(self.battery_fail_max):
            if self.update_battery():
                break
            time.sleep(0.2)

        self.cmd = (0.0, 0.0, 0.0)
        self.cmd_stamp = None
        self.cmd_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.i2c_fail_count = 0
        self.watchdog_armed = False
        self.watchdog_logged = False
        self.battery_next_due = time.monotonic() + (
            self.battery_low_period if self.battery_locked else self.battery_period)

        self.pose = [0.0, 0.0, 0.0]
        self.prev_counts = None
        self.prev_time = None
        self.last_odom_pub = 0.0

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.create_subscription(Twist, '/cmd_vel_final', self.on_cmd_vel, 10)
        if self.use_imu_yaw:
            self.create_subscription(Imu, self.imu_topic, self.on_imu, 10)
        self.create_service(Trigger, '/read_battery', self.on_read_battery)
        self.create_timer(0.5, self.on_battery_tick)

        self.control_thread = threading.Thread(target=self.control_loop, daemon=True)
        self.control_thread.start()

        self.get_logger().info(
            '麦轮底盘已就绪：%s 0x%02x，raw_per_mps=%.4f，%dHz 下发，看门狗 %dms'
            % (self.bus_path, self.bus_addr, self.raw_per_mps,
               int(1.0 / self.cmd_period), int(self.watchdog_s * 1000)))
        if self.use_imu_yaw:
            self.get_logger().info('IMU 偏航融合已启用（%s），启动时请保持车体静止以标定零偏'
                                   % self.imu_topic)

    # ---------- 输入 ----------

    def on_cmd_vel(self, msg):
        vx = self.clamp(msg.linear.x, self.max_linear)
        vy = self.clamp(msg.linear.y, self.max_linear)
        wz = self.clamp(msg.angular.z, self.max_angular)
        with self.cmd_lock:
            self.cmd = (vx, vy, wz)
            self.cmd_stamp = time.monotonic()
            self.watchdog_armed = True

    @staticmethod
    def clamp(value, limit):
        if math.isnan(value) or math.isinf(value):
            return 0.0
        return max(-limit, min(limit, value))

    # ---------- IMU 偏航融合 ----------

    def on_imu(self, msg):
        """只取陀螺 z 轴角速度：滤波、扣零偏、积分成相对 yaw。"""
        raw = msg.angular_velocity.z * self.imu_yaw_sign
        if not math.isfinite(raw):
            return

        if self.imu_filtered is None:
            self.imu_filtered = raw
        else:
            a = self.gyro_lpf_alpha
            self.imu_filtered = a * raw + (1.0 - a) * self.imu_filtered
        gz = self.imu_filtered
        now = time.monotonic()

        with self.imu_lock:
            if not self.imu_bias_ready:
                self.imu_bias_sum += gz
                self.imu_bias_count += 1
                if self.imu_bias_count >= self.gyro_bias_samples:
                    self.imu_bias = self.imu_bias_sum / self.imu_bias_count
                    self.imu_bias_ready = True
                    self.imu_last_time = now
                    self.imu_last_recv = now
                    self.get_logger().info(
                        'IMU 零偏标定完成：%.4f rad/s，启用 IMU 偏航' % self.imu_bias)
                return

            dt = 0.0 if self.imu_last_time is None else (now - self.imu_last_time)
            self.imu_last_time = now
            self.imu_last_recv = now
            if dt <= 0.0 or dt > 0.1:
                return
            self.imu_yaw += (gz - self.imu_bias) * dt

            # 轮速确认静止超过 2 秒后，才缓慢跟踪零偏（时间常数约 20s），
            # 避免把手动慢转或推行当成零偏吸收掉。
            still_since = self.wheel_still_since
            if (self.is_stopped() and still_since is not None
                    and (now - still_since) > 2.0):
                self.imu_bias = 0.9995 * self.imu_bias + 0.0005 * gz

    # ---------- 按需读取电池电压 ----------

    def is_stopped(self):
        """最近没有新指令、或指令为零，都算停车；锁定期间输出恒为零。"""
        if self.battery_locked:
            return True
        with self.cmd_lock:
            cmd = self.cmd
            stamp = self.cmd_stamp
        if stamp is None or (time.monotonic() - stamp) > self.watchdog_s:
            return True
        return all(abs(v) <= 1e-6 for v in cmd)

    def on_read_battery(self, request, response):
        """仅在车体确实没有输出速度时读 0x00，避免运行中读 I2C 干扰速度下发。"""
        if not self.is_stopped():
            response.success = False
            response.message = '车辆运动中，拒绝读取电池电压'
            return response

        try:
            voltage = self.bus.battery()
        except OSError as exc:
            self.get_logger().error('读取电池电压失败：%s' % exc)
            response.success = False
            response.message = 'I2C 读取失败'
            return response

        if not BATTERY_PLAUSIBLE_MIN <= voltage <= BATTERY_PLAUSIBLE_MAX:
            self.get_logger().warn('电压读数异常（%.2fV）' % voltage)
            response.success = False
            response.message = '电压读数异常'
            return response

        self.get_logger().info('电池电压 %.2f V' % voltage)
        response.success = True
        response.message = '%.2f V' % voltage
        return response

    # ---------- 电池监控 ----------

    def on_battery_tick(self):
        """只在停车且到达周期时读一次；低压或读失败锁定时改为 3 秒一次。"""
        if not self.is_stopped():
            return
        now = time.monotonic()
        if now < self.battery_next_due:
            return
        self.update_battery()
        interval = self.battery_low_period if self.battery_locked else self.battery_period
        self.battery_next_due = time.monotonic() + interval

    def update_battery(self):
        """读一次电压并更新锁定状态，返回本次是否读取成功。"""
        try:
            voltage = self.bus.battery()
        except OSError as exc:
            self.battery_fail_count += 1
            self.get_logger().warn('读取电池电压失败（%d/%d）：%s'
                                   % (self.battery_fail_count, self.battery_fail_max, exc))
            if self.battery_fail_count >= self.battery_fail_max and not self.battery_locked:
                self.battery_locked = True
                self.get_logger().error('连续 %d 次读取电压失败，已禁止底盘运动'
                                        % self.battery_fail_max)
            return False

        if not BATTERY_PLAUSIBLE_MIN <= voltage <= BATTERY_PLAUSIBLE_MAX:
            self.battery_fail_count += 1
            self.get_logger().warn('电压读数异常（%.2fV），已丢弃（%d/%d）'
                                   % (voltage, self.battery_fail_count, self.battery_fail_max))
            if self.battery_fail_count >= self.battery_fail_max and not self.battery_locked:
                self.battery_locked = True
                self.get_logger().error('连续 %d 次电压读数异常，已禁止底盘运动'
                                        % self.battery_fail_max)
            return False

        self.battery_fail_count = 0
        self.battery_voltage = voltage
        self.battery_pub.publish(Float32(data=float(voltage)))
        self.get_logger().info('电池电压 %.2f V' % voltage)

        if voltage < self.battery_min:
            if not self.battery_locked:
                self.battery_locked = True
                self.get_logger().warn('电压低于 %.1fV（当前 %.2fV），已禁止底盘运动'
                                       % (self.battery_min, voltage))
        elif voltage >= self.battery_recover and self.battery_locked:
            self.battery_locked = False
            self.get_logger().info('电压恢复至 %.2fV，已解除运动限制' % voltage)
        return True

    # ---------- 运动学 ----------

    def raw_from_cmd(self, vx, vy, wz):
        """车体速度 -> 四轮 raw（每 10ms 计数），按板上模型的组合方式。"""
        turn = self.turn_lever * wz
        base = (vx - vy - turn, -vx - vy + turn, -vx - vy - turn, vx - vy + turn)
        raw = []
        for i, b in enumerate(base):
            v = int(self.signs[i] * b * self.raw_per_mps)
            raw.append(max(-self.max_raw, min(self.max_raw, v)))
        return raw

    def integrate(self, delta, dt):
        u = [delta[i] * self.metres_per_count * BASE_SIGNS[i] * self.signs[i] for i in range(4)]
        a, b, c, d = u
        dx = (a + b + c + d) / 4.0
        dy = (-a + b + c - d) / 4.0
        dyaw_wheel = (-a - b + c + d) / (4.0 * self.turn_lever)

        # 偏航优先用 IMU 积分增量，超时则回退轮速
        with self.imu_lock:
            imu_dyaw = self.imu_yaw - self.imu_yaw_ref
            fresh = (self.use_imu_yaw and self.imu_bias_ready
                     and (time.monotonic() - self.imu_last_recv) <= self.imu_timeout)
            self.imu_yaw_ref = self.imu_yaw
        dyaw = imu_dyaw if fresh else dyaw_wheel

        if fresh != self.imu_active:
            self.imu_active = fresh
            if fresh:
                self.get_logger().info('IMU 偏航可用，改用 IMU yaw')
            else:
                self.get_logger().warn('IMU 偏航不可用，回退轮速 yaw')

        if abs(dyaw) < 1e-9:
            cx, cy = dx, dy
        else:
            s = math.sin(dyaw) / dyaw
            cterm = (1.0 - math.cos(dyaw)) / dyaw
            cx = s * dx - cterm * dy
            cy = cterm * dx + s * dy
        yaw = self.pose[2]
        self.pose[0] += math.cos(yaw) * cx - math.sin(yaw) * cy
        self.pose[1] += math.sin(yaw) * cx + math.cos(yaw) * cy
        self.pose[2] = math.atan2(math.sin(yaw + dyaw), math.cos(yaw + dyaw))
        return dx, dy, dyaw

    # ---------- 控制循环 ----------

    def control_loop(self):
        next_tick = time.monotonic()
        while not self.stop_event.is_set():
            now = time.monotonic()

            with self.cmd_lock:
                cmd = self.cmd
                stamp = self.cmd_stamp
            if stamp is None or (now - stamp) > self.watchdog_s:
                cmd = (0.0, 0.0, 0.0)
                if self.watchdog_armed and not self.watchdog_logged:
                    self.get_logger().warn('看门狗触发：超过 %dms 没有新指令，已回零'
                                           % int(self.watchdog_s * 1000))
                    self.watchdog_logged = True
            elif self.watchdog_logged:
                self.watchdog_logged = False

            # 电池过低或读不到电压时强制零速
            if self.battery_locked:
                cmd = (0.0, 0.0, 0.0)

            try:
                self.bus.set_raw(self.raw_from_cmd(*cmd))
                self.i2c_fail_count = 0
            except OSError as exc:
                # 瞬时 I2C 错误不应终止控制线程，继续重试；MCU 自身 100ms 后会停
                self.i2c_fail_count += 1
                if self.i2c_fail_count == 1 or self.i2c_fail_count % 200 == 0:
                    self.get_logger().error(
                        'I2C 写速度失败（连续 %d 次）：%s'
                        % (self.i2c_fail_count, exc))

            if now - self.last_odom_pub >= self.odom_period:
                try:
                    counts = self.bus.counts_validated()
                except OSError as exc:
                    self.get_logger().error('I2C 读计数失败：%s' % exc)
                    counts = None
                if counts is not None:
                    if self.prev_counts is None:
                        self.prev_counts, self.prev_time = counts, now
                    else:
                        dt = now - self.prev_time
                        delta = tuple(c - p for c, p in zip(counts, self.prev_counts))
                        # 轮速静止判定，供 IMU 零偏慢跟踪使用
                        if 0.005 < dt < 0.5 and all(abs(d) <= 2 for d in delta):
                            if self.wheel_still_since is None:
                                self.wheel_still_since = now
                        else:
                            self.wheel_still_since = None
                        limit = dt * abs(self.max_raw) * 110 + 200
                        if 0.005 < dt < 0.5 and all(abs(d) <= limit for d in delta):
                            self.integrate(delta, dt)
                        elif all(abs(d) <= limit for d in delta):
                            self.get_logger().warn('计数采样间隔异常，丢弃本次积分')
                        else:
                            self.get_logger().warn('计数跳变超限，丢弃本次积分')
                        self.prev_counts, self.prev_time = counts, now
                self.publish_odom(cmd)
                self.last_odom_pub = now

            next_tick += self.cmd_period
            sleep = next_tick - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_tick = time.monotonic()

    def publish_odom(self, cmd):
        stamp = self.get_clock().now().to_msg()
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = self.pose[0]
        msg.pose.pose.position.y = self.pose[1]
        msg.pose.pose.orientation.z = math.sin(self.pose[2] / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.pose[2] / 2.0)

        vx, vy, wz = cmd
        tw = msg.twist.twist
        tw.linear.x = vx * math.cos(self.pose[2]) - vy * math.sin(self.pose[2])
        tw.linear.y = vx * math.sin(self.pose[2]) + vy * math.cos(self.pose[2])
        tw.angular.z = wz

        # x/y 与 vx/vy 可信；z、roll、pitch、vz 没有测量，给大值表示不可信，
        # 否则下游滤波器会把它们当成真实观测强行钉在 0。
        pose_cov = {0: 1e-3, 7: 1e-3, 14: 1e6, 21: 1e6, 28: 1e6,
                    35: 1e-3 if self.imu_active else 1e-2}
        twist_cov = {0: 1e-2, 7: 1e-2, 14: 1e6, 21: 1e6, 28: 1e6, 35: 1e-2}
        for i, v in pose_cov.items():
            msg.pose.covariance[i] = v
        for i, v in twist_cov.items():
            msg.twist.covariance[i] = v
        self.odom_pub.publish(msg)

        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.pose[0]
            tf.transform.translation.y = self.pose[1]
            tf.transform.rotation.z = math.sin(self.pose[2] / 2.0)
            tf.transform.rotation.w = math.cos(self.pose[2] / 2.0)
            self.tf_broadcaster.sendTransform(tf)

    # ---------- 退出 ----------

    def shutdown(self):
        self.stop_event.set()
        if self.control_thread.is_alive():
            self.control_thread.join(timeout=1.0)
        try:
            for _ in range(3):
                self.bus.set_raw((0, 0, 0, 0))
        except OSError:
            pass
        self.bus.close()


def main(args=None):
    rclpy.init(args=args)
    node = MecanumBase()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
