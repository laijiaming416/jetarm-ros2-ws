#!/usr/bin/env python3
"""BMI088 IMU 节点：100Hz 发布 sensor_msgs/Imu 到 /imu/data_raw。

只发布原始加速度和角速度，不提供姿态四元数：
orientation_covariance[0] = -1 表示无姿态估计。
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Temperature

from bmi088_imu.bmi088_driver import (
    Bmi088Driver,
    Bmi088Error,
    ACC_RANGE_CFG,
    GYRO_RANGE_CFG,
)


class Bmi088ImuNode(Node):

    def __init__(self):
        super().__init__('bmi088_imu')

        self.declare_parameter('acc_device', '/dev/bmi088_spi')
        self.declare_parameter('gyro_device', '/dev/bmi088_gyro')
        self.declare_parameter('spi_speed_hz', 2000000)
        self.declare_parameter('rate_hz', 100.0)
        self.declare_parameter('acc_range', '6g')
        self.declare_parameter('gyro_range', '2000dps')
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_temperature', False)
        self.declare_parameter('temperature_period', 10)
        self.declare_parameter('accel_variance', 0.001)
        self.declare_parameter('gyro_variance', 1.0e-5)

        self.acc_device = self.get_parameter('acc_device').value
        self.gyro_device = self.get_parameter('gyro_device').value
        self.spi_speed_hz = int(self.get_parameter('spi_speed_hz').value)
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.acc_range = self.get_parameter('acc_range').value
        self.gyro_range = self.get_parameter('gyro_range').value
        self.frame_id = self.get_parameter('frame_id').value
        self.publish_temperature = bool(
            self.get_parameter('publish_temperature').value)
        self.temperature_period = int(
            self.get_parameter('temperature_period').value)
        self.accel_variance = float(
            self.get_parameter('accel_variance').value)
        self.gyro_variance = float(
            self.get_parameter('gyro_variance').value)

        if self.rate_hz <= 0.0:
            raise Bmi088Error(f'rate_hz 必须大于 0，当前 {self.rate_hz}')
        if self.acc_range not in ACC_RANGE_CFG:
            raise Bmi088Error(f'不支持的加速度量程: {self.acc_range}')
        if self.gyro_range not in GYRO_RANGE_CFG:
            raise Bmi088Error(f'不支持的陀螺仪量程: {self.gyro_range}')

        self.driver = Bmi088Driver(
            acc_dev=self.acc_device,
            gyro_dev=self.gyro_device,
            spi_speed_hz=self.spi_speed_hz,
        )
        try:
            self.driver.open()
            self.driver.init(acc_range=self.acc_range,
                             gyro_range=self.gyro_range)
        except Bmi088Error:
            self.driver.close()
            raise

        acc_id, gyro_id = self.driver.read_chip_ids()
        self.get_logger().info(
            f'BMI088 已就绪: ACC_CHIP_ID=0x{acc_id:02X}, '
            f'GYRO_CHIP_ID=0x{gyro_id:02X}, '
            f'量程 acc={self.acc_range} gyro={self.gyro_range}, '
            f'SPI={self.spi_speed_hz}Hz, 发布={self.rate_hz}Hz')

        self.pub_imu = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.pub_temp = None
        if self.publish_temperature:
            self.pub_temp = self.create_publisher(
                Temperature, 'imu/temperature', 10)

        self._cycle = 0
        self._fail_count = 0
        self._prev_raw = None
        self._static_count = 0

        self.timer = self.create_timer(1.0 / self.rate_hz, self.on_timer)

    # ------------------------------------------------------------------
    def on_timer(self):
        try:
            accel, gyro = self.driver.read_scaled()
        except Bmi088Error as exc:
            self._fail_count += 1
            if self._fail_count % 100 == 1:
                self.get_logger().error(f'IMU 读取失败: {exc}')
            return
        self._fail_count = 0

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.linear_acceleration.x = accel[0]
        msg.linear_acceleration.y = accel[1]
        msg.linear_acceleration.z = accel[2]

        msg.angular_velocity.x = gyro[0]
        msg.angular_velocity.y = gyro[1]
        msg.angular_velocity.z = gyro[2]

        # 不做姿态融合，明确标记为无姿态
        msg.orientation.w = 1.0
        msg.orientation_covariance[0] = -1.0

        msg.linear_acceleration_covariance[0] = self.accel_variance
        msg.linear_acceleration_covariance[4] = self.accel_variance
        msg.linear_acceleration_covariance[8] = self.accel_variance

        msg.angular_velocity_covariance[0] = self.gyro_variance
        msg.angular_velocity_covariance[4] = self.gyro_variance
        msg.angular_velocity_covariance[8] = self.gyro_variance

        self.pub_imu.publish(msg)

        self._cycle += 1
        if (self.pub_temp is not None and
                self._cycle % max(1, self.temperature_period) == 0):
            try:
                temp = Temperature()
                temp.header.stamp = msg.header.stamp
                temp.header.frame_id = self.frame_id
                temp.temperature = float(self.driver.read_temperature())
                temp.variance = 0.0
                self.pub_temp.publish(temp)
            except Bmi088Error:
                pass

    def destroy_node(self):
        self.driver.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Bmi088ImuNode()
        rclpy.spin(node)
    except Bmi088Error as exc:
        rclpy.logging.get_logger('bmi088_imu').fatal(f'启动失败: {exc}')
        raise SystemExit(1) from exc
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
