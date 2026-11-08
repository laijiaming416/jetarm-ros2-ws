"""BMI088 IMU 节点 + base_link->imu_link 静态 TF 一键启动。

安装位置（卷尺实测，2026-11-03）：
    距车头 25cm，车长 30cm -> base_link 后方 0.10m
    距车左 6.5cm，车宽 26cm -> base_link 左侧 0.065m
    高度约 0.20m
    IMU +y 朝车头、+x 朝车右 -> yaw = -90 度（-1.5708 rad）
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rate_hz = LaunchConfiguration('rate_hz')
    acc_range = LaunchConfiguration('acc_range')
    gyro_range = LaunchConfiguration('gyro_range')
    spi_speed_hz = LaunchConfiguration('spi_speed_hz')
    frame_id = LaunchConfiguration('frame_id')
    imu_x = LaunchConfiguration('imu_x')
    imu_y = LaunchConfiguration('imu_y')
    imu_z = LaunchConfiguration('imu_z')
    imu_yaw_rad = LaunchConfiguration('imu_yaw_rad')
    publish_tf = LaunchConfiguration('publish_tf')

    return LaunchDescription([
        DeclareLaunchArgument('rate_hz', default_value='100.0',
                              description='IMU 发布频率 Hz'),
        DeclareLaunchArgument('spi_speed_hz', default_value='2000000',
                              description='SPI 时钟 Hz（建议 1M~2M，10M 需先验证）'),
        DeclareLaunchArgument('acc_range', default_value='6g',
                              description='加速度量程: 3g/6g/12g/24g'),
        DeclareLaunchArgument('gyro_range', default_value='2000dps',
                              description='陀螺仪量程: 125/250/500/1000/2000dps'),
        DeclareLaunchArgument('frame_id', default_value='imu_link'),
        DeclareLaunchArgument('publish_tf', default_value='true',
                              description='是否发布 base_link -> imu_link 静态 TF'),
        DeclareLaunchArgument('imu_x', default_value='-0.10',
                              description='IMU 相对 base_link 前后位置，前为正'),
        DeclareLaunchArgument('imu_y', default_value='0.065',
                              description='IMU 相对 base_link 左右位置，左为正'),
        DeclareLaunchArgument('imu_z', default_value='0.20',
                              description='IMU 安装高度 m'),
        DeclareLaunchArgument('imu_yaw_rad', default_value='-1.5708',
                              description='IMU 相对 base_link 的偏航角（弧度）'),
        Node(package='bmi088_imu', executable='bmi088_imu_node',
             name='bmi088_imu', output='screen',
             parameters=[{
                 'rate_hz': rate_hz,
                 'spi_speed_hz': spi_speed_hz,
                 'acc_range': acc_range,
                 'gyro_range': gyro_range,
                 'frame_id': frame_id,
             }]),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='imu_tf', output='screen',
             arguments=['--x', imu_x, '--y', imu_y, '--z', imu_z,
                        '--roll', '0', '--pitch', '0', '--yaw', imu_yaw_rad,
                        '--frame-id', 'base_link',
                        '--child-frame-id', frame_id],
             condition=IfCondition(publish_tf)),
    ])
