"""传感器尺寸检查：雷达 + 车体轮廓 + RViz（不启动底盘 / IMU / 遥控）。

用于确认点云与车体轮廓的比例、雷达安装位置是否一致：
    - 栅格每格 1 m
    - 车体轮廓约 0.30 m × 0.26 m
    - 固定坐标系 base_link，点云按 base_link -> laser 变换叠加

用法：
    ros2 launch mecanum_nav sensor_check.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    x_offset_m = LaunchConfiguration('x_offset_m')
    y_offset_m = LaunchConfiguration('y_offset_m')
    yaw_offset_deg = LaunchConfiguration('yaw_offset_deg')
    mirror = LaunchConfiguration('mirror')

    scan = Node(
        package='d2_lidar', executable='d2_scan_node',
        name='d2_scan', output='screen',
        parameters=[{'mount_height': 0.285,
                     'x_offset_m': x_offset_m,
                     'y_offset_m': y_offset_m,
                     'yaw_offset_deg': yaw_offset_deg,
                     'mirror': mirror}])

    outline = Node(
        package='mecanum_nav', executable='car_outline',
        name='car_outline', output='screen')

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', PathJoinSubstitution(
            [FindPackageShare('mecanum_nav'), 'config', 'sensor_check.rviz'])],
        additional_env={'QT_QPA_PLATFORM': 'xcb'})

    return LaunchDescription([
        DeclareLaunchArgument('x_offset_m', default_value='-0.05',
                              description='雷达相对 base_link 的前后偏移（前为正）'),
        DeclareLaunchArgument('y_offset_m', default_value='0.0',
                              description='雷达相对 base_link 的左右偏移（左为正）'),
        DeclareLaunchArgument('yaw_offset_deg', default_value='80.0',
                              description='激光安装偏角（度）'),
        DeclareLaunchArgument('mirror', default_value='true',
                              description='雷达角方向为顺时针，按角度取反镜像'),
        scan,
        outline,
        rviz,
    ])
