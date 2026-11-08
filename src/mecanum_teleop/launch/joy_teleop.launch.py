"""单独启动手柄遥控节点（不启动底盘，仅发布 /cmd_vel）。

用法：
    ros2 launch mecanum_teleop joy_teleop.launch.py
    ros2 launch mecanum_teleop joy_teleop.launch.py device:=/dev/input/event7
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    device = LaunchConfiguration('device')

    return LaunchDescription([
        DeclareLaunchArgument(
            'device',
            default_value='/dev/input/by-id/usb-2563_USB_WirelessGamepad-event-joystick',
            description='手柄 evdev 设备路径'),
        Node(
            package='mecanum_teleop',
            executable='joy_teleop',
            name='joy_teleop',
            output='screen',
            parameters=[{'device': device}]),
    ])
