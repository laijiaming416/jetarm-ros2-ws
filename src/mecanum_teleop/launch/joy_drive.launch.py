"""手动驾驶：启动麦轮底盘节点 + 手柄遥控节点。

用法：
    ros2 launch mecanum_teleop joy_drive.launch.py
    ros2 launch mecanum_teleop joy_drive.launch.py buzzer_port:=/dev/ttyUSB0

启动后按手柄 START 使能（蜂鸣器响 1 声），摇杆控制底盘；
再次按 START 停止（响 2 声）。左肩减速、右肩加速。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    buzzer_port = LaunchConfiguration('buzzer_port')

    base = Node(
        package='mecanum_base', executable='mecanum_base_node',
        name='mecanum_base', output='screen')

    joy = Node(
        package='mecanum_teleop', executable='joy_teleop',
        name='joy_teleop', output='screen',
        parameters=[{'buzzer_port': buzzer_port}])

    mux = Node(
        package='mecanum_teleop', executable='cmd_mux',
        name='cmd_mux', output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('buzzer_port', default_value='/dev/ttyUSB0',
                              description='机械臂主控板串口（蜂鸣器）'),
        base,
        joy,
        mux,
    ])
