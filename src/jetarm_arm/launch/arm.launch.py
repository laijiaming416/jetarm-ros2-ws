"""机械臂单独启动。

用法：
    ros2 launch jetarm_arm arm.launch.py
    ros2 launch jetarm_arm arm.launch.py port:=/dev/ttyUSB1
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    port = LaunchConfiguration('port')
    state_rate = LaunchConfiguration('state_rate_hz')

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='auto',
                              description='机械臂串口；auto 表示只读探测'),
        DeclareLaunchArgument('state_rate_hz', default_value='5.0'),
        Node(
            package='jetarm_arm',
            executable='arm_node',
            name='arm_node',
            output='screen',
            parameters=[{
                'port': ParameterValue(port, value_type=str),
                'state_rate_hz': ParameterValue(state_rate, value_type=float),
            }],
        ),
    ])