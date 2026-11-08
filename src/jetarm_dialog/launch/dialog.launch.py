"""对话节点启动。

用法：
    ros2 launch jetarm_dialog dialog.launch.py
    ros2 launch jetarm_dialog dialog.launch.py wake_word:=你好小赖
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    wake = LaunchConfiguration('wake_word')
    window = LaunchConfiguration('wake_window_s')
    return LaunchDescription([
        DeclareLaunchArgument('wake_word', default_value='你好小车'),
        DeclareLaunchArgument('wake_window_s', default_value='8.0'),
        Node(
            package='jetarm_dialog',
            executable='dialog_node',
            name='dialog_node',
            output='screen',
            parameters=[{
                'wake_word': ParameterValue(wake, value_type=str),
                'wake_window_s': ParameterValue(window, value_type=float),
            }],
        ),
    ])