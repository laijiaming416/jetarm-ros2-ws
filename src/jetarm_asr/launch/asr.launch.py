"""ASR 节点启动（模型加载 30~90 秒）。

用法：
    ros2 launch jetarm_asr asr.launch.py
    ros2 launch jetarm_asr asr.launch.py threshold:=400
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    threshold = LaunchConfiguration('threshold')
    session = LaunchConfiguration('session_s')
    return LaunchDescription([
        DeclareLaunchArgument('threshold', default_value='400.0',
                              description='固定起点阈值；0 表示按底噪自动'),
        DeclareLaunchArgument('session_s', default_value='0.0',
                              description='0 表示常驻不设上限'),
        Node(
            package='jetarm_asr',
            executable='asr_node',
            name='asr_node',
            output='screen',
            parameters=[{
                'threshold': ParameterValue(threshold, value_type=float),
                'session_s': ParameterValue(session, value_type=float),
            }],
        ),
    ])