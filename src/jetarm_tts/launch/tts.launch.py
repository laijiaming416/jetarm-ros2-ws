"""TTS 节点启动（首次预热约 31 秒）。

用法：
    ros2 launch jetarm_tts tts.launch.py
    ros2 launch jetarm_tts tts.launch.py cue_after_speech:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    device = LaunchConfiguration('audio_device')
    cue = LaunchConfiguration('cue_after_speech')
    return LaunchDescription([
        DeclareLaunchArgument('audio_device', default_value='plughw:CARD=Device,DEV=0'),
        DeclareLaunchArgument('cue_after_speech', default_value='false'),
        Node(
            package='jetarm_tts',
            executable='tts_node',
            name='tts_node',
            output='screen',
            parameters=[{
                'device': ParameterValue(device, value_type=str),
                'cue_after_speech': ParameterValue(cue, value_type=bool),
            }],
        ),
    ])