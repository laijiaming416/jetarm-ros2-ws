"""语音三节点一起启动（ASR / 对话 / TTS）。

用法：
    ros2 launch jetarm_bringup voice.launch.py
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _include(package, file_name):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package), 'launch',
                                  file_name])))


def generate_launch_description():
    return LaunchDescription([
        _include('jetarm_asr', 'asr.launch.py'),
        _include('jetarm_tts', 'tts.launch.py'),
        _include('jetarm_dialog', 'dialog.launch.py'),
    ])
