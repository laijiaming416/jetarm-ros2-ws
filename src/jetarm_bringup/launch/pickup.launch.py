"""取物相关节点：机械臂 + 相机 + 抓取 + 任务 + 语音。

建图阶段不需要这些节点；切换导航时由 mode_manager 自动拉起，
切回建图时自动停掉，避免建图/导航时 CPU 被占满。
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
        _include('jetarm_arm', 'arm.launch.py'),
        _include('jetarm_camera', 'camera.launch.py'),
        _include('jetarm_grasp', 'grasp.launch.py'),
        _include('jetarm_mission', 'mission.launch.py'),
        _include('jetarm_bringup', 'voice.launch.py'),
    ])