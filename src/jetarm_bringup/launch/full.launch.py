"""整套系统启动：底盘/雷达/导航 + 网页面板 + 启动后的小调整。

取物相关节点（机械臂/相机/抓取/任务/语音）不在这里启动：
切到导航模式时由 mode_manager 自动拉起 jetarm_bringup/pickup.launch.py，
切回建图时自动停掉，避免建图阶段 CPU 被占满。

用法：
    ros2 launch jetarm_bringup full.launch.py start_mode:=mapping map_name:=room7
    ros2 launch jetarm_bringup full.launch.py start_mode:=navigation map_name:=room7
"""

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _include(package, file_name, arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package), 'launch',
                                  file_name])),
        launch_arguments=(arguments or {}).items())


def generate_launch_description():
    start_mode = LaunchConfiguration('start_mode')
    map_name = LaunchConfiguration('map_name')
    map_yaml = LaunchConfiguration('map')
    rviz = LaunchConfiguration('rviz')

    robot = _include('mecanum_nav', 'robot.launch.py', {
        'start_mode': start_mode,
        'map_name': map_name,
        'map': map_yaml,
        'web': 'true',
        'rviz': rviz,
    })

    def _setup():
        # launch 不允许复用同一个 action 实例，两个时间点各建一个
        return ExecuteProcess(
            cmd=['bash', PathJoinSubstitution(
                [FindPackageShare('jetarm_bringup'), 'scripts',
                 'setup_system.sh'])],
            output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('start_mode', default_value='mapping',
                              description='mapping 或 navigation'),
        DeclareLaunchArgument('map_name', default_value='room7'),
        DeclareLaunchArgument('map', default_value='/home/laijinxuan/maps/room7.yaml'),
        DeclareLaunchArgument('rviz', default_value='false'),
        robot,
        TimerAction(period=5.0, actions=[_setup()]),
        TimerAction(period=30.0, actions=[_setup()]),
    ])