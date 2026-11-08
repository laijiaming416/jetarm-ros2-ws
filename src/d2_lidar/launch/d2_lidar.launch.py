"""雷达启动（默认只发布 /scan，不启动避障节点）。

避障节点会和遥控抢 /cmd_vel（armed=false 时也会发零速），建图/遥控阶段保持关闭。
需要时再开：
    ros2 launch d2_lidar d2_lidar.launch.py avoid:=true armed:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    armed = LaunchConfiguration('armed')
    avoid = LaunchConfiguration('avoid')
    port = LaunchConfiguration('port')

    return LaunchDescription([
        DeclareLaunchArgument('armed', default_value='false',
                              description='避障节点是否允许输出非零速度'),
        DeclareLaunchArgument('avoid', default_value='false',
                              description='是否启动避障节点（默认关闭，避免与遥控抢 /cmd_vel）'),
        DeclareLaunchArgument('port', default_value='auto',
                              description='雷达串口，auto 表示自动探测 55AA 数据流'),
        Node(package='d2_lidar', executable='d2_scan_node', name='d2_scan',
             output='screen',
             parameters=[{'port': port, 'mount_height': 0.285,
                          'frame_id': 'laser', 'parent_frame': 'base_link'}]),
        Node(package='d2_lidar', executable='d2_avoid_node', name='d2_avoid',
             output='screen',
             condition=IfCondition(avoid),
             parameters=[{'armed': armed}]),
    ])
