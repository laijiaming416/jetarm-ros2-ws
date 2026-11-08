"""视觉抓取启动（需要先启动 arm_node 与 camera_node）。

用法：
    ros2 launch jetarm_grasp grasp.launch.py
    ros2 launch jetarm_grasp grasp.launch.py hue:=118 min_area:=8000
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    hue = LaunchConfiguration('hue')
    min_area = LaunchConfiguration('min_area')
    height = LaunchConfiguration('height_mm')

    return LaunchDescription([
        DeclareLaunchArgument('hue', default_value='118'),
        DeclareLaunchArgument('min_area', default_value='8000.0'),
        DeclareLaunchArgument('height_mm', default_value='70.0'),
        Node(
            package='jetarm_grasp',
            executable='grasp_node',
            name='grasp_node',
            output='screen',
            parameters=[{
                'hue': ParameterValue(hue, value_type=int),
                'min_area': ParameterValue(min_area, value_type=float),
                'height_mm': ParameterValue(height, value_type=float),
            }],
        ),
    ])