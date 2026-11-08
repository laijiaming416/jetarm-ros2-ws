"""发布车体轮廓折线，供 RViz 显示。

用法：
    ros2 launch mecanum_nav body.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mecanum_nav',
            executable='car_outline',
            name='car_outline',
            output='screen'),
    ])
