"""取物任务节点启动。

用法：
    ros2 launch jetarm_mission mission.launch.py
    ros2 launch jetarm_mission mission.launch.py skip_nav:=true skip_grasp:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    skip_nav = LaunchConfiguration('skip_nav')
    skip_grasp = LaunchConfiguration('skip_grasp')
    return LaunchDescription([
        DeclareLaunchArgument('skip_nav', default_value='false'),
        DeclareLaunchArgument('skip_grasp', default_value='false'),
        Node(
            package='jetarm_mission',
            executable='mission_node',
            name='mission_node',
            output='screen',
            parameters=[{
                'skip_nav': ParameterValue(skip_nav, value_type=bool),
                'skip_grasp': ParameterValue(skip_grasp, value_type=bool),
            }],
        ),
    ])