from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mecanum_base',
            executable='mecanum_base_node',
            name='mecanum_base',
            output='screen',
            parameters=[{
                'i2c_bus': '/dev/i2c-4',
                'max_linear': 0.30,
                'max_angular': 1.00,
                'cmd_rate_hz': 200.0,
                'watchdog_ms': 200.0,
                'publish_tf': True,
            }],
        ),
    ])
