"""腕部相机启动。

用法：
    ros2 launch jetarm_camera camera.launch.py
    ros2 launch jetarm_camera camera.launch.py publish_raw:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    device = LaunchConfiguration('camera_device')
    fps = LaunchConfiguration('fps')
    publish_raw = LaunchConfiguration('publish_raw')

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_device',
            default_value='/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_AY25B5F0034-video-index0',
            description='腕部相机设备；默认用序列号稳定路径'),
        DeclareLaunchArgument('fps', default_value='10'),
        DeclareLaunchArgument('publish_raw', default_value='false'),
        Node(
            package='jetarm_camera',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[{
                'device': ParameterValue(device, value_type=str),
                'fps': ParameterValue(fps, value_type=int),
                'publish_raw': ParameterValue(publish_raw, value_type=bool),
            }],
        ),
    ])