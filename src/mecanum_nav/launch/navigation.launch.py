"""导航：底盘 + 雷达 + 地图 + AMCL + nav2（静态地图定位导航）。

先决：用 mapping.launch.py 建好图并存成 ~/maps/map.yaml。

用法：
    ros2 launch mecanum_nav navigation.launch.py
    ros2 launch mecanum_nav navigation.launch.py map:=$HOME/maps/map.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    yaw_offset_deg = LaunchConfiguration('yaw_offset_deg')
    mirror = LaunchConfiguration('mirror')
    x_offset_m = LaunchConfiguration('x_offset_m')
    y_offset_m = LaunchConfiguration('y_offset_m')

    base = Node(
        package='mecanum_base', executable='mecanum_base_node',
        name='mecanum_base', output='screen',
        # 不跑 EKF，odom->base_link 由底盘节点发布
        parameters=[{'use_sim_time': use_sim_time}])

    imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('bmi088_imu'),
                                  'launch', 'bmi088_imu.launch.py'])),
        launch_arguments={'gyro_range': '500dps'}.items())

    scan = Node(
        package='d2_lidar', executable='d2_scan_node',
        name='d2_scan', output='screen',
        parameters=[{'mount_height': 0.285,
                     'x_offset_m': x_offset_m,
                     'y_offset_m': y_offset_m,
                     'yaw_offset_deg': yaw_offset_deg,
                     'mirror': mirror,
                     'use_sim_time': use_sim_time}])

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('nav2_bringup'),
                                  'launch', 'bringup_launch.py'])),
        launch_arguments={
            'slam': 'False',
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'params_file': PathJoinSubstitution(
                [FindPackageShare('mecanum_nav'), 'config', 'nav2_params.yaml']),
            'use_composition': 'False',
            'autostart': 'True',
        }.items())

    mux = Node(
        package='mecanum_teleop', executable='cmd_mux',
        name='cmd_mux', output='screen',
        parameters=[{'nav_topic': '/cmd_vel_nav'}])

    restorer = Node(
        package='mecanum_nav', executable='pose_restorer',
        name='pose_restorer', output='screen',
        parameters=[{'map_file': ParameterValue(map_yaml, value_type=str),
                     'use_sim_time': use_sim_time}])

    web = Node(
        package='mecanum_nav', executable='map_web_view',
        name='map_web_view', output='screen',
        parameters=[{'port': 8080}])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('map', default_value='/home/laijinxuan/maps/map.yaml',
                              description='地图 yaml 路径'),
        DeclareLaunchArgument('yaw_offset_deg', default_value='80.0'),
        DeclareLaunchArgument('mirror', default_value='true'),
        DeclareLaunchArgument('x_offset_m', default_value='-0.05',
                              description='雷达相对 base_link 的前后偏移（前为正；车长300mm、雷达距车尾100mm）'),
        DeclareLaunchArgument('y_offset_m', default_value='0.0',
                              description='雷达相对 base_link 的左右偏移（左为正，实测居中）'),
        base,
        imu,
        scan,
        mux,
        nav2,
        restorer,
        web,
    ])
