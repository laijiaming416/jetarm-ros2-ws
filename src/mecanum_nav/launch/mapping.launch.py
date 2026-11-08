"""建图：底盘 + 雷达 + slam_toolbox（mapping 异步模式）。

用法：
    ros2 launch mecanum_nav mapping.launch.py
    ros2 launch mecanum_nav mapping.launch.py rviz:=true
    ros2 launch mecanum_nav mapping.launch.py yaw_offset_deg:=5.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    yaw_offset_deg = LaunchConfiguration('yaw_offset_deg')
    mirror = LaunchConfiguration('mirror')
    x_offset_m = LaunchConfiguration('x_offset_m')
    y_offset_m = LaunchConfiguration('y_offset_m')
    rviz = LaunchConfiguration('rviz')
    map_name = LaunchConfiguration('map_name')

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

    outline = Node(
        package='mecanum_nav', executable='car_outline',
        name='car_outline', output='screen')

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('slam_toolbox'),
                                  'launch', 'online_async_launch.py'])),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': PathJoinSubstitution(
                [FindPackageShare('mecanum_nav'), 'config', 'slam_params.yaml']),
        }.items())

    web = Node(
        package='mecanum_nav', executable='map_web_view',
        name='map_web_view', output='screen',
        parameters=[{'port': 8080}])

    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', PathJoinSubstitution(
            [FindPackageShare('mecanum_nav'), 'config', 'mapping.rviz'])],
        # 板子的桌面是 labwc + Xwayland，Qt 默认走 wayland 会让 OGRE 建 GLX 窗口失败，
        # 这里强制 rviz2 用 xcb 后端。
        additional_env={'QT_QPA_PLATFORM': 'xcb'},
        condition=IfCondition(rviz))

    mux = Node(
        package='mecanum_teleop', executable='cmd_mux',
        name='cmd_mux', output='screen')

    restorer = Node(
        package='mecanum_nav', executable='pose_restorer',
        name='pose_restorer', output='screen',
        parameters=[{'map_file': ParameterValue(map_name, value_type=str),
                     'use_sim_time': use_sim_time,
                     'publish_initial': False,
                     'global_fallback': False,
                     'check_amcl': False,
                     'amcl_wait': 0.0}])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('yaw_offset_deg', default_value='80.0',
                              description='激光安装偏角（度）：左侧墙标定，车头在雷达 +90° 附近'),
        DeclareLaunchArgument('mirror', default_value='true',
                              description='雷达角方向为顺时针，按角度取反镜像'),
        DeclareLaunchArgument('x_offset_m', default_value='-0.05',
                              description='雷达相对 base_link 的前后偏移（前为正；车长300mm、雷达距车尾100mm）'),
        DeclareLaunchArgument('y_offset_m', default_value='0.0',
                              description='雷达相对 base_link 的左右偏移（左为正，实测居中）'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='是否在开发板显示器上打开 RViz2'),
        DeclareLaunchArgument('map_name', default_value='map',
                              description='与保存地图时用的名字一致，用于记录末位姿'),
        base,
        imu,
        scan,
        mux,
        outline,
        slam,
        web,
        rviz_node,
        restorer,
    ])
