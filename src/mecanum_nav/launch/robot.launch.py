#!/usr/bin/env python3
"""一键启动：底盘 + IMU + 雷达 + 网页面板 + 建图/导航模式管理。

用法：
    # 从建图开始
    ros2 launch mecanum_nav robot.launch.py start_mode:=mapping \
        map_name:=room5 map:=/home/laijinxuan/maps/room5.yaml

    # 直接进入导航
    ros2 launch mecanum_nav robot.launch.py start_mode:=navigation \
        map_name:=room5 map:=/home/laijinxuan/maps/room5.yaml

网页：http://<板卡IP>:8080/
"""
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_mode = LaunchConfiguration('start_mode')
    map_name = LaunchConfiguration('map_name')
    map_yaml = LaunchConfiguration('map')
    yaw_offset_deg = LaunchConfiguration('yaw_offset_deg')
    mirror = LaunchConfiguration('mirror')
    x_offset_m = LaunchConfiguration('x_offset_m')
    y_offset_m = LaunchConfiguration('y_offset_m')
    rviz = LaunchConfiguration('rviz')
    buzzer_port = LaunchConfiguration('buzzer_port')
    web_enable = LaunchConfiguration('web')

    base = Node(
        package='mecanum_base', executable='mecanum_base_node',
        name='mecanum_base', output='screen',
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

    mux = Node(
        package='mecanum_teleop', executable='cmd_mux',
        name='cmd_mux', output='screen',
        parameters=[{'nav_topic': '/cmd_vel_nav'}])

    web = Node(
        package='mecanum_nav', executable='map_web_view',
        name='map_web_view', output='screen',
        parameters=[{'port': 8080}],
        condition=IfCondition(web_enable))

    outline = Node(
        package='mecanum_nav', executable='car_outline',
        name='car_outline', output='screen')

    restorer = Node(
        package='mecanum_nav', executable='pose_restorer',
        name='pose_restorer', output='screen',
        parameters=[{'map_file': ParameterValue(map_name, value_type=str),
                     'use_sim_time': use_sim_time,
                     'publish_initial': False,
                     'global_fallback': False,
                     'check_amcl': False,
                     'amcl_wait': 0.0}])

    manager = Node(
        package='mecanum_nav', executable='mode_manager',
        name='mode_manager', output='screen',
        parameters=[{'map_name': ParameterValue(map_name, value_type=str),
                     'start_mode': ParameterValue(start_mode, value_type=str),
                     'use_sim_time': use_sim_time}])

    # nav2_bringup 的 launch 只给 lifecycle_manager 传 use_sim_time/autostart/
    # node_names，不会传 params_file，所以 bond_timeout 只能在 launch 里注入。
    # 这块板子负载高时 bond 心跳会丢，4 秒收不到就把整个导航栈拆掉重来，
    # 表现为 rviz 之外所有节点"莫名其妙"变 inactive、目标一发就 ABORTED。
    # Nav2 官方说明：bond_timeout 设为 0 即停用该看门狗。
    nav2 = GroupAction([
        SetParameter(name='bond_timeout', value=0.0),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('nav2_bringup'),
                                      'launch', 'bringup_launch.py'])),
            launch_arguments={
                'slam': 'False',
                'map': map_yaml,
                'use_sim_time': use_sim_time,
                'params_file': PathJoinSubstitution(
                    [FindPackageShare('mecanum_nav'), 'config', 'nav2_params.yaml']),
                'use_composition': 'True',
                'autostart': 'False',
            }.items()),
    ])

    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', PathJoinSubstitution(
            [FindPackageShare('mecanum_nav'), 'config', 'mapping.rviz'])],
        additional_env={'QT_QPA_PLATFORM': 'xcb'},
        condition=IfCondition(rviz))

    joy = Node(
        package='mecanum_teleop', executable='joy_teleop',
        name='joy_teleop', output='screen',
        parameters=[{'buzzer_port': buzzer_port}])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_mode', default_value='mapping',
                              description='mapping 或 navigation'),
        DeclareLaunchArgument('map_name', default_value='map',
                              description='地图名，与保存地图一致'),
        DeclareLaunchArgument('map', default_value='/home/laijinxuan/maps/map.yaml',
                              description='导航使用的地图 yaml 路径'),
        DeclareLaunchArgument('yaw_offset_deg', default_value='80.0'),
        DeclareLaunchArgument('mirror', default_value='true'),
        DeclareLaunchArgument('x_offset_m', default_value='-0.05'),
        DeclareLaunchArgument('y_offset_m', default_value='0.0'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='板端 RViz 显示（远程显示可设 false）'),
        DeclareLaunchArgument('web', default_value='false',
                              description='是否启动网页面板（默认关，需要时设 true）'),
        DeclareLaunchArgument('buzzer_port', default_value='/dev/ttyUSB0'),
        base,
        imu,
        scan,
        mux,
        web,
        outline,
        restorer,
        manager,
        nav2,
        joy,
        rviz_node,
    ])
