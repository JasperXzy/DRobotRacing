#!/usr/bin/env python3
"""Launch the robot prerequisites for YSVV keyboard waypoint recording."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    origincar_share = get_package_share_directory('origincar_nav2')
    ws_root = os.path.abspath(os.path.join(origincar_share, '..', '..', '..', '..'))

    default_output_file = os.path.join(
        ws_root, 'src', 'origincar_nav2', 'origincar_nav2', 'ysvv_waypoints.yaml')
    default_chassis_config = os.path.join(
        ws_root, 'config', 'drivers', 'chassis_driver.yaml')
    default_lidar_config = os.path.join(
        ws_root, 'config', 'drivers', 'lidar_driver.yaml')
    default_params_file = os.path.join(
        ws_root, 'config', 'nav2', 'nav2(6.13).yaml')

    ros_log_dir = LaunchConfiguration('ros_log_dir')
    start_chassis_driver = LaunchConfiguration('start_chassis_driver')
    start_lidar_driver = LaunchConfiguration('start_lidar_driver')
    start_localization = LaunchConfiguration('start_localization')
    start_recorder = LaunchConfiguration('start_recorder')
    record_only = LaunchConfiguration('record_only')

    chassis_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('chassis_driver'),
                'launch',
                'chassis.launch.py',
            ])
        ),
        launch_arguments={
            'chassis_config_file': LaunchConfiguration('chassis_config_file'),
        }.items(),
        condition=IfCondition(start_chassis_driver),
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('lidar_driver'),
                'launch',
                'lidar.launch.py',
            ])
        ),
        launch_arguments={
            'lidar_config_file': LaunchConfiguration('lidar_config_file'),
        }.items(),
        condition=IfCondition(start_lidar_driver),
    )

    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('state_estimation'),
                'launch',
                'ekf.launch.py',
            ])
        ),
        condition=IfCondition(start_localization),
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        condition=IfCondition(start_localization),
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        remappings=[('/scan', '/scan')],
        condition=IfCondition(start_localization),
    )

    amcl_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_amcl',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
        }],
        condition=IfCondition(start_localization),
    )

    recorder_node = Node(
        package='origincar_nav2',
        executable='record_ysvv_waypoints',
        name='record_ysvv_waypoints',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(start_recorder),
        parameters=[{
            'output_file': LaunchConfiguration('output_file'),
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            'target_frame': LaunchConfiguration('target_frame'),
            'base_frame': LaunchConfiguration('base_frame'),
            'linear_speed': ParameterValue(
                LaunchConfiguration('linear_speed'), value_type=float),
            'reverse_speed': ParameterValue(
                LaunchConfiguration('reverse_speed'), value_type=float),
            'angular_speed': ParameterValue(
                LaunchConfiguration('angular_speed'), value_type=float),
            'slow_turn_linear_speed': ParameterValue(
                LaunchConfiguration('slow_turn_linear_speed'), value_type=float),
            'record_interval': ParameterValue(
                LaunchConfiguration('record_interval'), value_type=float),
            'min_distance': ParameterValue(
                LaunchConfiguration('min_distance'), value_type=float),
            'min_yaw_delta': ParameterValue(
                LaunchConfiguration('min_yaw_delta'), value_type=float),
            'publish_rate': ParameterValue(
                LaunchConfiguration('publish_rate'), value_type=float),
            'record_only': ParameterValue(record_only, value_type=bool),
            'use_sim_time': False,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'ros_log_dir',
            default_value='/tmp/ros_log',
            description='Writable ROS log directory'),
        DeclareLaunchArgument(
            'start_chassis_driver',
            default_value='true',
            description='Start chassis_driver before recording'),
        DeclareLaunchArgument(
            'start_lidar_driver',
            default_value='true',
            description='Start lidar_driver before recording'),
        DeclareLaunchArgument(
            'start_localization',
            default_value='true',
            description='Start EKF, map_server, and AMCL before recording'),
        DeclareLaunchArgument(
            'start_recorder',
            default_value='false',
            description='Start recorder from launch. Keyboard input usually needs ros2 run'),
        DeclareLaunchArgument(
            'record_only',
            default_value='false',
            description='Recorder only listens to TF and does not publish cmd_vel'),
        DeclareLaunchArgument(
            'chassis_config_file',
            default_value=default_chassis_config,
            description='Chassis driver config YAML'),
        DeclareLaunchArgument(
            'lidar_config_file',
            default_value=default_lidar_config,
            description='Lidar driver config YAML'),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Nav2 params YAML used by map_server and AMCL'),
        DeclareLaunchArgument(
            'output_file',
            default_value=default_output_file,
            description='Output ysvv_waypoints.yaml path'),
        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='/cmd_vel',
            description='Velocity command topic published by keyboard recorder'),
        DeclareLaunchArgument(
            'target_frame',
            default_value='map',
            description='TF target frame for recorded waypoints'),
        DeclareLaunchArgument(
            'base_frame',
            default_value='base_link',
            description='TF robot base frame for recorded waypoints'),
        DeclareLaunchArgument(
            'linear_speed',
            default_value='0.15',
            description='Forward keyboard speed'),
        DeclareLaunchArgument(
            'reverse_speed',
            default_value='0.12',
            description='Reverse keyboard speed'),
        DeclareLaunchArgument(
            'angular_speed',
            default_value='0.35',
            description='Keyboard angular speed'),
        DeclareLaunchArgument(
            'slow_turn_linear_speed',
            default_value='0.10',
            description='Forward speed used by q/e slow-turn keys'),
        DeclareLaunchArgument(
            'record_interval',
            default_value='0.25',
            description='Minimum seconds between pose samples'),
        DeclareLaunchArgument(
            'min_distance',
            default_value='0.08',
            description='Minimum distance change before recording another point'),
        DeclareLaunchArgument(
            'min_yaw_delta',
            default_value='0.17',
            description='Minimum yaw change before recording another point'),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='20.0',
            description='Keyboard command publish rate'),
        SetEnvironmentVariable('ROS_LOG_DIR', ros_log_dir),
        chassis_launch,
        lidar_launch,
        TimerAction(period=2.0, actions=[ekf_launch]),
        TimerAction(period=4.0, actions=[map_server_node]),
        TimerAction(period=6.0, actions=[amcl_node, amcl_lifecycle_manager]),
        recorder_node,
    ])
