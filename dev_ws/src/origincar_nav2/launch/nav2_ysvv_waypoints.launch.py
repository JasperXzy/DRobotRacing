#!/usr/bin/env python3
"""Start Nav2 with nav2(6.13).yaml and run the YSVV waypoint route."""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


INSTALLED_BT_TO_POSE = (
    '/root/D_Robot/dev_ws/install/origincar_nav2/share/origincar_nav2/'
    'behavior_trees/ackermann_navigate_w_recovery.xml')
INSTALLED_BT_THROUGH_POSES = (
    '/root/D_Robot/dev_ws/install/origincar_nav2/share/origincar_nav2/'
    'behavior_trees/ackermann_navigate_through_poses_w_recovery.xml')
INSTALLED_MAP_YAML = (
    '/root/D_Robot/dev_ws/install/origincar_nav2/share/origincar_nav2/'
    'map/my_map.yaml')


def _as_bool(value):
    return str(value).lower() in ('1', 'true', 'yes', 'on')


def _make_nav2_params_file(params_file, origincar_share):
    bt_to_pose_xml = os.path.join(
        origincar_share, 'behavior_trees', 'ackermann_navigate_w_recovery.xml')
    bt_through_poses_xml = os.path.join(
        origincar_share, 'behavior_trees',
        'ackermann_navigate_through_poses_w_recovery.xml')
    map_yaml_path = os.path.join(origincar_share, 'map', 'my_map.yaml')

    with open(params_file, 'r') as stream:
        params_content = stream.read()

    params_content = params_content.replace(INSTALLED_BT_TO_POSE, bt_to_pose_xml)
    params_content = params_content.replace(
        INSTALLED_BT_THROUGH_POSES, bt_through_poses_xml)
    params_content = params_content.replace(INSTALLED_MAP_YAML, map_yaml_path)

    with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as stream:
        stream.write(params_content)
        return stream.name


def _launch_setup(context):
    origincar_share = get_package_share_directory('origincar_nav2')

    params_file = LaunchConfiguration('params_file').perform(context)
    waypoints_file = LaunchConfiguration('waypoints_file').perform(context)
    map_file = LaunchConfiguration('map').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    navigator_delay = float(LaunchConfiguration('navigator_delay').perform(context))
    node_start_delay = float(LaunchConfiguration('node_start_delay').perform(context))
    publish_initial_pose = _as_bool(
        LaunchConfiguration('publish_initial_pose').perform(context))

    nav2_params_fixed = _make_nav2_params_file(params_file, origincar_share)

    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('state_estimation'),
                'launch', 'ekf.launch.py',
            ])
        )
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[nav2_params_fixed],
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params_fixed],
        remappings=[('/scan', '/scan')],
    )

    amcl_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_amcl',
        output='screen',
        parameters=[{
            'use_sim_time': _as_bool(use_sim_time),
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
        }],
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch', 'navigation_launch.py',
            ])
        ),
        launch_arguments={
            'map': map_file,
            'params_file': nav2_params_fixed,
            'use_sim_time': use_sim_time,
            'use_amcl': 'false',
            'autostart': 'true',
        }.items(),
    )

    navigator_node = Node(
        package='origincar_nav2',
        executable='nav2_ysvv_waypoints',
        name='nav2_ysvv_waypoints',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info'],
        parameters=[{
            'waypoints_file': waypoints_file,
            'frame_id': 'map',
            'start_delay': node_start_delay,
            'publish_initial_pose': publish_initial_pose,
            'initial_x': 0.0,
            'initial_y': 0.0,
            'initial_yaw': 0.0,
            'exit_on_result': False,
            'use_sim_time': _as_bool(use_sim_time),
        }],
    )

    return [
        TimerAction(period=4.0, actions=[ekf_launch]),
        TimerAction(period=6.0, actions=[map_server_node]),
        TimerAction(period=8.0, actions=[amcl_node, amcl_lifecycle_manager]),
        TimerAction(period=12.0, actions=[nav2_launch]),
        TimerAction(period=navigator_delay, actions=[navigator_node]),
    ]


def generate_launch_description():
    origincar_share = get_package_share_directory('origincar_nav2')
    ws_root = os.path.abspath(os.path.join(origincar_share, '..', '..', '..', '..'))

    default_params_file = os.path.join(ws_root, 'config', 'nav2', 'nav2(6.13).yaml')
    default_waypoints_file = os.path.join(
        ws_root, 'src', 'origincar_nav2', 'origincar_nav2', 'ysvv_waypoints.yaml')
    default_map_file = os.path.join(origincar_share, 'map', 'my_map.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Nav2 params YAML, defaults to dev_ws/config/nav2/nav2(6.13).yaml'),
        DeclareLaunchArgument(
            'waypoints_file',
            default_value=default_waypoints_file,
            description='YSVV waypoint YAML file'),
        DeclareLaunchArgument(
            'map',
            default_value=default_map_file,
            description='Map YAML file'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock'),
        DeclareLaunchArgument(
            'navigator_delay',
            default_value='24.0',
            description='Seconds before sending ysvv_waypoints.yaml to Nav2'),
        DeclareLaunchArgument(
            'node_start_delay',
            default_value='0.5',
            description='Extra delay inside the waypoint sender node'),
        DeclareLaunchArgument(
            'publish_initial_pose',
            default_value='true',
            description='Publish initialpose before sending the route'),
        OpaqueFunction(function=_launch_setup),
    ])
