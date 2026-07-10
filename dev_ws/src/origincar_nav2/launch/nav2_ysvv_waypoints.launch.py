#!/usr/bin/env python3
"""Start Nav2 with nav2(6.13).yaml and run the YSVV route point-to-point."""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import SetEnvironmentVariable
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
    start_chassis_driver = _as_bool(
        LaunchConfiguration('start_chassis_driver').perform(context))
    start_lidar_driver = _as_bool(
        LaunchConfiguration('start_lidar_driver').perform(context))
    chassis_config_file = LaunchConfiguration('chassis_config_file').perform(context)
    lidar_config_file = LaunchConfiguration('lidar_config_file').perform(context)
    navigator_delay = float(LaunchConfiguration('navigator_delay').perform(context))
    node_start_delay = float(LaunchConfiguration('node_start_delay').perform(context))
    switch_distance = float(LaunchConfiguration('switch_distance').perform(context))
    lookahead_index_offset = int(
        LaunchConfiguration('lookahead_index_offset').perform(context))
    intermediate_goal_yaw_mode = LaunchConfiguration(
        'intermediate_goal_yaw_mode').perform(context)
    goal_update_min_interval = float(
        LaunchConfiguration('goal_update_min_interval').perform(context))
    goal_republish_distance = float(
        LaunchConfiguration('goal_republish_distance').perform(context))
    obstacle_escape_distance = float(
        LaunchConfiguration('obstacle_escape_distance').perform(context))
    blocked_check_radius = float(
        LaunchConfiguration('blocked_check_radius').perform(context))
    max_skip_waypoints = int(LaunchConfiguration('max_skip_waypoints').perform(context))
    blocked_cost_threshold = int(
        LaunchConfiguration('blocked_cost_threshold').perform(context))
    obstacle_center_cost_threshold = int(
        LaunchConfiguration('obstacle_center_cost_threshold').perform(context))
    obstacle_ahead_cost_threshold = int(
        LaunchConfiguration('obstacle_ahead_cost_threshold').perform(context))
    path_check_timeout = float(LaunchConfiguration('path_check_timeout').perform(context))
    costmap_topic = LaunchConfiguration('costmap_topic').perform(context)
    publish_initial_pose = _as_bool(
        LaunchConfiguration('publish_initial_pose').perform(context))

    nav2_params_fixed = _make_nav2_params_file(params_file, origincar_share)

    chassis_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('chassis_driver'),
                'launch', 'chassis.launch.py',
            ])
        ),
        launch_arguments={
            'chassis_config_file': chassis_config_file,
        }.items(),
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('lidar_driver'),
                'launch', 'lidar.launch.py',
            ])
        ),
        launch_arguments={
            'lidar_config_file': lidar_config_file,
        }.items(),
    )

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

    nav2_remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    nav2_lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'velocity_smoother',
    ]
    nav2_nodes = [
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_params_fixed],
            arguments=['--ros-args', '--log-level', 'info'],
            remappings=nav2_remappings + [('cmd_vel', 'cmd_vel_nav')],
        ),
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            parameters=[nav2_params_fixed],
            arguments=['--ros-args', '--log-level', 'info'],
            remappings=nav2_remappings,
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params_fixed],
            arguments=['--ros-args', '--log-level', 'info'],
            remappings=nav2_remappings,
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_params_fixed],
            arguments=['--ros-args', '--log-level', 'info'],
            remappings=nav2_remappings,
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_params_fixed],
            arguments=['--ros-args', '--log-level', 'info'],
            remappings=nav2_remappings,
        ),
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[nav2_params_fixed],
            arguments=['--ros-args', '--log-level', 'info'],
            remappings=nav2_remappings + [
                ('cmd_vel', 'cmd_vel_nav'),
                ('cmd_vel_smoothed', 'cmd_vel'),
            ],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            arguments=['--ros-args', '--log-level', 'info'],
            parameters=[{
                'use_sim_time': _as_bool(use_sim_time),
                'autostart': True,
                'node_names': nav2_lifecycle_nodes,
            }],
        ),
    ]

    navigator_node = Node(
        package='origincar_nav2',
        executable='nav2_ysvv_waypoints',
        name='nav2_ysvv_waypoints',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info'],
        parameters=[{
            'waypoints_file': waypoints_file,
            'action_name': 'navigate_to_pose',
            'frame_id': 'map',
            'base_frame': LaunchConfiguration('base_frame').perform(context),
            'start_delay': node_start_delay,
            'publish_initial_pose': publish_initial_pose,
            'initial_x': 0.0,
            'initial_y': 0.0,
            'initial_yaw': 0.0,
            'switch_distance': switch_distance,
            'lookahead_index_offset': lookahead_index_offset,
            'intermediate_goal_yaw_mode': intermediate_goal_yaw_mode,
            'goal_update_min_interval': goal_update_min_interval,
            'goal_republish_distance': goal_republish_distance,
            'obstacle_escape_distance': obstacle_escape_distance,
            'blocked_check_radius': blocked_check_radius,
            'max_skip_waypoints': max_skip_waypoints,
            'blocked_cost_threshold': blocked_cost_threshold,
            'obstacle_center_cost_threshold': obstacle_center_cost_threshold,
            'obstacle_ahead_cost_threshold': obstacle_ahead_cost_threshold,
            'path_check_timeout': path_check_timeout,
            'costmap_topic': costmap_topic,
            'exit_on_result': False,
            'use_sim_time': _as_bool(use_sim_time),
        }],
    )

    actions = []
    if start_chassis_driver:
        actions.append(chassis_launch)
    if start_lidar_driver:
        actions.append(lidar_launch)

    actions.extend([
        TimerAction(period=4.0, actions=[ekf_launch]),
        TimerAction(period=6.0, actions=[map_server_node]),
        TimerAction(period=8.0, actions=[amcl_node, amcl_lifecycle_manager]),
        TimerAction(period=12.0, actions=nav2_nodes),
        TimerAction(period=navigator_delay, actions=[navigator_node]),
    ])

    return actions


def generate_launch_description():
    origincar_share = get_package_share_directory('origincar_nav2')
    ws_root = os.path.abspath(os.path.join(origincar_share, '..', '..', '..', '..'))

    default_params_file = os.path.join(ws_root, 'config', 'nav2', 'nav2(6.13).yaml')
    default_chassis_config = os.path.join(
        ws_root, 'config', 'drivers', 'chassis_driver.yaml')
    default_lidar_config = os.path.join(
        ws_root, 'config', 'drivers', 'lidar_driver.yaml')
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
            'ros_log_dir',
            default_value='/tmp/ros_log',
            description='Writable ROS log directory'),
        DeclareLaunchArgument(
            'start_chassis_driver',
            default_value='true',
            description='Start chassis_driver for Nav2 route following'),
        DeclareLaunchArgument(
            'start_lidar_driver',
            default_value='true',
            description='Start lidar_driver for AMCL and costmaps'),
        DeclareLaunchArgument(
            'chassis_config_file',
            default_value=default_chassis_config,
            description='Chassis driver config YAML'),
        DeclareLaunchArgument(
            'lidar_config_file',
            default_value=default_lidar_config,
            description='Lidar driver config YAML'),
        DeclareLaunchArgument(
            'navigator_delay',
            default_value='24.0',
            description='Seconds before sending ysvv_waypoints.yaml to Nav2'),
        DeclareLaunchArgument(
            'node_start_delay',
            default_value='0.5',
            description='Extra delay inside the waypoint sender node'),
        DeclareLaunchArgument(
            'switch_distance',
            default_value='0.30',
            description='Distance in meters for switching early to the next waypoint'),
        DeclareLaunchArgument(
            'lookahead_index_offset',
            default_value='7',
            description='Waypoint index offset from the nearest route point for the rolling target'),
        DeclareLaunchArgument(
            'intermediate_goal_yaw_mode',
            default_value='current',
            description='Yaw for intermediate lookahead goals: current, path_tangent, or recorded'),
        DeclareLaunchArgument(
            'goal_update_min_interval',
            default_value='0.50',
            description='Minimum seconds between rolling lookahead goal updates'),
        DeclareLaunchArgument(
            'goal_republish_distance',
            default_value='0.12',
            description='Minimum target movement before republishing a lookahead goal'),
        DeclareLaunchArgument(
            'base_frame',
            default_value='base_link',
            description='Robot base frame used for lookahead tracking'),
        DeclareLaunchArgument(
            'obstacle_escape_distance',
            default_value='0.50',
            description='Minimum distance from obstacle center for the next safe waypoint'),
        DeclareLaunchArgument(
            'blocked_check_radius',
            default_value='0.60',
            description='Radius in meters for obstacle-center estimation around a blocked waypoint'),
        DeclareLaunchArgument(
            'max_skip_waypoints',
            default_value='15',
            description='Maximum number of later waypoints to inspect after a blocked waypoint'),
        DeclareLaunchArgument(
            'blocked_cost_threshold',
            default_value='95',
            description='Costmap value at or above this threshold is treated as a blocked waypoint'),
        DeclareLaunchArgument(
            'obstacle_center_cost_threshold',
            default_value='80',
            description='Costmap value used when estimating the nearby obstacle center'),
        DeclareLaunchArgument(
            'obstacle_ahead_cost_threshold',
            default_value='80',
            description='Costmap value that triggers early obstacle-ahead route switching'),
        DeclareLaunchArgument(
            'path_check_timeout',
            default_value='1.0',
            description='Timeout in seconds for compute_path_to_pose precheck'),
        DeclareLaunchArgument(
            'costmap_topic',
            default_value='/global_costmap/costmap',
            description='Global costmap topic used for blocked waypoint checks'),
        DeclareLaunchArgument(
            'publish_initial_pose',
            default_value='true',
            description='Publish initialpose before sending the route'),
        SetEnvironmentVariable('ROS_LOG_DIR', LaunchConfiguration('ros_log_dir')),
        OpaqueFunction(function=_launch_setup),
    ])
