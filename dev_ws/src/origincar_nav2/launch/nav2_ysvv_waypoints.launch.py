#!/usr/bin/env python3
"""Start Nav2 and run the YSVV route as rolling through-poses windows."""

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
INSTALLED_BT_YSVV_RPP = (
    '/root/D_Robot/dev_ws/install/origincar_nav2/share/origincar_nav2/'
    'behavior_trees/ackermann_ysvv_rpp_through_poses_window.xml')
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
    bt_ysvv_rpp_xml = os.path.join(
        origincar_share, 'behavior_trees',
        'ackermann_ysvv_rpp_through_poses_window.xml')
    map_yaml_path = os.path.join(origincar_share, 'map', 'my_map.yaml')

    with open(params_file, 'r') as stream:
        params_content = stream.read()

    params_content = params_content.replace(
        INSTALLED_BT_TO_POSE, bt_to_pose_xml)
    params_content = params_content.replace(
        INSTALLED_BT_THROUGH_POSES, bt_through_poses_xml)
    params_content = params_content.replace(
        INSTALLED_BT_YSVV_RPP, bt_ysvv_rpp_xml)
    params_content = params_content.replace(INSTALLED_MAP_YAML, map_yaml_path)

    with tempfile.NamedTemporaryFile(
            mode='w', suffix='.yaml', delete=False) as stream:
        stream.write(params_content)
        return stream.name


def _launch_setup(context):
    origincar_share = get_package_share_directory('origincar_nav2')

    params_file = LaunchConfiguration('params_file').perform(context)
    waypoints_file = LaunchConfiguration('waypoints_file').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    start_chassis_driver = _as_bool(
        LaunchConfiguration('start_chassis_driver').perform(context))
    start_lidar_driver = _as_bool(
        LaunchConfiguration('start_lidar_driver').perform(context))
    chassis_config_file = LaunchConfiguration(
        'chassis_config_file').perform(context)
    lidar_config_file = LaunchConfiguration(
        'lidar_config_file').perform(context)
    navigator_delay = float(
        LaunchConfiguration('navigator_delay').perform(context))
    node_start_delay = float(
        LaunchConfiguration('node_start_delay').perform(context))
    window_size = int(LaunchConfiguration('window_size').perform(context))
    window_start_offset = int(
        LaunchConfiguration('window_start_offset').perform(context))
    window_overlap_size = int(
        LaunchConfiguration('window_overlap_size').perform(context))
    intermediate_goal_yaw_mode = LaunchConfiguration(
        'intermediate_goal_yaw_mode').perform(context)
    goal_update_min_interval = float(
        LaunchConfiguration('goal_update_min_interval').perform(context))
    refresh_distance_threshold = float(
        LaunchConfiguration('refresh_distance_threshold').perform(context))
    window_end_reached_radius = float(
        LaunchConfiguration('window_end_reached_radius').perform(context))
    nav_cancel_timeout = float(
        LaunchConfiguration('nav_cancel_timeout').perform(context))
    nav_goal_anchor_count = int(
        LaunchConfiguration('nav_goal_anchor_count').perform(context))
    nav_goal_first_anchor_offset = int(
        LaunchConfiguration('nav_goal_first_anchor_offset').perform(context))
    nav_goal_min_first_distance = float(
        LaunchConfiguration('nav_goal_min_first_distance').perform(context))
    tangent_yaw_lookahead_distance = float(LaunchConfiguration(
        'tangent_yaw_lookahead_distance').perform(context))
    feedback_stall_timeout = float(
        LaunchConfiguration('feedback_stall_timeout').perform(context))
    feedback_stall_distance_epsilon = float(LaunchConfiguration(
        'feedback_stall_distance_epsilon').perform(context))
    enable_path_precheck = _as_bool(
        LaunchConfiguration('enable_path_precheck').perform(context))
    route_blocked_cost_threshold = int(
        LaunchConfiguration('route_blocked_cost_threshold').perform(context))
    unknown_is_blocked = _as_bool(
        LaunchConfiguration('unknown_is_blocked').perform(context))
    obstacle_entry_buffer = float(
        LaunchConfiguration('obstacle_entry_buffer').perform(context))
    obstacle_exit_buffer = float(
        LaunchConfiguration('obstacle_exit_buffer').perform(context))
    obstacle_merge_gap = float(
        LaunchConfiguration('obstacle_merge_gap').perform(context))
    obstacle_scan_waypoints = int(
        LaunchConfiguration('obstacle_scan_waypoints').perform(context))
    path_check_timeout = float(
        LaunchConfiguration('path_check_timeout').perform(context))
    path_check_max_retries = int(
        LaunchConfiguration('path_check_max_retries').perform(context))
    path_check_retry_delay = float(
        LaunchConfiguration('path_check_retry_delay').perform(context))
    navigation_max_retries = int(
        LaunchConfiguration('navigation_max_retries').perform(context))
    costmap_topic = LaunchConfiguration('costmap_topic').perform(context)
    obstacle_avoidance_topic = LaunchConfiguration(
        'obstacle_avoidance_topic').perform(context)
    cmd_vel_kp1 = float(LaunchConfiguration('cmd_vel_kp1').perform(context))
    cmd_vel_kp2 = float(LaunchConfiguration('cmd_vel_kp2').perform(context))
    publish_initial_pose = _as_bool(
        LaunchConfiguration('publish_initial_pose').perform(context))

    nav2_params_fixed = _make_nav2_params_file(params_file, origincar_share)
    ysvv_behavior_tree = os.path.join(
        origincar_share, 'behavior_trees',
        'ackermann_ysvv_rpp_through_poses_window.xml')

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
    cmd_vel_kp_scaler_node = Node(
        package='origincar_nav2',
        executable='cmd_vel_kp_scaler',
        name='cmd_vel_kp_scaler',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info'],
        parameters=[{
            'use_sim_time': _as_bool(use_sim_time),
            'input_topic': '/cmd_vel_rpp',
            'output_topic': '/cmd_vel_nav',
            'avoidance_topic': obstacle_avoidance_topic,
            'kp1': cmd_vel_kp1,
            'kp2': cmd_vel_kp2,
        }],
    )
    nav2_nodes = [
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_params_fixed],
            arguments=['--ros-args', '--log-level', 'info'],
            remappings=nav2_remappings + [('cmd_vel', 'cmd_vel_rpp')],
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
            'behavior_tree': ysvv_behavior_tree,
            'action_name': 'navigate_through_poses',
            'frame_id': 'map',
            'base_frame': LaunchConfiguration('base_frame').perform(context),
            'start_delay': node_start_delay,
            'publish_initial_pose': publish_initial_pose,
            'initial_x': 0.0,
            'initial_y': 0.0,
            'initial_yaw': 0.0,
            'window_size': window_size,
            'window_start_offset': window_start_offset,
            'window_overlap_size': window_overlap_size,
            'intermediate_goal_yaw_mode': intermediate_goal_yaw_mode,
            'goal_update_min_interval': goal_update_min_interval,
            'refresh_distance_threshold': refresh_distance_threshold,
            'window_end_reached_radius': window_end_reached_radius,
            'nav_cancel_timeout': nav_cancel_timeout,
            'nav_goal_anchor_count': nav_goal_anchor_count,
            'nav_goal_first_anchor_offset': nav_goal_first_anchor_offset,
            'nav_goal_min_first_distance': nav_goal_min_first_distance,
            'tangent_yaw_lookahead_distance': tangent_yaw_lookahead_distance,
            'feedback_stall_timeout': feedback_stall_timeout,
            'feedback_stall_distance_epsilon': feedback_stall_distance_epsilon,
            'enable_path_precheck': enable_path_precheck,
            'route_blocked_cost_threshold': route_blocked_cost_threshold,
            'unknown_is_blocked': unknown_is_blocked,
            'obstacle_entry_buffer': obstacle_entry_buffer,
            'obstacle_exit_buffer': obstacle_exit_buffer,
            'obstacle_merge_gap': obstacle_merge_gap,
            'obstacle_scan_waypoints': obstacle_scan_waypoints,
            'obstacle_avoidance_topic': obstacle_avoidance_topic,
            'path_check_timeout': path_check_timeout,
            'path_check_max_retries': path_check_max_retries,
            'path_check_retry_delay': path_check_retry_delay,
            'navigation_max_retries': navigation_max_retries,
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
        TimerAction(
            period=12.0, actions=[cmd_vel_kp_scaler_node] + nav2_nodes),
        TimerAction(period=navigator_delay, actions=[navigator_node]),
    ])

    return actions


def generate_launch_description():
    origincar_share = get_package_share_directory('origincar_nav2')
    ws_root = os.path.abspath(
        os.path.join(origincar_share, '..', '..', '..', '..'))

    default_params_file = os.path.join(
        ws_root, 'config', 'nav2', 'nav2(rpp_follower).yaml')
    default_chassis_config = os.path.join(
        ws_root, 'config', 'drivers', 'chassis_driver.yaml')
    default_lidar_config = os.path.join(
        ws_root, 'config', 'drivers', 'lidar_driver.yaml')
    default_waypoints_file = os.path.join(
        ws_root, 'src', 'origincar_nav2', 'origincar_nav2',
        'ysvv_waypoints.yaml')
    default_map_file = os.path.join(origincar_share, 'map', 'my_map.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description=(
                'Nav2 params YAML, defaults to '
                'dev_ws/config/nav2/nav2(rpp_follower).yaml')),
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
            'window_size',
            default_value='10',
            description=(
                'Ordered route poses per Nav2 batch, hard-capped at 10')),
        DeclareLaunchArgument(
            'window_start_offset',
            default_value='5',
            description='First batch pose offset from current route progress'),
        DeclareLaunchArgument(
            'window_overlap_size',
            default_value='5',
            description=(
                'Ordered poses retained between consecutive windows')),
        DeclareLaunchArgument(
            'intermediate_goal_yaw_mode',
            default_value='path_tangent',
            description=(
                'Yaw for intermediate batch poses: current, path_tangent, '
                'or recorded')),
        DeclareLaunchArgument(
            'goal_update_min_interval',
            default_value='0.2',
            description='Minimum seconds between rolling batch updates'),
        DeclareLaunchArgument(
            'refresh_distance_threshold',
            default_value='0.90',
            description=(
                'Legacy feedback distance threshold; window updates are gated '
                'by window_end_reached_radius')),
        DeclareLaunchArgument(
            'window_end_reached_radius',
            default_value='0.30',
            description=(
                'Publish the next global route window only after base_link is '
                'within this distance of the active window end pose')),
        DeclareLaunchArgument(
            'nav_cancel_timeout',
            default_value='0.2',
            description=(
                'Legacy cancel fallback timeout; direct preemption is used '
                'for normal window updates')),
        DeclareLaunchArgument(
            'nav_goal_anchor_count',
            default_value='2',
            description=(
                'Maximum Nav2 through poses sampled from each route window')),
        DeclareLaunchArgument(
            'nav_goal_first_anchor_offset',
            default_value='5',
            description=(
                'One-based route-window position used as the first Nav2 '
                'anchor before even spacing')),
        DeclareLaunchArgument(
            'nav_goal_min_first_distance',
            default_value='0.55',
            description=(
                'Skip Nav2 anchors nearer than this distance from base_link')),
        DeclareLaunchArgument(
            'tangent_yaw_lookahead_distance',
            default_value='0.45',
            description=(
                'Forward route distance used to compute path_tangent yaw')),
        DeclareLaunchArgument(
            'feedback_stall_timeout',
            default_value='4.0',
            description=(
                'Seconds without progress before refreshing the batch')),
        DeclareLaunchArgument(
            'feedback_stall_distance_epsilon',
            default_value='0.02',
            description=(
                'Minimum distance decrease treated as feedback progress')),
        DeclareLaunchArgument(
            'enable_path_precheck',
            default_value='true',
            description=(
                'Precheck each batch with ComputePathThroughPoses; disable '
                'to remove duplicate planning latency')),
        DeclareLaunchArgument(
            'base_frame',
            default_value='base_link',
            description='Robot base frame used for route progress tracking'),
        DeclareLaunchArgument(
            'route_blocked_cost_threshold',
            default_value='80',
            description='Global costmap value that excludes a route pose'),
        DeclareLaunchArgument(
            'unknown_is_blocked',
            default_value='true',
            description=(
                'Treat unknown global costmap cells as blocked route poses')),
        DeclareLaunchArgument(
            'obstacle_entry_buffer',
            default_value='0.35',
            description=(
                'Route distance removed before a blocked costmap interval')),
        DeclareLaunchArgument(
            'obstacle_exit_buffer',
            default_value='0.35',
            description=(
                'Route distance removed after a blocked costmap interval')),
        DeclareLaunchArgument(
            'obstacle_merge_gap',
            default_value='0.25',
            description=(
                'Merge obstacle intervals separated by a shorter route gap')),
        DeclareLaunchArgument(
            'obstacle_scan_waypoints',
            default_value='60',
            description=(
                'Maximum forward route poses inspected when building '
                'one batch')),
        DeclareLaunchArgument(
            'path_check_timeout',
            default_value='1.0',
            description=(
                'Timeout in seconds for compute_path_through_poses precheck')),
        DeclareLaunchArgument(
            'path_check_max_retries',
            default_value='2',
            description=(
                'Batch planning retries before stopping without further '
                'deletion')),
        DeclareLaunchArgument(
            'path_check_retry_delay',
            default_value='0.5',
            description='Delay between batch planning retries'),
        DeclareLaunchArgument(
            'navigation_max_retries',
            default_value='2',
            description='NavigateThroughPoses retries before stopping'),
        DeclareLaunchArgument(
            'costmap_topic',
            default_value='/global_costmap/costmap',
            description=(
                'Global costmap topic used for blocked waypoint checks')),
        DeclareLaunchArgument(
            'obstacle_avoidance_topic',
            default_value='/ysvv_obstacle_avoidance_active',
            description=(
                'Bool topic that switches cmd_vel scaling to obstacle mode')),
        DeclareLaunchArgument(
            'cmd_vel_kp1',
            default_value='1.10',
            description=(
                'Multiplier applied to RPP cmd_vel during normal navigation')),
        DeclareLaunchArgument(
            'cmd_vel_kp2',
            default_value='0.80',
            description=(
                'Multiplier applied to RPP cmd_vel during obstacle '
                'avoidance')),
        DeclareLaunchArgument(
            'publish_initial_pose',
            default_value='true',
            description='Publish initialpose before sending the route'),
        SetEnvironmentVariable(
            'ROS_LOG_DIR', LaunchConfiguration('ros_log_dir')),
        OpaqueFunction(function=_launch_setup),
    ])
