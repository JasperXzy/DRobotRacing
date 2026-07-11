#!/usr/bin/env python3
"""Start Nav2 and follow the YSVV route with point-to-point lookahead goals."""

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

    params_content = params_content.replace(
        INSTALLED_BT_TO_POSE, bt_to_pose_xml)
    params_content = params_content.replace(
        INSTALLED_BT_THROUGH_POSES, bt_through_poses_xml)
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
    switch_distance = float(
        LaunchConfiguration('switch_distance').perform(context))
    lookahead_index_offset = int(
        LaunchConfiguration('lookahead_index_offset').perform(context))
    closest_search_back_window = int(
        LaunchConfiguration('closest_search_back_window').perform(context))
    closest_search_forward_window = int(
        LaunchConfiguration('closest_search_forward_window').perform(context))
    intermediate_goal_yaw_mode = LaunchConfiguration(
        'intermediate_goal_yaw_mode').perform(context)
    goal_update_min_interval = float(
        LaunchConfiguration('goal_update_min_interval').perform(context))
    goal_republish_distance = float(
        LaunchConfiguration('goal_republish_distance').perform(context))
    feedback_stall_timeout = float(
        LaunchConfiguration('feedback_stall_timeout').perform(context))
    feedback_stall_distance_epsilon = float(LaunchConfiguration(
        'feedback_stall_distance_epsilon').perform(context))
    obstacle_escape_distance = float(
        LaunchConfiguration('obstacle_escape_distance').perform(context))
    blocked_check_radius = float(
        LaunchConfiguration('blocked_check_radius').perform(context))
    max_skip_waypoints = int(
        LaunchConfiguration('max_skip_waypoints').perform(context))
    blocked_cost_threshold = int(
        LaunchConfiguration('blocked_cost_threshold').perform(context))
    obstacle_center_cost_threshold = int(LaunchConfiguration(
        'obstacle_center_cost_threshold').perform(context))
    obstacle_ahead_cost_threshold = int(LaunchConfiguration(
        'obstacle_ahead_cost_threshold').perform(context))
    path_check_timeout = float(
        LaunchConfiguration('path_check_timeout').perform(context))
    planner_id = LaunchConfiguration('planner_id').perform(context)
    costmap_topic = LaunchConfiguration('costmap_topic').perform(context)
    obstacle_avoidance_topic = LaunchConfiguration(
        'obstacle_avoidance_topic').perform(context)
    cmd_vel_kp1 = float(LaunchConfiguration('cmd_vel_kp1').perform(context))
    cmd_vel_kp2 = float(LaunchConfiguration('cmd_vel_kp2').perform(context))
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
            'closest_search_back_window': closest_search_back_window,
            'closest_search_forward_window': closest_search_forward_window,
            'intermediate_goal_yaw_mode': intermediate_goal_yaw_mode,
            'goal_update_min_interval': goal_update_min_interval,
            'goal_republish_distance': goal_republish_distance,
            'feedback_stall_timeout': feedback_stall_timeout,
            'feedback_stall_distance_epsilon': feedback_stall_distance_epsilon,
            'obstacle_escape_distance': obstacle_escape_distance,
            'blocked_check_radius': blocked_check_radius,
            'max_skip_waypoints': max_skip_waypoints,
            'blocked_cost_threshold': blocked_cost_threshold,
            'obstacle_center_cost_threshold': obstacle_center_cost_threshold,
            'obstacle_ahead_cost_threshold': obstacle_ahead_cost_threshold,
            'obstacle_avoidance_topic': obstacle_avoidance_topic,
            'path_check_timeout': path_check_timeout,
            'planner_id': planner_id,
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
            'switch_distance',
            default_value='0.30',
            description='Distance threshold for advancing a point goal'),
        DeclareLaunchArgument(
            'lookahead_index_offset',
            default_value='15',
            description='Route points ahead of the nearest point used as goal'),
        DeclareLaunchArgument(
            'closest_search_back_window',
            default_value='5',
            description='Route points searched behind current progress'),
        DeclareLaunchArgument(
            'closest_search_forward_window',
            default_value='30',
            description='Route points searched ahead of current progress'),
        DeclareLaunchArgument(
            'intermediate_goal_yaw_mode',
            default_value='path_tangent',
            description=(
                'Yaw for point goals: current, path_tangent, '
                'or recorded')),
        DeclareLaunchArgument(
            'goal_update_min_interval',
            default_value='0.50',
            description='Minimum seconds between point-goal updates'),
        DeclareLaunchArgument(
            'goal_republish_distance',
            default_value='0.05',
            description='Minimum goal movement before publishing a new goal'),
        DeclareLaunchArgument(
            'feedback_stall_timeout',
            default_value='4.0',
            description=(
                'Seconds without progress before advancing the point goal')),
        DeclareLaunchArgument(
            'feedback_stall_distance_epsilon',
            default_value='0.02',
            description=(
                'Minimum distance decrease treated as feedback progress')),
        DeclareLaunchArgument(
            'base_frame',
            default_value='base_link',
            description='Robot base frame used for route progress tracking'),
        DeclareLaunchArgument(
            'obstacle_escape_distance',
            default_value='0.60',
            description='Required route distance beyond an obstacle center'),
        DeclareLaunchArgument(
            'blocked_check_radius',
            default_value='0.60',
            description='Radius used to estimate an obstacle center'),
        DeclareLaunchArgument(
            'max_skip_waypoints',
            default_value='20',
            description='Maximum route points searched after a blocked goal'),
        DeclareLaunchArgument(
            'blocked_cost_threshold',
            default_value='95',
            description='Costmap value that marks a point as blocked'),
        DeclareLaunchArgument(
            'obstacle_center_cost_threshold',
            default_value='80',
            description='Cost threshold used to estimate obstacle center'),
        DeclareLaunchArgument(
            'obstacle_ahead_cost_threshold',
            default_value='80',
            description='Cost threshold used to detect an obstacle ahead'),
        DeclareLaunchArgument(
            'path_check_timeout',
            default_value='1.0',
            description='Timeout for ComputePathToPose precheck'),
        DeclareLaunchArgument(
            'planner_id',
            default_value='GridBased',
            description='Planner plugin used by ComputePathToPose'),
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
