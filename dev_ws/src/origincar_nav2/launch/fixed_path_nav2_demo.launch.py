"""
固定路径 Nav2 演示启动文件
参考 racing.launch.py 时序，分阶段启动 EKF → 地图 → AMCL → Nav2 → 逐点路径点发送

启动时序（与 racing.launch.py 一致）:
  t=4s   → 第1阶段: EKF 融合定位（IMU+里程计）
  t=6s   → 第2阶段: 地图服务器
  t=8s   → 第3阶段: AMCL 粒子滤波定位
  t=12s  → 第4阶段: Nav2 导航堆栈
  t=35s  → 第5阶段: 固定路径点逐点发送节点（小车开始运动）

用法:
    ros2 launch origincar_nav2 fixed_path_nav2_demo.launch.py
    ros2 launch origincar_nav2 fixed_path_nav2_demo.launch.py waypoints_file:=/path/to/waypoints.yaml
"""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ============================================
    # 路径
    # ============================================
    origincar_share = get_package_share_directory('origincar_nav2')
    ws_root = os.path.abspath(os.path.join(origincar_share, '..', '..', '..', '..'))

    # ============================================
    # 声明启动参数
    # ============================================
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(ws_root, 'config', 'nav2', 'nav2(6.13).yaml'),
        description='Nav2 参数 YAML 文件路径')

    waypoints_file_arg = DeclareLaunchArgument(
        'waypoints_file',
        default_value=os.path.join(ws_root, 'config', 'nav2', 'demo_waypoints_ccw.yaml'),
        description='固定路径点 YAML 文件路径')

    switch_distance_arg = DeclareLaunchArgument(
        'switch_distance',
        default_value='0.30',
        description='提前切到下一个路径点的距离阈值，单位米')

    lookahead_index_offset_arg = DeclareLaunchArgument(
        'lookahead_index_offset',
        default_value='10',
        description='从离小车最近的路径点向后偏移多少个点作为导航目标')

    goal_update_min_interval_arg = DeclareLaunchArgument(
        'goal_update_min_interval',
        default_value='0.50',
        description='前视目标最小重发间隔，单位秒')

    goal_republish_distance_arg = DeclareLaunchArgument(
        'goal_republish_distance',
        default_value='0.12',
        description='前视目标移动超过该距离才重新发布，单位米')

    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='base_link',
        description='用于前视点跟踪的小车底盘坐标系')

    obstacle_escape_distance_arg = DeclareLaunchArgument(
        'obstacle_escape_distance',
        default_value='0.50',
        description='被障碍物挡住时，后续点必须离障碍物中心至少多少米')

    blocked_check_radius_arg = DeclareLaunchArgument(
        'blocked_check_radius',
        default_value='0.60',
        description='计算障碍物中心时扫描当前点周围的半径，单位米')

    max_skip_waypoints_arg = DeclareLaunchArgument(
        'max_skip_waypoints',
        default_value='15',
        description='被障碍物挡住时最多向后查找多少个路径点')

    blocked_cost_threshold_arg = DeclareLaunchArgument(
        'blocked_cost_threshold',
        default_value='95',
        description='costmap 代价大于等于该值时认为路径点被真正阻塞')

    obstacle_center_cost_threshold_arg = DeclareLaunchArgument(
        'obstacle_center_cost_threshold',
        default_value='80',
        description='计算障碍物中心时使用的 costmap 代价阈值')

    obstacle_ahead_cost_threshold_arg = DeclareLaunchArgument(
        'obstacle_ahead_cost_threshold',
        default_value='80',
        description='前方路径段代价大于等于该值时提前触发避障跳点')

    path_check_timeout_arg = DeclareLaunchArgument(
        'path_check_timeout',
        default_value='1.0',
        description='compute_path_to_pose 规划预检查超时时间，单位秒')

    costmap_topic_arg = DeclareLaunchArgument(
        'costmap_topic',
        default_value='/global_costmap/costmap',
        description='用于检查路径点是否被障碍物占用的 global costmap 话题')

    # ============================================
    # 动态修正 Nav2 参数文件中硬编码的路径
    # ============================================
    nav2_params_file = os.path.join(ws_root, 'config', 'nav2', 'nav2(6.13).yaml')
    bt_to_pose_xml = os.path.join(
        origincar_share, 'behavior_trees', 'ackermann_navigate_w_recovery.xml')
    bt_through_poses_xml = os.path.join(
        origincar_share, 'behavior_trees',
        'ackermann_navigate_through_poses_w_recovery.xml')
    map_yaml_path = os.path.join(origincar_share, 'map', 'my_map.yaml')

    with open(nav2_params_file, 'r') as f:
        params_content = f.read()
    params_content = params_content.replace(
        '/root/D_Robot/dev_ws/install/origincar_nav2/share/origincar_nav2/behavior_trees/ackermann_navigate_w_recovery.xml',
        bt_to_pose_xml)
    params_content = params_content.replace(
        '/root/D_Robot/dev_ws/install/origincar_nav2/share/origincar_nav2/behavior_trees/ackermann_navigate_through_poses_w_recovery.xml',
        bt_through_poses_xml)
    params_content = params_content.replace(
        '/root/D_Robot/dev_ws/install/origincar_nav2/share/origincar_nav2/map/my_map.yaml',
        map_yaml_path)

    with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
        f.write(params_content)
        nav2_params_fixed = f.name

    # ============================================
    # 第1阶段 (t=4s): EKF 融合定位
    # ============================================
    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('state_estimation'),
                'launch', 'ekf.launch.py'
            ])
        )
    )

    # ============================================
    # 第2阶段 (t=6s): 地图服务器
    # ============================================
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[nav2_params_fixed],
    )

    # ============================================
    # 第3阶段 (t=8s): AMCL 粒子滤波定位
    # ============================================
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params_fixed],
        remappings=[('/scan', '/scan')]
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
        }]
    )

    # ============================================
    # 第4阶段 (t=12s): Nav2 导航堆栈
    # ============================================
    nav2_map_file = os.path.join(origincar_share, 'map', 'my_map.yaml')

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch', 'navigation_launch.py'
            ])
        ),
        launch_arguments={
            'map': nav2_map_file,
            'params_file': nav2_params_fixed,
            'use_sim_time': 'false',
            'use_amcl': 'false',
            'autostart': 'true',
        }.items()
    )

    # ============================================
    # 第5阶段 (t=35s): 固定路径点逐点发送节点
    # ============================================
    fixed_path_node = Node(
        package='origincar_nav2',
        executable='fixed_waypoint_demo',
        name='fixed_waypoint_demo',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info'],
        parameters=[{
            'waypoints_file': LaunchConfiguration('waypoints_file'),
            'action_name': 'navigate_to_pose',
            'frame_id': 'map',
            'base_frame': LaunchConfiguration('base_frame'),
            'start_delay': 0.5,
            'publish_initial_pose': True,
            'initial_x': 0.0,
            'initial_y': 0.0,
            'initial_yaw': 0.0,
            'switch_distance': ParameterValue(
                LaunchConfiguration('switch_distance'), value_type=float),
            'lookahead_index_offset': ParameterValue(
                LaunchConfiguration('lookahead_index_offset'), value_type=int),
            'goal_update_min_interval': ParameterValue(
                LaunchConfiguration('goal_update_min_interval'), value_type=float),
            'goal_republish_distance': ParameterValue(
                LaunchConfiguration('goal_republish_distance'), value_type=float),
            'obstacle_escape_distance': ParameterValue(
                LaunchConfiguration('obstacle_escape_distance'), value_type=float),
            'blocked_check_radius': ParameterValue(
                LaunchConfiguration('blocked_check_radius'), value_type=float),
            'max_skip_waypoints': ParameterValue(
                LaunchConfiguration('max_skip_waypoints'), value_type=int),
            'blocked_cost_threshold': ParameterValue(
                LaunchConfiguration('blocked_cost_threshold'), value_type=int),
            'obstacle_center_cost_threshold': ParameterValue(
                LaunchConfiguration('obstacle_center_cost_threshold'), value_type=int),
            'obstacle_ahead_cost_threshold': ParameterValue(
                LaunchConfiguration('obstacle_ahead_cost_threshold'), value_type=int),
            'path_check_timeout': ParameterValue(
                LaunchConfiguration('path_check_timeout'), value_type=float),
            'costmap_topic': LaunchConfiguration('costmap_topic'),
            'exit_on_result': False,
            'use_sim_time': False,
        }])

    # ============================================
    # 组装: 分阶段启动（与 racing.launch.py 时序一致）
    # ============================================
    ld = LaunchDescription()

    ld.add_action(params_file_arg)
    ld.add_action(waypoints_file_arg)
    ld.add_action(switch_distance_arg)
    ld.add_action(lookahead_index_offset_arg)
    ld.add_action(goal_update_min_interval_arg)
    ld.add_action(goal_republish_distance_arg)
    ld.add_action(base_frame_arg)
    ld.add_action(obstacle_escape_distance_arg)
    ld.add_action(blocked_check_radius_arg)
    ld.add_action(max_skip_waypoints_arg)
    ld.add_action(blocked_cost_threshold_arg)
    ld.add_action(obstacle_center_cost_threshold_arg)
    ld.add_action(obstacle_ahead_cost_threshold_arg)
    ld.add_action(path_check_timeout_arg)
    ld.add_action(costmap_topic_arg)

    # ━ t=4s ━ EKF 融合定位
    ld.add_action(TimerAction(period=4.0, actions=[ekf_launch]))

    # ━ t=6s ━ 地图服务器
    ld.add_action(TimerAction(period=6.0, actions=[map_server_node]))

    # ━ t=8s ━ AMCL 粒子滤波定位
    ld.add_action(TimerAction(period=8.0, actions=[amcl_node, amcl_lifecycle_manager]))

    # ━ t=12s ━ Nav2 导航堆栈
    ld.add_action(TimerAction(period=12.0, actions=[nav2_launch]))

    # ━ t=35s ━ 固定路径点逐点发送（小车开始运动）
    ld.add_action(TimerAction(period=35.0, actions=[fixed_path_node]))

    return ld
