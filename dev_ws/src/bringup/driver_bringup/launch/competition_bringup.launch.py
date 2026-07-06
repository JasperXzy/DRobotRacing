"""
比赛总启动文件
一键启动所有硬件驱动 + 感知节点 + 定位导航 + 巡航节点
按时间分阶段启动，避免同时启动导致资源竞争

启动时序（严格遵守依赖关系，已收紧延时）:
  t=0s   → 第1阶段: 硬件驱动 + 静态位姿
            底盘驱动 / 激光雷达 / 摄像头 / 静态TF
  t=3s   → 切换日志级别 (WARN，抑制后续节点刷屏)
  t=4s   → 第2阶段: EKF 融合定位（IMU+里程计）
  t=6s   → 第3阶段: 地图服务器 + 感知节点（QR+VLM）
            （摄像头6s足够稳定出图）
  t=8s   → 第4阶段: AMCL 粒子滤波定位
            （等地图加载完毕，发布 map→odom TF）
  t=12s  → 第5阶段: Nav2 导航堆栈
            （AMCL约2s完成首次定位）
  t=24s  → 第6阶段: 巡航节点
            waypoint_cruiser
            （等Nav2 lifecycle完全激活）

总启动时间: ~24秒（优化前 ~40秒）

用法:
    ros2 launch driver_bringup competition_bringup.launch.py

启动后等待约25秒，然后在另一个终端发令:
    ros2 run origincar_nav2 trigger_cruise
"""

import os
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument,
                            IncludeLaunchDescription,
                            SetEnvironmentVariable,
                            TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ============================================
    # 路径
    # ============================================
    ws_root = os.path.abspath(os.path.join(
        get_package_share_directory('driver_bringup'), '..', '..', '..', '..'))

    # ============================================
    # 第1阶段 (t=0s): 硬件驱动 + 静态位姿
    # ============================================

    # 底盘驱动
    chassis_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('chassis_driver'),
                'launch', 'chassis.launch.py'
            ])
        )
    )

    # 激光雷达驱动
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('lidar_driver'),
                'launch', 'lidar.launch.py'
            ])
        )
    )

    # 摄像头驱动 (不包含深度相机)
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('camera_driver'),
                'launch', 'camera.launch.py'
            ])
        )
    )

    # 静态位姿发布 (origincar_description)
    origincar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('origincar_description'),
                'launch', 'display.launch.py'
            ])
        )
    )

    # ============================================
    # 第2阶段 (t=4s): EKF 融合定位
    # 等激光雷达稳定出scan + 底盘里程计有初值
    # ============================================

    # EKF 节点 — 融合 IMU + 里程计
    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('state_estimation'),
                'launch', 'ekf.launch.py'
            ])
        )
    )

    # ============================================
    # 第3阶段 (t=6s): 地图服务器 + 感知节点
    # ============================================

    nav2_params_file = os.path.join(ws_root, 'config', 'nav2', 'nav2xzt.yaml')

    # ★ 动态修正 YAML 中硬编码的路径（BT XML + 地图），避免依赖 /root/D_Robot 固定路径
    origincar_share = get_package_share_directory('origincar_nav2')
    bt_xml_path = os.path.join(origincar_share, 'behavior_trees', 'ackermann_navigate_w_recovery.xml')
    map_yaml_path = os.path.join(origincar_share, 'map', 'my_map.yaml')

    with open(nav2_params_file, 'r') as f:
        params_content = f.read()
    params_content = params_content.replace(
        '/root/D_Robot/dev_ws/install/origincar_nav2/share/origincar_nav2/behavior_trees/ackermann_navigate_w_recovery.xml',
        bt_xml_path)
    params_content = params_content.replace(
        '/root/D_Robot/dev_ws/install/origincar_nav2/share/origincar_nav2/map/my_map.yaml',
        map_yaml_path)

    with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
        f.write(params_content)
        nav2_params_fixed = f.name

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[nav2_params_fixed],
    )

    # 二维码检测节点
    qr_node = Node(
        package='qr_detector_cpp',
        executable='qr_detector_node',
        name='qr_detector_node',
        output='screen',
        parameters=[{
            'image_topic': '/camera/image',
            'stop_on_detect': True,
        }]
    )

    # VLM图生文节点 — 挂起状态，等 /sign4return=-1 触发
    vlm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('vlm_tts_demo'),
                'launch', 'vlm_tts_demo.launch.py'
            ])
        ),
        launch_arguments={
            'launch_camera': 'false',
            'trigger_value': '-1',
            'camera_topic': '/camera/image',
        }.items()
    )

    # ============================================
    # 第4阶段 (t=8s): AMCL 定位
    # 等地图加载完毕后初始化粒子滤波
    # ============================================

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params_fixed],
        remappings=[('/scan', '/scan')]
    )

    # AMCL + map_server 统一由 lifecycle_manager 激活
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
    # 第5阶段 (t=12s): Nav2 导航堆栈
    # AMCL 约2s完成首次定位，map→odom TF 就绪
    # ============================================

    nav2_map_file = os.path.join(
        get_package_share_directory('origincar_nav2'), 'map', 'my_map.yaml')

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
    # 第6阶段 (t=24s): 巡航节点
    # 等 Nav2 lifecycle 完全激活
    # ============================================

    cruiser_node = Node(
        package='origincar_nav2',
        executable='waypoint_cruiser',
        name='waypoint_cruiser',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info'],
        parameters=[{
            'cruise_mode': 'once',
            'start_topic': '/cruise_start',
            'prescan_file': os.path.join(
                get_package_share_directory('origincar_nav2'),
                'waypoints_prescan.yaml'),
            'cw_file': os.path.join(
                get_package_share_directory('origincar_nav2'),
                'waypoints_cw.yaml'),
            'ccw_file': os.path.join(
                get_package_share_directory('origincar_nav2'),
                'waypoints_ccw.yaml'),
            'qr_topic': '/sign_switch',
            'qr_sign_cw': 3,
            'qr_sign_ccw': 4,
            'qr_default': 'cw',
            'vlm_trigger_waypoint': 5,
            'vlm_trigger_value': -1,
            'vlm_trigger_topic': '/sign4return',
        }]
    )

    # ============================================
    # 组装: 分阶段启动（严格按依赖顺序 + 收紧延时）
    # ============================================
    ld = LaunchDescription()

    # ━ t=0s ━ 硬件驱动 + 静态TF
    ld.add_action(chassis_launch)
    ld.add_action(lidar_launch)
    ld.add_action(camera_launch)
    ld.add_action(origincar_launch)

    # ━ t=3s ━ 切换日志级别为 WARN（抑制后续节点 INFO 刷屏）
    ld.add_action(TimerAction(period=3.0, actions=[
        SetEnvironmentVariable('RCUTILS_LOGGING_SEVERITY_THRESHOLD', 'WARN')
    ]))

    # ━ t=4s ━ EKF 融合定位
    ld.add_action(TimerAction(period=4.0, actions=[ekf_launch]))

    # ━ t=6s ━ 地图服务器 + 感知节点（QR + VLM）
    ld.add_action(TimerAction(period=6.0, actions=[map_server_node]))
    ld.add_action(TimerAction(period=6.0, actions=[qr_node]))
    ld.add_action(TimerAction(period=6.0, actions=[vlm_launch]))

    # ━ t=8s ━ AMCL 粒子滤波定位
    ld.add_action(TimerAction(period=8.0, actions=[amcl_node, amcl_lifecycle_manager]))

    # ━ t=12s ━ Nav2 导航堆栈
    ld.add_action(TimerAction(period=12.0, actions=[nav2_launch]))

    # ━ t=24s ━ 巡航节点
    ld.add_action(TimerAction(period=24.0, actions=[cruiser_node]))

    return ld
