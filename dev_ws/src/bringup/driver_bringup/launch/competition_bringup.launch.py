"""
比赛总启动文件
一键启动所有硬件驱动 + 感知节点 + 定位导航 + 巡航节点
按时间分阶段启动，避免同时启动导致资源竞争

启动时序（严格遵守依赖关系）:
  t=0s   → 第1阶段: 硬件驱动 + 静态位姿
            底盘驱动 / 激光雷达 / 摄像头 / 静态TF
  t=4s   → 第2阶段: 感知节点
            QR检测 + VLM图生文
            （等摄像头稳定出图后再启动）
  t=6s   → 第3阶段: 定位
            EKF 融合（IMU+里程计）
            （等激光雷达稳定出scan + 里程计有初始值）
  t=12s  → 第4阶段: 导航
            Nav2堆栈（含 AMCL 粒子滤波定位）
            （等EKF完成初值收敛、odom→base_link TF稳定，至少6秒）
  t=14s  → 第5阶段: 巡航
            waypoint_cruiser
            （等Nav2 lifecycle完全激活）

日志级别策略:
  第1阶段 (硬件驱动) → 默认 INFO（硬件初始化信息很重要）
  第2~3阶段           → 全局 WARN（抑制 Nav2/AMCL/EKF 的海量 INFO 刷屏）
  waypoint_cruiser    → 单独设回 INFO（巡航进度是用户最关心的有效信息）

用法:
    ros2 launch driver_bringup competition_bringup.launch.py

启动后等待约12秒，然后在另一个终端发令:
    ros2 run origincar_nav2 trigger_cruise
"""

import os
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
    # 第1阶段 (0秒): 硬件驱动 + 静态位姿
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
    # 第2阶段 (4秒后): 感知节点
    # 等摄像头驱动稳定出图后再启动
    # ============================================

    # 二维码检测节点 — 扫到即退出
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
            'launch_camera': 'false',        # 摄像头已在上层启动
            'trigger_value': '-1',           # 由巡航节点通过 /sign4return=-1 触发
            'camera_topic': '/camera/image',
        }.items()
    )

    # ============================================
    # 第3阶段a (6秒后): 定位
    # 等激光雷达稳定出scan + 底盘里程计有初值后再启动
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
    # 地图服务器 — 发布静态地图（t=8s，比 AMCL 早 2s）
    # ============================================

    nav2_params_file = os.path.join(ws_root, 'config', 'nav2', 'nav2(6.13).yaml')

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[nav2_params_file],
    )

    # ============================================
    # AMCL 定位节点 — 粒子滤波，发布 map→odom TF（t=10s）
    # ============================================

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params_file],
        remappings=[('/scan', '/scan')]
    )

    # AMCL + map_server 都是 LifecycleNode，统一由 lifecycle_manager 激活
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
    # 导航 + 巡航 (12秒后)
    # 等AMCL完成首次定位、map→odom TF稳定后再启动 Nav2
    # use_amcl:='false' 避免 Nav2 内部重复拉起 AMCL
    # ============================================

    # Nav2 导航堆栈
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
            'params_file': nav2_params_file,
            'use_sim_time': 'false',
            'use_amcl': 'false',
            'autostart': 'true',
        }.items()
    )

    # 巡航节点 — 启动后等待 /cruise_start 发令
    # 显式设为 INFO：巡航进度是用户最关心的有效信息
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
            'vlm_trigger_waypoint': 8,
            'vlm_trigger_value': -1,
            'vlm_trigger_topic': '/sign4return',
        }]
    )

    # ============================================
    # 组装: 分阶段启动（严格按依赖顺序）
    # ============================================
    ld = LaunchDescription()

    # ━━━ 第1阶段 (t=0s) ━━━ 硬件驱动 + 静态TF（保持默认 INFO 级别）
    ld.add_action(chassis_launch)
    ld.add_action(lidar_launch)
    ld.add_action(camera_launch)
    ld.add_action(origincar_launch)

    # ━━━ 切换日志级别 (t=3s) ━━━
    # 硬件驱动已启动完毕，后续所有节点降为 WARN，避免 Nav2/AMCL/EKF 刷屏
    ld.add_action(TimerAction(period=3.0, actions=[
        SetEnvironmentVariable('RCUTILS_LOGGING_SEVERITY_THRESHOLD', 'WARN')
    ]))

    # ━━━ 第2阶段 (t=4s) ━━━ 定位 — EKF 融合里程计+IMU → 继承 WARN
    ld.add_action(TimerAction(period=4.0, actions=[ekf_launch]))

    # ━━━ 第4阶段 (t=8s) ━━━ 地图服务器（先加载静态地图）→ 继承 WARN
    ld.add_action(TimerAction(period=8.0, actions=[map_server_node]))

    # ━━━ 第5阶段 (t=10s) ━━━ AMCL 定位（等地图就绪后初始化粒子滤波，发布 map→odom TF）→ 继承 WARN
    ld.add_action(TimerAction(period=10.0, actions=[amcl_node, amcl_lifecycle_manager]))

    # ━━━ 第5阶段 (t=20s) ━━━ 导航（等AMCL定位稳定、map→odom 就绪）→ 继承 WARN
    ld.add_action(TimerAction(period=20.0, actions=[nav2_launch]))

    # ━━━ 第6阶段 (t=40s) ━━━ 巡航节点（等Nav2 lifecycle完全激活）→ INFO
    ld.add_action(TimerAction(period=40.0, actions=[cruiser_node]))

    # ━━━ 第3阶段 (t=30s) ━━━ 感知节点（等摄像头稳定）→ 继承 WARN
    ld.add_action(TimerAction(period=30.0, actions=[qr_node]))
    ld.add_action(TimerAction(period=30.0, actions=[vlm_launch]))



    return ld