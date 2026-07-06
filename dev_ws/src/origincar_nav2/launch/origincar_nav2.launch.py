#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command
from launch.conditions import UnlessCondition
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # ------------------------------
    # 1. 定义路径和参数
    # ------------------------------
    # 功能包名称（请根据实际情况修改）
    nav2_bringup_pkg = 'nav2_bringup'          # 官方 Nav2 bringup 包
    own_nav_pkg = 'origincar_nav2'                 # 你自己创建的 navigation 元包（或直接使用 nav2_bringup）
    robot_desc_pkg = 'origincar_description'   # 你的机器人描述包
    location_pkg = 'state_estimation'          # 你的 EKF 和 TF 启动包（可选）

    # 路径

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    nav2_params_file = LaunchConfiguration('params_file')
    # 注意：config 路径在 dev_ws/config/nav2/ 下，不是在包内
    default_config_path = os.path.join(
        get_package_share_directory(own_nav_pkg), '..', '..', '..', '..', 'config', 'nav2', 'nav2.yaml')

    # 声明 launch 参数
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_config_path,
        description='Path to the config file')

    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(get_package_share_directory(own_nav_pkg), 'map', 'my_map.yaml'),
        description='Full path to map file')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')


    # ------------------------------
    # 2. 机器人状态发布器 (robot_state_publisher)
    # ------------------------------
    # 使用 xacro 处理 urdf 文件
    path_to_xacro = os.path.join(
        get_package_share_directory(robot_desc_pkg), 'urdf', 'origincar.urdf')
    robot_description_content = Command(['xacro ', path_to_xacro])
    robot_description_param = ParameterValue(robot_description_content, value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_param,
                     'use_sim_time': use_sim_time}]
    )

    # joint_state_publisher (可选，用于发布非固定关节，如果没有关节则可不加)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=UnlessCondition(LaunchConfiguration('use_jsp', default='false'))
    )

    # ------------------------------
    # 3. 启动 EKF 和静态 TF（如果你没有单独启动 location 包）
    # ------------------------------
    # 这里假定你有一个单独的 launch 文件用于启动 ekf 和静态 tf
    # 如果没有，可以在这里直接包含，或者直接在你的总 launch 中启动相关节点
    # 以下是包含 location 包的例子（假设 location 包中有 ekf.launch.py）
    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(location_pkg), 'launch', 'ekf.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # ------------------------------
    # 4. 启动 Nav2 核心组件（使用官方 nav2_bringup 的 launch 文件）
    # ------------------------------
    # 官方 navigation_launch.py 会启动 controller_server, planner_server, bt_navigator, map_server 等
    # use_amcl:='true' 启用 AMCL 粒子滤波定位，发布 map→odom TF
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(nav2_bringup_pkg), 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': nav2_params_file,
            'use_sim_time': use_sim_time,
            'use_amcl': 'false',
            'autostart': 'true'
        }.items()
    )

    # ------------------------------
    # 5. 返回 LaunchDescription
    # ------------------------------
    ld = LaunchDescription()
    # 添加声明参数
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_map_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    # 添加机器人描述节点
    #ld.add_action(robot_state_publisher_node)
    #ld.add_action(joint_state_publisher_node)

    # 添加 EKF 启动（如果你需要）
    # ld.add_action(ekf_launch)   # 根据实际情况决定是否包含，避免重复启动

    # 添加 Nav2 核心
    ld.add_action(nav2_launch)
    # 添加地图 PNG 叠加发布器（RViz2 可视化用）
    map_overlay_node = Node(
        package='origincar_nav2',
        executable='map_overlay_publisher',
        name='map_overlay_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'alpha': 0.5,
                     'publish_grid': True,
                     'publish_marker': True,
                     'publish_image': True}]
    )

    return ld