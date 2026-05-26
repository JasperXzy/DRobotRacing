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

    ws_root = os.path.abspath(os.path.join(own_nav_pkg, '..'))
    nav2_params_file = LaunchConfiguration('params_file')
    default_config_path = os.path.join(ws_root, 'config', 'nav2', 'nav2.yaml')
    map_file = os.path.join(ws_root, 'src', 'origincar_nav2', 'map')
    
    path_to_xacro = os.path.join(ws_root, 'src', 'drivers', 'origincar_description', 'urdf', 'origincar.xarco')
    # 声明 launch 参数
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_config_path,
        description='Path to the config file')

    declare_map_cmd = DeclareLaunchArgument(
        'maps',
        default_value=os.path.join(get_package_share_directory(own_nav_pkg), 'maps', 'my_map.yaml'),
        description='Full path to map file')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')


    # ------------------------------
    # 2. 机器人状态发布器 (robot_state_publisher)
    # ------------------------------
    # 使用 xacro 处理 urdf 文件
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
    # 官方 navigation_launch.py 会启动 controller_server, planner_server, bt_navigator, amcl, map_server 等
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(nav2_bringup_pkg), 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'params_file': nav2_params_file,
            'use_sim_time': use_sim_time,
            'autostart': 'true'
        }.items()
    )

    # ------------------------------
    # 5. 键盘控制节点（适用于阿克曼转向）
    # ------------------------------
    # 方法：使用 teleop_twist_keyboard 发布 Twist 消息，然后通过 twist_to_ackermann 节点转换为阿克曼指令
    # 前提：需要安装 teleop_twist_keyboard 和 ackermann_msgs
    # sudo apt install ros-${ROS_DISTRO}-teleop-twist-keyboard

    # 5.1 键盘节点 (发布 /cmd_vel Twist)
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        prefix='xterm -e',          # 在新终端中运行，方便按键输入
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel', '/teleop_cmd_vel')]  # 先发布到中间话题
    )

    # 5.2 Twist -> AckermannDrive 转换节点 (简单 Python 节点)
    # 如果底盘驱动直接接收 /cmd_vel (Twist) 并处理阿克曼，则不需要转换。
    # 但通常阿克曼底盘驱动订阅的是 /ackermann_cmd (AckermannDrive)
    # 我们提供一个内嵌脚本或外部节点。这里假设你有一个可执行节点 twist_to_ackermann。
    # 如果没有，可以创建如下简单节点（也可以直接在 launch 中定义）
    twist_to_ackermann_node = Node(
        package='your_own_package',   # 请替换为实际包名，或者直接使用下面 inline 的脚本
        executable='twist_to_ackermann',
        name='twist_to_ackermann',
        output='screen',
        remappings=[
            ('twist', '/teleop_cmd_vel'),          # 订阅键盘发布的 Twist
            ('ackermann', '/ackermann_cmd')        # 发布给底盘驱动的阿克曼指令
        ],
        parameters=[{
            'wheelbase': 0.3,        # 轴距 (m)，根据你的车调整
            'max_steering_angle': 0.6,  # 最大转向角 (rad)，约 34 度
            'max_speed': 1.0         # 最大线速度 (m/s)
        }]
    )

    # 如果你没有单独编译 twist_to_ackermann 节点，可以内嵌一个 Python 脚本（需先创建）
    # 下面提供该脚本的内容，你需要将它保存为 `twist_to_ackermann.py` 并安装到你的包中
    # 这里为了完整性，给出了节点的定义，实际使用时需要确保该节点存在。

    # 另一种简单方案：如果底盘驱动直接接受 /cmd_vel 并且内部已做阿克曼转换，则只需要 teleop_node
    # 且 remappings 直接将 cmd_vel 映射到底盘订阅的话题，例如 /cmd_vel
    # 根据你的实际底盘驱动，可能需要修改。

    # 为了示例，我们假设底盘驱动订阅的是 /ackermann_cmd，因此添加转换节点。
    # 如果你底盘驱动已支持 /cmd_vel，可以省略 twist_to_ackermann_node。

    # ------------------------------
    # 6. 返回 LaunchDescription
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

    # 添加键盘控制
    #ld.add_action(teleop_node)
    #ld.add_action(twist_to_ackermann_node)   # 如果不需要则注释掉

    return ld