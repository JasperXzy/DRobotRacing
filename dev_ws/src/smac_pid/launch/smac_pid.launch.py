import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = get_package_share_directory('smac_pid')
    origincar_share = get_package_share_directory('origincar_nav2')
    dev_workspace = os.path.abspath(os.path.join(package_share, '..', '..', '..', '..'))

    default_params = os.path.join(package_share, 'config', 'smac_pid.yaml')
    # NavFn A* is the selected planner.
    default_nav2_params = os.path.join(package_share, 'config', 'nav2_navfn_pid.yaml')
    default_route = os.path.join(package_share, 'config', 'path1.yaml')
    default_map = os.path.join(origincar_share, 'map', 'my_map.yaml')
    default_chassis_config = os.path.join(
        dev_workspace, 'config', 'drivers', 'chassis_driver.yaml')
    default_lidar_config = os.path.join(
        dev_workspace, 'config', 'drivers', 'lidar_driver.yaml')

    params_file = LaunchConfiguration('params_file')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    route_file = LaunchConfiguration('route_file')
    map_file = LaunchConfiguration('map_file')
    enable_motion = LaunchConfiguration('enable_motion')
    start_hardware = LaunchConfiguration('start_hardware')
    start_foxglove = LaunchConfiguration('start_foxglove')
    foxglove_port = LaunchConfiguration('foxglove_port')
    initial_x = LaunchConfiguration('initial_x')
    initial_y = LaunchConfiguration('initial_y')
    initial_yaw = LaunchConfiguration('initial_yaw')
    max_speed = LaunchConfiguration('max_speed')

    arguments = [
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('nav2_params_file', default_value=default_nav2_params),
        DeclareLaunchArgument('route_file', default_value=default_route),
        DeclareLaunchArgument('map_file', default_value=default_map),
        DeclareLaunchArgument(
            'enable_motion', default_value='false',
            description='Publish /cmd_vel only after safe planning-only validation.'),
        DeclareLaunchArgument(
            'start_hardware', default_value='true',
            description='Start chassis and lidar drivers.'),
        DeclareLaunchArgument(
            'start_foxglove', default_value='true',
            description='Start foxglove_bridge together with the navigation stack.'),
        DeclareLaunchArgument(
            'foxglove_port', default_value='8765',
            description='WebSocket port exposed by foxglove_bridge.'),
        DeclareLaunchArgument(
            'initial_x', default_value='0.0',
            description='AMCL initial map X position.'),
        DeclareLaunchArgument(
            'initial_y', default_value='0.0',
            description='AMCL initial map Y position.'),
        DeclareLaunchArgument(
            'initial_yaw', default_value='0.0',
            description='AMCL initial map yaw in radians; 0 faces map +X.'),
        DeclareLaunchArgument(
            'max_speed', default_value='0.20',
            description='Maximum follower speed; keep 0.20 for the first closed-loop test.'),
    ]

    chassis = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('chassis_driver'), 'launch', 'chassis.launch.py'
            ])
        ),
        launch_arguments={'chassis_config_file': default_chassis_config}.items(),
        condition=IfCondition(start_hardware),
    )

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('lidar_driver'), 'launch', 'lidar.launch.py'
            ])
        ),
        launch_arguments={'lidar_config_file': default_lidar_config}.items(),
        condition=IfCondition(start_hardware),
    )

    ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('state_estimation'), 'launch', 'ekf.launch.py'
            ])
        )
    )

    foxglove = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('foxglove_bridge'),
                'launch',
                'foxglove_bridge_launch.xml',
            ])
        ),
        launch_arguments={'port': foxglove_port}.items(),
        condition=IfCondition(start_foxglove),
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[nav2_params_file, {'yaml_filename': map_file}],
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            nav2_params_file,
            {
                'initial_pose.x': ParameterValue(initial_x, value_type=float),
                'initial_pose.y': ParameterValue(initial_y, value_type=float),
                'initial_pose.yaw': ParameterValue(initial_yaw, value_type=float),
            },
        ],
        remappings=[('/scan', '/scan')],
    )

    localization_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_smac_pid_localization',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
        }],
    )

    planner = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_file],
    )

    planner_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_smac_pid_planner',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['planner_server'],
        }],
    )

    follower = Node(
        package='smac_pid',
        executable='smac_pid_node',
        name='smac_pid',
        output='screen',
        parameters=[
            params_file,
            {
                'route_file': route_file,
                'enable_motion': ParameterValue(enable_motion, value_type=bool),
                'max_speed': ParameterValue(max_speed, value_type=float),
            },
        ],
    )

    return LaunchDescription([
        *arguments,
        foxglove,
        chassis,
        lidar,
        TimerAction(period=2.0, actions=[ekf]),
        TimerAction(
            period=4.0,
            actions=[map_server, amcl, localization_lifecycle]),
        TimerAction(period=7.0, actions=[planner, planner_lifecycle]),
        TimerAction(period=9.0, actions=[follower]),
    ])
