"""
Full test bringup using lidar_location instead of AMCL.

The original test_bringup.launch.py remains unchanged for A/B comparison.
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    launch_dir = os.path.dirname(os.path.abspath(__file__))
    test_share = FindPackageShare('origincar_system')

    camera_config_file = LaunchConfiguration('camera_config_file')
    chassis_config_file = LaunchConfiguration('chassis_config_file')
    lidar_config_file = LaunchConfiguration('lidar_config_file')
    ekf_config_file = LaunchConfiguration('ekf_config_file')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    localization_params_file = LaunchConfiguration('localization_params_file')
    map_yaml_file = LaunchConfiguration('map_yaml_file')
    restriction_map_yaml_file = LaunchConfiguration('restriction_map_yaml_file')
    qr_detector_params_file = LaunchConfiguration('qr_detector_params_file')
    perception_detector_params_file = LaunchConfiguration('perception_detector_params_file')
    line_detector_params_file = LaunchConfiguration('line_detector_params_file')
    vlm_trigger_value = LaunchConfiguration('vlm_trigger_value')
    vlm_camera_topic = LaunchConfiguration('vlm_camera_topic')
    qr_stop_on_detect = LaunchConfiguration('qr_stop_on_detect')
    start_foxglove = LaunchConfiguration('start_foxglove')

    args = [
        DeclareLaunchArgument(
            'camera_config_file',
            default_value=PathJoinSubstitution([
                test_share, 'config', 'drivers', 'camera_driver.yaml',
            ]),
            description='Camera config maintained in the test workspace.',
        ),
        DeclareLaunchArgument(
            'chassis_config_file',
            default_value=PathJoinSubstitution([
                test_share, 'config', 'drivers', 'chassis_driver.yaml',
            ]),
            description='Chassis config maintained in the test workspace.',
        ),
        DeclareLaunchArgument(
            'lidar_config_file',
            default_value=PathJoinSubstitution([
                test_share, 'config', 'drivers', 'lidar_driver.yaml',
            ]),
            description='Lidar config maintained in the test workspace.',
        ),
        DeclareLaunchArgument(
            'ekf_config_file',
            default_value=PathJoinSubstitution([
                test_share, 'config', 'localization', 'ekf.yaml',
            ]),
            description='EKF config maintained in the test workspace.',
        ),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=PathJoinSubstitution([
                test_share, 'config', 'nav2', 'nav2.yaml',
            ]),
            description='Map server config maintained in the test workspace.',
        ),
        DeclareLaunchArgument(
            'localization_params_file',
            default_value=PathJoinSubstitution([
                test_share, 'config', 'localization', 'lidar_location.yaml',
            ]),
            description='Scan-matching localization config replacing AMCL.',
        ),
        DeclareLaunchArgument(
            'map_yaml_file',
            default_value=PathJoinSubstitution([
                test_share, 'map', 'race_modify.yaml',
            ]),
            description='Physical-wall map used by lidar localization.',
        ),
        DeclareLaunchArgument(
            'restriction_map_yaml_file',
            default_value=PathJoinSubstitution([
                test_share, 'map', 'race_keepout.yaml',
            ]),
            description='Keepout map used for waypoint_shift direction and static scan filtering.',
        ),
        DeclareLaunchArgument(
            'qr_detector_params_file',
            default_value=PathJoinSubstitution([
                test_share, 'config', 'perception', 'qr_detector.yaml',
            ]),
            description='QR detector config maintained in the test workspace.',
        ),
        DeclareLaunchArgument(
            'perception_detector_params_file',
            default_value=PathJoinSubstitution([
                test_share, 'config', 'perception', 'perception_detector.yaml',
            ]),
            description='Perception detector config maintained in the test workspace.',
        ),
        DeclareLaunchArgument(
            'line_detector_params_file',
            default_value=PathJoinSubstitution([
                test_share, 'config', 'perception', 'line_detector.yaml',
            ]),
            description='Line detector config maintained in the test workspace.',
        ),
        DeclareLaunchArgument(
            'vlm_trigger_value',
            default_value='8',
            description='Int32 value on /sign4return that triggers VLM capture/inference.',
        ),
        DeclareLaunchArgument(
            'vlm_camera_topic',
            default_value='/camera/image',
            description='Camera topic used by vlm_tts_demo.',
        ),
        DeclareLaunchArgument(
            'qr_stop_on_detect',
            default_value='true',
            description='Stop QR decoding after the first valid code selects the channel route.',
        ),
        DeclareLaunchArgument(
            'start_foxglove',
            default_value='true',
            description='Start foxglove_bridge together with the test bringup.',
        ),
    ]

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('camera_driver'),
                'launch',
                'camera.launch.py',
            ])
        ),
        launch_arguments={
            'camera_config_file': camera_config_file,
        }.items(),
    )

    lidar_node = LifecycleNode(
        package='lidar_driver',
        executable='lidar_driver_node',
        name='lidar_driver_node',
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=[lidar_config_file],
    )

    laser_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser',
        arguments=[
            '--x', '0.0325',
            '--y', '0.0',
            '--z', '0.05',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'laser',
        ],
    )

    chassis_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('chassis_driver'),
                'launch',
                'chassis.launch.py',
            ])
        ),
        launch_arguments={
            'chassis_config_file': chassis_config_file,
        }.items(),
    )

    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'ekf_test.launch.py')
        ),
        launch_arguments={
            'ekf_config_file': ekf_config_file,
        }.items(),
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            nav2_params_file,
            {'yaml_filename': map_yaml_file},
        ],
    )

    restriction_map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='restriction_map_server',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'yaml_filename': restriction_map_yaml_file,
            'topic_name': 'restriction_map',
            'frame_id': 'map',
        }],
    )

    lidar_location_node = Node(
        package='lidar_location',
        executable='lidar_location_node',
        name='lidar_location',
        output='screen',
        parameters=[localization_params_file],
    )

    map_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_test_lidar_location',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server', 'restriction_map_server'],
        }],
    )

    perception_detector_node = Node(
        package='perception_detector',
        executable='perception_detector_node',
        name='perception_detector_node',
        output='screen',
        parameters=[perception_detector_params_file],
    )

    line_detector_node = Node(
        package='line_detector',
        executable='line_detector_node',
        name='GetLineCoordinate',
        output='screen',
        parameters=[line_detector_params_file],
        arguments=['--ros-args', '--log-level', 'info'],
    )

    vlm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('vlm_tts_demo'),
                'launch',
                'vlm_tts_demo.launch.py',
            ])
        ),
        launch_arguments={
            'launch_camera': 'false',
            'launch_tts': 'true',
            'camera_topic': vlm_camera_topic,
            'trigger_value': vlm_trigger_value,
            'infer_on_capture': 'true',
            'captured_image_file':
                '/root/D_Robot/test/dev_ws/captures/vlm_tts_demo_capture.jpg',
        }.items(),
    )

    qr_detector_node = Node(
        package='qr_detector_cpp',
        executable='qr_detector_node',
        name='qr_detector_node',
        output='screen',
        parameters=[
            qr_detector_params_file,
            {
                'stop_on_detect': ParameterValue(qr_stop_on_detect, value_type=bool),
            },
        ],
    )

    wait_for_first_localization_lock = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'echo', '--no-daemon',
            '--filter', "m.data == 'LOCKED'", '--once',
            '/lidar_location/state', 'std_msgs/msg/String',
        ],
        output='screen',
    )

    startup_complete_announcement = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--once',
            '/tts_text', 'std_msgs/msg/String',
            "{data: '启动完成'}",
        ],
        output='screen',
    )

    def start_qr_after_localization_lock(event, _context):
        if event.returncode == 0:
            return [
                LogInfo(msg='定位已锁定，启动二维码检测。'),
                qr_detector_node,
            ]
        return [
            LogInfo(msg='错误：等待定位锁定的进程异常退出，二维码检测未启动。'),
        ]

    qr_start_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_first_localization_lock,
            on_exit=start_qr_after_localization_lock,
        )
    )

    foxglove_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('foxglove_bridge'),
                'launch',
                'foxglove_bridge_launch.xml',
            ])
        ),
        condition=IfCondition(start_foxglove),
    )

    return LaunchDescription([
        *args,
        qr_start_handler,

        # t=0s: hardware, the single measured laser TF and visualization bridge.
        chassis_launch,
        lidar_node,
        laser_static_tf,
        camera_launch,
        foxglove_launch,

        # t=4s: EKF publishes odom -> base_link.
        TimerAction(period=4.0, actions=[ekf_launch]),

        # t=7s: map servers plus lidar_location; start waiting for its first LOCKED state.
        TimerAction(period=7.0, actions=[
            map_server_node,
            restriction_map_server_node,
            lidar_location_node,
            map_lifecycle_manager,
            wait_for_first_localization_lock,
        ]),

        # t=15s: camera perception, line detection and VLM/TTS.
        TimerAction(period=15.0, actions=[
            perception_detector_node,
            line_detector_node,
            vlm_launch,
        ]),

        # t=18s: allow the TTS subscriber to start, then announce once.
        TimerAction(period=18.0, actions=[startup_complete_announcement]),

        # QR detector is started by qr_start_handler after the first LOCKED state.
    ])
