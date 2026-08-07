"""
Full test bringup using lidar_location instead of AMCL.

Keep non-localization behavior aligned with test_bringup.launch.py for A/B comparison.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
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
    logic3_static_restriction_map_yaml_file = LaunchConfiguration(
        'logic3_static_restriction_map_yaml_file'
    )
    logic3_runtime_restriction_map_yaml_file = LaunchConfiguration(
        'logic3_runtime_restriction_map_yaml_file'
    )
    qr_detector_params_file = LaunchConfiguration('qr_detector_params_file')
    perception_detector_params_file = LaunchConfiguration('perception_detector_params_file')
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
            'logic3_static_restriction_map_yaml_file',
            default_value=PathJoinSubstitution([
                test_share, 'map', 'race_keepout_logic3_static.yaml',
            ]),
            description='Dedicated channel map used only for logic3 pre-launch green detection.',
        ),
        DeclareLaunchArgument(
            'logic3_runtime_restriction_map_yaml_file',
            default_value=PathJoinSubstitution([
                test_share, 'map', 'race_keepout_logic3_runtime.yaml',
            ]),
            description='Map 4 filters logic3 channel lidar points and runtime red records.',
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

    # Start the driver directly so the test flow publishes exactly one
    # base_link -> laser transform using the measured horizontal offset.
    lidar_node = LifecycleNode(
        package='lidar_driver',
        executable='lidar_driver_node',
        name='lidar_driver_node',
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=[lidar_config_file],
        arguments=['--ros-args', '--log-level', 'warn'],
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
            '--ros-args', '--log-level', 'warn',
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
        arguments=['--ros-args', '--log-level', 'warn'],
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
        arguments=['--ros-args', '--log-level', 'warn'],
    )

    logic3_static_restriction_map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='logic3_static_restriction_map_server',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'yaml_filename': logic3_static_restriction_map_yaml_file,
            'topic_name': 'logic3_static_restriction_map',
            'frame_id': 'map',
        }],
        arguments=['--ros-args', '--log-level', 'warn'],
    )

    logic3_runtime_restriction_map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='logic3_runtime_restriction_map_server',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'yaml_filename': logic3_runtime_restriction_map_yaml_file,
            'topic_name': 'logic3_runtime_restriction_map',
            'frame_id': 'map',
        }],
        arguments=['--ros-args', '--log-level', 'warn'],
    )

    lidar_location_node = Node(
        package='lidar_location',
        executable='lidar_location_node',
        name='lidar_location',
        output='screen',
        parameters=[localization_params_file],
        arguments=['--ros-args', '--log-level', 'warn'],
    )

    map_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_test_lidar_location',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': [
                'map_server',
                'restriction_map_server',
                'logic3_static_restriction_map_server',
                'logic3_runtime_restriction_map_server',
            ],
        }],
        arguments=['--ros-args', '--log-level', 'warn'],
    )

    perception_detector_node = Node(
        package='perception_detector',
        executable='perception_detector_node',
        name='perception_detector_node',
        output='screen',
        parameters=[perception_detector_params_file],
        arguments=['--ros-args', '--log-level', 'warn'],
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
            'fallback_trigger_value': '88',
            'fallback_candidate_topic':
                '/visual_lidar_path_follower/image_capture_candidate',
            'fallback_candidate_dir':
                '/root/D_Robot/test/dev_ws/captures/image_capture_candidates',
            'fallback_candidate_max_frames': '80',
            'infer_on_capture': 'true',
            'playback_device': 'plughw:1,0',
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
                'speak_on_detect': False,
            },
        ],
        arguments=['--ros-args', '--log-level', 'warn'],
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

        # t=0s: hardware, the single measured laser TF and visualization bridge.
        chassis_launch,
        lidar_node,
        laser_static_tf,
        camera_launch,
        foxglove_launch,

        # t=3s: odom fusion and required IMU static TFs.
        TimerAction(period=3.0, actions=[ekf_launch]),

        # t=5s: physical map, restriction map and lidar_location.
        TimerAction(period=5.0, actions=[
            map_server_node,
            restriction_map_server_node,
            logic3_static_restriction_map_server_node,
            logic3_runtime_restriction_map_server_node,
            lidar_location_node,
            map_lifecycle_manager,
        ]),

        # t=7s: camera perception and VLM/TTS. Task1不再启动视觉巡线节点。
        TimerAction(period=7.0, actions=[
            perception_detector_node,
            vlm_launch,
        ]),

        # t=9s: QR detector.
        TimerAction(period=9.0, actions=[qr_detector_node]),
    ])
