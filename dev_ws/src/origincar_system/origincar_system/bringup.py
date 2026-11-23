"""Shared hardware, localization and perception launch description."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def _package_file(package, *parts):
    return PathJoinSubstitution([FindPackageShare(package), *parts])


def _system_argument(name, *parts, description):
    return DeclareLaunchArgument(
        name, default_value=_package_file('origincar_system', *parts), description=description,
    )


def _include(package, filename, **arguments):
    source = PythonLaunchDescriptionSource(_package_file(package, 'launch', filename))
    return IncludeLaunchDescription(source, launch_arguments=arguments.items())


def _launch_arguments(lidar_localization, competition):
    args = [
        _system_argument(
            'camera_config_file', 'config', 'drivers', 'camera_driver.yaml',
            description='USB camera and codec settings.',
        ),
        _system_argument(
            'chassis_config_file', 'config', 'drivers', 'chassis_driver.yaml',
            description='Chassis serial and command settings.',
        ),
        _system_argument(
            'lidar_config_file', 'config', 'drivers', 'lidar_driver.yaml',
            description='Laser scanner settings.',
        ),
        _system_argument(
            'ekf_config_file', 'config', 'localization', 'ekf.yaml',
            description='Wheel odometry and IMU fusion settings.',
        ),
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/chassis/imu/data_raw' if lidar_localization else '/bmi088/imu/data',
            description='IMU topic fused by the EKF.',
        ),
        _system_argument(
            'nav2_params_file', 'config', 'nav2', 'nav2.yaml',
            description='Map server and AMCL settings.',
        ),
    ]
    if lidar_localization:
        args.append(DeclareLaunchArgument(
            'localization_params_file',
            default_value=_package_file('lidar_location', 'config', 'lidar_location.yaml'),
            description='Laser scan-matching settings.',
        ))
    args.extend([
        _system_argument(
            'map_yaml_file', 'map', 'race_modify.yaml', description='Physical-wall map.',
        ),
        _system_argument(
            'restriction_map_yaml_file', 'map', 'race_keepout.yaml',
            description='Restriction map used for waypoint shifts and scan filtering.',
        ),
    ])
    if competition:
        args.extend([
            _system_argument(
                'logic3_static_restriction_map_yaml_file',
                'map', 'race_keepout_logic3_static.yaml',
                description='Restriction map for obstacle confirmation before departure.',
            ),
            _system_argument(
                'logic3_runtime_restriction_map_yaml_file',
                'map', 'race_keepout_logic3_runtime.yaml',
                description='Restriction map for in-race lidar processing.',
            ),
            _system_argument(
                'qr_detector_params_file', 'config', 'perception', 'qr_detector.yaml',
                description='QR decoder settings.',
            ),
        ])
    args.extend([
        _system_argument(
            'perception_detector_params_file', 'config', 'perception', 'perception_detector.yaml',
            description='Object detector settings.',
        ),
        DeclareLaunchArgument(
            'vlm_trigger_value', default_value='8',
            description='Value on /sign4return that selects the image for VLM.',
        ),
        DeclareLaunchArgument(
            'vlm_camera_topic', default_value='/camera/image_mjpeg_hbmem',
            description='Camera topic used by the VLM node.',
        ),
        DeclareLaunchArgument(
            'vlm_tts_speed', default_value='1.25',
            description='Pitch-preserving playback speed for VLM speech.',
        ),
    ])
    if competition:
        args.append(DeclareLaunchArgument(
            'qr_stop_on_detect', default_value='true',
            description='Stop QR decoding after the first valid code.',
        ))
    args.append(DeclareLaunchArgument(
        'start_foxglove', default_value='true', description='Start the Foxglove bridge.',
    ))
    return args


def _localization_actions(lidar_localization, competition, node_arguments):
    servers = [Node(
        package='nav2_map_server', executable='map_server', name='map_server', output='screen',
        parameters=[
            LaunchConfiguration('nav2_params_file'),
            {'yaml_filename': LaunchConfiguration('map_yaml_file')},
        ],
        **node_arguments,
    )]
    topics = ['restriction_map']
    if competition:
        topics.extend(['logic3_static_restriction_map', 'logic3_runtime_restriction_map'])
    for topic in topics:
        servers.append(Node(
            package='nav2_map_server', executable='map_server', name=f'{topic}_server',
            output='screen',
            parameters=[{
                'use_sim_time': False, 'yaml_filename': LaunchConfiguration(f'{topic}_yaml_file'),
                'topic_name': topic, 'frame_id': 'map',
            }],
            **node_arguments,
        ))

    managed_nodes = ['map_server', *(f'{topic}_server' for topic in topics)]
    if lidar_localization:
        localizer = Node(
            package='lidar_location', executable='lidar_location_node', name='lidar_location',
            output='screen', parameters=[LaunchConfiguration('localization_params_file')],
            **node_arguments,
        )
    else:
        localizer = Node(
            package='nav2_amcl', executable='amcl', name='amcl', output='screen',
            parameters=[LaunchConfiguration('nav2_params_file')], remappings=[('/scan', '/scan')],
            **node_arguments,
        )
        managed_nodes.append('amcl')

    mode = 'test' if competition else 'odom_test'
    backend = 'lidar_location' if lidar_localization else 'amcl'
    lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name=f'lifecycle_manager_{mode}_{backend}', output='screen',
        parameters=[{'use_sim_time': False, 'autostart': True, 'node_names': managed_nodes}],
        **node_arguments,
    )
    return [*servers, localizer, lifecycle]


def generate_bringup(*, lidar_localization, competition):
    """Build one of the four documented competition and odometry entry points."""
    warn = ['--ros-args', '--log-level', 'warn'] if competition else []
    node_arguments = {'arguments': warn} if warn else {}
    hardware = [
        _include('chassis_driver', 'chassis.launch.py',
                 chassis_config_file=LaunchConfiguration('chassis_config_file')),
        _include('bmi088_driver', 'bmi088.launch.py'),
        LifecycleNode(
            package='lidar_driver', executable='lidar_driver_node', name='lidar_driver_node',
            namespace='', output='screen', emulate_tty=True,
            parameters=[LaunchConfiguration('lidar_config_file')], **node_arguments,
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher', name='base_link_to_laser',
            arguments=[
                '--x', '0.035', '--y', '0.0', '--z', '0.05',
                '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                '--frame-id', 'base_link', '--child-frame-id', 'laser', *warn,
            ],
        ),
        _include('camera_driver', 'camera.launch.py',
                 camera_config_file=LaunchConfiguration('camera_config_file')),
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(_package_file(
                'foxglove_bridge', 'launch', 'foxglove_bridge_launch.xml',
            )),
            condition=IfCondition(LaunchConfiguration('start_foxglove')),
        ),
    ]
    ekf = _include(
        'origincar_system', 'ekf_test.launch.py',
        ekf_config_file=LaunchConfiguration('ekf_config_file'),
        imu_topic=LaunchConfiguration('imu_topic'),
    )
    perception = [
        Node(
            package='perception_detector', executable='perception_detector_node',
            name='perception_detector_node', output='screen',
            parameters=[LaunchConfiguration('perception_detector_params_file')],
            **node_arguments,
        ),
        _include(
            'vlm_tts_demo', 'vlm_tts_demo.launch.py', launch_camera='false', launch_tts='true',
            camera_topic=LaunchConfiguration('vlm_camera_topic'),
            trigger_value=LaunchConfiguration('vlm_trigger_value'),
            vlm_tts_speed=LaunchConfiguration('vlm_tts_speed'),
            infer_on_capture='false', defer_capture_write='true',
        ),
    ]
    actions = [
        *_launch_arguments(lidar_localization, competition),
        *hardware,
        TimerAction(period=3.0, actions=[ekf]),
        TimerAction(period=5.0, actions=_localization_actions(
            lidar_localization, competition, node_arguments,
        )),
        TimerAction(period=7.0, actions=perception),
    ]
    if competition:
        actions.append(TimerAction(period=9.0, actions=[Node(
            package='qr_detector_cpp', executable='qr_detector_node', name='qr_detector_node',
            output='screen',
            parameters=[
                LaunchConfiguration('qr_detector_params_file'),
                {
                    'stop_on_detect': ParameterValue(
                        LaunchConfiguration('qr_stop_on_detect'), value_type=bool,
                    ),
                    'speak_on_detect': False,
                },
            ],
            **node_arguments,
        )]))
    return LaunchDescription(actions)
