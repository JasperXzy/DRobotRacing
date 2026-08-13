"""Bring up the test stack for manual driving and odometry drift checks.

The line detector, QR detector and visual_control are intentionally not started.
Run the interactive keyboard teleop in a separate terminal so it owns stdin.
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import LifecycleNode, Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    launch_dir = os.path.dirname(os.path.abspath(__file__))
    test_share = FindPackageShare('origincar_system')

    camera_config_file = LaunchConfiguration('camera_config_file')
    chassis_config_file = LaunchConfiguration('chassis_config_file')
    lidar_config_file = LaunchConfiguration('lidar_config_file')
    ekf_config_file = LaunchConfiguration('ekf_config_file')
    imu_topic = LaunchConfiguration('imu_topic')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    map_yaml_file = LaunchConfiguration('map_yaml_file')
    restriction_map_yaml_file = LaunchConfiguration(
        'restriction_map_yaml_file'
    )
    perception_detector_params_file = LaunchConfiguration(
        'perception_detector_params_file'
    )
    vlm_trigger_value = LaunchConfiguration('vlm_trigger_value')
    vlm_camera_topic = LaunchConfiguration('vlm_camera_topic')
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
            'imu_topic',
            default_value='/bmi088/imu/data',
            description='IMU topic fused by the EKF.',
        ),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=PathJoinSubstitution([
                test_share, 'config', 'nav2', 'nav2.yaml',
            ]),
            description='Map server and AMCL config for the odom test.',
        ),
        DeclareLaunchArgument(
            'map_yaml_file',
            default_value=PathJoinSubstitution([
                test_share, 'map', 'race_modify.yaml',
            ]),
            description='Physical-wall map used by AMCL localization.',
        ),
        DeclareLaunchArgument(
            'restriction_map_yaml_file',
            default_value=PathJoinSubstitution([
                test_share, 'map', 'race_keepout.yaml',
            ]),
            description='Restriction map used by the test workspace.',
        ),
        DeclareLaunchArgument(
            'perception_detector_params_file',
            default_value=PathJoinSubstitution([
                test_share,
                'config',
                'perception',
                'perception_detector.yaml',
            ]),
            description='Obstacle and parking detector configuration.',
        ),
        DeclareLaunchArgument(
            'vlm_trigger_value',
            default_value='8',
            description='Int32 value on /sign4return that triggers VLM.',
        ),
        DeclareLaunchArgument(
            'vlm_camera_topic',
            default_value='/camera/image_mjpeg_hbmem',
            description='Camera topic used by vlm_tts_demo.',
        ),
        DeclareLaunchArgument(
            'start_foxglove',
            default_value='true',
            description='Start foxglove_bridge for odometry observation.',
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

    bmi088_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('bmi088_driver'),
                'launch',
                'bmi088.launch.py',
            ])
        ),
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
            '--x', '0.05',
            '--y', '0.0',
            '--z', '0.05',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'laser',
        ],
    )

    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'ekf_test.launch.py')
        ),
        launch_arguments={
            'ekf_config_file': ekf_config_file,
            'imu_topic': imu_topic,
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

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params_file],
        remappings=[
            ('/scan', '/scan'),
        ],
    )

    amcl_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_odom_test_amcl',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server', 'restriction_map_server', 'amcl'],
        }],
    )

    perception_detector_node = Node(
        package='perception_detector',
        executable='perception_detector_node',
        name='perception_detector_node',
        output='screen',
        parameters=[perception_detector_params_file],
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
            'infer_on_capture': 'false',
            'defer_capture_write': 'true',
        }.items(),
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

        # t=0s: hardware, sensor TF, camera and visualization.
        chassis_launch,
        bmi088_launch,
        lidar_node,
        laser_static_tf,
        camera_launch,
        foxglove_launch,

        # t=3s: fuse wheel odometry and IMU, publishing odom -> base_link.
        TimerAction(period=3.0, actions=[ekf_launch]),

        # t=5s: provide map -> odom so drift can be compared against AMCL.
        TimerAction(period=5.0, actions=[
            map_server_node,
            restriction_map_server_node,
            amcl_node,
            amcl_lifecycle_manager,
        ]),

        # t=7s: keep non-line, non-QR camera perception and VLM/TTS enabled.
        TimerAction(period=7.0, actions=[
            perception_detector_node,
            vlm_launch,
        ]),
    ])
