import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    launch_dir = os.path.dirname(os.path.abspath(__file__))

    lidar_config_file = LaunchConfiguration('lidar_config_file')
    ekf_config_file = LaunchConfiguration('ekf_config_file')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    qr_detector_params_file = LaunchConfiguration('qr_detector_params_file')
    perception_detector_params_file = LaunchConfiguration('perception_detector_params_file')
    vlm_trigger_value = LaunchConfiguration('vlm_trigger_value')
    vlm_camera_topic = LaunchConfiguration('vlm_camera_topic')
    qr_stop_on_detect = LaunchConfiguration('qr_stop_on_detect')
    start_foxglove = LaunchConfiguration('start_foxglove')

    args = [
        DeclareLaunchArgument(
            'lidar_config_file',
            default_value='/root/D_Robot/dev_ws/config/drivers/lidar_driver_test.yaml',
            description='Test lidar config file. Mainline lidar_driver.yaml is not modified.',
        ),
        DeclareLaunchArgument(
            'ekf_config_file',
            default_value='/root/D_Robot/dev_ws/config/localization/ekf_test.yaml',
            description='Test EKF config file. Mainline ekf.yaml is not modified.',
        ),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value='/root/D_Robot/dev_ws/config/nav2/nav2_test.yaml',
            description='Test map_server and AMCL params file.',
        ),
        DeclareLaunchArgument(
            'qr_detector_params_file',
            default_value='/root/D_Robot/dev_ws/config/perception/qr_detector.yaml',
            description='Mainline QR detector params file.',
        ),
        DeclareLaunchArgument(
            'perception_detector_params_file',
            default_value='/root/D_Robot/dev_ws/config/perception/perception_detector.yaml',
            description='Mainline perception detector params file.',
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
            default_value='false',
            description='Keep QR publishing in test flow so smooth_path_follower can start later.',
        ),
        DeclareLaunchArgument(
            'start_foxglove',
            default_value='false',
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
        )
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('lidar_driver'),
                'launch',
                'lidar.launch.py',
            ])
        ),
        launch_arguments={
            'lidar_config_file': lidar_config_file,
        }.items(),
    )

    test_lidar_hz_order = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '--once', '/lidar/order', 'std_msgs/msg/Int8', '{data: 12}'],
        output='screen',
    )

    chassis_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('chassis_driver'),
                'launch',
                'chassis.launch.py',
            ])
        )
    )

    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('origincar_description'),
                'launch',
                'display.launch.py',
            ])
        )
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
        parameters=[nav2_params_file],
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
        name='lifecycle_manager_test_amcl',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
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
            'infer_on_capture': 'true',
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

        # t=0s: hardware, robot model and optional visualization bridge.
        chassis_launch,
        lidar_launch,
        camera_launch,
        description_launch,
        foxglove_launch,

        # t=2s: test-only N10 frequency request. Mainline lidar driver/config stay unchanged.
        TimerAction(period=2.0, actions=[test_lidar_hz_order]),

        # t=3s: odom fusion and required IMU static TFs.
        TimerAction(period=3.0, actions=[ekf_launch]),

        # t=5s: map_server + AMCL, then lifecycle manager activates both.
        TimerAction(period=5.0, actions=[
            map_server_node,
            amcl_node,
            amcl_lifecycle_manager,
        ]),

        # t=7s: camera perception and VLM/TTS.
        TimerAction(period=7.0, actions=[
            perception_detector_node,
            vlm_launch,
        ]),

        # t=9s: QR detector. It keeps publishing in test flow by default.
        TimerAction(period=9.0, actions=[qr_detector_node]),
    ])
