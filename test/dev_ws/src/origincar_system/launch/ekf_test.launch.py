from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config_file = PathJoinSubstitution([
        FindPackageShare('origincar_system'),
        'config',
        'localization',
        'ekf.yaml',
    ])

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true',
    )

    ekf_config_file_arg = DeclareLaunchArgument(
        'ekf_config_file',
        default_value=default_config_file,
        description='EKF config file used by the test flow',
    )

    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value='/bmi088/imu/data',
        description=(
            'Calibrated IMU topic fused by the EKF. Set '
            '/chassis/imu/data_raw to temporarily restore the MPU6050.'
        ),
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            LaunchConfiguration('ekf_config_file'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'imu0': LaunchConfiguration('imu_topic'),
            },
        ],
        remappings=[
            ('odometry/filtered', '/odometry/filtered'),
        ],
        arguments=['--ros-args', '--log-level', 'warn'],
    )

    # Keep the same static TFs as the mainline state_estimation ekf.launch.py.
    imu_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_imu_link',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0.05',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'imu_link',
            '--ros-args', '--log-level', 'warn',
        ],
    )

    gyro_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_link_to_gyro_link',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', 'imu_link',
            '--child-frame-id', 'gyro_link',
            '--ros-args', '--log-level', 'warn',
        ],
    )

    # The BMI088 board is mounted with its printed X/Y/Z axes aligned with the
    # chassis axes. Keep the MPU6050 TFs above so its independent comparison
    # topic remains usable.
    bmi088_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_bmi088_link',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0.05',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'bmi088_link',
            '--ros-args', '--log-level', 'warn',
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        ekf_config_file_arg,
        imu_topic_arg,
        ekf_node,
        imu_static_tf,
        gyro_static_tf,
        bmi088_static_tf,
    ])
