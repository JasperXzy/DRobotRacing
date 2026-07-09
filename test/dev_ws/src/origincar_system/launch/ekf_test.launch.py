import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config_file = os.path.normpath(
        '/root/D_Robot/dev_ws/config/localization/ekf_test.yaml')

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

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            LaunchConfiguration('ekf_config_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        remappings=[
            ('odometry/filtered', '/odometry/filtered'),
        ],
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
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        ekf_config_file_arg,
        ekf_node,
        imu_static_tf,
        gyro_static_tf,
    ])
