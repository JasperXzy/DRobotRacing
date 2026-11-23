import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('state_estimation')
    
    workspace_dir = os.path.join(pkg_share, '..', '..', '..', '..')
    config_file = os.path.join(workspace_dir, 'config', 'localization', 'ekf.yaml')
    
    # Normalize path
    config_file = os.path.normpath(config_file)
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )
    
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('odometry/filtered', '/odometry/filtered'),
        ]
    )
    
    # Static TF: base_link -> imu_link
    # IMU is assumed to be at the center of the robot, slightly above base_link
    # New-style arguments: --x --y --z --roll --pitch --yaw --frame-id --child-frame-id
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
            '--child-frame-id', 'imu_link'
        ]
    )
    
    # Static TF: imu_link -> gyro_link
    # The chassis_driver publishes IMU data with frame_id='gyro_link'
    # gyro_link is at the same position as imu_link (identity transform)
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
            '--child-frame-id', 'gyro_link'
        ]
    )
    
    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        
        # Nodes
        ekf_node,
        imu_static_tf,
        gyro_static_tf,
    ])
