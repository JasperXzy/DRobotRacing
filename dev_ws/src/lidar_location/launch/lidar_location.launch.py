from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('lidar_location'),
                'config',
                'lidar_location.yaml',
            ]),
            description='Parameters for the lidar scan-matching localizer.',
        ),
        Node(
            package='lidar_location',
            executable='lidar_location_node',
            name='lidar_location',
            output='screen',
            parameters=[params_file],
        ),
    ])
