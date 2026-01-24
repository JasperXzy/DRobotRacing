from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    depth_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('depth_camera_driver'),
                'launch',
                'aurora930_launch.py'
            ])
        )
    )

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('camera_driver'),
                'launch',
                'camera.launch.py'
            ])
        )
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('lidar_driver'),
                'launch',
                'lidar.launch.py'
            ])
        )
    )

    return LaunchDescription([
        depth_camera_launch,
        camera_launch,
        lidar_launch,
    ])
