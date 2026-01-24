#!/usr/bin/python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    pkg_share = get_package_share_directory('lidar_driver')
    
    ws_root = os.path.abspath(os.path.join(pkg_share, '..', '..', '..', '..'))
    default_config_path = os.path.join(ws_root, 'config', 'lidar_driver.yaml')

    driver_dir = LaunchConfiguration('config_file')

    declare_driver_dir = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_path,
        description='Path to the config file'
    )
                     
    driver_node = LifecycleNode(package='lidar_driver',
                                executable='lidar_driver_node',
                                name='lidar_driver_node',
                                output='screen',
                                emulate_tty=True,
                                namespace='',
                                parameters=[driver_dir],
                                )

    return LaunchDescription([
        declare_driver_dir,
        driver_node,
    ])

