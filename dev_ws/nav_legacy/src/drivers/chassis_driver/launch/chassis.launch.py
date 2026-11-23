#!/usr/bin/python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_share = get_package_share_directory('chassis_driver')
    ws_root = os.path.abspath(os.path.join(pkg_share, '..', '..', '..', '..'))
    default_config_path = os.path.join(ws_root, 'config', 'drivers', 'chassis_driver.yaml')

    declare_config_file = DeclareLaunchArgument(
        'chassis_config_file',
        default_value=default_config_path,
        description='Path to the chassis config file'
    )

    config_file = LaunchConfiguration('chassis_config_file')
                     
    chassis_node = Node(
        package='chassis_driver',
        executable='chassis_driver_node',
        name='chassis_driver_node',
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([
        declare_config_file,
        chassis_node,
    ])
