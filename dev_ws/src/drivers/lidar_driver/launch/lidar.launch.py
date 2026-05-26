#!/usr/bin/python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    pkg_share = get_package_share_directory('lidar_driver')
    
    ws_root = os.path.abspath(os.path.join(pkg_share, '..', '..', '..', '..'))
    default_config_path = os.path.join(ws_root, 'config', 'drivers', 'lidar_driver.yaml')

    driver_dir = LaunchConfiguration('lidar_config_file')

    declare_driver_dir = DeclareLaunchArgument(
        'lidar_config_file',
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

    # Static TF: base_link -> laser
    # x=0.10, y=0.0, z=0.05
    tf_laser_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser',
        arguments=['0.1', '0.0', '0.05', '0.0', '0.0', '0.0', 'base_link', 'laser']
    )

    return LaunchDescription([
        declare_driver_dir,
        driver_node,
        #tf_laser_node,
    ])

