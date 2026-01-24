import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_share = get_package_share_directory('depth_camera_driver')
    
    ws_root = os.path.abspath(os.path.join(pkg_share, '..', '..', '..', '..'))
    
    default_params_file = os.path.join(ws_root, 'config', 'depth_camera_driver.yaml')
    
    params_file_launch_arg = DeclareLaunchArgument(
        'depth_camera_config_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file to use for the aurora node'
    )

    return LaunchDescription([
        params_file_launch_arg,
        Node(
            package="depth_camera_driver",
            executable="aurora930_node",
            namespace="depth_camera",
            name="depth_camera_node",
            parameters=[
                LaunchConfiguration('depth_camera_config_file'),
                {
                    "rgb_enable": LaunchConfiguration('rgb_enable', default=True),
                    "ir_enable": LaunchConfiguration('ir_enable', default=True),
                    "depth_enable": LaunchConfiguration('depth_enable', default=True),
                    "resolution_mode_index": LaunchConfiguration('resolution_mode_index', default=2),
                }
            ],
            output="screen",   
        )
    ])