import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch.conditions import IfCondition

def generate_launch_description():
    pkg_share = get_package_share_directory('depth_camera_driver')
    
    ws_root = os.path.abspath(os.path.join(pkg_share, '..', '..', '..', '..'))
    
    default_params_file = os.path.join(ws_root, 'config', 'drivers', 'depth_camera_driver.yaml')
    
    params_file_launch_arg = DeclareLaunchArgument(
        'depth_camera_config_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file to use for the aurora node'
    )

    # Launch argument to control RGB to NV12 conversion
    rgb_hbmem_enable_arg = DeclareLaunchArgument(
        'rgb_hbmem_enable',
        default_value='false',
        description='Enable RGB to NV12 shared memory conversion'
    )

    config_file = LaunchConfiguration('depth_camera_config_file')

    return LaunchDescription([
        params_file_launch_arg,
        rgb_hbmem_enable_arg,
        
        # Depth Camera Node
        Node(
            package="depth_camera_driver",
            executable="aurora930_node",
            namespace="depth_camera",
            name="depth_camera_node",
            parameters=[
                config_file,
                {
                    "rgb_enable": LaunchConfiguration('rgb_enable', default='true'),
                    "ir_enable": LaunchConfiguration('ir_enable', default='false'),
                    "depth_enable": LaunchConfiguration('depth_enable', default='true'),
                    "point_cloud_enable": LaunchConfiguration('point_cloud_enable', default=EnvironmentVariable('DEBUG_SHOW', default_value='false')),
                    "resolution_mode_index": LaunchConfiguration('resolution_mode_index', default='2'),
                }
            ],
            output="screen",   
        ),

        # RGB to NV12 Codec Node
        Node(
            package='hobot_codec',
            executable='hobot_codec_republish',
            name='depth_camera_rgb_nv12_codec_node',
            output='screen',
            parameters=[config_file],
            arguments=['--ros-args', '--log-level', 'error'],
            condition=IfCondition(LaunchConfiguration('rgb_hbmem_enable'))
        )
    ])