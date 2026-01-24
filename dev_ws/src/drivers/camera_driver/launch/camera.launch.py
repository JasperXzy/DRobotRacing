import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('camera_driver')
    
    ws_root = os.path.abspath(os.path.join(pkg_share, '..', '..', '..', '..'))
    
    default_config_path = os.path.join(ws_root, 'config', 'camera_driver.yaml')
    
    config_file_arg = DeclareLaunchArgument(
        'camera_config_file',
        default_value=default_config_path,
        description='Path to the config file'
    )

    config_file = LaunchConfiguration('camera_config_file')

    return LaunchDescription([
        config_file_arg,

        # USB Camera Driver
        Node(
            package='hobot_usb_cam',
            executable='hobot_usb_cam',
            name='usb_cam_node',
            parameters=[config_file],
            remappings=[
                ('/image', '/camera/image')
            ],
            arguments=['--ros-args', '--log-level', 'error']
        ),

        # Decoder for AI Usage
        # MJPEG -> NV12 (Shared Memory)
        Node(
            package='hobot_codec',
            executable='hobot_codec_republish',
            name='usb_cam_nv12_codec_node',
            output='screen',
            parameters=[config_file],
            arguments=['--ros-args', '--log-level', 'error']
        ),

        # Encoder for Remote Preview
        # NV12 (Shared Mem) -> JPEG (Compressed)
        Node(
            package='hobot_codec',
            executable='hobot_codec_republish',
            name='usb_cam_preview_codec_node',
            output='screen',
            parameters=[config_file],
            arguments=['--ros-args', '--log-level', 'error']
        )
    ])
