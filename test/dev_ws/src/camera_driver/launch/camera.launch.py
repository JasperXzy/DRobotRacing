import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('camera_driver')
    
    ws_root = os.path.abspath(os.path.join(pkg_share, '..', '..', '..', '..'))
    
    default_config_path = os.path.join(ws_root, 'config', 'drivers', 'camera_driver.yaml')
    
    config_file_arg = DeclareLaunchArgument(
        'camera_config_file',
        default_value=default_config_path,
        description='Path to the config file'
    )

    config_file = LaunchConfiguration('camera_config_file')

    # HbmMsg1080P contains a fixed 6.2 MB image buffer.  Load the vendor
    # Fast DDS profile before starting any image process so loaned messages
    # use data-sharing instead of being serialized and copied through DDS.
    shared_memory_transport = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hobot_shm'),
                'launch',
                'hobot_shm.launch.py',
            )
        )
    )

    return LaunchDescription([
        config_file_arg,
        shared_memory_transport,

        # USB Camera Driver
        Node(
            package='hobot_usb_cam',
            executable='hobot_usb_cam',
            name='usb_cam_node',
            parameters=[config_file],
            remappings=[
                ('/hbmem_img', '/camera/image_mjpeg_hbmem')
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

        # Compatibility/preview stream for QR snapshots, VLM and Foxglove.
        # The high-rate YOLO path above stays entirely in shared memory.
        Node(
            package='hobot_codec',
            executable='hobot_codec_republish',
            name='usb_cam_preview_codec_node',
            output='screen',
            parameters=[config_file],
            arguments=['--ros-args', '--log-level', 'error']
        )
    ])
