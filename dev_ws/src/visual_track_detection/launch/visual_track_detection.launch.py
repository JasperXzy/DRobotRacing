# visual_track_detection/launch/visual_track_detection.launch.py

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory


def generate_launch_description():
    visual_track_detection_resnet18_node = Node(
        package='visual_track_detection',
        executable='visual_track_detection',
        output='screen',
        parameters=[
            {"sub_img_topic": "/camera/image_hbmem"},
            {"model_path": "/root/D_Robot/model/race_track_detection.hbm"}
        ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    return LaunchDescription([visual_track_detection_resnet18_node])
