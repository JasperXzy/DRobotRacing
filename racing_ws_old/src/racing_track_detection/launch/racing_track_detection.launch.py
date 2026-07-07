# racing_track_detection/launch/racing_track_detection.launch.py

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
    racing_track_detection_resnet18_node = Node(
        package='racing_track_detection',
        executable='racing_track_detection',
        output='screen',
        parameters=[
            {"sub_img_topic": "/hbmem_img"},
            {"model_path": "config/race_track_detection.bin"}
        ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    return LaunchDescription([racing_track_detection_resnet18_node])
