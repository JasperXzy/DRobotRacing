import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ws_root = os.path.abspath(os.path.join(
        get_package_share_directory('perception_detector'),
        '..', '..', '..', '..'))
    params_file = os.path.join(
        ws_root, 'config', 'perception', 'perception_detector.yaml')

    return LaunchDescription([
        Node(
            package='perception_detector',
            executable='perception_detector_node',
            name='perception_detector_node',
            output='screen',
            parameters=[params_file],
            arguments=['--ros-args', '--log-level', 'info'],
        )
    ])
