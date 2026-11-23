import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ws_root = os.path.abspath(os.path.join(
        get_package_share_directory('line_detector'),
        '..', '..', '..', '..'))
    params_file = os.path.join(ws_root, 'config', 'perception', 'line_detector.yaml')

    return LaunchDescription([
        Node(
            package='line_detector',
            executable='line_detector_node',
            name='GetLineCoordinate',
            output='screen',
            parameters=[params_file],
            arguments=['--ros-args', '--log-level', 'info'],
        )
    ])
