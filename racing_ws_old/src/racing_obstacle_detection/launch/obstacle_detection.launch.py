# racing_obstacle_detection/launch/obstacle_detection.launch.py

import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    launch_actions = []
    
    obstacle_detection_node = Node(
        package='racing_obstacle_detection',
        executable='racing_obstacle_detection',
        name='racing_obstacle_detection',
        output='screen',
        parameters=[
        ],
        remappings=[
        ]
    )
    launch_actions.append(obstacle_detection_node)

    return LaunchDescription(launch_actions)
