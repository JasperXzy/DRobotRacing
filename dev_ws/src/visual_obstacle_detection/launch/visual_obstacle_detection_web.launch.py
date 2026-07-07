# visual_obstacle_detection/launch/obstacle_detection.launch.py

import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    launch_actions = []
    
    obstacle_detection_node = Node(
        package='visual_obstacle_detection',
        executable='visual_obstacle_detection',
        name='visual_obstacle_detection',
        output='screen',
        parameters=[
            {"image_topic": "/camera/image_hbmem"},
        ],
        remappings=[
        ]
    )
    launch_actions.append(obstacle_detection_node)

    websocket_node = Node(
        package='websocket',
        executable='websocket',
        name='websocket',
        output='screen',
        parameters=[
            {"image_topic": "/image"},
            {"image_type": "mjpeg"},
            {"only_show_image": False},
            {"output_fps": 0},
            {"smart_topic": "/visual_obstacle_detection"},
            {"channel": 0}
        ],
        arguments=['--ros-args', '--log-level', 'info']
    )
    launch_actions.append(websocket_node)

    return LaunchDescription(launch_actions)
