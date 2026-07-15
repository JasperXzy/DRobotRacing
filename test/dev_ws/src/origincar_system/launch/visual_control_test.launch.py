from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'pub_control_topic',
            default_value='/cmd_vel',
            description=(
                'Real chassis cmd_vel topic. Visual control is suspended at QR; '
                'the waypoint follower owns post-QR obstacle avoidance.'
            ),
        ),
        DeclareLaunchArgument(
            'avoid_angular_ratio',
            default_value='0.2',
            description='Angular speed ratio for obstacle avoidance.',
        ),
        DeclareLaunchArgument(
            'avoid_linear_speed',
            default_value='0.1',
            description='Linear speed for obstacle avoidance.',
        ),
        DeclareLaunchArgument(
            'recovering_linear_speed',
            default_value='0.7',
            description='Recovering speed after obstacle avoidance.',
        ),
        DeclareLaunchArgument(
            'recovering_angular_ratio',
            default_value='0.8',
            description='Recovering angular ratio after obstacle avoidance.',
        ),
        DeclareLaunchArgument(
            'qr_code_confidence_threshold',
            default_value='0.8',
            description='QR confidence threshold for line following.',
        ),
        DeclareLaunchArgument(
            'qr_code_hight_follow_threshold',
            default_value='500',
            description='QR height threshold for QR following.',
        ),
        DeclareLaunchArgument(
            'qr_code_height_threshold',
            default_value='54',
            description='QR height threshold for speed reduction.',
        ),
        DeclareLaunchArgument(
            'follow_angular_ratio',
            default_value='-1.0',
            description='Angular speed ratio for line following.',
        ),
        DeclareLaunchArgument(
            'follow_linear_speed',
            default_value='0.1',
            description='Linear speed for line following.',
        ),
        DeclareLaunchArgument(
            'bottom_threshold_caution',
            default_value='280',
            description='Bottom threshold for caution zone.',
        ),
        DeclareLaunchArgument(
            'bottom_threshold_avoid',
            default_value='320',
            description='Bottom threshold for avoid zone.',
        ),
        DeclareLaunchArgument(
            'parking_y_threshold',
            default_value='300',
            description='Y-pixel threshold for parking sign bottom edge to trigger stop.',
        ),
        DeclareLaunchArgument(
            'line_confidence_threshold',
            default_value='0.5',
            description='Line confidence threshold.',
        ),
        DeclareLaunchArgument(
            'line_center_threshold_x',
            default_value='10',
            description='Line center threshold on x axis.',
        ),
        DeclareLaunchArgument(
            'obstacle_confidence_threshold',
            default_value='0.5',
            description='Obstacle confidence threshold.',
        ),
        DeclareLaunchArgument(
            'obstacle_y_jump_threshold',
            default_value='10',
            description='Obstacle y jump threshold.',
        ),
        DeclareLaunchArgument(
            'delta_x_ratio',
            default_value='0.05',
            description='Obstacle x offset ratio.',
        ),
        DeclareLaunchArgument(
            'parking_sign_confidence_threshold',
            default_value='0.6',
            description='Parking sign confidence threshold.',
        ),
        DeclareLaunchArgument(
            'qr_code_speed_modifier',
            default_value='0.2',
            description='Speed modifier when QR is close.',
        ),
        DeclareLaunchArgument(
            'follow_qr_code_ratio',
            default_value='-4.0',
            description='Angular ratio for QR following.',
        ),
        DeclareLaunchArgument(
            'follow_parking_sign_ratio',
            default_value='-4.0',
            description='Angular ratio for parking sign following.',
        ),
        DeclareLaunchArgument(
            'debug_control_logs',
            default_value='false',
            description='Enable detailed visual_control debug logs.',
        ),
        DeclareLaunchArgument(
            'clear_detection_cache_on_resume',
            default_value='true',
            description='Clear cached line/target detections when visual_control resumes from /sign4return=6.',
        ),
        DeclareLaunchArgument(
            'drive_straight_without_initial_line',
            default_value='true',
            description='Drive straight with follow_linear_speed until the first fresh line message arrives.',
        ),
        DeclareLaunchArgument(
            'enable_parking_before_path',
            default_value='false',
            description='Ignore parking signs before the QR path handoff.',
        ),
        DeclareLaunchArgument(
            'enable_qr_after_path',
            default_value='false',
            description='Ignore QR targets after visual_control resumes from path following.',
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='warn',
            description='ROS log level for visual_control.',
        ),
        Node(
            package='visual_control',
            executable='visual_control',
            output='screen',
            parameters=[
                {'pub_control_topic': LaunchConfiguration('pub_control_topic')},
                {'avoid_angular_ratio': ParameterValue(LaunchConfiguration('avoid_angular_ratio'), value_type=float)},
                {'avoid_linear_speed': ParameterValue(LaunchConfiguration('avoid_linear_speed'), value_type=float)},
                {'recovering_linear_speed': ParameterValue(LaunchConfiguration('recovering_linear_speed'), value_type=float)},
                {'recovering_angular_ratio': ParameterValue(LaunchConfiguration('recovering_angular_ratio'), value_type=float)},
                {'qr_code_confidence_threshold': ParameterValue(LaunchConfiguration('qr_code_confidence_threshold'), value_type=float)},
                {'qr_code_hight_follow_threshold': ParameterValue(LaunchConfiguration('qr_code_hight_follow_threshold'), value_type=int)},
                {'qr_code_height_threshold': ParameterValue(LaunchConfiguration('qr_code_height_threshold'), value_type=int)},
                {'follow_angular_ratio': ParameterValue(LaunchConfiguration('follow_angular_ratio'), value_type=float)},
                {'follow_linear_speed': ParameterValue(LaunchConfiguration('follow_linear_speed'), value_type=float)},
                {'bottom_threshold_caution': ParameterValue(LaunchConfiguration('bottom_threshold_caution'), value_type=int)},
                {'bottom_threshold_avoid': ParameterValue(LaunchConfiguration('bottom_threshold_avoid'), value_type=int)},
                {'parking_y_threshold': ParameterValue(LaunchConfiguration('parking_y_threshold'), value_type=int)},
                {'line_confidence_threshold': ParameterValue(LaunchConfiguration('line_confidence_threshold'), value_type=float)},
                {'line_center_threshold_x': ParameterValue(LaunchConfiguration('line_center_threshold_x'), value_type=int)},
                {'obstacle_confidence_threshold': ParameterValue(LaunchConfiguration('obstacle_confidence_threshold'), value_type=float)},
                {'obstacle_y_jump_threshold': ParameterValue(LaunchConfiguration('obstacle_y_jump_threshold'), value_type=int)},
                {'delta_x_ratio': ParameterValue(LaunchConfiguration('delta_x_ratio'), value_type=float)},
                {'parking_sign_confidence_threshold': ParameterValue(LaunchConfiguration('parking_sign_confidence_threshold'), value_type=float)},
                {'qr_code_speed_modifier': ParameterValue(LaunchConfiguration('qr_code_speed_modifier'), value_type=float)},
                {'follow_qr_code_ratio': ParameterValue(LaunchConfiguration('follow_qr_code_ratio'), value_type=float)},
                {'follow_parking_sign_ratio': ParameterValue(LaunchConfiguration('follow_parking_sign_ratio'), value_type=float)},
                {'debug_control_logs': ParameterValue(LaunchConfiguration('debug_control_logs'), value_type=bool)},
                {'clear_detection_cache_on_resume': ParameterValue(LaunchConfiguration('clear_detection_cache_on_resume'), value_type=bool)},
                {'drive_straight_without_initial_line': ParameterValue(LaunchConfiguration('drive_straight_without_initial_line'), value_type=bool)},
                {'enable_parking_before_path': ParameterValue(LaunchConfiguration('enable_parking_before_path'), value_type=bool)},
                {'enable_qr_after_path': ParameterValue(LaunchConfiguration('enable_qr_after_path'), value_type=bool)},
            ],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        ),
    ])
