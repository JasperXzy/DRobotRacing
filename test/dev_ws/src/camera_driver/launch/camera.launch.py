import os

import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.logging import get_logger
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


_LOGGER = get_logger('camera_driver')


def _create_v4l2_control_actions(context, config_file):
    """Apply YAML exposure values with the control names exported by this camera."""
    config_path = config_file.perform(context)
    try:
        with open(config_path, 'r', encoding='utf-8') as stream:
            config = yaml.safe_load(stream) or {}
        camera_parameters = (
            config.get('usb_cam_node', {})
            .get('ros__parameters', {})
        )
        video_device = str(camera_parameters.get('video_device', '/dev/video0'))
        autoexposure = bool(camera_parameters.get('autoexposure', True))
        exposure = int(camera_parameters.get('exposure', 100))
        gain = int(camera_parameters.get('gain', -1))
    except (OSError, TypeError, ValueError, yaml.YAMLError) as error:
        _LOGGER.error(
            'Cannot load V4L2 exposure values from %s: %s'
            % (config_path, error)
        )
        return []

    exposure_mode = 3 if autoexposure else 1
    _LOGGER.info(
        'Applying camera YAML to V4L2: device=%s, autoexposure=%s, '
        'exposure=%d, gain=%d'
        % (video_device, autoexposure, exposure, gain)
    )

    actions = [
        ExecuteProcess(
            cmd=[
                'v4l2-ctl', '--device', video_device,
                '--set-ctrl=auto_exposure=%d' % exposure_mode,
            ],
            output='screen',
        )
    ]

    if not autoexposure:
        actions.append(
            TimerAction(
                period=0.15,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            'v4l2-ctl', '--device', video_device,
                            '--set-ctrl=exposure_time_absolute=%d' % exposure,
                        ],
                        output='screen',
                    )
                ],
            )
        )
        actions.append(
            TimerAction(
                period=0.25,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            'v4l2-ctl', '--device', video_device,
                            '--set-ctrl=exposure_dynamic_framerate=0',
                        ],
                        output='screen',
                    )
                ],
            )
        )

    if gain >= 0:
        actions.append(
            TimerAction(
                period=0.35,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            'v4l2-ctl', '--device', video_device,
                            '--set-ctrl=gain=%d' % gain,
                        ],
                        output='screen',
                    )
                ],
            )
        )

    actions.append(
        TimerAction(
            period=0.50,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'v4l2-ctl', '--device', video_device,
                        '--get-ctrl=auto_exposure,exposure_time_absolute,'
                        'exposure_dynamic_framerate,gain',
                    ],
                    output='screen',
                )
            ],
        )
    )
    return actions


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

    # hobot_usb_cam uses legacy exposure control aliases that this camera no
    # longer exports. Two seconds after startup, translate the same YAML values
    # to the camera's actual V4L2 control names and print the resulting values.
    apply_v4l2_controls = TimerAction(
        period=2.0,
        actions=[
            OpaqueFunction(
                function=_create_v4l2_control_actions,
                kwargs={'config_file': config_file},
            )
        ],
    )

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

        apply_v4l2_controls,

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
            arguments=['--ros-args', '--log-level', 'error'],
            condition=IfCondition(EnvironmentVariable('DEBUG_SHOW', default_value='false'))
        )
    ])
