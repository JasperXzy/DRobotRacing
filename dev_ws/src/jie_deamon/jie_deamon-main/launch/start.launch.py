"""
robot_nexus节点的ROS2启动文件
支持Web可视化和OpenCV调试模式
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_share = get_package_share_directory('jie_deamon')
    
    # 声明启动参数
    active_arg = DeclareLaunchArgument(
        'active',
        default_value='true',
        description='是否激活跟随功能'
    )
    
    enable_opencv_arg = DeclareLaunchArgument(
        'enable_opencv',
        default_value='false',
        description='是否启用OpenCV可视化(调试用)'
    )
    
    enable_web_arg = DeclareLaunchArgument(
        'enable_web',
        default_value='true',
        description='是否启用Web可视化'
    )

    playback_device_arg = DeclareLaunchArgument(
        'playback_device',
        default_value='plughw:0,0',
        description='ALSA playback device used by hobot_tts'
    )
    
    # 创建节点
    robot_nexus_node = Node(
        package='jie_deamon',
        executable='robot_nexus',
        name='robot_nexus',
        output='screen',
        parameters=[{
            'active': LaunchConfiguration('active'),
            'enable_opencv': LaunchConfiguration('enable_opencv'),
            'enable_web': LaunchConfiguration('enable_web'),
            'web_root': os.path.join(pkg_share, 'web'),
        }]
    )

    # 雷达驱动节点
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("lidar_driver"), 'launch', 'lidar.launch.py')
        )
    )

    # 底盘驱动
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('driver_bringup'), 'launch', 'drivers.launch.py')
        )
    )

    tts_launch = Node(
        package='hobot_tts',
        executable='hobot_tts',
        name='hobot_tts_node',
        output='screen',
        parameters=[{
            'playback_device': LaunchConfiguration('playback_device'),
            'topic_sub': '/tts_text',
        }]
    )

    return LaunchDescription([
        active_arg,
        enable_opencv_arg,
        enable_web_arg,
        playback_device_arg,
        robot_nexus_node,
        tts_launch,
        # lidar_launch,
        robot_launch,#已经包含雷达的lidar_launch
    ])
