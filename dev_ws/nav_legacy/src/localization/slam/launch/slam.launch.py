import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode

def generate_launch_description():
    slam_pkg_share = get_package_share_directory('slam')
    
    ws_root = os.path.abspath(os.path.join(slam_pkg_share, '..', '..', '..', '..'))
    slam_config_path = os.path.join(ws_root, 'config', 'localization', 'slam_toolbox.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')

    # Arguments
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock')

    declare_slam_params_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=slam_config_path,
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    # SLAM Toolbox Node
    start_async_slam_toolbox_node = LifecycleNode(
        parameters=[
          slam_params_file,
          {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        namespace=''
    )

    return LaunchDescription([
        declare_use_sim_time_arg,
        declare_slam_params_arg,
        start_async_slam_toolbox_node
    ])