from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution(
        [FindPackageShare('bmi088_driver'), 'config', 'bmi088.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='bmi088',
            description='Namespace that keeps this IMU independent from the chassis MPU6050.'),
        DeclareLaunchArgument(
            'config_file', default_value=default_config,
            description='BMI088 parameter file.'),
        DeclareLaunchArgument(
            'accel_device', default_value='/dev/spidev0.0',
            description='SPI device connected to BMI088 CS1 (accelerometer).'),
        DeclareLaunchArgument(
            'gyro_device', default_value='/dev/spidev0.1',
            description='SPI device connected to BMI088 CS2 (gyroscope).'),
        DeclareLaunchArgument(
            'mock_mode', default_value='false',
            description='Publish stationary test data without opening SPI devices.'),
        Node(
            package='bmi088_driver',
            executable='bmi088_node',
            name='bmi088_driver',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            emulate_tty=True,
            parameters=[
                LaunchConfiguration('config_file'),
                {
                    'accel_device': LaunchConfiguration('accel_device'),
                    'gyro_device': LaunchConfiguration('gyro_device'),
                    'mock_mode': ParameterValue(
                        LaunchConfiguration('mock_mode'), value_type=bool),
                },
            ],
        ),
    ])
