from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('vlm_tts_demo')

    default_env_file = PathJoinSubstitution([
        pkg_share,
        'config',
        'vlm_api.env',
    ])
    default_captured_image_file = PathJoinSubstitution([
        pkg_share,
        'captures',
        'vlm_tts_demo_capture.jpg',
    ])

    playback_device = LaunchConfiguration('playback_device')
    trigger_value = LaunchConfiguration('trigger_value')
    infer_trigger_value = LaunchConfiguration('infer_trigger_value')
    infer_on_capture = LaunchConfiguration('infer_on_capture')
    camera_topic = LaunchConfiguration('camera_topic')
    launch_camera = LaunchConfiguration('launch_camera')
    launch_tts = LaunchConfiguration('launch_tts')
    captured_image_file = LaunchConfiguration('captured_image_file')
    source_capture_file = LaunchConfiguration('source_capture_file')
    ding_enable = LaunchConfiguration('ding_enable')
    no_auth = LaunchConfiguration('no_auth')

    return LaunchDescription([
        DeclareLaunchArgument(
            'playback_device',
            default_value='plughw:0,0',
            description='ALSA playback device used by hobot_tts.',
        ),
        DeclareLaunchArgument(
            'trigger_value',
            default_value='-1',
            description='Int32 value on /sign4return that captures one image.',
        ),
        DeclareLaunchArgument(
            'infer_trigger_value',
            default_value='0',
            description='Int32 value on /sign4return that calls the vision API.',
        ),
        DeclareLaunchArgument(
            'infer_on_capture',
            default_value='false',
            description='Call the vision API immediately after capture.',
        ),
        DeclareLaunchArgument(
            'camera_topic',
            default_value='/camera/image',
            description='Compressed camera image topic captured by this demo.',
        ),
        DeclareLaunchArgument(
            'captured_image_file',
            default_value=default_captured_image_file,
            description='Path used to save the captured camera frame.',
        ),
        DeclareLaunchArgument(
            'source_capture_file',
            default_value='',
            description='Optional second copy of the captured image.',
        ),
        DeclareLaunchArgument(
            'launch_camera',
            default_value='true',
            description='Whether to launch camera_driver from this demo.',
        ),
        DeclareLaunchArgument(
            'launch_tts',
            default_value='true',
            description='Whether to launch hobot_tts from this demo.',
        ),
        DeclareLaunchArgument(
            'ding_enable',
            default_value='false',
            description='Whether to publish a short TTS start notice before API inference.',
        ),
        DeclareLaunchArgument(
            'start_notice',
            default_value='开始识别',
            description='Short TTS notice published before API inference.',
        ),
        DeclareLaunchArgument(
            'error_notice',
            default_value='识别失败，请检查网络或接口配置',
            description='TTS notice used when API inference fails.',
        ),
        DeclareLaunchArgument(
            'last_success_text_file',
            default_value='/root/D_Robot/test/dev_ws/captures/last_successful_vlm_text.txt',
            description='Persistent cache of the last successful VLM speech text.',
        ),
        DeclareLaunchArgument(
            'failure_prefix',
            default_value='一',
            description='Prefix spoken before the cached VLM text after inference failure.',
        ),
        DeclareLaunchArgument(
            'user_prompt',
            default_value='',
            description='Prompt override. Empty uses the node default prompt.',
        ),
        DeclareLaunchArgument(
            'env_file',
            default_value=default_env_file,
            description='Local env file for VLM_API_ENDPOINT, VLM_API_MODEL and VLM_API_KEY.',
        ),
        DeclareLaunchArgument(
            'api_endpoint',
            default_value='',
            description='OpenAI-compatible chat-completions endpoint or base URL.',
        ),
        DeclareLaunchArgument(
            'api_model',
            default_value='',
            description='Vision-capable model name.',
        ),
        DeclareLaunchArgument(
            'api_key',
            default_value='',
            description='API key. Prefer env_file or environment variables.',
        ),
        DeclareLaunchArgument(
            'api_timeout',
            default_value='15.0',
            description='API request timeout in seconds.',
        ),
        DeclareLaunchArgument(
            'api_max_tokens',
            default_value='120',
            description='Maximum output tokens.',
        ),
        DeclareLaunchArgument(
            'api_temperature',
            default_value='0.2',
            description='API sampling temperature.',
        ),
        DeclareLaunchArgument(
            'api_image_detail',
            default_value='auto',
            description='Vision API image detail hint: low, high or auto.',
        ),
        DeclareLaunchArgument(
            'no_auth',
            default_value='false',
            description='Do not send Authorization header.',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('camera_driver'),
                    'launch',
                    'camera.launch.py',
                ])
            ),
            condition=IfCondition(launch_camera),
        ),

        Node(
            package='hobot_tts',
            executable='hobot_tts',
            name='hobot_tts_node',
            output='screen',
            respawn=True,
            respawn_delay=3.0,
            parameters=[{
                'playback_device': playback_device,
                'topic_sub': '/tts_text',
            }],
            condition=IfCondition(launch_tts),
        ),

        Node(
            package='vlm_tts_demo',
            executable='vlm_tts_demo_node',
            name='vlm_tts_demo_node',
            output='screen',
            parameters=[{
                'trigger_value': ParameterValue(trigger_value, value_type=int),
                'infer_trigger_value': ParameterValue(infer_trigger_value, value_type=int),
                'infer_on_capture': ParameterValue(infer_on_capture, value_type=bool),
                'camera_topic': camera_topic,
                'captured_image_file': captured_image_file,
                'source_capture_file': source_capture_file,
                'ding_enable': ParameterValue(ding_enable, value_type=bool),
                'start_notice': LaunchConfiguration('start_notice'),
                'error_notice': LaunchConfiguration('error_notice'),
                'last_success_text_file': LaunchConfiguration('last_success_text_file'),
                'failure_prefix': LaunchConfiguration('failure_prefix'),
                'user_prompt': LaunchConfiguration('user_prompt'),
                'env_file': LaunchConfiguration('env_file'),
                'api_endpoint': LaunchConfiguration('api_endpoint'),
                'api_model': LaunchConfiguration('api_model'),
                'api_key': LaunchConfiguration('api_key'),
                'api_timeout': ParameterValue(
                    LaunchConfiguration('api_timeout'), value_type=float),
                'api_max_tokens': ParameterValue(
                    LaunchConfiguration('api_max_tokens'), value_type=int),
                'api_temperature': ParameterValue(
                    LaunchConfiguration('api_temperature'), value_type=float),
                'api_image_detail': LaunchConfiguration('api_image_detail'),
                'no_auth': ParameterValue(no_auth, value_type=bool),
            }],
        ),
    ])
