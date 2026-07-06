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

    default_vlm_params_file = PathJoinSubstitution([
        pkg_share,
        'config',
        'vlm_params.yaml',
    ])
    default_captured_image_file = PathJoinSubstitution([
        pkg_share,
        'captures',
        'vlm_tts_demo_capture.jpg',
    ])
    default_vit_model_file = PathJoinSubstitution([
        pkg_share,
        'model',
        'InternVL3_1B',
        'vit_model_int16_VL3_1B_Instruct.hbm',
    ])
    default_llm_model_file = PathJoinSubstitution([
        pkg_share,
        'model',
        'InternVL3_1B',
        'qwen2_5_q8_0_InternVL3_1B_Instruct.gguf',
    ])

    vlm_params_file = LaunchConfiguration('vlm_params_file')
    playback_device = LaunchConfiguration('playback_device')
    trigger_value = LaunchConfiguration('trigger_value')
    infer_trigger_value = LaunchConfiguration('infer_trigger_value')
    user_prompt = LaunchConfiguration('user_prompt')
    cute_words = LaunchConfiguration('cute_words')
    llamacpp_text_topic = LaunchConfiguration('llamacpp_text_topic')
    camera_topic = LaunchConfiguration('camera_topic')
    launch_camera = LaunchConfiguration('launch_camera')
    captured_image_file = LaunchConfiguration('captured_image_file')
    source_capture_file = LaunchConfiguration('source_capture_file')
    ding_enable = LaunchConfiguration('ding_enable')

    return LaunchDescription([
        DeclareLaunchArgument(
            'vlm_params_file',
            default_value=default_vlm_params_file,
            description='Path to hobot_llamacpp parameter file.',
        ),
        DeclareLaunchArgument(
            'playback_device',
            default_value='plughw:0,0',
            description='ALSA playback device used by hobot_tts.',
        ),
        DeclareLaunchArgument(
            'trigger_value',
            default_value='-1',
            description='Int32 value on /sign4return that triggers VLM capture.',
        ),
        DeclareLaunchArgument(
            'infer_trigger_value',
            default_value='0',
            description='Int32 value on /sign4return that triggers VLM inference.',
        ),
        DeclareLaunchArgument(
            'user_prompt',
            default_value='请描述画面中彩色矩形展示板上的医疗插画内容，重点说出患者、病床、输液或护理信息。',
            description='Prompt sent to hobot_llamacpp when triggered.',
        ),
        DeclareLaunchArgument(
            'cute_words',
            default_value='',
            description='Warm-up speech from hobot_llamacpp. Empty disables it.',
        ),
        DeclareLaunchArgument(
            'llamacpp_text_topic',
            default_value='/tts_text',
            description='hobot_llamacpp text topic consumed by hobot_tts.',
        ),
        DeclareLaunchArgument(
            'camera_topic',
            default_value='/camera/image',
            description='Compressed camera image topic captured by this demo.',
        ),
        DeclareLaunchArgument(
            'captured_image_file',
            default_value=default_captured_image_file,
            description='Path used to save one camera frame for hobot_llamacpp.',
        ),
        DeclareLaunchArgument(
            'source_capture_file',
            default_value='/root/D_Robot/dev_ws/src/drivers/vlm_tts_demo/captures/vlm_tts_demo_capture.jpg',
            description='Optional source-tree copy of the captured image for inspection.',
        ),
        DeclareLaunchArgument(
            'launch_camera',
            default_value='true',
            description='Whether to launch camera_driver from this demo.',
        ),
        DeclareLaunchArgument(
            'ding_enable',
            default_value='true',
            description='Whether to publish a short TTS start notice after capturing an image.',
        ),
        DeclareLaunchArgument(
            'start_notice',
            default_value='开始识别',
            description='Short TTS notice published before VLM starts.',
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
        ),

        Node(
            package='vlm_tts_demo',
            executable='vlm_tts_demo_node',
            name='vlm_tts_demo_node',
            output='screen',
            parameters=[{
                'trigger_value': ParameterValue(trigger_value, value_type=int),
                'infer_trigger_value': ParameterValue(infer_trigger_value, value_type=int),
                'user_prompt': user_prompt,
                'playback_device': playback_device,
                'vlm_params_file': vlm_params_file,
                'camera_topic': camera_topic,
                'captured_image_file': captured_image_file,
                'source_capture_file': source_capture_file,
                'ding_enable': ParameterValue(ding_enable, value_type=bool),
                'start_notice': LaunchConfiguration('start_notice'),
                'model_file_name': default_vit_model_file,
                'llm_model_name': default_llm_model_file,
                'cute_words': cute_words,
                'llamacpp_text_topic': llamacpp_text_topic,
                'forward_vlm_output': False,
            }],
        ),
    ])
