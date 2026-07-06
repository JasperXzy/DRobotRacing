import os
import threading
import subprocess

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32, String


class VlmTtsDemoNode(Node):
    """Capture one camera frame on trigger, then run hobot_llamacpp on that file."""

    def __init__(self):
        super().__init__('vlm_tts_demo_node')

        self.declare_parameter('trigger_topic', '/sign4return')
        self.declare_parameter('trigger_value', -1)
        self.declare_parameter('infer_trigger_value', 0)
        self.declare_parameter('prompt_topic', '/prompt_text')
        self.declare_parameter('vlm_output_topic', '/llama_cpp_node')
        self.declare_parameter('tts_topic', '/tts_text')
        self.declare_parameter('user_prompt', '描述这张图片。')
        self.declare_parameter('playback_device', 'plughw:0,0')
        self.declare_parameter('use_cached_result', False)
        self.declare_parameter('vlm_params_file', '')
        self.declare_parameter('camera_topic', '/camera/image')
        self.declare_parameter('captured_image_file', '/tmp/vlm_tts_demo_capture.jpg')
        self.declare_parameter('source_capture_file', '')
        self.declare_parameter('ding_enable', True)
        self.declare_parameter('start_notice', '开始识别')
        self.declare_parameter('model_file_name', '')
        self.declare_parameter('llm_model_name', '')
        self.declare_parameter('cute_words', '')
        self.declare_parameter('llamacpp_text_topic', '/tts_text')
        self.declare_parameter('forward_vlm_output', False)
        self.declare_parameter('llm_threads', 2)

        self.trigger_topic = self.get_parameter(
            'trigger_topic').get_parameter_value().string_value
        self.trigger_value = self.get_parameter(
            'trigger_value').get_parameter_value().integer_value
        self.infer_trigger_value = self.get_parameter(
            'infer_trigger_value').get_parameter_value().integer_value
        self.prompt_topic = self.get_parameter(
            'prompt_topic').get_parameter_value().string_value
        self.vlm_output_topic = self.get_parameter(
            'vlm_output_topic').get_parameter_value().string_value
        self.tts_topic = self.get_parameter(
            'tts_topic').get_parameter_value().string_value
        self.user_prompt = self.get_parameter(
            'user_prompt').get_parameter_value().string_value
        self.playback_device = self.get_parameter(
            'playback_device').get_parameter_value().string_value
        self.use_cached_result = self.get_parameter(
            'use_cached_result').get_parameter_value().bool_value
        self.vlm_params_file = self.get_parameter(
            'vlm_params_file').get_parameter_value().string_value
        self.camera_topic = self.get_parameter(
            'camera_topic').get_parameter_value().string_value
        self.captured_image_file = self.get_parameter(
            'captured_image_file').get_parameter_value().string_value
        self.source_capture_file = self.get_parameter(
            'source_capture_file').get_parameter_value().string_value
        self.ding_enable = self.get_parameter(
            'ding_enable').get_parameter_value().bool_value
        self.start_notice = self.get_parameter(
            'start_notice').get_parameter_value().string_value
        self.model_file_name = self.get_parameter(
            'model_file_name').get_parameter_value().string_value
        self.llm_model_name = self.get_parameter(
            'llm_model_name').get_parameter_value().string_value
        self.cute_words = self.get_parameter(
            'cute_words').get_parameter_value().string_value
        self.llamacpp_text_topic = self.get_parameter(
            'llamacpp_text_topic').get_parameter_value().string_value
        self.forward_vlm_output = self.get_parameter(
            'forward_vlm_output').get_parameter_value().bool_value
        self.llm_threads = self.get_parameter(
            'llm_threads').get_parameter_value().integer_value

        self._lock = threading.Lock()
        self._waiting_for_vlm = False
        self._waiting_for_image = False
        self._latest_image_data = None
        self._cached_vlm_text = ''
        self._last_spoken_text = ''
        self._vlm_process = None

        self.prompt_pub = self.create_publisher(String, self.prompt_topic, 10)
        self.tts_pub = self.create_publisher(String, self.tts_topic, 10)
        self.create_subscription(Int32, self.trigger_topic, self._on_trigger, 10)
        self.create_subscription(
            CompressedImage,
            self.camera_topic,
            self._on_camera_image,
            10,
        )
        self.create_subscription(String, self.vlm_output_topic, self._on_vlm_text, 10)

        self.get_logger().info(
            'VLM TTS demo ready: trigger %s == %d, camera %s, TTS %s'
            % (self.trigger_topic, self.trigger_value, self.camera_topic, self.tts_topic)
        )

    def _on_trigger(self, msg):
        # ★ 两步触发：拍照 ≠ 推理 ★
        if msg.data == self.trigger_value:
            # 第一步：只拍照，不推理
            if self._save_latest_image():
                self._publish_start_notice()
                self.get_logger().info(
                    'Capture trigger received, camera frame saved (VLM pending).')
                return
            # 还没收到图像帧，排队等
            with self._lock:
                self._waiting_for_image = True
            self.get_logger().info('Capture trigger received, waiting for one camera frame.')
            return

        if msg.data == self.infer_trigger_value:
            # 第二步：对已保存的图片启动推理
            self._publish_start_notice()
            self._waiting_for_vlm = True
            self._ensure_vlm_process()
            self.get_logger().info(
                'Infer trigger received, VLM process started.')
            return

        self.get_logger().debug(
            'Ignore trigger value %d' % msg.data)

    def _on_camera_image(self, msg):
        with self._lock:
            self._latest_image_data = bytes(msg.data)
            should_capture = self._waiting_for_image
            if should_capture:
                self._waiting_for_image = False

        if should_capture and self._save_latest_image():
            self._publish_start_notice()
            self.get_logger().info(
                'Camera frame saved (VLM pending).')

    def _on_vlm_text(self, msg):
        text = msg.data.strip()
        if not text:
            return

        should_speak = False
        with self._lock:
            self._cached_vlm_text = text
            if self._waiting_for_vlm:
                self._waiting_for_vlm = False
                should_speak = self.forward_vlm_output

        if should_speak:
            self.get_logger().info('VLM result received, forwarding to TTS.')
            self._publish_tts(text)
        else:
            self.get_logger().info('Cached VLM result from %s.' % self.vlm_output_topic)

    def _publish_tts(self, text):
        if not text:
            return

        tts_msg = String()
        tts_msg.data = text
        self.tts_pub.publish(tts_msg)
        with self._lock:
            self._last_spoken_text = text
        self.get_logger().info('Published TTS text to %s.' % self.tts_topic)

    def _ensure_vlm_process(self):
        with self._lock:
            if self._vlm_process is not None and self._vlm_process.poll() is None:
                self._vlm_process.terminate()
                try:
                    self._vlm_process.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    self._vlm_process.kill()
                    self._vlm_process.wait(timeout=2.0)

            command = [
                'ros2',
                'run',
                'hobot_llamacpp',
                'hobot_llamacpp',
                '--ros-args',
            ]
            if self.vlm_params_file:
                command.extend(['--params-file', self.vlm_params_file])
            command.extend(['-p', 'feed_type:=0'])
            command.extend(['-p', 'image:=%s' % self.captured_image_file])
            if self.model_file_name:
                command.extend(['-p', 'model_file_name:=%s' % self.model_file_name])
            if self.llm_model_name:
                command.extend(['-p', 'llm_model_name:=%s' % self.llm_model_name])
            if self.user_prompt:
                command.extend(['-p', 'user_prompt:=%s' % self.user_prompt])
            if self.cute_words:
                command.extend(['-p', 'cute_words:=%s' % self.cute_words])
            command.extend(['-p', 'llm_threads:=%d' % self.llm_threads])
            command.extend([
                '-p',
                'text_msg_pub_topic_name:=%s' % self.llamacpp_text_topic,
            ])

            self.get_logger().info('Starting hobot_llamacpp on trigger.')
            self._vlm_process = subprocess.Popen(command)

    def _save_latest_image(self):
        with self._lock:
            image_data = self._latest_image_data

        if not image_data:
            return False

        image_dir = os.path.dirname(self.captured_image_file)
        if image_dir:
            os.makedirs(image_dir, exist_ok=True)

        with open(self.captured_image_file, 'wb') as image_file:
            image_file.write(image_data)

        self.get_logger().info('Saved camera frame to %s.' % self.captured_image_file)
        if self.source_capture_file:
            self._write_image_copy(self.source_capture_file, image_data)
        return True

    def _write_image_copy(self, path, image_data):
        image_dir = os.path.dirname(path)
        if image_dir:
            os.makedirs(image_dir, exist_ok=True)

        with open(path, 'wb') as image_file:
            image_file.write(image_data)

        self.get_logger().info('Saved source copy to %s.' % path)

    def _publish_start_notice(self):
        if not self.ding_enable or not self.start_notice:
            return

        notice_msg = String()
        notice_msg.data = self.start_notice
        self.tts_pub.publish(notice_msg)
        self.get_logger().info('Published start notice to %s.' % self.tts_topic)


def main(args=None):
    rclpy.init(args=args)
    node = VlmTtsDemoNode()
    try:
        rclpy.spin(node)
    finally:
        if node._vlm_process is not None and node._vlm_process.poll() is None:
            node._vlm_process.terminate()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
