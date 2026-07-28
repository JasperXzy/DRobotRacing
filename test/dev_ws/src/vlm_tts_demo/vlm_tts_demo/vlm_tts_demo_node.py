#!/usr/bin/env python3
import base64
import json
import mimetypes
import os
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32, String


DEFAULT_PROMPT = (
    "只关注画面中央附近的医疗立牌或展示牌，不要背景信息。"
    "识别立牌上描绘的医疗场景，只输出场景内容本身。"
    "不要使用“立牌展示了”“画面中”“内容是”“这是一张”“可以看到”等开头。"
    "必须包含可见细节，不要回答“无法识别”或“看不清”等模糊词语。"
    "字数控制在25字以内。"
)


class VlmTtsDemoNode(Node):
    """Capture a camera frame on trigger, call a vision API, then publish TTS text."""

    def __init__(self):
        super().__init__('vlm_tts_demo_node')

        self.declare_parameter('trigger_topic', '/sign4return')
        self.declare_parameter('trigger_value', -1)
        self.declare_parameter('infer_trigger_value', 0)
        self.declare_parameter('infer_on_capture', False)
        self.declare_parameter('camera_topic', '/camera/image')
        self.declare_parameter('captured_image_file', '/tmp/vlm_tts_demo_capture.jpg')
        self.declare_parameter('source_capture_file', '')
        self.declare_parameter('tts_topic', '/tts_text')
        self.declare_parameter('result_topic', '/vlm_api_text')
        self.declare_parameter('ding_enable', False)
        self.declare_parameter('start_notice', '开始识别')
        self.declare_parameter('error_notice', '识别失败，请检查网络或接口配置')
        self.declare_parameter(
            'last_success_text_file',
            '/root/D_Robot/test/dev_ws/captures/last_successful_vlm_text.txt')
        self.declare_parameter('failure_prefix', '一')
        self.declare_parameter('user_prompt', DEFAULT_PROMPT)
        self.declare_parameter('env_file', '')
        self.declare_parameter('api_endpoint', '')
        self.declare_parameter('api_key', '')
        self.declare_parameter('api_key_env', 'OPENAI_API_KEY')
        self.declare_parameter('api_model', '')
        self.declare_parameter('api_timeout', 15.0)
        self.declare_parameter('api_max_tokens', 120)
        self.declare_parameter('api_temperature', 0.2)
        self.declare_parameter('api_image_detail', 'auto')
        self.declare_parameter('api_image_mime_type', 'image/jpeg')
        self.declare_parameter('no_auth', False)

        self.trigger_topic = self.get_parameter('trigger_topic').value
        self.trigger_value = self.get_parameter('trigger_value').value
        self.infer_trigger_value = self.get_parameter('infer_trigger_value').value
        self.infer_on_capture = self.get_parameter('infer_on_capture').value
        self.camera_topic = self.get_parameter('camera_topic').value
        self.captured_image_file = self.get_parameter('captured_image_file').value
        self.source_capture_file = self.get_parameter('source_capture_file').value
        self.tts_topic = self.get_parameter('tts_topic').value
        self.result_topic = self.get_parameter('result_topic').value
        self.ding_enable = self.get_parameter('ding_enable').value
        self.start_notice = self.get_parameter('start_notice').value
        self.error_notice = self.get_parameter('error_notice').value
        self.last_success_text_file = (
            self.get_parameter('last_success_text_file').value)
        self.failure_prefix = self.get_parameter('failure_prefix').value
        self.user_prompt = self.get_parameter('user_prompt').value or DEFAULT_PROMPT

        self.env_file = self.get_parameter('env_file').value
        self._load_env_file(self.env_file)

        api_key_env = self.get_parameter('api_key_env').value
        self.api_endpoint = (
            self.get_parameter('api_endpoint').value
            or os.environ.get('VLM_API_ENDPOINT', '')
            or 'https://api.openai.com/v1/chat/completions'
        )
        self.api_key = (
            self.get_parameter('api_key').value
            or os.environ.get('VLM_API_KEY', '')
            or os.environ.get(api_key_env, '')
        )
        self.api_model = (
            self.get_parameter('api_model').value
            or os.environ.get('VLM_API_MODEL', '')
            or 'gpt-4o-mini'
        )
        self.api_timeout = self.get_parameter('api_timeout').value
        self.api_max_tokens = self.get_parameter('api_max_tokens').value
        self.api_temperature = self.get_parameter('api_temperature').value
        self.api_image_detail = self.get_parameter('api_image_detail').value
        self.api_image_mime_type = self.get_parameter('api_image_mime_type').value
        self.no_auth = self.get_parameter('no_auth').value

        self._lock = threading.Lock()
        self._latest_image_data = None
        self._captured_image_data = None
        self._waiting_for_image = False
        self._api_busy = False
        self._api_thread = None
        self._last_success_text = self._load_last_success_text()

        self.tts_pub = self.create_publisher(String, self.tts_topic, 10)
        self.result_pub = self.create_publisher(String, self.result_topic, 10)
        self.create_subscription(Int32, self.trigger_topic, self._on_trigger, 10)
        self.create_subscription(
            CompressedImage,
            self.camera_topic,
            self._on_camera_image,
            10,
        )

        self.get_logger().info(
            'API VLM TTS ready: camera=%s, trigger=%s:%d capture, infer=%d, model=%s'
            % (
                self.camera_topic,
                self.trigger_topic,
                self.trigger_value,
                self.infer_trigger_value,
                self.api_model,
            )
        )

    def _load_env_file(self, env_file):
        if not env_file:
            return

        path = Path(env_file).expanduser()
        if not path.exists():
            self.get_logger().warn('API env file not found: %s' % path)
            return

        for raw_line in path.read_text(encoding='utf-8').splitlines():
            line = raw_line.strip()
            if not line or line.startswith('#') or '=' not in line:
                continue
            key, value = line.split('=', 1)
            key = key.strip()
            value = value.strip().strip('"').strip("'")
            if key and key not in os.environ:
                os.environ[key] = value
        self.get_logger().info('Loaded API env file: %s' % path)

    def _load_last_success_text(self):
        path = Path(self.last_success_text_file).expanduser()
        try:
            if not path.exists():
                return ''
            text = path.read_text(encoding='utf-8').strip()
            if text:
                self.get_logger().info(
                    'Loaded last successful VLM text from %s.' % path)
            return text
        except OSError as exc:
            self.get_logger().warn(
                'Could not load last successful VLM text from %s: %s'
                % (path, exc))
            return ''

    def _store_last_success_text(self, text):
        text = (text or '').strip()
        if not text:
            return
        with self._lock:
            self._last_success_text = text
        path = Path(self.last_success_text_file).expanduser()
        temporary_path = path.with_name(path.name + '.tmp')
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            temporary_path.write_text(text + '\n', encoding='utf-8')
            temporary_path.replace(path)
        except OSError as exc:
            self.get_logger().error(
                'Could not persist successful VLM text to %s: %s'
                % (path, exc))

    def _publish_failure_fallback(self):
        with self._lock:
            previous_text = self._last_success_text
        fallback = '%s%s' % (self.failure_prefix, previous_text)
        if not fallback:
            return
        self.get_logger().warn(
            'Publishing VLM failure fallback using last successful text.')
        self._publish_tts(fallback)

    def _on_trigger(self, msg):
        if msg.data == self.trigger_value:
            self.get_logger().info('Capture trigger received.')
            if self._save_latest_image():
                if self.infer_on_capture:
                    self._start_api_inference()
                return

            with self._lock:
                self._waiting_for_image = True
            self.get_logger().info('Waiting for one camera frame to capture.')
            return

        if msg.data == self.infer_trigger_value:
            self.get_logger().info('Infer trigger received.')
            self._publish_start_notice()
            self._start_api_inference()
            return

        self.get_logger().debug('Ignore trigger value %d.' % msg.data)

    def _on_camera_image(self, msg):
        with self._lock:
            self._latest_image_data = bytes(msg.data)
            should_capture = self._waiting_for_image
            if should_capture:
                self._waiting_for_image = False

        if should_capture and self._save_latest_image() and self.infer_on_capture:
            self._start_api_inference()

    def _save_latest_image(self):
        with self._lock:
            image_data = self._latest_image_data

        if not image_data:
            self.get_logger().warn('No camera frame available yet.')
            return False

        self._write_image_file(self.captured_image_file, image_data)
        if self.source_capture_file:
            self._write_image_file(self.source_capture_file, image_data)

        with self._lock:
            self._captured_image_data = image_data

        self.get_logger().info('Saved camera frame to %s.' % self.captured_image_file)
        return True

    def _write_image_file(self, path, image_data):
        image_path = Path(path).expanduser()
        image_path.parent.mkdir(parents=True, exist_ok=True)
        image_path.write_bytes(image_data)

    def _start_api_inference(self):
        with self._lock:
            if self._api_busy:
                self.get_logger().warn('API inference is already running, ignore trigger.')
                return
            image_data = self._captured_image_data or self._latest_image_data
            self._api_busy = True

        if not image_data:
            with self._lock:
                self._api_busy = False
            self.get_logger().warn('No captured image available for API inference.')
            self._publish_failure_fallback()
            return

        self._api_thread = threading.Thread(
            target=self._run_api_inference,
            args=(image_data,),
            daemon=True,
        )
        self._api_thread.start()

    def _run_api_inference(self, image_data):
        started = time.monotonic()
        try:
            text = self._call_vision_api(image_data)
            text = self._clean_for_speech(text)
            if not text:
                raise RuntimeError('API response has no text content.')

            elapsed = time.monotonic() - started
            self.get_logger().info('API VLM result in %.2fs: %s' % (elapsed, text))
            self._store_last_success_text(text)
            self._publish_result(text)
            self._publish_tts(text)
        except Exception as exc:
            self.get_logger().error('API VLM inference failed: %s' % exc)
            self._publish_failure_fallback()
        finally:
            with self._lock:
                self._api_busy = False

    def _call_vision_api(self, image_data):
        if not self.no_auth and not self.api_key:
            raise RuntimeError('Missing API key. Set VLM_API_KEY or api_key parameter.')

        mime_type = self.api_image_mime_type
        if not mime_type:
            mime_type = mimetypes.guess_type(self.captured_image_file)[0]
        if not mime_type:
            mime_type = 'image/jpeg'

        image_b64 = base64.b64encode(image_data).decode('ascii')
        image_url = {
            'url': 'data:%s;base64,%s' % (mime_type, image_b64),
        }
        if self.api_image_detail:
            image_url['detail'] = self.api_image_detail

        payload = {
            'model': self.api_model,
            'messages': [
                {
                    'role': 'user',
                    'content': [
                        {'type': 'text', 'text': self.user_prompt},
                        {'type': 'image_url', 'image_url': image_url},
                    ],
                },
            ],
            'max_tokens': self.api_max_tokens,
            'temperature': self.api_temperature,
        }

        headers = {'Content-Type': 'application/json'}
        if not self.no_auth:
            headers['Authorization'] = 'Bearer %s' % self.api_key

        request = urllib.request.Request(
            self._normalize_chat_completions_url(self.api_endpoint),
            data=json.dumps(payload).encode('utf-8'),
            headers=headers,
            method='POST',
        )

        try:
            with urllib.request.urlopen(request, timeout=self.api_timeout) as response:
                raw = response.read().decode('utf-8')
        except urllib.error.HTTPError as exc:
            body = exc.read().decode('utf-8', errors='replace')
            raise RuntimeError('API HTTP %d: %s' % (exc.code, body[:500])) from exc
        except urllib.error.URLError as exc:
            raise RuntimeError('API network error: %s' % exc) from exc

        data = json.loads(raw)
        return self._extract_message_text(data)

    def _extract_message_text(self, data):
        choices = data.get('choices') or []
        if not choices:
            raise RuntimeError('API response has no choices: %s' % data)

        content = choices[0].get('message', {}).get('content', '')
        if isinstance(content, str):
            return content.strip()
        if isinstance(content, list):
            parts = []
            for item in content:
                if isinstance(item, str):
                    parts.append(item)
                elif isinstance(item, dict):
                    text = item.get('text') or item.get('content')
                    if isinstance(text, str):
                        parts.append(text)
            return ''.join(parts).strip()
        return str(content).strip()

    def _clean_for_speech(self, text):
        text = (text or '').strip()
        prefixes = [
            '立牌展示了',
            '立牌展示',
            '立牌上展示了',
            '立牌上是',
            '画面中',
            '内容是',
            '这是一张',
            '可以看到',
        ]
        changed = True
        while changed:
            changed = False
            for prefix in prefixes:
                if text.startswith(prefix):
                    text = text[len(prefix):].lstrip(' ，,。:：')
                    changed = True
        return text.strip()

    def _normalize_chat_completions_url(self, url):
        url = url.strip().rstrip('/')
        if url.endswith('/chat/completions'):
            return url
        return url + '/chat/completions'

    def _publish_result(self, text):
        msg = String()
        msg.data = text
        self.result_pub.publish(msg)

    def _publish_tts(self, text):
        if not text:
            return
        msg = String()
        msg.data = text
        self.tts_pub.publish(msg)

    def _publish_start_notice(self):
        if self.ding_enable and self.start_notice:
            self._publish_tts(self.start_notice)


def main(args=None):
    rclpy.init(args=args)
    node = VlmTtsDemoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
