#!/usr/bin/env python3
"""
API VLM + TTS demo.

This script can:
1. Read a local image or capture one compressed frame from a ROS2 camera topic.
2. Send it to an OpenAI-compatible vision chat-completions API.
3. Print the result and optionally publish it to /tts_text for hobot_tts.
"""

import argparse
import base64
import json
import mimetypes
import os
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path


DEFAULT_IMAGE = Path(__file__).resolve().parents[1] / "hobot_vlm_test" / "config" / "panda.jpg"
DEFAULT_ENV_FILE = Path(__file__).with_name("api_vlm_tts_demo.env")
DEFAULT_CAPTURE = Path(__file__).with_name("api_vlm_tts_capture.jpg")
DEFAULT_PROMPT = (
    "只关注画面中央附近的医疗立牌或展示牌，不要背景信息。"
    "识别立牌上描绘的医疗场景，只输出场景内容本身。"
    "不要使用“立牌展示了”“画面中”“内容是”“这是一张”“可以看到”等开头。"
    "必须包含可见细节，不要回答“无法识别”或“看不清”等模糊词语。"
    "字数控制在25字以内。"
)


def load_env_file(path):
    path = Path(path).expanduser()
    if not path.exists():
        return

    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        key = key.strip()
        value = value.strip().strip('"').strip("'")
        if key and key not in os.environ:
            os.environ[key] = value


def parse_args():
    env_file = os.environ.get("VLM_API_ENV_FILE", str(DEFAULT_ENV_FILE))
    load_env_file(env_file)

    parser = argparse.ArgumentParser(
        description="Call a vision API and publish the result to a ROS2 TTS topic."
    )
    parser.add_argument(
        "--image",
        default=str(DEFAULT_IMAGE),
        help="Local image path. Ignored when --camera-topic is set.",
    )
    parser.add_argument(
        "--camera-topic",
        default="",
        help="Optional sensor_msgs/msg/CompressedImage topic, e.g. /camera/image.",
    )
    parser.add_argument(
        "--camera-timeout",
        type=float,
        default=5.0,
        help="Seconds to wait for one camera frame.",
    )
    parser.add_argument(
        "--save-capture",
        default=str(DEFAULT_CAPTURE),
        help="Where to save the camera frame when --camera-topic is used.",
    )
    parser.add_argument(
        "--prompt",
        default=DEFAULT_PROMPT,
        help="Prompt sent with the image.",
    )
    parser.add_argument(
        "--api-endpoint",
        default=os.environ.get(
            "VLM_API_ENDPOINT",
            "https://api.openai.com/v1/chat/completions",
        ),
        help="OpenAI-compatible chat completions endpoint.",
    )
    parser.add_argument(
        "--api-key",
        default=os.environ.get("VLM_API_KEY", ""),
        help="API key. Prefer environment variables instead of passing this directly.",
    )
    parser.add_argument(
        "--api-key-env",
        default="OPENAI_API_KEY",
        help="Environment variable that stores the API key.",
    )
    parser.add_argument(
        "--no-auth",
        action="store_true",
        help="Do not send an Authorization header, useful for local gateways.",
    )
    parser.add_argument(
        "--model",
        default=os.environ.get("VLM_API_MODEL", "gpt-4o-mini"),
        help="Vision-capable model name.",
    )
    parser.add_argument(
        "--max-tokens",
        type=int,
        default=120,
        help="Maximum output tokens.",
    )
    parser.add_argument(
        "--temperature",
        type=float,
        default=0.2,
        help="Sampling temperature.",
    )
    parser.add_argument(
        "--image-detail",
        default="auto",
        choices=["low", "high", "auto"],
        help="Image detail hint for compatible APIs.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=15.0,
        help="API request timeout in seconds.",
    )
    parser.add_argument(
        "--tts-topic",
        default="/tts_text",
        help="ROS2 std_msgs/String topic consumed by hobot_tts.",
    )
    parser.add_argument(
        "--result-topic",
        default="/vlm_api_text",
        help="ROS2 std_msgs/String topic for raw VLM result.",
    )
    parser.add_argument(
        "--no-tts",
        action="store_true",
        help="Only print the API result; do not publish to ROS2 TTS topics.",
    )
    parser.add_argument(
        "--start-notice",
        default="开始识别",
        help="Optional short TTS notice published before the API request.",
    )
    return parser.parse_args()


def read_image(args):
    if args.camera_topic:
        image_data = capture_ros_compressed_image(
            args.camera_topic,
            args.camera_timeout,
        )
        save_path = Path(args.save_capture)
        save_path.parent.mkdir(parents=True, exist_ok=True)
        save_path.write_bytes(image_data)
        print(f"Saved camera frame: {save_path}")
        return image_data, "image/jpeg"

    image_path = Path(args.image).expanduser()
    if not image_path.is_absolute():
        image_path = Path.cwd() / image_path
    if not image_path.exists():
        raise FileNotFoundError(f"Image not found: {image_path}")

    mime_type = mimetypes.guess_type(str(image_path))[0] or "image/jpeg"
    return image_path.read_bytes(), mime_type


def capture_ros_compressed_image(topic, timeout):
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import CompressedImage
    except ImportError as exc:
        raise RuntimeError(
            "ROS2 Python packages are required for --camera-topic. "
            "Please source ROS2 and the workspace first."
        ) from exc

    class OneFrameNode(Node):
        def __init__(self):
            super().__init__("api_vlm_tts_frame_grabber")
            self.frame = None
            self.create_subscription(
                CompressedImage,
                topic,
                self._on_image,
                10,
            )

        def _on_image(self, msg):
            self.frame = bytes(msg.data)

    rclpy.init(args=None)
    node = OneFrameNode()
    deadline = time.monotonic() + timeout
    try:
        while rclpy.ok() and node.frame is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.frame is None:
            raise TimeoutError(f"No image received from {topic} within {timeout:.1f}s.")
        return node.frame
    finally:
        node.destroy_node()
        rclpy.shutdown()


def call_vision_api(args, image_data, mime_type):
    api_key = args.api_key or os.environ.get(args.api_key_env, "")
    if not args.no_auth and not api_key:
        raise RuntimeError(
            f"Missing API key. Set {args.api_key_env} or VLM_API_KEY."
        )

    image_b64 = base64.b64encode(image_data).decode("ascii")
    image_url = {
        "url": f"data:{mime_type};base64,{image_b64}",
        "detail": args.image_detail,
    }
    payload = {
        "model": args.model,
        "messages": [
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": args.prompt},
                    {"type": "image_url", "image_url": image_url},
                ],
            }
        ],
        "max_tokens": args.max_tokens,
        "temperature": args.temperature,
    }

    headers = {"Content-Type": "application/json"}
    if not args.no_auth:
        headers["Authorization"] = f"Bearer {api_key}"

    endpoint = normalize_chat_completions_url(args.api_endpoint)
    request = urllib.request.Request(
        endpoint,
        data=json.dumps(payload).encode("utf-8"),
        headers=headers,
        method="POST",
    )
    try:
        with urllib.request.urlopen(request, timeout=args.timeout) as response:
            raw = response.read().decode("utf-8")
    except urllib.error.HTTPError as exc:
        body = exc.read().decode("utf-8", errors="replace")
        raise RuntimeError(f"API HTTP {exc.code}: {body[:500]}") from exc
    except urllib.error.URLError as exc:
        raise RuntimeError(f"API network error: {exc}") from exc

    data = json.loads(raw)
    return extract_message_text(data), data


def normalize_chat_completions_url(url):
    url = url.strip().rstrip("/")
    if url.endswith("/chat/completions"):
        return url
    return url + "/chat/completions"


def extract_message_text(data):
    choices = data.get("choices") or []
    if not choices:
        raise RuntimeError(f"API response has no choices: {data}")

    content = choices[0].get("message", {}).get("content", "")
    if isinstance(content, str):
        return content.strip()
    if isinstance(content, list):
        parts = []
        for item in content:
            if isinstance(item, str):
                parts.append(item)
            elif isinstance(item, dict):
                text = item.get("text") or item.get("content")
                if isinstance(text, str):
                    parts.append(text)
        return "".join(parts).strip()
    return str(content).strip()


def publish_ros_text(topic, text):
    try:
        import rclpy
        from std_msgs.msg import String
    except ImportError as exc:
        raise RuntimeError(
            "ROS2 Python packages are required to publish TTS text. "
            "Use --no-tts for API-only testing."
        ) from exc

    rclpy.init(args=None)
    node = rclpy.create_node("api_vlm_tts_text_pub")
    publisher = node.create_publisher(String, topic, 10)
    msg = String()
    msg.data = text
    try:
        # Give DDS discovery a brief moment, then publish a few times.
        end = time.monotonic() + 0.8
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        for _ in range(3):
            publisher.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    args = parse_args()

    if not args.no_tts and args.start_notice:
        try:
            publish_ros_text(args.tts_topic, args.start_notice)
        except Exception as exc:
            print(f"Warning: start notice was not published: {exc}", file=sys.stderr)

    image_data, mime_type = read_image(args)
    text, _raw = call_vision_api(args, image_data, mime_type)

    print("\n=== VLM Result ===")
    print(text)

    if not args.no_tts:
        publish_ros_text(args.result_topic, text)
        publish_ros_text(args.tts_topic, text)
        print(f"\nPublished result to {args.result_topic} and {args.tts_topic}.")


if __name__ == "__main__":
    main()
