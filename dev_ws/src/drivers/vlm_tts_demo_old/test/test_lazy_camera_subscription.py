import importlib
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))


class _FakeParameterValue:
    def __init__(self, value):
        self.string_value = value if isinstance(value, str) else ''
        self.integer_value = value if isinstance(value, int) and not isinstance(value, bool) else 0
        self.bool_value = value if isinstance(value, bool) else False


class _FakeParameter:
    def __init__(self, value):
        self._value = value

    def get_parameter_value(self):
        return _FakeParameterValue(self._value)


class _FakeNode:
    def __init__(self, name):
        self.name = name
        self.parameters = {}
        self.subscriptions = []

    def declare_parameter(self, name, default_value):
        self.parameters[name] = default_value

    def get_parameter(self, name):
        return _FakeParameter(self.parameters[name])

    def create_publisher(self, *_args, **_kwargs):
        return types.SimpleNamespace(publish=lambda _msg: None)

    def create_subscription(self, msg_type, topic, callback, qos):
        sub = types.SimpleNamespace(
            msg_type=msg_type,
            topic=topic,
            callback=callback,
            qos=qos,
            destroyed=False,
        )
        self.subscriptions.append(sub)
        return sub

    def destroy_subscription(self, sub):
        sub.destroyed = True

    def get_logger(self):
        return types.SimpleNamespace(
            info=lambda *_args, **_kwargs: None,
            debug=lambda *_args, **_kwargs: None,
        )


def _fake_ros_modules():
    rclpy = types.ModuleType('rclpy')
    rclpy.init = lambda *args, **kwargs: None
    rclpy.spin = lambda *args, **kwargs: None
    rclpy.shutdown = lambda *args, **kwargs: None

    rclpy_node = types.ModuleType('rclpy.node')
    rclpy_node.Node = _FakeNode

    sensor_msgs = types.ModuleType('sensor_msgs')
    sensor_msgs_msg = types.ModuleType('sensor_msgs.msg')
    sensor_msgs_msg.CompressedImage = type('CompressedImage', (), {})

    std_msgs = types.ModuleType('std_msgs')
    std_msgs_msg = types.ModuleType('std_msgs.msg')
    std_msgs_msg.Int32 = type('Int32', (), {})
    std_msgs_msg.String = type('String', (), {})

    return {
        'rclpy': rclpy,
        'rclpy.node': rclpy_node,
        'sensor_msgs': sensor_msgs,
        'sensor_msgs.msg': sensor_msgs_msg,
        'std_msgs': std_msgs,
        'std_msgs.msg': std_msgs_msg,
    }


def _load_node_module():
    sys.modules.pop('vlm_tts_demo.vlm_tts_demo_node', None)
    return importlib.import_module('vlm_tts_demo.vlm_tts_demo_node')


class LazyCameraSubscriptionTest(unittest.TestCase):

    def test_camera_subscription_is_lazy_until_capture_trigger(self):
        with patch.dict(sys.modules, _fake_ros_modules()):
            module = _load_node_module()

            node = module.VlmTtsDemoNode()

            self.assertFalse(
                any(sub.topic == '/camera/image' for sub in node.subscriptions)
            )
            node._on_trigger(types.SimpleNamespace(data=-1))
            self.assertTrue(
                any(sub.topic == '/camera/image' for sub in node.subscriptions)
            )

    def test_camera_subscription_is_destroyed_after_one_capture(self):
        with patch.dict(sys.modules, _fake_ros_modules()):
            module = _load_node_module()

            node = module.VlmTtsDemoNode()
            saved_images = []
            node._save_image_data = lambda image_data: saved_images.append(image_data) or True

            node._on_trigger(types.SimpleNamespace(data=-1))
            camera_sub = next(sub for sub in node.subscriptions if sub.topic == '/camera/image')
            node._on_camera_image(types.SimpleNamespace(data=b'jpeg-data'))

            self.assertEqual(saved_images, [b'jpeg-data'])
            self.assertTrue(camera_sub.destroyed)
            self.assertIsNone(node._camera_subscription)


if __name__ == '__main__':
    unittest.main()
