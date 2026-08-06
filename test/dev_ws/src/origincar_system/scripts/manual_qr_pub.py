#!/usr/bin/env python3
"""手动二维码注入工具：等待 /qr_code 出现订阅者后发布数字并退出。

用法：
    python3 manual_qr_pub.py <纯数字>
被 start_path_follower.sh --qr <数字> 自动调用，也可在运行中手动执行。
"""
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ManualQrPub(Node):
    def __init__(self, number: str):
        super().__init__("manual_qr_pub")
        self.number = number
        self.publisher_ = self.create_publisher(String, "/qr_code", 10)

    def wait_and_publish(self, timeout_sec: float = 180.0) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self.count_subscribers("/qr_code") > 0:
                msg = String()
                msg.data = self.number
                for _ in range(2):
                    self.publisher_.publish(msg)
                    time.sleep(0.5)
                parity = "顺时针(奇数)" if int(self.number) % 2 == 1 else "逆时针(偶数)"
                self.get_logger().info(
                    "已发布手动二维码 %s 到 /qr_code，方向判定：%s",
                    self.number, parity)
                return True
            time.sleep(1.0)
        self.get_logger().warn(
            "等待 %d 秒仍未发现 /qr_code 订阅者（终端二主控可能未启动），发布失败",
            int(timeout_sec))
        return False


def main() -> int:
    if len(sys.argv) < 2 or not sys.argv[1].isdigit():
        print("用法：manual_qr_pub.py <纯数字>", file=sys.stderr)
        return 2
    rclpy.init()
    node = ManualQrPub(sys.argv[1])
    ok = node.wait_and_publish()
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
