#!/usr/bin/env python3
"""
发令脚本：向 /cruise_start 发布一次空消息后退出。
用于比赛时在终端一键启动巡航。

用法:
    ros2 run origincar_nav2 trigger_cruise
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty


class TriggerCruise(Node):
    def __init__(self):
        super().__init__('trigger_cruise')
        self.pub = self.create_publisher(Empty, '/cruise_start', 10)
        self._wait_timer = self.create_timer(0.5, self._wait_and_fire)

    def _wait_and_fire(self):
        if self.pub.get_subscription_count() == 0:
            self.get_logger().info('等待巡航节点就绪...', throttle_duration_sec=3.0)
            return
        self._wait_timer.cancel()
        self.pub.publish(Empty())
        self.get_logger().info('发令成功！巡航开始！')
        raise SystemExit


def main():
    rclpy.init()
    try:
        rclpy.spin(TriggerCruise())
    except SystemExit:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
