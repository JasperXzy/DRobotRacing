#!/usr/bin/env python3
"""Scale Nav2 velocity commands before they are sent downstream."""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool


class CmdVelKpScaler(Node):
    def __init__(self):
        super().__init__('cmd_vel_kp_scaler')

        self.declare_parameter('input_topic', '/cmd_vel_teb')
        self.declare_parameter('output_topic', '/cmd_vel_nav')
        self.declare_parameter('avoidance_topic', '/ysvv_obstacle_avoidance_active')
        self.declare_parameter('kp1', 1.0)
        self.declare_parameter('kp2', 1.0)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.avoidance_topic = self.get_parameter('avoidance_topic').value
        self.kp1 = float(self.get_parameter('kp1').value)
        self.kp2 = float(self.get_parameter('kp2').value)
        self.avoidance_active = False

        self.publisher = self.create_publisher(Twist, self.output_topic, 10)
        self.subscription = self.create_subscription(
            Twist, self.input_topic, self._on_cmd_vel, 10)
        self.avoidance_subscription = self.create_subscription(
            Bool, self.avoidance_topic, self._on_avoidance_state, 10)

        self.get_logger().info(
            f'cmd_vel Kp scaler: {self.input_topic} -> {self.output_topic}, '
            f'kp1={self.kp1:.3f}, kp2={self.kp2:.3f}, '
            f'avoidance_topic={self.avoidance_topic}')

    def _on_avoidance_state(self, msg):
        active = bool(msg.data)
        if active != self.avoidance_active:
            mode = 'obstacle avoidance' if active else 'normal navigation'
            self.get_logger().info(f'cmd_vel Kp scaler switched to {mode} mode')
        self.avoidance_active = active

    def _on_cmd_vel(self, msg):
        self.kp1 = float(self.get_parameter('kp1').value)
        self.kp2 = float(self.get_parameter('kp2').value)
        kp = self.kp2 if self.avoidance_active else self.kp1
        scaled = Twist()
        scaled.linear.x = msg.linear.x * kp
        scaled.linear.y = msg.linear.y * kp
        scaled.linear.z = msg.linear.z * kp
        scaled.angular.x = msg.angular.x * kp
        scaled.angular.y = msg.angular.y * kp
        scaled.angular.z = msg.angular.z * kp
        self.publisher.publish(scaled)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelKpScaler()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
