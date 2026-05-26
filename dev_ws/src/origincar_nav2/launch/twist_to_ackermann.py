#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDrive

class TwistToAckermann(Node):
    def __init__(self):
        super().__init__('twist_to_ackermann')
        self.declare_parameter('wheelbase', 0.3)
        self.declare_parameter('max_steering_angle', 0.6)
        self.declare_parameter('max_speed', 1.0)
        self.wheelbase = self.get_parameter('wheelbase').value
        self.max_steer = self.get_parameter('max_steering_angle').value
        self.max_speed = self.get_parameter('max_speed').value

        self.sub = self.create_subscription(Twist, 'twist', self.twist_callback, 10)
        self.pub = self.create_publisher(AckermannDrive, 'ackermann', 10)

    def twist_callback(self, msg):
        v = msg.linear.x
        omega = msg.angular.z
        # 阿克曼转向近似：前轮转角 delta = atan(omega * wheelbase / v)  当 v != 0
        if abs(v) < 0.01:
            steering = 0.0
        else:
            steering = omega * self.wheelbase / v
        steering = max(min(steering, self.max_steer), -self.max_steer)

        ackermann_msg = AckermannDrive()
        ackermann_msg.speed = max(min(v, self.max_speed), -self.max_speed)
        ackermann_msg.steering_angle = steering
        self.pub.publish(ackermann_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TwistToAckermann()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()