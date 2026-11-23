#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty
 
settings = termios.tcgetattr(sys.stdin)
 
msg = """
地瓜智能车 WASD 控制终端!
---------------------------
移动控制:
        w (前进)
   a (左转)    s (后退)    d (右转)
空格键 / x : 紧急刹车
Ctrl-C 退出
"""
 
moveBindings = {
    'w': (0.15, 0.0),   # x速度 0.15m/s, z角速度 0.0
    's': (-0.15, 0.0),  # x速度 -0.15m/s
    'a': (0.15, 0.3),   # 前进并左转 (阿克曼需要有线速度才能转角)
    'd': (0.15, -0.3),  # 前进并右转
}
 
def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key
 
def main():
    rclpy.init()
    node = rclpy.create_node('wasd_teleop')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    
    speed = 0.0
    turn = 0.0
    
    try:
        print(msg)
        while True:
            key = getKey()
            if key in moveBindings.keys():
                speed = moveBindings[key][0]
                turn = moveBindings[key][1]
            elif key == ' ' or key == 'x':
                speed = 0.0
                turn = 0.0
            elif key == '\x03': # Ctrl-C
                break
            else:
                continue
 
            twist = Twist()
            twist.linear.x = float(speed)
            twist.angular.z = float(turn)
            pub.publish(twist)
 
    except Exception as e:
        print(e)
    finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()
 
if __name__ == '__main__':
    main()