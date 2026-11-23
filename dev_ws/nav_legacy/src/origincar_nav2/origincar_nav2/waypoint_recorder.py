#!/usr/bin/env python3
"""
waypoint_recorder.py — 路径点录制节点
在 RViz2 中用 "Publish Point" 工具点击地图，
自动记录坐标并保存为 YAML 文件。

用法：
  ros2 run origincar_nav2 waypoint_recorder

在 RViz2 中：
  1. 工具栏选择 "Publish Point"（📍图标）
  2. 依次点击地图上的路径点位置
  3. 每点一次，终端打印一个记录
  4. 按 Ctrl+C 结束录制，自动保存为 recorded_waypoints.yaml
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import yaml
import os
import math


class WaypointRecorder(Node):
    def __init__(self):
        super().__init__('waypoint_recorder')

        self.declare_parameter('output_file', '')
        self.output_file = self.get_parameter('output_file').value
        if not self.output_file:
            self.output_file = os.path.join(os.getcwd(), 'recorded_waypoints.yaml')

        self.waypoints = []

        self.sub = self.create_subscription(
            PointStamped, '/clicked_point', self.clicked_callback, 10)

        self.get_logger().info('=' * 50)
        self.get_logger().info('路径点录制器已启动')
        self.get_logger().info('请在 RViz2 中使用 "Publish Point" 工具点击地图')
        self.get_logger().info(f'录制完成后按 Ctrl+C 保存到: {self.output_file}')
        self.get_logger().info('=' * 50)

    def clicked_callback(self, msg: PointStamped):
        x, y = msg.point.x, msg.point.y
        wp = [round(x, 3), round(y, 3), 0.0]       # yaw 先占位，保存时自动计算
        self.waypoints.append(wp)
        self.get_logger().info(f'✅ #{len(self.waypoints)}: ({x:.3f}, {y:.3f})')

    def save(self):
        # 自动计算每个路径点的朝向（面朝下一个点）
        for i in range(len(self.waypoints)):
            nxt = self.waypoints[(i + 1) % len(self.waypoints)]  # 环形：最后一点面朝第一个
            dx = nxt[0] - self.waypoints[i][0]
            dy = nxt[1] - self.waypoints[i][1]
            yaw = math.atan2(dy, dx)
            self.waypoints[i][2] = round(yaw, 3)
        data = {'waypoints': self.waypoints}
        with open(self.output_file, 'w') as f:
            yaml.dump(data, f, default_flow_style=None, allow_unicode=True)
        self.get_logger().info(f'💾 已保存 {len(self.waypoints)} 个路径点到: {self.output_file}')

    def destroy_node(self):
        self.save()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()