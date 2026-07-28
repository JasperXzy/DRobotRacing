#!/usr/bin/env python3
"""Full-task closed-loop simulator for visual_lidar_path_follower."""

import math
import os
import time
import json
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from PIL import Image
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32, String
from tf2_ros import TransformBroadcaster


MAP_ROOT = Path('/root/D_Robot/test/dev_ws/src/origincar_system/map')
MAP_TOPICS = {
    '/map': MAP_ROOT / 'race_modify.yaml',
    '/restriction_map': MAP_ROOT / 'race_keepout.yaml',
    '/logic3_static_restriction_map': MAP_ROOT / 'race_keepout_logic3_static.yaml',
    '/logic3_runtime_restriction_map': MAP_ROOT / 'race_keepout_logic3_runtime.yaml',
}
OBSTACLES = [
    # 大厅6个：两组前后遮挡点用于验证“静态绿色 + 运行中新红点”。
    ('H1_task1_front', 0.86, 0.38),
    ('H2_qr_nearby_hidden', 1.45, 0.65),
    ('H3_transition_reverse', 1.95, 1.15),
    ('H4_lower_mid_front', 2.35, 0.32),
    ('H5_lower_mid_hidden', 3.36, 0.46),
    ('H6_upper_hall', 3.25, 1.24),
    # 通道5个：包含左侧上下圆弧重叠窗口及三处水平/右侧通道障碍。
    ('C1_lower_left_curve', 0.44, 2.91),
    ('C2_upper_left_curve', 0.44, 3.90),
    ('C3_lower_channel', 1.54, 2.91),
    ('C4_upper_middle', 2.58, 4.06),
    ('C5_upper_right', 3.42, 3.72),
]
QR_VALUE = os.environ.get('SIM_QR_VALUE', '1')


def quaternion_from_yaw(yaw):
    return 0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw)


def load_map(yaml_path):
    metadata = yaml.safe_load(yaml_path.read_text(encoding='utf-8'))
    image_path = yaml_path.parent / metadata['image']
    image = Image.open(image_path).convert('L')
    width, height = image.size
    pixels = list(image.getdata())
    negate = bool(metadata.get('negate', 0))
    occupied_threshold = float(metadata.get('occupied_thresh', 0.65))
    free_threshold = float(metadata.get('free_thresh', 0.25))
    mode = metadata.get('mode', 'trinary')
    values = []
    for row in range(height - 1, -1, -1):
        for column in range(width):
            pixel = pixels[row * width + column] / 255.0
            occupancy = pixel if negate else 1.0 - pixel
            if mode == 'scale':
                if occupancy >= occupied_threshold:
                    value = 100
                elif occupancy <= free_threshold:
                    value = 0
                else:
                    value = round(
                        100.0 * (occupancy - free_threshold) /
                        (occupied_threshold - free_threshold))
            else:
                value = 100 if occupancy >= occupied_threshold else (
                    0 if occupancy <= free_threshold else -1)
            values.append(value)

    message = OccupancyGrid()
    message.header.frame_id = 'map'
    message.info.resolution = float(metadata['resolution'])
    message.info.width = width
    message.info.height = height
    origin = metadata['origin']
    message.info.origin.position.x = float(origin[0])
    message.info.origin.position.y = float(origin[1])
    message.info.origin.orientation.w = 1.0
    message.data = values
    return message


class FullTaskSimulator(Node):
    def __init__(self):
        super().__init__('full_task_simulator')
        transient = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_publishers = {
            topic: self.create_publisher(OccupancyGrid, topic, transient)
            for topic in MAP_TOPICS
        }
        self.maps = {
            topic: load_map(path)
            for topic, path in MAP_TOPICS.items()
        }
        self.scan_publisher = self.create_publisher(LaserScan, '/scan', 10)
        self.odom_publisher = self.create_publisher(Odometry, '/chassis/odom', 10)
        self.qr_publisher = self.create_publisher(String, '/qr_code', 10)
        self.cmd_subscription = self.create_subscription(
            Twist, '/cmd_vel', self.on_cmd, 10)
        self.snapshot_subscription = self.create_subscription(
            String, '/visual_lidar_path_follower/route_snapshot',
            self.on_snapshot, transient)
        self.image_trigger_subscription = self.create_subscription(
            Int32, '/sign4return', self.on_image_trigger, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.started_at = time.monotonic()
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.cmd = Twist()
        self.first_cmd_time = None
        self.last_step_time = time.monotonic()
        self.chassis_enabled = False
        self.qr_sent = False
        self.reverse_negative_since = None
        self.reverse_clear_since = None
        self.reverse_active = False
        self.reverse_event_count = 0
        self.event('SIM_START', f'障碍数量={len(OBSTACLES)}，二维码={QR_VALUE}')
        self.create_timer(0.02, self.step)
        self.create_timer(0.05, self.publish_scan)
        self.create_timer(0.5, self.publish_maps)
        self.publish_maps()
        self.get_logger().info('检测到控制指令后静止5秒，再模拟打开底盘。')

    def event(self, name, detail=''):
        elapsed = time.monotonic() - self.started_at
        suffix = f'：{detail}' if detail else ''
        self.get_logger().info(f'EVENT +{elapsed:07.3f}s {name}{suffix}')

    def on_cmd(self, message):
        self.cmd = message
        if self.first_cmd_time is None:
            self.first_cmd_time = time.monotonic()
            self.event('CMD_READY', '开始5秒静态READY窗口')

        now = time.monotonic()
        if message.linear.x < -0.05:
            self.reverse_clear_since = None
            if self.reverse_negative_since is None:
                self.reverse_negative_since = now
            if (not self.reverse_active and self.reverse_event_count == 0 and
                    now - self.reverse_negative_since >= 0.15):
                self.reverse_active = True
                self.reverse_event_count = 1
                self.event(
                    'QR_REVERSE_START',
                    f'pose=({self.x:.3f},{self.y:.3f})')
        else:
            self.reverse_negative_since = None
            if self.reverse_active:
                if self.reverse_clear_since is None:
                    self.reverse_clear_since = now
                elif now - self.reverse_clear_since >= 0.20:
                    self.reverse_active = False
                    self.event(
                        'QR_REVERSE_END',
                        f'pose=({self.x:.3f},{self.y:.3f})')

    def on_snapshot(self, message):
        try:
            data = json.loads(message.data)
        except (TypeError, ValueError):
            return
        event = str(data.get('event', 'UNKNOWN'))
        revision = int(data.get('frozen_revision', 0))
        self.event(
            f'SNAPSHOT_{event.upper()}',
            f'revision={revision}, green={len(data.get("green", []))}, '
            f'red={len(data.get("red", []))}')

    def on_image_trigger(self, message):
        if message.data == 8:
            self.event('IMAGE_TO_TEXT_TRIGGER', 'value=8')

    def publish_maps(self):
        stamp = self.get_clock().now().to_msg()
        for topic, message in self.maps.items():
            message.header.stamp = stamp
            self.map_publishers[topic].publish(message)

    def step(self):
        now_monotonic = time.monotonic()
        dt = min(0.05, now_monotonic - self.last_step_time)
        self.last_step_time = now_monotonic
        if (not self.chassis_enabled and self.first_cmd_time is not None and
                now_monotonic - self.first_cmd_time >= 5.0):
            self.chassis_enabled = True
            self.event('CHASSIS_ON')

        linear = self.cmd.linear.x if self.chassis_enabled else 0.0
        angular = self.cmd.angular.z if self.chassis_enabled else 0.0
        self.yaw = math.atan2(
            math.sin(self.yaw + angular * dt),
            math.cos(self.yaw + angular * dt))
        self.x += linear * math.cos(self.yaw) * dt
        self.y += linear * math.sin(self.yaw) * dt

        stamp = self.get_clock().now().to_msg()
        qx, qy, qz, qw = quaternion_from_yaw(self.yaw)
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(transform)

        odometry = Odometry()
        odometry.header.stamp = stamp
        odometry.header.frame_id = 'odom'
        odometry.child_frame_id = 'base_link'
        odometry.pose.pose.position.x = self.x
        odometry.pose.pose.position.y = self.y
        odometry.pose.pose.orientation.x = qx
        odometry.pose.pose.orientation.y = qy
        odometry.pose.pose.orientation.z = qz
        odometry.pose.pose.orientation.w = qw
        odometry.twist.twist.linear.x = linear
        odometry.twist.twist.angular.z = angular
        self.odom_publisher.publish(odometry)

        # 提前在x门槛之前发布，验证控制节点只缓存方向，直到x>1.80才切换阶段。
        if self.chassis_enabled and not self.qr_sent and self.x >= 1.00:
            qr = String()
            qr.data = QR_VALUE
            self.qr_publisher.publish(qr)
            self.qr_sent = True
            self.event(
                'QR_PUBLISHED_EARLY',
                f'value={QR_VALUE}, pose=({self.x:.3f},{self.y:.3f})')

    def publish_scan(self):
        count = 1440
        angle_min = -math.pi
        angle_increment = 2.0 * math.pi / count
        ranges = [float('inf')] * count
        obstacle_radius = 0.055
        cosine = math.cos(self.yaw)
        sine = math.sin(self.yaw)
        for _, obstacle_x, obstacle_y in OBSTACLES:
            delta_x = obstacle_x - self.x
            delta_y = obstacle_y - self.y
            center_x = cosine * delta_x + sine * delta_y
            center_y = -sine * delta_x + cosine * delta_y
            center_range = math.hypot(center_x, center_y)
            if center_range <= obstacle_radius or center_range > 9.5:
                continue
            center_angle = math.atan2(center_y, center_x)
            half_span = math.asin(min(0.999, obstacle_radius / center_range))
            first = max(
                0, int((center_angle - half_span - angle_min) / angle_increment) - 1)
            last = min(
                count - 1,
                int((center_angle + half_span - angle_min) / angle_increment) + 1)
            for index in range(first, last + 1):
                angle = angle_min + index * angle_increment
                direction_x = math.cos(angle)
                direction_y = math.sin(angle)
                projection = center_x * direction_x + center_y * direction_y
                discriminant = (
                    projection * projection -
                    (center_range * center_range - obstacle_radius * obstacle_radius))
                if projection <= 0.0 or discriminant < 0.0:
                    continue
                distance = projection - math.sqrt(discriminant)
                if 0.05 <= distance <= 10.0:
                    ranges[index] = min(ranges[index], distance)

        scan = LaserScan()
        # 使用最近一次已进入TF缓存的位姿时刻，避免“scan比最新TF超前几
        # 毫秒”导致专项仿真被误判为缺少采样时刻TF。
        scan.header.stamp = (
            self.get_clock().now() - Duration(seconds=0.03)).to_msg()
        scan.header.frame_id = 'base_link'
        scan.angle_min = angle_min
        scan.angle_max = angle_min + (count - 1) * angle_increment
        scan.angle_increment = angle_increment
        scan.scan_time = 0.05
        scan.range_min = 0.05
        scan.range_max = 10.0
        scan.ranges = ranges
        self.scan_publisher.publish(scan)


def main():
    rclpy.init()
    node = FullTaskSimulator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
