#!/usr/bin/env python3
"""Cache SZC logic-1 route updates and render PNG files after route finish."""

import math
import os
from datetime import datetime
from pathlib import Path

import rclpy
from nav_msgs.msg import Path as PathMessage
from PIL import Image, ImageDraw, ImageFont, ImageOps
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
import yaml


class SzcRouteSnapshotRenderer(Node):
    _SHIFT_EPSILON_METERS = 1e-5
    _FINISH_RENDER_DELAY_SECONDS = 0.25
    _MAX_FINISH_WAIT_TICKS = 8

    def __init__(self):
        super().__init__('szc_route_snapshot_renderer')

        try:
            nice_value = os.nice(15)
            self.get_logger().info(f'SZC PNG 渲染进程 nice={nice_value}')
        except OSError as exc:
            self.get_logger().warning(f'无法降低 SZC PNG 渲染进程优先级: {exc}')

        path_topic = str(self.declare_parameter(
            'path_topic', '/visual_lidar_path_follower_szc/debug_path').value)
        finish_topic = str(self.declare_parameter(
            'finish_topic',
            '/visual_lidar_path_follower_szc/route_finished').value)
        self._map_yaml = Path(str(
            self.declare_parameter('map_yaml', '').value)).expanduser()
        self._output_root = Path(str(self.declare_parameter(
            'output_root',
            '/root/D_Robot/test/dev_ws/src/origincar_system/path/szc').value)).expanduser()

        if not self._map_yaml.is_file():
            raise FileNotFoundError(f'地图 YAML 不存在: {self._map_yaml}')

        session = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._session_dir = self._output_root / session
        self._base = None
        self._world_to_pixel = None
        self._original_route = None
        self._historical_shifted_route = None
        self._finish_received = False
        self._finish_wait_ticks = 0
        self._finish_timer = None
        self._rendered = False

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._path_subscription = self.create_subscription(
            PathMessage, path_topic, self._on_path, qos)
        self._finish_subscription = self.create_subscription(
            Bool, finish_topic, self._on_route_finished, qos)
        self.get_logger().info(
            'SZC PNG 节点运行中仅缓存路径；环线完成后才加载地图并写入 '
            f'{self._session_dir}')

    def _on_path(self, message):
        if self._rendered:
            return
        route = [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in message.poses
        ]
        if len(route) < 2:
            return

        if (self._original_route is None or
                len(route) != len(self._original_route)):
            if self._original_route is not None:
                self.get_logger().info(
                    '检测到路线长度变化，已按新路线重新开始缓存。')
            self._original_route = list(route)
            self._historical_shifted_route = list(route)
            return

        # 逻辑1会在车辆通过后把当前路线释放回原坐标。这里仅在某个 waypoint
        # 当前仍有非零偏移时覆盖历史值，因此最终图保留它最后一次确认过的方向和偏移量。
        epsilon_squared = self._SHIFT_EPSILON_METERS ** 2
        for index, (current, original) in enumerate(
                zip(route, self._original_route)):
            dx = current[0] - original[0]
            dy = current[1] - original[1]
            if dx * dx + dy * dy > epsilon_squared:
                self._historical_shifted_route[index] = current

    def _on_route_finished(self, message):
        if not message.data or self._finish_received or self._rendered:
            return

        self._finish_received = True
        self.get_logger().info(
            '收到环线完成信号，等待最后一帧路径后开始生成 PNG。')
        self._finish_timer = self.create_timer(
            self._FINISH_RENDER_DELAY_SECONDS, self._render_after_finish)

    def _render_after_finish(self):
        if self._rendered:
            self._cancel_finish_timer()
            return
        if self._original_route is None or self._historical_shifted_route is None:
            self._finish_wait_ticks += 1
            if self._finish_wait_ticks < self._MAX_FINISH_WAIT_TICKS:
                return
            self._cancel_finish_timer()
            self.get_logger().error('环线已完成，但未收到可绘制的 SZC 路径。')
            return

        self._cancel_finish_timer()
        try:
            self._render_cached_routes()
            self._rendered = True
        except Exception as exc:  # 绘图失败不能影响已经完成的控制节点。
            self.get_logger().error(f'SZC 路径 PNG 保存失败: {exc}')

    def _cancel_finish_timer(self):
        if self._finish_timer is not None:
            self._finish_timer.cancel()

    def _render_cached_routes(self):
        # 地图读取、放大、Pillow 绘图和磁盘写入全部延迟到路线完成之后。
        self._base, self._world_to_pixel = self._load_map(self._map_yaml)
        self._session_dir.mkdir(parents=True, exist_ok=True)
        self._output_root.mkdir(parents=True, exist_ok=True)

        original_image = self._base.copy()
        self._draw_route(
            original_image, self._original_route, (25, 93, 190, 255), 5)
        self._draw_points(
            original_image, self._original_route, (25, 93, 190, 255))
        self._draw_title(original_image, 'SZC original route')
        self._save_png(
            original_image, self._session_dir / '01_original_route.png')

        image = self._base.copy()
        self._draw_route(image, self._original_route, (70, 70, 70, 100), 3)
        self._draw_route(
            image, self._historical_shifted_route, (255, 140, 25, 255), 7)
        self._draw_points(
            image, self._historical_shifted_route, (255, 110, 10, 255))
        self._draw_title(image, 'SZC final confirmed logic-1 shifted route')

        session_path = self._session_dir / '02_final_shifted_route.png'
        latest_path = self._output_root / 'latest_final_shifted_route.png'
        self._save_png(image, session_path)
        self._save_png(image, latest_path)
        self.get_logger().info(
            f'环线完成，已生成 SZC 最终偏移路径 PNG: {session_path}')

    @staticmethod
    def _load_map(map_yaml_path):
        with map_yaml_path.open('r', encoding='utf-8') as stream:
            metadata = yaml.safe_load(stream)

        image_path = Path(str(metadata['image']))
        if not image_path.is_absolute():
            image_path = map_yaml_path.parent / image_path
        resolution = float(metadata['resolution'])
        origin_x, origin_y, origin_yaw = [float(value) for value in metadata['origin']]

        gray = Image.open(image_path).convert('L')
        colored = ImageOps.colorize(
            gray, black='#cfe5c6', white='#ffe8a3').convert('RGBA')
        scale = 5
        resampling = getattr(Image, 'Resampling', Image)
        colored = colored.resize(
            (colored.width * scale, colored.height * scale), resampling.NEAREST)
        pixels_per_meter = scale / resolution
        cos_yaw = math.cos(origin_yaw)
        sin_yaw = math.sin(origin_yaw)

        def world_to_pixel(point):
            dx = float(point[0]) - origin_x
            dy = float(point[1]) - origin_y
            grid_x_m = cos_yaw * dx + sin_yaw * dy
            grid_y_m = -sin_yaw * dx + cos_yaw * dy
            return (
                grid_x_m * pixels_per_meter,
                colored.height - 1 - grid_y_m * pixels_per_meter,
            )

        return colored, world_to_pixel

    def _draw_route(self, image, route, color, width):
        if len(route) < 2:
            return
        overlay = Image.new('RGBA', image.size, (0, 0, 0, 0))
        draw = ImageDraw.Draw(overlay)
        draw.line(
            [self._world_to_pixel(point) for point in route],
            fill=color, width=width, joint='curve')
        image.alpha_composite(overlay)

    def _draw_points(self, image, route, color):
        overlay = Image.new('RGBA', image.size, (0, 0, 0, 0))
        draw = ImageDraw.Draw(overlay)
        radius = 4
        for point in route:
            x, y = self._world_to_pixel(point)
            draw.ellipse(
                (x - radius, y - radius, x + radius, y + radius),
                fill=color)
        image.alpha_composite(overlay)

    @staticmethod
    def _draw_title(image, title):
        overlay = Image.new('RGBA', image.size, (0, 0, 0, 0))
        draw = ImageDraw.Draw(overlay)
        font = ImageFont.load_default()
        try:
            box = draw.textbbox((0, 0), title, font=font)
            width = box[2] - box[0]
            height = box[3] - box[1]
        except (AttributeError, ValueError):
            width, height = draw.textsize(title, font=font)
        draw.rectangle((10, 10, width + 22, height + 20), fill=(255, 255, 255, 210))
        draw.text((16, 14), title, fill=(20, 20, 20, 255), font=font)
        image.alpha_composite(overlay)

    @staticmethod
    def _save_png(image, path):
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_name(f'.{path.stem}.tmp{path.suffix}')
        image.convert('RGB').save(temporary, format='PNG')
        temporary.replace(path)


def main(args=None):
    rclpy.init(args=args)
    node = SzcRouteSnapshotRenderer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
