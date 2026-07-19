#!/usr/bin/env python3
"""Render logic-2/3 channel planning snapshots received as compact JSON events."""

import json
import math
import re
import shutil
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
import yaml
from PIL import Image, ImageDraw, ImageFont, ImageOps


class RouteSnapshotRenderer(Node):
    def __init__(self):
        super().__init__('route_snapshot_renderer')
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=3,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            String,
            '/visual_lidar_path_follower/route_snapshot',
            self._on_snapshot,
            qos,
        )

    def _on_snapshot(self, message):
        try:
            data = json.loads(message.data)
            self._render(data)
        except Exception as exc:  # Keep the control node independent from plot failures.
            self.get_logger().error(f'路线快照绘图失败: {exc}')

    def _render(self, data):
        logic = int(data.get('logic', 2))
        event = str(data.get('event', ''))
        session = re.sub(r'[^0-9_]', '', str(data.get('session', '')))
        valid_events = {'selected', 'final'}
        if not session or logic not in {2, 3} or event not in valid_events:
            raise ValueError('invalid snapshot event/session')

        output_root = Path(str(data['output_root'])).expanduser()
        output_dir = output_root / session
        output_dir.mkdir(parents=True, exist_ok=True)

        base, world_to_pixel, pixels_per_meter = self._load_map(Path(str(data['map_yaml'])))
        original = data.get('original', [])
        baseline = data.get('baseline', [])
        route = data.get('route', [])
        red = data.get('red', [])
        red_identity_radius = float(data.get('red_identity_radius', 0.5))

        if logic == 3:
            self._render_logic3(
                data, event, output_dir, base, world_to_pixel, pixels_per_meter,
                original, baseline, route, red, red_identity_radius)
            return

        original_path = output_dir / '01_original_route.png'
        final_path = output_dir / '02_final_dynamic_route.png'

        if event == 'selected':
            image = base.copy()
            self._draw_route(image, original, world_to_pixel, (25, 93, 190, 255), 5)
            self._draw_title(image, '01 Original unshifted channel route')
            self._save_png(image, original_path)

            image = base.copy()
            self._draw_route(image, original, world_to_pixel, (75, 75, 75, 105), 3)
            self._draw_route(image, baseline, world_to_pixel, (15, 90, 215, 255), 6)
            self._draw_title(image, '02 Final dynamic route (waiting for task completion)')
            self._save_png(image, final_path)
            self.get_logger().info(f'已生成逻辑2原始路线图: {original_path}')
            return

        image = base.copy()
        self._draw_route(image, original, world_to_pixel, (75, 75, 75, 90), 3)
        self._draw_route(image, baseline, world_to_pixel, (15, 90, 215, 255), 6)
        if red:
            self._draw_route_changes(
                image, baseline, route, world_to_pixel, (215, 40, 40, 255), 7)
            self._draw_clusters(
                image, red, world_to_pixel, pixels_per_meter,
                red_identity_radius, (225, 35, 35), True)
        self._draw_title(
            image,
            f'02 Final dynamic route / red clusters (history R={red_identity_radius:.2f}m)')
        self._save_png(image, final_path)
        self.get_logger().info(f'已生成逻辑2任务结束路线图: {final_path}')

    def _render_logic3(
            self, data, event, output_dir, base, world_to_pixel, pixels_per_meter,
            original, frozen, route, red, red_identity_radius):
        green = data.get('green', [])
        green_radius = float(data.get('green_radius', 0.5))
        original_path = output_dir / '01_original_route.png'
        frozen_path = output_dir / '02_frozen_route.png'
        final_path = output_dir / '03_final_route.png'

        if event == 'selected':
            image = base.copy()
            self._draw_route(image, original, world_to_pixel, (25, 93, 190, 255), 5)
            self._draw_title(image, '01 Original unshifted channel route')
            self._save_png(image, original_path)

            image = base.copy()
            self._draw_route(image, original, world_to_pixel, (75, 75, 75, 105), 3)
            self._draw_route(image, frozen, world_to_pixel, (15, 90, 215, 255), 6)
            self._draw_clusters(
                image, green, world_to_pixel, pixels_per_meter,
                green_radius, (20, 180, 65), True)
            self._draw_title(
                image, f'02 Frozen route / green clusters (identity R={green_radius:.2f}m)')
            self._save_png(image, frozen_path)
            shutil.copyfile(frozen_path, final_path)
            self.get_logger().info(f'已生成逻辑3扫码方向冻结路线图: {output_dir}')
            return

        if not red and frozen_path.exists():
            shutil.copyfile(frozen_path, final_path)
        else:
            image = base.copy()
            self._draw_route(image, original, world_to_pixel, (75, 75, 75, 90), 3)
            self._draw_route(image, frozen, world_to_pixel, (15, 90, 215, 255), 6)
            self._draw_route_changes(
                image, frozen, route, world_to_pixel, (215, 40, 40, 255), 7)
            self._draw_clusters(
                image, green, world_to_pixel, pixels_per_meter,
                green_radius, (20, 180, 65), True)
            self._draw_clusters(
                image, red, world_to_pixel, pixels_per_meter,
                red_identity_radius, (225, 35, 35), True)
            self._draw_title(
                image,
                f'03 Final route / red clusters (history R={red_identity_radius:.2f}m)')
            self._save_png(image, final_path)
        self.get_logger().info(f'已生成逻辑3任务结束路线图: {final_path}')

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
        # race_keepout: restricted area is pale green, traversable channel is pale yellow.
        colored = ImageOps.colorize(gray, black='#cfe5c6', white='#ffe8a3').convert('RGBA')
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
            x = grid_x_m * pixels_per_meter
            y = colored.height - 1 - grid_y_m * pixels_per_meter
            return x, y

        return colored, world_to_pixel, pixels_per_meter

    @staticmethod
    def _draw_route(image, route, transform, color, width):
        if len(route) < 2:
            return
        overlay = Image.new('RGBA', image.size, (0, 0, 0, 0))
        draw = ImageDraw.Draw(overlay)
        draw.line([transform(point) for point in route], fill=color, width=width, joint='curve')
        image.alpha_composite(overlay)

    @staticmethod
    def _draw_route_changes(image, baseline, route, transform, color, width):
        if len(route) < 2 or len(route) != len(baseline):
            RouteSnapshotRenderer._draw_route(image, route, transform, color, width)
            return
        overlay = Image.new('RGBA', image.size, (0, 0, 0, 0))
        draw = ImageDraw.Draw(overlay)
        for index in range(len(route) - 1):
            changed = any(
                math.hypot(
                    float(route[point_index][0]) - float(baseline[point_index][0]),
                    float(route[point_index][1]) - float(baseline[point_index][1]),
                ) > 0.005
                for point_index in (index, index + 1)
            )
            if changed:
                draw.line(
                    [transform(route[index]), transform(route[index + 1])],
                    fill=color, width=width)
        image.alpha_composite(overlay)

    @staticmethod
    def _draw_clusters(
            image, clusters, transform, pixels_per_meter, radius_m, color, draw_radius):
        overlay = Image.new('RGBA', image.size, (0, 0, 0, 0))
        draw = ImageDraw.Draw(overlay)
        font = ImageFont.load_default()
        for point in clusters:
            x, y = transform(point)
            if draw_radius and radius_m > 0.0:
                radius_px = radius_m * pixels_per_meter
                draw.ellipse(
                    (x - radius_px, y - radius_px, x + radius_px, y + radius_px),
                    fill=(*color, 38), outline=(*color, 190), width=3)
            center_radius = 7
            draw.ellipse(
                (x - center_radius, y - center_radius, x + center_radius, y + center_radius),
                fill=(*color, 255), outline=(255, 255, 255, 255), width=2)
            label = f'({float(point[0]):.2f},{float(point[1]):.2f})'
            draw.text((x + 9, y - 14), label, fill=(25, 25, 25, 255), font=font)
        image.alpha_composite(overlay)

    @staticmethod
    def _draw_title(image, title):
        overlay = Image.new('RGBA', image.size, (0, 0, 0, 0))
        draw = ImageDraw.Draw(overlay)
        font = ImageFont.load_default()
        try:
            box = draw.textbbox((0, 0), title, font=font)
            text_width, text_height = box[2] - box[0], box[3] - box[1]
        except (AttributeError, ValueError):
            text_width, text_height = draw.textsize(title, font=font)
        draw.rectangle(
            (10, 10, text_width + 22, text_height + 20),
            fill=(255, 255, 255, 205))
        draw.text((16, 14), title, fill=(20, 20, 20, 255), font=font)
        image.alpha_composite(overlay)

    @staticmethod
    def _save_png(image, path):
        image.convert('RGB').save(path)


def main(args=None):
    rclpy.init(args=args)
    node = RouteSnapshotRenderer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
