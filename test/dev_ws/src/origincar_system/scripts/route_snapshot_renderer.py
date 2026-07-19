#!/usr/bin/env python3
"""Render logic-3 channel planning snapshots received as compact JSON events."""

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
            depth=32,
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
        logic = int(data.get('logic', 3))
        event = str(data.get('event', ''))
        session = str(data.get('session', ''))
        valid_session = (
            re.fullmatch(r'\d{4}\.\d{2}\.\d{2}_\d{2}:\d{2}:\d{2}', session) or
            re.fullmatch(r'\d{8}_\d{6}', session)
        )
        valid_events = {'selected', 'final'}
        if not valid_session or logic != 3 or event not in valid_events:
            raise ValueError('invalid snapshot event/session')

        output_root = Path(str(data['output_root'])).expanduser()
        output_dir = output_root / session
        output_dir.mkdir(parents=True, exist_ok=True)

        base, world_to_pixel, pixels_per_meter = self._load_map(
            Path(str(data['map_yaml'])), data.get('parking_zone', {}),
            data.get('final_pose'))
        original = data.get('original', [])
        baseline = data.get('baseline', [])
        red = data.get('red', [])
        red_identity_radius = float(data.get('red_identity_radius', 0.5))

        self._render_logic3(
            data, event, output_dir, base, world_to_pixel, pixels_per_meter,
            original, baseline, red, red_identity_radius)

    def _render_logic3(
            self, data, event, output_dir, base, world_to_pixel, pixels_per_meter,
            original, frozen, red, red_identity_radius):
        green = data.get('green', [])
        green_radius = float(data.get('green_radius', 0.5))
        frozen_revision = max(0, int(data.get('frozen_revision', 0)))
        original_path = output_dir / '01_original_route.png'
        initial_frozen_path = output_dir / '02_frozen_route.png'
        frozen_path = (
            initial_frozen_path if frozen_revision == 0 else
            output_dir / f'02_frozen_route（{frozen_revision}）.png'
        )
        final_path = output_dir / '03_final_route.png'

        if event == 'selected':
            # 第0版是二维码后的首次规划，固定写01/02。每次实际倒车完全
            # 结束后的重规划只追加02（n），保留最初的01/02作为对照。
            if frozen_revision == 0 or not original_path.exists():
                image = base.copy()
                self._draw_route(image, original, world_to_pixel, (25, 93, 190, 255), 5)
                self._draw_parking_zone(image, data, world_to_pixel)
                self._draw_title(image, '01 Original unshifted channel route')
                self._save_png(image, original_path)

            image = base.copy()
            self._draw_route(image, original, world_to_pixel, (75, 75, 75, 105), 3)
            self._draw_route(image, frozen, world_to_pixel, (15, 90, 215, 255), 6)
            self._draw_clusters(
                image, green, world_to_pixel, pixels_per_meter,
                green_radius, (20, 180, 65), True)
            self._draw_parking_zone(image, data, world_to_pixel)
            self._draw_title(
                image,
                f'02 Frozen route revision {frozen_revision} / green clusters '
                f'(identity R={green_radius:.2f}m)')
            self._save_png(image, frozen_path)
            shutil.copyfile(frozen_path, final_path)
            self.get_logger().info(f'已生成逻辑3冻结路线图: {frozen_path}')
            return

        image = base.copy()
        self._draw_route(image, original, world_to_pixel, (75, 75, 75, 90), 3)
        self._draw_route(image, frozen, world_to_pixel, (15, 90, 215, 255), 6)
        self._draw_clusters(
            image, green, world_to_pixel, pixels_per_meter,
            green_radius, (20, 180, 65), True)
        if red:
            self._draw_clusters(
                image, red, world_to_pixel, pixels_per_meter,
                red_identity_radius, (225, 35, 35), True)
        self._draw_parking_zone(image, data, world_to_pixel)
        self._draw_final_pose(image, data, world_to_pixel, pixels_per_meter)
        self._draw_title(
            image,
            f'03 Final route / red records (history R={red_identity_radius:.2f}m)')
        self._save_png(image, final_path)
        self.get_logger().info(f'已生成逻辑3任务结束路线图: {final_path}')

    @staticmethod
    def _load_map(map_yaml_path, parking_zone, final_pose):
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
        raw_width = colored.width
        raw_height = colored.height
        pixels_per_meter = scale / resolution
        cos_yaw = math.cos(origin_yaw)
        sin_yaw = math.sin(origin_yaw)

        def raw_world_to_pixel(point):
            dx = float(point[0]) - origin_x
            dy = float(point[1]) - origin_y
            grid_x_m = cos_yaw * dx + sin_yaw * dy
            grid_y_m = -sin_yaw * dx + cos_yaw * dy
            x = grid_x_m * pixels_per_meter
            y = raw_height - 1 - grid_y_m * pixels_per_meter
            return x, y

        padding_left = 0
        padding_top = 0
        padding_right = 0
        padding_bottom = 0
        boundary_pixels = []
        center = parking_zone.get('center', [])
        if isinstance(center, list) and len(center) >= 2:
            center_x = float(center[0])
            center_y = float(center[1])
            length_x = max(0.0, float(parking_zone.get('length_x', 0.0)))
            upper_y = max(0.0, float(parking_zone.get('upper_y', 0.0)))
            lower_y = max(0.0, float(parking_zone.get('lower_y', 0.0)))
            values = (center_x, center_y, length_x, upper_y, lower_y)
            if all(math.isfinite(value) for value in values) and length_x > 0.0:
                half_length = 0.5 * length_x
                corners = [
                    (center_x - half_length, center_y - lower_y),
                    (center_x + half_length, center_y - lower_y),
                    (center_x + half_length, center_y + upper_y),
                    (center_x - half_length, center_y + upper_y),
                ]
                boundary_pixels.extend(raw_world_to_pixel(point) for point in corners)

        if isinstance(final_pose, list) and len(final_pose) >= 3:
            pose_x = float(final_pose[0])
            pose_y = float(final_pose[1])
            pose_yaw = float(final_pose[2])
            if all(math.isfinite(value) for value in (pose_x, pose_y, pose_yaw)):
                marker_radius_m = 0.35
                pose_bounds = [
                    (pose_x - marker_radius_m, pose_y),
                    (pose_x + marker_radius_m, pose_y),
                    (pose_x, pose_y - marker_radius_m),
                    (pose_x, pose_y + marker_radius_m),
                ]
                boundary_pixels.extend(
                    raw_world_to_pixel(point) for point in pose_bounds)

        if boundary_pixels:
            xs = [point[0] for point in boundary_pixels]
            ys = [point[1] for point in boundary_pixels]
            margin = 24
            padding_left = max(0, int(math.ceil(margin - min(xs))))
            padding_top = max(0, int(math.ceil(margin - min(ys))))
            padding_right = max(
                0, int(math.ceil(max(xs) + margin - (raw_width - 1))))
            padding_bottom = max(
                0, int(math.ceil(max(ys) + margin - (raw_height - 1))))

        if any((padding_left, padding_top, padding_right, padding_bottom)):
            colored = ImageOps.expand(
                colored,
                border=(padding_left, padding_top, padding_right, padding_bottom),
                fill=(238, 238, 238, 255))

        def world_to_pixel(point):
            x, y = raw_world_to_pixel(point)
            return x + padding_left, y + padding_top

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
    def _draw_parking_zone(image, data, transform):
        zone = data.get('parking_zone', {})
        center = zone.get('center', [])
        if not isinstance(center, list) or len(center) < 2:
            return

        center_x = float(center[0])
        center_y = float(center[1])
        length_x = max(0.0, float(zone.get('length_x', 0.0)))
        upper_y = max(0.0, float(zone.get('upper_y', 0.0)))
        lower_y = max(0.0, float(zone.get('lower_y', 0.0)))
        values = (center_x, center_y, length_x, upper_y, lower_y)
        if not all(math.isfinite(value) for value in values) or length_x <= 0.0:
            return

        half_length = 0.5 * length_x
        corners = [
            (center_x - half_length, center_y - lower_y),
            (center_x + half_length, center_y - lower_y),
            (center_x + half_length, center_y + upper_y),
            (center_x - half_length, center_y + upper_y),
        ]
        pixels = [transform(point) for point in corners]
        overlay = Image.new('RGBA', image.size, (0, 0, 0, 0))
        draw = ImageDraw.Draw(overlay)
        outline = (135, 35, 190, 245)
        draw.polygon(pixels, fill=(170, 75, 215, 42))
        draw.line(pixels + [pixels[0]], fill=outline, width=5, joint='curve')

        center_pixel_x, center_pixel_y = transform((center_x, center_y))
        cross_radius = 8
        draw.line(
            (center_pixel_x - cross_radius, center_pixel_y,
             center_pixel_x + cross_radius, center_pixel_y),
            fill=outline, width=3)
        draw.line(
            (center_pixel_x, center_pixel_y - cross_radius,
             center_pixel_x, center_pixel_y + cross_radius),
            fill=outline, width=3)

        label = (
            f'PARKING x={center_x:.2f}±{half_length:.2f}, '
            f'y=[{center_y - lower_y:.2f},{center_y + upper_y:.2f}]')
        font = ImageFont.load_default()
        label_x, label_y = pixels[3]
        try:
            box = draw.textbbox((label_x + 7, label_y - 18), label, font=font)
        except (AttributeError, ValueError):
            text_width, text_height = draw.textsize(label, font=font)
            box = (
                label_x + 7, label_y - 18,
                label_x + 7 + text_width, label_y - 18 + text_height)
        draw.rectangle(
            (box[0] - 3, box[1] - 2, box[2] + 3, box[3] + 2),
            fill=(255, 255, 255, 220))
        draw.text((label_x + 7, label_y - 18), label, fill=outline, font=font)
        image.alpha_composite(overlay)

    @staticmethod
    def _draw_final_pose(image, data, transform, pixels_per_meter):
        pose = data.get('final_pose')
        if not isinstance(pose, list) or len(pose) < 3:
            return

        pose_x = float(pose[0])
        pose_y = float(pose[1])
        pose_yaw = float(pose[2])
        if not all(math.isfinite(value) for value in (pose_x, pose_y, pose_yaw)):
            return

        center_x, center_y = transform((pose_x, pose_y))
        arrow_length_m = 0.30
        tip_world = (
            pose_x + arrow_length_m * math.cos(pose_yaw),
            pose_y + arrow_length_m * math.sin(pose_yaw),
        )
        tip_x, tip_y = transform(tip_world)
        vector_x = tip_x - center_x
        vector_y = tip_y - center_y
        vector_length = max(1.0, math.hypot(vector_x, vector_y))
        unit_x = vector_x / vector_length
        unit_y = vector_y / vector_length
        perpendicular_x = -unit_y
        perpendicular_y = unit_x
        head_length = min(14.0, 0.35 * arrow_length_m * pixels_per_meter)
        head_half_width = min(8.0, 0.20 * arrow_length_m * pixels_per_meter)
        arrow_head = [
            (tip_x, tip_y),
            (tip_x - head_length * unit_x + head_half_width * perpendicular_x,
             tip_y - head_length * unit_y + head_half_width * perpendicular_y),
            (tip_x - head_length * unit_x - head_half_width * perpendicular_x,
             tip_y - head_length * unit_y - head_half_width * perpendicular_y),
        ]

        overlay = Image.new('RGBA', image.size, (0, 0, 0, 0))
        draw = ImageDraw.Draw(overlay)
        color = (0, 105, 225, 255)
        draw.line((center_x, center_y, tip_x, tip_y), fill=color, width=6)
        draw.polygon(arrow_head, fill=color)
        center_radius = 8
        draw.ellipse(
            (center_x - center_radius, center_y - center_radius,
             center_x + center_radius, center_y + center_radius),
            fill=color, outline=(255, 255, 255, 255), width=2)

        yaw_degrees = math.degrees(pose_yaw)
        label = (
            f'FINAL base_link=({pose_x:.3f},{pose_y:.3f}), '
            f'yaw={pose_yaw:.3f}rad/{yaw_degrees:.1f}deg')
        font = ImageFont.load_default()
        label_x = center_x + 11
        label_y = center_y - 24
        try:
            box = draw.textbbox((label_x, label_y), label, font=font)
        except (AttributeError, ValueError):
            text_width, text_height = draw.textsize(label, font=font)
            box = (label_x, label_y, label_x + text_width, label_y + text_height)
        draw.rectangle(
            (box[0] - 3, box[1] - 2, box[2] + 3, box[3] + 2),
            fill=(255, 255, 255, 225))
        draw.text((label_x, label_y), label, fill=color, font=font)
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
