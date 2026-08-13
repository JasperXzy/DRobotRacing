#!/usr/bin/env python3
"""Render the four-stage route snapshots received as compact JSON events."""

import json
import math
import re
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
import yaml
from PIL import Image, ImageColor, ImageDraw, ImageFont, ImageOps


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
        event = str(data.get('event', ''))
        session = str(data.get('session', ''))
        valid_session = (
            re.fullmatch(r'\d{4}\.\d{2}\.\d{2}_\d{2}:\d{2}:\d{2}', session) or
            re.fullmatch(r'\d{8}_\d{6}', session)
        )
        valid_events = {
            'task1_frozen', 'task1_dynamic',
            'final_frozen', 'final_dynamic',
            'task1_before_red', 'final_before_red',
        }
        if not valid_session or event not in valid_events:
            raise ValueError('invalid snapshot event/session')

        output_root = Path(str(data['output_root'])).expanduser()
        if re.fullmatch(r'\d{4}\.\d{2}\.\d{2}_\d{2}:\d{2}:\d{2}', session):
            day = session[:10]
        else:
            day = f'{session[:4]}.{session[4:6]}.{session[6:8]}'
        # 每天一个一级目录；二级目录继续保留完整发车时间戳，确保同一天
        # 多次发车互不覆盖，并让二维码/图生文节点能锁定同一批次。
        output_dir = output_root / day / session
        output_dir.mkdir(parents=True, exist_ok=True)

        base, world_to_pixel, pixels_per_meter = self._load_map(
            Path(str(data['map_yaml'])), data.get('base_zone', {}),
            data.get('parking_zone', {}))
        original = data.get('original', [])
        baseline = data.get('baseline', [])
        red = data.get('red', [])
        red_identity_radius = float(data.get('red_identity_radius', 0.25))
        red_filter_radius = float(data.get('red_filter_radius', 0.50))
        red_radius = float(data.get('red_radius', 0.35))

        self._render_route_snapshot(
            data, event, output_dir, base, world_to_pixel, pixels_per_meter,
            original, baseline, red, red_identity_radius, red_filter_radius,
            red_radius)

    def _render_route_snapshot(
            self, data, event, output_dir, base, world_to_pixel, pixels_per_meter,
            original, frozen, red, red_identity_radius, red_filter_radius,
            red_radius):
        green = data.get('green', [])
        try:
            orange_begin = int(data.get('orange_green_begin_index', len(green)))
        except (TypeError, ValueError):
            orange_begin = len(green)
        orange_begin = max(0, min(orange_begin, len(green)))
        departure_green = green[:orange_begin]
        post_departure_orange = green[orange_begin:]
        green_radius = float(data.get('green_radius', 0.50))
        green_identity_radius = float(
            data.get('green_identity_radius', 0.25))
        actual_route = data.get('actual_route', [])
        actual_route_enabled = bool(data.get('actual_route_enabled', False))
        actual_route_width = max(1, min(
            30, int(data.get('actual_route_line_width', 5))))
        try:
            actual_route_color = ImageColor.getcolor(
                str(data.get('actual_route_color', '#d020d0')), 'RGBA')
        except (TypeError, ValueError):
            actual_route_color = (208, 32, 208, 255)
            self.get_logger().warning(
                '实际路线颜色无效，改用默认颜色 #d020d0')
        frozen_revision = max(0, int(data.get('frozen_revision', 0)))
        if event == 'task1_frozen':
            image = base.copy()
            self._draw_task3_slowdown_zone(image, data, world_to_pixel)
            self._draw_route(image, original, world_to_pixel, (75, 75, 75, 105), 3)
            self._draw_route(image, frozen, world_to_pixel, (15, 90, 215, 255), 6)
            self._draw_clusters(
                image, departure_green, world_to_pixel, pixels_per_meter,
                green_radius, (20, 180, 65), True)
            self._draw_clusters(
                image, post_departure_orange, world_to_pixel, pixels_per_meter,
                green_radius, (245, 140, 20), True)
            self._draw_base_zone(image, data, world_to_pixel)
            self._draw_parking_zone(image, data, world_to_pixel)
            self._draw_title(
                image,
                f'Task1 frozen route / green at departure, orange learned after departure '
                f'(green/orange filter/draw R={green_radius:.2f}m, '
                f'association R={green_identity_radius:.2f}m)')
            output_path = output_dir / '01_Task1_frozen_route.png'
            self._save_png(image, output_path)
            self.get_logger().debug(f'已生成Task1冻结路线图: {output_path}')
            return

        if event == 'final_frozen':
            image = base.copy()
            self._draw_task3_slowdown_zone(image, data, world_to_pixel)
            self._draw_route(image, original, world_to_pixel, (75, 75, 75, 105), 3)
            self._draw_route(image, frozen, world_to_pixel, (15, 90, 215, 255), 6)
            self._draw_clusters(
                image, departure_green, world_to_pixel, pixels_per_meter,
                green_radius, (20, 180, 65), True)
            self._draw_clusters(
                image, post_departure_orange, world_to_pixel, pixels_per_meter,
                green_radius, (245, 140, 20), True)
            self._draw_base_zone(image, data, world_to_pixel)
            self._draw_parking_zone(image, data, world_to_pixel)
            self._draw_snapshot_markers(image, data, world_to_pixel)
            self._draw_title(
                image,
                f'Final frozen route revision {frozen_revision} / '
                f'green at departure, orange learned after departure '
                f'(green/orange filter/draw R={green_radius:.2f}m, '
                f'association R={green_identity_radius:.2f}m)')
            output_path = (
                output_dir / '03_Final_frozen_route.png'
                if frozen_revision == 0 else
                output_dir / f'03_Final_frozen_route({frozen_revision}).png'
            )
            self._save_png(image, output_path)
            self.get_logger().debug(f'已生成Final冻结路线图: {output_path}')
            return

        image = base.copy()
        self._draw_task3_slowdown_zone(image, data, world_to_pixel)
        self._draw_route(image, original, world_to_pixel, (75, 75, 75, 90), 3)
        self._draw_route(image, frozen, world_to_pixel, (15, 90, 215, 255), 6)
        if actual_route_enabled and event == 'final_dynamic':
            self._draw_route(
                image, actual_route, world_to_pixel,
                actual_route_color, actual_route_width)
        self._draw_clusters(
            image, departure_green, world_to_pixel, pixels_per_meter,
            green_radius, (20, 180, 65), True)
        self._draw_clusters(
            image, post_departure_orange, world_to_pixel, pixels_per_meter,
            green_radius, (245, 140, 20), True)
        if red:
            self._draw_clusters(
                image, red, world_to_pixel, pixels_per_meter,
                red_radius, (225, 35, 35), True)
        self._draw_base_zone(image, data, world_to_pixel)
        self._draw_parking_zone(image, data, world_to_pixel)
        self._draw_snapshot_markers(image, data, world_to_pixel)
        self._draw_final_pose(image, data, world_to_pixel, pixels_per_meter)
        task1_dynamic = event in ('task1_dynamic', 'task1_before_red')
        before_red = event in ('task1_before_red', 'final_before_red')
        red_trigger_index = max(0, int(data.get('red_trigger_index', 0)))
        self._draw_title(
            image,
            f'{"Task1" if task1_dynamic else "Final"} dynamic route / '
            f'{"actual base_link trace / " if event == "final_dynamic" and actual_route_enabled else ""}'
            f'green at departure, orange learned after departure, '
            f'red actual lidar avoidance '
            f'(green/orange filter/draw R={green_radius:.2f}m, '
            f'red draw/takeover R={red_radius:.2f}m, '
            f'green/orange identity R={green_identity_radius:.2f}m, '
            f'red identity R={red_identity_radius:.2f}m, '
            f'red dedup R={red_filter_radius:.2f}m)')
        if before_red:
            if red_trigger_index <= 0:
                raise ValueError('invalid red trigger index')
            output_path = output_dir / (
                f'02_Task1_dynamic_route_before_red({red_trigger_index}).png'
                if task1_dynamic else
                f'04_Final_dynamic_route_before_red({red_trigger_index}).png')
        else:
            output_path = output_dir / (
                '02_Task1_dynamic_route.png'
                if task1_dynamic else '04_Final_dynamic_route.png')
        self._save_png(image, output_path)
        self.get_logger().debug(f'已生成动态路线图: {output_path}')

    @staticmethod
    def _load_map(map_yaml_path, base_zone, parking_zone):
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
        for zone in (base_zone, parking_zone):
            bounds = RouteSnapshotRenderer._zone_bounds(zone)
            if bounds is None:
                continue
            min_x, max_x, min_y, max_y = bounds
            boundary_pixels.extend(
                raw_world_to_pixel(point) for point in (
                    (min_x, min_y), (max_x, min_y),
                    (max_x, max_y), (min_x, max_y)))

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
    def _zone_bounds(zone):
        if not isinstance(zone, dict):
            return None
        try:
            min_x = float(zone['min_x'])
            max_x = float(zone['max_x'])
            min_y = float(zone['min_y'])
            max_y = float(zone['max_y'])
        except (KeyError, TypeError, ValueError):
            return None
        values = (min_x, max_x, min_y, max_y)
        if not all(math.isfinite(value) for value in values):
            return None
        if min_x > max_x or min_y > max_y:
            return None
        return values

    @staticmethod
    def _draw_zone(image, transform, bounds, outline, fill, label):
        if bounds is None:
            return
        min_x, max_x, min_y, max_y = bounds
        corners = [
            (min_x, min_y), (max_x, min_y),
            (max_x, max_y), (min_x, max_y),
        ]
        pixels = [transform(point) for point in corners]
        overlay = Image.new('RGBA', image.size, (0, 0, 0, 0))
        draw = ImageDraw.Draw(overlay)
        draw.polygon(pixels, fill=fill)
        draw.line(pixels + [pixels[0]], fill=outline, width=5, joint='curve')
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
    def _draw_base_zone(image, data, transform):
        bounds = RouteSnapshotRenderer._zone_bounds(data.get('base_zone', {}))
        if bounds is None:
            return
        min_x, max_x, min_y, max_y = bounds
        label = (
            f'BASE x=[{min_x:.2f},{max_x:.2f}], '
            f'y=[{min_y:.2f},{max_y:.2f}]')
        RouteSnapshotRenderer._draw_zone(
            image, transform, bounds,
            (25, 105, 225, 245), (45, 125, 235, 28), label)

    @staticmethod
    def _draw_parking_zone(image, data, transform):
        zone = data.get('parking_zone', {})
        bounds = RouteSnapshotRenderer._zone_bounds(zone)
        if bounds is None:
            return
        min_x, max_x, min_y, max_y = bounds
        confirm_seconds = max(0.0, float(zone.get('confirm_seconds', 0.0)))
        label = (
            f'PARKING x=[{min_x:.2f},{max_x:.2f}], '
            f'y=[{min_y:.2f},{max_y:.2f}], hold={confirm_seconds:.2f}s')
        RouteSnapshotRenderer._draw_zone(
            image, transform, bounds,
            (135, 35, 190, 245), (170, 75, 215, 42), label)

    @staticmethod
    def _draw_task3_slowdown_zone(image, data, transform):
        """Draw the unbounded x<max_x, y<start_y region clipped to the map canvas."""
        zone = data.get('task3_slowdown_zone', {})
        if not isinstance(zone, dict) or not bool(zone.get('enabled', False)):
            return
        try:
            max_x = float(zone['max_x'])
            start_y = float(zone['start_y'])
            end_y = float(zone['end_y'])
            start_speed = float(zone['start_speed'])
            min_speed = float(zone['min_speed'])
        except (KeyError, TypeError, ValueError):
            return
        if not all(math.isfinite(value) for value in (
                max_x, start_y, end_y, start_speed, min_speed)):
            return

        # 当前比赛map不旋转；区域的负无穷边界以最终图片左边和下边裁剪。
        limit_x, start_pixel_y = transform((max_x, start_y))
        _, end_pixel_y = transform((max_x, end_y))
        limit_x = max(0.0, min(float(image.width - 1), limit_x))
        start_pixel_y = max(0.0, min(float(image.height - 1), start_pixel_y))
        end_pixel_y = max(0.0, min(float(image.height - 1), end_pixel_y))

        overlay = Image.new('RGBA', image.size, (0, 0, 0, 0))
        draw = ImageDraw.Draw(overlay)
        # 粉色半透明区域始终放在路线/障碍物下层，不遮挡蓝线和点云标记。
        draw.rectangle(
            (0, start_pixel_y, limit_x, image.height - 1),
            fill=(255, 80, 170, 48), outline=(230, 45, 145, 235), width=4)
        # y=end_y是达到最低速度的位置，用更深的粉色实线明确标出。
        draw.line(
            ((0, end_pixel_y), (limit_x, end_pixel_y)),
            fill=(205, 25, 125, 235), width=4)

        label = (
            f'TASK3 SLOWDOWN x<{max_x:.2f}, y<{start_y:.2f}: '
            f'{start_speed:.2f}->{min_speed:.2f}m/s @ y={end_y:.2f}')
        font = ImageFont.load_default()
        label_x = 8
        label_y = min(float(image.height - 18), start_pixel_y + 8)
        try:
            box = draw.textbbox((label_x, label_y), label, font=font)
        except (AttributeError, ValueError):
            text_width, text_height = draw.textsize(label, font=font)
            box = (
                label_x, label_y,
                label_x + text_width, label_y + text_height)
        draw.rectangle(
            (box[0] - 3, box[1] - 2, box[2] + 3, box[3] + 2),
            fill=(255, 245, 250, 225))
        draw.text(
            (label_x, label_y), label,
            fill=(180, 20, 105, 255), font=font)
        image.alpha_composite(overlay)

    @staticmethod
    def _draw_marker(image, marker, transform, color, text_color):
        if not isinstance(marker, dict):
            return
        point = marker.get('point')
        if not isinstance(point, list) or len(point) < 2:
            return
        point_x = float(point[0])
        point_y = float(point[1])
        if not all(math.isfinite(value) for value in (point_x, point_y)):
            return

        x, y = transform((point_x, point_y))
        overlay = Image.new('RGBA', image.size, (0, 0, 0, 0))
        draw = ImageDraw.Draw(overlay)
        radius = 9
        draw.ellipse(
            (x - radius, y - radius, x + radius, y + radius),
            fill=color, outline=(255, 255, 255, 255), width=3)
        label = str(marker.get('label', 'EVENT'))
        text = f'{label} ({point_x:.2f},{point_y:.2f})'
        font = ImageFont.load_default()
        draw.text((x + 12, y - 15), text, fill=text_color, font=font)
        image.alpha_composite(overlay)

    @staticmethod
    def _draw_route_start_heading(image, marker, transform):
        if not isinstance(marker, dict):
            return
        point = marker.get('point')
        if not isinstance(point, list) or len(point) < 2:
            return
        try:
            point_x = float(point[0])
            point_y = float(point[1])
            point_yaw = float(marker.get('yaw'))
            heading_distance = float(marker.get('heading_distance', 0.30))
        except (TypeError, ValueError):
            return
        if not all(math.isfinite(value) for value in (
                point_x, point_y, point_yaw, heading_distance)):
            return
        heading_distance = max(0.01, heading_distance)

        start_x, start_y = transform((point_x, point_y))
        tip_x, tip_y = transform((
            point_x + heading_distance * math.cos(point_yaw),
            point_y + heading_distance * math.sin(point_yaw),
        ))
        vector_x = tip_x - start_x
        vector_y = tip_y - start_y
        vector_length = max(1.0, math.hypot(vector_x, vector_y))
        unit_x = vector_x / vector_length
        unit_y = vector_y / vector_length
        perpendicular_x = -unit_y
        perpendicular_y = unit_x
        head_length = min(14.0, 0.30 * vector_length)
        head_half_width = min(8.0, 0.16 * vector_length)
        arrow_head = [
            (tip_x, tip_y),
            (tip_x - head_length * unit_x + head_half_width * perpendicular_x,
             tip_y - head_length * unit_y + head_half_width * perpendicular_y),
            (tip_x - head_length * unit_x - head_half_width * perpendicular_x,
             tip_y - head_length * unit_y - head_half_width * perpendicular_y),
        ]
        overlay = Image.new('RGBA', image.size, (0, 0, 0, 0))
        draw = ImageDraw.Draw(overlay)
        color = (20, 100, 235, 255)
        draw.line((start_x, start_y, tip_x, tip_y), fill=color, width=5)
        draw.polygon(arrow_head, fill=color)
        image.alpha_composite(overlay)

    @staticmethod
    def _draw_snapshot_markers(image, data, transform):
        RouteSnapshotRenderer._draw_marker(
            image, data.get('qr_marker'), transform,
            (245, 135, 25, 255), (120, 60, 0, 255))
        RouteSnapshotRenderer._draw_route_start_heading(
            image, data.get('route_start_marker'), transform)
        RouteSnapshotRenderer._draw_marker(
            image, data.get('route_start_marker'), transform,
            (20, 100, 235, 255), (0, 45, 130, 255))

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
