#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Sequence


PACKAGE_DIR = Path(__file__).resolve().parents[1]
DEFAULT_CLOCKWISE_OUTPUT = PACKAGE_DIR / "config" / "waypoints_channel_clockwise.yaml"
DEFAULT_COUNTERCLOCKWISE_OUTPUT = (
    PACKAGE_DIR / "config" / "waypoints_channel_counterclockwise.yaml"
)
DEFAULT_MAP_YAML = PACKAGE_DIR / "map" / "race_keepout.yaml"
DEFAULT_CLOCKWISE_PREVIEW_OUTPUT = (
    PACKAGE_DIR / "map" / "waypoints_channel_clockwise_on_restriction_map.png"
)
DEFAULT_COUNTERCLOCKWISE_PREVIEW_OUTPUT = (
    PACKAGE_DIR / "map" / "waypoints_channel_counterclockwise_on_restriction_map.png"
)
DEFAULT_SCHEMATIC_BACKGROUND = (
    PACKAGE_DIR / "map" / "waypoints_task1_on_schematic.png"
)
DEFAULT_CLOCKWISE_SCHEMATIC_PREVIEW_OUTPUT = (
    PACKAGE_DIR / "map" / "waypoints_channel_clockwise_on_schematic.png"
)
DEFAULT_COUNTERCLOCKWISE_SCHEMATIC_PREVIEW_OUTPUT = (
    PACKAGE_DIR / "map" / "waypoints_channel_counterclockwise_on_schematic.png"
)

Point = tuple[float, float]

# channel 环线的原入口/出口交点，以及Task3正常规划路线的原末点。
CHANNEL_LOOP_ENDPOINT: Point = (1.95, 1.80)
RETURN_ROUTE_ENDPOINT: Point = (0.00, 0.00)

# 出通道到Task3原路线末点的手动控制点，按实际行驶顺序填写。脚本会像
# generate_task1_waypoints.py一样，把每个转角替换成零端曲率五次曲线，
# 再对整条路线按TARGET_SPACING等距采样。第一个点必须与环线最后一段
# 的终点相同；前两个点保持x相同，可确保车辆先沿通道方向竖直驶出。
# 后续只需修改这里的中间点，就能直接调整大厅内的返程曲线形状。
RETURN_CONTROL_POINTS: list[Point] = [
    (1.95, 2.05),
    CHANNEL_LOOP_ENDPOINT,
    (1.85, 1.60),
    (1.35, 1.20),
    (1.00, 1.10),
    (0.33, 0.75),
    (0.18, 0.65),
    (0.13, 0.60),
    (0.08, 0.55),
    (0.03, 0.45),
    (0.00, 0.30),
    RETURN_ROUTE_ENDPOINT,
]
RETURN_CORNER_TRANSITION_LENGTH = 0.28

# 顺、逆时针分别维护完整的有序几何，不能再由另一方向反向读取。
# 两套路线目前保持原有环线中心线形状；出通道后统一接入上面的
# RETURN_CONTROL_POINTS，不再用一条长直线直接连接原路线末点。
COUNTERCLOCKWISE_SEGMENTS: list[tuple[tuple[float, float], tuple[float, float]]] = [
    ((1.95, 1.80), (1.95, 2.50)),
    ((2.35, 2.95), (3.30, 2.95)),
    ((3.65, 3.20), (3.65, 3.85)),
    ((3.30, 4.10), (0.45, 4.10)),
    ((0.20, 3.85), (0.20, 3.20)),
    ((0.45, 2.95), (1.55, 2.95)),
    ((1.95, 2.50), RETURN_CONTROL_POINTS[0]),
]

CLOCKWISE_SEGMENTS: list[tuple[tuple[float, float], tuple[float, float]]] = [
    ((1.95, 1.80), (1.95, 2.50)),
    ((1.55, 2.95), (0.45, 2.95)),
    ((0.20, 3.20), (0.20, 3.85)),
    ((0.45, 4.10), (3.30, 4.10)),
    ((3.65, 3.85), (3.65, 3.20)),
    ((3.30, 2.95), (2.35, 2.95)),
    ((1.95, 2.50), RETURN_CONTROL_POINTS[0]),
]

TARGET_SPACING = 0.02
POINT_PASS_RADIUS = 0.6
COUNTERCLOCKWISE_TRANSITION_HANDLE_RATIOS = [
    0.3,  # 进入环线
    0.3,
    0.3,
    0.3,
    0.3,
    0.3,  # 环线圆滑接入竖直出通道段
]
CLOCKWISE_TRANSITION_HANDLE_RATIOS = [
    0.3,  # 进入环线
    0.3,
    0.3,
    0.3,
    0.3,
    0.3,  # 环线圆滑接入竖直出通道段
]


@dataclass(frozen=True)
class Piece:
    kind: str
    length: float
    point_at: Callable[[float], Point]


def distance(a: Point, b: Point) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])


def normalize(v: Point) -> Point:
    length = math.hypot(v[0], v[1])
    if length <= 1e-12:
        raise ValueError("route contains duplicate adjacent vertices")
    return (v[0] / length, v[1] / length)


def add(a: Point, b: Point) -> Point:
    return (a[0] + b[0], a[1] + b[1])


def sub(a: Point, b: Point) -> Point:
    return (a[0] - b[0], a[1] - b[1])


def mul(v: Point, scale: float) -> Point:
    return (v[0] * scale, v[1] * scale)


def line_piece(start: Point, end: Point) -> Piece | None:
    length = distance(start, end)
    if length <= 1e-9:
        return None

    dx = end[0] - start[0]
    dy = end[1] - start[1]

    def point_at(
        s: float,
        start: Point = start,
        dx: float = dx,
        dy: float = dy,
        length: float = length,
    ) -> Point:
        ratio = max(0.0, min(length, s)) / length
        return (start[0] + dx * ratio, start[1] + dy * ratio)

    return Piece(kind="line", length=length, point_at=point_at)


def cubic_bezier(p0: Point, p1: Point, p2: Point, p3: Point, t: float) -> Point:
    one_minus_t = 1.0 - t
    return (
        one_minus_t**3 * p0[0]
        + 3.0 * one_minus_t**2 * t * p1[0]
        + 3.0 * one_minus_t * t**2 * p2[0]
        + t**3 * p3[0],
        one_minus_t**3 * p0[1]
        + 3.0 * one_minus_t**2 * t * p1[1]
        + 3.0 * one_minus_t * t**2 * p2[1]
        + t**3 * p3[1],
    )


def bezier_piece(
    start: Point,
    start_direction: Point,
    end: Point,
    end_direction: Point,
    handle_ratio: float,
) -> Piece | None:
    chord = distance(start, end)
    if chord <= 1e-9:
        return None

    handle = chord * handle_ratio
    p1 = add(start, mul(start_direction, handle))
    p2 = sub(end, mul(end_direction, handle))

    samples = [cubic_bezier(start, p1, p2, end, i / 80.0) for i in range(81)]
    cumulative = [0.0]
    for i in range(len(samples) - 1):
        cumulative.append(cumulative[-1] + distance(samples[i], samples[i + 1]))
    length = cumulative[-1]

    def point_at(s: float) -> Point:
        target = max(0.0, min(length, s))
        for i in range(len(cumulative) - 1):
            if target <= cumulative[i + 1]:
                section_length = cumulative[i + 1] - cumulative[i]
                ratio = (
                    0.0
                    if section_length <= 1e-12
                    else (target - cumulative[i]) / section_length
                )
                return (
                    samples[i][0] + (samples[i + 1][0] - samples[i][0]) * ratio,
                    samples[i][1] + (samples[i + 1][1] - samples[i][1]) * ratio,
                )
        return samples[-1]

    return Piece(kind="bezier", length=length, point_at=point_at)


def build_pieces(
    segments: Sequence[tuple[Point, Point]],
    transition_handle_ratios: Sequence[float],
) -> list[Piece]:
    if len(segments) < 1:
        raise ValueError("route needs at least one straight segment")
    if len(transition_handle_ratios) != len(segments) - 1:
        raise ValueError("each gap between straight segments needs one transition handle ratio")
    pieces: list[Piece] = []
    for index, segment in enumerate(segments):
        line = line_piece(segment[0], segment[1])
        if line is not None:
            pieces.append(line)

        if index + 1 < len(segments):
            next_segment = segments[index + 1]
            transition = bezier_piece(
                segment[1],
                normalize(sub(segment[1], segment[0])),
                next_segment[0],
                normalize(sub(next_segment[1], next_segment[0])),
                transition_handle_ratios[index],
            )
            if transition is not None:
                pieces.append(transition)
    return pieces


def point_at_distance(pieces: Sequence[Piece], target_distance: float) -> Point:
    remaining = target_distance
    for piece in pieces:
        if remaining <= piece.length:
            return piece.point_at(remaining)
        remaining -= piece.length
    return pieces[-1].point_at(pieces[-1].length)


def sample_route(pieces: Sequence[Piece], spacing: float) -> list[Point]:
    total_length = sum(piece.length for piece in pieces)
    step_count = max(1, round(total_length / spacing))
    actual_spacing = total_length / step_count
    return [point_at_distance(pieces, actual_spacing * i) for i in range(step_count + 1)]


def smooth_control_polyline(
    control_points: Sequence[Point], corner_length: float, dense_spacing: float = 0.005
) -> list[Point]:
    """Round manual control-point corners with Task1's quintic C2 construction."""
    if len(control_points) < 2:
        raise ValueError("return route needs at least two control points")
    if corner_length <= 0.0 or len(control_points) < 3:
        return list(control_points)

    incoming_cuts = [0.0] * len(control_points)
    outgoing_cuts = [0.0] * len(control_points)
    for index in range(1, len(control_points) - 1):
        previous = control_points[index - 1]
        corner = control_points[index]
        following = control_points[index + 1]
        incoming = sub(corner, previous)
        outgoing = sub(following, corner)
        incoming_length = math.hypot(*incoming)
        outgoing_length = math.hypot(*outgoing)
        if incoming_length <= 1e-9 or outgoing_length <= 1e-9:
            continue
        turn_cosine = max(
            -1.0,
            min(
                1.0,
                (incoming[0] * outgoing[0] + incoming[1] * outgoing[1])
                / (incoming_length * outgoing_length),
            ),
        )
        if math.acos(turn_cosine) < math.radians(1.0):
            continue
        incoming_cuts[index] = corner_length
        outgoing_cuts[index] = corner_length

    # 相邻圆角必须在共享控制线段上留下正向余量；短线段会自动等比例缩短圆角。
    for index in range(len(control_points) - 1):
        segment_length = distance(control_points[index], control_points[index + 1])
        used_length = outgoing_cuts[index] + incoming_cuts[index + 1]
        maximum_used_length = 0.85 * segment_length
        if used_length > maximum_used_length and used_length > 1e-9:
            scale = maximum_used_length / used_length
            outgoing_cuts[index] *= scale
            incoming_cuts[index + 1] *= scale

    shaped_points = [control_points[0]]
    for index in range(1, len(control_points) - 1):
        previous = control_points[index - 1]
        corner = control_points[index]
        following = control_points[index + 1]
        incoming_vector = sub(corner, previous)
        outgoing_vector = sub(following, corner)
        incoming_length = math.hypot(*incoming_vector)
        outgoing_length = math.hypot(*outgoing_vector)
        cut_in = incoming_cuts[index]
        cut_out = outgoing_cuts[index]
        if (
            incoming_length <= 1e-9
            or outgoing_length <= 1e-9
            or cut_in <= 1e-9
            or cut_out <= 1e-9
        ):
            shaped_points.append(corner)
            continue

        tangent_in = mul(incoming_vector, 1.0 / incoming_length)
        tangent_out = mul(outgoing_vector, 1.0 / outgoing_length)
        entry = sub(corner, mul(tangent_in, cut_in))
        exit_point = add(corner, mul(tangent_out, cut_out))
        delta = sub(exit_point, entry)
        derivative_scale = math.hypot(*delta)
        velocity_start = mul(tangent_in, derivative_scale)
        velocity_finish = mul(tangent_out, derivative_scale)
        coefficient3 = tuple(
            10.0 * delta[axis]
            - 6.0 * velocity_start[axis]
            - 4.0 * velocity_finish[axis]
            for axis in range(2)
        )
        coefficient4 = tuple(
            -15.0 * delta[axis]
            + 8.0 * velocity_start[axis]
            + 7.0 * velocity_finish[axis]
            for axis in range(2)
        )
        coefficient5 = tuple(
            6.0 * delta[axis]
            - 3.0 * velocity_start[axis]
            - 3.0 * velocity_finish[axis]
            for axis in range(2)
        )

        if distance(shaped_points[-1], entry) > 1e-9:
            shaped_points.append(entry)
        sample_count = max(8, math.ceil((cut_in + cut_out) / dense_spacing))
        for sample_index in range(1, sample_count + 1):
            t = sample_index / sample_count
            shaped_points.append(
                tuple(
                    entry[axis]
                    + velocity_start[axis] * t
                    + coefficient3[axis] * t**3
                    + coefficient4[axis] * t**4
                    + coefficient5[axis] * t**5
                    for axis in range(2)
                )
            )

    if distance(shaped_points[-1], control_points[-1]) > 1e-9:
        shaped_points.append(control_points[-1])
    return shaped_points


def polyline_pieces(points: Sequence[Point]) -> list[Piece]:
    pieces: list[Piece] = []
    for start, end in zip(points, points[1:]):
        piece = line_piece(start, end)
        if piece is not None:
            pieces.append(piece)
    return pieces


def first_sample_index_at_distance(
    pieces: Sequence[Piece], points: Sequence[Point], start_distance: float
) -> int:
    """Return the first equal-distance sample on or after a route boundary."""
    if not pieces or len(points) < 2:
        raise ValueError("route needs at least two sampled points")
    total_length = sum(piece.length for piece in pieces)
    sampled_spacing = total_length / (len(points) - 1)
    return min(
        len(points) - 1,
        max(1, math.ceil((start_distance - 1e-9) / sampled_spacing)),
    )


def yaw_at(points: Sequence[Point], index: int) -> float:
    if len(points) < 2:
        return 0.0
    if index == 0:
        a, b = points[0], points[1]
    elif index == len(points) - 1:
        a, b = points[-2], points[-1]
    else:
        a, b = points[index - 1], points[index + 1]
    return math.atan2(b[1] - a[1], b[0] - a[0])


def fmt(value: float) -> str:
    if abs(value) < 0.0000005:
        value = 0.0
    return f"{value:.6f}".rstrip("0").rstrip(".")


def render_yaml(
    points: Sequence[Point],
    spacing: float,
    direction: str,
    return_start_index: int,
    return_corner_transition_length: float = RETURN_CORNER_TRANSITION_LENGTH,
    generator_name: str = "generate_channel_waypoints.py",
    task3_control_points_name: str = "RETURN_CONTROL_POINTS",
    task3_profile: str | None = None,
) -> str:
    direction_label = "顺时针" if direction == "clockwise" else "逆时针"
    lines = [
        f"# channel {direction_label}独立航点，禁止通过反向读取另一方向生成。",
        f"# 由 scripts/{generator_name} 生成：环线使用独立Bezier过渡，",
        "# 出通道到Task3原路线末点使用"
        f"{task3_control_points_name}手动设点和五次C2圆滑。",
        f"# task3_start标记{task3_control_points_name}首点，即大厅返程曲线起点。",
        f"# 按 point_001..point_{len(points):03d} 的文件顺序行驶。",
        "# Task2环线请改generate_channel_waypoints.py的"
        f"{direction.upper()}_SEGMENTS；Task3请改{generator_name}的"
        f"{task3_control_points_name}。",
        f"# target_spacing: {fmt(spacing)} m",
        "# return_corner_transition_length: "
        f"{fmt(return_corner_transition_length)} m",
    ]
    if task3_profile is not None:
        lines.append(f"task3_profile: {task3_profile}")
    lines.extend(
        [
            f"route_direction: {direction}",
            "routes:",
            f"  {direction}:",
        ]
    )

    for index, point in enumerate(points):
        waypoint_lines = [
            f"  - name: point_{index + 1:03d}",
            f"    x: {fmt(point[0])}",
            f"    y: {fmt(point[1])}",
            f"    yaw: {fmt(yaw_at(points, index))}",
            f"    pass_radius: {fmt(POINT_PASS_RADIUS)}",
        ]
        if index == return_start_index:
            waypoint_lines.append("    task3_start: true")
        lines.extend(waypoint_lines)
    return "\n".join(lines) + "\n"


def spacing_stats(points: Sequence[Point]) -> tuple[float, float, float]:
    distances = [distance(points[i], points[i + 1]) for i in range(len(points) - 1)]
    return min(distances), max(distances), sum(distances) / len(distances)


def render_map_preview(
    points: Sequence[Point], map_yaml: Path, output: Path, direction: str
) -> None:
    try:
        import yaml
        from PIL import Image, ImageDraw
    except ImportError as exc:
        raise RuntimeError("PNG preview requires PyYAML and Pillow") from exc

    metadata = yaml.safe_load(map_yaml.read_text(encoding="utf-8"))
    image_path = Path(metadata["image"])
    if not image_path.is_absolute():
        image_path = map_yaml.parent / image_path

    resolution = float(metadata["resolution"])
    origin = metadata["origin"]
    origin_x = float(origin[0])
    origin_y = float(origin[1])

    image = Image.open(image_path).convert("RGB")
    width, height = image.size

    def world_to_pixel(point: Point) -> tuple[int, int]:
        pixel_x = round((point[0] - origin_x) / resolution)
        map_y = round((point[1] - origin_y) / resolution)
        return pixel_x, height - 1 - map_y

    pixels = [world_to_pixel(point) for point in points]
    draw = ImageDraw.Draw(image)
    route_color = (255, 145, 0) if direction == "clockwise" else (0, 150, 255)
    direction_label = "CLOCKWISE (CW)" if direction == "clockwise" else "COUNTERCLOCKWISE (CCW)"

    if len(pixels) > 1:
        draw.line(pixels, fill=route_color, width=2)
    for pixel_x, pixel_y in pixels[::5]:
        draw.ellipse(
            (pixel_x - 1, pixel_y - 1, pixel_x + 1, pixel_y + 1),
            fill=route_color,
        )

    # 沿 waypoint 顺序画三角箭头；即使两条路线几何重合，也能直接看出行驶方向。
    if len(pixels) > 2:
        arrow_step = max(1, len(pixels) // 10)
        for index in range(arrow_step, len(pixels) - arrow_step // 2, arrow_step):
            before = pixels[max(0, index - 4)]
            after = pixels[min(len(pixels) - 1, index + 4)]
            dx = float(after[0] - before[0])
            dy = float(after[1] - before[1])
            length = math.hypot(dx, dy)
            if length <= 1e-9:
                continue
            ux = dx / length
            uy = dy / length
            px = -uy
            py = ux
            center_x, center_y = pixels[index]
            tip = (center_x + 7.0 * ux, center_y + 7.0 * uy)
            base_x = center_x - 4.0 * ux
            base_y = center_y - 4.0 * uy
            left = (base_x + 4.0 * px, base_y + 4.0 * py)
            right = (base_x - 4.0 * px, base_y - 4.0 * py)
            draw.polygon([tip, left, right], fill=route_color, outline=(0, 0, 0))

    if pixels:
        start_x, start_y = pixels[0]
        end_x, end_y = pixels[-1]
        draw.ellipse(
            (start_x - 7, start_y - 7, start_x + 7, start_y + 7),
            outline=(0, 190, 60),
            width=3,
        )
        draw.rectangle(
            (end_x - 4, end_y - 4, end_x + 4, end_y + 4),
            fill=(230, 35, 35),
            outline=(0, 0, 0),
        )
        draw.text((start_x + 9, start_y - 14), "START", fill=(0, 120, 35))
        draw.text((end_x + 9, end_y + 3), "END", fill=(190, 0, 0))

    # 标题放在地图上方的独立区域，避免遮挡地图和航点轨迹。
    text_width, text_height = draw.textsize(direction_label)
    title_padding_x = 6
    title_padding_y = 5
    title_height = text_height + 2 * title_padding_y

    preview = Image.new("RGB", (width, height + title_height), (255, 255, 255))
    preview.paste(image, (0, title_height))
    preview_draw = ImageDraw.Draw(preview)
    preview_draw.rectangle(
        (0, 0, width - 1, title_height - 1),
        fill=(255, 255, 255),
        outline=route_color,
        width=2,
    )
    title_x = max(title_padding_x, (width - text_width) // 2)
    title_y = title_padding_y
    preview_draw.text((title_x, title_y), direction_label, fill=(0, 0, 0))

    output.parent.mkdir(parents=True, exist_ok=True)
    preview.save(output, format="PNG", optimize=True)


def render_schematic_preview(
    points: Sequence[Point],
    map_yaml: Path,
    background: Path,
    output: Path,
    direction: str,
) -> None:
    """Map channel waypoints onto the same schematic coordinate frame as Task1."""
    try:
        import yaml
        from PIL import Image, ImageDraw
    except ImportError as exc:
        raise RuntimeError("PNG preview requires PyYAML and Pillow") from exc

    metadata = yaml.safe_load(map_yaml.read_text(encoding="utf-8"))
    map_image_path = Path(metadata["image"])
    if not map_image_path.is_absolute():
        map_image_path = map_yaml.parent / map_image_path

    resolution = float(metadata["resolution"])
    origin_x = float(metadata["origin"][0])
    origin_y = float(metadata["origin"][1])
    with Image.open(map_image_path) as map_image:
        map_width, map_height = map_image.size
    map_width_metres = map_width * resolution
    map_height_metres = map_height * resolution

    image = Image.open(background).convert("RGBA")
    width, height = image.size

    def world_to_pixel(point: Point) -> tuple[int, int]:
        normalized_x = (point[0] - origin_x) / map_width_metres
        normalized_y = (point[1] - origin_y) / map_height_metres
        return (
            round(normalized_x * (width - 1)),
            round((1.0 - normalized_y) * (height - 1)),
        )

    pixels = [world_to_pixel(point) for point in points]
    overlay = Image.new("RGBA", image.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(overlay)
    route_color = (
        (245, 105, 20, 240)
        if direction == "clockwise"
        else (0, 125, 245, 240)
    )
    line_width = max(3, round(min(width, height) / 225))
    waypoint_radius = max(1, round(line_width / 3))

    if len(pixels) > 1:
        draw.line(pixels, fill=route_color, width=line_width, joint="curve")
    for pixel_x, pixel_y in pixels[::5]:
        draw.ellipse(
            (
                pixel_x - waypoint_radius,
                pixel_y - waypoint_radius,
                pixel_x + waypoint_radius,
                pixel_y + waypoint_radius,
            ),
            fill=route_color,
        )

    result = Image.alpha_composite(image, overlay).convert("RGB")
    output.parent.mkdir(parents=True, exist_ok=True)
    result.save(output, format="PNG", optimize=True)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Generate independent clockwise/counterclockwise channel routes and previews."
    )
    parser.add_argument(
        "--clockwise-output",
        type=Path,
        default=DEFAULT_CLOCKWISE_OUTPUT,
        help="Clockwise YAML file to overwrite.",
    )
    parser.add_argument(
        "--counterclockwise-output",
        type=Path,
        default=DEFAULT_COUNTERCLOCKWISE_OUTPUT,
        help="Counterclockwise YAML file to overwrite.",
    )
    parser.add_argument(
        "--spacing",
        type=float,
        default=TARGET_SPACING,
        help="Target point spacing in meters.",
    )
    parser.add_argument(
        "--map-yaml",
        type=Path,
        default=DEFAULT_MAP_YAML,
        help="Restriction-map YAML used as the PNG preview background.",
    )
    parser.add_argument(
        "--clockwise-preview-output",
        type=Path,
        default=DEFAULT_CLOCKWISE_PREVIEW_OUTPUT,
        help="Clockwise PNG preview file to overwrite.",
    )
    parser.add_argument(
        "--counterclockwise-preview-output",
        type=Path,
        default=DEFAULT_COUNTERCLOCKWISE_PREVIEW_OUTPUT,
        help="Counterclockwise PNG preview file to overwrite.",
    )
    parser.add_argument(
        "--schematic-background",
        type=Path,
        default=DEFAULT_SCHEMATIC_BACKGROUND,
        help="Task1 schematic preview used as the channel schematic background.",
    )
    parser.add_argument(
        "--clockwise-schematic-preview-output",
        type=Path,
        default=DEFAULT_CLOCKWISE_SCHEMATIC_PREVIEW_OUTPUT,
        help="Clockwise channel-on-schematic PNG file to overwrite.",
    )
    parser.add_argument(
        "--counterclockwise-schematic-preview-output",
        type=Path,
        default=DEFAULT_COUNTERCLOCKWISE_SCHEMATIC_PREVIEW_OUTPUT,
        help="Counterclockwise channel-on-schematic PNG file to overwrite.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print both YAML documents instead of writing files.",
    )
    args = parser.parse_args()

    if args.spacing <= 0.0 or not math.isfinite(args.spacing):
        parser.error("--spacing must be a finite positive value")
    if len(RETURN_CONTROL_POINTS) < 2:
        parser.error("RETURN_CONTROL_POINTS needs at least two points")
    if distance(RETURN_CONTROL_POINTS[-1], RETURN_ROUTE_ENDPOINT) > 1e-9:
        parser.error("RETURN_CONTROL_POINTS must end at RETURN_ROUTE_ENDPOINT")

    route_jobs = [
        (
            "clockwise",
            CLOCKWISE_SEGMENTS,
            CLOCKWISE_TRANSITION_HANDLE_RATIOS,
            args.clockwise_output,
            args.clockwise_preview_output,
            args.clockwise_schematic_preview_output,
        ),
        (
            "counterclockwise",
            COUNTERCLOCKWISE_SEGMENTS,
            COUNTERCLOCKWISE_TRANSITION_HANDLE_RATIOS,
            args.counterclockwise_output,
            args.counterclockwise_preview_output,
            args.counterclockwise_schematic_preview_output,
        ),
    ]

    for (
        direction,
        segments,
        transition_ratios,
        output,
        preview_output,
        schematic_preview_output,
    ) in route_jobs:
        if distance(segments[-1][1], RETURN_CONTROL_POINTS[0]) > 1e-9:
            parser.error(
                f"{direction} channel exit must equal RETURN_CONTROL_POINTS[0]"
            )
        channel_pieces = build_pieces(segments, transition_ratios)
        return_polyline = smooth_control_polyline(
            RETURN_CONTROL_POINTS, RETURN_CORNER_TRANSITION_LENGTH
        )
        return_pieces = polyline_pieces(return_polyline)
        if not return_pieces:
            parser.error("RETURN_CONTROL_POINTS produced an empty return route")
        pieces = channel_pieces + return_pieces
        points = sample_route(pieces, args.spacing)
        return_start_distance = sum(piece.length for piece in channel_pieces)
        return_start_index = first_sample_index_at_distance(
            pieces, points, return_start_distance
        )
        rendered = render_yaml(points, args.spacing, direction, return_start_index)

        if args.dry_run:
            print(f"# ===== {direction} =====")
            print(rendered, end="")
        else:
            output.parent.mkdir(parents=True, exist_ok=True)
            output.write_text(rendered, encoding="utf-8")
            render_map_preview(points, args.map_yaml, preview_output, direction)
            render_schematic_preview(
                points,
                args.map_yaml,
                args.schematic_background,
                schematic_preview_output,
                direction,
            )

        min_spacing, max_spacing, avg_spacing = spacing_stats(points)
        total_length = sum(piece.length for piece in pieces)
        print(
            f"{direction}: generated {len(points)} points, length={total_length:.3f}m, "
            f"spacing min/avg/max={min_spacing:.3f}/{avg_spacing:.3f}/{max_spacing:.3f}m",
            file=sys.stderr,
        )
        print(f"{direction} output: {output}", file=sys.stderr)
        if not args.dry_run:
            print(f"{direction} preview: {preview_output}", file=sys.stderr)
            print(
                f"{direction} schematic preview: {schematic_preview_output}",
                file=sys.stderr,
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
