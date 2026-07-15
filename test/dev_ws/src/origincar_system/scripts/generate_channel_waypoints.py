#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Sequence


PACKAGE_DIR = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = PACKAGE_DIR / "config" / "waypoints_channel.yaml"
DEFAULT_MAP_YAML = PACKAGE_DIR / "map" / "race_keepout.yaml"
DEFAULT_PREVIEW_OUTPUT = PACKAGE_DIR / "map" / "waypoints_channel_on_restriction_map.png"

# The route is modeled as 7 straight center-line segments. Each gap between
# adjacent segments is connected by a smooth cubic Bezier transition.
STRAIGHT_SEGMENTS: list[tuple[tuple[float, float], tuple[float, float]]] = [
    ((1.95, 1.80), (1.95, 2.50)),
    ((2.35, 2.90), (3.30, 2.90)),
    ((3.60, 3.20), (3.60, 3.85)),
    ((3.30, 4.05), (0.45, 4.05)),
    ((0.15, 3.85), (0.15, 3.20)),
    ((0.45, 2.90), (1.55, 2.90)),
    ((1.95, 2.50), (1.95, 1.80)),
]

TARGET_SPACING = 0.02
ROUTE_DIRECTION = "clockwise"
POINT_PASS_RADIUS = 0.6
SMOOTH_HANDLE_RATIO = 0.3

Point = tuple[float, float]


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

    def point_at(s: float, start: Point = start, dx: float = dx, dy: float = dy, length: float = length) -> Point:
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
                ratio = 0.0 if section_length <= 1e-12 else (target - cumulative[i]) / section_length
                return (
                    samples[i][0] + (samples[i + 1][0] - samples[i][0]) * ratio,
                    samples[i][1] + (samples[i + 1][1] - samples[i][1]) * ratio,
                )
        return samples[-1]

    return Piece(kind="bezier", length=length, point_at=point_at)


def build_pieces(segments: Sequence[tuple[Point, Point]]) -> list[Piece]:
    if len(segments) < 1:
        raise ValueError("route needs at least one straight segment")
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
                SMOOTH_HANDLE_RATIO,
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


def render_yaml(points: Sequence[Point], spacing: float) -> str:
    lines = [
        "# channel 环线只维护一份逆时针航点。",
        "# 由 scripts/generate_channel_waypoints.py 生成：7 条直线段 + 6 段贝塞尔平滑过渡。",
        f"# 逆时针按 point_001..point_{len(points):03d} 读取；顺时针由 visual_lidar_path_follower 自动反向读取。",
        "# 修改轨迹时先改脚本里的 STRAIGHT_SEGMENTS，再重新运行脚本。",
        f"# target_spacing: {fmt(spacing)} m",
        f"route_direction: {ROUTE_DIRECTION}",
        "routes:",
        "  counterclockwise:",
    ]

    for index, point in enumerate(points, start=1):
        lines.extend(
            [
                f"  - name: point_{index:03d}",
                f"    x: {fmt(point[0])}",
                f"    y: {fmt(point[1])}",
                f"    yaw: {fmt(yaw_at(points, index - 1))}",
                f"    pass_radius: {fmt(POINT_PASS_RADIUS)}",
            ]
        )
    return "\n".join(lines) + "\n"


def spacing_stats(points: Sequence[Point]) -> tuple[float, float, float]:
    distances = [distance(points[i], points[i + 1]) for i in range(len(points) - 1)]
    return min(distances), max(distances), sum(distances) / len(distances)


def render_map_preview(points: Sequence[Point], map_yaml: Path, output: Path) -> None:
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
    if len(pixels) > 1:
        draw.line(pixels, fill=(0, 170, 255), width=1)
    for pixel_x, pixel_y in pixels:
        draw.ellipse(
            (pixel_x - 1, pixel_y - 1, pixel_x + 1, pixel_y + 1),
            fill=(255, 40, 40),
        )

    if pixels:
        start_x, start_y = pixels[0]
        draw.ellipse(
            (start_x - 3, start_y - 3, start_x + 3, start_y + 3),
            fill=(0, 220, 70),
            outline=(0, 0, 0),
        )

    output.parent.mkdir(parents=True, exist_ok=True)
    image.save(output, format="PNG", optimize=True)


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate channel waypoints from straight segments and smooth transitions.")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT, help="YAML file to overwrite.")
    parser.add_argument("--spacing", type=float, default=TARGET_SPACING, help="Target point spacing in meters.")
    parser.add_argument(
        "--map-yaml",
        type=Path,
        default=DEFAULT_MAP_YAML,
        help="Restriction-map YAML used as the PNG preview background.",
    )
    parser.add_argument(
        "--preview-output",
        type=Path,
        default=DEFAULT_PREVIEW_OUTPUT,
        help="PNG preview file to overwrite.",
    )
    parser.add_argument("--dry-run", action="store_true", help="Print the YAML instead of writing the file.")
    args = parser.parse_args()

    pieces = build_pieces(STRAIGHT_SEGMENTS)
    points = sample_route(pieces, args.spacing)
    rendered = render_yaml(points, args.spacing)

    if args.dry_run:
        print(rendered, end="")
    else:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding="utf-8")
        render_map_preview(points, args.map_yaml, args.preview_output)

    min_spacing, max_spacing, avg_spacing = spacing_stats(points)
    total_length = sum(piece.length for piece in pieces)
    print(
        f"generated {len(points)} points, length={total_length:.3f}m, "
        f"spacing min/avg/max={min_spacing:.3f}/{avg_spacing:.3f}/{max_spacing:.3f}m",
        file=sys.stderr,
    )
    print(f"output: {args.output}", file=sys.stderr)
    if not args.dry_run:
        print(f"preview: {args.preview_output}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
