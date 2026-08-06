#!/usr/bin/env python3
"""把任意航点 YAML 转换成 PNG 路线示意图。

背景支持两种：
  --background map        占用地图背景（race_keepout.pgm，紫/橙/蓝路线）
  --background schematic  竞赛示意图背景（task1_schematic_background.png，紫色路线）

航点结构兼容主控 load_waypoints 能读的三种格式：
  1. 顶层 waypoints: [ {name, x, y}, ... ]     （Task1 航点文件格式）
  2. routes: { 方向: [ {name, x, y}, ... ] }     （channel 航点文件格式）
  3. 顶层直接是 [ [x, y], ... ]                  （最简格式）

用法：
  python3 yaml_to_schematic.py --yaml config/waypoints_task1_backup_45deg.yaml \
      --output map/my_route_on_schematic.png
  python3 yaml_to_schematic.py --yaml config/waypoints_channel_clockwise.yaml \
      --output map/my_route_on_map.png --background map --title "MY CW ROUTE"
"""
from __future__ import annotations

import argparse
from pathlib import Path

import yaml

PACKAGE_DIR = Path(__file__).resolve().parents[1]
DEFAULT_MAP_YAML = PACKAGE_DIR / "map" / "race_keepout.yaml"
DEFAULT_SCHEMATIC_BACKGROUND = PACKAGE_DIR / "map" / "task1_schematic_background.png"


def load_points(yaml_file: Path) -> list[tuple[float, float]]:
    """解析航点文件，返回 map 系 (x, y) 点列，兼容主控的三种结构。"""
    data = yaml.safe_load(yaml_file.read_text(encoding="utf-8"))

    items = None
    if isinstance(data, dict):
        routes = data.get("routes")
        if isinstance(routes, dict):
            for direction, seq in routes.items():
                if isinstance(seq, list) and seq:
                    items = seq
                    break
        if items is None:
            items = data.get("waypoints")
        if items is None:
            raise ValueError("未找到航点序列：文件需要顶层 waypoints 或 routes")
    elif isinstance(data, list):
        items = data
    else:
        raise ValueError("无法识别的航点文件结构")

    points: list[tuple[float, float]] = []
    for index, item in enumerate(items):
        if isinstance(item, dict) and "x" in item and "y" in item:
            points.append((float(item["x"]), float(item["y"])))
        elif isinstance(item, (list, tuple)) and len(item) >= 2:
            points.append((float(item[0]), float(item[1])))
        else:
            raise ValueError(f"第 {index + 1} 个航点格式无法识别")
    if len(points) < 2:
        raise ValueError("有效航点不足 2 个，无法画路线")
    return points


def load_map_metadata(map_yaml: Path) -> dict:
    metadata = yaml.safe_load(map_yaml.read_text(encoding="utf-8"))
    image_path = Path(metadata["image"])
    if not image_path.is_absolute():
        image_path = map_yaml.parent / image_path
    return {
        "image_path": image_path,
        "resolution": float(metadata["resolution"]),
        "origin_x": float(metadata["origin"][0]),
        "origin_y": float(metadata["origin"][1]),
    }


def render_on_map(points, metadata, output, route_color, title):
    from PIL import Image, ImageDraw

    image = Image.open(metadata["image_path"]).convert("RGB")
    width, height = image.size
    res = metadata["resolution"]
    origin_x = metadata["origin_x"]
    origin_y = metadata["origin_y"]

    def world_to_pixel(point):
        pixel_x = round((point[0] - origin_x) / res)
        map_y = round((point[1] - origin_y) / res)
        return pixel_x, height - 1 - map_y

    pixels = [world_to_pixel(point) for point in points]
    draw = ImageDraw.Draw(image)
    if len(pixels) > 1:
        draw.line(pixels, fill=route_color, width=2)
    for pixel_x, pixel_y in pixels[::5]:
        draw.ellipse(
            (pixel_x - 1, pixel_y - 1, pixel_x + 1, pixel_y + 1),
            fill=route_color)

    if pixels:
        start_x, start_y = pixels[0]
        end_x, end_y = pixels[-1]
        draw.ellipse(
            (start_x - 7, start_y - 7, start_x + 7, start_y + 7),
            outline=(0, 190, 60), width=3)
        draw.rectangle(
            (end_x - 4, end_y - 4, end_x + 4, end_y + 4),
            fill=(230, 35, 35), outline=(0, 0, 0))
        draw.text((start_x + 9, start_y - 14), "START", fill=(0, 120, 35))
        draw.text((end_x + 9, end_y + 3), "END", fill=(190, 0, 0))

    text_width, text_height = draw.textsize(title)
    title_padding_x = 6
    title_padding_y = 5
    title_height = text_height + 2 * title_padding_y
    preview = Image.new("RGB", (width, height + title_height), (255, 255, 255))
    preview.paste(image, (0, title_height))
    preview_draw = ImageDraw.Draw(preview)
    preview_draw.rectangle(
        (0, 0, width - 1, title_height - 1),
        fill=(255, 255, 255), outline=route_color, width=2)
    title_x = max(title_padding_x, (width - text_width) // 2)
    preview_draw.text((title_x, title_padding_y), title, fill=(0, 0, 0))

    output.parent.mkdir(parents=True, exist_ok=True)
    preview.save(output, format="PNG", optimize=True)


def render_on_schematic(points, metadata, background, output, route_color, title):
    from PIL import Image, ImageDraw

    with Image.open(metadata["image_path"]) as map_image:
        map_width, map_height = map_image.size
    map_width_metres = map_width * metadata["resolution"]
    map_height_metres = map_height * metadata["resolution"]

    image = Image.open(background).convert("RGBA")
    width, height = image.size
    origin_x = metadata["origin_x"]
    origin_y = metadata["origin_y"]

    def world_to_pixel(point):
        normalized_x = (point[0] - origin_x) / map_width_metres
        normalized_y = (point[1] - origin_y) / map_height_metres
        return (
            round(normalized_x * (width - 1)),
            round((1.0 - normalized_y) * (height - 1)),
        )

    pixels = [world_to_pixel(point) for point in points]
    overlay = Image.new("RGBA", image.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(overlay)
    line_width = max(3, round(min(width, height) / 225))
    waypoint_radius = max(1, round(line_width / 3))
    if len(pixels) > 1:
        draw.line(pixels, fill=route_color, width=line_width, joint="curve")
    for pixel_x, pixel_y in pixels[::5]:
        draw.ellipse(
            (pixel_x - waypoint_radius, pixel_y - waypoint_radius,
             pixel_x + waypoint_radius, pixel_y + waypoint_radius),
            fill=route_color)

    result = Image.alpha_composite(image, overlay).convert("RGB")
    output.parent.mkdir(parents=True, exist_ok=True)
    result.save(output, format="PNG", optimize=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--yaml", type=Path, required=True, help="输入的航点 YAML 文件")
    parser.add_argument("--output", type=Path, required=True, help="输出 PNG 文件路径")
    parser.add_argument(
        "--map-yaml", type=Path, default=DEFAULT_MAP_YAML,
        help="地图 YAML（用于坐标映射，默认 race_keepout.yaml）")
    parser.add_argument(
        "--background", choices=["map", "schematic"], default="schematic",
        help="背景类型：map=占用地图，schematic=竞赛示意图（默认）")
    parser.add_argument(
        "--schematic-background", type=Path, default=DEFAULT_SCHEMATIC_BACKGROUND,
        help="竞赛示意图背景文件（仅 --background schematic 时使用）")
    parser.add_argument("--title", default="", help="PNG 标题（默认取输入文件名）")
    args = parser.parse_args()

    try:
        import PIL  # noqa: F401
    except ImportError as exc:
        raise RuntimeError("PNG 预览需要 Pillow，请先 pip install pillow") from exc

    points = load_points(args.yaml)
    metadata = load_map_metadata(args.map_yaml)
    title = args.title or args.yaml.stem.replace("_", " ").upper()

    if args.background == "map":
        render_on_map(points, metadata, args.output, (170, 70, 230), title)
    else:
        render_on_schematic(
            points, metadata, args.schematic_background, args.output,
            (170, 70, 230, 235), title)

    print(f"已生成 {len(points)} 个航点的示意图: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
