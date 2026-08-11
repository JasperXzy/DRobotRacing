#!/usr/bin/env python3
"""Generate the Task1 route by equal-distance interpolation of map control points."""

import argparse
import math
from pathlib import Path

import yaml


PACKAGE_DIR = Path(__file__).resolve().parents[1]
DEFAULT_MAP_YAML = PACKAGE_DIR / 'map' / 'race_keepout.yaml'
DEFAULT_PREVIEW_OUTPUT = (
    PACKAGE_DIR / 'map' / 'waypoints_task1_on_restriction_map.png')
DEFAULT_SCHEMATIC_BACKGROUND = (
    PACKAGE_DIR / 'map' / 'task1_schematic_background.png')
DEFAULT_SCHEMATIC_PREVIEW_OUTPUT = (
    PACKAGE_DIR / 'map' / 'waypoints_task1_on_schematic.png')


## original
# DEFAULT_CONTROL_POINTS = [
#     (0.00, 0.00),
#     (1.50, 0.60),
#     (3.75, 0.65),
#     (4.10, 1.60),
# ]

## szc
DEFAULT_CONTROL_POINTS = [
    (0.00, 0.00),
    (0.70, 0.00),
    (1.50, 0.20),
    (3.50, 0.30),
    (4.30, 1.50),
]

# 备用路线保留起点、倒数第二控制点和终点。第一段沿 map 坐标系 45°
# 右上方向行驶到 (0.70, 0.70)，之后直连倒数第二控制点。
BACKUP_45DEG_CONTROL_POINTS = [
    (0.00, 0.00),
    (0.60, 0.60),
    (3.30, 0.60),
    (4.30, 1.50),
]

# ZYR 自定义路线：起点直连终点的一条斜线。
ZYR_CONTROL_POINTS = [
    (0.00, 0.00),
    (2.70, 0.90),
]

CONTROL_POINT_VARIANTS = {
    'default': DEFAULT_CONTROL_POINTS,
    'backup_45deg': BACKUP_45DEG_CONTROL_POINTS,
    'zyr': ZYR_CONTROL_POINTS,
}

# 各变体输出文件的后缀（与现有配置/launch 引用保持兼容：
# waypoints_task1.yaml / waypoints_task1_backup_45deg.yaml / waypoints_task1_ZYR.yaml）
VARIANT_NAME_SUFFIX = {
    'default': '',
    'backup_45deg': '_backup_45deg',
    'zyr': '_ZYR',
}


def interpolate_polyline(control_points, spacing):
    points = [control_points[0]]
    for start, finish in zip(control_points, control_points[1:]):
        dx = finish[0] - start[0]
        dy = finish[1] - start[1]
        length = math.hypot(dx, dy)
        segment_count = max(1, int(math.ceil(length / spacing)))
        for index in range(1, segment_count + 1):
            ratio = index / segment_count
            points.append((
                start[0] + ratio * dx,
                start[1] + ratio * dy,
            ))
    return points


def render_map_preview(points, map_yaml, output):
    """Draw the generated Task1 route on the occupancy map."""
    try:
        from PIL import Image, ImageDraw
    except ImportError as exc:
        raise RuntimeError('PNG preview requires Pillow') from exc

    metadata = yaml.safe_load(map_yaml.read_text(encoding='utf-8'))
    image_path = Path(metadata['image'])
    if not image_path.is_absolute():
        image_path = map_yaml.parent / image_path

    resolution = float(metadata['resolution'])
    origin_x = float(metadata['origin'][0])
    origin_y = float(metadata['origin'][1])

    image = Image.open(image_path).convert('RGB')
    width, height = image.size

    def world_to_pixel(point):
        pixel_x = round((point[0] - origin_x) / resolution)
        map_y = round((point[1] - origin_y) / resolution)
        return pixel_x, height - 1 - map_y

    pixels = [world_to_pixel(point) for point in points]
    draw = ImageDraw.Draw(image)
    route_color = (170, 70, 230)

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
            outline=(0, 190, 60),
            width=3)
        draw.rectangle(
            (end_x - 4, end_y - 4, end_x + 4, end_y + 4),
            fill=(230, 35, 35),
            outline=(0, 0, 0))
        draw.text((start_x + 9, start_y - 14), 'START', fill=(0, 120, 35))
        draw.text((end_x + 9, end_y + 3), 'END', fill=(190, 0, 0))

    title = 'TASK1'
    text_width, text_height = draw.textsize(title)
    title_padding_x = 6
    title_padding_y = 5
    title_height = text_height + 2 * title_padding_y

    preview = Image.new(
        'RGB', (width, height + title_height), (255, 255, 255))
    preview.paste(image, (0, title_height))
    preview_draw = ImageDraw.Draw(preview)
    preview_draw.rectangle(
        (0, 0, width - 1, title_height - 1),
        fill=(255, 255, 255),
        outline=route_color,
        width=2)
    title_x = max(title_padding_x, (width - text_width) // 2)
    preview_draw.text(
        (title_x, title_padding_y), title, fill=(0, 0, 0))

    output.parent.mkdir(parents=True, exist_ok=True)
    preview.save(output, format='PNG', optimize=True)


def render_schematic_preview(points, map_yaml, background, output):
    """Draw map-coordinate Task1 waypoints on the competition schematic."""
    try:
        from PIL import Image, ImageDraw
    except ImportError as exc:
        raise RuntimeError('PNG preview requires Pillow') from exc

    metadata = yaml.safe_load(map_yaml.read_text(encoding='utf-8'))
    map_image_path = Path(metadata['image'])
    if not map_image_path.is_absolute():
        map_image_path = map_yaml.parent / map_image_path

    resolution = float(metadata['resolution'])
    origin_x = float(metadata['origin'][0])
    origin_y = float(metadata['origin'][1])
    with Image.open(map_image_path) as map_image:
        map_width, map_height = map_image.size
    map_width_metres = map_width * resolution
    map_height_metres = map_height * resolution

    image = Image.open(background).convert('RGBA')
    width, height = image.size

    def world_to_pixel(point):
        normalized_x = (point[0] - origin_x) / map_width_metres
        normalized_y = (point[1] - origin_y) / map_height_metres
        return (
            round(normalized_x * (width - 1)),
            round((1.0 - normalized_y) * (height - 1)),
        )

    pixels = [world_to_pixel(point) for point in points]
    overlay = Image.new('RGBA', image.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(overlay)
    route_color = (170, 70, 230, 235)
    line_width = max(3, round(min(width, height) / 225))
    waypoint_radius = max(1, round(line_width / 3))

    if len(pixels) > 1:
        draw.line(pixels, fill=route_color, width=line_width, joint='curve')
    for pixel_x, pixel_y in pixels[::5]:
        draw.ellipse(
            (
                pixel_x - waypoint_radius,
                pixel_y - waypoint_radius,
                pixel_x + waypoint_radius,
                pixel_y + waypoint_radius,
            ),
            fill=route_color)

    result = Image.alpha_composite(image, overlay).convert('RGB')
    output.parent.mkdir(parents=True, exist_ok=True)
    result.save(output, format='PNG', optimize=True)


def generate_variant(
        variant, spacing, map_yaml, schematic_background,
        output=None, preview_output=None, schematic_preview_output=None):
    """Generate one variant: waypoint YAML + map/schematic previews.

    Output paths default to the variant's standard files under
    config/ and map/; pass explicit paths to override them.
    """
    control_points = CONTROL_POINT_VARIANTS[variant]
    points = interpolate_polyline(control_points, spacing)
    suffix = VARIANT_NAME_SUFFIX[variant]
    output = output or (
        PACKAGE_DIR / 'config' / f'waypoints_task1{suffix}.yaml')
    preview_output = preview_output or (
        PACKAGE_DIR / 'map' /
        f'waypoints_task1{suffix}_on_restriction_map.png')
    schematic_preview_output = schematic_preview_output or (
        PACKAGE_DIR / 'map' / f'waypoints_task1{suffix}_on_schematic.png')
    output.parent.mkdir(parents=True, exist_ok=True)
    waypoint_prefix = (
        'task1' if variant == 'default' else f'task1_{variant}')
    payload = {
        'route_direction': 'task1',
        'route_variant': variant,
        'control_points': [list(point) for point in control_points],
        'waypoints': [
            {
                'name': f'{waypoint_prefix}_{index:03d}',
                'x': round(x, 6),
                'y': round(y, 6),
            }
            for index, (x, y) in enumerate(points)
        ],
    }
    with output.open('w', encoding='utf-8') as stream:
        yaml.safe_dump(payload, stream, sort_keys=False, allow_unicode=True)
    render_map_preview(points, map_yaml, preview_output)
    render_schematic_preview(
        points, map_yaml, schematic_background, schematic_preview_output)
    return len(points), output, preview_output, schematic_preview_output


def main():
    parser = argparse.ArgumentParser(
        description='Generate Task1 waypoints. Default: regenerate all '
                    'variants (default/backup_45deg/zyr) in sequence.')
    parser.add_argument(
        '--variant',
        choices=['all'] + sorted(CONTROL_POINT_VARIANTS),
        default='all',
        help='Task1 control-point variant to generate; '
             "'all' regenerates every variant (default).")
    parser.add_argument(
        '--output',
        default=str(
            Path(__file__).resolve().parents[1] /
            'config' / 'waypoints_task1.yaml'))
    parser.add_argument(
        '--spacing', type=float, default=0.02,
        help='Maximum distance between adjacent Task1 waypoints in metres.')
    parser.add_argument(
        '--map-yaml', type=Path, default=DEFAULT_MAP_YAML,
        help='Occupancy-map YAML used as the PNG preview background.')
    parser.add_argument(
        '--preview-output', type=Path, default=DEFAULT_PREVIEW_OUTPUT,
        help='Task1 PNG preview file to overwrite (single-variant mode only).')
    parser.add_argument(
        '--schematic-background',
        type=Path,
        default=DEFAULT_SCHEMATIC_BACKGROUND,
        help='Competition schematic used as the second preview background.')
    parser.add_argument(
        '--schematic-preview-output',
        type=Path,
        default=DEFAULT_SCHEMATIC_PREVIEW_OUTPUT,
        help='Task1-on-schematic PNG preview file to overwrite '
             '(single-variant mode only).')
    args = parser.parse_args()
    if not math.isfinite(args.spacing) or args.spacing <= 0.0:
        raise ValueError('--spacing must be a positive finite number')

    default_output = Path(__file__).resolve().parents[1] / 'config' / 'waypoints_task1.yaml'
    if args.variant == 'all':
        for variant in sorted(CONTROL_POINT_VARIANTS):
            count, output, preview_output, schematic_preview_output = (
                generate_variant(
                    variant,
                    args.spacing,
                    args.map_yaml.expanduser(),
                    args.schematic_background.expanduser()))
            print(
                f'Generated {count} Task1 waypoints '
                f'(variant={variant}) at {output} '
                f'(spacing <= {args.spacing:.3f} m)\n'
                f'Task1 map preview: {preview_output}\n'
                f'Task1 schematic preview: {schematic_preview_output}')
        return

    output = Path(args.output).expanduser()
    preview_output = Path(args.preview_output).expanduser()
    schematic_preview_output = Path(
        args.schematic_preview_output).expanduser()
    if output == default_output:
        output = None
    if preview_output == DEFAULT_PREVIEW_OUTPUT:
        preview_output = None
    if schematic_preview_output == DEFAULT_SCHEMATIC_PREVIEW_OUTPUT:
        schematic_preview_output = None
    count, output, preview_output, schematic_preview_output = generate_variant(
        args.variant,
        args.spacing,
        args.map_yaml.expanduser(),
        args.schematic_background.expanduser(),
        output, preview_output, schematic_preview_output)
    print(
        f'Generated {count} Task1 waypoints '
        f'(variant={args.variant}) at {output} '
        f'(spacing <= {args.spacing:.3f} m)\n'
        f'Task1 map preview: {preview_output}\n'
        f'Task1 schematic preview: {schematic_preview_output}')


if __name__ == '__main__':
    main()
