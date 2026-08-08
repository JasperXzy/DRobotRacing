#!/usr/bin/env python3
"""Generate a C2-smoothed, equal-distance Task1 route from map control points."""

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
    (0.50, 0.00),
    (1.50, 0.40),
    (3.60, 0.30),
    (4.00, 1.10),
]


def resample_polyline(points, spacing):
    """Resample an already-shaped polyline at nearly equal arc-length spacing."""
    if len(points) < 2:
        return list(points)

    cumulative = [0.0]
    for start, finish in zip(points, points[1:]):
        cumulative.append(cumulative[-1] + math.hypot(
            finish[0] - start[0], finish[1] - start[1]))
    total_length = cumulative[-1]
    if total_length <= 1.0e-9:
        return [points[0]]

    result = [points[0]]
    segment_index = 0
    target_distance = spacing
    while target_distance < total_length:
        while (segment_index + 1 < len(cumulative) - 1 and
               cumulative[segment_index + 1] < target_distance):
            segment_index += 1
        segment_start_distance = cumulative[segment_index]
        segment_finish_distance = cumulative[segment_index + 1]
        segment_length = segment_finish_distance - segment_start_distance
        ratio = ((target_distance - segment_start_distance) / segment_length
                 if segment_length > 1.0e-12 else 0.0)
        start = points[segment_index]
        finish = points[segment_index + 1]
        result.append((
            start[0] + ratio * (finish[0] - start[0]),
            start[1] + ratio * (finish[1] - start[1]),
        ))
        target_distance += spacing
    if math.hypot(
            result[-1][0] - points[-1][0],
            result[-1][1] - points[-1][1]) > 1.0e-9:
        result.append(points[-1])
    return result


def smooth_control_polyline(control_points, spacing, corner_length):
    """Replace each interior corner with a zero-end-curvature quintic curve."""
    if len(control_points) < 3 or corner_length <= 0.0:
        return resample_polyline(control_points, spacing)

    incoming_cuts = [0.0] * len(control_points)
    outgoing_cuts = [0.0] * len(control_points)
    for index in range(1, len(control_points) - 1):
        previous = control_points[index - 1]
        corner = control_points[index]
        following = control_points[index + 1]
        incoming = (corner[0] - previous[0], corner[1] - previous[1])
        outgoing = (following[0] - corner[0], following[1] - corner[1])
        incoming_length = math.hypot(*incoming)
        outgoing_length = math.hypot(*outgoing)
        if incoming_length <= 1.0e-9 or outgoing_length <= 1.0e-9:
            continue
        turn_cosine = max(-1.0, min(1.0, (
            incoming[0] * outgoing[0] + incoming[1] * outgoing[1]) /
            (incoming_length * outgoing_length)))
        if math.acos(turn_cosine) < math.radians(1.0):
            continue
        incoming_cuts[index] = corner_length
        outgoing_cuts[index] = corner_length

    # Adjacent rounded corners must leave a straight, forward-moving remainder
    # on their shared control segment. Endpoint segments only have one cut.
    for index in range(len(control_points) - 1):
        segment_length = math.hypot(
            control_points[index + 1][0] - control_points[index][0],
            control_points[index + 1][1] - control_points[index][1])
        used_length = outgoing_cuts[index] + incoming_cuts[index + 1]
        maximum_used_length = 0.85 * segment_length
        if used_length > maximum_used_length and used_length > 1.0e-9:
            scale = maximum_used_length / used_length
            outgoing_cuts[index] *= scale
            incoming_cuts[index + 1] *= scale

    shaped_points = [control_points[0]]
    dense_spacing = max(0.001, min(0.005, 0.25 * spacing))
    for index in range(1, len(control_points) - 1):
        previous = control_points[index - 1]
        corner = control_points[index]
        following = control_points[index + 1]
        incoming_vector = (
            corner[0] - previous[0], corner[1] - previous[1])
        outgoing_vector = (
            following[0] - corner[0], following[1] - corner[1])
        incoming_length = math.hypot(*incoming_vector)
        outgoing_length = math.hypot(*outgoing_vector)
        cut_in = incoming_cuts[index]
        cut_out = outgoing_cuts[index]
        if (incoming_length <= 1.0e-9 or outgoing_length <= 1.0e-9 or
                cut_in <= 1.0e-9 or cut_out <= 1.0e-9):
            shaped_points.append(corner)
            continue

        tangent_in = (
            incoming_vector[0] / incoming_length,
            incoming_vector[1] / incoming_length)
        tangent_out = (
            outgoing_vector[0] / outgoing_length,
            outgoing_vector[1] / outgoing_length)
        entry = (
            corner[0] - cut_in * tangent_in[0],
            corner[1] - cut_in * tangent_in[1])
        exit_point = (
            corner[0] + cut_out * tangent_out[0],
            corner[1] + cut_out * tangent_out[1])
        delta = (exit_point[0] - entry[0], exit_point[1] - entry[1])
        derivative_scale = math.hypot(*delta)
        velocity_start = (
            derivative_scale * tangent_in[0],
            derivative_scale * tangent_in[1])
        velocity_finish = (
            derivative_scale * tangent_out[0],
            derivative_scale * tangent_out[1])
        coefficient3 = tuple(
            10.0 * delta[axis] - 6.0 * velocity_start[axis] -
            4.0 * velocity_finish[axis] for axis in range(2))
        coefficient4 = tuple(
            -15.0 * delta[axis] + 8.0 * velocity_start[axis] +
            7.0 * velocity_finish[axis] for axis in range(2))
        coefficient5 = tuple(
            6.0 * delta[axis] - 3.0 * velocity_start[axis] -
            3.0 * velocity_finish[axis] for axis in range(2))

        if math.hypot(
                shaped_points[-1][0] - entry[0],
                shaped_points[-1][1] - entry[1]) > 1.0e-9:
            shaped_points.append(entry)
        sample_count = max(
            8, int(math.ceil((cut_in + cut_out) / dense_spacing)))
        for sample_index in range(1, sample_count + 1):
            t = sample_index / sample_count
            point = tuple(
                entry[axis] + velocity_start[axis] * t +
                coefficient3[axis] * t ** 3 +
                coefficient4[axis] * t ** 4 +
                coefficient5[axis] * t ** 5
                for axis in range(2))
            shaped_points.append(point)

    if math.hypot(
            shaped_points[-1][0] - control_points[-1][0],
            shaped_points[-1][1] - control_points[-1][1]) > 1.0e-9:
        shaped_points.append(control_points[-1])
    return resample_polyline(shaped_points, spacing)


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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--output',
        default=str(
            Path(__file__).resolve().parents[1] /
            'config' / 'waypoints_task1.yaml'))
    parser.add_argument(
        '--spacing', type=float, default=0.02,
        help='Maximum distance between adjacent Task1 waypoints in metres.')
    parser.add_argument(
        '--corner-transition-length', type=float, default=0.35,
        help=(
            'Distance cut from each side of an interior corner before the '
            'C2 quintic transition; 0 disables smoothing.'))
    parser.add_argument(
        '--map-yaml', type=Path, default=DEFAULT_MAP_YAML,
        help='Occupancy-map YAML used as the PNG preview background.')
    parser.add_argument(
        '--preview-output', type=Path, default=DEFAULT_PREVIEW_OUTPUT,
        help='Task1 PNG preview file to overwrite.')
    parser.add_argument(
        '--schematic-background',
        type=Path,
        default=DEFAULT_SCHEMATIC_BACKGROUND,
        help='Competition schematic used as the second preview background.')
    parser.add_argument(
        '--schematic-preview-output',
        type=Path,
        default=DEFAULT_SCHEMATIC_PREVIEW_OUTPUT,
        help='Task1-on-schematic PNG preview file to overwrite.')
    args = parser.parse_args()
    if not math.isfinite(args.spacing) or args.spacing <= 0.0:
        raise ValueError('--spacing must be a positive finite number')
    if (not math.isfinite(args.corner_transition_length) or
            args.corner_transition_length < 0.0):
        raise ValueError(
            '--corner-transition-length must be a finite non-negative number')

    points = smooth_control_polyline(
        DEFAULT_CONTROL_POINTS, args.spacing,
        args.corner_transition_length)
    output = Path(args.output).expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        'route_direction': 'task1',
        'control_points': [list(point) for point in DEFAULT_CONTROL_POINTS],
        'smoothing': {
            'method': 'quintic_hermite_c2',
            'corner_transition_length': args.corner_transition_length,
        },
        'waypoints': [
            {'name': f'task1_{index:03d}', 'x': round(x, 6), 'y': round(y, 6)}
            for index, (x, y) in enumerate(points)
        ],
    }
    with output.open('w', encoding='utf-8') as stream:
        yaml.safe_dump(payload, stream, sort_keys=False, allow_unicode=True)
    preview_output = args.preview_output.expanduser()
    render_map_preview(points, args.map_yaml.expanduser(), preview_output)
    schematic_preview_output = args.schematic_preview_output.expanduser()
    render_schematic_preview(
        points,
        args.map_yaml.expanduser(),
        args.schematic_background.expanduser(),
        schematic_preview_output)
    print(
        f'Generated {len(points)} Task1 waypoints at {output} '
        f'(spacing <= {args.spacing:.3f} m, C2 corner transition '
        f'{args.corner_transition_length:.3f} m)\n'
        f'Task1 map preview: {preview_output}\n'
        f'Task1 schematic preview: {schematic_preview_output}')


if __name__ == '__main__':
    main()
