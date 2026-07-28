#!/usr/bin/env python3
"""Generate the Task1 route by equal-distance interpolation of map control points."""

import argparse
import math
from pathlib import Path

import yaml


DEFAULT_CONTROL_POINTS = [
    (0.00, 0.00),
    (1.50, 0.60),
    (3.75, 0.65),
    (4.10, 1.60),
]


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
    args = parser.parse_args()
    if not math.isfinite(args.spacing) or args.spacing <= 0.0:
        raise ValueError('--spacing must be a positive finite number')

    points = interpolate_polyline(DEFAULT_CONTROL_POINTS, args.spacing)
    output = Path(args.output).expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        'route_direction': 'task1',
        'control_points': [list(point) for point in DEFAULT_CONTROL_POINTS],
        'waypoints': [
            {'name': f'task1_{index:03d}', 'x': round(x, 6), 'y': round(y, 6)}
            for index, (x, y) in enumerate(points)
        ],
    }
    with output.open('w', encoding='utf-8') as stream:
        yaml.safe_dump(payload, stream, sort_keys=False, allow_unicode=True)
    print(
        f'Generated {len(points)} Task1 waypoints at {output} '
        f'(spacing <= {args.spacing:.3f} m)')


if __name__ == '__main__':
    main()
