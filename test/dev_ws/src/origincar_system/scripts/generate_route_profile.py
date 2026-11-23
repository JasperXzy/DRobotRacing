#!/usr/bin/env python3
"""Shared generators for independently selectable Task1 and Task3 profiles."""

from __future__ import annotations

import math
import re
import sys
from pathlib import Path
from typing import Sequence

import yaml

import generate_channel_waypoints as channel
import generate_task1_waypoints as task1


PACKAGE_DIR = Path(__file__).resolve().parents[1]
CONFIG_DIR = PACKAGE_DIR / "config"
MAP_DIR = PACKAGE_DIR / "map"
DEFAULT_SPACING = 0.02
DEFAULT_TASK1_CORNER_TRANSITION_LENGTH = 0.35
DEFAULT_TASK3_CORNER_TRANSITION_LENGTH = 0.28

Point = tuple[float, float]


def _validate_points(label: str, points: Sequence[Point]) -> list[Point]:
    if len(points) < 2:
        raise ValueError(f"{label} must contain at least two control points")

    validated: list[Point] = []
    for index, point in enumerate(points):
        if len(point) != 2:
            raise ValueError(f"{label}[{index}] must contain x and y")
        x = float(point[0])
        y = float(point[1])
        if not math.isfinite(x) or not math.isfinite(y):
            raise ValueError(f"{label}[{index}] must be finite")
        if validated and channel.distance(validated[-1], (x, y)) <= 1.0e-9:
            raise ValueError(f"{label}[{index}] duplicates the preceding point")
        validated.append((x, y))
    return validated


def _profile_paths(profile_name: str) -> dict[str, Path]:
    return {
        "task1_yaml": CONFIG_DIR / f"waypoints_task1_{profile_name}.yaml",
        "clockwise_yaml": (
            CONFIG_DIR / f"waypoints_channel_clockwise_{profile_name}.yaml"
        ),
        "counterclockwise_yaml": (
            CONFIG_DIR / f"waypoints_channel_counterclockwise_{profile_name}.yaml"
        ),
        "task1_map": (
            MAP_DIR / f"waypoints_task1_{profile_name}_on_restriction_map.png"
        ),
        "task1_schematic": (
            MAP_DIR / f"waypoints_task1_{profile_name}_on_schematic.png"
        ),
        "clockwise_map": (
            MAP_DIR
            / f"waypoints_channel_clockwise_{profile_name}_on_restriction_map.png"
        ),
        "counterclockwise_map": (
            MAP_DIR
            / f"waypoints_channel_counterclockwise_{profile_name}_on_restriction_map.png"
        ),
        "clockwise_schematic": (
            MAP_DIR / f"waypoints_channel_clockwise_{profile_name}_on_schematic.png"
        ),
        "counterclockwise_schematic": (
            MAP_DIR
            / f"waypoints_channel_counterclockwise_{profile_name}_on_schematic.png"
        ),
    }


def _generate_task1(
    profile_name: str,
    control_points: Sequence[Point],
    spacing: float,
    corner_transition_length: float,
    paths: dict[str, Path],
) -> None:
    points = task1.smooth_control_polyline(
        control_points, spacing, corner_transition_length
    )
    payload = {
        "task1_profile": profile_name,
        "route_direction": "task1",
        "control_points": [list(point) for point in control_points],
        "smoothing": {
            "method": "quintic_hermite_c2",
            "corner_transition_length": corner_transition_length,
        },
        "waypoints": [
            {
                "name": f"task1_{index:03d}",
                "x": round(x, 6),
                "y": round(y, 6),
            }
            for index, (x, y) in enumerate(points)
        ],
    }

    output = paths["task1_yaml"]
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(payload, stream, sort_keys=False, allow_unicode=True)

    task1.render_map_preview(
        points, task1.DEFAULT_MAP_YAML, paths["task1_map"]
    )
    task1.render_schematic_preview(
        points,
        task1.DEFAULT_MAP_YAML,
        task1.DEFAULT_SCHEMATIC_BACKGROUND,
        paths["task1_schematic"],
    )
    print(
        f"{profile_name} Task1: generated {len(points)} points -> {output}",
        file=sys.stderr,
    )


def _generate_channel_and_task3(
    profile_name: str,
    task3_control_points: Sequence[Point],
    spacing: float,
    corner_transition_length: float,
    paths: dict[str, Path],
) -> None:
    route_jobs = [
        (
            "clockwise",
            channel.CLOCKWISE_SEGMENTS,
            channel.CLOCKWISE_TRANSITION_HANDLE_RATIOS,
            paths["clockwise_yaml"],
            paths["clockwise_map"],
            paths["clockwise_schematic"],
        ),
        (
            "counterclockwise",
            channel.COUNTERCLOCKWISE_SEGMENTS,
            channel.COUNTERCLOCKWISE_TRANSITION_HANDLE_RATIOS,
            paths["counterclockwise_yaml"],
            paths["counterclockwise_map"],
            paths["counterclockwise_schematic"],
        ),
    ]

    for (
        direction,
        base_segments,
        transition_ratios,
        output,
        preview_output,
        schematic_preview_output,
    ) in route_jobs:
        # Task2 channel geometry is shared. Only its final exit endpoint is
        # connected to the Task3 control points owned by this profile.
        segments = list(base_segments)
        segments[-1] = (segments[-1][0], task3_control_points[0])
        channel_pieces = channel.build_pieces(segments, transition_ratios)
        task3_polyline = channel.smooth_control_polyline(
            task3_control_points, corner_transition_length
        )
        task3_pieces = channel.polyline_pieces(task3_polyline)
        if not task3_pieces:
            raise ValueError("Task3 control points produced an empty route")

        pieces = channel_pieces + task3_pieces
        points = channel.sample_route(pieces, spacing)
        task3_start_distance = sum(piece.length for piece in channel_pieces)
        task3_start_index = channel.first_sample_index_at_distance(
            pieces, points, task3_start_distance
        )
        rendered = channel.render_yaml(
            points,
            spacing,
            direction,
            task3_start_index,
            return_corner_transition_length=corner_transition_length,
            generator_name=f"generate_task3_{profile_name}.py",
            task3_control_points_name="TASK3_CONTROL_POINTS",
            task3_profile=profile_name,
        )

        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(rendered, encoding="utf-8")
        channel.render_map_preview(
            points, channel.DEFAULT_MAP_YAML, preview_output, direction
        )
        channel.render_schematic_preview(
            points,
            channel.DEFAULT_MAP_YAML,
            task1.DEFAULT_SCHEMATIC_BACKGROUND,
            schematic_preview_output,
            direction,
        )

        minimum, maximum, average = channel.spacing_stats(points)
        print(
            f"{profile_name} {direction}: generated {len(points)} points -> "
            f"{output} (spacing min/avg/max="
            f"{minimum:.3f}/{average:.3f}/{maximum:.3f} m)",
            file=sys.stderr,
        )


def _validate_profile_name(profile_name: str) -> None:
    if not re.fullmatch(r"[a-z0-9_]+", profile_name):
        raise ValueError("profile_name may contain only a-z, 0-9 and underscore")


def _validate_spacing(spacing: float) -> None:
    if not math.isfinite(spacing) or spacing <= 0.0:
        raise ValueError("spacing must be a finite positive value")


def _validate_corner_transition_length(label: str, value: float) -> None:
    if not math.isfinite(value) or value < 0.0:
        raise ValueError(f"{label} must be finite and non-negative")


def generate_task1_profile(
    profile_name: str,
    task1_control_points: Sequence[Point],
    *,
    spacing: float = DEFAULT_SPACING,
    task1_corner_transition_length: float = (
        DEFAULT_TASK1_CORNER_TRANSITION_LENGTH
    ),
) -> int:
    """Generate one Task1 YAML plus its map and schematic previews."""
    _validate_profile_name(profile_name)
    _validate_spacing(spacing)
    _validate_corner_transition_length(
        "Task1 corner transition length", task1_corner_transition_length
    )
    task1_points = _validate_points("TASK1_CONTROL_POINTS", task1_control_points)
    paths = _profile_paths(profile_name)
    _generate_task1(
        profile_name,
        task1_points,
        spacing,
        task1_corner_transition_length,
        paths,
    )
    print(f"Task1 {profile_name}: route file and previews generated")
    return 0


def generate_task3_profile(
    profile_name: str,
    task3_control_points: Sequence[Point],
    *,
    spacing: float = DEFAULT_SPACING,
    task3_corner_transition_length: float = (
        DEFAULT_TASK3_CORNER_TRANSITION_LENGTH
    ),
) -> int:
    """Generate one Task3 profile for both shared Task2 channel directions."""
    _validate_profile_name(profile_name)
    _validate_spacing(spacing)
    _validate_corner_transition_length(
        "Task3 corner transition length", task3_corner_transition_length
    )
    task3_points = _validate_points("TASK3_CONTROL_POINTS", task3_control_points)
    paths = _profile_paths(profile_name)
    _generate_channel_and_task3(
        profile_name,
        task3_points,
        spacing,
        task3_corner_transition_length,
        paths,
    )
    print(f"Task3 {profile_name}: route files and previews generated")
    return 0
