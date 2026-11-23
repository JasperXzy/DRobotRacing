#!/usr/bin/env python3
"""Generate the direct-to-start Task3 fast route."""

from generate_route_profile import generate_task3_profile


# Task3 fast：从通道出口直接连到发车起点。
TASK3_CONTROL_POINTS = [
    (1.95, 1.80), 
    (1.95, 1.60),   #圆滑点
    (-0.10, 0.00),
]


if __name__ == "__main__":
    raise SystemExit(generate_task3_profile("fast", TASK3_CONTROL_POINTS))
