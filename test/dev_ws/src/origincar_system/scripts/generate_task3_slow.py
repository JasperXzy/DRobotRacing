#!/usr/bin/env python3
"""Generate the current Task3 slow return route."""

from generate_route_profile import generate_task3_profile


# 只需改这里：Task3 slow 从通道出口返回终点的控制点。
TASK3_CONTROL_POINTS = [
    (1.95, 2.05),
    (1.95, 1.80),
    (1.85, 1.60),
    (1.35, 1.20),
    (1.00, 1.10),
    (0.33, 0.75),
    (0.18, 0.65),
    (0.13, 0.60),
    (0.08, 0.55),
    (0.03, 0.45),
    (0.00, 0.30),
    (0.00, 0.00),
]


if __name__ == "__main__":
    raise SystemExit(generate_task3_profile("slow", TASK3_CONTROL_POINTS))
