#!/usr/bin/env python3
"""Generate the editable Task1 high route."""

from generate_route_profile import generate_task1_profile


# 当前先复制 low 路线；待调整：Task1 high 控制点。
TASK1_CONTROL_POINTS = [
    (0.00, 0.00),
    (0.40, 0.00),
    (1.00, 0.40),
    (1.50, 0.35),
    (2.10, 0.30),
    (2.60, 0.25),
    (3.10, 0.30),
    (3.80, 0.70),
    (3.95, 1.10),
]


if __name__ == "__main__":
    raise SystemExit(generate_task1_profile("high", TASK1_CONTROL_POINTS))
