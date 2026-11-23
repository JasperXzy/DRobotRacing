#!/usr/bin/env python3
"""Generate the current Task1 low route."""

from generate_route_profile import generate_task1_profile


# 只需改这里：Task1 low 控制点，按行驶顺序填写。
TASK1_CONTROL_POINTS = [
    (0.00, 0.00),
    (0.50, 0.05),
    (1.00, 0.10),
    (1.50, 0.15),
    (2.80, 0.35),
    (3.20, 0.40),
    (3.60, 0.55),
    (3.80, 0.70),
    (3.95, 1.10),
]


if __name__ == "__main__":
    raise SystemExit(generate_task1_profile("low", TASK1_CONTROL_POINTS))
