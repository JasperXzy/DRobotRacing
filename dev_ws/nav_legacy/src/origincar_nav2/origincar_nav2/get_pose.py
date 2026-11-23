#!/usr/bin/env python3
"""打印当前坐标，格式直接粘到 waypoints yaml 里"""
import math, subprocess, re

def echo_topic(topic, timeout=3):
    out = subprocess.check_output(
        ['ros2', 'topic', 'echo', '--once', topic],
        timeout=timeout, stderr=subprocess.DEVNULL
    ).decode()
    return out

def parse_pose(output):
    """从 ros2 topic echo 输出里提取 position x/y 和 orientation z/w"""
    # 找到 orientation 块，用其位置锚定 orientation 的 x,y,z,w
    orient_match = re.search(r'orientation:\s*\n\s*x:\s*(-?[\d.]+)\s*\n\s*y:\s*(-?[\d.]+)\s*\n\s*z:\s*(-?[\d.]+)\s*\n\s*w:\s*(-?[\d.]+)', output)
    if orient_match:
        qx_orient, qy_orient, qz, qw = map(float, orient_match.groups())

    # position 在 orientation 上面，找 position 块的 x,y
    pos_match = re.search(r'position:\s*\n\s*x:\s*(-?[\d.]+)\s*\n\s*y:\s*(-?[\d.]+)', output)
    if pos_match:
        x, y = map(float, pos_match.groups())
    else:
        raise Exception("position not found")

    yaw = 2 * math.atan2(qz, qw)
    return x, y, yaw

for topic in ['/amcl_pose', '/chassis/odom', '/chassis/robot_pose']:
    try:
        msg = echo_topic(topic)
        x, y, yaw = parse_pose(msg)
        source = 'AMCL(地图坐标)' if 'amcl' in topic else '里程计'
        print(f"\n[{x:.3f}, {y:.3f}, {yaw:.3f}]  ← {source} ({topic})\n")
        break
    except Exception as e:
        continue
else:
    topics = subprocess.check_output(['ros2', 'topic', 'list'], timeout=3).decode()
    print(f"\n无法获取坐标，当前话题列表:\n{topics}")
