#!/usr/bin/env python3
"""Read-only terminal dashboard for runtime performance CSV logs."""

import argparse
import csv
import json
import math
from pathlib import Path
import sys
import time


DEFAULT_ROOT = Path('/root/D_Robot/test/dev_ws/debug_log/runtime_performance')


def number(value):
    try:
        result = float(value)
        return result if math.isfinite(result) else None
    except (TypeError, ValueError):
        return None


def latest_session(root):
    candidates = sorted(
        path for path in root.iterdir()
        if path.is_dir() and (path / 'metrics.csv').is_file())
    if not candidates:
        raise FileNotFoundError(f'未找到性能记录: {root}')
    return candidates[-1]


def read_csv(path):
    if not path.is_file():
        return []
    with path.open('r', newline='', encoding='utf-8') as stream:
        return list(csv.DictReader(stream))


def read_events(path, limit=8):
    if not path.is_file():
        return []
    events = []
    with path.open('r', encoding='utf-8') as stream:
        for line in stream:
            try:
                events.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return events[-limit:]


def metric_line(label, rows, field, unit):
    values = [number(row.get(field)) for row in rows]
    valid = [value for value in values if value is not None]
    if not valid:
        return f'{label:<18} {"--":>8} {"--":>8} {"--":>8} {"--":>8}  {unit}'
    return (
        f'{label:<18} {valid[-1]:>8.2f} {sum(valid) / len(valid):>8.2f} '
        f'{min(valid):>8.2f} {max(valid):>8.2f}  {unit}')


def render(session):
    metrics = read_csv(session / 'metrics.csv')
    commands = read_csv(session / 'cmd_vel.csv')
    events = read_events(session / 'events.jsonl')

    if metrics:
        elapsed = number(metrics[-1].get('elapsed_seconds')) or 0.0
        wall_time = metrics[-1].get('wall_time', '--')
    else:
        elapsed = 0.0
        wall_time = '--'

    lines = [
        'D-Robot Runtime Performance',
        f'批次: {session.name}   已记录: {elapsed:.1f}s   最新数据: {wall_time}',
        '',
        f'{"指标":<16} {"当前":>10} {"本次平均":>8} {"本次最小":>8} {"本次最大":>8}',
        '-' * 64,
        metric_line('雷达 FPS', metrics, 'lidar_fps', 'Hz'),
        metric_line('YOLO FPS', metrics, 'yolo_fps', 'Hz'),
        metric_line('控制 FPS', metrics, 'control_fps', 'Hz'),
        metric_line('Match score', metrics, 'match_score_latest', ''),
        metric_line('控制周期平均', metrics, 'control_period_mean_ms', 'ms'),
        metric_line('控制周期 P95', metrics, 'control_period_p95_ms', 'ms'),
        metric_line('控制周期 P99', metrics, 'control_period_p99_ms', 'ms'),
        metric_line('控制周期最大', metrics, 'control_period_max_ms', 'ms'),
        '',
    ]

    if commands:
        command = commands[-1]
        lines.append(
            '最新 /cmd_vel: '
            f'linear_x={number(command.get("linear_x")) or 0.0:.3f} m/s  '
            f'angular_z={number(command.get("angular_z")) or 0.0:.3f} rad/s')
    else:
        lines.append('最新 /cmd_vel: --')

    lines.extend(['', '最近状态事件:', '-' * 64])
    if not events:
        lines.append('  --')
    for event in events:
        timestamp = str(event.get('wall_time', '--'))[11:19]
        component = str(event.get('component', '--'))
        previous = event.get('previous')
        state = str(event.get('state', '--'))
        transition = f'{previous} -> {state}' if previous is not None else state
        detail = str(event.get('detail', '') or '')
        lines.append(
            f'  {timestamp}  {component:<14} {transition}' +
            (f'  {detail}' if detail else ''))
    lines.extend(['', 'Ctrl-C 退出；仪表盘只读日志，不发布ROS消息。'])
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description='在SSH终端查看性能记录')
    parser.add_argument('--root', type=Path, default=DEFAULT_ROOT)
    parser.add_argument('--session', default='latest', help='时间批次名或 latest')
    parser.add_argument('--interval', type=float, default=1.0, help='刷新间隔秒')
    parser.add_argument('--once', action='store_true', help='只输出一次')
    args = parser.parse_args()

    root = args.root.expanduser()
    try:
        while True:
            session = latest_session(root) if args.session == 'latest' else root / args.session
            output = render(session)
            if args.once or not sys.stdout.isatty():
                print(output)
                return
            print('\033[2J\033[H' + output, end='', flush=True)
            time.sleep(max(0.2, args.interval))
    except FileNotFoundError as exc:
        parser.error(str(exc))
    except KeyboardInterrupt:
        print()


if __name__ == '__main__':
    main()
