#!/usr/bin/env python3
"""Low-rate runtime metrics recorder with state-change-only console output."""

import csv
import json
import math
from pathlib import Path
import statistics
import time

from ai_msgs.msg import PerceptionTargets
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Int32, String


def percentile(values, quantile):
    if not values:
        return math.nan
    ordered = sorted(values)
    index = (len(ordered) - 1) * quantile
    lower = int(math.floor(index))
    upper = int(math.ceil(index))
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (index - lower)


class RuntimePerformanceMonitor(Node):
    def __init__(self):
        super().__init__('runtime_performance_monitor')
        self.declare_parameter(
            'output_root',
            '/root/D_Robot/test/dev_ws/debug_log/runtime_performance')
        self.declare_parameter('summary_rate_hz', 1.0)
        self.declare_parameter('match_record_rate_hz', 5.0)
        self.declare_parameter('cmd_vel_record_rate_hz', 20.0)
        self.declare_parameter('stale_timeout_seconds', 1.0)
        self.declare_parameter('match_degraded_threshold', 0.20)
        self.declare_parameter('match_recovered_threshold', 0.30)

        output_root = Path(self.get_parameter('output_root').value).expanduser()
        session = time.strftime('%Y.%m.%d_%H:%M:%S')
        self.output_dir = output_root / session
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self._session_started = time.monotonic()

        self._metrics_file, self._metrics = self._open_csv(
            'metrics.csv', [
                'wall_time', 'elapsed_seconds', 'monotonic_time',
                'lidar_fps', 'yolo_fps', 'control_fps',
                'control_period_mean_ms', 'control_period_p95_ms',
                'control_period_p99_ms', 'control_period_max_ms',
                'match_score_latest', 'match_score_mean',
            ])
        self._match_file, self._match = self._open_csv(
            'match_score.csv', [
                'wall_time', 'elapsed_seconds', 'monotonic_time', 'score'])
        self._cmd_file, self._cmd = self._open_csv(
            'cmd_vel.csv', [
                'wall_time', 'elapsed_seconds', 'monotonic_time',
                'linear_x', 'linear_y', 'angular_z',
            ])
        self._events_file = (self.output_dir / 'events.jsonl').open(
            'a', encoding='utf-8', buffering=1)

        self._interval_started = time.monotonic()
        self._counts = {'lidar': 0, 'yolo': 0, 'control': 0}
        self._last_seen = {'lidar': None, 'yolo': None, 'control': None, 'match': None}
        self._states = {}
        self._control_last_time = None
        self._control_periods = []
        self._match_values = []
        self._latest_match = math.nan
        self._last_match_record = 0.0
        self._last_cmd_record = 0.0
        self._yolo_shutdown_expected = False

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LaserScan, '/scan', self._on_scan, sensor_qos)
        self.create_subscription(
            PerceptionTargets, '/racing_obstacle_detection',
            self._on_yolo, sensor_qos)
        self.create_subscription(Float32, '/lidar_location/match_score',
                                 self._on_match, sensor_qos)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, sensor_qos)
        self.create_subscription(Int32, '/sign4return', self._on_sign, 10)
        snapshot_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(
            String, '/visual_lidar_path_follower/route_snapshot',
            self._on_route_event, snapshot_qos)
        self.create_subscription(
            String, '/visual_lidar_path_follower/state_event',
            self._on_controller_state_event, 10)

        summary_rate = max(0.1, float(self.get_parameter('summary_rate_hz').value))
        self._match_period = 1.0 / max(
            0.1, float(self.get_parameter('match_record_rate_hz').value))
        self._cmd_period = 1.0 / max(
            0.1, float(self.get_parameter('cmd_vel_record_rate_hz').value))
        self._stale_timeout = max(
            0.2, float(self.get_parameter('stale_timeout_seconds').value))
        self._match_degraded = float(
            self.get_parameter('match_degraded_threshold').value)
        self._match_recovered = max(
            self._match_degraded,
            float(self.get_parameter('match_recovered_threshold').value))
        self.create_timer(1.0 / summary_rate, self._write_summary)

    @staticmethod
    def _wall_time():
        return time.strftime('%Y-%m-%dT%H:%M:%S') + time.strftime('%z')

    def _open_csv(self, filename, fields):
        stream = (self.output_dir / filename).open(
            'a', newline='', encoding='utf-8', buffering=1)
        writer = csv.DictWriter(stream, fieldnames=fields)
        if stream.tell() == 0:
            writer.writeheader()
        return stream, writer

    def _on_scan(self, _message):
        now = time.monotonic()
        self._counts['lidar'] += 1
        self._last_seen['lidar'] = now

    def _on_yolo(self, _message):
        now = time.monotonic()
        self._counts['yolo'] += 1
        self._last_seen['yolo'] = now

    def _on_match(self, message):
        now = time.monotonic()
        score = float(message.data)
        if not math.isfinite(score):
            return
        self._last_seen['match'] = now
        self._latest_match = score
        self._match_values.append(score)
        if now - self._last_match_record >= self._match_period:
            self._match.writerow({
                'wall_time': self._wall_time(),
                'elapsed_seconds': f'{now - self._session_started:.6f}',
                'monotonic_time': f'{now:.6f}',
                'score': f'{score:.6f}',
            })
            self._last_match_record = now

    def _on_cmd_vel(self, message):
        now = time.monotonic()
        self._counts['control'] += 1
        self._last_seen['control'] = now
        if self._control_last_time is not None:
            self._control_periods.append(now - self._control_last_time)
        self._control_last_time = now
        if now - self._last_cmd_record >= self._cmd_period:
            self._cmd.writerow({
                'wall_time': self._wall_time(),
                'elapsed_seconds': f'{now - self._session_started:.6f}',
                'monotonic_time': f'{now:.6f}',
                'linear_x': f'{message.linear.x:.6f}',
                'linear_y': f'{message.linear.y:.6f}',
                'angular_z': f'{message.angular.z:.6f}',
            })
            self._last_cmd_record = now

    def _on_sign(self, message):
        if message.data == 10:
            self._yolo_shutdown_expected = True
            self._change_state('YOLO', 'STOPPED', 'Task1 vision complete')

    def _on_route_event(self, message):
        try:
            data = json.loads(message.data)
            self._write_event('ROUTE', str(data.get('event', 'UNKNOWN')), {
                'session': data.get('session'),
                'frozen_revision': data.get('frozen_revision', 0),
                'green_count': len(data.get('green', [])),
                'red_count': len(data.get('red', [])),
            })
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            self._write_event('ROUTE', 'INVALID_EVENT', {'error': str(exc)})

    def _on_controller_state_event(self, message):
        try:
            data = json.loads(message.data)
            component = str(data.pop('component', 'CONTROLLER')).upper()
            state = str(data.pop('state', 'UNKNOWN'))
            previous = data.pop('previous', None)
            self._write_event(component, state, data, previous)
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            self._write_event('CONTROLLER', 'INVALID_EVENT', {'error': str(exc)})

    def _write_event(self, component, state, details=None, previous=None):
        now = time.monotonic()
        event = {
            'wall_time': self._wall_time(),
            'elapsed_seconds': now - self._session_started,
            'monotonic_time': now,
            'component': component,
            'state': state,
        }
        if previous is not None:
            event['previous'] = previous
        if details:
            event.update(details)
        self._events_file.write(json.dumps(event, ensure_ascii=False) + '\n')

    def _change_state(self, component, state, detail=''):
        previous = self._states.get(component)
        if previous == state:
            return
        self._states[component] = state
        self._write_event(component, state, {'detail': detail}, previous)
        suffix = f' ({detail})' if detail else ''
        print(f'[{self._wall_time()}] {component} {previous or "START"} -> {state}{suffix}',
              flush=True)

    def _health_state(self, component, key, now):
        last_seen = self._last_seen[key]
        if last_seen is None:
            state = 'WAITING'
        elif now - last_seen > self._stale_timeout:
            state = 'STALE'
        else:
            state = 'OK'
        self._change_state(component, state)

    def _update_states(self, now):
        self._health_state('LIDAR', 'lidar', now)
        self._health_state('CONTROL', 'control', now)
        if not self._yolo_shutdown_expected:
            self._health_state('YOLO', 'yolo', now)

        match_last = self._last_seen['match']
        if match_last is None:
            localization_state = 'WAITING'
        elif now - match_last > self._stale_timeout:
            localization_state = 'STALE'
        else:
            previous = self._states.get('LOCALIZATION')
            threshold = (
                self._match_recovered if previous == 'DEGRADED'
                else self._match_degraded)
            localization_state = (
                'DEGRADED' if self._latest_match < threshold else 'OK')
        detail = (
            f'score={self._latest_match:.3f}'
            if math.isfinite(self._latest_match) else '')
        self._change_state('LOCALIZATION', localization_state, detail)

    def _write_summary(self):
        now = time.monotonic()
        elapsed = max(1e-6, now - self._interval_started)
        periods_ms = [period * 1000.0 for period in self._control_periods]
        match_mean = (
            statistics.fmean(self._match_values) if self._match_values else math.nan)
        self._metrics.writerow({
            'wall_time': self._wall_time(),
            'elapsed_seconds': f'{now - self._session_started:.6f}',
            'monotonic_time': f'{now:.6f}',
            'lidar_fps': f'{self._counts["lidar"] / elapsed:.3f}',
            'yolo_fps': f'{self._counts["yolo"] / elapsed:.3f}',
            'control_fps': f'{self._counts["control"] / elapsed:.3f}',
            'control_period_mean_ms': (
                f'{statistics.fmean(periods_ms):.3f}' if periods_ms else ''),
            'control_period_p95_ms': (
                f'{percentile(periods_ms, 0.95):.3f}' if periods_ms else ''),
            'control_period_p99_ms': (
                f'{percentile(periods_ms, 0.99):.3f}' if periods_ms else ''),
            'control_period_max_ms': (
                f'{max(periods_ms):.3f}' if periods_ms else ''),
            'match_score_latest': (
                f'{self._latest_match:.6f}' if math.isfinite(self._latest_match) else ''),
            'match_score_mean': (
                f'{match_mean:.6f}' if math.isfinite(match_mean) else ''),
        })
        self._update_states(now)
        self._counts = {'lidar': 0, 'yolo': 0, 'control': 0}
        self._control_periods.clear()
        self._match_values.clear()
        self._interval_started = now

    def destroy_node(self):
        for stream in (
                self._metrics_file, self._match_file,
                self._cmd_file, self._events_file):
            stream.flush()
            stream.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RuntimePerformanceMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
