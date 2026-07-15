#!/usr/bin/env python3
"""Send rolling YSVV route windows to Nav2 with NavigateThroughPoses."""

import bisect
import math
import os

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import PackageNotFoundError
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import ComputePathThroughPoses
from nav2_msgs.action import NavigateThroughPoses
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool
from rclpy.time import Time as RclpyTime
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformListener


MAX_BATCH_SIZE = 10


def _package_source_or_share(relative_path):
    try:
        share_dir = get_package_share_directory('origincar_nav2')
        ws_root = os.path.abspath(
            os.path.join(share_dir, '..', '..', '..', '..'))
        source_file = os.path.join(
            ws_root, 'src', 'origincar_nav2', relative_path)
        if os.path.exists(source_file):
            return source_file
        return os.path.join(share_dir, relative_path)
    except PackageNotFoundError:
        return os.path.join(
            os.path.dirname(os.path.abspath(__file__)), relative_path)


def _default_waypoints_file():
    return _package_source_or_share(
        os.path.join('origincar_nav2', 'ysvv_waypoints.yaml'))


def _default_behavior_tree_file():
    return _package_source_or_share(
        os.path.join(
            'behavior_trees', 'ackermann_ysvv_through_poses_window.xml'))


class YsvvWaypointNavigator(Node):
    IDLE = 'IDLE'
    PATH_CHECK_PENDING = 'PATH_CHECK_PENDING'
    GOAL_PENDING = 'GOAL_PENDING'
    NAVIGATING = 'NAVIGATING'
    CANCELING = 'CANCELING'
    STOPPED = 'STOPPED'
    FINISHED = 'FINISHED'

    def __init__(self):
        super().__init__('nav2_ysvv_waypoints')

        self.declare_parameter('waypoints_file', _default_waypoints_file())
        self.declare_parameter('behavior_tree', _default_behavior_tree_file())
        self.declare_parameter('action_name', 'navigate_through_poses')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('start_delay', 0.5)
        self.declare_parameter('publish_initial_pose', True)
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw', 0.0)

        self.declare_parameter('window_size', 10)
        self.declare_parameter('window_start_offset', 5)
        self.declare_parameter('window_overlap_size', 5)
        self.declare_parameter('goal_update_min_interval', 0.2)
        self.declare_parameter('refresh_distance_threshold', 0.90)
        self.declare_parameter('window_end_reached_radius', 0.30)
        self.declare_parameter('nav_cancel_timeout', 0.2)
        self.declare_parameter('nav_goal_anchor_count', 2)
        self.declare_parameter('nav_goal_first_anchor_offset', 5)
        self.declare_parameter('nav_goal_min_first_distance', 0.55)
        self.declare_parameter('closest_search_back_window', 5)
        self.declare_parameter('closest_search_forward_window', 20)
        self.declare_parameter('intermediate_goal_yaw_mode', 'path_tangent')
        self.declare_parameter('tangent_yaw_lookahead_distance', 0.45)

        self.declare_parameter('feedback_stall_timeout', 4.0)
        self.declare_parameter('feedback_stall_distance_epsilon', 0.02)
        self.declare_parameter('enable_path_precheck', True)
        self.declare_parameter('path_check_timeout', 1.0)
        self.declare_parameter('path_check_max_retries', 2)
        self.declare_parameter('path_check_retry_delay', 0.5)
        self.declare_parameter('navigation_max_retries', 2)
        self.declare_parameter('planner_id', 'GridBased')

        self.declare_parameter('costmap_topic', '/global_costmap/costmap')
        self.declare_parameter('route_blocked_cost_threshold', 80)
        self.declare_parameter('unknown_is_blocked', True)
        self.declare_parameter('obstacle_entry_buffer', 0.35)
        self.declare_parameter('obstacle_exit_buffer', 0.35)
        self.declare_parameter('obstacle_merge_gap', 0.25)
        self.declare_parameter('obstacle_scan_waypoints', 60)
        self.declare_parameter(
            'obstacle_avoidance_topic', '/ysvv_obstacle_avoidance_active')
        self.declare_parameter('exit_on_result', False)

        self.waypoints_file = self.get_parameter('waypoints_file').value
        self.behavior_tree = self.get_parameter('behavior_tree').value
        self.action_name = self.get_parameter('action_name').value
        self.frame_id = self.get_parameter('frame_id').value
        self.base_frame = self.get_parameter('base_frame').value
        self.start_delay = float(self.get_parameter('start_delay').value)
        self.publish_initial_pose_enabled = bool(
            self.get_parameter('publish_initial_pose').value)
        self.initial_x = float(self.get_parameter('initial_x').value)
        self.initial_y = float(self.get_parameter('initial_y').value)
        self.initial_yaw = float(self.get_parameter('initial_yaw').value)

        requested_window_size = int(self.get_parameter('window_size').value)
        self.window_size = max(1, min(MAX_BATCH_SIZE, requested_window_size))
        if requested_window_size != self.window_size:
            self.get_logger().warn(
                f'window_size={requested_window_size} is outside 1-'
                f'{MAX_BATCH_SIZE}; using {self.window_size}')
        self.window_start_offset = max(
            1, int(self.get_parameter('window_start_offset').value))
        requested_overlap = int(
            self.get_parameter('window_overlap_size').value)
        if self.window_size == 1:
            self.window_overlap_size = 0
        else:
            self.window_overlap_size = max(
                1, min(self.window_size - 1, requested_overlap))
        self.goal_update_min_interval = max(
            0.0, float(self.get_parameter('goal_update_min_interval').value))
        self.refresh_distance_threshold = max(
            0.0,
            float(self.get_parameter('refresh_distance_threshold').value))
        self.window_end_reached_radius = max(
            0.0,
            float(self.get_parameter('window_end_reached_radius').value))
        self.nav_cancel_timeout = max(
            0.1, float(self.get_parameter('nav_cancel_timeout').value))
        requested_anchor_count = int(
            self.get_parameter('nav_goal_anchor_count').value)
        self.nav_goal_anchor_count = max(1, min(5, requested_anchor_count))
        if requested_anchor_count != self.nav_goal_anchor_count:
            self.get_logger().warn(
                f'nav_goal_anchor_count={requested_anchor_count} is outside '
                f'1-5; using {self.nav_goal_anchor_count}')
        self.nav_goal_first_anchor_offset = max(
            1, int(self.get_parameter('nav_goal_first_anchor_offset').value))
        self.nav_goal_min_first_distance = max(
            0.0,
            float(self.get_parameter('nav_goal_min_first_distance').value))
        self.closest_search_back_window = max(
            0, int(self.get_parameter('closest_search_back_window').value))
        self.closest_search_forward_window = max(
            1, int(self.get_parameter('closest_search_forward_window').value))
        self.intermediate_goal_yaw_mode = str(
            self.get_parameter(
                'intermediate_goal_yaw_mode').value).strip().lower()
        if self.intermediate_goal_yaw_mode not in (
                'current', 'path_tangent', 'recorded'):
            self.get_logger().warn(
                f'Unsupported intermediate_goal_yaw_mode='
                f'{self.intermediate_goal_yaw_mode!r}; using path_tangent')
            self.intermediate_goal_yaw_mode = 'path_tangent'
        self.tangent_yaw_lookahead_distance = max(
            0.0,
            float(self.get_parameter('tangent_yaw_lookahead_distance').value))

        self.feedback_stall_timeout = max(
            0.0, float(self.get_parameter('feedback_stall_timeout').value))
        self.feedback_stall_distance_epsilon = max(
            0.0,
            float(self.get_parameter('feedback_stall_distance_epsilon').value))
        self.enable_path_precheck = bool(
            self.get_parameter('enable_path_precheck').value)
        self.path_check_timeout = max(
            0.1, float(self.get_parameter('path_check_timeout').value))
        self.path_check_max_retries = max(
            0, int(self.get_parameter('path_check_max_retries').value))
        self.path_check_retry_delay = max(
            0.0, float(self.get_parameter('path_check_retry_delay').value))
        self.navigation_max_retries = max(
            0, int(self.get_parameter('navigation_max_retries').value))
        self.planner_id = self.get_parameter('planner_id').value

        self.costmap_topic = self.get_parameter('costmap_topic').value
        self.route_blocked_cost_threshold = int(
            self.get_parameter('route_blocked_cost_threshold').value)
        self.unknown_is_blocked = bool(
            self.get_parameter('unknown_is_blocked').value)
        self.obstacle_entry_buffer = max(
            0.0, float(self.get_parameter('obstacle_entry_buffer').value))
        self.obstacle_exit_buffer = max(
            0.0, float(self.get_parameter('obstacle_exit_buffer').value))
        self.obstacle_merge_gap = max(
            0.0, float(self.get_parameter('obstacle_merge_gap').value))
        self.obstacle_scan_waypoints = max(
            self.window_size,
            int(self.get_parameter('obstacle_scan_waypoints').value))
        self.obstacle_avoidance_topic = self.get_parameter(
            'obstacle_avoidance_topic').value
        self.exit_on_result = bool(self.get_parameter('exit_on_result').value)

        self.waypoints = self._load_waypoints(self.waypoints_file)
        self.route_distances = self._build_route_distances()

        self.nav_client = ActionClient(
            self, NavigateThroughPoses, self.action_name)
        self.planner_client = ActionClient(
            self, ComputePathThroughPoses, 'compute_path_through_poses')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 'initialpose', 10)
        self.costmap_sub = self.create_subscription(
            OccupancyGrid, self.costmap_topic, self._costmap_callback, 10)
        self.obstacle_avoidance_pub = self.create_publisher(
            Bool, self.obstacle_avoidance_topic, 10)

        self.started = False
        self.state = self.IDLE
        self.route_progress_index = 0
        self.last_logged_progress_index = None
        self.latest_costmap = None

        self.path_check_generation = 0
        self.pending_path_check = None
        self.path_failure_signature = None
        self.path_failure_count = 0
        self.refresh_retry_at_ns = None
        self.prepared_batch_indices = []
        self.prepared_goal_indices = []

        self.nav_generation_counter = 0
        self.current_nav_generation = None
        self.active_nav_goal_handle = None
        self.active_batch_indices = []
        self.active_goal_indices = []
        self.active_batch_start_progress = 0
        self.pending_preempt = None
        self.refresh_requested = False
        self.cancel_started_ns = None
        self.last_goal_update_ns = None
        self.navigation_failure_signature = None
        self.navigation_failure_count = 0

        self.next_retry_at_ns = None
        self.last_feedback_distance = None
        self.last_feedback_progress_index = None
        self.last_feedback_improvement_ns = None

        self.obstacle_avoidance_active = False
        self.obstacle_avoidance_until_index = None
        self.last_obstacle_interval_signature = None

        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(0.2, self._tick)
        self.timeout_timer = self.create_timer(0.2, self._timeout_tick)
        self.tracking_timer = self.create_timer(0.2, self._tracking_tick)

        self.get_logger().info('=' * 60)
        self.get_logger().info('YSVV rolling-window navigator started')
        self.get_logger().info(f'  waypoints: {self.waypoints_file}')
        self.get_logger().info(f'  waypoint count: {len(self.waypoints)}')
        self.get_logger().info(
            f'  send mode: NavigateThroughPoses, max {self.window_size} poses')
        self.get_logger().info(
            f'  rolling window: start_offset={self.window_start_offset}, '
            f'overlap={self.window_overlap_size}, '
            f'min_interval={self.goal_update_min_interval:.2f}s, '
            f'end_reached_radius={self.window_end_reached_radius:.2f}m')
        self.get_logger().info(
            f'  Nav2 anchors: count={self.nav_goal_anchor_count}, '
            f'first_offset={self.nav_goal_first_anchor_offset}, '
            f'min_first_distance={self.nav_goal_min_first_distance:.2f}m, '
            f'tangent_lookahead={self.tangent_yaw_lookahead_distance:.2f}m')
        self.get_logger().info(
            f'  batch path precheck: {self.enable_path_precheck}')
        self.get_logger().info(
            f'  obstacle route buffers: '
            f'entry={self.obstacle_entry_buffer:.2f}m, '
            f'exit={self.obstacle_exit_buffer:.2f}m, '
            f'merge_gap={self.obstacle_merge_gap:.2f}m, '
            f'cost_threshold={self.route_blocked_cost_threshold}')
        self.get_logger().info(f'  behavior_tree: {self.behavior_tree}')
        self.get_logger().info('=' * 60)

    def _load_waypoints(self, file_path):
        if not file_path:
            self.get_logger().error('waypoints_file is empty')
            return []

        file_path = os.path.abspath(os.path.expanduser(file_path))
        if not os.path.exists(file_path):
            self.get_logger().error(
                f'waypoints_file does not exist: {file_path}')
            return []

        try:
            with open(file_path, 'r') as stream:
                data = yaml.safe_load(stream) or {}
            waypoints = data.get('waypoints', [])
        except Exception as exc:
            self.get_logger().error(f'Failed to read waypoints: {exc}')
            return []

        result = []
        for index, waypoint in enumerate(waypoints, start=1):
            if not isinstance(waypoint, (list, tuple)) or len(waypoint) != 3:
                self.get_logger().error(
                    f'Waypoint #{index} must be [x, y, yaw], got: {waypoint}')
                return []
            try:
                result.append(tuple(float(value) for value in waypoint))
            except (TypeError, ValueError) as exc:
                self.get_logger().error(f'Waypoint #{index} is invalid: {exc}')
                return []
        return result

    def _build_route_distances(self):
        if not self.waypoints:
            return []
        distances = [0.0]
        for index in range(1, len(self.waypoints)):
            previous = self.waypoints[index - 1]
            current = self.waypoints[index]
            distances.append(
                distances[-1] + math.hypot(
                    current[0] - previous[0], current[1] - previous[1]))
        return distances

    def _costmap_callback(self, msg):
        self.latest_costmap = msg

    def _tick(self):
        if self.started:
            return
        if not self.waypoints:
            self._stop_navigation('YSVV waypoint list is empty')
            self.started = True
            return

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        if elapsed < self.start_delay:
            return

        nav_ready = self.nav_client.wait_for_server(timeout_sec=0.1)
        planner_ready = self.planner_client.wait_for_server(timeout_sec=0.1)
        if not nav_ready or not planner_ready:
            waiting = []
            if not nav_ready:
                waiting.append(self.action_name)
            if not planner_ready:
                waiting.append('compute_path_through_poses')
            self.get_logger().info(
                f'Waiting for Nav2 actions: {", ".join(waiting)}',
                throttle_duration_sec=2.0)
            return

        self.started = True
        self.timer.cancel()
        if self.publish_initial_pose_enabled:
            self._publish_initial_pose()
        self._request_new_batch('initial window')

    def _tracking_tick(self):
        if not self.started or self.state in (self.STOPPED, self.FINISHED):
            return

        self._update_route_progress()

        now_ns = self.get_clock().now().nanoseconds
        if self.state == self.IDLE:
            if self.pending_path_check:
                return
            if self.prepared_batch_indices:
                self._send_prepared_batch('old batch already finished')
                return
            if (self.next_retry_at_ns is None or
                    now_ns >= self.next_retry_at_ns):
                self.next_retry_at_ns = None
                self._request_new_batch('rolling update')
            return

        if self.state != self.NAVIGATING or not self.active_batch_indices:
            return
        if self.active_batch_indices[-1] >= len(self.waypoints) - 1:
            return
        if not self._goal_update_interval_elapsed(now_ns):
            return
        if (self.pending_path_check or self.prepared_batch_indices or
                self.pending_preempt):
            return
        if (self.refresh_retry_at_ns is not None and
                now_ns < self.refresh_retry_at_ns):
            return

        reached, end_index, distance = self._active_window_end_reached()
        if reached:
            self._request_refresh(
                f'window end waypoint {end_index + 1} reached '
                f'within {distance:.3f}m')

    def _timeout_tick(self):
        now_ns = self.get_clock().now().nanoseconds
        if self.pending_path_check:
            elapsed = (
                now_ns - self.pending_path_check['start_ns']) * 1e-9
            if elapsed >= self.path_check_timeout:
                pending = self.pending_path_check
                self.pending_path_check = None
                self.path_check_generation += 1
                goal_handle = pending.get('goal_handle')
                if goal_handle is not None:
                    goal_handle.cancel_goal_async()
                if not pending['is_refresh']:
                    self.state = self.IDLE
                self._handle_path_check_failed(
                    pending['indices'],
                    pending.get('goal_indices', []),
                    f'batch path check timed out after {elapsed:.1f}s',
                    pending['is_refresh'])

        if (self.state == self.CANCELING and
                self.cancel_started_ns is not None):
            elapsed = (now_ns - self.cancel_started_ns) * 1e-9
            if elapsed >= self.nav_cancel_timeout:
                generation = self.current_nav_generation
                self.get_logger().warn(
                    f'NavigateThroughPoses cancel timed out after '
                    f'{elapsed:.1f}s; '
                    'sending the refreshed batch as a Nav2 preemption')
                self._complete_cancellation(generation, 'cancel timeout')

    def _request_new_batch(self, reason):
        if self.state != self.IDLE:
            return
        if not self._update_route_progress():
            self.get_logger().warn(
                f'Waiting for TF {self.frame_id}->{self.base_frame} before '
                'building a route window',
                throttle_duration_sec=2.0)
            return

        final_index = len(self.waypoints) - 1
        if self.route_progress_index >= final_index:
            self._finish_navigation('robot reached final route point')
            return

        indices, intervals, error = self._build_goal_window()
        if error:
            self._stop_navigation(error)
            return
        if not indices:
            self._stop_navigation(
                'No safe route poses are available for the next batch')
            return

        self._register_obstacle_intervals(indices, intervals)

        if self.enable_path_precheck:
            self._request_path_check(indices, reason)
        else:
            self._send_navigation_goal(indices)

    def _build_goal_window(self, requested_start_index=None):
        final_index = len(self.waypoints) - 1
        if requested_start_index is None:
            requested_start_index = (
                self.route_progress_index + self.window_start_offset)
        start_index = min(
            final_index,
            max(self.route_progress_index + 1, requested_start_index))
        scan_end = min(
            final_index,
            start_index + self.obstacle_scan_waypoints - 1)
        intervals = self._blocked_route_intervals(start_index, scan_end)

        if self._route_point_is_blocked(final_index):
            for interval in intervals:
                if interval[0] <= final_index <= interval[1]:
                    return (
                        [], intervals,
                        'Final YSVV pose is in a blocked costmap area')

        indices = []
        interval_position = 0
        for index in range(start_index, scan_end + 1):
            while (interval_position < len(intervals) and
                   intervals[interval_position][1] < index):
                interval_position += 1
            if (interval_position < len(intervals) and
                    intervals[interval_position][0] <= index <=
                    intervals[interval_position][1]):
                continue
            indices.append(index)
            if len(indices) >= self.window_size:
                break

        if not indices and scan_end < final_index:
            return (
                [], intervals,
                f'Obstacle exclusion extends beyond the next '
                f'{self.obstacle_scan_waypoints} route poses')
        if not indices:
            return (
                [], intervals,
                'No safe pose remains before the final route point')
        return (indices, intervals, None)

    def _register_obstacle_intervals(self, indices, intervals):
        if not indices:
            return
        relevant_intervals = [
            interval for interval in intervals
            if interval[0] <= indices[-1]
        ]
        if not relevant_intervals:
            return
        until_index = max(interval[1] for interval in relevant_intervals)
        signature = tuple(relevant_intervals)
        if signature != self.last_obstacle_interval_signature:
            self.get_logger().warn(
                f'YSVV obstacle route intervals '
                f'{self._format_intervals(relevant_intervals)}; '
                f'window={self._format_indices(indices)}')
            self.last_obstacle_interval_signature = signature
        self._set_obstacle_avoidance(
            True, 'costmap route interval filtering', until_index)

    def _blocked_route_intervals(self, scan_start, scan_end):
        if self.latest_costmap is None or scan_start > scan_end:
            return []

        blocked_runs = []
        run_start = None
        for index in range(scan_start, scan_end + 1):
            blocked = self._route_point_is_blocked(index)
            if blocked and run_start is None:
                run_start = index
            elif not blocked and run_start is not None:
                blocked_runs.append((run_start, index - 1))
                run_start = None
        if run_start is not None:
            blocked_runs.append((run_start, scan_end))

        expanded = []
        for run_start, run_end in blocked_runs:
            start_distance = (
                self.route_distances[run_start] - self.obstacle_entry_buffer)
            end_distance = (
                self.route_distances[run_end] + self.obstacle_exit_buffer)
            expanded_start = max(
                scan_start,
                bisect.bisect_left(self.route_distances, start_distance))
            expanded_end = min(
                scan_end,
                bisect.bisect_right(self.route_distances, end_distance) - 1)
            expanded.append((expanded_start, expanded_end))

        merged = []
        for interval in expanded:
            if not merged:
                merged.append(interval)
                continue
            gap_distance = (
                self.route_distances[interval[0]] -
                self.route_distances[merged[-1][1]])
            if gap_distance > self.obstacle_merge_gap:
                merged.append(interval)
                continue
            merged[-1] = (
                merged[-1][0], max(merged[-1][1], interval[1]))
        return merged

    def _nav_goal_indices_for_window(self, window_indices):
        if not window_indices:
            return []

        first_position = min(
            len(window_indices) - 1,
            self.nav_goal_first_anchor_offset - 1)
        anchor_count = min(
            self.nav_goal_anchor_count,
            len(window_indices) - first_position)

        positions = []
        if anchor_count <= 1:
            positions = [len(window_indices) - 1]
        else:
            span = len(window_indices) - 1 - first_position
            for step in range(anchor_count):
                position = first_position + int(
                    math.ceil(span * step / (anchor_count - 1)))
                positions.append(min(len(window_indices) - 1, position))

        goal_indices = []
        for position in positions:
            index = window_indices[position]
            if not goal_indices or goal_indices[-1] != index:
                goal_indices.append(index)

        if goal_indices[-1] != window_indices[-1]:
            goal_indices.append(window_indices[-1])

        return self._filter_near_nav_goal_indices(goal_indices)

    def _filter_near_nav_goal_indices(self, goal_indices):
        if (not goal_indices or
                self.nav_goal_min_first_distance <= 0.0 or
                len(goal_indices) <= 1):
            return list(goal_indices)

        robot_pose = self._lookup_robot_pose()
        if robot_pose is None:
            return list(goal_indices)

        kept = []
        dropped = []
        final_index = len(self.waypoints) - 1
        for index in goal_indices:
            waypoint = self.waypoints[index]
            distance = math.hypot(
                waypoint[0] - robot_pose[0],
                waypoint[1] - robot_pose[1])
            if distance >= self.nav_goal_min_first_distance or index >= final_index:
                kept.append(index)
            else:
                dropped.append((index, distance))

        if not kept:
            kept = [goal_indices[-1]]

        if dropped:
            dropped_text = ', '.join(
                f'{index + 1}({distance:.2f}m)'
                for index, distance in dropped)
            self.get_logger().info(
                f'Skipping near Nav2 anchors {dropped_text}; '
                f'using {self._format_indices(kept)}')
        return kept

    @staticmethod
    def _relaxed_nav_goal_indices(goal_indices):
        goal_indices = list(goal_indices)
        if len(goal_indices) > 2:
            return [goal_indices[0], goal_indices[-1]]
        if len(goal_indices) == 2:
            return [goal_indices[-1]]
        return []

    def _request_path_check(
            self, indices, reason, is_refresh=False, goal_indices=None):
        self.path_check_generation += 1
        generation = self.path_check_generation
        if goal_indices is None:
            goal_indices = self._nav_goal_indices_for_window(indices)
        else:
            goal_indices = list(goal_indices)
        if not goal_indices:
            self._handle_path_check_failed(
                indices, goal_indices, 'no Nav2 anchors are available',
                is_refresh)
            return
        self.pending_path_check = {
            'generation': generation,
            'indices': list(indices),
            'goal_indices': list(goal_indices),
            'start_ns': self.get_clock().now().nanoseconds,
            'goal_handle': None,
            'is_refresh': is_refresh,
        }
        if not is_refresh:
            self.state = self.PATH_CHECK_PENDING

        goal_msg = ComputePathThroughPoses.Goal()
        goal_msg.goals = self._goal_poses_for_nav_indices(goal_indices)
        goal_msg.planner_id = self.planner_id
        goal_msg.use_start = False

        self.get_logger().info(
            f'Checking YSVV window {self._format_indices(indices)} via '
            f'anchors {self._format_indices(goal_indices)}: {reason}')
        future = self.planner_client.send_goal_async(goal_msg)
        future.add_done_callback(
            lambda done_future, gen=generation:
                self._path_goal_response_callback(done_future, gen))

    def _path_goal_response_callback(self, future, generation):
        pending = self._current_path_check(generation)
        if pending is None:
            return
        try:
            goal_handle = future.result()
        except Exception as exc:
            indices = pending['indices']
            goal_indices = pending['goal_indices']
            is_refresh = pending['is_refresh']
            self.pending_path_check = None
            if not is_refresh:
                self.state = self.IDLE
            self._handle_path_check_failed(
                indices, goal_indices,
                f'batch path check send failed: {exc}', is_refresh)
            return

        if not goal_handle.accepted:
            indices = pending['indices']
            goal_indices = pending['goal_indices']
            is_refresh = pending['is_refresh']
            self.pending_path_check = None
            if not is_refresh:
                self.state = self.IDLE
            self._handle_path_check_failed(
                indices, goal_indices,
                'batch path check was rejected by planner_server',
                is_refresh)
            return

        pending['goal_handle'] = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda done_future, gen=generation:
                self._path_result_callback(done_future, gen))

    def _path_result_callback(self, future, generation):
        pending = self._current_path_check(generation)
        if pending is None:
            return
        indices = pending['indices']
        goal_indices = pending['goal_indices']
        is_refresh = pending['is_refresh']
        self.pending_path_check = None
        if not is_refresh:
            self.state = self.IDLE
        try:
            response = future.result()
        except Exception as exc:
            self._handle_path_check_failed(
                indices, goal_indices,
                f'batch path check result failed: {exc}', is_refresh)
            return

        path = getattr(response.result, 'path', None)
        path_poses = getattr(path, 'poses', []) if path is not None else []
        if response.status == GoalStatus.STATUS_SUCCEEDED and path_poses:
            self.path_failure_signature = None
            self.path_failure_count = 0
            if is_refresh:
                self._activate_prepared_batch(
                    indices, 'background path precheck succeeded',
                    goal_indices)
            else:
                self._send_navigation_goal(indices, goal_indices)
            return

        self._handle_path_check_failed(
            indices,
            goal_indices,
            f'batch is unreachable, planner status={response.status}',
            is_refresh)

    def _current_path_check(self, generation):
        if not self.pending_path_check:
            return None
        if self.pending_path_check['generation'] != generation:
            return None
        return self.pending_path_check

    def _handle_path_check_failed(
            self, indices, goal_indices, reason, is_refresh=False):
        relaxed_goal_indices = self._relaxed_nav_goal_indices(goal_indices)
        if relaxed_goal_indices:
            self.get_logger().warn(
                f'{reason}; relaxing Nav2 anchors from '
                f'{self._format_indices(goal_indices)} to '
                f'{self._format_indices(relaxed_goal_indices)} for window '
                f'{self._format_indices(indices)}')
            self._request_path_check(
                indices,
                f'{reason}; relaxed anchors',
                is_refresh=is_refresh,
                goal_indices=relaxed_goal_indices)
            return

        signature = (tuple(indices), tuple(goal_indices))
        if signature != self.path_failure_signature:
            self.path_failure_signature = signature
            self.path_failure_count = 0
        self.path_failure_count += 1

        if is_refresh:
            retry_delay = max(0.5, self.path_check_retry_delay)
            self.refresh_retry_at_ns = (
                self.get_clock().now().nanoseconds + int(retry_delay * 1e9))
            if self.path_failure_count <= self.path_check_max_retries:
                self.get_logger().warn(
                    f'{reason}; anchors {self._format_indices(goal_indices)}; '
                    f'old batch remains active, background retry '
                    f'{self.path_failure_count}/{self.path_check_max_retries}')
            else:
                self.get_logger().error(
                    f'{reason}; anchors {self._format_indices(goal_indices)}; '
                    'keeping the old batch instead of deleting additional poses')
                self.path_failure_signature = None
                self.path_failure_count = 0
            return

        if self.path_failure_count <= self.path_check_max_retries:
            self.next_retry_at_ns = (
                self.get_clock().now().nanoseconds +
                int(self.path_check_retry_delay * 1e9))
            self.get_logger().warn(
                f'{reason}; anchors {self._format_indices(goal_indices)}; '
                f'retry {self.path_failure_count}/'
                f'{self.path_check_max_retries} without deleting more poses')
            return

        self._stop_navigation(
            f'{reason}; anchors {self._format_indices(goal_indices)}; '
            f'retry limit reached for batch '
            f'{self._format_indices(indices)}')

    def _activate_prepared_batch(self, indices, reason, goal_indices=None):
        self.refresh_retry_at_ns = None
        if self.state == self.NAVIGATING:
            self._preempt_active_goal(indices, reason, goal_indices)
        elif self.state == self.GOAL_PENDING:
            self.prepared_batch_indices = list(indices)
            self.prepared_goal_indices = list(goal_indices or [])
            self.refresh_requested = True
        elif self.state == self.IDLE:
            self.prepared_batch_indices = list(indices)
            self.prepared_goal_indices = list(goal_indices or [])
            self._send_prepared_batch(reason)

    def _send_prepared_batch(self, reason):
        if self.state != self.IDLE or not self.prepared_batch_indices:
            return
        requested_start = self.prepared_batch_indices[0]
        prepared_indices = list(self.prepared_batch_indices)
        prepared_goal_indices = list(self.prepared_goal_indices)
        indices, intervals, error = self._build_goal_window(requested_start)
        self.prepared_batch_indices = []
        self.prepared_goal_indices = []
        if error:
            self._stop_navigation(error)
            return
        if indices != prepared_indices:
            prepared_goal_indices = []
        self._register_obstacle_intervals(indices, intervals)
        self.get_logger().info(
            f'Switching to prepared YSVV batch '
            f'{self._format_indices(indices)}: {reason}')
        self._send_navigation_goal(indices, prepared_goal_indices or None)

    def _preempt_active_goal(self, indices, reason, goal_indices=None):
        if (self.state != self.NAVIGATING or
                self.active_nav_goal_handle is None or
                self.pending_preempt is not None):
            return

        self.nav_generation_counter += 1
        generation = self.nav_generation_counter
        indices = list(indices)
        if goal_indices is None:
            goal_indices = self._nav_goal_indices_for_window(indices)
        else:
            goal_indices = list(goal_indices)
        if not goal_indices:
            return
        self.prepared_batch_indices = []
        self.prepared_goal_indices = []
        self.pending_preempt = {
            'generation': generation,
            'indices': indices,
            'goal_indices': goal_indices,
            'reason': reason,
            'start_ns': self.get_clock().now().nanoseconds,
        }

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = self._goal_poses_for_nav_indices(goal_indices)
        goal_msg.behavior_tree = self.behavior_tree

        self.get_logger().warn(
            f'Preempting YSVV batch '
            f'{self._format_indices(self.active_batch_indices)} with '
            f'{self._format_indices(indices)} via anchors '
            f'{self._format_indices(goal_indices)} after {reason}; '
            'old batch remains active until the new goal is accepted')
        future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=lambda feedback_msg, gen=generation:
                self._feedback_callback(feedback_msg, gen))
        future.add_done_callback(
            lambda done_future, gen=generation:
                self._preempt_goal_response_callback(done_future, gen))

    def _preempt_goal_response_callback(self, future, generation):
        pending = self.pending_preempt
        if pending is None or pending['generation'] != generation:
            return

        try:
            goal_handle = future.result()
        except Exception as exc:
            self._handle_preempt_send_failed(
                generation, f'failed to send preempting goal: {exc}')
            return

        if not goal_handle.accepted:
            self._handle_preempt_send_failed(
                generation, 'preempting NavigateThroughPoses goal was rejected')
            return

        old_indices = list(self.active_batch_indices)
        indices = list(pending['indices'])
        goal_indices = list(pending['goal_indices'])
        self.pending_preempt = None
        self.current_nav_generation = generation
        self.active_nav_goal_handle = goal_handle
        self.active_batch_indices = indices
        self.active_goal_indices = goal_indices
        self.active_batch_start_progress = self.route_progress_index
        self.refresh_requested = False
        self.cancel_started_ns = None
        self.last_goal_update_ns = self.get_clock().now().nanoseconds
        self.navigation_failure_signature = None
        self.navigation_failure_count = 0
        self.state = self.NAVIGATING
        self._reset_feedback_watchdog()

        self.get_logger().info(
            f'YSVV direct preemption accepted: '
            f'{self._format_indices(old_indices)} -> '
            f'{self._format_indices(indices)} '
            f'(anchors {self._format_indices(goal_indices)})')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda done_future, gen=generation:
                self._result_callback(done_future, gen))

    def _handle_preempt_send_failed(self, generation, reason):
        pending = self.pending_preempt
        if pending is None or pending['generation'] != generation:
            return
        indices = list(pending['indices'])
        old_result_status = pending.get('old_result_status')
        self.pending_preempt = None
        retry_delay = max(0.2, self.path_check_retry_delay)
        retry_at_ns = (
            self.get_clock().now().nanoseconds + int(retry_delay * 1e9))
        if old_result_status is not None:
            self._invalidate_navigation_goal()
            self.next_retry_at_ns = retry_at_ns
            self.get_logger().warn(
                f'{reason} for {self._format_indices(indices)} after the old '
                f'batch had already finished with status={old_result_status}; '
                f'rebuilding after {retry_delay:.1f}s')
            return
        self.refresh_retry_at_ns = retry_at_ns
        self.get_logger().warn(
            f'{reason} for {self._format_indices(indices)}; '
            f'old batch {self._format_indices(self.active_batch_indices)} '
            f'remains active, retrying after {retry_delay:.1f}s')

    def _send_navigation_goal(self, indices, goal_indices=None):
        if self.state != self.IDLE:
            return
        self.nav_generation_counter += 1
        generation = self.nav_generation_counter
        if goal_indices is None:
            goal_indices = self._nav_goal_indices_for_window(indices)
        else:
            goal_indices = list(goal_indices)
        if not goal_indices:
            self._stop_navigation(
                'No Nav2 anchor poses are available for the next batch')
            return
        self.current_nav_generation = generation
        self.active_batch_indices = list(indices)
        self.active_goal_indices = list(goal_indices)
        self.active_batch_start_progress = self.route_progress_index
        self.active_nav_goal_handle = None
        self.refresh_requested = False
        self.cancel_started_ns = None
        self.last_goal_update_ns = self.get_clock().now().nanoseconds
        self._reset_feedback_watchdog()
        self.state = self.GOAL_PENDING

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = self._goal_poses_for_nav_indices(goal_indices)
        goal_msg.behavior_tree = self.behavior_tree

        self.get_logger().warn(
            f'Sending YSVV window {self._format_indices(indices)} via '
            f'anchors {self._format_indices(goal_indices)} '
            f'({len(goal_indices)} Nav2 poses)')
        future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=lambda feedback_msg, gen=generation:
                self._feedback_callback(feedback_msg, gen))
        future.add_done_callback(
            lambda done_future, gen=generation:
                self._goal_response_callback(done_future, gen))

    def _goal_response_callback(self, future, generation):
        if generation != self.current_nav_generation:
            return
        try:
            goal_handle = future.result()
        except Exception as exc:
            indices = list(self.active_batch_indices)
            self._invalidate_navigation_goal()
            self._handle_navigation_failed(
                indices, f'failed to send NavigateThroughPoses goal: {exc}')
            return

        if not goal_handle.accepted:
            indices = list(self.active_batch_indices)
            self._invalidate_navigation_goal()
            self._handle_navigation_failed(
                indices, 'NavigateThroughPoses goal was rejected by Nav2')
            return

        self.active_nav_goal_handle = goal_handle
        self.state = self.NAVIGATING
        self.get_logger().info(
            f'YSVV batch {self._format_indices(self.active_batch_indices)} '
            f'accepted by Nav2 via anchors '
            f'{self._format_indices(self.active_goal_indices)}')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda done_future, gen=generation:
                self._result_callback(done_future, gen))

        if self.refresh_requested and self.prepared_batch_indices:
            indices = list(self.prepared_batch_indices)
            goal_indices = list(self.prepared_goal_indices)
            self.prepared_batch_indices = []
            self.prepared_goal_indices = []
            self.refresh_requested = False
            self._preempt_active_goal(
                indices, 'refresh prepared while initial goal was pending',
                goal_indices or None)

    def _feedback_callback(self, feedback_msg, generation):
        if generation != self.current_nav_generation:
            return
        self._update_route_progress()

        feedback = feedback_msg.feedback
        distance = getattr(feedback, 'distance_remaining', None)
        remaining = getattr(feedback, 'number_of_poses_remaining', None)
        if distance is None:
            return
        self.get_logger().info(
            f'YSVV batch '
            f'{self._format_indices(self.active_batch_indices)} running, '
            f'anchors={self._format_indices(self.active_goal_indices)}, '
            f'route_progress={self.route_progress_index + 1}, '
            f'nav2_poses_remaining={remaining}, distance={distance:.3f}m',
            throttle_duration_sec=2.0)
        self._update_feedback_watchdog(distance)

    def _result_callback(self, future, generation):
        if generation != self.current_nav_generation:
            return
        indices = list(self.active_batch_indices)
        try:
            response = future.result()
        except Exception as exc:
            self._invalidate_navigation_goal()
            self._handle_navigation_failed(
                indices,
                f'failed to receive NavigateThroughPoses result: {exc}')
            return

        if self.pending_preempt is not None:
            self.pending_preempt['old_result_status'] = response.status
            self.get_logger().info(
                f'Old YSVV batch result status={response.status} arrived '
                'while direct preemption is awaiting acceptance')
            return

        if (self.state == self.CANCELING or
                response.status == GoalStatus.STATUS_CANCELED):
            self._complete_cancellation(
                generation, f'action result status={response.status}')
            return

        self._invalidate_navigation_goal()
        if response.status != GoalStatus.STATUS_SUCCEEDED:
            self._handle_navigation_failed(
                indices,
                f'NavigateThroughPoses finished with status={response.status}')
            return

        self.navigation_failure_signature = None
        self.navigation_failure_count = 0
        self._update_route_progress()
        if indices and indices[-1] > self.route_progress_index:
            self.route_progress_index = indices[-1]
            self._update_obstacle_avoidance_progress()
            self.get_logger().info(
                f'YSVV route progress advanced to active window end '
                f'{self.route_progress_index + 1}/{len(self.waypoints)} '
                'after NavigateThroughPoses succeeded')
        self.get_logger().warn(
            f'YSVV batch {self._format_indices(indices)} succeeded')
        if indices and indices[-1] >= len(self.waypoints) - 1:
            self.route_progress_index = len(self.waypoints) - 1
            self._finish_navigation('final batch succeeded')
            return
        if self.pending_path_check and self.pending_path_check['is_refresh']:
            self.get_logger().info(
                'Previous batch finished while the next batch precheck is '
                'still running')
            return
        if self.prepared_batch_indices:
            self._send_prepared_batch('previous batch succeeded')
            return
        self._request_new_batch('previous batch succeeded')

    def _handle_navigation_failed(self, indices, reason):
        if self.pending_path_check and self.pending_path_check['is_refresh']:
            self.state = self.IDLE
            self.get_logger().warn(
                f'{reason}; waiting for the background-prepared batch')
            return
        signature = tuple(indices)
        progress_advanced = (
            self.route_progress_index > self.active_batch_start_progress)
        if progress_advanced or signature != self.navigation_failure_signature:
            self.navigation_failure_signature = signature
            self.navigation_failure_count = 0
        self.navigation_failure_count += 1

        if self.navigation_failure_count <= self.navigation_max_retries:
            self.state = self.IDLE
            self.next_retry_at_ns = (
                self.get_clock().now().nanoseconds +
                int(self.path_check_retry_delay * 1e9))
            self.get_logger().warn(
                f'{reason}; rebuilding the window, retry '
                f'{self.navigation_failure_count}/'
                f'{self.navigation_max_retries}')
            return
        self._stop_navigation(f'{reason}; navigation retry limit reached')

    def _request_refresh(self, reason):
        if self.state != self.NAVIGATING:
            return
        if (not self.active_batch_indices or
                self.active_batch_indices[-1] >= len(self.waypoints) - 1):
            return
        if (self.pending_path_check or self.prepared_batch_indices or
                self.pending_preempt):
            return
        if not self._goal_update_interval_elapsed(
                self.get_clock().now().nanoseconds):
            return
        overlap = min(
            self.window_overlap_size, len(self.active_batch_indices))
        if overlap <= 0:
            return
        requested_start = self.active_batch_indices[-overlap]
        indices, intervals, error = self._build_goal_window(requested_start)
        if error:
            self.get_logger().error(
                f'Cannot prepare the next YSVV window: {error}')
            self.refresh_retry_at_ns = (
                self.get_clock().now().nanoseconds +
                int(max(0.5, self.path_check_retry_delay) * 1e9))
            return
        self._register_obstacle_intervals(indices, intervals)
        self.get_logger().info(
            f'Preparing next YSVV batch {self._format_indices(indices)} '
            f'while old batch remains active: {reason}')
        if self.enable_path_precheck:
            self._request_path_check(indices, reason, is_refresh=True)
        else:
            self._activate_prepared_batch(indices, 'precheck disabled')

    def _cancel_active_goal(self, reason):
        if (self.state != self.NAVIGATING or
                self.active_nav_goal_handle is None):
            return
        generation = self.current_nav_generation
        self.state = self.CANCELING
        self.cancel_started_ns = self.get_clock().now().nanoseconds
        self.get_logger().warn(
            f'Refreshing YSVV batch after {reason}; canceling '
            f'{self._format_indices(self.active_batch_indices)}')
        future = self.active_nav_goal_handle.cancel_goal_async()
        future.add_done_callback(
            lambda done_future, gen=generation:
                self._cancel_response_callback(done_future, gen))

    def _cancel_response_callback(self, future, generation):
        if (generation != self.current_nav_generation or
                self.state != self.CANCELING):
            return
        try:
            response = future.result()
            if not response.goals_canceling:
                self.get_logger().warn(
                    'Nav2 returned no goal in canceling state; '
                    'waiting for the action result or cancel timeout')
            else:
                self.get_logger().info(
                    'Nav2 accepted the batch cancellation; waiting for '
                    'the old action result')
        except Exception as exc:
            self.get_logger().warn(
                f'NavigateThroughPoses cancel response failed: {exc}')

    def _complete_cancellation(self, generation, source):
        if generation != self.current_nav_generation:
            return
        self.get_logger().info(f'Old YSVV batch retired by {source}')
        self._invalidate_navigation_goal()
        if self.prepared_batch_indices:
            self._send_prepared_batch('prepared before old batch cancellation')
        else:
            self._request_new_batch('window refresh fallback')

    def _invalidate_navigation_goal(self):
        self.current_nav_generation = None
        self.active_nav_goal_handle = None
        self.active_batch_indices = []
        self.active_goal_indices = []
        self.prepared_goal_indices = []
        self.pending_preempt = None
        self.refresh_requested = False
        self.cancel_started_ns = None
        if self.state not in (self.STOPPED, self.FINISHED):
            self.state = self.IDLE

    def _goal_update_interval_elapsed(self, now_ns):
        if self.last_goal_update_ns is None:
            return True
        return (
            now_ns - self.last_goal_update_ns >=
            int(self.goal_update_min_interval * 1e9))

    def _reset_feedback_watchdog(self):
        now_ns = self.get_clock().now().nanoseconds
        self.last_feedback_distance = None
        self.last_feedback_progress_index = self.route_progress_index
        self.last_feedback_improvement_ns = now_ns

    def _update_feedback_watchdog(self, distance):
        if self.feedback_stall_timeout <= 0.0:
            return
        now_ns = self.get_clock().now().nanoseconds
        progress_advanced = (
            self.last_feedback_progress_index is None or
            self.route_progress_index > self.last_feedback_progress_index)
        distance_improved = (
            self.last_feedback_distance is None or
            distance < self.last_feedback_distance -
            self.feedback_stall_distance_epsilon)
        if progress_advanced or distance_improved:
            self.last_feedback_distance = distance
            self.last_feedback_progress_index = self.route_progress_index
            self.last_feedback_improvement_ns = now_ns
            return

        stalled_for = (
            now_ns - self.last_feedback_improvement_ns) * 1e-9
        if stalled_for < self.feedback_stall_timeout:
            return
        self.last_feedback_distance = distance
        self.last_feedback_progress_index = self.route_progress_index
        self.last_feedback_improvement_ns = now_ns
        self.get_logger().warn(
            f'Feedback stalled for {stalled_for:.1f}s at {distance:.3f}m; '
            'waiting for the active window end before publishing a new batch',
            throttle_duration_sec=2.0)

    def _update_route_progress(self):
        robot_pose = self._lookup_robot_pose()
        if robot_pose is None:
            return False
        robot_x, robot_y, _ = robot_pose
        details = self._find_closest_route_index(
            robot_x, robot_y, return_details=True)
        closest_index, closest_distance, search_start, search_end = details
        if closest_index is None:
            return False

        previous_progress = self.route_progress_index
        if closest_index > self.route_progress_index:
            self.route_progress_index = closest_index
        self._update_obstacle_avoidance_progress()

        if (self.route_progress_index != previous_progress or
                self.last_logged_progress_index != self.route_progress_index):
            self.get_logger().info(
                f'YSVV route progress {self.route_progress_index + 1}/'
                f'{len(self.waypoints)} (nearest={closest_index + 1}, '
                f'distance={closest_distance:.3f}m, '
                f'search={search_start + 1}-{search_end + 1})')
            self.last_logged_progress_index = self.route_progress_index
        return True

    def _active_window_end_reached(self):
        if not self.active_batch_indices:
            return (False, None, float('inf'))
        if self.window_end_reached_radius <= 0.0:
            return (False, self.active_batch_indices[-1], float('inf'))

        end_index = self.active_batch_indices[-1]
        robot_pose = self._lookup_robot_pose()
        if robot_pose is None:
            return (False, end_index, float('inf'))

        end_x, end_y, _ = self.waypoints[end_index]
        distance = math.hypot(end_x - robot_pose[0], end_y - robot_pose[1])
        if distance > self.window_end_reached_radius:
            return (False, end_index, distance)

        previous_progress = self.route_progress_index
        if end_index > self.route_progress_index:
            self.route_progress_index = end_index
            self._update_obstacle_avoidance_progress()
        if (self.route_progress_index != previous_progress or
                self.last_logged_progress_index != self.route_progress_index):
            self.get_logger().info(
                f'YSVV active window end reached: waypoint {end_index + 1}/'
                f'{len(self.waypoints)}, distance={distance:.3f}m')
            self.last_logged_progress_index = self.route_progress_index
        return (True, end_index, distance)

    def _find_closest_route_index(
            self, robot_x, robot_y, return_details=False):
        if not self.waypoints:
            if return_details:
                return (None, None, None, None)
            return None
        search_start = max(
            0, self.route_progress_index - self.closest_search_back_window)
        search_end = min(
            len(self.waypoints) - 1,
            self.route_progress_index + self.closest_search_forward_window)
        closest_index = search_start
        closest_distance = None
        for index in range(search_start, search_end + 1):
            x, y, _ = self.waypoints[index]
            distance = math.hypot(x - robot_x, y - robot_y)
            if closest_distance is None or distance < closest_distance:
                closest_index = index
                closest_distance = distance
        if return_details:
            return (closest_index, closest_distance, search_start, search_end)
        return closest_index

    def _lookup_robot_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.frame_id, self.base_frame, RclpyTime())
        except TransformException as exc:
            self.get_logger().warn(
                f'Waiting for TF {self.frame_id}->{self.base_frame}: {exc}',
                throttle_duration_sec=2.0)
            return None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z))
        return (translation.x, translation.y, yaw)

    def _route_point_is_blocked(self, waypoint_index):
        if self.latest_costmap is None:
            return False
        cost = self._waypoint_cost(waypoint_index)
        if cost is None or cost < 0:
            return self.unknown_is_blocked
        return cost >= self.route_blocked_cost_threshold

    def _waypoint_cost(self, waypoint_index):
        x, y, _ = self.waypoints[waypoint_index]
        return self._cost_at_world(x, y)

    def _cost_at_world(self, x, y):
        cell = self._world_to_costmap(x, y)
        if cell is None:
            return None
        return self._cost_at_cell(cell[0], cell[1])

    def _cost_at_cell(self, cell_x, cell_y):
        info = self.latest_costmap.info
        if (cell_x < 0 or cell_x >= info.width or
                cell_y < 0 or cell_y >= info.height):
            return None
        index = cell_y * info.width + cell_x
        if index < 0 or index >= len(self.latest_costmap.data):
            return None
        return self.latest_costmap.data[index]

    def _world_to_costmap(self, x, y):
        if self.latest_costmap is None:
            return None
        info = self.latest_costmap.info
        if info.resolution <= 0.0:
            return None
        cell_x = int(math.floor(
            (x - info.origin.position.x) / info.resolution))
        cell_y = int(math.floor(
            (y - info.origin.position.y) / info.resolution))
        if (cell_x < 0 or cell_x >= info.width or
                cell_y < 0 or cell_y >= info.height):
            return None
        return (cell_x, cell_y)

    def _set_obstacle_avoidance(self, active, reason='', until_index=None):
        previous_active = self.obstacle_avoidance_active
        previous_until = self.obstacle_avoidance_until_index
        if active:
            self.obstacle_avoidance_active = True
            if until_index is not None:
                if self.obstacle_avoidance_until_index is None:
                    self.obstacle_avoidance_until_index = until_index
                else:
                    self.obstacle_avoidance_until_index = max(
                        self.obstacle_avoidance_until_index, until_index)
        else:
            self.obstacle_avoidance_active = False
            self.obstacle_avoidance_until_index = None

        if (previous_active != self.obstacle_avoidance_active or
                previous_until != self.obstacle_avoidance_until_index):
            message = Bool()
            message.data = self.obstacle_avoidance_active
            self.obstacle_avoidance_pub.publish(message)
        if previous_active != self.obstacle_avoidance_active:
            state = 'ON' if self.obstacle_avoidance_active else 'OFF'
            self.get_logger().warn(
                f'YSVV obstacle avoidance speed mode {state}: {reason}')

    def _update_obstacle_avoidance_progress(self):
        if not self.obstacle_avoidance_active:
            return
        if self.obstacle_avoidance_until_index is None:
            return
        if self.route_progress_index >= self.obstacle_avoidance_until_index:
            self._set_obstacle_avoidance(
                False,
                f'passed route pose {self.obstacle_avoidance_until_index + 1}')
            self.last_obstacle_interval_signature = None

    def _publish_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = Time(sec=0, nanosec=0)
        msg.pose.pose.position.x = self.initial_x
        msg.pose.pose.position.y = self.initial_y
        msg.pose.pose.orientation.z = math.sin(self.initial_yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.initial_yaw / 2.0)
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.068
        self.initial_pose_pub.publish(msg)
        self.get_logger().info(
            f'Published initialpose: '
            f'({self.initial_x:.2f}, {self.initial_y:.2f}, '
            f'yaw={self.initial_yaw:.3f})')

    def _waypoint_to_pose(self, waypoint):
        x, y, yaw = waypoint
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = Time(sec=0, nanosec=0)
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _goal_poses_for_nav_indices(self, indices):
        yaws = self._anchor_yaws_for_nav_indices(indices)
        poses = []
        for index, yaw in zip(indices, yaws):
            x, y, _ = self.waypoints[index]
            poses.append(self._waypoint_to_pose((x, y, yaw)))
        return poses

    def _anchor_yaws_for_nav_indices(self, indices):
        if not indices:
            return []

        robot_pose = self._lookup_robot_pose()
        yaws = []
        for position, index in enumerate(indices):
            yaw = None
            if len(indices) == 1:
                yaw = self._yaw_from_robot_or_route(index, robot_pose)
            elif position == 0:
                yaw = self._yaw_between_waypoints(index, indices[position + 1])
            elif position >= len(indices) - 1:
                yaw = self._yaw_between_waypoints(indices[position - 1], index)
            else:
                yaw = self._yaw_between_waypoints(
                    indices[position - 1], indices[position + 1])

            if yaw is None:
                yaw = self._yaw_from_robot_or_route(index, robot_pose)
            yaws.append(yaw)
        return yaws

    def _yaw_between_waypoints(self, from_index, to_index):
        from_x, from_y, _ = self.waypoints[from_index]
        to_x, to_y, _ = self.waypoints[to_index]
        if math.hypot(to_x - from_x, to_y - from_y) <= 1e-3:
            return None
        return math.atan2(to_y - from_y, to_x - from_x)

    def _yaw_from_robot_or_route(self, waypoint_index, robot_pose):
        x, y, recorded_yaw = self.waypoints[waypoint_index]
        if robot_pose is not None:
            dx = x - robot_pose[0]
            dy = y - robot_pose[1]
            if math.hypot(dx, dy) > 1e-3:
                return math.atan2(dy, dx)
        tangent_yaw = self._path_tangent_yaw(waypoint_index)
        if tangent_yaw is not None:
            return tangent_yaw
        return recorded_yaw

    def _goal_pose_for_waypoint_index(self, waypoint_index):
        x, y, _ = self.waypoints[waypoint_index]
        yaw, _ = self._goal_yaw_for_index(waypoint_index)
        return self._waypoint_to_pose((x, y, yaw))

    def _goal_yaw_for_index(self, waypoint_index):
        _, _, recorded_yaw = self.waypoints[waypoint_index]
        if waypoint_index >= len(self.waypoints) - 1:
            return (recorded_yaw, 'final_recorded')
        if self.intermediate_goal_yaw_mode == 'recorded':
            return (recorded_yaw, 'recorded')
        if self.intermediate_goal_yaw_mode == 'current':
            robot_pose = self._lookup_robot_pose()
            if robot_pose is not None:
                return (robot_pose[2], 'current_robot')
        tangent_yaw = self._path_tangent_yaw(waypoint_index)
        if tangent_yaw is not None:
            return (tangent_yaw, 'path_tangent')
        return (recorded_yaw, 'recorded_fallback')

    def _path_tangent_yaw(self, waypoint_index):
        x, y, _ = self.waypoints[waypoint_index]
        if (self.tangent_yaw_lookahead_distance > 0.0 and
                waypoint_index < len(self.waypoints) - 1 and
                waypoint_index < len(self.route_distances)):
            target_distance = (
                self.route_distances[waypoint_index] +
                self.tangent_yaw_lookahead_distance)
            other_index = bisect.bisect_left(
                self.route_distances,
                target_distance,
                waypoint_index + 1)
            other_index = min(other_index, len(self.waypoints) - 1)
            while other_index < len(self.waypoints):
                other_x, other_y, _ = self.waypoints[other_index]
                if math.hypot(other_x - x, other_y - y) > 1e-3:
                    return math.atan2(other_y - y, other_x - x)
                other_index += 1

        for other_index in range(waypoint_index + 1, len(self.waypoints)):
            other_x, other_y, _ = self.waypoints[other_index]
            if math.hypot(other_x - x, other_y - y) > 1e-3:
                return math.atan2(other_y - y, other_x - x)
        for other_index in range(waypoint_index - 1, -1, -1):
            other_x, other_y, _ = self.waypoints[other_index]
            if math.hypot(x - other_x, y - other_y) > 1e-3:
                return math.atan2(y - other_y, x - other_x)
        return None

    def _finish_navigation(self, reason):
        if self.state == self.FINISHED:
            return
        self.state = self.FINISHED
        self._set_obstacle_avoidance(False, 'navigation finished')
        self.get_logger().warn(f'YSVV navigation finished: {reason}')
        if self.exit_on_result and rclpy.ok():
            rclpy.shutdown()

    def _stop_navigation(self, message):
        self.state = self.STOPPED
        self._set_obstacle_avoidance(False, 'navigation stopped')
        self.get_logger().error(message)
        if self.exit_on_result and rclpy.ok():
            rclpy.shutdown()

    @staticmethod
    def _format_indices(indices):
        if not indices:
            return '[]'
        ranges = []
        range_start = indices[0]
        previous = indices[0]
        for index in indices[1:]:
            if index == previous + 1:
                previous = index
                continue
            ranges.append((range_start, previous))
            range_start = index
            previous = index
        ranges.append((range_start, previous))
        parts = []
        for start, end in ranges:
            if start == end:
                parts.append(str(start + 1))
            else:
                parts.append(f'{start + 1}-{end + 1}')
        return '[' + ', '.join(parts) + ']'

    @staticmethod
    def _format_intervals(intervals):
        return YsvvWaypointNavigator._format_indices([
            index
            for start, end in intervals
            for index in range(start, end + 1)
        ])


def main(args=None):
    rclpy.init(args=args)
    node = YsvvWaypointNavigator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
