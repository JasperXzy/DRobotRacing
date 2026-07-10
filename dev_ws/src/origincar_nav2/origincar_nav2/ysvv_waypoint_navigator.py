#!/usr/bin/env python3
"""Send ysvv_waypoints.yaml to Nav2 one NavigateToPose goal at a time."""

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
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time as RclpyTime
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformListener


def _default_waypoints_file():
    try:
        share_dir = get_package_share_directory('origincar_nav2')
        ws_root = os.path.abspath(os.path.join(share_dir, '..', '..', '..', '..'))
        source_file = os.path.join(
            ws_root, 'src', 'origincar_nav2', 'origincar_nav2', 'ysvv_waypoints.yaml')
        if os.path.exists(source_file):
            return source_file
        return os.path.join(share_dir, 'ysvv_waypoints.yaml')
    except PackageNotFoundError:
        return os.path.join(os.path.dirname(os.path.abspath(__file__)), 'ysvv_waypoints.yaml')


class YsvvWaypointNavigator(Node):
    def __init__(self):
        super().__init__('nav2_ysvv_waypoints')

        self.declare_parameter('waypoints_file', _default_waypoints_file())
        self.declare_parameter('action_name', 'navigate_to_pose')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('start_delay', 0.5)
        self.declare_parameter('publish_initial_pose', True)
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw', 0.0)
        self.declare_parameter('switch_distance', 0.30)
        self.declare_parameter('lookahead_index_offset', 7)
        self.declare_parameter('closest_search_back_window', 5)
        self.declare_parameter('closest_search_forward_window', 30)
        self.declare_parameter('intermediate_goal_yaw_mode', 'current')
        self.declare_parameter('goal_update_min_interval', 0.50)
        self.declare_parameter('goal_republish_distance', 0.12)
        self.declare_parameter('costmap_topic', '/global_costmap/costmap')
        self.declare_parameter('obstacle_escape_distance', 0.50)
        self.declare_parameter('blocked_check_radius', 0.60)
        self.declare_parameter('max_skip_waypoints', 15)
        self.declare_parameter('blocked_cost_threshold', 95)
        self.declare_parameter('obstacle_center_cost_threshold', 80)
        self.declare_parameter('obstacle_ahead_cost_threshold', 80)
        self.declare_parameter('path_check_timeout', 1.0)
        self.declare_parameter('planner_id', 'GridBased')
        self.declare_parameter('exit_on_result', False)

        self.waypoints_file = self.get_parameter('waypoints_file').value
        self.action_name = self.get_parameter('action_name').value
        self.frame_id = self.get_parameter('frame_id').value
        self.base_frame = self.get_parameter('base_frame').value
        self.start_delay = float(self.get_parameter('start_delay').value)
        self.publish_initial_pose_enabled = self.get_parameter('publish_initial_pose').value
        self.initial_x = float(self.get_parameter('initial_x').value)
        self.initial_y = float(self.get_parameter('initial_y').value)
        self.initial_yaw = float(self.get_parameter('initial_yaw').value)
        self.switch_distance = float(self.get_parameter('switch_distance').value)
        self.lookahead_index_offset = int(
            self.get_parameter('lookahead_index_offset').value)
        self.closest_search_back_window = int(
            self.get_parameter('closest_search_back_window').value)
        self.closest_search_forward_window = int(
            self.get_parameter('closest_search_forward_window').value)
        self.intermediate_goal_yaw_mode = str(
            self.get_parameter('intermediate_goal_yaw_mode').value).strip().lower()
        if self.intermediate_goal_yaw_mode not in ('current', 'path_tangent', 'recorded'):
            self.get_logger().warn(
                f'Unsupported intermediate_goal_yaw_mode='
                f'{self.intermediate_goal_yaw_mode!r}; using current')
            self.intermediate_goal_yaw_mode = 'current'
        self.goal_update_min_interval = float(
            self.get_parameter('goal_update_min_interval').value)
        self.goal_republish_distance = float(
            self.get_parameter('goal_republish_distance').value)
        self.costmap_topic = self.get_parameter('costmap_topic').value
        self.obstacle_escape_distance = float(
            self.get_parameter('obstacle_escape_distance').value)
        self.blocked_check_radius = float(self.get_parameter('blocked_check_radius').value)
        self.max_skip_waypoints = int(self.get_parameter('max_skip_waypoints').value)
        self.blocked_cost_threshold = int(
            self.get_parameter('blocked_cost_threshold').value)
        self.obstacle_center_cost_threshold = int(
            self.get_parameter('obstacle_center_cost_threshold').value)
        self.obstacle_ahead_cost_threshold = int(
            self.get_parameter('obstacle_ahead_cost_threshold').value)
        self.path_check_timeout = float(self.get_parameter('path_check_timeout').value)
        self.planner_id = self.get_parameter('planner_id').value
        self.exit_on_result = self.get_parameter('exit_on_result').value

        self.waypoints = self._load_waypoints(self.waypoints_file)
        self.nav_client = ActionClient(self, NavigateToPose, self.action_name)
        self.planner_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 'initialpose', 10)
        self.costmap_sub = self.create_subscription(
            OccupancyGrid, self.costmap_topic, self._costmap_callback, 10)

        self.started = False
        self.route_progress_index = 0
        self.last_logged_progress_index = None
        self.current_waypoint_index = 0
        self.latest_costmap = None
        self.path_check_generation = 0
        self.pending_path_check = None
        self.pending_nav_goal_index = None
        self.last_goal_update_time = None
        self.last_sent_waypoint_index = None
        self.last_sent_goal_xy = None
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(0.2, self._tick)
        self.path_check_timer = self.create_timer(0.2, self._path_check_timeout_tick)
        self.tracking_timer = self.create_timer(0.2, self._tracking_tick)

        self.get_logger().info('=' * 60)
        self.get_logger().info('YSVV waypoint navigator started')
        self.get_logger().info(f'  waypoints: {self.waypoints_file}')
        self.get_logger().info(f'  waypoint count: {len(self.waypoints)}')
        self.get_logger().info('  send mode: point-to-point NavigateToPose')
        self.get_logger().info(f'  action: {self.action_name}')
        self.get_logger().info(f'  start_delay: {self.start_delay:.1f}s')
        self.get_logger().info(f'  switch_distance: {self.switch_distance:.2f}m')
        self.get_logger().info(
            f'  lookahead tracking: {self.frame_id}->{self.base_frame}, '
            f'nearest_index+{self.lookahead_index_offset}, '
            f'search_window=-{self.closest_search_back_window}'
            f'/+{self.closest_search_forward_window}, '
            f'update_interval={self.goal_update_min_interval:.2f}s')
        self.get_logger().info(
            f'  intermediate_goal_yaw_mode: {self.intermediate_goal_yaw_mode}')
        self.get_logger().info(f'  costmap_topic: {self.costmap_topic}')
        self.get_logger().info(
            f'  obstacle_escape_distance: {self.obstacle_escape_distance:.2f}m')
        self.get_logger().info('=' * 60)

    def _load_waypoints(self, file_path):
        if not file_path:
            self.get_logger().error('waypoints_file is empty')
            return []

        file_path = os.path.abspath(os.path.expanduser(file_path))
        if not os.path.exists(file_path):
            self.get_logger().error(f'waypoints_file does not exist: {file_path}')
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

    def _costmap_callback(self, msg):
        self.latest_costmap = msg

    def _tick(self):
        if self.started:
            return

        if not self.waypoints:
            self.get_logger().error('YSVV waypoint list is empty; nothing to navigate')
            self.started = True
            return

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        if elapsed < self.start_delay:
            return

        nav_ready = self.nav_client.wait_for_server(timeout_sec=0.1)
        planner_ready = self.planner_client.wait_for_server(timeout_sec=0.1)
        if not nav_ready or not planner_ready:
            parts = []
            if not nav_ready:
                parts.append(f'BT navigator ({self.action_name})')
            if not planner_ready:
                parts.append('planner_server (compute_path_to_pose)')
            self.get_logger().info(
                f'Waiting for: {", ".join(parts)}',
                throttle_duration_sec=2.0)
            return

        self.started = True
        self.timer.cancel()

        if self.publish_initial_pose_enabled:
            self._publish_initial_pose()

        self._send_next_waypoint()

    def _send_next_waypoint(self):
        lookahead_index = self._select_lookahead_waypoint_index()
        if (lookahead_index is None and self.last_sent_waypoint_index is None and
                self.lookahead_index_offset > 0):
            self.get_logger().warn(
                f'Waiting for TF {self.frame_id}->{self.base_frame} before '
                'sending the first lookahead waypoint',
                throttle_duration_sec=2.0)
            return
        if (lookahead_index is not None and
                lookahead_index > self.current_waypoint_index):
            self.get_logger().info(
                f'YSVV lookahead selected waypoint {lookahead_index + 1} '
                f'instead of {self.current_waypoint_index + 1}')
            self.current_waypoint_index = lookahead_index

        self._request_current_waypoint()

    def _request_current_waypoint(self):
        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().warn('YSVV navigation finished, status=SUCCEEDED')
            if self.exit_on_result and rclpy.ok():
                rclpy.shutdown()
            return

        next_index = self._resolve_upcoming_obstacle(self.current_waypoint_index)
        if next_index is None:
            self._stop_navigation('No safe YSVV waypoint found after obstacle ahead')
            return
        if next_index != self.current_waypoint_index:
            self.current_waypoint_index = next_index

        next_index = self._resolve_blocked_waypoint(self.current_waypoint_index)
        if next_index is None:
            self._stop_navigation('No safe YSVV waypoint found after blocked point')
            return
        if next_index != self.current_waypoint_index:
            self.current_waypoint_index = next_index

        self._request_path_check(self.current_waypoint_index)

    def _send_navigation_goal(self, waypoint_index):
        x, y, recorded_yaw = self.waypoints[waypoint_index]
        goal_yaw, yaw_source = self._goal_yaw_for_index(waypoint_index)
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._waypoint_to_pose((x, y, goal_yaw))
        self.pending_nav_goal_index = waypoint_index
        self.last_goal_update_time = self.get_clock().now()
        self.last_sent_waypoint_index = waypoint_index
        self.last_sent_goal_xy = (x, y)
        self.get_logger().warn(
            f'Sending YSVV waypoint {waypoint_index + 1}/{len(self.waypoints)}: '
            f'({x:.3f}, {y:.3f}, goal_yaw={goal_yaw:.3f}, '
            f'recorded_yaw={recorded_yaw:.3f}, yaw_source={yaw_source})')
        future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=lambda feedback_msg, index=waypoint_index:
                self._feedback_callback(feedback_msg, index))
        future.add_done_callback(
            lambda done_future, index=waypoint_index:
                self._goal_response_callback(done_future, index))

    def _stop_navigation(self, message):
        self.get_logger().error(message)
        if self.exit_on_result and rclpy.ok():
            rclpy.shutdown()

    def _tracking_tick(self):
        if not self.started:
            return
        self._maybe_update_lookahead_goal('timer')

    def _maybe_update_lookahead_goal(self, reason):
        if self.current_waypoint_index + 1 >= len(self.waypoints):
            return
        if self.pending_path_check or self.pending_nav_goal_index is not None:
            return

        now = self.get_clock().now()
        if self.last_goal_update_time is not None:
            elapsed = (now - self.last_goal_update_time).nanoseconds * 1e-9
            if elapsed < self.goal_update_min_interval:
                return

        next_index = self._select_lookahead_waypoint_index()
        if next_index is None or next_index <= self.current_waypoint_index:
            return

        if (self.last_sent_goal_xy is not None and
                self.last_sent_waypoint_index is not None and
                next_index + 1 < len(self.waypoints)):
            x, y, _ = self.waypoints[next_index]
            moved = math.hypot(
                x - self.last_sent_goal_xy[0],
                y - self.last_sent_goal_xy[1])
            if moved < self.goal_republish_distance:
                return

        self.get_logger().warn(
            f'YSVV lookahead update by {reason}: '
            f'goal waypoint {self.current_waypoint_index + 1} -> {next_index + 1}, '
            f'progress waypoint {self.route_progress_index + 1}')
        self.current_waypoint_index = next_index
        self._request_current_waypoint()

    def _select_lookahead_waypoint_index(self):
        robot_pose = self._lookup_robot_pose()
        if robot_pose is None:
            return None

        robot_x, robot_y, _ = robot_pose
        closest_index, closest_distance, search_start, search_end = (
            self._find_closest_route_index(
                robot_x, robot_y, return_details=True))
        if closest_index is None:
            return None

        previous_progress_index = self.route_progress_index
        if closest_index > self.route_progress_index:
            self.route_progress_index = closest_index

        target_index = min(
            self.route_progress_index + max(0, self.lookahead_index_offset),
            len(self.waypoints) - 1)
        should_log_progress = (
            self.route_progress_index != previous_progress_index or
            self.last_logged_progress_index != self.route_progress_index)
        if should_log_progress:
            self.get_logger().warn(
                f'YSVV current reached/progress waypoint '
                f'{self.route_progress_index + 1}/{len(self.waypoints)} '
                f'(nearest={closest_index + 1}, dist={closest_distance:.3f}m, '
                f'robot=({robot_x:.3f}, {robot_y:.3f}), '
                f'search={search_start + 1}-{search_end + 1}, '
                f'next_goal={target_index + 1})')
            self.last_logged_progress_index = self.route_progress_index
        return max(self.current_waypoint_index, target_index)

    def _find_closest_route_index(self, robot_x, robot_y, return_details=False):
        if not self.waypoints:
            if return_details:
                return (None, None, None, None)
            return None

        search_start = max(
            0,
            self.route_progress_index - max(0, self.closest_search_back_window))
        search_end = min(
            len(self.waypoints) - 1,
            self.route_progress_index + max(1, self.closest_search_forward_window))
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
                self.frame_id,
                self.base_frame,
                RclpyTime())
        except TransformException as exc:
            self.get_logger().warn(
                f'waiting for TF {self.frame_id}->{self.base_frame}: {exc}',
                throttle_duration_sec=2.0)
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z))
        return (translation.x, translation.y, yaw)

    def _request_path_check(self, waypoint_index):
        self.path_check_generation += 1
        generation = self.path_check_generation
        self.pending_path_check = {
            'generation': generation,
            'index': waypoint_index,
            'start_time': self.get_clock().now(),
        }

        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = self._goal_pose_for_waypoint_index(waypoint_index)
        goal_msg.planner_id = self.planner_id
        goal_msg.use_start = False

        future = self.planner_client.send_goal_async(goal_msg)
        future.add_done_callback(
            lambda done_future, gen=generation, index=waypoint_index:
                self._path_goal_response_callback(done_future, gen, index))

    def _path_goal_response_callback(self, future, generation, waypoint_index):
        if not self._is_current_path_check(generation, waypoint_index):
            return

        try:
            goal_handle = future.result()
        except Exception as exc:
            self._handle_path_check_failed(
                waypoint_index, f'YSVV path check send failed: {exc}')
            return

        if not goal_handle.accepted:
            self._handle_path_check_failed(
                waypoint_index, 'YSVV path check was rejected by planner_server')
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda done_future, gen=generation, index=waypoint_index:
                self._path_result_callback(done_future, gen, index))

    def _path_result_callback(self, future, generation, waypoint_index):
        if not self._is_current_path_check(generation, waypoint_index):
            return

        self.pending_path_check = None
        try:
            response = future.result()
        except Exception as exc:
            self._handle_path_check_failed(
                waypoint_index, f'YSVV path check result failed: {exc}')
            return

        path = getattr(response.result, 'path', None)
        path_poses = getattr(path, 'poses', []) if path is not None else []
        if response.status == GoalStatus.STATUS_SUCCEEDED and path_poses:
            self._send_navigation_goal(waypoint_index)
            return

        self._handle_path_check_failed(
            waypoint_index, f'YSVV waypoint is unreachable, planner status={response.status}')

    def _path_check_timeout_tick(self):
        if not self.pending_path_check:
            return

        now = self.get_clock().now()
        elapsed = (
            now - self.pending_path_check['start_time']).nanoseconds * 1e-9
        if elapsed < self.path_check_timeout:
            return

        waypoint_index = self.pending_path_check['index']
        self.pending_path_check = None
        self._handle_path_check_failed(
            waypoint_index,
            f'YSVV path check timed out ({self.path_check_timeout:.1f}s)')

    def _is_current_path_check(self, generation, waypoint_index):
        if not self.pending_path_check:
            return False
        return (
            self.pending_path_check['generation'] == generation and
            self.pending_path_check['index'] == waypoint_index and
            waypoint_index == self.current_waypoint_index)

    def _handle_path_check_failed(self, waypoint_index, reason):
        if waypoint_index != self.current_waypoint_index:
            return

        if waypoint_index + 1 >= len(self.waypoints):
            self._stop_navigation(f'{reason}; final waypoint cannot be skipped')
            return

        x, y, _ = self.waypoints[waypoint_index]
        obstacle_center = self._find_obstacle_center((x, y)) or (x, y)
        next_index = self._find_escape_waypoint_index(waypoint_index, obstacle_center)
        if next_index is None:
            self._stop_navigation(
                f'{reason}; no safe waypoint found within the next '
                f'{self.max_skip_waypoints} waypoints')
            return

        self.get_logger().warn(
            f'{reason}; skipping waypoint {waypoint_index + 1}, '
            f'obstacle_center≈({obstacle_center[0]:.3f}, {obstacle_center[1]:.3f}), '
            f'sending waypoint {next_index + 1}')
        self.current_waypoint_index = next_index
        self._request_current_waypoint()

    def _resolve_blocked_waypoint(self, waypoint_index):
        if self.latest_costmap is None:
            self.get_logger().warn(
                'No global costmap received yet; skipping obstacle-center precheck',
                throttle_duration_sec=3.0)
            return waypoint_index

        if not self._is_waypoint_blocked(waypoint_index):
            return waypoint_index

        if waypoint_index + 1 >= len(self.waypoints):
            self.get_logger().error('Final YSVV waypoint is inside a high-cost area')
            return None

        x, y, _ = self.waypoints[waypoint_index]
        blocked_cost = self._cost_at_world(x, y)
        obstacle_center = self._find_obstacle_center((x, y)) or (x, y)
        next_index = self._find_escape_waypoint_index(waypoint_index, obstacle_center)
        if next_index is None:
            self.get_logger().error(
                f'YSVV waypoint {waypoint_index + 1} is blocked, and no safe point '
                f'at least {self.obstacle_escape_distance:.2f}m from the obstacle '
                f'center was found within {self.max_skip_waypoints} waypoints')
            return None

        self.get_logger().warn(
            f'YSVV waypoint {waypoint_index + 1} is in a high-cost area, '
            f'cost={blocked_cost}, '
            f'obstacle_center≈({obstacle_center[0]:.3f}, {obstacle_center[1]:.3f}), '
            f'jumping to waypoint {next_index + 1}')
        return next_index

    def _resolve_upcoming_obstacle(self, waypoint_index):
        if self.latest_costmap is None:
            return waypoint_index

        robot_pose = self._lookup_robot_pose()
        if robot_pose is None:
            return waypoint_index

        robot_x, robot_y, _ = robot_pose
        closest_index = self._find_closest_route_index(robot_x, robot_y)
        if closest_index is None:
            return waypoint_index

        scan_start = max(0, closest_index)
        scan_end = min(
            len(self.waypoints) - 1,
            max(waypoint_index, closest_index + max(1, self.lookahead_index_offset)))

        for blocked_index in range(scan_start, scan_end + 1):
            cost = self._waypoint_cost(blocked_index)
            if not self._is_blocking_cost(cost, self.obstacle_ahead_cost_threshold):
                continue

            if blocked_index + 1 >= len(self.waypoints):
                self.get_logger().error(
                    f'Obstacle ahead reaches final YSVV waypoint {blocked_index + 1}')
                return None

            x, y, _ = self.waypoints[blocked_index]
            obstacle_center = self._find_obstacle_center((x, y)) or (x, y)
            next_index = self._find_escape_waypoint_index(
                blocked_index, obstacle_center)
            if next_index is None:
                self.get_logger().error(
                    f'Obstacle ahead near YSVV waypoint {blocked_index + 1}, '
                    f'cost={cost}, but no safe point at least '
                    f'{self.obstacle_escape_distance:.2f}m from the obstacle center '
                    f'was found within {self.max_skip_waypoints} waypoints')
                return None

            self.get_logger().warn(
                f'Obstacle ahead near YSVV waypoint {blocked_index + 1}, '
                f'cost={cost}, closest={closest_index + 1}, goal={waypoint_index + 1}, '
                f'obstacle_center≈({obstacle_center[0]:.3f}, {obstacle_center[1]:.3f}), '
                f'jumping early to waypoint {next_index + 1}')
            return next_index

        return waypoint_index

    def _find_escape_waypoint_index(self, blocked_index, obstacle_center):
        last_candidate = min(
            len(self.waypoints) - 1, blocked_index + self.max_skip_waypoints)
        for candidate_index in range(blocked_index + 1, last_candidate + 1):
            x, y, _ = self.waypoints[candidate_index]
            distance = math.hypot(x - obstacle_center[0], y - obstacle_center[1])
            if distance < self.obstacle_escape_distance:
                continue
            if self._is_waypoint_blocked(candidate_index):
                continue
            return candidate_index
        return None

    def _is_waypoint_blocked(self, waypoint_index):
        if self.latest_costmap is None:
            return False

        cost = self._waypoint_cost(waypoint_index)
        if cost is None:
            return True
        return self._is_blocking_cost(cost)

    def _waypoint_cost(self, waypoint_index):
        x, y, _ = self.waypoints[waypoint_index]
        return self._cost_at_world(x, y)

    def _find_obstacle_center(self, point):
        if self.latest_costmap is None:
            return None

        center_cell = self._world_to_costmap(point[0], point[1])
        if center_cell is None:
            return None

        info = self.latest_costmap.info
        radius_cells = max(1, int(math.ceil(self.blocked_check_radius / info.resolution)))
        total_x = 0.0
        total_y = 0.0
        count = 0

        min_x = max(0, center_cell[0] - radius_cells)
        max_x = min(info.width - 1, center_cell[0] + radius_cells)
        min_y = max(0, center_cell[1] - radius_cells)
        max_y = min(info.height - 1, center_cell[1] + radius_cells)

        for cell_y in range(min_y, max_y + 1):
            for cell_x in range(min_x, max_x + 1):
                world_x, world_y = self._cell_to_world(cell_x, cell_y)
                if math.hypot(world_x - point[0], world_y - point[1]) > self.blocked_check_radius:
                    continue
                cost = self._cost_at_cell(cell_x, cell_y)
                if self._is_blocking_cost(cost, self.obstacle_center_cost_threshold):
                    total_x += world_x
                    total_y += world_y
                    count += 1

        if count == 0:
            return None
        return (total_x / count, total_y / count)

    def _cost_at_world(self, x, y):
        cell = self._world_to_costmap(x, y)
        if cell is None:
            return None
        return self._cost_at_cell(cell[0], cell[1])

    def _cost_at_cell(self, cell_x, cell_y):
        info = self.latest_costmap.info
        if cell_x < 0 or cell_x >= info.width or cell_y < 0 or cell_y >= info.height:
            return None
        index = cell_y * info.width + cell_x
        if index < 0 or index >= len(self.latest_costmap.data):
            return None
        return self.latest_costmap.data[index]

    def _world_to_costmap(self, x, y):
        if self.latest_costmap is None:
            return None

        info = self.latest_costmap.info
        resolution = info.resolution
        if resolution <= 0.0:
            return None

        origin_x = info.origin.position.x
        origin_y = info.origin.position.y
        cell_x = int(math.floor((x - origin_x) / resolution))
        cell_y = int(math.floor((y - origin_y) / resolution))
        if cell_x < 0 or cell_x >= info.width or cell_y < 0 or cell_y >= info.height:
            return None
        return (cell_x, cell_y)

    def _cell_to_world(self, cell_x, cell_y):
        info = self.latest_costmap.info
        world_x = info.origin.position.x + (cell_x + 0.5) * info.resolution
        world_y = info.origin.position.y + (cell_y + 0.5) * info.resolution
        return (world_x, world_y)

    def _is_blocking_cost(self, cost, threshold=None):
        if cost is None:
            return True
        if threshold is None:
            threshold = self.blocked_cost_threshold
        return cost < 0 or cost >= threshold

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
            f'Published initialpose: ({self.initial_x:.2f}, {self.initial_y:.2f}, '
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
                return (tangent_yaw, 'path_tangent_fallback')
            return (recorded_yaw, 'recorded_fallback')

        tangent_yaw = self._path_tangent_yaw(waypoint_index)
        if tangent_yaw is not None:
            return (tangent_yaw, 'path_tangent')
        return (recorded_yaw, 'recorded_fallback')

    def _path_tangent_yaw(self, waypoint_index):
        x, y, _ = self.waypoints[waypoint_index]

        for other_index in range(waypoint_index + 1, len(self.waypoints)):
            other_x, other_y, _ = self.waypoints[other_index]
            if math.hypot(other_x - x, other_y - y) > 1e-3:
                return math.atan2(other_y - y, other_x - x)

        for other_index in range(waypoint_index - 1, -1, -1):
            other_x, other_y, _ = self.waypoints[other_index]
            if math.hypot(x - other_x, y - other_y) > 1e-3:
                return math.atan2(y - other_y, x - other_x)

        return None

    def _goal_response_callback(self, future, waypoint_index):
        try:
            goal_handle = future.result()
        except Exception as exc:
            if self.pending_nav_goal_index == waypoint_index:
                self.pending_nav_goal_index = None
            self.get_logger().error(
                f'Failed to send YSVV waypoint {waypoint_index + 1}: {exc}')
            return

        if self.pending_nav_goal_index == waypoint_index:
            self.pending_nav_goal_index = None

        if waypoint_index < self.current_waypoint_index:
            self.get_logger().info(
                f'YSVV waypoint {waypoint_index + 1} was superseded; ignoring response')
            return

        if not goal_handle.accepted:
            self.get_logger().error(
                f'YSVV waypoint {waypoint_index + 1} was rejected by Nav2')
            return

        self.get_logger().info(
            f'YSVV waypoint {waypoint_index + 1}/{len(self.waypoints)} accepted by Nav2')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda done_future, index=waypoint_index:
                self._result_callback(done_future, index))

    def _feedback_callback(self, feedback_msg, waypoint_index):
        if waypoint_index != self.current_waypoint_index:
            return

        feedback = feedback_msg.feedback
        distance = getattr(feedback, 'distance_remaining', None)
        if distance is not None:
            self.get_logger().info(
                f'YSVV waypoint {waypoint_index + 1}/{len(self.waypoints)} '
                f'running, progress={self.route_progress_index + 1}, '
                f'distance_remaining={distance:.3f}',
                throttle_duration_sec=2.0)
            if (waypoint_index + 1 < len(self.waypoints) and
                    distance <= self.switch_distance):
                self._maybe_update_lookahead_goal(
                    f'feedback distance={distance:.3f}m')

    def _result_callback(self, future, waypoint_index):
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(
                f'Failed to get YSVV waypoint {waypoint_index + 1} result: {exc}')
            return

        if waypoint_index < self.current_waypoint_index:
            self.get_logger().info(
                f'Ignoring old YSVV waypoint {waypoint_index + 1} result, '
                f'status={result.status}')
            return

        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(
                f'YSVV waypoint {waypoint_index + 1} finished, '
                f'status={result.status}')
            if self.exit_on_result and rclpy.ok():
                rclpy.shutdown()
            return

        self.get_logger().warn(
            f'YSVV waypoint {waypoint_index + 1}/{len(self.waypoints)} succeeded')

        final_waypoint_index = len(self.waypoints) - 1
        if waypoint_index >= final_waypoint_index:
            self.route_progress_index = final_waypoint_index
            self.get_logger().warn(
                f'YSVV current reached/progress waypoint '
                f'{self.route_progress_index + 1}/{len(self.waypoints)} '
                f'(goal succeeded)')
            self.last_logged_progress_index = self.route_progress_index
        else:
            self.get_logger().info(
                'YSVV lookahead goal succeeded; keeping route progress pose-based')

        self.current_waypoint_index = waypoint_index + 1
        self._send_next_waypoint()


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
