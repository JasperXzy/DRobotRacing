#!/usr/bin/env python3
"""Send a fixed waypoint YAML to Nav2 one NavigateToPose goal at a time."""

import math
import os

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time as RclpyTime
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformListener


class FixedWaypointDemo(Node):
    def __init__(self):
        super().__init__('fixed_waypoint_demo')

        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('action_name', 'navigate_to_pose')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('start_delay', 0.5)
        self.declare_parameter('publish_initial_pose', True)
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw', 0.0)
        self.declare_parameter('switch_distance', 0.30)
        self.declare_parameter('lookahead_index_offset', 10)
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
        self.current_waypoint_index = 0
        self.latest_costmap = None
        self.path_check_generation = 0
        self.pending_path_check = None
        self.pending_nav_goal_index = None
        self.last_goal_update_time = None
        self.last_sent_waypoint_index = None
        self.last_sent_goal_xy = None
        self.nav_client = ActionClient(self, NavigateToPose, self.action_name)
        # 全局规划器 action client：确保 planner_server 已就绪再发路径点
        self.planner_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.costmap_sub = self.create_subscription(
            OccupancyGrid, self.costmap_topic, self._costmap_callback, 10)

        # 状态机: INIT -> INITIAL_POSE -> PLANNER_WAIT -> SENT
        self._state = 'INIT'
        self._state_start_time = self.get_clock().now()
        self.timer = self.create_timer(0.2, self._tick)
        self.path_check_timer = self.create_timer(0.2, self._path_check_timeout_tick)
        self.tracking_timer = self.create_timer(0.2, self._tracking_tick)

        self.get_logger().info('=' * 60)
        self.get_logger().info('固定路径 Nav2 demo 已启动')
        self.get_logger().info(f'  waypoints: {self.waypoints_file}')
        self.get_logger().info(f'  waypoint count: {len(self.waypoints)}')
        self.get_logger().info(f'  action: {self.action_name}')
        self.get_logger().info(f'  start_delay: {self.start_delay:.1f}s')
        self.get_logger().info(f'  switch_distance: {self.switch_distance:.2f}m')
        self.get_logger().info(
            f'  lookahead tracking: {self.frame_id}->{self.base_frame}, '
            f'nearest_index+{self.lookahead_index_offset}, '
            f'update_interval={self.goal_update_min_interval:.2f}s')
        self.get_logger().info(f'  costmap_topic: {self.costmap_topic}')
        self.get_logger().info(
            f'  obstacle_escape_distance: {self.obstacle_escape_distance:.2f}m')
        self.get_logger().info('=' * 60)

    def _load_waypoints(self, file_path):
        if not file_path:
            self.get_logger().error('waypoints_file 为空，无法发送固定路径')
            return []

        if not os.path.exists(file_path):
            self.get_logger().error(f'waypoints_file 不存在: {file_path}')
            return []

        try:
            with open(file_path, 'r') as stream:
                data = yaml.safe_load(stream) or {}
            waypoints = data.get('waypoints', [])
            return [tuple(float(v) for v in waypoint) for waypoint in waypoints]
        except Exception as exc:
            self.get_logger().error(f'读取固定路径失败: {exc}')
            return []

    def _costmap_callback(self, msg):
        self.latest_costmap = msg

    def _tick(self):
        if self._state == 'SENT':
            return

        if not self.waypoints:
            self.get_logger().error('固定路径为空，退出 demo')
            self._state = 'SENT'
            return

        now = self.get_clock().now()
        elapsed = (now - self._state_start_time).nanoseconds * 1e-9

        # ━ 阶段1: 等待初始延迟后发布 initial_pose ━
        if self._state == 'INIT':
            if elapsed < self.start_delay:
                return
            if self.publish_initial_pose_enabled:
                self._publish_initial_pose()
            self._state = 'INITIAL_POSE'
            self._state_start_time = now
            self.get_logger().info('initial_pose 已发布，等待 AMCL 收敛 (1s)...')
            return

        # ━ 阶段2: 等待 initial_pose 被 AMCL 吸收 ━
        if self._state == 'INITIAL_POSE':
            if elapsed < 1.0:
                return
            self._state = 'PLANNER_WAIT'
            self._state_start_time = now
            self.get_logger().info('开始等待全局规划器 & 局部规划器就绪...')
            return

        # ━ 阶段3: 等待两个 action server 都就绪 ━
        if self._state == 'PLANNER_WAIT':
            nav_ready = self.nav_client.wait_for_server(timeout_sec=0.1)
            planner_ready = self.planner_client.wait_for_server(timeout_sec=0.1)

            if not nav_ready or not planner_ready:
                parts = []
                if not nav_ready:
                    parts.append(f'BT navigator ({self.action_name})')
                if not planner_ready:
                    parts.append('planner_server (compute_path_to_pose)')
                self.get_logger().info(
                    f'等待: {", ".join(parts)}', throttle_duration_sec=2.0)
                return

            self._state = 'SENT'
            self.timer.cancel()
            self.get_logger().warn(
                f'全局 & 局部规划器已就绪，开始逐点发送固定路径: {len(self.waypoints)} 个点')
            self._send_next_waypoint()

    def _send_next_waypoint(self):
        lookahead_index = self._select_lookahead_waypoint_index()
        if (lookahead_index is None and self.last_sent_waypoint_index is None and
                self.lookahead_index_offset > 0):
            self.get_logger().warn(
                f'等待 TF {self.frame_id}->{self.base_frame} 后再发送第一个前视路径点',
                throttle_duration_sec=2.0)
            return
        if (lookahead_index is not None and
                lookahead_index > self.current_waypoint_index):
            self.get_logger().info(
                f'前视点选择第 {lookahead_index + 1} 个点，'
                f'替代第 {self.current_waypoint_index + 1} 个点')
            self.current_waypoint_index = lookahead_index

        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().warn('固定路径逐点巡航完成，status=SUCCEEDED')
            if self.exit_on_result and rclpy.ok():
                rclpy.shutdown()
            return

        next_index = self._resolve_upcoming_obstacle(self.current_waypoint_index)
        if next_index is None:
            self._stop_navigation('固定路径前方有障碍物，且后续没有可用的安全路径点')
            return
        if next_index != self.current_waypoint_index:
            self.current_waypoint_index = next_index

        next_index = self._resolve_blocked_waypoint(self.current_waypoint_index)
        if next_index is None:
            self._stop_navigation('固定路径后续没有可用的安全路径点')
            return
        if next_index != self.current_waypoint_index:
            self.current_waypoint_index = next_index

        self._request_path_check(self.current_waypoint_index)

    def _send_navigation_goal(self, waypoint_index):
        x, y, yaw = self.waypoints[waypoint_index]
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._waypoint_to_pose((x, y, yaw))
        self.pending_nav_goal_index = waypoint_index
        self.last_goal_update_time = self.get_clock().now()
        self.last_sent_waypoint_index = waypoint_index
        self.last_sent_goal_xy = (x, y)
        self.get_logger().warn(
            f'发送固定路径点 {waypoint_index + 1}/{len(self.waypoints)}: '
            f'({x:.3f}, {y:.3f}, yaw={yaw:.3f})')
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
        if self._state != 'SENT':
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
            f'前视点由 {reason} 更新：'
            f'第 {self.current_waypoint_index + 1} 个点 -> 第 {next_index + 1} 个点')
        self.current_waypoint_index = next_index
        self._send_next_waypoint()

    def _select_lookahead_waypoint_index(self):
        robot_pose = self._lookup_robot_pose()
        if robot_pose is None:
            return None

        robot_x, robot_y, _ = robot_pose
        closest_index = self._find_closest_route_index(robot_x, robot_y)
        if closest_index is None:
            return None

        target_index = min(
            closest_index + max(0, self.lookahead_index_offset),
            len(self.waypoints) - 1)
        return max(self.current_waypoint_index, target_index)

    def _find_closest_route_index(self, robot_x, robot_y):
        if not self.waypoints:
            return None

        search_start = max(0, self.current_waypoint_index - 10)
        closest_index = search_start
        closest_distance = None
        for index in range(search_start, len(self.waypoints)):
            x, y, _ = self.waypoints[index]
            distance = math.hypot(x - robot_x, y - robot_y)
            if closest_distance is None or distance < closest_distance:
                closest_index = index
                closest_distance = distance

        return closest_index

    def _lookup_robot_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.frame_id,
                self.base_frame,
                RclpyTime())
        except TransformException as exc:
            self.get_logger().warn(
                f'等待 TF {self.frame_id}->{self.base_frame}: {exc}',
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
        goal_msg.goal = self._waypoint_to_pose(self.waypoints[waypoint_index])
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
                waypoint_index, f'固定路径点规划检查发送失败: {exc}')
            return

        if not goal_handle.accepted:
            self._handle_path_check_failed(
                waypoint_index, '固定路径点规划检查被 planner_server 拒绝')
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
                waypoint_index, f'固定路径点规划检查结果失败: {exc}')
            return

        path = getattr(response.result, 'path', None)
        path_poses = getattr(path, 'poses', []) if path is not None else []
        if response.status == GoalStatus.STATUS_SUCCEEDED and path_poses:
            self._send_navigation_goal(waypoint_index)
            return

        self._handle_path_check_failed(
            waypoint_index, f'固定路径点不可达，规划状态={response.status}')

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
            f'固定路径点规划检查超时 ({self.path_check_timeout:.1f}s)')

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
            self._stop_navigation(f'{reason}；最后一个点不可跳过，停止')
            return

        x, y, _ = self.waypoints[waypoint_index]
        blocked_cost = self._cost_at_world(x, y)
        obstacle_center = self._find_obstacle_center((x, y)) or (x, y)
        next_index = self._find_escape_waypoint_index(waypoint_index, obstacle_center)
        if next_index is None:
            self._stop_navigation(
                f'{reason}；从第 {waypoint_index + 1} 个点往后 '
                f'{self.max_skip_waypoints} 个点内没有找到安全点')
            return

        self.get_logger().warn(
            f'{reason}；跳过第 {waypoint_index + 1} 个点，'
            f'障碍物中心≈({obstacle_center[0]:.3f}, {obstacle_center[1]:.3f})，'
            f'改发第 {next_index + 1} 个点')
        self.current_waypoint_index = next_index
        self._send_next_waypoint()

    def _resolve_blocked_waypoint(self, waypoint_index):
        if self.latest_costmap is None:
            self.get_logger().warn(
                '还没有收到 global costmap，跳过障碍物中心预检查，仅做路径规划检查',
                throttle_duration_sec=3.0)
            return waypoint_index

        if not self._is_waypoint_blocked(waypoint_index):
            return waypoint_index

        if waypoint_index + 1 >= len(self.waypoints):
            self.get_logger().error('最后一个固定路径点在障碍物/高代价区域内')
            return None

        x, y, _ = self.waypoints[waypoint_index]
        blocked_cost = self._cost_at_world(x, y)
        obstacle_center = self._find_obstacle_center((x, y)) or (x, y)
        next_index = self._find_escape_waypoint_index(waypoint_index, obstacle_center)
        if next_index is None:
            self.get_logger().error(
                f'固定路径点 {waypoint_index + 1} 被阻塞，且后续 '
                f'{self.max_skip_waypoints} 个点内没有离障碍物中心 '
                f'{self.obstacle_escape_distance:.2f}m 外的安全点')
            return None

        self.get_logger().warn(
            f'固定路径点 {waypoint_index + 1} 在障碍物/高代价区域内，'
            f'cost={blocked_cost}，'
            f'障碍物中心≈({obstacle_center[0]:.3f}, {obstacle_center[1]:.3f})，'
            f'跳到第 {next_index + 1} 个点')
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
                    f'前方障碍物已经到达最后一个固定路径点 {blocked_index + 1}')
                return None

            x, y, _ = self.waypoints[blocked_index]
            obstacle_center = self._find_obstacle_center((x, y)) or (x, y)
            next_index = self._find_escape_waypoint_index(
                blocked_index, obstacle_center)
            if next_index is None:
                self.get_logger().error(
                    f'前方障碍物靠近固定路径点 {blocked_index + 1}，'
                    f'cost={cost}，但后续 {self.max_skip_waypoints} 个点内没有离障碍物中心 '
                    f'{self.obstacle_escape_distance:.2f}m 外的安全点')
                return None

            self.get_logger().warn(
                f'前方障碍物靠近固定路径点 {blocked_index + 1}，'
                f'cost={cost}，closest={closest_index + 1}，goal={waypoint_index + 1}，'
                f'障碍物中心≈({obstacle_center[0]:.3f}, {obstacle_center[1]:.3f})，'
                f'提前跳到第 {next_index + 1} 个点')
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
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = self.initial_x
        msg.pose.pose.position.y = self.initial_y
        msg.pose.pose.orientation.z = math.sin(self.initial_yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.initial_yaw / 2.0)
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.068
        self.initial_pose_pub.publish(msg)
        self.get_logger().info(
            f'已发布 initialpose: ({self.initial_x:.2f}, {self.initial_y:.2f}, '
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

    def _goal_response_callback(self, future, waypoint_index):
        try:
            goal_handle = future.result()
        except Exception as exc:
            if self.pending_nav_goal_index == waypoint_index:
                self.pending_nav_goal_index = None
            self.get_logger().error(
                f'固定路径点 {waypoint_index + 1} 发送失败: {exc}')
            return

        if self.pending_nav_goal_index == waypoint_index:
            self.pending_nav_goal_index = None

        if waypoint_index < self.current_waypoint_index:
            self.get_logger().info(
                f'固定路径点 {waypoint_index + 1} 已被后续目标接管，忽略响应')
            return

        if not goal_handle.accepted:
            self.get_logger().error(
                f'固定路径点 {waypoint_index + 1} 被 Nav2 拒绝')
            return

        self.get_logger().info(
            f'固定路径点 {waypoint_index + 1}/{len(self.waypoints)} 已被 Nav2 接受')
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
                f'固定路径点 {waypoint_index + 1}/{len(self.waypoints)} '
                f'巡航中，distance_remaining={distance:.3f}',
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
                f'获取固定路径点 {waypoint_index + 1} 结果失败: {exc}')
            return

        if waypoint_index < self.current_waypoint_index:
            self.get_logger().info(
                f'固定路径点 {waypoint_index + 1} 的旧结果已忽略，status={result.status}')
            return

        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(
                f'固定路径点 {waypoint_index + 1} 结束，status={result.status}')
            if self.exit_on_result and rclpy.ok():
                rclpy.shutdown()
            return

        self.get_logger().warn(
            f'固定路径点 {waypoint_index + 1}/{len(self.waypoints)} 已到达')
        self.current_waypoint_index = waypoint_index + 1
        self._send_next_waypoint()


def main(args=None):
    rclpy.init(args=args)
    node = FixedWaypointDemo()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
