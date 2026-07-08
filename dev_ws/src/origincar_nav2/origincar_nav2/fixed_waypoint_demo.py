#!/usr/bin/env python3
"""Send a fixed waypoint YAML to Nav2 NavigateThroughPoses for planner testing."""

import math
import os

import rclpy
import yaml
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import ComputePathThroughPoses, NavigateThroughPoses
from rclpy.action import ActionClient
from rclpy.node import Node


class FixedWaypointDemo(Node):
    def __init__(self):
        super().__init__('fixed_waypoint_demo')

        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('action_name', 'navigate_through_poses')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('start_delay', 0.5)
        self.declare_parameter('publish_initial_pose', True)
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw', 0.0)
        self.declare_parameter('exit_on_result', False)

        self.waypoints_file = self.get_parameter('waypoints_file').value
        self.action_name = self.get_parameter('action_name').value
        self.frame_id = self.get_parameter('frame_id').value
        self.start_delay = float(self.get_parameter('start_delay').value)
        self.publish_initial_pose_enabled = self.get_parameter('publish_initial_pose').value
        self.initial_x = float(self.get_parameter('initial_x').value)
        self.initial_y = float(self.get_parameter('initial_y').value)
        self.initial_yaw = float(self.get_parameter('initial_yaw').value)
        self.exit_on_result = self.get_parameter('exit_on_result').value

        self.waypoints = self._load_waypoints(self.waypoints_file)
        self.nav_client = ActionClient(self, NavigateThroughPoses, self.action_name)
        # 全局规划器 action client：确保 planner_server 已就绪再发路径点
        self.planner_client = ActionClient(self, ComputePathThroughPoses, 'compute_path_through_poses')
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)

        # 状态机: INIT -> INITIAL_POSE -> PLANNER_WAIT -> SENT
        self._state = 'INIT'
        self._state_start_time = self.get_clock().now()
        self.timer = self.create_timer(0.2, self._tick)

        self.get_logger().info('=' * 60)
        self.get_logger().info('固定路径 Nav2 demo 已启动')
        self.get_logger().info(f'  waypoints: {self.waypoints_file}')
        self.get_logger().info(f'  waypoint count: {len(self.waypoints)}')
        self.get_logger().info(f'  action: {self.action_name}')
        self.get_logger().info(f'  start_delay: {self.start_delay:.1f}s')
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
                    parts.append('planner_server (compute_path_through_poses)')
                self.get_logger().info(
                    f'等待: {", ".join(parts)}', throttle_duration_sec=2.0)
                return

            self._state = 'SENT'
            self.timer.cancel()

            goal_msg = NavigateThroughPoses.Goal()
            goal_msg.poses = self._waypoints_to_poses(self.waypoints)
            self.get_logger().warn(
                f'全局 & 局部规划器已就绪，发送固定路径: {len(goal_msg.poses)} 个点')
            future = self.nav_client.send_goal_async(
                goal_msg,
                feedback_callback=self._feedback_callback)
            future.add_done_callback(self._goal_response_callback)

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

    def _waypoints_to_poses(self, waypoints):
        poses = []
        for x, y, yaw in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = self.frame_id
            pose.header.stamp = Time(sec=0, nanosec=0)
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            poses.append(pose)
        return poses

    def _goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f'固定路径目标发送失败: {exc}')
            return

        if not goal_handle.accepted:
            self.get_logger().error('固定路径目标被 Nav2 拒绝')
            return

        self.get_logger().info('固定路径目标已被 Nav2 接受')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        remaining = getattr(feedback, 'number_of_poses_remaining', None)
        if remaining is not None:
            self.get_logger().info(
                f'固定路径巡航中，remaining={remaining}', throttle_duration_sec=2.0)

    def _result_callback(self, future):
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f'获取固定路径结果失败: {exc}')
            return

        self.get_logger().warn(f'固定路径巡航结束，status={result.status}')
        if self.exit_on_result:
            raise SystemExit


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
