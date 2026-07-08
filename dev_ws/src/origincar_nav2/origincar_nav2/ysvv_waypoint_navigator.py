#!/usr/bin/env python3
"""Send ysvv_waypoints.yaml to Nav2 in file order."""

import math
import os

import rclpy
import yaml
from ament_index_python.packages import PackageNotFoundError
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateThroughPoses
from rclpy.action import ActionClient
from rclpy.node import Node


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
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 'initialpose', 10)

        self.sent = False
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(0.2, self._tick)

        self.get_logger().info('=' * 60)
        self.get_logger().info('YSVV waypoint navigator started')
        self.get_logger().info(f'  waypoints: {self.waypoints_file}')
        self.get_logger().info(f'  waypoint count: {len(self.waypoints)}')
        self.get_logger().info(f'  action: {self.action_name}')
        self.get_logger().info(f'  start_delay: {self.start_delay:.1f}s')
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

    def _tick(self):
        if self.sent:
            return

        if not self.waypoints:
            self.get_logger().error('YSVV waypoint list is empty; nothing to navigate')
            self.sent = True
            return

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        if elapsed < self.start_delay:
            return

        if not self.nav_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().info(
                f'Waiting for Nav2 action server: {self.action_name}',
                throttle_duration_sec=2.0)
            return

        self.sent = True
        self.timer.cancel()

        if self.publish_initial_pose_enabled:
            self._publish_initial_pose()

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = self._waypoints_to_poses(self.waypoints)
        self.get_logger().warn(
            f'Sending {len(goal_msg.poses)} YSVV waypoints in YAML order')
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
            f'Published initialpose: ({self.initial_x:.2f}, {self.initial_y:.2f}, '
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
            self.get_logger().error(f'Failed to send YSVV goal: {exc}')
            return

        if not goal_handle.accepted:
            self.get_logger().error('YSVV goal was rejected by Nav2')
            return

        self.get_logger().info('YSVV goal accepted by Nav2')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        remaining = getattr(feedback, 'number_of_poses_remaining', None)
        if remaining is not None:
            self.get_logger().info(
                f'YSVV navigation running, remaining={remaining}',
                throttle_duration_sec=2.0)

    def _result_callback(self, future):
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f'Failed to get YSVV result: {exc}')
            return

        self.get_logger().warn(f'YSVV navigation finished, status={result.status}')
        if self.exit_on_result:
            raise SystemExit


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
