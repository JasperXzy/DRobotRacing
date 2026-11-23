#!/usr/bin/env python3
"""Keyboard teleop and map-frame waypoint recorder for the YSVV route."""

import math
import os
import select
import sys
import termios
import tty

import rclpy
from ament_index_python.packages import PackageNotFoundError
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformListener


HELP_TEXT = """
YSVV waypoint recorder
----------------------
w: forward        s: reverse
a: forward-left   d: forward-right
q/e: slow left/right
space or x: stop
p: save now       z: undo last waypoint
Ctrl-C: save and exit
record_only:=true disables cmd_vel publishing
"""


def _default_output_file():
    """Prefer the source package folder requested by the project."""
    try:
        share_dir = get_package_share_directory('origincar_nav2')
        ws_root = os.path.abspath(os.path.join(share_dir, '..', '..', '..', '..'))
        source_dir = os.path.join(ws_root, 'src', 'origincar_nav2', 'origincar_nav2')
        if os.path.isdir(source_dir):
            return os.path.join(source_dir, 'ysvv_waypoints.yaml')
    except PackageNotFoundError:
        pass

    return os.path.join(os.path.dirname(os.path.abspath(__file__)), 'ysvv_waypoints.yaml')


def _yaw_from_quaternion(quaternion):
    siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy_cosp = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _angle_delta(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


class RecordYsvvWaypoints(Node):
    def __init__(self):
        super().__init__('record_ysvv_waypoints')

        self.declare_parameter('output_file', _default_output_file())
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('reverse_speed', 0.12)
        self.declare_parameter('angular_speed', 0.35)
        self.declare_parameter('slow_turn_linear_speed', 0.10)
        self.declare_parameter('record_interval', 0.25)
        self.declare_parameter('min_distance', 0.08)
        self.declare_parameter('min_yaw_delta', 0.17)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('record_only', False)

        self.output_file = self.get_parameter('output_file').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.target_frame = self.get_parameter('target_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.reverse_speed = float(self.get_parameter('reverse_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.slow_turn_linear_speed = float(
            self.get_parameter('slow_turn_linear_speed').value)
        self.record_interval = float(self.get_parameter('record_interval').value)
        self.min_distance = float(self.get_parameter('min_distance').value)
        self.min_yaw_delta = float(self.get_parameter('min_yaw_delta').value)
        publish_rate = float(self.get_parameter('publish_rate').value)
        self.record_only = bool(self.get_parameter('record_only').value)

        self.cmd_pub = None
        if not self.record_only:
            self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.waypoints = []
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.last_record_time = None
        self.dirty = True
        self.terminal_settings = None
        self.shutdown_requested = False

        self._configure_terminal()

        timer_period = 1.0 / max(1.0, publish_rate)
        self.timer = self.create_timer(timer_period, self._tick)

        self.get_logger().info(HELP_TEXT)
        if self.record_only:
            self.get_logger().info('record_only: true; cmd_vel publishing disabled')
        else:
            self.get_logger().info(f'cmd_vel topic: {self.cmd_vel_topic}')
        self.get_logger().info(
            f'record frame: {self.target_frame} -> {self.base_frame}')
        self.get_logger().info(f'output file: {self.output_file}')

    def _configure_terminal(self):
        if not sys.stdin.isatty():
            self.get_logger().warn('stdin is not a TTY; keyboard control is disabled')
            return

        self.terminal_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

    def restore_terminal(self):
        if self.terminal_settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.terminal_settings)
            self.terminal_settings = None

    def _tick(self):
        self._handle_keyboard()
        if not self.record_only:
            self._publish_cmd()
        self._record_pose_if_needed()

        if self.shutdown_requested:
            self.get_logger().info('Saving waypoints and shutting down...')
            self.save()
            if not self.record_only:
                self._publish_stop()
            rclpy.shutdown()

    def _handle_keyboard(self):
        if self.terminal_settings is None:
            return

        while True:
            ready, _, _ = select.select([sys.stdin], [], [], 0.0)
            if not ready:
                return

            key = sys.stdin.read(1)
            if key == '\x03':
                self.shutdown_requested = True
                return
            if key == 'p':
                self.save()
                continue
            if key == 'z':
                self._undo_last_waypoint()
                continue

            if self.record_only:
                if key in ('w', 's', 'a', 'd', 'q', 'e', ' ', 'x'):
                    self.get_logger().warn(
                        'record_only=true; cmd_vel publishing is disabled',
                        throttle_duration_sec=2.0)
                continue

            if key == 'w':
                self._set_cmd(self.linear_speed, 0.0)
            elif key == 's':
                self._set_cmd(-self.reverse_speed, 0.0)
            elif key == 'a':
                self._set_cmd(self.linear_speed, self.angular_speed)
            elif key == 'd':
                self._set_cmd(self.linear_speed, -self.angular_speed)
            elif key == 'q':
                self._set_cmd(self.slow_turn_linear_speed, self.angular_speed)
            elif key == 'e':
                self._set_cmd(self.slow_turn_linear_speed, -self.angular_speed)
            elif key in (' ', 'x'):
                self._set_cmd(0.0, 0.0)

    def _set_cmd(self, linear, angular):
        self.current_linear = linear
        self.current_angular = angular
        self.get_logger().info(
            f'cmd_vel set: linear={linear:.2f}, angular={angular:.2f}')

    def _publish_cmd(self):
        if self.cmd_pub is None:
            return

        msg = Twist()
        msg.linear.x = self.current_linear
        msg.angular.z = self.current_angular
        self.cmd_pub.publish(msg)

    def _publish_stop(self):
        self.current_linear = 0.0
        self.current_angular = 0.0
        self._publish_cmd()

    def _record_pose_if_needed(self):
        now = self.get_clock().now()
        if self.last_record_time is not None:
            elapsed = (now - self.last_record_time).nanoseconds * 1e-9
            if elapsed < self.record_interval:
                return

        pose = self._lookup_current_pose()
        if pose is None:
            return

        if self._should_record(pose):
            self.waypoints.append(pose)
            self.dirty = True
            self.last_record_time = now
            self.get_logger().info(
                f'#{len(self.waypoints)} [{pose[0]:.3f}, {pose[1]:.3f}, {pose[2]:.3f}]')

    def _lookup_current_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.base_frame,
                Time())
        except TransformException as exc:
            self.get_logger().warn(
                f'waiting for TF {self.target_frame}->{self.base_frame}: {exc}',
                throttle_duration_sec=2.0)
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = _yaw_from_quaternion(rotation)
        return (round(translation.x, 3), round(translation.y, 3), round(yaw, 3))

    def _should_record(self, pose):
        if not self.waypoints:
            return True

        last_x, last_y, last_yaw = self.waypoints[-1]
        distance = math.hypot(pose[0] - last_x, pose[1] - last_y)
        yaw_delta = abs(_angle_delta(pose[2], last_yaw))
        return distance >= self.min_distance or yaw_delta >= self.min_yaw_delta

    def _undo_last_waypoint(self):
        if not self.waypoints:
            self.get_logger().warn('No waypoint to undo')
            return

        removed = self.waypoints.pop()
        self.dirty = True
        self.get_logger().warn(
            f'Removed waypoint #{len(self.waypoints) + 1}: '
            f'[{removed[0]:.3f}, {removed[1]:.3f}, {removed[2]:.3f}]')

    def save(self):
        if not self.dirty:
            return

        output_file = os.path.abspath(os.path.expanduser(self.output_file))
        os.makedirs(os.path.dirname(output_file), exist_ok=True)

        with open(output_file, 'w') as stream:
            if not self.waypoints:
                stream.write('waypoints: []\n')
            else:
                stream.write('waypoints:\n')
                for x, y, yaw in self.waypoints:
                    stream.write(f'- [{x:.3f}, {y:.3f}, {yaw:.3f}]\n')

        self.dirty = False
        self.get_logger().info(
            f'Saved {len(self.waypoints)} waypoints to {output_file}')


def main(args=None):
    rclpy.init(args=args)
    node = RecordYsvvWaypoints()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node._publish_stop()
        node.save()
        node.restore_terminal()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
