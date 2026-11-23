#!/usr/bin/env python3
"""
waypoint_cruiser.py — 竞赛巡航节点 (v2: NavigateThroughPoses 平滑巡航版)

比赛流程:
  1. 启动后等待QR方向信号
  2. /sign_switch == 3/4 后直接加载顺时针或逆时针文件
  3. 顺时针/逆时针都批量发送全部航点，由BT在规划前修正障碍内目标点
  4. 到达拍照触发点时触发VLM拍照 → 立即启动推理（不等巡航结束）
  5. 走完全部航点 → 恢复visual_control并结束巡航

与v1的区别:
  - v1: NavigateToPose 逐点发送 → 每点到达后停0.5s → 下一帧
  - v2: NavigateThroughPoses 批量发送 → TEB free_goal_vel=true 连续穿过航点

QR方向:
  /sign_switch.sign_data == 3 → 顺时针 → waypoints_cw.yaml
  /sign_switch.sign_data == 4 → 逆时针 → waypoints_ccw.yaml
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateThroughPoses
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Empty, Int32, String
from origincar_msg.msg import Sign
from ament_index_python.packages import get_package_share_directory
import math
import os
import yaml


class WaypointCruiser(Node):
    # 巡航状态
    WAITING_QR = 'WAITING_QR'
    POST_QR = 'POST_QR'
    DONE = 'DONE'

    def __init__(self):
        super().__init__('waypoint_cruiser')

        # ---- 参数声明 ----
        self.declare_parameter('cruise_mode', 'once')
        self.declare_parameter('waypoint_tolerance', 0.5)
        self.declare_parameter('start_topic', '/cruise_start')

        # 航点文件
        self.declare_parameter('cw_file', '')
        self.declare_parameter('ccw_file', '')

        # QR相关
        self.declare_parameter('qr_topic', '/sign_switch')
        self.declare_parameter('qr_sign_cw', 3)
        self.declare_parameter('qr_sign_ccw', 4)
        self.declare_parameter('qr_default', 'cw')

        # VLM相关
        self.declare_parameter('vlm_trigger_waypoint', 8)
        self.declare_parameter('vlm_trigger_value', -1)
        self.declare_parameter('vlm_trigger_topic', '/sign4return')
        self.declare_parameter('ccw_photo_x', 0.25)
        self.declare_parameter('ccw_photo_y', 4.06)
        self.declare_parameter('photo_waypoint_match_eps', 0.08)

        # 读取参数
        self.cruise_mode = self.get_parameter('cruise_mode').value
        self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').value
        self.start_topic = self.get_parameter('start_topic').value

        self.cw_file = self.get_parameter('cw_file').value
        self.ccw_file = self.get_parameter('ccw_file').value

        self.qr_topic = self.get_parameter('qr_topic').value
        self.qr_sign_cw = self.get_parameter('qr_sign_cw').value
        self.qr_sign_ccw = self.get_parameter('qr_sign_ccw').value
        self.qr_default = self.get_parameter('qr_default').value

        self.vlm_trigger_waypoint = self.get_parameter('vlm_trigger_waypoint').value
        self.vlm_trigger_value = self.get_parameter('vlm_trigger_value').value
        self.vlm_trigger_topic = self.get_parameter('vlm_trigger_topic').value
        self.ccw_photo_x = float(self.get_parameter('ccw_photo_x').value)
        self.ccw_photo_y = float(self.get_parameter('ccw_photo_y').value)
        self.photo_waypoint_match_eps = float(
            self.get_parameter('photo_waypoint_match_eps').value)

        # 解析航点文件路径
        pkg_share = get_package_share_directory('origincar_nav2')
        if not self.cw_file:
            self.cw_file = os.path.join(pkg_share, 'waypoints_cw.yaml')
        if not self.ccw_file:
            self.ccw_file = os.path.join(pkg_share, 'waypoints_ccw.yaml')

        # ---- 状态变量 ----
        self._state = self.WAITING_QR
        self.waypoints = []              # 当前航点列表 [(x, y, yaw), ...]
        self.last_qr_sign = None         # 最新QR方向 (3=顺时针, 4=逆时针)
        self.announced_qr_sign = None    # 已播报过的QR方向，避免重复播报
        self.vlm_capture_triggered = False  # VLM拍照是否已触发
        self.active_vlm_trigger_waypoint = self.vlm_trigger_waypoint
        self.feedback_schema_warned = False
        self.nav_goal_generation = 0
        self.active_goal_handle = None
        self.active_goal_generation = None
        self.current_direction = None

        # ---- Nav2 NavigateThroughPoses 动作客户端 ----
        # ★ v2 核心改动：使用 navigate_through_poses 而非 navigate_to_pose
        self.nav_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')

        # ---- 初始位姿发布者 ----
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 'initialpose', 10)

        # ---- VLM触发发布者 ----
        self.vlm_trigger_pub = self.create_publisher(
            Int32, self.vlm_trigger_topic, 10)

        # ---- TTS语音发布者 ----
        self.tts_pub = self.create_publisher(String, '/tts_text', 10)

        # ---- 订阅: 发令信号 ----
        self.create_subscription(
            Empty, self.start_topic, self._on_start_signal, 10)

        # ---- 订阅: QR方向结果 ----
        self.create_subscription(
            Sign, self.qr_topic, self._on_qr_result, 10)

        self.get_logger().info('=' * 60)
        self.get_logger().info('竞赛巡航节点已启动 (NavigateThroughPoses v2)')
        self.get_logger().info(f'  状态: {self._state}')
        self.get_logger().info(f'  手动兜底发令话题: {self.start_topic}')
        self.get_logger().info(f'  顺时针:  {self.cw_file}')
        self.get_logger().info(f'  逆时针:  {self.ccw_file}')
        self.get_logger().info(f'  QR话题:  {self.qr_topic}')
        self.get_logger().info(
            f'  VLM触发: 默认第{self.vlm_trigger_waypoint}个航点，'
            f'逆时针自动匹配({self.ccw_photo_x:.2f}, {self.ccw_photo_y:.2f})')
        self.get_logger().info('  等待QR方向信号中...')
        self.get_logger().info('=' * 60)

        # ---- 等待Nav2就绪 + 发布初始位姿 ----
        self.get_logger().info('等待 Nav2 动作服务器 (navigate_through_poses)...')
        self.nav_client.wait_for_server()
        self.get_logger().info('Nav2 已就绪！')
        self.publish_initial_pose()

    # ================================================================
    # 信号回调
    # ================================================================

    def _on_start_signal(self, msg):
        """收到手动发令信号，按当前QR方向或默认方向直接进入waypoint巡航。"""
        if self._state != self.WAITING_QR:
            self.get_logger().warn(f'当前状态 {self._state}，忽略发令信号')
            return

        if self.last_qr_sign is None:
            self.get_logger().warn(
                f'收到手动发令但尚未收到QR方向，使用默认方向: {self.qr_default}')
        else:
            self.get_logger().info('收到手动发令，按已记录QR方向进入巡航')
        self._do_switch_to_post_qr()

    def _on_qr_result(self, msg: Sign):
        """QR节点扫描结果：记录方向，立即切到后续巡航。"""
        sign_data = msg.sign_data
        if sign_data == self.qr_sign_cw:
            self.last_qr_sign = sign_data
            self.get_logger().warn(f'QR方向已记录: 顺时针 (sign={sign_data})')
        elif sign_data == self.qr_sign_ccw:
            self.last_qr_sign = sign_data
            self.get_logger().warn(f'QR方向已记录: 逆时针 (sign={sign_data})')
        else:
            self.get_logger().debug(f'忽略未知QR信号: sign={sign_data}')
            return

        self._announce_qr_direction(sign_data)

        if self._state == self.WAITING_QR:
            self.get_logger().warn('QR已扫到，立即进入顺/逆时针巡航阶段！')
            self._do_switch_to_post_qr()
        elif self._state == self.POST_QR:
            self.get_logger().debug('已在顺/逆时针巡航阶段，忽略重复QR结果')
        else:
            self.get_logger().debug(f'当前状态 {self._state}，忽略QR结果')

    # ================================================================
    # 航点加载
    # ================================================================

    def _load_waypoints(self, file_path: str) -> list:
        """从YAML文件加载航点，返回 [(x, y, yaw), ...]"""
        try:
            with open(file_path, 'r') as f:
                data = yaml.safe_load(f)
            wp_list = data.get('waypoints', [])
            result = [tuple(float(v) for v in w) for w in wp_list]
            self.get_logger().info(f'从文件加载 {len(result)} 个航点: {file_path}')
            return result
        except Exception as e:
            self.get_logger().error(f'加载航点文件失败 ({file_path}): {e}')
            return []

    def _find_waypoint_index(self, target_x: float, target_y: float):
        for index, waypoint in enumerate(self.waypoints):
            if (abs(waypoint[0] - target_x) <= self.photo_waypoint_match_eps and
                    abs(waypoint[1] - target_y) <= self.photo_waypoint_match_eps):
                return index
        return None

    def _select_vlm_trigger_waypoint(self, direction: str) -> int:
        if direction != 'ccw':
            return int(self.vlm_trigger_waypoint)

        photo_index = self._find_waypoint_index(self.ccw_photo_x, self.ccw_photo_y)
        if photo_index is None:
            self.get_logger().warn(
                '逆时针未找到配置的拍照点 '
                f'({self.ccw_photo_x:.2f}, {self.ccw_photo_y:.2f})，'
                f'使用默认第{self.vlm_trigger_waypoint}个航点触发')
            return int(self.vlm_trigger_waypoint)

        trigger_waypoint = photo_index + 1
        self.get_logger().warn(
            f'逆时针VLM拍照触发点: 第{trigger_waypoint}个航点 '
            f'({self.ccw_photo_x:.2f}, {self.ccw_photo_y:.2f})')
        return trigger_waypoint

    # ================================================================
    # 导航控制 — NavigateThroughPoses 批量发送
    # ================================================================

    def publish_initial_pose(self):
        """发布初始位姿 (原点)"""
        x, y, yaw = 0.0, 0.0, 0.0
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.068
        self.initial_pose_pub.publish(msg)
        self.get_logger().info(f'已发布初始位姿: ({x:.2f}, {y:.2f}, yaw={yaw:.2f})')

    def _waypoints_to_poses(self, waypoints: list) -> list:
        """将 [(x, y, yaw), ...] 转换为 [PoseStamped, ...]"""
        poses = []
        for x, y, yaw in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            # 置 0 让 Nav2 使用最新 TF，避免长时间巡航后按旧时间查 map->odom。
            pose.header.stamp = Time(sec=0, nanosec=0)
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            poses.append(pose)
        return poses

    def _send_all_goals(self):
        """将当前阶段全部航点打包为一个 NavigateThroughPoses 目标，一次性发送"""
        self._send_goal_batch(self.waypoints, self.POST_QR)

    def _send_goal_batch(self, waypoints: list, phase: str):
        """发送一个 NavigateThroughPoses 批次。"""
        if self._state == self.DONE:
            return

        if not waypoints:
            self.get_logger().error('航点列表为空，无法发送目标！')
            return

        batch_waypoints = list(waypoints)
        self.waypoints = batch_waypoints
        self.nav_goal_generation += 1
        generation = self.nav_goal_generation
        self.get_logger().info(
            f'[{phase}] 批量发送 {len(batch_waypoints)} 个航点 → NavigateThroughPoses')

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = self._waypoints_to_poses(batch_waypoints)

        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=lambda feedback_msg, generation=generation, phase=phase:
                self._feedback_callback(feedback_msg, generation, phase))
        send_goal_future.add_done_callback(
            lambda future, generation=generation, phase=phase, waypoint_count=len(batch_waypoints):
                self._goal_response_callback(future, generation, phase, waypoint_count))

    def _goal_response_callback(self, future, generation, phase, waypoint_count):
        """目标被Nav2接受后的回调"""
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f'NavigateThroughPoses 目标发送失败: {e}')
            return

        if not goal_handle.accepted:
            self.get_logger().error(f'[{phase}] NavigateThroughPoses 目标被拒绝！')
            self._state = self.DONE
            return

        if generation != self.nav_goal_generation:
            self.get_logger().info(f'[{phase}] 取消已过期目标 generation={generation}')
            goal_handle.cancel_goal_async()
            return

        self.active_goal_handle = goal_handle
        self.active_goal_generation = generation
        self.get_logger().info(
            f'[{phase}] 全部 {waypoint_count} 个航点已被 Nav2 接受，开始连续巡航...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future, generation=generation, phase=phase:
                self._result_callback(future, generation, phase))

    def _result_callback(self, future, generation, phase):
        """全部航点完成后的回调"""
        if generation != self.nav_goal_generation:
            self.get_logger().debug(f'忽略旧目标结果: phase={phase}, generation={generation}')
            return

        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f'[{phase}] 获取巡航结果失败: {e}')
            if self.active_goal_generation == generation:
                self.active_goal_handle = None
                self.active_goal_generation = None
            return

        if self.active_goal_generation == generation:
            self.active_goal_handle = None
            self.active_goal_generation = None

        status = result.status

        if status == 4:  # SUCCEEDED
            self.get_logger().info(f'✅ [{phase}] 全部航点到达！')
        else:
            self.get_logger().warn(f'⚠️ [{phase}] 巡航结束, status={status}')

        if phase == self.POST_QR and self._state == self.POST_QR:
            if status == 4:
                self._resume_visual_control()
            else:
                self.get_logger().warn('巡航未成功完成，保持visual_control挂起，等待人工处理')
            self.get_logger().info('所有航点已完成，巡航结束！')
            self._state = self.DONE

    def _feedback_callback(self, feedback_msg, generation, phase):
        """巡航过程中的反馈 — 用于VLM触发检测"""
        if generation != self.nav_goal_generation or phase != self.POST_QR:
            return

        feedback = feedback_msg.feedback
        current_wp_number = self._current_waypoint_number_from_feedback(feedback)
        if current_wp_number is None:
            return

        # VLM拍照+推理触发：在POST_QR阶段，首次到达指定航点序号时触发
        if self._state == self.POST_QR and not self.vlm_capture_triggered:
            # active_vlm_trigger_waypoint 是当前方向下的航点序号 (1-based)
            if current_wp_number >= self.active_vlm_trigger_waypoint:
                self._trigger_vlm_capture()
                self.vlm_capture_triggered = True
                # ★ 拍照后立即启动推理（不等巡航结束）
                self._trigger_vlm_infer()

    def _current_waypoint_number_from_feedback(self, feedback):
        """Return the current target waypoint number as 1-based.

        ROS2 Humble NavigateThroughPoses feedback does not expose
        current_waypoint_index, only number_of_poses_remaining.
        """
        if hasattr(feedback, 'current_waypoint_index'):
            return int(feedback.current_waypoint_index) + 1

        if hasattr(feedback, 'number_of_poses_remaining'):
            total = len(self.waypoints)
            if total <= 0:
                return None
            remaining = max(0, min(int(feedback.number_of_poses_remaining), total))
            return max(1, min(total - remaining + 1, total))

        if not self.feedback_schema_warned:
            self.feedback_schema_warned = True
            self.get_logger().warn(
                'NavigateThroughPoses feedback里没有航点进度字段，跳过VLM触发检测')
        return None

    def _do_switch_to_post_qr(self):
        """实际执行QR方向判断并加载后续航点，批量发送"""
        if self._state == self.POST_QR:
            self.get_logger().debug('已经处于后续巡航阶段，跳过重复切换')
            return
        if self._state == self.DONE:
            self.get_logger().warn('巡航已结束，忽略新的方向切换请求')
            return

        direction = 'unknown'
        sign_data = self.last_qr_sign

        if sign_data == self.qr_sign_cw:
            direction = 'cw'
        elif sign_data == self.qr_sign_ccw:
            direction = 'ccw'
        else:
            direction = self.qr_default
            self.get_logger().warn(
                f'未收到QR方向信号，使用默认方向: {direction}')

        if direction == 'cw':
            next_file = self.cw_file
            self.get_logger().warn('>>> QR方向: 顺时针 <<<')
            tts_text = '已选择顺时针方向'
        else:
            next_file = self.ccw_file
            self.get_logger().warn('>>> QR方向: 逆时针 <<<')
            tts_text = '已选择逆时针方向'

        # 如果QR是在等待阶段才扫到，回调里已经播报；默认方向仍在这里播报。
        if sign_data is None or self.announced_qr_sign != sign_data:
            self._publish_tts(tts_text)
            self.announced_qr_sign = sign_data

        self.waypoints = self._load_waypoints(next_file)
        if not self.waypoints:
            self.get_logger().error(f'无法加载航点文件: {next_file}，巡航终止！')
            self._state = self.DONE
            return

        self._state = self.POST_QR
        self.current_direction = direction
        self.active_vlm_trigger_waypoint = self._select_vlm_trigger_waypoint(direction)
        self.vlm_capture_triggered = False  # 重置VLM触发标记

        self.get_logger().warn(
            f'已加载后续航点: {len(self.waypoints)} 个点 → 批量发送，连续巡航！')
        self._send_all_goals()

    def _announce_qr_direction(self, sign_data):
        """扫描到有效QR方向后立即播报一次。"""
        if self.announced_qr_sign == sign_data:
            return

        if sign_data == self.qr_sign_cw:
            tts_text = '已选择顺时针方向'
        elif sign_data == self.qr_sign_ccw:
            tts_text = '已选择逆时针方向'
        else:
            return

        self._publish_tts(tts_text)
        self.announced_qr_sign = sign_data

    def _publish_tts(self, text):
        """发布TTS文本，并在日志中保留一条可查记录。"""
        tts_msg = String()
        tts_msg.data = text
        self.tts_pub.publish(tts_msg)
        self.get_logger().info(f'TTS播报: {text}')

    # ================================================================
    # VLM 触发
    # ================================================================

    def _resume_visual_control(self):
        """顺/逆时针巡航完成后恢复visual_control。"""
        trigger_msg = Int32()
        trigger_msg.data = 6  # 6 = 恢复visual_control
        self.vlm_trigger_pub.publish(trigger_msg)
        self.get_logger().info('已发布 /sign4return=6，恢复visual_control')

    def _trigger_vlm_capture(self):
        """触发VLM拍照（到达指定航点，只拍照不推理）"""
        trigger_msg = Int32()
        trigger_msg.data = self.vlm_trigger_value  # -1 = 拍照
        self.vlm_trigger_pub.publish(trigger_msg)
        self.get_logger().info(
            f'📸 触发VLM拍照! /sign4return={self.vlm_trigger_value}')

    def _trigger_vlm_infer(self):
        """触发VLM推理（终点，对已拍照片做推理）"""
        trigger_msg = Int32()
        trigger_msg.data = 0  # 0 = 推理
        self.vlm_trigger_pub.publish(trigger_msg)
        self.get_logger().info('🚀 触发VLM推理! /sign4return=0')


def main(args=None):
    rclpy.init(args=args)
    node = WaypointCruiser()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('巡航被用户中断')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
