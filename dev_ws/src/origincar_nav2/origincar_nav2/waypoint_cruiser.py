#!/usr/bin/env python3
"""
waypoint_cruiser.py — 竞赛巡航节点 (v2: NavigateThroughPoses 平滑巡航版)

比赛流程:
  1. 启动后等待 /cruise_start 发令
  2. 批量发送 waypoints_prescan.yaml 全部航点 → 连续巡航，中间不停
  3. prescan走完 → 根据QR结果加载顺时针或逆时针文件
  4. 批量发送后续航点 → 连续巡航
  5. 到达 vlm_trigger_waypoint 时触发VLM拍照（不等结果，继续走）
  6. 走完全部航点 → 触发VLM推理 → 巡航结束

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
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Empty, Int32, String
from origincar_msg.msg import Sign
from ament_index_python.packages import get_package_share_directory
import math
import os
import yaml


class WaypointCruiser(Node):
    # 巡航状态
    WAITING_START = 'WAITING_START'
    PRESCAN = 'PRESCAN'
    WAITING_QR = 'WAITING_QR'
    POST_QR = 'POST_QR'
    DONE = 'DONE'

    def __init__(self):
        super().__init__('waypoint_cruiser')

        # ---- 参数声明 ----
        self.declare_parameter('cruise_mode', 'once')
        self.declare_parameter('waypoint_tolerance', 0.5)
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('start_topic', '/cruise_start')

        # 航点文件
        self.declare_parameter('prescan_file', '')
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

        # 读取参数
        self.cruise_mode = self.get_parameter('cruise_mode').value
        self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').value
        self.start_topic = self.get_parameter('start_topic').value

        self.prescan_file = self.get_parameter('prescan_file').value
        self.cw_file = self.get_parameter('cw_file').value
        self.ccw_file = self.get_parameter('ccw_file').value

        self.qr_topic = self.get_parameter('qr_topic').value
        self.qr_sign_cw = self.get_parameter('qr_sign_cw').value
        self.qr_sign_ccw = self.get_parameter('qr_sign_ccw').value
        self.qr_default = self.get_parameter('qr_default').value

        self.vlm_trigger_waypoint = self.get_parameter('vlm_trigger_waypoint').value
        self.vlm_trigger_value = self.get_parameter('vlm_trigger_value').value
        self.vlm_trigger_topic = self.get_parameter('vlm_trigger_topic').value

        # 解析航点文件路径
        pkg_share = get_package_share_directory('origincar_nav2')
        if not self.prescan_file:
            self.prescan_file = os.path.join(pkg_share, 'waypoints_prescan.yaml')
        if not self.cw_file:
            self.cw_file = os.path.join(pkg_share, 'waypoints_cw.yaml')
        if not self.ccw_file:
            self.ccw_file = os.path.join(pkg_share, 'waypoints_ccw.yaml')

        # ---- 状态变量 ----
        self._state = self.WAITING_START
        self.waypoints = []              # 当前航点列表 [(x, y, yaw), ...]
        self.last_qr_sign = None         # 最新QR方向 (3=顺时针, 4=逆时针)
        self.vlm_capture_triggered = False  # VLM拍照是否已触发

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
        self.get_logger().info(f'  发令话题: {self.start_topic}')
        self.get_logger().info(f'  prescan: {self.prescan_file}')
        self.get_logger().info(f'  顺时针:  {self.cw_file}')
        self.get_logger().info(f'  逆时针:  {self.ccw_file}')
        self.get_logger().info(f'  QR话题:  {self.qr_topic}')
        self.get_logger().info(f'  VLM触发: 第{self.vlm_trigger_waypoint}个航点')
        self.get_logger().info('  等待发令中...')
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
        """收到发令信号，批量发送prescan全部航点"""
        if self._state != self.WAITING_START:
            self.get_logger().warn(f'当前状态 {self._state}，忽略发令信号')
            return

        self.get_logger().info('收到发令信号！开始巡航！')
        self._state = self.PRESCAN

        # 加载prescan航点
        self.waypoints = self._load_waypoints(self.prescan_file)
        if not self.waypoints:
            self.get_logger().error('prescan航点为空，无法开始巡航！')
            return

        self.get_logger().info(
            f'已加载 prescan 航点: {len(self.waypoints)} 个点 → 批量发送，连续巡航！')
        self._send_all_goals()

    def _on_qr_result(self, msg: Sign):
        """QR节点扫描结果：记录方向，若正在等待则立即触发下一阶段"""
        sign_data = msg.sign_data
        if sign_data == self.qr_sign_cw:
            self.last_qr_sign = sign_data
            self.get_logger().info(f'QR方向已记录: 顺时针 (sign={sign_data})')
        elif sign_data == self.qr_sign_ccw:
            self.last_qr_sign = sign_data
            self.get_logger().info(f'QR方向已记录: 逆时针 (sign={sign_data})')
        else:
            self.get_logger().debug(f'忽略未知QR信号: sign={sign_data}')
            return

        # ★ 如果正在等QR，扫到后立刻进入下一个阶段 ★
        if self._state == self.WAITING_QR:
            self.get_logger().info('QR已扫到，立即进入巡航阶段！')
            self._do_switch_to_post_qr()

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
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            poses.append(pose)
        return poses

    def _send_all_goals(self):
        """将当前阶段全部航点打包为一个 NavigateThroughPoses 目标，一次性发送"""
        if self._state == self.DONE:
            return

        if not self.waypoints:
            self.get_logger().error('航点列表为空，无法发送目标！')
            return

        phase = 'PRESCAN' if self._state == self.PRESCAN else 'POST_QR'
        self.get_logger().info(
            f'[{phase}] 批量发送 {len(self.waypoints)} 个航点 → NavigateThroughPoses')

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = self._waypoints_to_poses(self.waypoints)

        send_goal_future = self.nav_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_callback)
        send_goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        """目标被Nav2接受后的回调"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('NavigateThroughPoses 目标被拒绝！')
            return

        phase = 'PRESCAN' if self._state == self.PRESCAN else 'POST_QR'
        self.get_logger().info(
            f'[{phase}] 全部 {len(self.waypoints)} 个航点已被 Nav2 接受，开始连续巡航...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        """全部航点完成后的回调"""
        result = future.result()
        status = result.status

        phase = 'PRESCAN' if self._state == self.PRESCAN else 'POST_QR'
        if status == 4:  # SUCCEEDED
            self.get_logger().info(f'✅ [{phase}] 全部航点到达！')
        else:
            self.get_logger().warn(f'⚠️ [{phase}] 巡航结束, status={status}')

        if self._state == self.PRESCAN:
            # prescan走完 → 根据QR方向加载后续航点
            self._switch_to_post_qr()
        elif self._state == self.POST_QR:
            # 全部走完 → 触发 VLM 推理
            self.get_logger().info('所有航点已完成，触发VLM推理！')
            self._trigger_vlm_infer()
            self._state = self.DONE

    def _feedback_callback(self, feedback_msg):
        """巡航过程中的反馈 — 用于VLM触发检测"""
        feedback = feedback_msg.feedback
        current_idx = feedback.current_waypoint_index  # 0-based

        # VLM拍照触发：在POST_QR阶段，首次到达指定航点序号时触发
        if self._state == self.POST_QR and not self.vlm_capture_triggered:
            # current_waypoint_index 是机器人正前往的航点索引 (0-based)
            # vlm_trigger_waypoint 是用户指定的航点序号 (1-based)
            if current_idx + 1 >= self.vlm_trigger_waypoint:
                self._trigger_vlm_capture()
                self.vlm_capture_triggered = True

    # ================================================================
    # 阶段切换
    # ================================================================

    def _switch_to_post_qr(self):
        """prescan走完，检查QR：已扫到→继续，没扫到→原地等待"""
        if self.last_qr_sign is not None:
            # QR 在 prescan 过程中已扫到，直接进入
            self._do_switch_to_post_qr()
        else:
            # 还没扫到，停在原地等 QR
            self._state = self.WAITING_QR
            self.get_logger().info('prescan已完成，原地等待QR扫描...')
            # ★ TTS 提示等待 ★
            tts_msg = String()
            tts_msg.data = '等待扫描二维码'
            self.tts_pub.publish(tts_msg)

    def _do_switch_to_post_qr(self):
        """实际执行QR方向判断并加载后续航点，批量发送"""
        direction = 'unknown'

        if self.last_qr_sign == self.qr_sign_cw:
            direction = 'cw'
        elif self.last_qr_sign == self.qr_sign_ccw:
            direction = 'ccw'
        else:
            direction = self.qr_default
            self.get_logger().warn(
                f'未收到QR方向信号，使用默认方向: {direction}')

        if direction == 'cw':
            next_file = self.cw_file
            self.get_logger().info('>>> QR方向: 顺时针 <<<')
            tts_text = '已选择顺时针方向'
        else:
            next_file = self.ccw_file
            self.get_logger().info('>>> QR方向: 逆时针 <<<')
            tts_text = '已选择逆时针方向'

        # ★ TTS语音播报方向决策 ★
        tts_msg = String()
        tts_msg.data = tts_text
        self.tts_pub.publish(tts_msg)
        self.get_logger().info(f'TTS播报: {tts_text}')

        self.waypoints = self._load_waypoints(next_file)
        if not self.waypoints:
            self.get_logger().error(f'无法加载航点文件: {next_file}，巡航终止！')
            self._state = self.DONE
            return

        self._state = self.POST_QR
        self.vlm_capture_triggered = False  # 重置VLM触发标记
        self.get_logger().info(
            f'已加载后续航点: {len(self.waypoints)} 个点 → 批量发送，连续巡航！')

        # 立即批量发送全部后续航点
        self._send_all_goals()

    # ================================================================
    # VLM 触发
    # ================================================================

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
