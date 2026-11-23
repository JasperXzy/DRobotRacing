#include "origincar_system/visual_lidar_path_follower.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

// 扫码后到进入waypoint_channel或达到配置轮数上限前，PENDING始终表示
// “倒车机制已武装”。qr_reverse_max_count为0时不武装、完全禁用倒车。
// 新一轮倒车由前方命中或过渡入口X超限的一次性请求触发。两种触发模式
// 使用完全相同的正常退出条件：既达到本轮锁定最小编码器累计里程，又确认
// 前方清空。
// 因此前方仍有障碍时会继续倒，入口X强制轮在前方本来就清空时也必须倒满
// 最小距离。本轮距离由倒车启动map.x线性计算，启动后不随车辆位置变化；
// 是否到达该距离只累计/chassis/odom实测纵向轮速为负时的逐帧位移增量；
// 正向惯性阶段持续发布负速度但不计里程，也不回退到map定位净位移。
// 后方非墙障碍、map.y和map.x是同级安全硬保护，可在最小距离完成前强制
// 结束；单轮最大编码器里程是两种模式共享的绝对上限。实际倒车退出后进入非阻塞
// POST_STOP状态，持续发布零速度；停车结束后才置位重规划并重新等待新雷达帧。
// 每次触发先进入BACKING_ARMED并计数；若首条负速度前已命中硬保护，则以
// 0m里程退出，但仍统一进入POST_STOP和倒车后重规划，生成对应03(n)。
bool VisualLidarPathFollower::apply_qr_reverse_maneuver(
  const VisualLidarPathFollower::RobotPose & robot_pose,
  geometry_msgs::msg::Twist & cmd)
{
  if (qr_escape_state_ == QrEscapeState::IDLE) {
    return false;
  }

  // 正常控制流程会在进入本函数前处理这两个状态。保留这里的零速兜底，
  // 防止未来调用顺序调整后意外绕过倒车前制动门。
  if (qr_escape_state_ == QrEscapeState::PRE_REVERSE_BRAKE ||
    qr_escape_state_ == QrEscapeState::WAITING_FOR_PRE_REVERSE_SCAN)
  {
    cmd = geometry_msgs::msg::Twist();
    cmd.angular.z = qr_reverse_hold_angular_command_;
    return true;
  }

  if (qr_escape_state_ == QrEscapeState::POST_STOP) {
    cmd = geometry_msgs::msg::Twist();
    cmd.angular.z = qr_reverse_hold_angular_command_;
    if (stop_brake_sequence_active_) {
      // 倒车结束时统一制动会自动使用正向脉冲。完整制动结束前不得重建
      // 下一条路线，也不得把下一阶段的正向命令覆盖到制动序列上。
      return true;
    }
    if (qr_reverse_post_stop_started_at_ ==
      std::chrono::steady_clock::time_point{})
    {
      // 停车t从反向制动及最终1次零速终止帧全部完成后才开始计算；
      // 不再把制动时间吞进用户配置的完整停车时长。
      qr_reverse_post_stop_started_at_ = std::chrono::steady_clock::now();
      if (qr_reverse_negative_command_published_) {
        RCLCPP_INFO(
          get_logger(),
          "二维码倒车反相制动及零速终止帧已完成，开始完整停车%.3fs。",
          qr_reverse_post_stop_seconds_);
      } else {
        RCLCPP_INFO(
          get_logger(),
          "强制倒车状态在首条负速度前被安全硬保护解除；"
          "未执行反相制动，保持零速度并开始完整停车%.3fs。",
          qr_reverse_post_stop_seconds_);
      }
    }
    const double stopped_seconds = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - qr_reverse_post_stop_started_at_).count();
    if (stopped_seconds + 1.0e-9 < qr_reverse_post_stop_seconds_) {
      return true;
    }

    qr_escape_state_ = QrEscapeState::WAITING_FOR_POST_STOP_SCAN;
    channel_entry_route_rebuild_pending_ = true;
    channel_entry_route_rebuild_after_reverse_ = true;
    // 停车结束后会先重规划并再保持一周期零速。之后第一次
    // 正向起步使用事件专用加速度，不使用全局加速度。
    arm_event_start_acceleration(EventStartContext::POST_REVERSE_RESUME);
    // 停车期间到达的雷达帧也不允许决定下一动作。以停车结束时为新边界，
    // 重规划完成后的正向动态避障必须再等一帧严格更新的雷达数据。
    channel_lidar_obstacle_active_ = false;
    channel_lidar_logged_direction_ = 0;
    channel_lidar_avoid_direction_ = 0;
    channel_lidar_route_side_action_active_ = false;
    channel_lidar_route_reverse_phase_active_ = false;
    channel_lidar_route_toward_direction_ = 0;
    channel_lidar_route_trigger_obstacle_valid_ = false;
    channel_lidar_locked_target_valid_ = false;
    channel_lidar_locked_target_is_permanent_ = false;
    channel_lidar_locked_target_is_task1_clearance_fallback_ = false;
    channel_lidar_waiting_for_post_reverse_scan_ = true;
    channel_lidar_post_reverse_scan_sequence_ = scan_sequence_;
    channel_lidar_post_reverse_time_nanoseconds_ = now().nanoseconds();
    qr_reverse_negative_command_published_ = false;
    std::cout << "-------------------------------倒车后停车结束"
                 "-------------------------------" << std::endl;
    std::cout << std::fixed << std::setprecision(3)
              << "已停车" << stopped_seconds
              << "s；开始以当前base_link重规划，并等待停车后的新雷达帧"
                 "重新判断避障方向" << std::endl;
    return false;
  }

  const bool waiting_for_post_stop_scan =
    qr_escape_state_ == QrEscapeState::WAITING_FOR_POST_STOP_SCAN;
  if (waiting_for_post_stop_scan) {
    cmd = geometry_msgs::msg::Twist();
    cmd.angular.z = qr_reverse_hold_angular_command_;
    if (!latest_scan_ || scan_sequence_ <= channel_lidar_post_reverse_scan_sequence_) {
      return true;
    }
    const std::int64_t scan_stamp_nanoseconds =
      rclcpp::Time(latest_scan_->header.stamp, get_clock()->get_clock_type()).nanoseconds();
    if (scan_stamp_nanoseconds != 0 &&
      scan_stamp_nanoseconds <= channel_lidar_post_reverse_time_nanoseconds_)
    {
      return true;
    }
  }

  RobotPose scan_to_base;
  RobotPose scan_to_map;
  if (!prepare_lidar_scan(scan_to_base, scan_to_map)) {
    const bool reverse_is_active =
      qr_escape_state_ == QrEscapeState::BACKING_RIGHT;
    const bool reverse_is_armed =
      qr_escape_state_ == QrEscapeState::BACKING_ARMED;
    const bool forced_reverse_is_pending =
      qr_escape_state_ == QrEscapeState::PENDING &&
      qr_reverse_x_force_pending_;
    const bool must_hold_stop = reverse_is_active || reverse_is_armed ||
      waiting_for_post_stop_scan || forced_reverse_is_pending;
    if (!qr_reverse_waiting_for_inputs_) {
      if (reverse_is_active) {
        RCLCPP_WARN(
          get_logger(),
          "过渡阶段正在倒车但雷达、物理地图或采样时刻TF暂不可用，保持停车。");
      } else if (reverse_is_armed) {
        RCLCPP_WARN(
          get_logger(),
          "强制倒车状态已经进入，但雷达、物理地图或采样时刻TF暂不可用；"
          "继续保持零速度，禁止在安全检查前发送负速度。");
      } else if (waiting_for_post_stop_scan) {
        RCLCPP_WARN(
          get_logger(),
          "倒车后停车已结束，但停车后的新雷达帧、物理地图或采样时刻TF"
          "暂不可用，继续保持停车，不执行下一动作。");
      } else if (forced_reverse_is_pending) {
        RCLCPP_WARN(
          get_logger(),
          "二维码强制倒车已进入待启动状态，但新雷达帧、物理地图或采样时刻TF"
          "暂不可用，继续保持停车，禁止恢复正向巡线。");
      } else {
        RCLCPP_WARN(
          get_logger(),
          "过渡阶段倒车机制的雷达输入暂不可用：本周期不触发倒车，继续正向路线跟踪，不停车。");
      }
      qr_reverse_waiting_for_inputs_ = true;
    }
    if (must_hold_stop) {
      cmd = geometry_msgs::msg::Twist();
      cmd.angular.z = qr_reverse_hold_angular_command_;
      return true;
    }
    return false;
  }
  qr_reverse_waiting_for_inputs_ = false;

  if (waiting_for_post_stop_scan) {
    qr_escape_state_ = qr_reverse_rearm_after_post_stop_ ?
      QrEscapeState::PENDING : QrEscapeState::IDLE;
    qr_reverse_rearm_after_post_stop_ = false;
    if (qr_escape_state_ == QrEscapeState::IDLE) {
      return false;
    }
  }

  const auto front_obstacle = find_lidar_obstacle(
    0.0, qr_reverse_front_obstacle_distance_, qr_reverse_front_fov_deg_,
    qr_reverse_front_corridor_half_width_, scan_to_base, scan_to_map,
    false, false, true, false);
  const auto rear_obstacle = find_lidar_obstacle(
    M_PI, qr_reverse_rear_obstacle_distance_, qr_reverse_rear_fov_deg_,
    qr_reverse_rear_corridor_half_width_, scan_to_base, scan_to_map,
    false, false, false, true);
  const auto odom_checked_at = std::chrono::steady_clock::now();
  const double wheel_odom_age_seconds = std::chrono::duration<double>(
    odom_checked_at - latest_chassis_odom_received_at_).count();
  const bool wheel_odom_fresh = latest_chassis_odom_position_valid_ &&
    std::isfinite(wheel_odom_age_seconds) && wheel_odom_age_seconds >= 0.0 &&
    wheel_odom_age_seconds <= qr_reverse_odom_timeout_seconds_;

  // 当前底盘在负线速度下使用正angular.z向右后倒。
  // 转向量按当前map.x在配置起止点之间线性插值，区间外夹紧。
  // 后方命中只作为硬保护，绝不再用来选择满舵继续倒车。
  const auto reverse_angular_command = [&]() {
      const double ramp_ratio = std::clamp(
        (robot_pose.x - qr_reverse_angular_ramp_start_x_) /
        (qr_reverse_angular_ramp_end_x_ - qr_reverse_angular_ramp_start_x_),
        0.0, 1.0);
      return qr_reverse_angular_speed_ +
             ramp_ratio * (qr_reverse_angular_speed_max_ - qr_reverse_angular_speed_);
    };
  const auto reverse_min_distance_for_start_x = [&](double start_map_x) {
      const double ramp_ratio = std::clamp(
        (start_map_x - qr_reverse_min_distance_ramp_start_x_) /
        (qr_reverse_min_distance_ramp_end_x_ -
        qr_reverse_min_distance_ramp_start_x_),
        0.0, 1.0);
      return qr_reverse_min_distance_ +
             ramp_ratio *
             (qr_reverse_min_distance_max_ - qr_reverse_min_distance_);
    };
  const auto set_reverse_command = [&]() {
      cmd.linear.x = -qr_reverse_linear_speed_;
      cmd.angular.z = reverse_angular_command();
      qr_reverse_hold_angular_command_ = cmd.angular.z;
    };
  const bool y_limit_reached = robot_pose.y <= qr_reverse_y_threshold_;
  const bool x_limit_reached = robot_pose.x >= qr_reverse_x_limit_;
  const bool rear_protection_reached = rear_obstacle.found;

  if (qr_escape_state_ == QrEscapeState::PENDING) {
    const bool qr_x_force_requested = qr_reverse_x_force_pending_;
    const bool reverse_triggered = front_obstacle.found || qr_x_force_requested;
    const double requested_min_distance =
      reverse_min_distance_for_start_x(robot_pose.x);
    std::ostringstream trigger_reason_stream;
    trigger_reason_stream << std::fixed << std::setprecision(3);
    if (qr_x_force_requested) {
      trigger_reason_stream <<
        (qr_reverse_force_source_label_.empty() ?
        "Task1→Task2过渡入口" : qr_reverse_force_source_label_)
                            << "map.x=" << qr_reverse_force_source_x_ << ">"
                            << qr_reverse_force_qr_x_threshold_
                            << "，强制触发";
      if (front_obstacle.found) {
        trigger_reason_stream << "，同时前方有障碍";
      }
    } else {
      trigger_reason_stream << "前方有障碍";
    }
    const std::string trigger_reason = trigger_reason_stream.str();
    if (reverse_triggered) {
      // 先正式进入强制倒车状态并计入轮次，再在该状态内部检查后方及XY
      // 硬保护。进入状态不等于发送负速度：安全不允许时以0m里程退出，
      // 但仍统一经过POST_STOP、倒车后重规划及03_Final_frozen_route(n)。
      ++channel_completed_reverse_count_;
      qr_escape_state_ = QrEscapeState::BACKING_ARMED;
      qr_reverse_negative_command_published_ = false;
      channel_lidar_obstacle_active_ = false;
      channel_lidar_logged_direction_ = 0;
      channel_lidar_avoid_direction_ = 0;
      channel_lidar_route_side_action_active_ = false;
      channel_lidar_route_reverse_phase_active_ = false;
      channel_lidar_route_toward_direction_ = 0;
      channel_lidar_route_trigger_obstacle_valid_ = false;
      channel_lidar_locked_target_valid_ = false;
      channel_lidar_locked_target_is_permanent_ = false;
      channel_lidar_locked_target_is_task1_clearance_fallback_ = false;
      channel_lidar_waiting_for_post_reverse_scan_ = false;
      qr_reverse_rearm_after_post_stop_ = false;
      qr_reverse_active_trigger_reason_ = trigger_reason;
      qr_reverse_active_x_force_ = qr_x_force_requested;
      qr_reverse_active_start_map_x_ = robot_pose.x;
      qr_reverse_active_min_distance_ = requested_min_distance;
      if (qr_x_force_requested) {
        qr_reverse_x_force_pending_ = false;
      }
      qr_reverse_wheel_odom_distance_ = 0.0;
      qr_reverse_last_odom_position_valid_ = false;
      qr_reverse_motion_confirmed_ = false;
      qr_reverse_odom_fault_ = false;
      qr_reverse_waiting_for_odom_ = false;
      qr_reverse_hold_angular_command_ = reverse_angular_command();
      std::cout << "-------------------------------过渡阶段强制倒车状态进入"
                   "-------------------------------" << std::endl;
      std::cout << std::fixed << std::setprecision(3)
                << "当前map.x=" << robot_pose.x
                << "m；预定倒车转向量angular.z="
                << qr_reverse_hold_angular_command_
                << "（x=" << qr_reverse_angular_ramp_start_x_ << "时为"
                << qr_reverse_angular_speed_ << "，x="
                << qr_reverse_angular_ramp_end_x_ << "时为"
                << qr_reverse_angular_speed_max_ << "）"
                << "；本轮锁定最小编码器累计里程="
                << qr_reverse_active_min_distance_ << "m"
                << "（启动x=" << qr_reverse_active_start_map_x_
                << "，x=" << qr_reverse_min_distance_ramp_start_x_ << "时"
                << qr_reverse_min_distance_ << "m，x="
                << qr_reverse_min_distance_ramp_end_x_ << "时"
                << qr_reverse_min_distance_max_ << "m）；"
                << "本轮编号=" << channel_completed_reverse_count_
                << "；接下来先检查后方障碍及XY硬保护，安全通过后才发送负速度。"
                << std::endl;
      RCLCPP_WARN(
        get_logger(),
        "已进入过渡阶段强制倒车第%zu轮：触发原因=%s；"
        "尚未发送负速度，先在倒车状态内部检查后方障碍、map.x及map.y硬保护。",
        channel_completed_reverse_count_,
        trigger_reason.c_str());
    }
    else if (debug_control_logs_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 500,
        "debug: 过渡阶段条件倒车机制保持武装，前障碍=%s，后障碍=%s，"
        "map.y=%.3f（仅作退出保护%.3f）。",
        front_obstacle.found ? "是" : "否",
        rear_obstacle.found ? "是" : "否",
        robot_pose.y, qr_reverse_y_threshold_);
    }
    if (!reverse_triggered) {
      return false;
    }
  }

  if (qr_escape_state_ == QrEscapeState::BACKING_ARMED) {
    // 安全检查发生在已经正式进入倒车状态之后，但必须发生在首条负速度
    // 之前。命中时不允许“试倒一帧”，直接走下方统一退出流程。
    const bool pre_command_hard_protection =
      rear_protection_reached || y_limit_reached || x_limit_reached;
    if (!pre_command_hard_protection && !wheel_odom_fresh) {
      cmd = geometry_msgs::msg::Twist();
      cmd.angular.z = qr_reverse_hold_angular_command_;
      if (!qr_reverse_waiting_for_odom_) {
        RCLCPP_WARN(
          get_logger(),
          "强制倒车状态已进入，但底盘轮式里程计%s无新鲜数据；"
          "保持零速度，数据恢复后再发送首条负速度。",
          logic3_odom_topic_.c_str());
        qr_reverse_waiting_for_odom_ = true;
      }
      return true;
    }
    qr_reverse_waiting_for_odom_ = false;
    if (!pre_command_hard_protection) {
      qr_escape_state_ = QrEscapeState::BACKING_RIGHT;
      qr_reverse_negative_command_published_ = true;
      qr_reverse_last_odom_position_ = latest_chassis_odom_position_;
      qr_reverse_last_odom_position_valid_ = true;
      set_reverse_command();
      std::cout << "-------------------------------过渡阶段条件倒车启动"
                   "-------------------------------" << std::endl;
      std::cout << std::fixed << std::setprecision(3)
                << "当前map.x=" << robot_pose.x
                << "m；倒车转向量angular.z=" << cmd.angular.z
                << "；本轮锁定最小编码器累计里程="
                << qr_reverse_active_min_distance_ << "m；"
                << "后方及XY硬保护已检查清空，开始发送负速度。" << std::endl;
      RCLCPP_WARN(
        get_logger(),
        "开始过渡阶段条件倒车第%zu轮：触发原因=%s；前方障碍=%s，"
        "map.y=%.3f（仅作退出保护%.3f），后方保护区=清空；"
        "目标编码器累计里程=%.2fm，单轮最大里程=%.2fm；"
        "输出 linear.x=%.2f, angular.z=%.2f。",
        channel_completed_reverse_count_,
        qr_reverse_active_trigger_reason_.c_str(),
        front_obstacle.found ? "是" : "否",
        robot_pose.y, qr_reverse_y_threshold_,
        qr_reverse_active_min_distance_, qr_reverse_max_distance_,
        cmd.linear.x, cmd.angular.z);
      return true;
    }
  }

  const bool front_clear = !front_obstacle.found;
  if (qr_reverse_negative_command_published_ && !wheel_odom_fresh) {
    qr_reverse_odom_fault_ = true;
  }
  const bool odom_protection_reached =
    qr_reverse_negative_command_published_ && qr_reverse_odom_fault_;
  const double reverse_distance = qr_reverse_wheel_odom_distance_;
  const bool min_distance_reached =
    qr_reverse_motion_confirmed_ &&
    reverse_distance + 1.0e-6 >= qr_reverse_active_min_distance_;
  const bool max_distance_reached =
    qr_reverse_motion_confirmed_ &&
    reverse_distance + 1.0e-6 >= qr_reverse_max_distance_;
  const bool hard_boundary_reached =
    rear_protection_reached || y_limit_reached || x_limit_reached ||
    odom_protection_reached;
  const bool normal_exit_reached =
    qr_reverse_motion_confirmed_ && front_clear && min_distance_reached;
  if (hard_boundary_reached || max_distance_reached ||
    normal_exit_reached)
  {
    const std::string completed_trigger_reason =
      qr_reverse_active_trigger_reason_.empty() ?
      "前方有障碍" : qr_reverse_active_trigger_reason_;
    const bool completed_x_force_reverse = qr_reverse_active_x_force_;
    const bool completed_reverse_motion_confirmed = qr_reverse_motion_confirmed_;
    // 本轮已在进入BACKING_ARMED时计数；这里只统一完成状态，不重复加1。
    const bool reverse_count_limit_reached =
      qr_reverse_max_count_ <= 0 ||
      channel_completed_reverse_count_ >=
      static_cast<std::size_t>(qr_reverse_max_count_);
    qr_escape_state_ = QrEscapeState::POST_STOP;
    qr_reverse_rearm_after_post_stop_ = !reverse_count_limit_reached;
    // 真正的停车计时必须等统一反向制动和最后1次零速终止帧结束后开始。
    qr_reverse_post_stop_started_at_ = std::chrono::steady_clock::time_point{};
    qr_reverse_last_odom_position_valid_ = false;
    qr_reverse_motion_confirmed_ = false;
    qr_reverse_odom_fault_ = false;
    qr_reverse_waiting_for_odom_ = false;
    qr_reverse_active_x_force_ = false;
    // 本周期立即清除旧避障方向，但重规划待办和新雷达帧边界必须等停车结束
    // 才建立，确保停车期间不会提前计算下一动作。
    channel_lidar_obstacle_active_ = false;
    channel_lidar_logged_direction_ = 0;
    channel_lidar_avoid_direction_ = 0;
    channel_lidar_route_side_action_active_ = false;
    channel_lidar_route_reverse_phase_active_ = false;
    channel_lidar_route_toward_direction_ = 0;
    channel_lidar_route_trigger_obstacle_valid_ = false;
    channel_lidar_locked_target_valid_ = false;
    channel_lidar_locked_target_is_permanent_ = false;
    channel_lidar_locked_target_is_task1_clearance_fallback_ = false;
    channel_lidar_waiting_for_post_reverse_scan_ = false;
    std::string exit_reason;
    if (hard_boundary_reached) {
      std::vector<std::string> hard_protection_reasons;
      if (rear_protection_reached) {
        std::ostringstream rear_reason;
        rear_reason << std::fixed << std::setprecision(3)
                    << "后方非墙障碍距离=" << rear_obstacle.range
                    << "m<=" << qr_reverse_rear_obstacle_distance_ << "m";
        hard_protection_reasons.push_back(rear_reason.str());
      }
      if (y_limit_reached) {
        hard_protection_reasons.emplace_back("map.y后退硬边界");
      }
      if (x_limit_reached) {
        hard_protection_reasons.emplace_back("map.x后退硬边界");
      }
      if (odom_protection_reached) {
        hard_protection_reasons.emplace_back("底盘轮式里程计无效或超时");
      }
      std::ostringstream exit_reason_stream;
      exit_reason_stream << "触发";
      for (std::size_t i = 0; i < hard_protection_reasons.size(); ++i) {
        if (i > 0) {
          exit_reason_stream << "、";
        }
        exit_reason_stream << hard_protection_reasons[i];
      }
      exit_reason_stream << "安全硬保护";
      exit_reason = exit_reason_stream.str();
    } else if (max_distance_reached) {
      exit_reason = "达到单轮最大编码器里程";
    } else {
      exit_reason = "前方清空且已达到最小编码器累计里程";
    }
    const std::string next_action = reverse_count_limit_reached ?
      "本次过渡已完成允许的" + std::to_string(qr_reverse_max_count_) +
      "轮条件倒车，停车结束后重规划并继续行驶" :
      "停车结束后重规划，仍允许前方障碍再次触发条件倒车";
    std::ostringstream terminal_message;
    terminal_message << std::fixed << std::setprecision(3)
                     << "触发原因=" << completed_trigger_reason
                     << "；退出原因=" << exit_reason
                     << "；负速度已下发="
                     << (qr_reverse_negative_command_published_ ? "是" : "否")
                     << "；编码器累计里程=" << reverse_distance << "/"
                     << qr_reverse_active_min_distance_ << "m；启动map.x="
                     << qr_reverse_active_start_map_x_
                     << "m；最小距离线性参数[x="
                     << qr_reverse_min_distance_ramp_start_x_ << "时"
                     << qr_reverse_min_distance_ << "m，x="
                     << qr_reverse_min_distance_ramp_end_x_ << "时"
                     << qr_reverse_min_distance_max_ << "m]；单轮最大编码器里程="
                     << qr_reverse_max_distance_ << "m；倒车后停车时间="
                     << qr_reverse_post_stop_seconds_ << "s；后续=" << next_action;
    std::cout << "-------------------------------过渡阶段条件倒车退出"
                 "-------------------------------" << std::endl;
    std::cout << terminal_message.str() << std::endl;
    if (reverse_count_limit_reached) {
      RCLCPP_WARN(
        get_logger(),
        "过渡阶段条件倒车第%zu轮结束：触发原因=%s；退出原因=%s；%s；"
        "退出判定[模式=%s，编码器累计里程=%.3f/%.3fm，单轮最大里程=%.3fm，"
        "实测倒转=%s，前方清空=%s，后方保护=%s，里程计保护=%s，"
        "map.y=%.3f<=%.3f=%s，map.x=%.3f>=%.3f=%s]；"
        "停车%.3fs后以当前base_link重规划；已达到配置上限%d轮，"
        "本次过渡不再触发条件倒车。",
        channel_completed_reverse_count_,
        completed_trigger_reason.c_str(),
        exit_reason.c_str(),
        hard_boundary_reached && !min_distance_reached ?
        "硬边界优先，最小距离被安全截断" : "退出条件满足",
        completed_x_force_reverse ? "入口X强制" : "普通条件",
        reverse_distance, qr_reverse_active_min_distance_, qr_reverse_max_distance_,
        completed_reverse_motion_confirmed ? "已确认" : "未确认",
        front_clear ? "是" : "否",
        rear_protection_reached ? "命中" : "清空",
        odom_protection_reached ? "命中" : "正常",
        robot_pose.y, qr_reverse_y_threshold_, y_limit_reached ? "是" : "否",
        robot_pose.x, qr_reverse_x_limit_, x_limit_reached ? "是" : "否",
        qr_reverse_post_stop_seconds_,
        qr_reverse_max_count_);
    } else {
      RCLCPP_INFO(
        get_logger(),
        "过渡阶段条件倒车第%zu轮结束：触发原因=%s；退出原因=%s；%s；"
        "退出判定[模式=%s，编码器累计里程=%.3f/%.3fm，单轮最大里程=%.3fm，"
        "实测倒转=%s，前方清空=%s，后方保护=%s，里程计保护=%s，"
        "map.y=%.3f<=%.3f=%s，map.x=%.3f>=%.3f=%s]；"
        "停车%.3fs后以当前base_link重规划；倒车机制继续保持武装%s。",
        channel_completed_reverse_count_,
        completed_trigger_reason.c_str(),
        exit_reason.c_str(),
        hard_boundary_reached && !min_distance_reached ?
        "硬边界优先，最小距离被安全截断" : "退出条件满足",
        completed_x_force_reverse ? "入口X强制" : "普通条件",
        reverse_distance, qr_reverse_active_min_distance_, qr_reverse_max_distance_,
        completed_reverse_motion_confirmed ? "已确认" : "未确认",
        front_clear ? "是" : "否",
        rear_protection_reached ? "命中" : "清空",
        odom_protection_reached ? "命中" : "正常",
        robot_pose.y, qr_reverse_y_threshold_, y_limit_reached ? "是" : "否",
        robot_pose.x, qr_reverse_x_limit_, x_limit_reached ? "是" : "否",
        qr_reverse_post_stop_seconds_,
        "");
    }
    qr_reverse_active_trigger_reason_.clear();
    // 退出倒车的第一帧就锁存本应继续执行的倒车转向。随后反相制动、
    // 配置停车时长、等待新雷达帧及重规划交接帧都只把linear.x置零，
    // 不再让舵机突然回中。
    qr_reverse_hold_angular_command_ = reverse_angular_command();
    cmd = geometry_msgs::msg::Twist();
    cmd.angular.z = qr_reverse_hold_angular_command_;
    return true;
  }

  set_reverse_command();
  if (debug_control_logs_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 500,
      "debug: 过渡阶段条件倒车，实测倒转=%s，实测轮速=%.3fm/s，"
      "编码器累计里程=%.3f/%.3fm，map.y=%.3f，"
      "前障碍=%s，后方保护=清空，"
      "输出=%.2f/%.2f。",
      qr_reverse_motion_confirmed_ ? "已确认" : "等待中",
      latest_chassis_linear_speed_,
      reverse_distance, qr_reverse_active_min_distance_, robot_pose.y,
      front_obstacle.found ? "是" : "否",
      cmd.linear.x, cmd.angular.z);
  }
  return true;
}

// 进入channel航点邻域后结束进场速度覆盖。A→C→B已经与Task2一起完成
// 静态偏移；Task2参数可按原门限提前切换，但过渡段全障碍雷达覆盖模式下，
// 绿色掩码仍要等路线进度真正越过B后才恢复。
void VisualLidarPathFollower::update_qr_channel_approach_phase(
  const VisualLidarPathFollower::RobotPose & robot_pose)
{
  if (qr_escape_state_ == QrEscapeState::PRE_REVERSE_BRAKE ||
    qr_escape_state_ == QrEscapeState::WAITING_FOR_PRE_REVERSE_SCAN ||
    qr_escape_state_ == QrEscapeState::BACKING_ARMED ||
    qr_escape_state_ == QrEscapeState::BACKING_RIGHT ||
    qr_escape_state_ == QrEscapeState::POST_STOP ||
    qr_escape_state_ == QrEscapeState::WAITING_FOR_POST_STOP_SCAN)
  {
    return;
  }

  if (waypoints_.empty()) {
    return;
  }
  const std::size_t entry_index = std::min(
    channel_entry_waypoint_index_, waypoints_.size() - 1);
  const double entry_waypoint_distance = std::hypot(
    waypoints_[entry_index].x - robot_pose.x,
    waypoints_[entry_index].y - robot_pose.y);
  bool crossed_entry_section = false;
  if (entry_index + 1 < waypoints_.size()) {
    const double tangent_x =
      waypoints_[entry_index + 1].x - waypoints_[entry_index].x;
    const double tangent_y =
      waypoints_[entry_index + 1].y - waypoints_[entry_index].y;
    const double tangent_length = std::hypot(tangent_x, tangent_y);
    if (tangent_length > 1.0e-8) {
      const double unit_x = tangent_x / tangent_length;
      const double unit_y = tangent_y / tangent_length;
      const double relative_x = robot_pose.x - waypoints_[entry_index].x;
      const double relative_y = robot_pose.y - waypoints_[entry_index].y;
      const double along_route = relative_x * unit_x + relative_y * unit_y;
      const double cross_track = std::abs(-relative_x * unit_y + relative_y * unit_x);
      crossed_entry_section =
        along_route >= 0.0 && cross_track <= waypoint_pass_radius(entry_index);
    }
  }
  if (entry_waypoint_distance > channel_handoff_waypoint_distance_ &&
    !crossed_entry_section)
  {
    return;
  }

  // 未提前达到配置轮数上限时，进入waypoint_channel永久关闭倒车机制。
  qr_channel_approach_active_ = false;
  qr_escape_state_ = QrEscapeState::IDLE;
  qr_pre_reverse_scan_sequence_ = 0;
  qr_pre_reverse_time_nanoseconds_ = 0;
  qr_reverse_rearm_after_post_stop_ = false;
  qr_reverse_x_force_pending_ = false;
  qr_reverse_active_trigger_reason_.clear();
  qr_reverse_active_x_force_ = false;
  qr_reverse_negative_command_published_ = false;
  qr_reverse_hold_angular_command_ = 0.0;
  qr_reverse_wheel_odom_distance_ = 0.0;
  qr_reverse_last_odom_position_valid_ = false;
  qr_reverse_motion_confirmed_ = false;
  qr_reverse_odom_fault_ = false;
  qr_reverse_waiting_for_odom_ = false;
  set_task_phase(TaskPhase::TASK2);
  current_index_ = std::min(
    std::max(current_index_, entry_index), waypoints_.size() - 1);
  first_waypoint_reached_ = true;
  RCLCPP_INFO(
    get_logger(),
    "已进入 waypoint_channel：延长段=%zu点，入口首点距离=%.3fm，入口截面穿越=%s；"
    "切换到Task2参数，并按当前路线控制模式继续处理记录点掩码。",
    entry_index, entry_waypoint_distance, crossed_entry_section ? "是" : "否");
}

double VisualLidarPathFollower::estimate_channel_waypoint_spacing(
  const std::vector<Waypoint> & route) const
{
  std::vector<double> segment_lengths;
  segment_lengths.reserve(route.size() > 1 ? route.size() - 1 : 0);
  for (std::size_t index = 1; index < route.size(); ++index) {
    const double length = std::hypot(
      route[index].x - route[index - 1].x,
      route[index].y - route[index - 1].y);
    if (std::isfinite(length) && length > 1.0e-6) {
      segment_lengths.push_back(length);
    }
  }
  if (segment_lengths.empty()) {
    return 0.02;
  }
  std::sort(segment_lengths.begin(), segment_lengths.end());
  const std::size_t middle = segment_lengths.size() / 2;
  if (segment_lengths.size() % 2 == 0) {
    return 0.5 * (segment_lengths[middle - 1] + segment_lengths[middle]);
  }
  return segment_lengths[middle];
}

// 以当前map->base_link位置为A，以配置坐标为C，以原始waypoint_channel
// 首点为B。A→C整段使用五次Bezier：A端切线直接取重规划时base_link.yaw，
// C端切线沿CB，且两端曲率为0；不再生成M点。B入口继续使用原有圆滑。
// 所有圆滑只修改“偏移前基准路线”；基准路线固化后先接上完整Task2路线，
// 再在A→C→B→Task2组合路线
// 上统一回放冻结绿/橙簇。B附近障碍的before/after窗口可以自然跨越B，
// 整条组合路线直接执行，不在阶段边界替换。
bool VisualLidarPathFollower::rebuild_channel_entry_route(
  const VisualLidarPathFollower::RobotPose & start_pose, const char * reason)
{
  if (channel_route_template_.empty() ||
    !std::isfinite(start_pose.x) || !std::isfinite(start_pose.y) ||
    !std::isfinite(start_pose.yaw))
  {
    return false;
  }
  const bool rebuild_after_reverse = channel_entry_route_rebuild_after_reverse_;

  const auto & channel_entry = channel_route_template_.front();
  const Point2D channel_start{start_pose.x, start_pose.y};
  const Point2D channel_start_tangent{
    std::cos(start_pose.yaw), std::sin(start_pose.yaw)};
  const Point2D channel_via{channel_entry_via_x_, channel_entry_via_y_};
  const Point2D channel_finish{channel_entry.x, channel_entry.y};
  const double ac_distance = std::hypot(
    channel_via.x - channel_start.x, channel_via.y - channel_start.y);
  const double cb_distance = std::hypot(
    channel_finish.x - channel_via.x, channel_finish.y - channel_via.y);
  double entry_path_distance = ac_distance + cb_distance;
  const double spacing = std::max(
    1.0e-3, estimate_channel_waypoint_spacing(channel_route_template_));
  auto segment_point_count = [spacing](double distance) -> std::size_t {
      return distance > 1.0e-6 ?
        std::max<std::size_t>(
        1, static_cast<std::size_t>(std::llround(distance / spacing))) : 0;
    };
  std::size_t ac_point_count = segment_point_count(ac_distance);
  const std::size_t cb_point_count = segment_point_count(cb_distance);
  std::size_t extension_point_count =
    ac_point_count + cb_point_count;

  std::vector<Waypoint> extended_route;
  extended_route.reserve(channel_route_template_.size() + extension_point_count + 1);
  auto append_entry_segment =
    [&](const Point2D & segment_start, const Point2D & segment_finish,
      std::size_t point_count, const std::string & name_prefix) {
      for (std::size_t index = 0; index < point_count; ++index) {
        const double ratio = static_cast<double>(index) /
          static_cast<double>(point_count);
        Waypoint extension = channel_entry;
        extension.name = name_prefix + std::to_string(index + 1);
        extension.x = segment_start.x + ratio * (segment_finish.x - segment_start.x);
        extension.y = segment_start.y + ratio * (segment_finish.y - segment_start.y);
        extension.reverse = false;
        extension.pass_radius = pass_radius_;
        extension.reverse_pass_radius = reverse_pass_radius_;
        extended_route.push_back(extension);
      }
    };
  const std::size_t channel_start_waypoint_index = extended_route.size();
  append_entry_segment(
    channel_start, channel_via, ac_point_count, "channel_entry_a_to_c_");
  std::size_t channel_via_waypoint_index = extended_route.size();
  append_entry_segment(
    channel_via, channel_finish, cb_point_count, "channel_entry_c_to_b_");
  channel_entry_waypoint_index_ = extended_route.size();
  // 默认与B重合，表示还没有生成需要保护的偏移前圆滑段。
  channel_entry_smoothing_first_index_ = channel_entry_waypoint_index_;
  extended_route.insert(
    extended_route.end(), channel_route_template_.begin(), channel_route_template_.end());

  // 阶段1a（偏移前）：把整段直线AC改写为端点零曲率的五次Bezier。
  // P0/P1/P2沿重规划时base_link.yaw等距共线，P3/P4/P5沿CB等距共线，
  // 因此曲线在A、C
  // 不仅切向连续，而且曲率严格回到0，不会在直线交界处突然跳转角。
  std::size_t channel_via_smoothing_last_index = channel_via_waypoint_index;
  double ac_curve_length = ac_distance;
  double ac_curve_max_curvature = 0.0;
  double ac_curve_handle_scale = 1.0;
  if (channel_ac_curve_enabled_ && ac_point_count >= 4 &&
    ac_distance > 1.0e-8 && cb_distance > 1.0e-8)
  {
    const Point2D cb_tangent{
      (channel_finish.x - channel_via.x) / cb_distance,
      (channel_finish.y - channel_via.y) / cb_distance};
    struct AcCurveCandidate
    {
      bool valid{false};
      double handle_scale{1.0};
      double curve_length{0.0};
      double max_curvature{0.0};
      std::vector<Point2D> dense_curve;
      std::vector<double> cumulative_lengths;
    };
    auto build_ac_candidate = [&](double handle_scale, bool check_self_intersection) {
        AcCurveCandidate candidate;
        candidate.handle_scale = handle_scale;
        const double start_handle = ac_distance *
          channel_ac_curve_start_handle_ratio_ * handle_scale;
        const double finish_handle = ac_distance *
          channel_ac_curve_finish_handle_ratio_ * handle_scale;
        const std::vector<Point2D> control{
          channel_start,
          Point2D{channel_start.x + start_handle * channel_start_tangent.x,
            channel_start.y + start_handle * channel_start_tangent.y},
          Point2D{channel_start.x + 2.0 * start_handle * channel_start_tangent.x,
            channel_start.y + 2.0 * start_handle * channel_start_tangent.y},
          Point2D{channel_via.x - 2.0 * finish_handle * cb_tangent.x,
            channel_via.y - 2.0 * finish_handle * cb_tangent.y},
          Point2D{channel_via.x - finish_handle * cb_tangent.x,
            channel_via.y - finish_handle * cb_tangent.y},
          channel_via};
        // 控制多边形在整个候选中不变；一、二阶导数控制点只计算一次，
        // 不再在每个稠密采样点中重复构造。
        const Point2D derivative_control[5]{
          Point2D{5.0 * (control[1].x - control[0].x),
            5.0 * (control[1].y - control[0].y)},
          Point2D{5.0 * (control[2].x - control[1].x),
            5.0 * (control[2].y - control[1].y)},
          Point2D{5.0 * (control[3].x - control[2].x),
            5.0 * (control[3].y - control[2].y)},
          Point2D{5.0 * (control[4].x - control[3].x),
            5.0 * (control[4].y - control[3].y)},
          Point2D{5.0 * (control[5].x - control[4].x),
            5.0 * (control[5].y - control[4].y)}};
        const Point2D second_control[4]{
          Point2D{4.0 * (derivative_control[1].x - derivative_control[0].x),
            4.0 * (derivative_control[1].y - derivative_control[0].y)},
          Point2D{4.0 * (derivative_control[2].x - derivative_control[1].x),
            4.0 * (derivative_control[2].y - derivative_control[1].y)},
          Point2D{4.0 * (derivative_control[3].x - derivative_control[2].x),
            4.0 * (derivative_control[3].y - derivative_control[2].y)},
          Point2D{4.0 * (derivative_control[4].x - derivative_control[3].x),
            4.0 * (derivative_control[4].y - derivative_control[3].y)}};
        const std::size_t dense_count = std::max<std::size_t>(
          120, 8 * ac_point_count);
        candidate.dense_curve.reserve(dense_count + 1);
        candidate.cumulative_lengths.reserve(dense_count + 1);
        const double distance_change_tolerance = std::max(
          1.0e-10, 1.0e-7 * ac_distance * ac_distance);
        double previous_distance_to_c_sq = 0.0;
        double previous_tangent_yaw = 0.0;
        double cumulative_heading_change = 0.0;
        bool approaching_c = false;
        for (std::size_t index = 0; index <= dense_count; ++index) {
          const double t = static_cast<double>(index) /
            static_cast<double>(dense_count);
          const double u = 1.0 - t;
          const double weights[6]{
            u * u * u * u * u,
            5.0 * u * u * u * u * t,
            10.0 * u * u * u * t * t,
            10.0 * u * u * t * t * t,
            5.0 * u * t * t * t * t,
            t * t * t * t * t};
          Point2D point;
          for (std::size_t control_index = 0; control_index < 6; ++control_index) {
            point.x += weights[control_index] * control[control_index].x;
            point.y += weights[control_index] * control[control_index].y;
          }
          if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
            return candidate;
          }
          const double distance_to_c_sq =
            (point.x - channel_via.x) * (point.x - channel_via.x) +
            (point.y - channel_via.y) * (point.y - channel_via.y);
          if (index > 0) {
            const double distance_change =
              distance_to_c_sq - previous_distance_to_c_sq;
            if (distance_change < -distance_change_tolerance) {
              approaching_c = true;
            } else if (approaching_c &&
              distance_change > distance_change_tolerance)
            {
              // 允许车头与AC弦夹角过大时先短暂远离C；一旦开始接近C，
              // 后续不允许再次远离，避免S形往返和绕圈。
              return candidate;
            }
          }
          previous_distance_to_c_sq = distance_to_c_sq;
          candidate.dense_curve.push_back(point);
          if (index == 0) {
            candidate.cumulative_lengths.push_back(0.0);
          } else {
            candidate.cumulative_lengths.push_back(
              candidate.cumulative_lengths.back() + std::hypot(
                point.x - candidate.dense_curve[index - 1].x,
                point.y - candidate.dense_curve[index - 1].y));
          }
          const double derivative_weights[5]{
            u * u * u * u, 4.0 * u * u * u * t,
            6.0 * u * u * t * t, 4.0 * u * t * t * t,
            t * t * t * t};
          const double second_weights[4]{
            u * u * u, 3.0 * u * u * t,
            3.0 * u * t * t, t * t * t};
          Point2D derivative;
          Point2D second_derivative;
          for (std::size_t derivative_index = 0;
            derivative_index < 5; ++derivative_index)
          {
            derivative.x += derivative_weights[derivative_index] *
              derivative_control[derivative_index].x;
            derivative.y += derivative_weights[derivative_index] *
              derivative_control[derivative_index].y;
          }
          for (std::size_t derivative_index = 0;
            derivative_index < 4; ++derivative_index)
          {
            second_derivative.x += second_weights[derivative_index] *
              second_control[derivative_index].x;
            second_derivative.y += second_weights[derivative_index] *
              second_control[derivative_index].y;
          }
          const double speed = std::hypot(derivative.x, derivative.y);
          if (speed < 1.0e-8) {
            return candidate;
          }
          const double tangent_yaw = std::atan2(derivative.y, derivative.x);
          if (index > 0) {
            const double heading_delta = std::atan2(
              std::sin(tangent_yaw - previous_tangent_yaw),
              std::cos(tangent_yaw - previous_tangent_yaw));
            cumulative_heading_change += std::abs(heading_delta);
            // 单段AC最多允许累计转向270度，超过即视为绕圈。
            if (cumulative_heading_change > 1.5 * M_PI) {
              return candidate;
            }
          }
          previous_tangent_yaw = tangent_yaw;
          const double curvature = std::abs(
            derivative.x * second_derivative.y -
            derivative.y * second_derivative.x) /
            (speed * speed * speed);
          if (!std::isfinite(curvature)) {
            return candidate;
          }
          candidate.max_curvature = std::max(
            candidate.max_curvature, curvature);
        }
        candidate.curve_length = candidate.cumulative_lengths.back();
        if (!approaching_c || candidate.curve_length <= 1.0e-8 ||
          candidate.curve_length > 2.5 * ac_distance ||
          !std::isfinite(candidate.max_curvature))
        {
          return candidate;
        }

        if (check_self_intersection) {
          // 只对最终候选做精确自交检查。先按线段包围盒min_x排序，只有
          // X/Y包围盒可能重叠的非相邻线段才进入叉积判断；结果与旧版
          // 全量两两检查一致，通常复杂度由O(N²)降至接近O(N)。
          struct SegmentBounds
          {
            std::size_t index{0};
            double min_x{0.0};
            double max_x{0.0};
            double min_y{0.0};
            double max_y{0.0};
          };
          std::vector<SegmentBounds> segments;
          segments.reserve(candidate.dense_curve.size() - 1);
          for (std::size_t index = 0;
            index + 1 < candidate.dense_curve.size(); ++index)
          {
            const auto & first = candidate.dense_curve[index];
            const auto & second = candidate.dense_curve[index + 1];
            segments.push_back(SegmentBounds{
              index,
              std::min(first.x, second.x), std::max(first.x, second.x),
              std::min(first.y, second.y), std::max(first.y, second.y)});
          }
          std::sort(
            segments.begin(), segments.end(),
            [](const SegmentBounds & lhs, const SegmentBounds & rhs) {
              if (lhs.min_x != rhs.min_x) {
                return lhs.min_x < rhs.min_x;
              }
              return lhs.index < rhs.index;
            });
          auto cross = [](const Point2D & origin, const Point2D & first,
              const Point2D & second) {
              return (first.x - origin.x) * (second.y - origin.y) -
                     (first.y - origin.y) * (second.x - origin.x);
            };
          constexpr double intersection_epsilon = 1.0e-12;
          for (std::size_t left_position = 0;
            left_position < segments.size(); ++left_position)
          {
            const auto & left = segments[left_position];
            for (std::size_t right_position = left_position + 1;
              right_position < segments.size(); ++right_position)
            {
              const auto & right = segments[right_position];
              if (right.min_x > left.max_x + intersection_epsilon) {
                break;
              }
              if (left.index + 1 == right.index ||
                right.index + 1 == left.index ||
                right.min_y > left.max_y + intersection_epsilon ||
                left.min_y > right.max_y + intersection_epsilon)
              {
                continue;
              }
              const auto & a = candidate.dense_curve[left.index];
              const auto & b = candidate.dense_curve[left.index + 1];
              const auto & c = candidate.dense_curve[right.index];
              const auto & d = candidate.dense_curve[right.index + 1];
              const double ab_c = cross(a, b, c);
              const double ab_d = cross(a, b, d);
              const double cd_a = cross(c, d, a);
              const double cd_b = cross(c, d, b);
              const bool c_different_sides =
                (ab_c > intersection_epsilon && ab_d < -intersection_epsilon) ||
                (ab_c < -intersection_epsilon && ab_d > intersection_epsilon);
              const bool a_different_sides =
                (cd_a > intersection_epsilon && cd_b < -intersection_epsilon) ||
                (cd_a < -intersection_epsilon && cd_b > intersection_epsilon);
              if (c_different_sides && a_different_sides) {
                return candidate;
              }
            }
          }
        }
        candidate.valid = true;
        return candidate;
      };

    struct AcCurveScore
    {
      double handle_scale{1.0};
      double max_curvature{std::numeric_limits<double>::infinity()};
      bool within_curvature_target{false};
    };
    std::vector<AcCurveScore> candidate_scores;
    candidate_scores.reserve(41);
    // 第一阶段仍检查全部41个用户允许的比例，但只做O(N)的长度、曲率、
    // 单调接近C和累计转角筛选；不再让每个候选都做稠密线段两两相交。
    for (int scale_step = 0; scale_step <= 40; ++scale_step) {
      const double scale = 1.0 + 0.025 * static_cast<double>(scale_step);
      const auto candidate = build_ac_candidate(scale, false);
      if (!candidate.valid) {
        continue;
      }
      candidate_scores.push_back(AcCurveScore{
        scale, candidate.max_curvature,
        candidate.max_curvature <= channel_ac_curve_max_curvature_});
    }

    // 与旧选择原则一致：优先最小scale且达到曲率目标的短粉线；如果没有
    // 达标候选，则按曲率从低到高尝试。只有当前最优候选自交时才检查下一条。
    std::stable_sort(
      candidate_scores.begin(), candidate_scores.end(),
      [](const AcCurveScore & lhs, const AcCurveScore & rhs) {
        if (lhs.within_curvature_target != rhs.within_curvature_target) {
          return lhs.within_curvature_target;
        }
        if (lhs.within_curvature_target) {
          return lhs.handle_scale < rhs.handle_scale;
        }
        if (lhs.max_curvature != rhs.max_curvature) {
          return lhs.max_curvature < rhs.max_curvature;
        }
        return lhs.handle_scale < rhs.handle_scale;
      });
    AcCurveCandidate selected_candidate;
    for (const auto & score : candidate_scores) {
      auto candidate = build_ac_candidate(score.handle_scale, true);
      if (candidate.valid) {
        selected_candidate = std::move(candidate);
        break;
      }
    }
    if (selected_candidate.valid) {
      // 原AC点数按直线弦长计算；五次曲线更长，必须在C之前补足点槽，
      // 才能继续保持与channel模板一致的约0.02m等弧长点距。
      const std::size_t curve_point_count = segment_point_count(
        selected_candidate.curve_length);
      if (curve_point_count > ac_point_count) {
        const std::size_t additional_point_count =
          curve_point_count - ac_point_count;
        Waypoint additional_waypoint = channel_entry;
        additional_waypoint.reverse = false;
        additional_waypoint.pass_radius = pass_radius_;
        additional_waypoint.reverse_pass_radius = reverse_pass_radius_;
        extended_route.insert(
          extended_route.begin() + channel_via_waypoint_index,
          additional_point_count, additional_waypoint);
        ac_point_count = curve_point_count;
        channel_via_waypoint_index += additional_point_count;
        channel_via_smoothing_last_index = channel_via_waypoint_index;
        channel_entry_waypoint_index_ += additional_point_count;
        extension_point_count += additional_point_count;
      }
      for (std::size_t step = 1; step < ac_point_count; ++step) {
        const double target_length = selected_candidate.curve_length *
          static_cast<double>(step) / static_cast<double>(ac_point_count);
        const auto upper = std::lower_bound(
          selected_candidate.cumulative_lengths.begin(),
          selected_candidate.cumulative_lengths.end(), target_length);
        const std::size_t upper_index = static_cast<std::size_t>(
          std::distance(selected_candidate.cumulative_lengths.begin(), upper));
        const std::size_t lower_index = upper_index > 0 ? upper_index - 1 : 0;
        const double section_length =
          selected_candidate.cumulative_lengths[upper_index] -
          selected_candidate.cumulative_lengths[lower_index];
        const double ratio = section_length > 1.0e-12 ?
          (target_length - selected_candidate.cumulative_lengths[lower_index]) /
          section_length : 0.0;
        auto & waypoint = extended_route[channel_start_waypoint_index + step];
        waypoint.name = "channel_entry_a_to_c_curve_" + std::to_string(step + 1);
        waypoint.x = selected_candidate.dense_curve[lower_index].x + ratio *
          (selected_candidate.dense_curve[upper_index].x -
          selected_candidate.dense_curve[lower_index].x);
        waypoint.y = selected_candidate.dense_curve[lower_index].y + ratio *
          (selected_candidate.dense_curve[upper_index].y -
          selected_candidate.dense_curve[lower_index].y);
      }
      ac_curve_length = selected_candidate.curve_length;
      ac_curve_max_curvature = selected_candidate.max_curvature;
      ac_curve_handle_scale = selected_candidate.handle_scale;
      entry_path_distance = ac_curve_length + cb_distance;
      channel_entry_smoothing_first_index_ = std::min(
        channel_entry_smoothing_first_index_, channel_start_waypoint_index);
      const double direction_change = std::acos(std::clamp(
        channel_start_tangent.x * cb_tangent.x +
        channel_start_tangent.y * cb_tangent.y,
        -1.0, 1.0)) * 180.0 / M_PI;
      RCLCPP_INFO(
        get_logger(),
        "A→C→B整段AC圆滑：A端直接使用base_link.yaw，"
        "五次Bezier等弧长插点，方向变化=%.1f°，A/C端曲率=0，"
        "曲线长=%.3fm，最大曲率=%.3f/m（最小半径=%.3fm），"
        "控制柄缩放=%.3f。",
        direction_change, ac_curve_length, ac_curve_max_curvature,
        ac_curve_max_curvature > 1.0e-9 ?
        1.0 / ac_curve_max_curvature : std::numeric_limits<double>::infinity(),
        ac_curve_handle_scale);
    }
  }

  // 阶段1b（偏移前）：C圆滑区之后的CB直线末段再圆滑到B。使用五次
  // Bezier让起点沿CB、终点沿channel首段，并让两端曲率都回到0。
  // 路线仍精确经过B；circle_envelope随后在完整路线的同一s轴上偏移时，
  // 不会把仅有C1连续的曲率接缝放大成B附近的小尖峰。
  // B圆滑的起点不得越过上面的C圆滑终点。
  const bool has_cb_entry_tangent = cb_distance > 1.0e-8;
  const Point2D entry_tangent_start = has_cb_entry_tangent ? channel_via : channel_start;
  const std::size_t entry_tail_point_count = has_cb_entry_tangent ?
    channel_entry_waypoint_index_ - channel_via_smoothing_last_index :
    ac_point_count;
  if (entry_tail_point_count >= 2 &&
    channel_route_template_.size() >= 2 &&
    channel_entry_corner_smoothing_length_ > spacing)
  {
    const Point2D line_vector{
      channel_entry.x - entry_tangent_start.x,
      channel_entry.y - entry_tangent_start.y};
    const double line_length = std::hypot(line_vector.x, line_vector.y);
    Point2D channel_vector;
    double channel_length = 0.0;
    for (std::size_t index = 1; index < channel_route_template_.size(); ++index) {
      channel_vector = Point2D{
        channel_route_template_[index].x - channel_entry.x,
        channel_route_template_[index].y - channel_entry.y};
      channel_length = std::hypot(channel_vector.x, channel_vector.y);
      if (channel_length > 1.0e-8) {
        break;
      }
    }
    if (line_length > 1.0e-8 && channel_length > 1.0e-8) {
      const Point2D line_tangent{
        line_vector.x / line_length, line_vector.y / line_length};
      const Point2D channel_tangent{
        channel_vector.x / channel_length, channel_vector.y / channel_length};
      const double tangent_alignment = std::clamp(
        line_tangent.x * channel_tangent.x +
        line_tangent.y * channel_tangent.y, -1.0, 1.0);
      if (tangent_alignment < 0.9995) {
        const double requested_blend_length = std::min(
          channel_entry_corner_smoothing_length_, line_length);
        const std::size_t blend_segments = std::min(
          entry_tail_point_count,
          std::max<std::size_t>(
            2, static_cast<std::size_t>(
              std::llround(requested_blend_length / spacing))));
        const std::size_t anchor_index =
          channel_entry_waypoint_index_ - blend_segments;
        channel_entry_smoothing_first_index_ = std::min(
          channel_entry_smoothing_first_index_, anchor_index);
        const Point2D start{
          extended_route[anchor_index].x, extended_route[anchor_index].y};
        const Point2D finish{channel_entry.x, channel_entry.y};
        const double chord_length = std::hypot(
          finish.x - start.x, finish.y - start.y);
        // 五次Bezier的P0/P1/P2与P3/P4/P5分别等距共线。这样二阶导数
        // 在两端都为0；handle=chord/5还保持与旧三次曲线相同量级的
        // 端点一阶速度，不额外放大B前摆幅。
        const double handle_length = chord_length / 5.0;
        const Point2D control[6]{
          start,
          Point2D{start.x + handle_length * line_tangent.x,
            start.y + handle_length * line_tangent.y},
          Point2D{start.x + 2.0 * handle_length * line_tangent.x,
            start.y + 2.0 * handle_length * line_tangent.y},
          Point2D{finish.x - 2.0 * handle_length * channel_tangent.x,
            finish.y - 2.0 * handle_length * channel_tangent.y},
          Point2D{finish.x - handle_length * channel_tangent.x,
            finish.y - handle_length * channel_tangent.y},
          finish};
        auto bezier_point = [&](double t) {
            const double one_minus_t = 1.0 - t;
            const double weights[6]{
              one_minus_t * one_minus_t * one_minus_t * one_minus_t * one_minus_t,
              5.0 * one_minus_t * one_minus_t * one_minus_t * one_minus_t * t,
              10.0 * one_minus_t * one_minus_t * one_minus_t * t * t,
              10.0 * one_minus_t * one_minus_t * t * t * t,
              5.0 * one_minus_t * t * t * t * t,
              t * t * t * t * t};
            Point2D point;
            for (std::size_t control_index = 0;
              control_index < 6; ++control_index)
            {
              point.x += weights[control_index] * control[control_index].x;
              point.y += weights[control_index] * control[control_index].y;
            }
            return point;
          };

        const std::size_t dense_count = std::max<std::size_t>(
          40, 8 * blend_segments);
        std::vector<Point2D> dense_curve;
        std::vector<double> cumulative_lengths;
        dense_curve.reserve(dense_count + 1);
        cumulative_lengths.reserve(dense_count + 1);
        dense_curve.push_back(bezier_point(0.0));
        cumulative_lengths.push_back(0.0);
        for (std::size_t index = 1; index <= dense_count; ++index) {
          dense_curve.push_back(bezier_point(
              static_cast<double>(index) / static_cast<double>(dense_count)));
          cumulative_lengths.push_back(
            cumulative_lengths.back() + std::hypot(
              dense_curve[index].x - dense_curve[index - 1].x,
              dense_curve[index].y - dense_curve[index - 1].y));
        }
        const double curve_length = cumulative_lengths.back();
        for (std::size_t step = 1; step < blend_segments; ++step) {
          const double target_length = curve_length *
            static_cast<double>(step) / static_cast<double>(blend_segments);
          const auto upper = std::lower_bound(
            cumulative_lengths.begin(), cumulative_lengths.end(), target_length);
          const std::size_t upper_index = static_cast<std::size_t>(
            std::distance(cumulative_lengths.begin(), upper));
          const std::size_t lower_index = upper_index > 0 ? upper_index - 1 : 0;
          const double section_length =
            cumulative_lengths[upper_index] - cumulative_lengths[lower_index];
          const double ratio = section_length > 1.0e-12 ?
            (target_length - cumulative_lengths[lower_index]) / section_length : 0.0;
          auto & waypoint = extended_route[anchor_index + step];
          waypoint.x = dense_curve[lower_index].x + ratio *
            (dense_curve[upper_index].x - dense_curve[lower_index].x);
          waypoint.y = dense_curve[lower_index].y + ratio *
            (dense_curve[upper_index].y - dense_curve[lower_index].y);
        }
        RCLCPP_INFO(
          get_logger(),
          "A→C→B偏移前圆滑：在B前%.3fm内使用%zu段等弧长五次Bezier，"
          "转角=%.1f°，两端曲率=0；随后与Task2拼成完整路线并统一执行静态偏移。",
          curve_length, blend_segments,
          std::acos(tangent_alignment) * 180.0 / M_PI);
      }
    }
  }
  waypoints_ = std::move(extended_route);

  // 阶段边界：从这里开始，original_waypoints_保存的就是已在A/C/B处圆滑的
  // 偏移前基准路线。后续只允许通用waypoint_shift读取该基准路线并偏移。
  reset_waypoint_shift_state();
  // 仅保存A到B（含B）作为本轮最新的无偏移ACB保护线。倒车后的重建
  // 会再次执行本函数并覆盖旧线，Task2其余部分由顺/逆原始路线保护。
  const std::size_t guard_point_count = std::min(
    channel_entry_waypoint_index_ + 1, original_waypoints_.size());
  logic3_latest_channel_entry_update_guard_route_.assign(
    original_waypoints_.begin(), original_waypoints_.begin() + guard_point_count);
  current_index_ = 0;
  first_waypoint_reached_ = false;
  reset_image_capture_corridor();
  channel_entry_route_rebuild_pending_ = false;
  channel_entry_route_rebuild_after_reverse_ = false;

  if (logic3_planning_pose_anchored_) {
    final_frozen_marker_ = Point2D{start_pose.x, start_pose.y};
    final_frozen_marker_yaw_ = start_pose.yaw;
    if (rebuild_after_reverse) {
      final_frozen_marker_label_ =
        "REPLAN " + std::to_string(channel_completed_reverse_count_);
    } else {
      final_frozen_marker_label_ = "ROUTE START";
    }
    final_frozen_marker_valid_ = true;
    // A′只服务于二维码倒车完成后的这一轮前后分类，不参与首次进场或
    // 其他阶段重规划，更不改变实际路线起点A。
    channel_entry_rear_static_filter_active_ = rebuild_after_reverse;
    channel_entry_rear_static_filter_start_x_ = start_pose.x;
    channel_entry_rear_static_filter_start_y_ = start_pose.y;
    channel_entry_rear_static_filter_start_yaw_ = start_pose.yaw;
    channel_entry_rear_static_filter_first_index_ = 0;
    if (rebuild_after_reverse) {
      channel_entry_rear_static_filter_start_x_ +=
        channel_entry_rear_static_reference_forward_distance_ *
        std::cos(start_pose.yaw);
      channel_entry_rear_static_filter_start_y_ +=
        channel_entry_rear_static_reference_forward_distance_ *
        std::sin(start_pose.yaw);
      // M点已删除，但倒车后重规划仍保留原先0.30m量级的车后静态影响
      // 保护长度。保护边界沿实际A→C曲线累计，不生成几何M点。
      double protected_route_length = 0.0;
      const std::size_t protection_last_index = std::min(
        channel_via_waypoint_index, original_waypoints_.size() - 1);
      for (std::size_t index = 1; index <= protection_last_index; ++index) {
        protected_route_length += std::hypot(
          original_waypoints_[index].x - original_waypoints_[index - 1].x,
          original_waypoints_[index].y - original_waypoints_[index - 1].y);
        channel_entry_rear_static_filter_first_index_ = index;
        if (protected_route_length + 1.0e-9 >=
          channel_entry_heading_reference_distance_)
        {
          break;
        }
      }
    }
    apply_logic3_frozen_clusters_to_current_route("final_frozen");
    channel_entry_rear_static_filter_active_ = false;
    // 二维码内容只在第一次正式进场的蓝色起点和03图均已更新后播报。
    // 当前流程在强制倒车前不生成A→C→B，因此第一次正式进场本身可能
    // 就是rebuild_after_reverse=true。这里始终尝试发布，实际是否需要
    // 播报由pending/published状态去重；后续重规划不会重复语音。
    publish_pending_qr_speech();
  }
  // 即使冻结绿色簇为空、偏移量没有变化，倒车后的A点也已经改变，
  // 所以必须无条件刷新调试路线。
  publish_debug_route();

  RCLCPP_WARN(
    get_logger(),
    "%s：以base_link A=(%.3f, %.3f, yaw=%.3f)为起点，"
    "A端切线直接使用车头朝向，经C=(%.3f, %.3f)连接"
    "waypoint_channel首点B=(%.3f, %.3f)；不生成M点。"
    "AC曲线=%.3fm，CB=%.3fm，总长=%.3fm，"
    "channel中位点距=%.4fm，新增%zu个等间距点；"
    "%s。",
    reason ? reason : "channel进场重建", start_pose.x, start_pose.y, start_pose.yaw,
    channel_via.x, channel_via.y, channel_entry.x, channel_entry.y,
    ac_curve_length, cb_distance, entry_path_distance, spacing,
    extension_point_count,
    logic3_planning_pose_anchored_ ?
    "随后已按冻结静态坐标重新规划路线" :
    "冻结静态快照尚未锁定，锁定后再执行偏移规划");
  return true;
}
