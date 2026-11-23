#include "origincar_system/visual_lidar_path_follower.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

// 请求停车。车辆此前由本节点发布过非零线速度时，统一速度出口会先执行
// 与基地停车相同的反相制动；已经静止时仍只发布零速度，不重复反相。
void VisualLidarPathFollower::publish_stop()
{
  reset_path_pid_state();
  reset_linear_speed_limit();
  if (cmd_pub_) {
    publish_velocity_command(geometry_msgs::msg::Twist());
  }
}

// 只停止车身平移，保留调用方明确给出的转向量。用于图生文及二维码
// 临时停车，避免停车阶段把舵机强制回中；真正的全停车仍走publish_stop()。
void VisualLidarPathFollower::publish_translation_stop(double angular_command)
{
  geometry_msgs::msg::Twist cmd;
  cmd.angular.z = std::isfinite(angular_command) ? angular_command : 0.0;
  reset_linear_speed_limit();
  if (cmd_pub_) {
    publish_velocity_command(cmd);
  }
}

// 直接停止平移且明确跳过统一反相制动。只用于用户指定的两处交接：
// 1. 真实二维码触发Task1→Task2；2. Task1末点停车等待二维码。
// 同时取消尚未完成的中途制动状态并解除武装，后续重复零速不会再根据
// 仍有余量的/chassis/odom轮速补触发反相脉冲。其他停车仍走上面的统一出口。
void VisualLidarPathFollower::publish_translation_stop_without_reverse_brake(
  double angular_command)
{
  if (!cmd_pub_) {
    return;
  }

  geometry_msgs::msg::Twist cmd;
  cmd.angular.z = std::isfinite(angular_command) ? angular_command : 0.0;
  stop_brake_sequence_active_ = false;
  stop_brake_reverse_cycles_remaining_ = 0;
  stop_brake_zero_cycles_remaining_ = 0;
  stop_brake_linear_command_ = 0.0;
  stop_brake_motion_direction_ = 0;
  stop_brake_start_odom_sequence_ = chassis_odom_sequence_;
  stop_brake_armed_ = false;
  last_published_motion_linear_command_ = 0.0;
  reset_linear_speed_limit();
  cmd_pub_->publish(cmd);
  last_published_angular_command_ = cmd.angular.z;
}

bool VisualLidarPathFollower::should_publish_stop_on_exit() const
{
  return publish_stop_on_exit_;
}

const char * VisualLidarPathFollower::control_state_name(VisualLidarPathFollower::ControlState state) const
{
  switch (state) {
    case ControlState::WAITING_FOR_POSE:
      return "等待定位";
    case ControlState::WAITING_FOR_QR:
      return "等待二维码";
    case ControlState::PATH_FOLLOWING:
      return "巡线";
    case ControlState::RETURNING_TO_START:
      return "返回起点";
    case ControlState::FINISHED:
      return "完成";
  }
  return "未知";
}

void VisualLidarPathFollower::set_control_state(VisualLidarPathFollower::ControlState next_state)
{
  if (control_state_ == next_state) {
    return;
  }
  if (debug_control_logs_) {
    RCLCPP_INFO(
      get_logger(), "debug: 状态 %s -> %s。",
      control_state_name(control_state_), control_state_name(next_state));
  }
  control_state_ = next_state;
}

void VisualLidarPathFollower::perform_return_brake_sequence()
{
  if (!cmd_pub_) {
    return;
  }

  // 终点随后会shutdown，没有后续控制周期可推进非阻塞状态机，因此这里
  // 同步发送同一套命令。方向仍按停车前实际运动方向反相。
  stop_brake_sequence_active_ = false;
  stop_brake_reverse_cycles_remaining_ = 0;
  stop_brake_zero_cycles_remaining_ = 0;
  stop_brake_linear_command_ = 0.0;
  double motion_linear_command = last_published_motion_linear_command_;
  if (latest_chassis_linear_speed_valid_ &&
    std::isfinite(latest_chassis_linear_speed_) &&
    std::abs(latest_chassis_linear_speed_) > start_speed_threshold_)
  {
    motion_linear_command = latest_chassis_linear_speed_;
  }
  if (!std::isfinite(motion_linear_command) ||
    std::abs(motion_linear_command) <= 1.0e-6)
  {
    // Task3正常从正向返程进入基地；无可用速度时保持原基地制动方向。
    motion_linear_command = 1.0;
  }

  const auto command_period = std::chrono::duration<double>(
    1.0 / std::max(control_frequency_, 1.0));

  if (return_brake_reverse_speed_ > 0.0 && return_brake_reverse_cycles_ > 0) {
    geometry_msgs::msg::Twist reverse_cmd;
    reverse_cmd.linear.x = -std::copysign(
      return_brake_reverse_speed_, motion_linear_command);
    for (int cycle = 0; cycle < return_brake_reverse_cycles_; ++cycle) {
      cmd_pub_->publish(reverse_cmd);
      std::this_thread::sleep_for(command_period);
    }
  }

  const geometry_msgs::msg::Twist stop_cmd;
  for (int cycle = 0; cycle < return_brake_zero_cycles_; ++cycle) {
    cmd_pub_->publish(stop_cmd);
    std::this_thread::sleep_for(command_period);
  }
  // 即使把零速保持周期设为0，也保证反向制动后至少发送一次停止命令。
  cmd_pub_->publish(stop_cmd);
  stop_brake_armed_ = false;
  last_published_motion_linear_command_ = 0.0;
  reset_path_pid_state();
  reset_linear_speed_limit();

  RCLCPP_INFO(
    get_logger(),
    "同步停车制动完成：停车前方向=%s，反相速度=%+.2fm/s，"
    "反向=%d周期，零速保持=%d周期，控制频率=%.1fHz。",
    motion_linear_command > 0.0 ? "前进" : "倒车",
    -std::copysign(return_brake_reverse_speed_, motion_linear_command),
    return_brake_reverse_cycles_,
    return_brake_zero_cycles_, control_frequency_);
}

void VisualLidarPathFollower::apply_qr_code_speed_limit(
  geometry_msgs::msg::Twist & cmd)
{
  if (task_phase_ != TaskPhase::TASK1 || valid_qr_received_ ||
    !latest_return_visual_targets_)
  {
    return;
  }

  for (const auto & target : latest_return_visual_targets_->targets) {
    if (target.type != "qr_code" || target.rois.empty()) {
      continue;
    }
    const int height = return_visual_target_height(target);
    if (height < qr_code_height_threshold_) {
      continue;
    }
    if (cmd.linear.x > qr_code_speed_limit_) {
      cmd.linear.x = qr_code_speed_limit_;
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "二维码高度=%d达到阈值=%d，尚未收到有效解码，线速度封顶%.2fm/s。",
        height, qr_code_height_threshold_, qr_code_speed_limit_);
    }
    return;
  }
}

void VisualLidarPathFollower::reset_linear_speed_limit()
{
  linear_speed_limit_active_ = false;
  linear_speed_limit_direction_ = 0;
  linear_speed_limit_magnitude_ = 0.0;
  linear_speed_limit_update_time_valid_ = false;
}

void VisualLidarPathFollower::arm_event_start_acceleration(
  EventStartContext context)
{
  // 不重置当前限速基准。二维码/图生文停车期间的零速命令会
  // 正常清空基准，但本标志保持，直到下一次起步达到当时请求速度。
  event_start_context_ = context;
  event_start_accel_active_ = true;
}

double VisualLidarPathFollower::active_event_start_speed() const
{
  switch (event_start_context_) {
    case EventStartContext::TRUSTED_DEPARTURE:
      return trusted_departure_start_speed_;
    case EventStartContext::IMAGE_CAPTURE_RESUME:
      return image_capture_start_speed_;
    case EventStartContext::POST_REVERSE_RESUME:
      return post_reverse_start_speed_;
  }
  return start_speed_;
}

double VisualLidarPathFollower::active_event_start_max_accel() const
{
  switch (event_start_context_) {
    case EventStartContext::TRUSTED_DEPARTURE:
      return trusted_departure_start_max_accel_;
    case EventStartContext::IMAGE_CAPTURE_RESUME:
      return image_capture_start_max_accel_;
    case EventStartContext::POST_REVERSE_RESUME:
      return post_reverse_start_max_accel_;
  }
  return global_max_accel_;
}

void VisualLidarPathFollower::apply_global_linear_speed_limit(
  geometry_msgs::msg::Twist & cmd)
{
  constexpr double kCommandEpsilon = 1.0e-6;
  const double requested_speed = cmd.linear.x;
  if (!std::isfinite(requested_speed) || std::abs(requested_speed) <= kCommandEpsilon) {
    // 零速和停车制动必须立即生效，同时为下一次冷启动清空旧状态。
    reset_linear_speed_limit();
    return;
  }

  const int requested_direction = requested_speed > 0.0 ? 1 : -1;
  const double requested_magnitude = std::abs(requested_speed);
  const auto steady_now = std::chrono::steady_clock::now();
  const bool chassis_is_stationary = !latest_chassis_linear_speed_valid_ ||
    std::abs(latest_chassis_linear_speed_) <= start_speed_threshold_;
  const double active_start_speed = event_start_accel_active_ ?
    active_event_start_speed() : start_speed_;

  if (enable_start_acceleration_limit_ && active_start_speed > 0.0 &&
    chassis_is_stationary)
  {
    // 冷启动不是从0按加速度慢慢爬升，而是直接给当前事件的独立
    // start_speed；未武装事件时使用公共start_speed。只要实测轮速仍在
    // 静止阈值内就保持该值，确认运动后再按对应事件加速度继续上升。
    linear_speed_limit_active_ = true;
    linear_speed_limit_direction_ = requested_direction;
    linear_speed_limit_magnitude_ = std::min(
      requested_magnitude, active_start_speed);
    linear_speed_limit_updated_at_ = steady_now;
    linear_speed_limit_update_time_valid_ = true;
    cmd.linear.x = requested_direction * linear_speed_limit_magnitude_;
    return;
  }

  if (!linear_speed_limit_active_ ||
    linear_speed_limit_direction_ != requested_direction)
  {
    // 节点在车辆已经运动时启动，或上层没有先请求零速就直接换向时，
    // 以本次目标建立新基准。正常停车和反相制动会在进入这里前立即执行。
    linear_speed_limit_active_ = true;
    linear_speed_limit_direction_ = requested_direction;
    linear_speed_limit_magnitude_ = requested_magnitude;
    linear_speed_limit_updated_at_ = steady_now;
    linear_speed_limit_update_time_valid_ = true;
    return;
  }

  double elapsed_seconds = 1.0 / std::max(control_frequency_, 1.0);
  if (linear_speed_limit_update_time_valid_) {
    elapsed_seconds = std::chrono::duration<double>(
      steady_now - linear_speed_limit_updated_at_).count();
  }
  // 控制线程偶发阻塞时不允许一次补偿整个空档，避免恢复周期突然跳速。
  const double maximum_step_duration = 2.0 / std::max(control_frequency_, 1.0);
  elapsed_seconds = std::clamp(elapsed_seconds, 0.0, maximum_step_duration);
  linear_speed_limit_updated_at_ = steady_now;
  linear_speed_limit_update_time_valid_ = true;

  if (requested_magnitude > linear_speed_limit_magnitude_ + kCommandEpsilon) {
    // 三个指定事件后的下一次起步使用独立斜率；其他加速继续使用全局值。
    // 0表示本次加速侧限制关闭，直接采用目标速度。
    const double active_max_accel = event_start_accel_active_ ?
      active_event_start_max_accel() : global_max_accel_;
    if (active_max_accel <= 0.0) {
      linear_speed_limit_magnitude_ = requested_magnitude;
    } else {
      linear_speed_limit_magnitude_ = std::min(
        requested_magnitude,
        linear_speed_limit_magnitude_ + active_max_accel * elapsed_seconds);
    }
  } else if (requested_magnitude + kCommandEpsilon < linear_speed_limit_magnitude_) {
    // 0表示关闭减速侧限制，弯道降速等请求立即生效。
    if (global_max_decel_ <= 0.0) {
      linear_speed_limit_magnitude_ = requested_magnitude;
    } else {
      linear_speed_limit_magnitude_ = std::max(
        requested_magnitude,
        linear_speed_limit_magnitude_ - global_max_decel_ * elapsed_seconds);
    }
  } else {
    linear_speed_limit_magnitude_ = requested_magnitude;
  }
  if (event_start_accel_active_ &&
    linear_speed_limit_magnitude_ + kCommandEpsilon >= requested_magnitude)
  {
    event_start_accel_active_ = false;
  }
  cmd.linear.x = requested_direction * linear_speed_limit_magnitude_;
}

// 全部速度命令最终都会经过这里。只要本节点确实发布过运动命令，随后首次
// 收到零线速度请求，就使用基地停车的同一组参数执行一次反相制动。制动按
// 控制周期非阻塞推进；反相段用启动后收到的新/chassis/odom轮速闭环：
// 轮速接近零或首次越过零点就立刻结束反相，并保证序列最后一帧为零速。
// 前进停车输出负速度；倒车停车输出正速度。制动过程中任何新运动命令都要
// 等到反相段和零速保持段完成，防止下一阶段在制动中途覆盖停车命令。
bool VisualLidarPathFollower::apply_unified_stop_brake(
  geometry_msgs::msg::Twist & cmd)
{
  constexpr double kCommandEpsilon = 1.0e-6;
  const bool stop_requested =
    !std::isfinite(cmd.linear.x) ||
    std::abs(cmd.linear.x) <= kCommandEpsilon;
  if (!stop_brake_sequence_active_ && !stop_requested) {
    return false;
  }

  // 只保留调用方本周期明确计算或锁存的转向量。publish_stop()传入0，
  // 仍会让舵机回中；图生文和二维码临停传入非零值时，反相制动与后续
  // 零线速度保持都不再擅自清空angular.z。
  const double requested_angular_command = std::isfinite(cmd.angular.z) ?
    cmd.angular.z : 0.0;
  cmd = geometry_msgs::msg::Twist();
  cmd.angular.z = requested_angular_command;
  if (!stop_brake_sequence_active_) {
    double motion_linear_command = last_published_motion_linear_command_;
    if (latest_chassis_linear_speed_valid_ &&
      std::isfinite(latest_chassis_linear_speed_) &&
      std::abs(latest_chassis_linear_speed_) > start_speed_threshold_)
    {
      // 实测轮速优先，可正确处理条件倒车结束后的正向制动脉冲。
      motion_linear_command = latest_chassis_linear_speed_;
    }

    const bool can_reverse_brake = stop_brake_armed_ &&
      std::isfinite(motion_linear_command) &&
      std::abs(motion_linear_command) > kCommandEpsilon &&
      return_brake_reverse_speed_ > kCommandEpsilon &&
      temporary_stop_brake_reverse_cycles_ > 0;
    stop_brake_armed_ = false;
    last_published_motion_linear_command_ = 0.0;
    if (!can_reverse_brake) {
      return true;
    }

    stop_brake_sequence_active_ = true;
    // 参数表示本次中途制动序列的总周期上限；最后1周期强制留给零速，
    // 因此配置5时最多发4周期反相速度，再发1周期零速。
    stop_brake_reverse_cycles_remaining_ = std::max(
      0, temporary_stop_brake_reverse_cycles_ - 1);
    stop_brake_zero_cycles_remaining_ = 1;
    stop_brake_linear_command_ = -std::copysign(
      return_brake_reverse_speed_, motion_linear_command);
    stop_brake_motion_direction_ = motion_linear_command > 0.0 ? 1 : -1;
    stop_brake_start_odom_sequence_ = chassis_odom_sequence_;
    reset_path_pid_state();
    reset_linear_speed_limit();
    RCLCPP_INFO(
      get_logger(),
      "统一停车反相制动开始：停车前方向=%s，反相速度=%+.2fm/s，"
      "总上限=%d周期（最多%d周期反相+最终1周期零速），"
      "新轮速|v|<=%.3fm/s或首次反向即提前结束反相，控制频率=%.1fHz。",
      motion_linear_command > 0.0 ? "前进" : "倒车",
      stop_brake_linear_command_, temporary_stop_brake_reverse_cycles_,
      std::max(0, temporary_stop_brake_reverse_cycles_ - 1),
      temporary_stop_brake_release_speed_threshold_,
      control_frequency_);
  }

  if (stop_brake_reverse_cycles_remaining_ > 0 &&
    chassis_odom_sequence_ > stop_brake_start_odom_sequence_ &&
    latest_chassis_linear_speed_valid_ &&
    std::isfinite(latest_chassis_linear_speed_))
  {
    const bool near_zero = std::abs(latest_chassis_linear_speed_) <=
      temporary_stop_brake_release_speed_threshold_;
    const bool crossed_zero =
      (stop_brake_motion_direction_ > 0 && latest_chassis_linear_speed_ < 0.0) ||
      (stop_brake_motion_direction_ < 0 && latest_chassis_linear_speed_ > 0.0);
    if (near_zero || crossed_zero) {
      stop_brake_reverse_cycles_remaining_ = 0;
      RCLCPP_INFO(
        get_logger(),
        "统一停车反相制动轮速解除：新鲜/chassis/odom v=%+.3fm/s，"
        "原因=%s；本周期立即发送最终零速。",
        latest_chassis_linear_speed_,
        crossed_zero ? "轮速首次反向" : "轮速接近零");
    }
  }

  if (stop_brake_reverse_cycles_remaining_ > 0) {
    cmd.linear.x = stop_brake_linear_command_;
    --stop_brake_reverse_cycles_remaining_;
  } else if (stop_brake_zero_cycles_remaining_ > 0) {
    --stop_brake_zero_cycles_remaining_;
  }

  if (stop_brake_reverse_cycles_remaining_ == 0 &&
    stop_brake_zero_cycles_remaining_ == 0)
  {
    stop_brake_sequence_active_ = false;
    stop_brake_linear_command_ = 0.0;
    stop_brake_motion_direction_ = 0;
    stop_brake_start_odom_sequence_ = chassis_odom_sequence_;
    RCLCPP_INFO(get_logger(), "统一停车反相制动完成，当前保持零速度。");
  }
  return true;
}

void VisualLidarPathFollower::publish_velocity_command(
  geometry_msgs::msg::Twist cmd)
{
  if (!cmd_pub_) {
    return;
  }
  if (apply_unified_stop_brake(cmd)) {
    reset_linear_speed_limit();
    cmd_pub_->publish(cmd);
    last_published_angular_command_ = std::isfinite(cmd.angular.z) ?
      cmd.angular.z : 0.0;
    return;
  }
  // 放在所有巡线、雷达、二维码和任务控制之后，统一限制最终线速度的
  // 上升与下降斜率，不改变angular.z。零速和反相制动已由上面立即处理。
  apply_global_linear_speed_limit(cmd);
  cmd_pub_->publish(cmd);
  last_published_angular_command_ = std::isfinite(cmd.angular.z) ?
    cmd.angular.z : 0.0;
  if (std::isfinite(cmd.linear.x) && std::abs(cmd.linear.x) > 1.0e-6) {
    last_published_motion_linear_command_ = cmd.linear.x;
    stop_brake_armed_ = true;
  }
}
