#include "origincar_system/visual_lidar_path_follower.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <string>
#include <vector>

#include "follower_utils.hpp"

using origincar_system::to_lower;
using origincar_system::route_direction_name;
using origincar_system::normalize_angle;

const char * VisualLidarPathFollower::task_phase_name(
  VisualLidarPathFollower::TaskPhase phase) const
{
  switch (phase) {
    case TaskPhase::TASK1:
      return "TASK1";
    case TaskPhase::TRANSITION:
      return "TASK1_TO_TASK2_TRANSITION";
    case TaskPhase::TASK2:
      return "TASK2";
    case TaskPhase::TASK3:
      return "TASK3";
    case TaskPhase::FINISHED:
      return "FINISHED";
    case TaskPhase::UNKNOWN:
    default:
      return "UNKNOWN";
  }
}

void VisualLidarPathFollower::apply_stage_profile(
  const VisualLidarPathFollower::StageControlProfile & profile)
{
  linear_speed_ = profile.linear_speed;
  turn_angular_gain_ = profile.turn_angular_gain;
  enable_turn_speed_slowdown_ = profile.enable_turn_speed_slowdown;
  max_turn_speed_reduction_percent_ =
    profile.max_turn_speed_reduction_percent;
  pass_radius_ = profile.pass_radius;
  route_projection_max_distance_ =
    profile.route_projection_max_distance;
  segment_advance_t_ = profile.segment_advance_t;
  channel_lidar_avoid_distance_ = profile.lidar_avoid_distance;
  channel_lidar_avoid_fov_deg_ = profile.lidar_avoid_fov_deg;
  channel_lidar_corridor_half_width_ =
    profile.lidar_corridor_half_width;
  channel_lidar_avoid_linear_speed_ =
    profile.lidar_avoid_linear_speed;
  channel_lidar_avoid_angular_gain_ =
    profile.lidar_avoid_angular_gain;
  channel_lidar_direction_upper_angle_deg_ =
    profile.lidar_direction_upper_angle_deg;
  channel_lidar_direction_lower_angle_deg_ =
    profile.lidar_direction_lower_angle_deg;
}

void VisualLidarPathFollower::set_task_phase(
  VisualLidarPathFollower::TaskPhase phase)
{
  if (task_phase_ == phase) {
    return;
  }
  const TaskPhase previous_phase = task_phase_;
  task_phase_ = phase;
  if (phase != previous_phase) {
    switch (phase) {
      case TaskPhase::TASK1:
        task1_lidar_avoid_count_ = 0;
        task1_lidar_last_avoid_direction_ = 0;
        break;
      case TaskPhase::TRANSITION:
        transition_lidar_avoid_count_ = 0;
        transition_lidar_last_avoid_direction_ = 0;
        break;
      case TaskPhase::TASK2:
        task2_lidar_avoid_count_ = 0;
        task2_lidar_last_avoid_direction_ = 0;
        break;
      case TaskPhase::TASK3:
        task3_lidar_avoid_count_ = 0;
        task3_lidar_last_avoid_direction_ = 0;
        task3_return_slowdown_active_ = false;
        task3_return_slowdown_speed_limit_ =
          std::max(0.0, task3_profile_.linear_speed);
        break;
      case TaskPhase::FINISHED:
      case TaskPhase::UNKNOWN:
        break;
    }
  }
  switch (phase) {
    case TaskPhase::TASK1:
      apply_stage_profile(task1_profile_);
      break;
    case TaskPhase::TRANSITION:
      apply_stage_profile(transition_profile_);
      break;
    case TaskPhase::TASK2:
      apply_stage_profile(task2_profile_);
      break;
    case TaskPhase::TASK3:
      apply_stage_profile(task3_profile_);
      break;
    case TaskPhase::FINISHED:
    case TaskPhase::UNKNOWN:
      break;
  }
  if (logic3_planning_pose_anchored_ && previous_phase != TaskPhase::UNKNOWN) {
    // Task1和过渡阶段使用不同的点数/连续帧标准。阶段切换时只丢弃尚在
    // 累积的短期观测轨迹，避免上一阶段的帧数带入下一阶段凑成确认；
    // 已经写入logic3_green_clusters_的绿/橙点不会因阶段切换、短期失联
    // 或后来与其他永久点进入覆盖半径而删除。
    logic3_runtime_green_tracks_.clear();
  }
  // 输入重新有效后会清除此标志；阶段切换也视为进入一轮新的等待状态，
  // 因而允许新阶段最多再报告一次对应的“等待输入”信息。
  qr_reverse_waiting_for_inputs_ = false;
  channel_lidar_waiting_for_inputs_ = false;
  // 普通任务阶段切换不能截断正在进行的同一次雷达避障。特别是车辆在B点
  // 附近由TRANSITION切入Task2时，继续保持既定绕行方向，直到首次锁定
  // 目标真正离开检测区。只有流程结束或回到未知状态才强制清除动作。
  if (phase == TaskPhase::FINISHED || phase == TaskPhase::UNKNOWN) {
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
  }
  reset_path_pid_state();
  RCLCPP_INFO(
    get_logger(), "当前任务状态：%s。", task_phase_name(task_phase_));
  if (logic3_planning_pose_anchored_ && logic3_green_route_replan_pending_ &&
    (task_phase_ != TaskPhase::TRANSITION ||
    tracking_controller_mode_ == "smac_pid") &&
    task_phase_ != TaskPhase::UNKNOWN &&
    task_phase_ != TaskPhase::FINISHED)
  {
    // 过渡段高分观测只维护全局绿点库；一进入Task2就立刻把期间新增或
    // 修正的绿点应用到Task2路线，而不必等待该障碍再次被看到。
    rebuild_logic3_route_from_green_clusters("阶段切换应用待处理全局绿点");
  }
}

bool VisualLidarPathFollower::stop_if_route_finished(
  const VisualLidarPathFollower::RobotPose & pose)
{
  if (waypoints_.empty() || !first_waypoint_reached_ ||
    current_index_ < waypoints_.size() - 1)
  {
    return false;
  }

  if (following_qr_path_) {
    if (!task1_endpoint_waiting_) {
      task1_endpoint_waiting_ = true;
      task1_endpoint_wait_timer_started_ = true;
      task1_endpoint_wait_started_at_ = std::chrono::steady_clock::now();
      qr_reverse_hold_angular_command_ =
        std::isfinite(last_published_angular_command_) ?
        last_published_angular_command_ : 0.0;
      RCLCPP_INFO(
        get_logger(),
        "Task1已到达最后一个waypoint：直接零速停车，不执行反相制动；"
        "等待期间保留转向angular.z=%.3f，等待二维码%.2f秒。",
        qr_reverse_hold_angular_command_, task1_endpoint_qr_wait_seconds_);
    }
    qr_path_finished_ = true;
    set_control_state(ControlState::WAITING_FOR_QR);
    publish_translation_stop_without_reverse_brake(qr_reverse_hold_angular_command_);
    return true;
  }

  // Task3原路线末点是真实规划点，不再被虚拟停车点替换。末点被专用
  // 精准半径约束，只有真正到达且仍未进入紫色区时才激活路线外保底点。
  if (return_to_start_enabled_ && task3_route_available_) {
    start_task3_terminal_homing(pose);
    control_task3_terminal_homing(pose);
    return true;
  }

  if (return_to_start_enabled_) {
    start_return_to_start();
    return true;
  }

  set_control_state(ControlState::FINISHED);
  if (logic3_planning_pose_anchored_ &&
    !logic3_final_snapshot_published_)
  {
    record_logic3_actual_route(pose, true);
    logic3_final_pose_ = pose;
    logic3_final_pose_valid_ = std::isfinite(pose.x) &&
      std::isfinite(pose.y) && std::isfinite(pose.yaw);
  }
  if (resume_visual_control_on_finish_) {
    RCLCPP_INFO(
      get_logger(), "环线追点完成，执行统一反相制动后恢复visual_control接管。");
    perform_return_brake_sequence();
    resume_visual_control();
    publish_stop_on_exit_ = false;
  } else {
    RCLCPP_INFO(get_logger(), "路线追点完成，执行统一反相制动。");
    perform_return_brake_sequence();
  }
  // 无Task3标记的旧路线同样先完成底盘反相制动，再序列化最终快照。
  if (logic3_planning_pose_anchored_ &&
    !logic3_final_snapshot_published_)
  {
    publish_logic3_snapshot("final_dynamic");
    logic3_final_snapshot_published_ = true;
  }
  set_task_phase(TaskPhase::FINISHED);
  rclcpp::shutdown();
  return true;
}

bool VisualLidarPathFollower::handle_task3_start(
  const VisualLidarPathFollower::RobotPose & pose)
{
  if (!return_to_start_enabled_ || !task3_route_available_ ||
    task3_start_index_ >= waypoints_.size() ||
    current_index_ < task3_start_index_)
  {
    return false;
  }

  if (!task3_route_active_) {
    // current_index_ 会被离散前瞻提前提交，不能只凭索引就认定车已
    // 进入返程线。再用返程首段的纵向投影限制，避免在0.65m前瞻距离处
    // 过早切入Task3停车判定。
    if (task3_start_index_ + 1 < waypoints_.size()) {
      const auto & start = waypoints_[task3_start_index_];
      const auto & next = waypoints_[task3_start_index_ + 1];
      const double segment_x = next.x - start.x;
      const double segment_y = next.y - start.y;
      const double segment_length = std::hypot(segment_x, segment_y);
      if (segment_length > 1.0e-8) {
        const double relative_x = pose.x - start.x;
        const double relative_y = pose.y - start.y;
        const double along_route =
          (relative_x * segment_x + relative_y * segment_y) / segment_length;
        const double cross_track = std::abs(
          -relative_x * segment_y + relative_y * segment_x) / segment_length;
        if (along_route < -0.05 || cross_track > route_projection_max_distance_) {
          return false;
        }
      }
    }

    task3_route_active_ = true;
    return_stop_zone_entry_active_ = false;
    set_task_phase(TaskPhase::TASK3);
    RCLCPP_INFO(
      get_logger(),
      "进入Task3（起始航点=%zu/%zu）；继续使用定位路线、静态偏移和"
      "实时雷达避障，保留原路线全部航点；只有原末点真正循完且仍未停车，"
      "才启用路线外保底点(%.2f,%.2f)。base_link进入有限紫色曲边停车区"
      "后立即停车。障碍簇中心Y>%.2f走完整静态偏移，Y<=%.2f走实时雷达。",
      task3_start_index_ + 1, waypoints_.size(),
      task3_terminal_target_x_, task3_terminal_target_y_,
      return_obstacle_split_y_, return_obstacle_split_y_);
    return false;
  }

  // Task3停车只看map定位；无论当前是跟线还是实时绕障，都独立检查
  // 有限紫色曲边停车区，base_link一进入便在当前控制周期停车。
  if (return_stop_zone_confirmed(pose)) {
    finish_return_to_start(pose, "base_link中心进入有限紫色曲边停车区");
    return true;
  }
  return false;
}

void VisualLidarPathFollower::start_task3_terminal_homing(
  const VisualLidarPathFollower::RobotPose & pose)
{
  if (task3_terminal_homing_active_) {
    return;
  }
  task3_terminal_homing_active_ = true;
  task3_terminal_homing_used_ = true;
  reset_path_pid_state();
  RCLCPP_WARN(
    get_logger(),
    "Task3原路线全部航点已循完，base_link仍未进入紫色停车区；"
    "激活不属于waypoints且不参与静态规划的保底点(%.2f, %.2f)。"
    "当前base_link=(%.3f, %.3f)。",
    task3_terminal_target_x_, task3_terminal_target_y_, pose.x, pose.y);
}

void VisualLidarPathFollower::control_task3_terminal_homing(
  const VisualLidarPathFollower::RobotPose & pose)
{
  set_control_state(ControlState::RETURNING_TO_START);

  // 紫色曲边停车区始终高于保底追点和实时雷达；进入当周期立即终停。
  if (return_stop_zone_confirmed(pose)) {
    finish_return_to_start(pose, "路线外保底追点期间base_link中心进入紫色曲边停车区");
    return;
  }

  TargetPoint target;
  target.point = Point2D{task3_terminal_target_x_, task3_terminal_target_y_};
  target.waypoint_index = waypoints_.size();
  target.reverse = false;
  geometry_msgs::msg::Twist cmd = compute_tracking_command(pose, target);
  apply_turn_speed_limit(cmd);

  // 保底点不进入waypoints_，因此没有静态偏移或路径重规划；但Task3原有
  // 实时雷达安全接管仍继续运行，红点记录/次数和方向策略保持原参数。
  if (apply_channel_lidar_obstacle_avoidance(cmd)) {
    reset_path_pid_state();
  }
  apply_task3_return_slowdown(pose, cmd);

  publish_velocity_command(cmd);
  if (task1_dynamic_snapshot_pending_) {
    task1_dynamic_snapshot_pending_ = false;
    publish_logic3_snapshot("task1_dynamic");
  }
  if (!pending_control_snapshot_event_.empty()) {
    const std::string snapshot_event = pending_control_snapshot_event_;
    pending_control_snapshot_event_.clear();
    publish_logic3_snapshot(snapshot_event);
  }
  publish_debug_visualization(pose);
}

void VisualLidarPathFollower::start_return_to_start()
{
  if (return_to_start_active_) {
    return;
  }

  return_to_start_active_ = true;
  return_stop_zone_entry_active_ = false;
  latest_return_visual_targets_.reset();
  latest_return_visual_received_at_ = std::chrono::steady_clock::time_point();
  set_task_phase(TaskPhase::TASK3);
  set_control_state(ControlState::RETURNING_TO_START);

  // YOLO只用于Task1二维码识别，离开Task1后已经退出并释放BPU；
  // 旧版返程也不得重新启动视觉模型。
  publish_stop();
  RCLCPP_INFO(
    get_logger(),
    "channel巡点完成，开始返回虚拟牵引点(%.2f, %.2f)；"
    "base_link进入有限紫色曲边停车区后立即停车。",
    task3_terminal_target_x_, task3_terminal_target_y_);
}

void VisualLidarPathFollower::control_return_to_start(
  const VisualLidarPathFollower::RobotPose & pose)
{
  set_control_state(ControlState::RETURNING_TO_START);

  // 有限曲边停车区是唯一停车判据，且不能被同帧避障覆盖。
  if (return_stop_zone_confirmed(pose)) {
    finish_return_to_start(pose, "base_link中心进入有限紫色曲边停车区");
    return;
  }

  // 旧版返程不允许重新启动YOLO；没有外部新鲜检测时保持停车。
  if (!return_visual_detection_is_fresh()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "返回起点等待新鲜视觉检测：topic=%s，超时=%.2fs，当前停车。",
      return_visual_detection_topic_.c_str(), return_visual_detection_timeout_);
    publish_stop();
    return;
  }

  const double target_yaw = std::atan2(
    task3_terminal_target_y_ - pose.y, task3_terminal_target_x_ - pose.x);
  const double heading_error = normalize_angle(target_yaw - pose.yaw);
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = linear_speed_;
  cmd.angular.z = turn_angular_gain_ * heading_error;
  apply_turn_speed_limit(cmd);

  if (const auto * obstacle = find_return_visual_avoidance_obstacle()) {
    double center_offset = static_cast<double>(
      return_visual_target_center_x(*obstacle)) -
      static_cast<double>(return_visual_control_width_) * 0.5;
    if (center_offset >= 0.0 && center_offset < 5.0) {
      center_offset = 5.0;
    } else if (center_offset < 0.0 && center_offset > -5.0) {
      center_offset = -5.0;
    }

    const double half_width = static_cast<double>(return_visual_control_width_) * 0.5;
    if (center_offset > 0.0) {
      cmd.angular.z = -return_visual_avoid_angular_ratio_ *
        (half_width - center_offset);
    } else {
      cmd.angular.z = -return_visual_avoid_angular_ratio_ *
        (-half_width - center_offset);
    }
    cmd.linear.x = return_visual_avoid_linear_speed_;
  } else if (return_visual_obstacle_in_caution_zone()) {
    cmd.linear.x = std::min(
      std::abs(cmd.linear.x), return_visual_avoid_linear_speed_);
  }

  apply_task3_return_slowdown(pose, cmd);

  publish_velocity_command(cmd);
  publish_debug_visualization(pose);
}

void VisualLidarPathFollower::apply_task3_return_slowdown(
  const VisualLidarPathFollower::RobotPose & pose,
  geometry_msgs::msg::Twist & cmd)
{
  if (!task3_return_slowdown_enabled_ || task_phase_ != TaskPhase::TASK3) {
    return;
  }

  const double distance_to_center = std::hypot(
    pose.x - task3_terminal_target_x_,
    pose.y - task3_terminal_target_y_);
  if (!task3_return_slowdown_active_) {
    if (distance_to_center > task3_return_slowdown_radius_) {
      return;
    }
    // 首次进入圆内后锁定；之后即使定位抖动或绕障短时离开圆，也不取消
    // 减速状态，更不会恢复到已经经过的更高速度上限。
    task3_return_slowdown_active_ = true;
    task3_return_slowdown_speed_limit_ =
      std::max(0.0, task3_profile_.linear_speed);
    RCLCPP_INFO(
      get_logger(),
      "进入Task3圆形减速区：圆心=(%.2f, %.2f)，半径=%.2fm，"
      "当前径向距离=%.3fm；速度上限开始从%.2f向%.2fm/s线性降低。",
      task3_terminal_target_x_, task3_terminal_target_y_,
      task3_return_slowdown_radius_, distance_to_center,
      task3_profile_.linear_speed, task3_return_slowdown_min_speed_);
  }

  if (cmd.linear.x <= 0.0) {
    // 停车或倒车指令保持原样，但进入区域的锁定状态已经记录。
    return;
  }

  const double start_speed = std::max(0.0, task3_profile_.linear_speed);
  const double minimum_speed = std::min(
    start_speed, task3_return_slowdown_min_speed_);
  const double progress = std::clamp(
    (task3_return_slowdown_radius_ - distance_to_center) /
    task3_return_slowdown_radius_,
    0.0, 1.0);
  const double position_speed_limit = start_speed +
    (minimum_speed - start_speed) * progress;
  task3_return_slowdown_speed_limit_ = std::min(
    task3_return_slowdown_speed_limit_, position_speed_limit);
  // 只做上限，绝不把转弯降速、雷达降速或停车产生的更低速度抬到0.50。
  cmd.linear.x = std::min(cmd.linear.x, task3_return_slowdown_speed_limit_);
}

void VisualLidarPathFollower::finish_return_to_start(
  const VisualLidarPathFollower::RobotPose & pose, const char * reason)
{
  task3_route_active_ = false;
  task3_terminal_homing_active_ = false;
  return_to_start_active_ = false;
  return_stop_zone_entry_active_ = false;
  set_control_state(ControlState::FINISHED);
  if (logic3_planning_pose_anchored_ &&
    !logic3_final_snapshot_published_)
  {
    record_logic3_actual_route(pose, true);
    logic3_final_pose_ = pose;
    logic3_final_pose_valid_ = std::isfinite(pose.x) &&
      std::isfinite(pose.y) && std::isfinite(pose.yaw);
  }
  set_task_phase(TaskPhase::FINISHED);
  RCLCPP_INFO(
    get_logger(),
    "到达map起终点：%s；最终base_link map坐标=(%.3f, %.3f)，yaw=%.3frad，执行反向制动。",
    reason, pose.x, pose.y, pose.yaw);
  perform_return_brake_sequence();
  // 最终PNG所需状态已经在上面锁存；先完成基地反向制动和零速保持，
  // 再做JSON序列化/可靠话题发布，磁盘绘图绝不能延迟停车。
  if (logic3_planning_pose_anchored_ &&
    !logic3_final_snapshot_published_)
  {
    publish_logic3_snapshot("final_dynamic");
    logic3_final_snapshot_published_ = true;
  }
  rclcpp::shutdown();
}

bool VisualLidarPathFollower::return_stop_zone_contains(
  const VisualLidarPathFollower::RobotPose & pose) const
{
  if (pose.y < return_stop_zone_min_y_ ||
    pose.y > return_stop_zone_max_y_ ||
    pose.x < return_stop_zone_min_x_ ||
    pose.x > return_stop_zone_max_x_)
  {
    return false;
  }
  // 直线段依次连接；左上到左下使用下偏二次Bezier。
  // 将曲线细分为24段后使用通用奇偶射线法，既支持曲边，也不会把
  // 曲线向停车区内部凹入后误当成凸多边形处理。
  std::vector<Point2D> boundary{
    Point2D{return_stop_zone_min_x_, return_stop_zone_min_y_},
    Point2D{return_stop_zone_max_x_, return_stop_zone_min_y_},
    Point2D{return_stop_zone_max_x_, return_stop_zone_right_top_y_},
    Point2D{return_stop_zone_top_max_x_, return_stop_zone_max_y_},
    Point2D{return_stop_zone_top_min_x_, return_stop_zone_max_y_}};
  constexpr std::size_t curve_segment_count = 24;
  const Point2D curve_start{
    return_stop_zone_top_min_x_, return_stop_zone_max_y_};
  const Point2D curve_control{
    return_stop_zone_left_curve_control_x_,
    return_stop_zone_left_curve_control_y_};
  const Point2D curve_finish{
    return_stop_zone_min_x_, return_stop_zone_min_y_};
  boundary.reserve(boundary.size() + curve_segment_count - 1);
  for (std::size_t step = 1; step < curve_segment_count; ++step) {
    const double t = static_cast<double>(step) /
      static_cast<double>(curve_segment_count);
    const double one_minus_t = 1.0 - t;
    boundary.push_back(Point2D{
      one_minus_t * one_minus_t * curve_start.x +
      2.0 * one_minus_t * t * curve_control.x +
      t * t * curve_finish.x,
      one_minus_t * one_minus_t * curve_start.y +
      2.0 * one_minus_t * t * curve_control.y +
      t * t * curve_finish.y});
  }

  bool inside = false;
  for (std::size_t current = 0, previous = boundary.size() - 1;
    current < boundary.size(); previous = current++)
  {
    const auto & first = boundary[previous];
    const auto & second = boundary[current];
    const double segment_x = second.x - first.x;
    const double segment_y = second.y - first.y;
    const double relative_x = pose.x - first.x;
    const double relative_y = pose.y - first.y;
    const double cross = segment_x * relative_y - segment_y * relative_x;
    const double dot = relative_x * segment_x + relative_y * segment_y;
    const double segment_length_squared =
      segment_x * segment_x + segment_y * segment_y;
    if (std::abs(cross) <= 1.0e-9 && dot >= -1.0e-9 &&
      dot <= segment_length_squared + 1.0e-9)
    {
      return true;
    }
    const bool crosses_pose_y =
      (first.y > pose.y) != (second.y > pose.y);
    if (crosses_pose_y) {
      const double intersection_x = first.x +
        (pose.y - first.y) * segment_x / (second.y - first.y);
      if (pose.x < intersection_x) {
        inside = !inside;
      }
    }
  }
  return inside;
}

bool VisualLidarPathFollower::return_stop_zone_confirmed(
  const VisualLidarPathFollower::RobotPose & pose)
{
  if (!return_stop_zone_contains(pose)) {
    return_stop_zone_entry_active_ = false;
    return false;
  }
  const auto steady_now = std::chrono::steady_clock::now();
  if (!return_stop_zone_entry_active_) {
    return_stop_zone_entry_active_ = true;
    return_stop_zone_entered_at_ = steady_now;
    return return_stop_confirm_seconds_ <= 0.0;
  }
  return std::chrono::duration<double>(
    steady_now - return_stop_zone_entered_at_).count() >=
         return_stop_confirm_seconds_;
}

bool VisualLidarPathFollower::return_visual_detection_is_fresh() const
{
  if (!latest_return_visual_targets_) {
    return false;
  }
  const double age = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - latest_return_visual_received_at_).count();
  return age <= return_visual_detection_timeout_;
}

int VisualLidarPathFollower::scale_return_visual_x(int x) const
{
  return static_cast<int>(std::round(
    static_cast<double>(x) * static_cast<double>(return_visual_control_width_) /
    static_cast<double>(return_visual_source_width_)));
}

int VisualLidarPathFollower::scale_return_visual_y(int y) const
{
  return static_cast<int>(std::round(
    static_cast<double>(y) * static_cast<double>(return_visual_control_height_) /
    static_cast<double>(return_visual_source_height_)));
}

int VisualLidarPathFollower::return_visual_target_height(
  const ai_msgs::msg::Target & target) const
{
  return scale_return_visual_y(
    static_cast<int>(target.rois[0].rect.height));
}

int VisualLidarPathFollower::return_visual_target_center_x(
  const ai_msgs::msg::Target & target) const
{
  const auto & rect = target.rois[0].rect;
  return scale_return_visual_x(static_cast<int>(rect.x_offset + rect.width / 2));
}

int VisualLidarPathFollower::return_visual_target_center_y(
  const ai_msgs::msg::Target & target) const
{
  const auto & rect = target.rois[0].rect;
  return scale_return_visual_y(static_cast<int>(rect.y_offset + rect.height / 2));
}

int VisualLidarPathFollower::return_visual_target_bottom_y(
  const ai_msgs::msg::Target & target) const
{
  const auto & rect = target.rois[0].rect;
  return scale_return_visual_y(static_cast<int>(rect.y_offset + rect.height));
}

const ai_msgs::msg::Target *
VisualLidarPathFollower::find_return_visual_avoidance_obstacle() const
{
  if (!latest_return_visual_targets_) {
    return nullptr;
  }
  const double center_x = static_cast<double>(return_visual_control_width_) * 0.5;
  for (const auto & target : latest_return_visual_targets_->targets) {
    if (target.type != "construction_cone" || target.rois.empty() ||
      target.rois[0].rect.width == 0)
    {
      continue;
    }
    const int bottom = return_visual_target_bottom_y(target);
    const int delta_x = static_cast<int>(std::abs(
      static_cast<double>(return_visual_target_center_x(target)) - center_x));
    const double threshold = static_cast<double>(return_visual_bottom_threshold_avoid_) +
      static_cast<double>(delta_x) * return_visual_delta_x_ratio_;
    if (target.rois[0].confidence >= return_visual_obstacle_confidence_threshold_ &&
      static_cast<double>(bottom) >= threshold)
    {
      return &target;
    }
  }
  return nullptr;
}

bool VisualLidarPathFollower::return_visual_obstacle_in_caution_zone() const
{
  if (!latest_return_visual_targets_) {
    return false;
  }
  const double center_x = static_cast<double>(return_visual_control_width_) * 0.5;
  for (const auto & target : latest_return_visual_targets_->targets) {
    if (target.type != "construction_cone" || target.rois.empty()) {
      continue;
    }
    const int bottom = return_visual_target_bottom_y(target);
    const int delta_x = static_cast<int>(std::abs(
      static_cast<double>(return_visual_target_center_x(target)) - center_x));
    const double threshold = static_cast<double>(return_visual_bottom_threshold_caution_) +
      static_cast<double>(delta_x) * return_visual_delta_x_ratio_;
    if (target.rois[0].confidence >= return_visual_obstacle_confidence_threshold_ &&
      static_cast<double>(bottom) >= threshold)
    {
      return true;
    }
  }
  return false;
}

bool VisualLidarPathFollower::return_visual_parking_condition_met() const
{
  if (!latest_return_visual_targets_) {
    return false;
  }
  for (const auto & target : latest_return_visual_targets_->targets) {
    if (target.type == "parking_sign" && !target.rois.empty() &&
      target.rois[0].confidence >=
      return_visual_parking_sign_confidence_threshold_)
    {
      // 与visual_control一致：第一个满足置信度的停车标志达到中心Y阈值即停车；
      // 这里只判停，绝不把标志位置用作转向目标。
      return return_visual_target_center_y(target) >=
             return_visual_parking_y_threshold_;
    }
  }
  return false;
}

void VisualLidarPathFollower::reset_image_capture_corridor()
{
  image_capture_corridor_state_ = ImageCaptureCorridorState::WAITING;
  image_capture_candidate_selected_ = false;
  image_capture_stop_timer_started_ = false;
  image_capture_stop_started_at_ = std::chrono::steady_clock::time_point{};
  image_capture_route_guard_active_ = false;
  image_capture_route_anchor_valid_ = false;
  image_capture_route_resume_reprojected_ = false;
  image_capture_route_anchor_ = RouteProjection();
  image_capture_stop_point_valid_ = false;
  image_capture_stop_point_ = Point2D();
  image_capture_stop_point_yaw_ = 0.0;
  image_capture_point_valid_ = false;
  image_capture_point_ = Point2D();
  image_capture_point_yaw_ = 0.0;
}

void VisualLidarPathFollower::begin_image_capture_stop(
  const VisualLidarPathFollower::RobotPose & pose, const char * reason)
{
  // 本周期末的统一速度出口会先执行反相制动。静止拍照计时必须等制动
  // 完整结束后才开始，不能把反向脉冲误计入“已停车”时长。
  image_capture_stop_timer_started_ = false;
  image_capture_stop_started_at_ = std::chrono::steady_clock::time_point{};
  // 此处记录的是决定停车并首次禁止平移前的map位姿。拍照计时结束时会
  // 另记一份位姿，两者差值用于观察急停期间的定位漂移。
  image_capture_stop_point_ = Point2D{pose.x, pose.y};
  image_capture_stop_point_yaw_ = pose.yaw;
  image_capture_stop_point_valid_ = true;
  image_capture_route_guard_active_ = false;
  image_capture_route_anchor_valid_ = false;
  image_capture_route_resume_reprojected_ = false;
  image_capture_route_anchor_ = RouteProjection();
  if (!image_capture_route_progress_guard_enabled_) {
    return;
  }

  RouteProjection projection;
  if (!find_route_projection(pose, projection)) {
    RCLCPP_WARN(
      get_logger(), "图生文停车无法锁定路线投影锚点，停车仍继续：%s。", reason);
    return;
  }
  image_capture_route_anchor_ = projection;
  image_capture_route_anchor_valid_ = true;
  image_capture_route_guard_active_ = true;
  RCLCPP_INFO(
    get_logger(),
    "图生文停车锁定路线投影：segment=%zu，t=%.3f，point=(%.3f, %.3f)，"
    "起步重投影范围=[后%.2fm, 前%.2fm]，原因=%s。",
    projection.segment_index, std::clamp(projection.t, 0.0, 1.0),
    projection.point.x, projection.point.y,
    image_capture_reprojection_backward_distance_,
    image_capture_reprojection_forward_distance_, reason);
}

void VisualLidarPathFollower::reproject_image_capture_route_on_resume(
  const VisualLidarPathFollower::RobotPose & pose)
{
  if (!image_capture_route_guard_active_ ||
    !image_capture_route_anchor_valid_ ||
    image_capture_route_resume_reprojected_)
  {
    return;
  }
  RouteProjection projection;
  if (!find_route_projection(pose, projection)) {
    RCLCPP_WARN(
      get_logger(), "图生文重新起步受限重投影失败，继续使用停车前路线索引%zu。",
      current_index_);
  } else {
    RCLCPP_INFO(
      get_logger(),
      "图生文重新起步已受限重投影：pose=(%.3f, %.3f)，"
      "segment=%zu，t=%.3f，projection=(%.3f, %.3f)，distance=%.3fm。",
      pose.x, pose.y, projection.segment_index,
      std::clamp(projection.t, 0.0, 1.0),
      projection.point.x, projection.point.y, projection.distance);
  }
  image_capture_route_resume_reprojected_ = true;
}

void VisualLidarPathFollower::maybe_release_image_capture_route_guard()
{
  if (!image_capture_route_guard_active_ ||
    !image_capture_route_resume_reprojected_ ||
    image_capture_corridor_state_ == ImageCaptureCorridorState::STOPPING)
  {
    return;
  }
  const bool chassis_started = latest_chassis_linear_speed_valid_ &&
    std::abs(latest_chassis_linear_speed_) > start_speed_threshold_;
  if (!chassis_started && latest_chassis_linear_speed_valid_) {
    return;
  }
  if (!latest_chassis_linear_speed_valid_) {
    RCLCPP_WARN(
      get_logger(),
      "图生文起步没有有效底盘轮速，已在完成一次受限重投影后解除路线保护。");
  } else {
    RCLCPP_INFO(
      get_logger(), "图生文检测到实际起步轮速%.3fm/s，解除路线投影保护。",
      latest_chassis_linear_speed_);
  }
  image_capture_route_guard_active_ = false;
  image_capture_route_anchor_valid_ = false;
}

void VisualLidarPathFollower::record_image_capture_point(
  const VisualLidarPathFollower::RobotPose & pose)
{
  image_capture_point_ = Point2D{pose.x, pose.y};
  image_capture_point_yaw_ = pose.yaw;
  image_capture_point_valid_ = true;
  publish_logic3_snapshot("image_capture");
  const double stop_to_capture_distance = image_capture_stop_point_valid_ ?
    std::hypot(
      image_capture_point_.x - image_capture_stop_point_.x,
      image_capture_point_.y - image_capture_stop_point_.y) : 0.0;
  RCLCPP_INFO(
    get_logger(),
    "图生文拍照点已记录：map=(%.3f, %.3f)，yaw=%.3frad；"
    "停车点到拍照点偏移=%.3fm；已请求生成TuShengWen.png。",
    image_capture_point_.x, image_capture_point_.y, image_capture_point_yaw_,
    stop_to_capture_distance);
}

bool VisualLidarPathFollower::image_capture_corridor_contains(
  const VisualLidarPathFollower::RobotPose & pose) const
{
  if (!enable_image_analysis_trigger_ || following_qr_path_ ||
    task_phase_ != TaskPhase::TASK2)
  {
    return false;
  }

  const std::string direction = to_lower(selected_route_direction_);
  double enter_x = 0.0;
  double exit_x = 0.0;
  if (direction == "clockwise") {
    enter_x = image_capture_clockwise_enter_x_;
    exit_x = image_capture_clockwise_exit_x_;
  } else if (direction == "counterclockwise") {
    enter_x = image_capture_counterclockwise_enter_x_;
    exit_x = image_capture_counterclockwise_exit_x_;
  } else {
    return false;
  }

  const double min_x = std::min(enter_x, exit_x);
  const double max_x = std::max(enter_x, exit_x);
  return pose.x >= min_x && pose.x <= max_x &&
         pose.y >= image_capture_corridor_min_y_ &&
         pose.y <= image_capture_corridor_max_y_;
}

// 返回true表示本控制周期必须禁止平移，但转向仍由巡线或雷达避障控制。
// 候选触发只让VLM在内存中保存最新静止帧；离开走廊时才写盘并调用一次API。
bool VisualLidarPathFollower::update_image_capture_corridor(
  const VisualLidarPathFollower::RobotPose & pose)
{
  if (!enable_image_analysis_trigger_ || following_qr_path_ ||
    task_phase_ != TaskPhase::TASK2 ||
    image_capture_corridor_state_ == ImageCaptureCorridorState::FINISHED)
  {
    return false;
  }

  const std::string direction = to_lower(selected_route_direction_);
  const bool clockwise = direction == "clockwise";
  const bool counterclockwise = direction == "counterclockwise";
  if (!clockwise && !counterclockwise) {
    return false;
  }

  const double enter_x = clockwise ?
    image_capture_clockwise_enter_x_ : image_capture_counterclockwise_enter_x_;
  const double exit_x = clockwise ?
    image_capture_clockwise_exit_x_ : image_capture_counterclockwise_exit_x_;
  const bool reached_enter = clockwise ? pose.x >= enter_x : pose.x <= enter_x;
  const bool reached_exit = clockwise ? pose.x >= exit_x : pose.x <= exit_x;
  const bool within_height =
    pose.y >= image_capture_corridor_min_y_ &&
    pose.y <= image_capture_corridor_max_y_;

  if (image_capture_corridor_state_ == ImageCaptureCorridorState::WAITING) {
    if (!reached_enter || !within_height) {
      return false;
    }
    image_capture_corridor_state_ = ImageCaptureCorridorState::COLLECTING;
    RCLCPP_INFO(
      get_logger(),
      "进入%s图生文拍照走廊：x从%.2f到%.2f，y在[%.2f, %.2f]，"
      "矩形基础速度%.2fm/s，转弯减速继续生效；航向容差%.1f°，"
      "合格后停车%.2fs拍照。",
      route_direction_name(direction).c_str(), enter_x, exit_x,
      image_capture_corridor_min_y_, image_capture_corridor_max_y_,
      image_capture_corridor_speed_,
      image_capture_yaw_tolerance_ * 180.0 / M_PI, image_capture_stop_duration_);
  }

  if (image_capture_corridor_state_ == ImageCaptureCorridorState::STOPPING) {
    if (stop_brake_sequence_active_) {
      return true;
    }
    if (!image_capture_stop_timer_started_) {
      image_capture_stop_timer_started_ = true;
      image_capture_stop_started_at_ = std::chrono::steady_clock::now();
      RCLCPP_INFO(
        get_logger(),
        "图生文统一反相制动已完成，开始静止保持并计时%.2f秒。",
        image_capture_stop_duration_);
    }
    const double stopped_seconds = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - image_capture_stop_started_at_).count();
    if (stopped_seconds >= image_capture_stop_duration_) {
      publish_image_analysis_trigger(
        image_capture_trigger_value_, "停车计时结束，保存最新静止帧");
      record_image_capture_point(pose);
      image_capture_candidate_selected_ = true;
      image_capture_corridor_state_ = ImageCaptureCorridorState::CAPTURED;
      // 本周期仍保持零速；下一次恢复正向运动时使用事件专用
      // 起步加速度，不再与global_max_accel共用。
      arm_event_start_acceleration(EventStartContext::IMAGE_CAPTURE_RESUME);
      RCLCPP_INFO(
        get_logger(), "图生文停车%.3fs后已选取最新帧，本周期继续保持零速。",
        stopped_seconds);
    }
    return true;
  }

  if (reached_exit) {
    // 整段航向都不合格时，不再行驶中直接抓取出口帧。复用同一个
    // STOPPING状态强制完成一次停车拍照，保证每次经过走廊至少停一次。
    if (!image_capture_candidate_selected_) {
      begin_image_capture_stop(pose, "出口兜底停车");
      image_capture_corridor_state_ = ImageCaptureCorridorState::STOPPING;
      RCLCPP_WARN(
        get_logger(),
        "离开%s图生文拍照走廊时尚未停车拍照，开始出口兜底停车%.2fs。",
        route_direction_name(direction).c_str(), image_capture_stop_duration_);
      return true;
    }
    publish_image_analysis_trigger(image_infer_trigger_value_, "离开拍照走廊，开始识别");
    image_capture_corridor_state_ = ImageCaptureCorridorState::FINISHED;
    RCLCPP_INFO(
      get_logger(), "离开%s图生文拍照走廊，已提交最后一张候选图识别。",
      route_direction_name(direction).c_str());
    return false;
  }

  const double target_yaw = clockwise ? 0.0 : M_PI;
  const double yaw_error = std::abs(normalize_angle(target_yaw - pose.yaw));
  if (image_capture_corridor_state_ == ImageCaptureCorridorState::COLLECTING &&
    within_height && yaw_error <= image_capture_yaw_tolerance_)
  {
    begin_image_capture_stop(pose, "位置和航向合格");
    image_capture_corridor_state_ = ImageCaptureCorridorState::STOPPING;
    RCLCPP_INFO(
      get_logger(), "图生文位置和航向合格（误差%.1f°），开始一次性停车%.2fs。",
      yaw_error * 180.0 / M_PI, image_capture_stop_duration_);
    return true;
  }
  return false;
}

// ********************************************************************************
// ******************************** QR 与图生文触发 ********************************
// ********************************************************************************
// QR 回调：
//   1. 根据 QR 数字判断顺/逆时针；
//   2. 从二维码前轨迹切换到该方向独立的 channel 航点文件。
void VisualLidarPathFollower::qr_callback(const std_msgs::msg::String::ConstSharedPtr msg)
{
  if (qr_route_selected_) {
    return;
  }
  const std::string direction = direction_from_qr(msg->data);
  if (!direction.empty()) {
    valid_qr_received_ = true;
    // 回调只锁存二维码内容、扫码位置和待选方向。路线文件加载、重规划及
    // 快照等非控制工作统一留给后续控制周期，避免相机回调阻塞底盘控制。
    RobotPose scan_pose;
    const bool scan_pose_valid = logic3_planning_pose_anchored_ ?
      lookup_logic3_planning_pose(scan_pose) :
      lookup_robot_pose(scan_pose.x, scan_pose.y, scan_pose.yaw);
    if (!task1_transition_marker_valid_) {
      qr_reverse_x_force_evaluated_ = false;
      qr_reverse_x_force_pending_ = false;
      qr_reverse_force_source_label_.clear();
      if (scan_pose_valid) {
        task1_transition_marker_ = Point2D{scan_pose.x, scan_pose.y};
        task1_transition_marker_label_ = "QR";
        task1_transition_marker_valid_ = true;
      }
    }
    // 传感器节点只负责上报；大厅半场门槛前缓存原始内容但不播报，
    // 直到Final冻结路线已经以真正的过渡起点生成完成。
    if (task1_pending_qr_text_.empty() ||
      task1_pending_qr_is_fallback_)
    {
      task1_pending_qr_text_ = msg->data;
      task1_pending_qr_is_fallback_ = false;
      task1_qr_speech_pending_ = true;
      task1_qr_speech_published_ = false;
    }

    task1_pending_direction_ = direction;
    task1_pending_reason_ = "识别到有效二维码";
    task1_pending_from_qr_ = true;
    task1_pending_gate_logged_ = true;

    const bool gate_open = logic3_planning_pose_anchored_ && scan_pose_valid &&
      scan_pose.x >= task1_transition_min_x_;
    if (gate_open) {
      // 门槛已经满足时，扫码回调只做这一条最高优先级控制动作：立刻
      // 直接零速并保留当前转向，不执行反相制动。下一控制周期再切路线。
      qr_reverse_hold_angular_command_ =
        std::isfinite(last_published_angular_command_) ?
        last_published_angular_command_ : 0.0;
      publish_translation_stop_without_reverse_brake(qr_reverse_hold_angular_command_);
      RCLCPP_INFO(
        get_logger(),
        "已识别有效二维码：当前base_link.x=%.3f>=%.2fm，已立即直接零速；"
        "不执行反相制动，路线切换交由下一控制周期。",
        scan_pose.x, task1_transition_min_x_);
    } else if (logic3_planning_pose_anchored_ && scan_pose_valid) {
      RCLCPP_INFO(
        get_logger(),
        "Task1方向已缓存：%s；当前base_link.x=%.3f<%.2fm，继续执行Task1，"
        "达到门槛后再进入过渡阶段。",
        route_direction_name(direction).c_str(), scan_pose.x,
        task1_transition_min_x_);
    } else {
      RCLCPP_INFO(
        get_logger(),
        "Task1方向已缓存：%s；等待可信发车且base_link.x>=%.2fm后再进入过渡阶段。",
        route_direction_name(direction).c_str(), task1_transition_min_x_);
    }
    shutdown_yolo_detector("已识别有效二维码并缓存方向");
    return;
  }
}

bool VisualLidarPathFollower::complete_task1_and_select_route(
  const std::string & direction, const char * reason, bool from_qr)
{
  if (qr_route_selected_) {
    return true;
  }

  const std::string reason_text = reason ? reason : "方向已确定";
  RobotPose selection_pose;
  const bool selection_pose_valid = logic3_planning_pose_anchored_ ?
    lookup_logic3_planning_pose(selection_pose) :
    lookup_robot_pose(selection_pose.x, selection_pose.y, selection_pose.yaw);

  if (from_qr && !task1_transition_marker_valid_ && selection_pose_valid) {
    task1_transition_marker_ = Point2D{selection_pose.x, selection_pose.y};
    task1_transition_marker_label_ = "QR";
    task1_transition_marker_valid_ = true;
  }

  // 已缓存真实二维码后，超时/末点的默认顺时针请求不得覆盖它。
  if (!from_qr && task1_pending_from_qr_ &&
    !task1_pending_direction_.empty())
  {
    return false;
  }

  const bool gate_open =
    logic3_planning_pose_anchored_ && selection_pose_valid &&
    selection_pose.x >= task1_transition_min_x_;
  if (!gate_open) {
    if (from_qr || task1_pending_direction_.empty()) {
      task1_pending_direction_ = direction;
      task1_pending_reason_ = reason_text;
      task1_pending_from_qr_ = from_qr;
    }
    if (!task1_pending_gate_logged_) {
      if (logic3_planning_pose_anchored_ && selection_pose_valid) {
        RCLCPP_INFO(
          get_logger(),
          "Task1方向已缓存：%s；等待base_link.x>=%.2fm后再进入过渡阶段"
          "（当前x=%.3f）。",
          route_direction_name(task1_pending_direction_).c_str(),
          task1_transition_min_x_, selection_pose.x);
      } else {
        RCLCPP_INFO(
          get_logger(),
          "Task1方向已缓存：%s；等待可信发车且base_link.x>=%.2fm后再进入过渡阶段。",
          route_direction_name(task1_pending_direction_).c_str(),
          task1_transition_min_x_);
      }
      task1_pending_gate_logged_ = true;
    }
    return false;
  }

  // 用户所称的“QR点”是正式进入下一阶段这一刻的
  // base_link坐标，不是二维码被相机识别时的早期坐标。真实QR
  // 与超时/末点默认选路都只在此处锁存一次入口X。
  if (!qr_reverse_x_force_evaluated_) {
    qr_reverse_x_force_evaluated_ = true;
    qr_reverse_force_source_x_ = selection_pose.x;
    qr_reverse_force_source_label_ = "Task1→Task2过渡入口";
    qr_reverse_x_force_pending_ =
      selection_pose.x >= qr_reverse_force_qr_x_threshold_;
  }

  const bool pre_reverse_brake_required =
    qr_reverse_max_count_ > 0 && qr_reverse_x_force_pending_;
  const bool skip_pre_reverse_brake = pre_reverse_brake_required && from_qr;
  if (from_qr) {
    qr_reverse_hold_angular_command_ =
      std::isfinite(last_published_angular_command_) ?
      last_published_angular_command_ : 0.0;
  }
  if (pre_reverse_brake_required) {
    // 速度命令是最高优先级：先停车，再进行路线文件切换。真实二维码
    // 明确走直接零速通道，不执行反相制动；超时/默认选路仍保留原制动逻辑。
    if (!from_qr) {
      qr_reverse_hold_angular_command_ =
        std::isfinite(last_published_angular_command_) ?
        last_published_angular_command_ : 0.0;
    }
    if (skip_pre_reverse_brake) {
      publish_translation_stop_without_reverse_brake(qr_reverse_hold_angular_command_);
    } else {
      publish_translation_stop(qr_reverse_hold_angular_command_);
    }
  }

  // 02图与01图使用完全相同的完整Task1原始/冻结路线；02额外叠加
  // 红点历史、实际二维码位置，以及正式进入Task1→Task2过渡时的
  // base_link蓝色发车点，不再裁成已经行驶的前缀。默认顺时针没有
  // 真实二维码坐标，因此不伪造QR事件标记。
  task1_dynamic_original_route_ = task1_original_route_;
  task1_dynamic_route_ = task1_frozen_route_;
  if (!switch_route(direction)) {
    return false;
  }
  if (pre_reverse_brake_required) {
    if (skip_pre_reverse_brake) {
      // 禁止使用停车前/扫码前缓存的点云决定倒车方向。零速已经发布，
      // 从此刻开始等一帧新雷达，再在最新位姿上启动正式倒车判断。
      qr_pre_reverse_scan_sequence_ = scan_sequence_;
      qr_pre_reverse_time_nanoseconds_ = now().nanoseconds();
      qr_escape_state_ = QrEscapeState::WAITING_FOR_PRE_REVERSE_SCAN;
    } else {
      qr_escape_state_ = QrEscapeState::PRE_REVERSE_BRAKE;
      qr_pre_reverse_scan_sequence_ = 0;
      qr_pre_reverse_time_nanoseconds_ = 0;
    }
  }

  // 默认选路没有真实二维码回调，必须在Task1正式结束边界补发终止值。
  // 真实二维码路径已提前发送；helper内部会去重。
  shutdown_yolo_detector(from_qr ? "Task1二维码选路完成" : "Task1默认选路完成");

  qr_route_selected_ = true;
  task1_endpoint_waiting_ = false;
  task1_endpoint_wait_timer_started_ = false;
  task1_endpoint_wait_started_at_ = std::chrono::steady_clock::time_point{};
  task1_pending_direction_.clear();
  task1_pending_reason_.clear();
  task1_pending_from_qr_ = false;
  task1_pending_gate_logged_ = false;
  // 此处就是Task1正式结束、进入下一阶段的唯一边界。基础03图记录这一刻
  // 的base_link和QR点；强制倒车状态结束后，03(n)再以实际重规划A点刷新。
  final_frozen_marker_ = Point2D{selection_pose.x, selection_pose.y};
  final_frozen_marker_yaw_ = selection_pose.yaw;
  final_frozen_marker_label_ = "ROUTE START";
  final_frozen_marker_valid_ = true;
  // 04图对应的Final阶段从Task1正式结束这一刻开始；实际轨迹在这里
  // 清零并写入首点，后续倒车重规划只更新03基线，不截断已行驶轨迹。
  start_logic3_actual_route(selection_pose);
  // 02图到此收口；统一红点ID库继续跨阶段保留身份。每个ID分别记录是否
  // 在Task1及Task1后真正触发，04系列据此筛选，不需要复制第二份坐标库。
  set_task_phase(TaskPhase::TRANSITION);
  // 基础03图只是过渡入口事件图：不生成A→C→B、不执行静态规划，只把
  // 正式入口base_link位置/朝向和真实QR扫描点交给外部绘图进程。真正
  // 可执行路线只在强制倒车状态结束后生成，并使用倒车轮次输出03(n)。
  publish_logic3_snapshot("final_transition_marker");
  // 02图快照序列化不得阻塞二维码→倒车的控制交接。先锁存事件；二维码
  // 倒车/重规划分支每周期都会优先返回，待恢复正常控制并先发布速度后再画。
  task1_dynamic_snapshot_pending_ = true;
  if (qr_sub_) {
    qr_sub_.reset();
  }
  RCLCPP_INFO(
    get_logger(),
    "Task1结束：%s；选择%s路线并进入过渡阶段；倒车前停车=%s。",
    reason_text.c_str(),
    route_direction_name(direction).c_str(),
    skip_pre_reverse_brake ? "真实二维码直接零速（不反相制动）" :
    (pre_reverse_brake_required ? "保留原统一反相制动" :
    "本次入口X未要求强制倒车"));
  return true;
}

void VisualLidarPathFollower::publish_pending_qr_speech()
{
  if (!task1_qr_speech_pending_ || task1_qr_speech_published_ ||
    task1_pending_qr_text_.empty() || !tts_pub_)
  {
    return;
  }

  const std::string direction = direction_from_qr(task1_pending_qr_text_);
  if (direction.empty()) {
    return;
  }
  const bool clockwise = direction == "clockwise";
  std_msgs::msg::String message;
  message.data =
    "数字：" + task1_pending_qr_text_ +
    (clockwise ? "，为奇数，进行顺时针绕行" : "，为偶数，进行逆时针绕行");
  tts_pub_->publish(message);
  task1_qr_speech_pending_ = false;
  task1_qr_speech_published_ = true;
  task1_pending_qr_is_fallback_ = false;
  RCLCPP_INFO(
    get_logger(),
    "已进入Task1→Task2过渡并完成Final冻结路线，现播报缓存二维码：%s。",
    task1_pending_qr_text_.c_str());
}

// 将 QR 字符串转换为路线方向。
// use_qr_parity_direction_=true 时：奇数=clockwise，偶数=counterclockwise。
// 否则使用 clockwise_qr_value_ 和 counterclockwise_qr_value_ 做精确匹配。
std::string VisualLidarPathFollower::direction_from_qr(const std::string & data) const
{
  if (!enable_qr_direction_select_) {
    return "";
  }
  int value = 0;
  try {
    value = std::stoi(data);
  } catch (const std::exception &) {
    return "";
  }
  if (use_qr_parity_direction_) {
    return value % 2 == 0 ? "counterclockwise" : "clockwise";
  }
  if (value == clockwise_qr_value_) {
    return "clockwise";
  }
  if (value == counterclockwise_qr_value_) {
    return "counterclockwise";
  }
  return "";
}

// 切换路线：
//   扫码前在 task1_waypoints_file_ 中跑二维码前轨迹；
//   扫到 QR 后加载该方向独立的 channel 文件，不再反向读取另一方向路线。
bool VisualLidarPathFollower::switch_route(const std::string & direction)
{
  if (!waiting_for_qr_start_ && !following_qr_path_ && direction == selected_route_direction_) {
    return true;
  }

  const std::string normalized_direction = to_lower(direction);
  std::string route_waypoints_file;
  if (normalized_direction == "clockwise") {
    route_waypoints_file = route_waypoints_clockwise_file_;
  } else if (normalized_direction == "counterclockwise") {
    route_waypoints_file = route_waypoints_counterclockwise_file_;
  } else {
    RCLCPP_WARN(get_logger(), "无法选择未知 channel 方向：%s。", direction.c_str());
    return false;
  }

  std::string resolved_direction;
  const auto new_waypoints = load_waypoints(
    route_waypoints_file, normalized_direction, &resolved_direction);
  if (new_waypoints.empty()) {
    const auto route_name = route_direction_name(direction);
    RCLCPP_WARN(
      get_logger(), "QR 选择了路线“%s”，但未从独立文件 %s 加载到对应航点。",
      route_name.c_str(), route_waypoints_file.c_str());
    return false;
  }

  const bool was_following_qr_path = following_qr_path_;
  const bool was_waiting_for_qr = waiting_for_qr_start_;
  channel_route_template_ = new_waypoints;
  waypoints_ = new_waypoints;
  selected_route_direction_ = resolved_direction.empty() ? direction : resolved_direction;
  channel_entry_waypoint_index_ = 0;
  channel_entry_smoothing_first_index_ = 0;
  channel_completed_reverse_count_ = 0;
  qr_pre_reverse_scan_sequence_ = 0;
  qr_pre_reverse_time_nanoseconds_ = 0;
  qr_reverse_active_trigger_reason_.clear();
  qr_reverse_active_x_force_ = false;
  qr_reverse_negative_command_published_ = false;
  qr_reverse_rearm_after_post_stop_ = false;
  channel_lidar_waiting_for_post_reverse_scan_ = false;
  channel_lidar_post_reverse_scan_sequence_ = 0;
  channel_lidar_post_reverse_time_nanoseconds_ = 0;
  channel_entry_rear_static_filter_active_ = false;
  task3_terminal_homing_active_ = false;
  task3_terminal_homing_used_ = false;
  logic3_latest_channel_entry_update_guard_route_.clear();
  if (qr_reverse_max_count_ <= 0) {
    qr_reverse_x_force_pending_ = false;
  }
  reset_image_capture_corridor();
  reset_waypoint_shift_state();
  channel_entry_route_rebuild_pending_ = true;
  channel_entry_route_rebuild_after_reverse_ = false;
  waiting_for_qr_start_ = false;
  following_qr_path_ = false;
  qr_path_finished_ = false;
  if (was_following_qr_path || was_waiting_for_qr) {
    current_index_ = 0;
    first_waypoint_reached_ = false;
    qr_channel_approach_active_ = true;
    qr_reverse_wheel_odom_distance_ = 0.0;
    qr_reverse_last_odom_position_valid_ = false;
    qr_reverse_motion_confirmed_ = false;
    qr_reverse_odom_fault_ = false;
    qr_reverse_waiting_for_odom_ = false;
    qr_escape_state_ = qr_reverse_max_count_ > 0 ?
      QrEscapeState::PENDING : QrEscapeState::IDLE;
  } else {
    current_index_ = std::min(current_index_, waypoints_.size() - 1);
    first_waypoint_reached_ = current_index_ > 0;
  }
  const auto route_name = route_direction_name(selected_route_direction_);
  RCLCPP_INFO(
    get_logger(),
    "QR 选择%s方向模板；倒车前不生成A→C→B且不做静态规划；"
    "强制倒车状态结束后才从最新base_link A使用当前yaw作为起始切线，"
    "生成A→C五次Bezier并连接waypoint_channel首点B，再按冻结静态坐标规划；"
    "A→C→B→Task2按统一before/after窗口静态偏移，过渡段同时对全部障碍做动态避障。",
    route_name.c_str());
  if (debug_control_logs_) {
    RCLCPP_INFO(
      get_logger(), "debug: 环线文件=%s，起点=%zu/%zu（%s）。",
      route_waypoints_file.c_str(), current_index_ + 1, waypoints_.size(),
      waypoints_[current_index_].name.c_str());
  }
  // 正式Task1→Task2切换时，此处只有方向模板，不是车辆将执行的路线；
  // 倒车状态结束后rebuild_channel_entry_route会发布真正的A→C→B路线。
  if (!was_following_qr_path && !was_waiting_for_qr) {
    publish_debug_route();
  }
  return true;
}

void VisualLidarPathFollower::suspend_visual_control()
{
  if (!suspend_visual_control_on_qr_ || !visual_control_suspend_pub_) {
    return;
  }

  std_msgs::msg::Int32 msg;
  msg.data = visual_control_suspend_value_;
  for (int i = 0; i < 3; ++i) {
    visual_control_suspend_pub_->publish(msg);
  }
  RCLCPP_INFO(get_logger(), "已通知 visual_control 挂起，visual_lidar_path_follower 接管。");
  if (debug_control_logs_) {
    RCLCPP_INFO(
      get_logger(), "debug: suspend_topic=%s, value=%d。",
      visual_control_suspend_topic_.c_str(), visual_control_suspend_value_);
  }
}

void VisualLidarPathFollower::shutdown_yolo_detector(const char * reason)
{
  if (yolo_shutdown_requested_) {
    return;
  }
  if (!visual_control_suspend_pub_) {
    RCLCPP_ERROR(get_logger(), "无法终止YOLO：/sign4return发布器不存在。");
    return;
  }

  yolo_shutdown_requested_ = true;

  std_msgs::msg::Int32 msg;
  msg.data = perception_shutdown_value_;
  for (int i = 0; i < 3; ++i) {
    visual_control_suspend_pub_->publish(msg);
  }
  RCLCPP_WARN(
    get_logger(), "YOLO任务结束（%s）：发布终止值，要求感知进程释放BPU并退出；"
    "topic=%s, value=%d。",
    reason ? reason : "Task1结束", visual_control_suspend_topic_.c_str(),
    perception_shutdown_value_);
}

void VisualLidarPathFollower::resume_visual_control()
{
  if (!visual_control_suspend_pub_) {
    return;
  }

  std_msgs::msg::Int32 msg;
  msg.data = visual_control_resume_value_;
  for (int i = 0; i < 3; ++i) {
    visual_control_suspend_pub_->publish(msg);
  }
  RCLCPP_INFO(get_logger(), "已通知 visual_control 恢复；YOLO保持退出，不再恢复。");
  if (debug_control_logs_) {
    RCLCPP_INFO(
      get_logger(), "debug: resume_topic=%s, visual_value=%d。",
      visual_control_suspend_topic_.c_str(), visual_control_resume_value_);
  }
}

void VisualLidarPathFollower::publish_image_analysis_trigger(
  int value, const char * reason)
{
  if (!enable_image_analysis_trigger_ || !image_analysis_trigger_pub_) {
    return;
  }

  std_msgs::msg::Int32 msg;
  msg.data = value;
  image_analysis_trigger_pub_->publish(msg);
  if (debug_control_logs_) {
    RCLCPP_INFO(
      get_logger(), "debug: 图生文触发话题=%s，值=%d（%s）。",
      image_analysis_trigger_topic_.c_str(), value, reason ? reason : "未注明");
  }
}
