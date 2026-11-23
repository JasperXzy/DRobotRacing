#include "origincar_system/visual_lidar_path_follower.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>

#include "tf2/exceptions.h"
#include "follower_utils.hpp"

using origincar_system::normalize_angle;

bool VisualLidarPathFollower::lookup_pose(
  const std::string & target_frame, const std::string & source_frame,
  const rclcpp::Time & stamp, VisualLidarPathFollower::RobotPose & pose) const
{
  try {
    const auto transform = tf_buffer_.lookupTransform(target_frame, source_frame, stamp);
    const auto & translation = transform.transform.translation;
    const auto & rotation = transform.transform.rotation;
    pose.x = translation.x;
    pose.y = translation.y;
    pose.yaw = std::atan2(
      2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
      1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z));
    return true;
  } catch (const tf2::TransformException &) {
    return false;
  }
}

VisualLidarPathFollower::Point2D VisualLidarPathFollower::base_to_map(
  const VisualLidarPathFollower::RobotPose & robot_pose,
  const VisualLidarPathFollower::Point2D & base_point) const
{
  const double cos_yaw = std::cos(robot_pose.yaw);
  const double sin_yaw = std::sin(robot_pose.yaw);
  return Point2D{
    robot_pose.x + cos_yaw * base_point.x - sin_yaw * base_point.y,
    robot_pose.y + sin_yaw * base_point.x + cos_yaw * base_point.y};
}

void VisualLidarPathFollower::latch_logic3_match_diagnostics_to_latest_scan()
{
  if (!latest_scan_ || !latest_match_diagnostics_valid_) {
    return;
  }
  const std::int64_t scan_stamp_nanoseconds = rclcpp::Time(
    latest_scan_->header.stamp, get_clock()->get_clock_type()).nanoseconds();
  if (scan_stamp_nanoseconds != latest_match_diagnostics_stamp_nanoseconds_) {
    return;
  }
  latest_lidar_match_score_valid_ = true;
  latest_match_score_scan_sequence_ = scan_sequence_;
  latest_match_diagnostics_scan_sequence_ = scan_sequence_;
}

void VisualLidarPathFollower::logic3_odom_callback(
  const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  const auto received_at = std::chrono::steady_clock::now();
  ++chassis_odom_sequence_;
  const double measured_linear_speed = msg->twist.twist.linear.x;
  const bool measured_linear_speed_valid = std::isfinite(measured_linear_speed);
  if (measured_linear_speed_valid) {
    latest_chassis_linear_speed_ = measured_linear_speed;
    latest_chassis_linear_speed_valid_ = true;
  }
  const double odom_x = msg->pose.pose.position.x;
  const double odom_y = msg->pose.pose.position.y;
  if (std::isfinite(odom_x) && std::isfinite(odom_y)) {
    const Point2D current_odom_position{odom_x, odom_y};
    if (qr_escape_state_ == QrEscapeState::BACKING_RIGHT) {
      if (!qr_reverse_last_odom_position_valid_ ||
        !latest_chassis_odom_position_valid_)
      {
        qr_reverse_odom_fault_ = true;
      } else {
        const double sample_gap = std::chrono::duration<double>(
          received_at - latest_chassis_odom_received_at_).count();
        const double distance_increment = std::hypot(
          current_odom_position.x - qr_reverse_last_odom_position_.x,
          current_odom_position.y - qr_reverse_last_odom_position_.y);
        if (!measured_linear_speed_valid ||
          !std::isfinite(sample_gap) || sample_gap < 0.0 ||
          sample_gap > qr_reverse_odom_timeout_seconds_ ||
          !std::isfinite(distance_increment))
        {
          qr_reverse_odom_fault_ = true;
        } else if (measured_linear_speed < -1.0e-6) {
          if (!qr_reverse_motion_confirmed_) {
            qr_reverse_motion_confirmed_ = true;
            RCLCPP_INFO(
              get_logger(),
              "过渡阶段条件倒车已检测到实测负轮速%.3fm/s；"
              "从本帧开始累计编码器倒车里程。",
              measured_linear_speed);
          }
          // /chassis/odom由底盘轮速积分产生。只在实测纵向轮速为负时
          // 累加逐帧位移；正向惯性和零轮速都不计入倒车里程。
          qr_reverse_wheel_odom_distance_ += distance_increment;
        }
      }
      qr_reverse_last_odom_position_ = current_odom_position;
      qr_reverse_last_odom_position_valid_ = true;
    }
    latest_chassis_odom_position_ = current_odom_position;
    latest_chassis_odom_position_valid_ = true;
    latest_chassis_odom_received_at_ = received_at;
  } else if (qr_escape_state_ == QrEscapeState::BACKING_RIGHT) {
    qr_reverse_odom_fault_ = true;
  }
  if (logic3_planning_pose_anchored_) {
    return;
  }

  if (!logic3_odom_reference_valid_) {
    logic3_odom_reference_valid_ = true;
    logic3_odom_reference_x_ = odom_x;
    logic3_odom_reference_y_ = odom_y;
    return;
  }

  const double speed = std::hypot(
    msg->twist.twist.linear.x, msg->twist.twist.linear.y);
  const double displacement = std::hypot(
    odom_x - logic3_odom_reference_x_, odom_y - logic3_odom_reference_y_);
  const bool moving = speed >= logic3_motion_speed_threshold_ ||
    displacement >= logic3_motion_displacement_threshold_;
  // 新READY规划结束后必须先由一条更新的静止里程计确认才允许提交；
  // 如果这条里程计已经显示发车，则丢弃候选并保留上次完整路线。
  resolve_logic3_ready_preplan_candidate(moving);
  auto capture_departure_snapshot = [&]() {
      if (logic3_ready_preplan_enabled_ && logic3_ready_preplan_valid_ &&
        logic3_ready_preplanned_snapshot_.valid)
      {
        RCLCPP_INFO(
          get_logger(),
          "静态+动态方案可信发车直接冻结最近一次完整READY预规划版本：%zu簇；"
          "不在发车瞬间重新计算路线。",
          logic3_ready_preplanned_snapshot_.obstacles.size());
        return logic3_ready_preplanned_snapshot_;
      }
      if (logic3_last_complete_ready_snapshot_.valid) {
        RCLCPP_INFO(
          get_logger(),
          "静态+动态方案可信发车固定采用运动前最后一次READY快照：%zu簇，"
          "不读取触发瞬间可能受定位漂移影响的当前点集。",
          logic3_last_complete_ready_snapshot_.obstacles.size());
        return logic3_last_complete_ready_snapshot_;
      }

      Logic3PlanningSnapshot snapshot;
      snapshot.valid = true;
      snapshot.stamp_seconds = now().seconds();
      RCLCPP_WARN(
        get_logger(),
        "静态+动态方案可信发车前从未形成READY快照：按规则冻结空快照，"
        "不使用触发瞬间的当前点集兜底。");
      return snapshot;
    };

  RobotPose start_zone_pose;
  if (logic3_start_zone_guard_enabled_) {
    if (!lookup_robot_pose(
        start_zone_pose.x, start_zone_pose.y, start_zone_pose.yaw))
    {
      // 可信发车依赖map中的基地边界；TF缺失时宁可继续观察，也不能仅凭
      // 里程计速度/位移提前冻结绿色快照。
      logic3_motion_candidate_ = false;
      logic3_start_zone_motion_pending_ = false;
      logic3_motion_candidate_count_ = 0;
      logic3_motion_pending_snapshot_ = Logic3PlanningSnapshot();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "静态+动态方案可信发车等待map->base_link；基地保护区判定不可用，暂不冻结。");
      return;
    }

    const double guarded_min_x =
      logic3_start_zone_min_x_ - logic3_start_zone_guard_margin_;
    const double guarded_max_x =
      logic3_start_zone_max_x_ + logic3_start_zone_guard_margin_;
    const double guarded_min_y =
      logic3_start_zone_min_y_ - logic3_start_zone_guard_margin_;
    const double guarded_max_y =
      logic3_start_zone_max_y_ + logic3_start_zone_guard_margin_;
    const bool inside_guarded_start_zone =
      start_zone_pose.x >= guarded_min_x && start_zone_pose.x <= guarded_max_x &&
      start_zone_pose.y >= guarded_min_y && start_zone_pose.y <= guarded_max_y;
    if (inside_guarded_start_zone) {
      logic3_motion_candidate_ = false;
      logic3_motion_candidate_count_ = 0;
      if (moving) {
        if (!logic3_start_zone_motion_pending_) {
          logic3_start_zone_motion_pending_ = true;
          logic3_motion_pending_snapshot_ = capture_departure_snapshot();
          RCLCPP_INFO(
            get_logger(),
            "静态+动态方案基地内检测到起步运动：预存%zu个READY绿色簇并暂停静态更新；"
            "base_link驶出12cm保护区后才确认可信发车。",
            logic3_motion_pending_snapshot_.obstacles.size());
        }
        // 保护区内持续刷新位移参考；这样人工挪车一旦停下，下一条低速
        // 里程计就能取消预存，而不会被先前累计位移永久判成仍在运动。
        logic3_odom_reference_x_ = odom_x;
        logic3_odom_reference_y_ = odom_y;
        return;
      }

      if (logic3_start_zone_motion_pending_) {
        RCLCPP_INFO(
          get_logger(),
          "静态+动态方案基地内运动已停止：取消预存发车快照，恢复静态绿色点更新。");
      }
      logic3_start_zone_motion_pending_ = false;
      logic3_motion_pending_snapshot_ = Logic3PlanningSnapshot();
      // 基地内允许人工调整和底盘小动；持续刷新里程计参考，避免驶出边界
      // 后沿用基地内累计位移造成瞬时误判。驶出后仍需满足速度或新位移门限。
      logic3_odom_reference_x_ = odom_x;
      logic3_odom_reference_y_ = odom_y;
      return;
    }
  }

  if (!moving) {
    const bool motion_candidate_was_active =
      logic3_motion_candidate_ || logic3_start_zone_motion_pending_;
    if (logic3_motion_candidate_ || logic3_start_zone_motion_pending_) {
      RCLCPP_INFO(
        get_logger(), "静态+动态方案发车候选取消：速度%.3fm/s，累计位移%.3fm。",
        speed, displacement);
    }
    logic3_motion_candidate_ = false;
    logic3_start_zone_motion_pending_ = false;
    logic3_motion_candidate_count_ = 0;
    logic3_motion_pending_snapshot_ = Logic3PlanningSnapshot();
    if (motion_candidate_was_active) {
      // 候选期间没有把起步震动帧写入静态状态机；候选取消后继续用
      // 原2秒窗口和状态1有效样本，下一帧会按时间窗口正常淘汰旧数据。
      RCLCPP_INFO(
        get_logger(),
        "静态+动态方案发车候选取消：恢复静态绿色点更新，不清空既有ID与有效样本。");
    }
    return;
  }

  if (!logic3_motion_candidate_) {
    const bool uses_start_zone_snapshot =
      logic3_start_zone_motion_pending_ && logic3_motion_pending_snapshot_.valid;
    logic3_motion_candidate_ = true;
    logic3_start_zone_motion_pending_ = false;
    logic3_motion_candidate_count_ = 1;
    // 第一条可信发车候选到来时立刻停止采用后续震动帧，只冻结此刻仍处于
    // 各自状态1且已经完成10个有效观测的ID。其他正在确认、失联或重采样
    // 的ID不影响这些稳定结果；仍然绝不回退到未经确认的原始雷达帧。
    if (!uses_start_zone_snapshot) {
      logic3_motion_pending_snapshot_ = capture_departure_snapshot();
    }
    RCLCPP_INFO(
      get_logger(),
      logic3_start_zone_guard_enabled_ ?
      "静态+动态方案检测到第1条发车候选：base_link已驶出基地12cm保护区，"
      "map=(%.3f, %.3f)，速度=%.3fm/s，新位移=%.3fm，采用%s快照%zu簇。" :
      "静态+动态方案检测到第1条发车候选：map=(%.3f, %.3f)，"
      "速度=%.3fm/s，累计位移=%.3fm，采用%s快照%zu簇。",
      start_zone_pose.x, start_zone_pose.y, speed, displacement,
      uses_start_zone_snapshot ? "基地内预存READY" : "最后READY缓存",
      logic3_motion_pending_snapshot_.obstacles.size());
  } else {
    ++logic3_motion_candidate_count_;
  }

  if (logic3_motion_candidate_count_ >= logic3_motion_confirm_messages_) {
    freeze_logic3_route();
  }
}

bool VisualLidarPathFollower::lookup_logic3_planning_pose(
  VisualLidarPathFollower::RobotPose & pose)
{
  const auto latest_stamp = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  if (!logic3_planning_pose_anchored_ ||
    logic3_planning_pose_source_ == "live_lidar_location")
  {
    return lookup_pose(frame_id_, base_frame_id_, latest_stamp, pose);
  }

  RobotPose odom_to_base;
  if (!lookup_pose(logic3_odom_frame_id_, base_frame_id_, latest_stamp, odom_to_base)) {
    return false;
  }
  RobotPose live_map_odom;
  const bool has_live_map_odom = lookup_pose(
    frame_id_, logic3_odom_frame_id_, latest_stamp, live_map_odom);
  if (!logic3_map_odom_plan_valid_) {
    if (!has_live_map_odom) {
      return false;
    }
    logic3_map_odom_plan_ = live_map_odom;
    logic3_map_odom_plan_valid_ = true;
    logic3_last_pose_correction_at_ = std::chrono::steady_clock::now();
  }

  if (logic3_planning_pose_source_ == "filtered_map_odom" && has_live_map_odom) {
    const auto current_time = std::chrono::steady_clock::now();
    const double dt = std::max(
      0.0, std::chrono::duration<double>(
        current_time - logic3_last_pose_correction_at_).count());
    logic3_last_pose_correction_at_ = current_time;
    const double dx = live_map_odom.x - logic3_map_odom_plan_.x;
    const double dy = live_map_odom.y - logic3_map_odom_plan_.y;
    const double translation_error = std::hypot(dx, dy);
    const double yaw_error = normalize_angle(
      live_map_odom.yaw - logic3_map_odom_plan_.yaw);
    if (translation_error <= logic3_pose_outlier_xy_ &&
      std::abs(yaw_error) <= logic3_pose_outlier_yaw_)
    {
      if (translation_error > logic3_pose_correction_deadband_xy_ && dt > 0.0) {
        const double step = std::min(
          translation_error, logic3_pose_correction_rate_xy_ * dt);
        logic3_map_odom_plan_.x += step * dx / translation_error;
        logic3_map_odom_plan_.y += step * dy / translation_error;
      }
      if (std::abs(yaw_error) > logic3_pose_correction_deadband_yaw_ && dt > 0.0) {
        const double step = std::min(
          std::abs(yaw_error), logic3_pose_correction_rate_yaw_ * dt);
        logic3_map_odom_plan_.yaw = normalize_angle(
          logic3_map_odom_plan_.yaw + std::copysign(step, yaw_error));
      }
    }
  }

  const double cos_yaw = std::cos(logic3_map_odom_plan_.yaw);
  const double sin_yaw = std::sin(logic3_map_odom_plan_.yaw);
  pose.x = logic3_map_odom_plan_.x +
    cos_yaw * odom_to_base.x - sin_yaw * odom_to_base.y;
  pose.y = logic3_map_odom_plan_.y +
    sin_yaw * odom_to_base.x + cos_yaw * odom_to_base.y;
  pose.yaw = normalize_angle(logic3_map_odom_plan_.yaw + odom_to_base.yaw);
  return true;
}

// 从 TF 查询 frame_id_ -> base_frame_id_ 的最新位姿。
// 返回 x/y/yaw，其中 yaw 是机器人在 map 坐标系下的航向角。
bool VisualLidarPathFollower::lookup_robot_pose(double & x, double & y, double & yaw)
{
  RobotPose pose;
  if (!lookup_logic3_planning_pose(pose)) {
    return false;
  }
  x = pose.x;
  y = pose.y;
  yaw = pose.yaw;
  return true;
}
