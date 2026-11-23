#include "origincar_system/visual_lidar_path_follower.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "follower_utils.hpp"

using origincar_system::normalize_angle;

// ********************************************************************************
// ******************************** 路线线段进度 ********************************
// ********************************************************************************
// 正常巡线只按车辆在当前路线上的局部投影推进。控制前瞻只负责选转向目标，
// 不再提交路线进度；普通航点的pass_radius也不再参与推进。
// current_index_ 表示当前参考线段起点：waypoint[i] -> waypoint[i + 1]。
void VisualLidarPathFollower::update_route_progress(const VisualLidarPathFollower::RobotPose & pose)
{
  if (waypoints_.size() < 2 || current_index_ >= waypoints_.size() - 1) {
    return;
  }

  RouteProjection projection;
  if (!find_route_projection(pose, projection)) {
    return;
  }

  std::size_t candidate_index = projection.segment_index;
  if (projection.t >= segment_advance_t_) {
    candidate_index = projection.segment_index + 1;
  }
  candidate_index = std::min(candidate_index, waypoints_.size() - 1);
  if (candidate_index <= current_index_) {
    return;
  }

  std::size_t allowed_index = current_index_;
  if (!can_advance_to_index(candidate_index, pose, allowed_index)) {
    return;
  }
  advance_current_index_to(allowed_index, "路线投影");
}

double VisualLidarPathFollower::route_projection_arc_length(
  const VisualLidarPathFollower::RouteProjection & projection) const
{
  if (waypoints_.size() < 2) {
    return 0.0;
  }
  const std::size_t last_segment = waypoints_.size() - 2;
  const std::size_t target_segment = std::min(
    projection.segment_index, last_segment);
  double progress = 0.0;
  for (std::size_t index = 0; index < target_segment; ++index) {
    progress += std::hypot(
      waypoints_[index + 1].x - waypoints_[index].x,
      waypoints_[index + 1].y - waypoints_[index].y);
  }
  const double segment_length = std::hypot(
    waypoints_[target_segment + 1].x - waypoints_[target_segment].x,
    waypoints_[target_segment + 1].y - waypoints_[target_segment].y);
  progress += std::clamp(projection.t, 0.0, 1.0) * segment_length;
  return progress;
}

// 在current_index_附近按路线实际弧长建立局部窗口。控制投影允许落在当前
// 进度后方以吸收索引误差，但update_route_progress()仍只允许任务进度向前。
// 窗口内只要存在非零长度线段就选空间距离最近者；raw_t和距离只用于描述
// 结果，不再作为普通失败门槛。局部窗口可避免回环路线投影到远处支路。
bool VisualLidarPathFollower::find_route_projection(
  const VisualLidarPathFollower::RobotPose & pose,
  VisualLidarPathFollower::RouteProjection & projection)
{
  projection = RouteProjection();
  if (waypoints_.size() < 2 || current_index_ >= waypoints_.size() - 1) {
    return false;
  }

  const std::size_t last_segment = waypoints_.size() - 2;
  const bool guard_projection =
    image_capture_route_progress_guard_enabled_ &&
    image_capture_route_guard_active_ &&
    image_capture_route_anchor_valid_ &&
    image_capture_route_anchor_.segment_index <= last_segment;
  std::vector<double> cumulative_lengths;
  double guard_min_progress = 0.0;
  double guard_max_progress = std::numeric_limits<double>::infinity();
  std::size_t start_segment = 0;
  std::size_t end_segment = last_segment;
  if (guard_projection) {
    cumulative_lengths.assign(waypoints_.size(), 0.0);
    for (std::size_t index = 0; index <= last_segment; ++index) {
      cumulative_lengths[index + 1] = cumulative_lengths[index] + std::hypot(
        waypoints_[index + 1].x - waypoints_[index].x,
        waypoints_[index + 1].y - waypoints_[index].y);
    }
    const double anchor_progress = route_projection_arc_length(
      image_capture_route_anchor_);
    guard_min_progress = std::max(
      0.0, anchor_progress - image_capture_reprojection_backward_distance_);
    guard_max_progress = std::min(
      cumulative_lengths.back(),
      anchor_progress + image_capture_reprojection_forward_distance_);
  } else {
    start_segment = current_index_;
    double backward_length = 0.0;
    while (start_segment > 0 &&
      backward_length < route_projection_search_backward_distance_)
    {
      --start_segment;
      backward_length += std::hypot(
        waypoints_[start_segment + 1].x - waypoints_[start_segment].x,
        waypoints_[start_segment + 1].y - waypoints_[start_segment].y);
    }

    end_segment = current_index_;
    double forward_length = 0.0;
    while (end_segment < last_segment &&
      forward_length < route_projection_search_forward_distance_)
    {
      forward_length += std::hypot(
        waypoints_[end_segment + 1].x - waypoints_[end_segment].x,
        waypoints_[end_segment + 1].y - waypoints_[end_segment].y);
      ++end_segment;
    }
  }

  double best_distance = std::numeric_limits<double>::infinity();

  for (std::size_t index = start_segment; index <= end_segment; ++index) {
    const auto & start = waypoints_[index];
    const auto & end = waypoints_[index + 1];
    const double seg_x = end.x - start.x;
    const double seg_y = end.y - start.y;
    const double seg_len_sq = seg_x * seg_x + seg_y * seg_y;
    if (seg_len_sq < 1e-6) {
      continue;
    }

    const double robot_x = pose.x - start.x;
    const double robot_y = pose.y - start.y;
    const double raw_t = (robot_x * seg_x + robot_y * seg_y) / seg_len_sq;
    double minimum_t = 0.0;
    double maximum_t = 1.0;
    if (guard_projection) {
      const double segment_length = std::sqrt(seg_len_sq);
      const double segment_start_progress = cumulative_lengths[index];
      const double segment_end_progress = cumulative_lengths[index + 1];
      if (segment_end_progress + 1.0e-9 < guard_min_progress ||
        segment_start_progress - 1.0e-9 > guard_max_progress)
      {
        continue;
      }
      minimum_t = std::clamp(
        (guard_min_progress - segment_start_progress) / segment_length,
        0.0, 1.0);
      maximum_t = std::clamp(
        (guard_max_progress - segment_start_progress) / segment_length,
        0.0, 1.0);
      if (minimum_t > maximum_t + 1.0e-9) {
        continue;
      }
    }
    const double t = std::clamp(raw_t, minimum_t, maximum_t);
    const Point2D projected{start.x + seg_x * t, start.y + seg_y * t};
    const double distance = std::hypot(pose.x - projected.x, pose.y - projected.y);

    if (distance < best_distance) {
      best_distance = distance;
      projection.valid = true;
      projection.segment_index = index;
      // 图生文停车保护期间返回受限后的t，确保进度提交和前瞻目标都不能
      // 越过停车锚点前方允许距离；正常巡线仍保留原始raw_t语义。
      projection.t = guard_projection ? t : raw_t;
      projection.distance = distance;
      projection.point = projected;
    }
  }

  if (projection.valid && projection.distance > route_projection_max_distance_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "路线局部最佳投影距离=%.3fm，超过当前阶段诊断阈值=%.3fm；"
      "仍使用线段%zu继续控制，不判失败、不回退。",
      projection.distance, route_projection_max_distance_,
      projection.segment_index);
  }

  return projection.valid;
}

bool VisualLidarPathFollower::find_nearest_waypoint(
  const VisualLidarPathFollower::RobotPose & pose, std::size_t & nearest_index,
  double & nearest_distance) const
{
  if (waypoints_.empty()) {
    return false;
  }

  nearest_index = 0;
  nearest_distance = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index < waypoints_.size(); ++index) {
    const auto & waypoint = waypoints_[index];
    const double distance = std::hypot(waypoint.x - pose.x, waypoint.y - pose.y);
    if (distance < nearest_distance) {
      nearest_distance = distance;
      nearest_index = index;
    }
  }
  return true;
}

// 先把车辆投影到当前局部路线，再从投影点沿路线弧长前进指定距离。
// 目标允许落在线段内部，避免连续S弯中用“车辆到离散点的空间距离”选点时
// 跳过中间反弯。精准点和正/倒车切换点仍然禁止被前瞻跨过。
VisualLidarPathFollower::TargetPoint VisualLidarPathFollower::compute_lookahead_target(
  const VisualLidarPathFollower::RobotPose & pose, double lookahead_distance)
{
  TargetPoint target;
  if (waypoints_.empty()) {
    return target;
  }
  if (waypoints_.size() < 2 || current_index_ >= waypoints_.size() - 1) {
    const std::size_t index = waypoints_.size() - 1;
    target.point = Point2D{waypoints_[index].x, waypoints_[index].y};
    target.waypoint_index = index;
    target.reverse = false;
    return target;
  }
  if (!first_waypoint_reached_) {
    target.point = Point2D{waypoints_.front().x, waypoints_.front().y};
    target.waypoint_index = 0;
    target.reverse = waypoints_.front().reverse;
    return target;
  }

  RouteProjection projection;
  if (!find_route_projection(pose, projection)) {
    // 仅当局部窗口没有有效线段时保留一个确定目标；正常路线始终走下方
    // 的投影加弧长算法。
    const std::size_t next_index = std::min(
      current_index_ + 1, waypoints_.size() - 1);
    target.waypoint_index = next_index;
    target.reverse = waypoints_[next_index].reverse;
    target.point = Point2D{waypoints_[next_index].x, waypoints_[next_index].y};
    return target;
  }

  const std::size_t last_segment = waypoints_.size() - 2;
  std::size_t segment_index = std::min(projection.segment_index, last_segment);
  double segment_t = std::clamp(projection.t, 0.0, 1.0);
  const bool route_reverse = is_segment_reverse(segment_index);
  double remaining = std::max(0.0, lookahead_distance);
  if (route_reverse) {
    remaining = std::min(remaining, reverse_goal_lookahead_distance_);
  }

  while (segment_index <= last_segment) {
    const std::size_t finish_index = segment_index + 1;
    const auto & start = waypoints_[segment_index];
    const auto & finish = waypoints_[finish_index];
    const double segment_x = finish.x - start.x;
    const double segment_y = finish.y - start.y;
    const double segment_length = std::hypot(segment_x, segment_y);

    // 精准点必须真正追到；也不能从当前运动方向跨入相反方向段。
    const bool precision_finish = is_precision_waypoint(finish_index);
    const bool direction_changes_after_finish =
      (segment_index < last_segment &&
      is_segment_reverse(segment_index + 1) != route_reverse);

    if (segment_length > 1.0e-6) {
      const double available = (1.0 - segment_t) * segment_length;
      if (remaining <= available && !precision_finish) {
        const double target_t = std::clamp(
          segment_t + remaining / segment_length, 0.0, 1.0);
        target.point = Point2D{
          start.x + target_t * segment_x,
          start.y + target_t * segment_y};
        target.waypoint_index = finish_index;
        target.reverse = route_reverse;
        return target;
      }
      remaining = std::max(0.0, remaining - available);
    }

    target.point = Point2D{finish.x, finish.y};
    target.waypoint_index = finish_index;
    target.reverse = route_reverse;
    if (precision_finish || direction_changes_after_finish ||
      segment_index == last_segment || remaining <= 1.0e-9)
    {
      return target;
    }

    ++segment_index;
    segment_t = 0.0;
  }

  return target;
}

// 精准点不能被窗口投影直接跨过去；只有车真的到它半径内，才允许推进到该点。
bool VisualLidarPathFollower::can_advance_to_index(
  std::size_t candidate_index, const VisualLidarPathFollower::RobotPose & pose,
  std::size_t & allowed_index) const
{
  candidate_index = std::min(candidate_index, waypoints_.size() - 1);
  allowed_index = candidate_index;
  for (std::size_t index = current_index_ + 1; index <= candidate_index; ++index) {
    if (!is_precision_waypoint(index)) {
      continue;
    }

    const auto & waypoint = waypoints_[index];
    const double distance = std::hypot(waypoint.x - pose.x, waypoint.y - pose.y);
    if (distance <= waypoint_pass_radius(index)) {
      allowed_index = index;
      return true;
    }

    allowed_index = index > 0 ? index - 1 : 0;
    return allowed_index > current_index_;
  }
  return true;
}

void VisualLidarPathFollower::advance_current_index_to(std::size_t new_index, const char * reason)
{
  new_index = std::min(new_index, waypoints_.size() - 1);
  while (current_index_ < new_index) {
    ++current_index_;
    first_waypoint_reached_ = true;
    if (debug_control_logs_) {
      const auto & waypoint = waypoints_[current_index_];
      RCLCPP_INFO(
        get_logger(), "debug: %s 推进到航点 %zu/%zu（%s）。",
        reason, current_index_ + 1, waypoints_.size(), waypoint.name.c_str());
    }
  }
}

double VisualLidarPathFollower::waypoint_pass_radius(std::size_t index) const
{
  if (index >= waypoints_.size()) {
    return pass_radius_;
  }
  // Task1最后一点正对二维码。它不能沿用普通航点0.60m通过半径，否则
  // 小车会在摄像头尚未获得足够正视时间时提前结束Task1。
  if (following_qr_path_ && index + 1 == waypoints_.size()) {
    return task1_endpoint_arrival_radius_;
  }
  const auto & waypoint = waypoints_[index];
  if (task3_route_available_ && task3_route_active_ &&
    index + 1 == waypoints_.size())
  {
    return task3_terminal_route_end_radius_;
  }
  if (waypoint.reverse) {
    return std::max(waypoint.reverse_pass_radius, reverse_goal_lookahead_distance_);
  }
  return pass_radius_;
}

bool VisualLidarPathFollower::is_segment_reverse(std::size_t segment_index) const
{
  return segment_index + 1 < waypoints_.size() && waypoints_[segment_index + 1].reverse;
}

bool VisualLidarPathFollower::is_precision_waypoint(std::size_t index) const
{
  if (index >= waypoints_.size()) {
    return false;
  }
  // Task1末点必须真正进入专用到达半径：既禁止路线投影提前提交，也禁止
  // 离散前瞻跨过。到点后仍由原有0.30s二维码等待逻辑接管。
  if (following_qr_path_ && index + 1 == waypoints_.size()) {
    return true;
  }
  // Task3原路线末点也必须实际进入专用半径，不能被投影前瞻提前提交；
  // 只有完成这一点后才允许启用不属于路线的(0,0)保底牵引。
  if (task3_route_available_ && task3_route_active_ &&
    index + 1 == waypoints_.size())
  {
    return true;
  }
  return false;
}

// ********************************************************************************
// ******************************** 巡线控制 ********************************
// ********************************************************************************
// 根据离散前瞻 waypoint 计算原始跟随速度，不处理避障。
geometry_msgs::msg::Twist VisualLidarPathFollower::compute_tracking_command(
  const VisualLidarPathFollower::RobotPose & pose, const VisualLidarPathFollower::TargetPoint & target) const
{
  const double dx = target.point.x - pose.x;
  const double dy = target.point.y - pose.y;

  // 将目标向量从 map 坐标系旋转到 base_link 坐标系。
  const double cos_yaw = std::cos(pose.yaw);
  const double sin_yaw = std::sin(pose.yaw);
  const double target_x_base = cos_yaw * dx + sin_yaw * dy;
  const double target_y_base = -sin_yaw * dx + cos_yaw * dy;

  double heading_error = std::atan2(target_y_base, target_x_base);

  geometry_msgs::msg::Twist cmd;

  if (target.reverse) {
    // 倒车时目标方向按车尾方向处理，linear.x 为负。
    heading_error = std::atan2(-target_y_base, -target_x_base);
    cmd.linear.x = -linear_speed_;
    cmd.angular.z = reverse_angular_gain_ * heading_error;
  } else {
    cmd.linear.x = linear_speed_;
    cmd.angular.z = turn_angular_gain_ * heading_error;
  }
  return cmd;
}

void VisualLidarPathFollower::reset_path_pid_state()
{
  path_pid_state_valid_ = false;
  path_pid_lateral_integral_ = 0.0;
  path_pid_last_lateral_error_ = 0.0;
  path_pid_filtered_lateral_derivative_ = 0.0;
  path_pid_last_angular_command_ = 0.0;
}

void VisualLidarPathFollower::rebuild_path_pid_route_cache()
{
  path_pid_route_s_cache_.assign(waypoints_.size(), 0.0);
  for (std::size_t index = 1; index < waypoints_.size(); ++index) {
    path_pid_route_s_cache_[index] = path_pid_route_s_cache_[index - 1] + std::hypot(
      waypoints_[index].x - waypoints_[index - 1].x,
      waypoints_[index].y - waypoints_[index - 1].y);
  }
}

// smac_pid是本节点的配置模式名，实际控制由三部分组成：
//   1. 路线曲率前馈 g*v*kappa；
//   2. 路线切向与车身的航向误差P；
//   3. 车体相对路线的有符号横向误差PID。
// 只接管正向普通路线；倒车和特殊动作继续使用原控制器。
bool VisualLidarPathFollower::compute_path_pid_tracking_command(
  const VisualLidarPathFollower::RobotPose & pose,
  geometry_msgs::msg::Twist & cmd)
{
  if (waypoints_.size() < 3 || current_index_ >= waypoints_.size() - 1) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "smac_pid失败[route_unavailable]：路线点=%zu，current_index=%zu；"
      "需要至少3个路线点且索引必须位于最后一点之前；"
      "本周期输出零速，禁止回退legacy_lookahead。",
      waypoints_.size(), current_index_);
    reset_path_pid_state();
    return false;
  }

  RouteProjection projection;
  if (!find_route_projection(pose, projection)) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "smac_pid失败[route_geometry_unavailable]："
      "路线点=%zu，current_index=%zu，局部搜索后=%.2fm/前=%.2fm；"
      "局部窗口内没有任何非零长度线段，无法计算路线几何。"
      "本周期输出零速，禁止回退legacy_lookahead。",
      waypoints_.size(), current_index_,
      route_projection_search_backward_distance_,
      route_projection_search_forward_distance_);
    reset_path_pid_state();
    return false;
  }

  if (path_pid_route_s_cache_.size() != waypoints_.size()) {
    rebuild_path_pid_route_cache();
  }
  const auto & route_s = path_pid_route_s_cache_;
  const std::size_t projection_segment = std::min(
    projection.segment_index, waypoints_.size() - 2);
  const double projection_t = std::clamp(projection.t, 0.0, 1.0);
  const double segment_length = route_s[projection_segment + 1] -
    route_s[projection_segment];
  if (segment_length < 1.0e-6) {
    const double geometric_length = std::hypot(
      waypoints_[projection_segment + 1].x - waypoints_[projection_segment].x,
      waypoints_[projection_segment + 1].y - waypoints_[projection_segment].y);
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "smac_pid失败[zero_length_segment]：投影线段=%zu，"
      "弧长缓存长度=%.9fm，当前几何长度=%.9fm，cache=%zu/route=%zu；"
      "可能存在连续重复点或路线更新后弧长缓存未同步；"
      "本周期输出零速，禁止回退legacy_lookahead。",
      projection_segment, segment_length, geometric_length,
      route_s.size(), waypoints_.size());
    reset_path_pid_state();
    return false;
  }
  const double projection_s = route_s[projection_segment] +
    projection_t * segment_length;

  // 不让曲率窗口越过正向/倒车分界，避免前视到另一套运动学定义。
  std::size_t forward_last_segment = projection_segment;
  while (forward_last_segment + 1 < waypoints_.size() - 1 &&
    !is_segment_reverse(forward_last_segment + 1))
  {
    ++forward_last_segment;
  }
  std::size_t backward_first_segment = projection_segment;
  while (backward_first_segment > 0 &&
    !is_segment_reverse(backward_first_segment - 1))
  {
    --backward_first_segment;
  }
  const double minimum_sample_s = route_s[backward_first_segment];
  const double maximum_sample_s = route_s[forward_last_segment + 1];

  auto sample_route = [&](double requested_s) {
      const double sample_s = std::clamp(
        requested_s, minimum_sample_s, maximum_sample_s);
      auto upper = std::upper_bound(route_s.begin(), route_s.end(), sample_s);
      std::size_t finish_index = upper == route_s.end() ?
        waypoints_.size() - 1 : static_cast<std::size_t>(upper - route_s.begin());
      finish_index = std::max<std::size_t>(1, finish_index);
      const std::size_t start_index = finish_index - 1;
      const double length = route_s[finish_index] - route_s[start_index];
      const double t = length > 1.0e-9 ?
        (sample_s - route_s[start_index]) / length : 0.0;
      return Point2D{
        waypoints_[start_index].x +
        t * (waypoints_[finish_index].x - waypoints_[start_index].x),
        waypoints_[start_index].y +
        t * (waypoints_[finish_index].y - waypoints_[start_index].y)};
    };

  const double preview_distance = std::min(
    path_pid_preview_max_,
    path_pid_preview_base_ +
    path_pid_preview_time_ * std::abs(latest_chassis_linear_speed_));
  const double curvature_center_s = std::min(
    projection_s + preview_distance, maximum_sample_s);
  const double half_window = 0.5 * path_pid_curvature_window_length_;
  const Point2D curvature_before = sample_route(curvature_center_s - half_window);
  const Point2D curvature_center = sample_route(curvature_center_s);
  const Point2D curvature_after = sample_route(curvature_center_s + half_window);

  const double chord_ab = std::hypot(
    curvature_center.x - curvature_before.x,
    curvature_center.y - curvature_before.y);
  const double chord_bc = std::hypot(
    curvature_after.x - curvature_center.x,
    curvature_after.y - curvature_center.y);
  const double chord_ac = std::hypot(
    curvature_after.x - curvature_before.x,
    curvature_after.y - curvature_before.y);
  double curvature = 0.0;
  const double curvature_denominator = chord_ab * chord_bc * chord_ac;
  if (curvature_denominator > 1.0e-8) {
    const double cross =
      (curvature_center.x - curvature_before.x) *
      (curvature_after.y - curvature_before.y) -
      (curvature_center.y - curvature_before.y) *
      (curvature_after.x - curvature_before.x);
    curvature = std::clamp(
      2.0 * cross / curvature_denominator,
      -path_pid_max_curvature_, path_pid_max_curvature_);
  }

  double tangent_x = curvature_after.x - curvature_before.x;
  double tangent_y = curvature_after.y - curvature_before.y;
  double tangent_length = std::hypot(tangent_x, tangent_y);
  if (tangent_length < 1.0e-6) {
    tangent_x = waypoints_[projection_segment + 1].x -
      waypoints_[projection_segment].x;
    tangent_y = waypoints_[projection_segment + 1].y -
      waypoints_[projection_segment].y;
    tangent_length = std::hypot(tangent_x, tangent_y);
  }
  if (tangent_length < 1.0e-6) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "smac_pid失败[invalid_tangent]：投影线段=%zu，projection_s=%.3f，"
      "曲率中心s=%.3f，采样窗口=%.3fm，采样弦长[AB=%.9f, BC=%.9f, AC=%.9f]m；"
      "前视采样和当前线段均无法形成有效切线，可能存在重复点、回折或路线末端退化；"
      "本周期输出零速，禁止回退legacy_lookahead。",
      projection_segment, projection_s, curvature_center_s,
      path_pid_curvature_window_length_, chord_ab, chord_bc, chord_ac);
    reset_path_pid_state();
    return false;
  }
  tangent_x /= tangent_length;
  tangent_y /= tangent_length;

  const double heading_error = normalize_angle(
    std::atan2(tangent_y, tangent_x) - pose.yaw);
  const double projection_tangent_x =
    (waypoints_[projection_segment + 1].x -
    waypoints_[projection_segment].x) / segment_length;
  const double projection_tangent_y =
    (waypoints_[projection_segment + 1].y -
    waypoints_[projection_segment].y) / segment_length;
  const double lateral_error =
    projection_tangent_x * (pose.y - projection.point.y) -
    projection_tangent_y * (pose.x - projection.point.x);
  const double dt = 1.0 / std::max(control_frequency_, 1.0);
  double raw_lateral_derivative = 0.0;
  if (path_pid_state_valid_) {
    raw_lateral_derivative =
      (lateral_error - path_pid_last_lateral_error_) / dt;
    path_pid_lateral_integral_ = std::clamp(
      path_pid_lateral_integral_ + lateral_error * dt,
      -path_pid_integral_limit_, path_pid_integral_limit_);
  } else {
    path_pid_lateral_integral_ = 0.0;
    path_pid_filtered_lateral_derivative_ = 0.0;
  }
  path_pid_filtered_lateral_derivative_ =
    path_pid_derivative_filter_alpha_ * raw_lateral_derivative +
    (1.0 - path_pid_derivative_filter_alpha_) *
    path_pid_filtered_lateral_derivative_;

  // smac_pid只用路线曲率计算转向，不再用曲率计算速度。这里先输出当前
  // 阶段速度，控制主循环随后复用legacy_lookahead的|angular.z|转弯降速。
  const double nominal_speed = std::max(0.0, linear_speed_);
  const double linear_command = nominal_speed;

  const double yaw_scale = curvature >= 0.0 ?
    path_pid_yaw_scale_left_ : path_pid_yaw_scale_right_;
  double angular_command =
    yaw_scale * linear_command * curvature +
    path_pid_heading_kp_ * heading_error -
    path_pid_lateral_kp_ * lateral_error -
    path_pid_lateral_ki_ * path_pid_lateral_integral_ -
    path_pid_lateral_kd_ * path_pid_filtered_lateral_derivative_;
  const double kinematic_angular_limit = std::max(
    0.30, yaw_scale * linear_command * path_pid_max_curvature_);
  const double angular_limit = std::min(
    path_pid_absolute_angular_limit_, kinematic_angular_limit);
  angular_command = std::clamp(
    angular_command, -angular_limit, angular_limit);
  if (path_pid_state_valid_ && path_pid_max_angular_rate_ > 0.0) {
    const double angular_step = path_pid_max_angular_rate_ * dt;
    angular_command = std::clamp(
      angular_command,
      path_pid_last_angular_command_ - angular_step,
      path_pid_last_angular_command_ + angular_step);
  }

  cmd = geometry_msgs::msg::Twist();
  cmd.linear.x = linear_command;
  cmd.angular.z = angular_command;
  path_pid_state_valid_ = true;
  path_pid_last_lateral_error_ = lateral_error;
  path_pid_last_angular_command_ = angular_command;

  if (debug_control_logs_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 500,
      "smac_pid: ey=%.3f, e_heading=%.3f, kappa=%.3f, preview=%.3f, cmd=%.3f/%.3f。",
      lateral_error, heading_error, curvature, preview_distance,
      cmd.linear.x, cmd.angular.z);
  }
  return true;
}

// 沿用原巡点控制：根据即将下发的|cmd.angular.z|线性降速。这里的
// angular.z是底盘角速度指令，不是舵机的物理前轮转角；左右取绝对值后
// 使用相同降速比例，舵机映射仍由下位机处理。
void VisualLidarPathFollower::apply_turn_speed_limit(
  geometry_msgs::msg::Twist & cmd) const
{
  if (!enable_turn_speed_slowdown_ || std::abs(cmd.linear.x) <= 1.0e-9) {
    return;
  }

  const double turn_ratio = std::clamp(
    std::abs(cmd.angular.z) / turn_speed_full_steering_angle_, 0.0, 1.0);
  const double reduction_ratio = max_turn_speed_reduction_percent_ / 100.0;
  cmd.linear.x *= 1.0 - reduction_ratio * turn_ratio;
}
