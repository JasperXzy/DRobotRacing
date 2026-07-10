#include "origincar_system/smooth_path_follower.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <exception>
#include <functional>
#include <limits>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "tf2/time.h"

/*
 * smooth_path_follower.cpp
 *
 * 作用：
 *   这是 test/dev_ws 里的 91 点路径跟随节点。它不调用 Nav2 action，
 *   而是直接根据 map 坐标系下的航点、当前机器人位姿和雷达障碍物信息，
 *   计算 geometry_msgs/Twist 并发布到 /cmd_vel。
 *
 * 输入：
 *   1. YAML 航点文件：包含顺时针/逆时针路线，每个点至少有 x、y。
 *   2. TF：frame_id -> base_frame_id，默认 map -> base_link。
   *   3. /scan：LaserScan，用于检查前方矩形通行区并临时接管转向。
 *   4. /qr_info 或 /qr_code：可选 QR 数字，用于选择顺/逆时针路线。
 *
 * 输出：
 *   1. /cmd_vel：底盘速度命令。
 *   2. /person_trigger 或 /sign4return：到指定航点时触发图像分析/图生文。
 *
 * 坐标单位：
 *   ROS map 坐标默认单位为米；速度单位 m/s，角速度单位 rad/s。
 */

namespace
{

// 从 YAML 节点里读取布尔值；兼容 YAML bool 和字符串形式。
bool yaml_bool(const YAML::Node & node, bool default_value = false)
{
  if (!node) {
    return default_value;
  }
  try {
    return node.as<bool>();
  } catch (const YAML::Exception &) {
    try {
      const auto text = node.as<std::string>();
      return text == "1" || text == "true" || text == "True" || text == "TRUE" ||
             text == "yes" || text == "on" || text == "ON";
    } catch (const YAML::Exception &) {
      return default_value;
    }
  }
}

// 将字符串转成小写，用于 route_direction 等参数的宽松匹配。
std::string to_lower(std::string text)
{
  std::transform(text.begin(), text.end(), text.begin(), [](unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  return text;
}

}  // namespace

// 构造函数完成参数声明、参数读取、航点加载、话题订阅/发布和控制定时器创建。
SmoothPathFollower::SmoothPathFollower()
: rclcpp::Node("smooth_path_follower"),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  double control_frequency = 40.0;

  // 坐标系与话题
  declare_parameter<std::string>("waypoints_file", waypoints_file_);
  get_parameter("waypoints_file", waypoints_file_);
  declare_parameter<std::string>("frame_id", frame_id_);
  get_parameter("frame_id", frame_id_);
  declare_parameter<std::string>("base_frame_id", base_frame_id_);
  get_parameter("base_frame_id", base_frame_id_);
  declare_parameter<std::string>("cmd_vel_topic", cmd_vel_topic_);
  get_parameter("cmd_vel_topic", cmd_vel_topic_);
  declare_parameter<std::string>("scan_topic", scan_topic_);
  get_parameter("scan_topic", scan_topic_);

  // 基础巡线控制
  declare_parameter<double>("control_frequency", control_frequency);
  get_parameter("control_frequency", control_frequency);
  declare_parameter<double>("linear_speed", linear_speed_);
  get_parameter("linear_speed", linear_speed_);
  declare_parameter<double>("lookahead_distance", lookahead_distance_);
  get_parameter("lookahead_distance", lookahead_distance_);
  declare_parameter<double>("pass_radius", pass_radius_);
  get_parameter("pass_radius", pass_radius_);
  declare_parameter<double>("goal_tolerance", goal_tolerance_);
  get_parameter("goal_tolerance", goal_tolerance_);
  declare_parameter<int>("route_projection_window", route_projection_window_);
  get_parameter("route_projection_window", route_projection_window_);
  declare_parameter<double>("route_projection_max_distance", route_projection_max_distance_);
  get_parameter("route_projection_max_distance", route_projection_max_distance_);
  declare_parameter<double>("segment_advance_t", segment_advance_t_);
  get_parameter("segment_advance_t", segment_advance_t_);

  // 倒车控制
  declare_parameter<double>("turn_angular_gain", turn_angular_gain_);
  get_parameter("turn_angular_gain", turn_angular_gain_);
  declare_parameter<double>("reverse_angular_gain", reverse_angular_gain_);
  get_parameter("reverse_angular_gain", reverse_angular_gain_);
  declare_parameter<double>("reverse_goal_lookahead_distance", reverse_goal_lookahead_distance_);
  get_parameter("reverse_goal_lookahead_distance", reverse_goal_lookahead_distance_);
  declare_parameter<double>("reverse_pass_radius", reverse_pass_radius_);
  get_parameter("reverse_pass_radius", reverse_pass_radius_);
  declare_parameter<bool>("enable_reverse_approach_nudge", enable_reverse_approach_nudge_);
  get_parameter("enable_reverse_approach_nudge", enable_reverse_approach_nudge_);
  declare_parameter<int>("reverse_approach_nudge_waypoint", reverse_approach_nudge_waypoint_);
  get_parameter("reverse_approach_nudge_waypoint", reverse_approach_nudge_waypoint_);
  declare_parameter<double>("reverse_approach_nudge_distance", reverse_approach_nudge_distance_);
  get_parameter("reverse_approach_nudge_distance", reverse_approach_nudge_distance_);
  declare_parameter<double>("reverse_approach_nudge_angular_z", reverse_approach_nudge_angular_z_);
  get_parameter("reverse_approach_nudge_angular_z", reverse_approach_nudge_angular_z_);

  // 雷达矩形通行区避障
  declare_parameter<bool>("enable_obstacle_avoidance", enable_obstacle_avoidance_);
  get_parameter("enable_obstacle_avoidance", enable_obstacle_avoidance_);
  declare_parameter<int>("obstacle_enable_from_waypoint", obstacle_enable_from_waypoint_);
  get_parameter("obstacle_enable_from_waypoint", obstacle_enable_from_waypoint_);
  declare_parameter<double>("obstacle_corridor_length", obstacle_corridor_length_);
  get_parameter("obstacle_corridor_length", obstacle_corridor_length_);
  declare_parameter<double>("obstacle_corridor_half_width", obstacle_corridor_half_width_);
  get_parameter("obstacle_corridor_half_width", obstacle_corridor_half_width_);
  declare_parameter<double>("min_obstacle_range", min_obstacle_range_);
  get_parameter("min_obstacle_range", min_obstacle_range_);
  declare_parameter<int>("obstacle_min_points", obstacle_min_points_);
  get_parameter("obstacle_min_points", obstacle_min_points_);
  declare_parameter<double>("avoid_angular_z", avoid_angular_z_);
  get_parameter("avoid_angular_z", avoid_angular_z_);
  declare_parameter<double>("avoid_linear_speed", avoid_linear_speed_);
  get_parameter("avoid_linear_speed", avoid_linear_speed_);
  declare_parameter<int>("obstacle_clear_frames", obstacle_clear_frames_);
  get_parameter("obstacle_clear_frames", obstacle_clear_frames_);

  // QR 与图生文触发
  declare_parameter<bool>("enable_qr_skip_first", enable_qr_skip_first_);
  get_parameter("enable_qr_skip_first", enable_qr_skip_first_);
  declare_parameter<std::string>("qr_topic", qr_topic_);
  get_parameter("qr_topic", qr_topic_);
  declare_parameter<std::string>("route_direction", route_direction_);
  get_parameter("route_direction", route_direction_);
  declare_parameter<bool>("enable_qr_direction_select", enable_qr_direction_select_);
  get_parameter("enable_qr_direction_select", enable_qr_direction_select_);
  declare_parameter<bool>("use_qr_parity_direction", use_qr_parity_direction_);
  get_parameter("use_qr_parity_direction", use_qr_parity_direction_);
  declare_parameter<int>("clockwise_qr_value", clockwise_qr_value_);
  get_parameter("clockwise_qr_value", clockwise_qr_value_);
  declare_parameter<int>("counterclockwise_qr_value", counterclockwise_qr_value_);
  get_parameter("counterclockwise_qr_value", counterclockwise_qr_value_);
  declare_parameter<bool>("enable_image_analysis_trigger", enable_image_analysis_trigger_);
  get_parameter("enable_image_analysis_trigger", enable_image_analysis_trigger_);
  declare_parameter<int>("image_analysis_waypoint", image_analysis_waypoint_);
  get_parameter("image_analysis_waypoint", image_analysis_waypoint_);
  declare_parameter<std::string>("image_analysis_trigger_topic", image_analysis_trigger_topic_);
  get_parameter("image_analysis_trigger_topic", image_analysis_trigger_topic_);

  route_direction_ = to_lower(route_direction_);
  lookahead_distance_ = std::max(0.05, lookahead_distance_);
  route_projection_window_ = std::max(1, route_projection_window_);
  route_projection_max_distance_ = std::max(0.10, route_projection_max_distance_);
  segment_advance_t_ = std::max(0.50, std::min(1.20, segment_advance_t_));
  reverse_approach_nudge_waypoint_ = std::max(1, reverse_approach_nudge_waypoint_);
  reverse_approach_nudge_distance_ = std::max(0.0, reverse_approach_nudge_distance_);
  obstacle_enable_from_waypoint_ = std::max(0, obstacle_enable_from_waypoint_);
  obstacle_corridor_length_ = std::max(0.20, obstacle_corridor_length_);
  obstacle_corridor_half_width_ = std::max(0.05, obstacle_corridor_half_width_);
  obstacle_min_points_ = std::max(1, obstacle_min_points_);
  avoid_angular_z_ = std::max(0.0, avoid_angular_z_);
  avoid_linear_speed_ = std::max(0.0, avoid_linear_speed_);
  obstacle_clear_frames_ = std::max(1, obstacle_clear_frames_);
  image_analysis_waypoint_ = std::max(1, image_analysis_waypoint_);

  // 按 route_direction_ 从 YAML 中加载初始路线；auto 会读取 YAML 顶层 route_direction。
  waypoints_ = load_waypoints(route_direction_);

  // 发布速度命令，最终由主线底盘驱动订阅 /cmd_vel 执行。
  cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

  // 发布图像分析触发。transient_local 让后启动的订阅者也能收到最后一次触发值。
  image_analysis_trigger_pub_ =
    create_publisher<std_msgs::msg::Int32>(image_analysis_trigger_topic_, rclcpp::QoS(10).transient_local());

  // 保存最近一帧雷达数据，控制循环中用于避障。
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_, 10,
    [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) { latest_scan_ = msg; });

  // 只有需要 QR 选方向或扫到 QR 后跳过第一点时，才订阅 QR 话题。
  if (enable_qr_skip_first_ || enable_qr_direction_select_) {
    qr_sub_ = create_subscription<std_msgs::msg::String>(
      qr_topic_, 10, std::bind(&SmoothPathFollower::qr_callback, this, std::placeholders::_1));
  }

  if (waypoints_.empty()) {
    RCLCPP_ERROR(get_logger(), "No waypoints loaded; smooth path follower will stop.");
    publish_stop();
    return;
  }

  // 只用于启动日志，帮助确认倒车点数量和关键参数。
  const auto reverse_count = std::count_if(
    waypoints_.begin(), waypoints_.end(), [](const Waypoint & w) { return w.reverse; });
  RCLCPP_INFO(
    get_logger(),
    "Loaded %zu waypoint(s) (%zu reverse), route=%s, lookahead=%.2fm, projection_window=%d, projection_max_distance=%.2fm, linear_speed=%.2fm/s, obstacle_from_waypoint=%d, obstacle_avoidance=%s.",
    waypoints_.size(), static_cast<std::size_t>(reverse_count), selected_route_direction_.c_str(), lookahead_distance_,
    route_projection_window_, route_projection_max_distance_, linear_speed_, obstacle_enable_from_waypoint_,
    enable_obstacle_avoidance_ ? "true" : "false");

  // 定时控制循环：每次读取 TF、用路线窗口更新进度、追离散前瞻 waypoint，再按雷达通行区临时接管转向。
  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / std::max(control_frequency, 1.0)),
    std::bind(&SmoothPathFollower::control_once, this));
}

// 立即发布零速度。用于启动失败、定位暂不可用、到达终点或程序退出时停车。
void SmoothPathFollower::publish_stop()
{
  if (cmd_pub_) {
    cmd_pub_->publish(geometry_msgs::msg::Twist());
  }
}

const char * SmoothPathFollower::control_state_name(SmoothPathFollower::ControlState state) const
{
  switch (state) {
    case ControlState::WAITING_FOR_POSE:
      return "WAITING_FOR_POSE";
    case ControlState::PATH_FOLLOWING:
      return "PATH_FOLLOWING";
    case ControlState::OBSTACLE_AVOIDING:
      return "OBSTACLE_AVOIDING";
    case ControlState::FINISHED:
      return "FINISHED";
  }
  return "UNKNOWN";
}

void SmoothPathFollower::set_control_state(SmoothPathFollower::ControlState next_state)
{
  if (control_state_ == next_state) {
    return;
  }
  RCLCPP_INFO(
    get_logger(), "Control state: %s -> %s",
    control_state_name(control_state_), control_state_name(next_state));
  control_state_ = next_state;
}

// ********************************************************************************
// ******************************** 主控制流程 ********************************
// ********************************************************************************

// 主控制循环：拿位姿 -> 更新路线线段进度 -> 算离散前瞻 waypoint 速度 -> 必要时由矩形通行区避障接管 -> 发布。
// 这里保留完整顺序，避免为了几行赋值再跳到小包装函数里。
void SmoothPathFollower::control_once()
{
  RobotPose pose;
  if (!lookup_robot_pose(pose.x, pose.y, pose.yaw)) {
    set_control_state(ControlState::WAITING_FOR_POSE);
    publish_stop();
    return;
  }

  next_control_state_ = ControlState::PATH_FOLLOWING;

  if (!first_waypoint_reached_) {
    const auto & first = waypoints_.front();
    const double first_distance = std::hypot(first.x - pose.x, first.y - pose.y);
    if (first_distance <= waypoint_pass_radius(0)) {
      first_waypoint_reached_ = true;
      RCLCPP_INFO(get_logger(), "Initial waypoint reached; starting segment progress.");
    }
  }

  if (first_waypoint_reached_) {
    update_route_progress(pose);
  }

  if (stop_if_final_goal_reached(pose)) {
    return;
  }

  const auto target = compute_lookahead_target(pose);
  auto cmd = compute_tracking_command(pose, target);
  cmd = apply_corridor_obstacle_avoidance(pose, cmd, target.reverse);

  set_control_state(next_control_state_);
  cmd_pub_->publish(cmd);
}

// 终点使用 goal_tolerance_ 判定；同时要求路线进度已经接近末段，避免闭环路线在起点附近误停。
bool SmoothPathFollower::stop_if_final_goal_reached(const SmoothPathFollower::RobotPose & pose)
{
  const auto & final = waypoints_.back();
  const double final_distance = std::hypot(final.x - pose.x, final.y - pose.y);
  const bool final_segment_active = current_index_ + 1 >= waypoints_.size() - 1;
  if (final_segment_active && final_distance <= goal_tolerance_) {
    current_index_ = waypoints_.size() - 1;
    set_control_state(ControlState::FINISHED);
    RCLCPP_INFO(get_logger(), "Final waypoint reached; stopping.");
    publish_stop();
    rclcpp::shutdown();
    return true;
  }
  return false;
}

// ********************************************************************************
// ******************************** 路线线段进度 ********************************
// ********************************************************************************

// 正常巡线只看当前线段，避免高速时提前跳线；避障已激活时才看后方窗口来跳过被挡住的点。
// current_index_ 表示当前参考线段起点：waypoint[i] -> waypoint[i + 1]。
void SmoothPathFollower::update_route_progress(const SmoothPathFollower::RobotPose & pose)
{
  if (waypoints_.size() < 2 || current_index_ >= waypoints_.size() - 1) {
    return;
  }

  const std::size_t extra_segments = obstacle_avoidance_active_ ?
    static_cast<std::size_t>(route_projection_window_) : 0U;
  RouteProjection projection;
  if (!find_route_projection(pose, extra_segments, projection)) {
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
  advance_current_index_to(allowed_index, "Route projection");
}

// 在 current_index_ 到 current_index_ + extra_segments 的线段窗口里找车的投影。
// 只向前找，不全局找，避免闭环/椭圆路线匹配到另一侧。
bool SmoothPathFollower::find_route_projection(
  const SmoothPathFollower::RobotPose & pose, std::size_t extra_segments,
  SmoothPathFollower::RouteProjection & projection) const
{
  if (waypoints_.size() < 2 || current_index_ >= waypoints_.size() - 1) {
    return false;
  }

  const std::size_t last_segment = waypoints_.size() - 2;
  const std::size_t end_segment = std::min(
    last_segment, current_index_ + extra_segments);
  double best_score = std::numeric_limits<double>::infinity();

  for (std::size_t index = current_index_; index <= end_segment; ++index) {
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
    if (raw_t < -0.25) {
      continue;
    }

    const double t = std::max(0.0, std::min(1.0, raw_t));
    const Point2D projected{start.x + seg_x * t, start.y + seg_y * t};
    const double distance = std::hypot(pose.x - projected.x, pose.y - projected.y);
    if (distance > route_projection_max_distance_) {
      continue;
    }

    const double forward_penalty = 0.06 * static_cast<double>(index - current_index_);
    const double score = distance + forward_penalty;
    if (score < best_score) {
      best_score = score;
      projection.valid = true;
      projection.segment_index = index;
      projection.t = raw_t;
      projection.distance = distance;
      projection.point = projected;
    }
  }

  return projection.valid;
}

// 控制目标是离散 waypoint：从 current_index_ + 1 往后找第一个离车达到 lookahead_distance_ 的点。
SmoothPathFollower::TargetPoint SmoothPathFollower::compute_lookahead_target(
  const SmoothPathFollower::RobotPose & pose) const
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

  const std::size_t next_index = current_index_ + 1;
  std::size_t target_index = next_index;

  // 精准点作为当前线段终点时，先认真追到该点，不用前瞻跨过去。
  if (is_precision_waypoint(next_index)) {
    target.waypoint_index = next_index;
    target.reverse = waypoints_[next_index].reverse;
    target.point = Point2D{waypoints_[next_index].x, waypoints_[next_index].y};
    return target;
  }

  const double lookahead = is_segment_reverse(current_index_) ?
    std::min(lookahead_distance_, reverse_goal_lookahead_distance_) : lookahead_distance_;
  for (std::size_t index = next_index; index < waypoints_.size(); ++index) {
    target_index = index;
    if (index > next_index && is_precision_waypoint(index)) {
      break;
    }
    const auto & waypoint = waypoints_[index];
    const double distance = std::hypot(waypoint.x - pose.x, waypoint.y - pose.y);
    if (distance >= lookahead) {
      break;
    }
  }

  const auto & waypoint = waypoints_[target_index];
  target.waypoint_index = target_index;
  target.reverse = waypoint.reverse;
  target.point = Point2D{waypoint.x, waypoint.y};
  return target;
}

// 精准点不能被窗口投影直接跨过去；只有车真的到它半径内，才允许推进到该点。
bool SmoothPathFollower::can_advance_to_index(
  std::size_t candidate_index, const SmoothPathFollower::RobotPose & pose, std::size_t & allowed_index) const
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

void SmoothPathFollower::advance_current_index_to(std::size_t new_index, const char * reason)
{
  new_index = std::min(new_index, waypoints_.size() - 1);
  while (current_index_ < new_index) {
    ++current_index_;
    first_waypoint_reached_ = true;
    RCLCPP_INFO(
      get_logger(), "%s advanced route progress to waypoint %zu/%zu.",
      reason, current_index_ + 1, waypoints_.size());
    trigger_image_analysis_if_needed(current_index_ + 1);
  }
}

double SmoothPathFollower::waypoint_pass_radius(std::size_t index) const
{
  if (index >= waypoints_.size()) {
    return pass_radius_;
  }
  if (index == waypoints_.size() - 1) {
    return goal_tolerance_;
  }
  const auto & waypoint = waypoints_[index];
  if (waypoint.reverse) {
    return std::max(waypoint.reverse_pass_radius, reverse_goal_lookahead_distance_);
  }
  return waypoint.pass_radius;
}

bool SmoothPathFollower::is_segment_reverse(std::size_t segment_index) const
{
  return segment_index + 1 < waypoints_.size() && waypoints_[segment_index + 1].reverse;
}

// 哪些点必须精确通过，不允许被线段窗口直接跨过。
// 当前策略：第 2 点、最后一个点和图像分析触发点为精准点；第 1 点是路线起点。
bool SmoothPathFollower::is_precision_waypoint(std::size_t index) const
{
  if (index >= waypoints_.size()) {
    return false;
  }
  if (index == 1 || index == waypoints_.size() - 1) {
    return true;
  }
  return enable_image_analysis_trigger_ &&
    index + 1 == static_cast<std::size_t>(image_analysis_waypoint_);
}

// ********************************************************************************
// ******************************** 巡线控制 ********************************
// ********************************************************************************

// 根据离散前瞻 waypoint 计算原始跟随速度，不处理避障。
geometry_msgs::msg::Twist SmoothPathFollower::compute_tracking_command(
  const SmoothPathFollower::RobotPose & pose, const SmoothPathFollower::TargetPoint & target) const
{
  const double dx = target.point.x - pose.x;
  const double dy = target.point.y - pose.y;

  // 将目标向量从 map 坐标系旋转到 base_link 坐标系。
  const double cos_yaw = std::cos(pose.yaw);
  const double sin_yaw = std::sin(pose.yaw);
  const double target_x_base = cos_yaw * dx + sin_yaw * dy;
  const double target_y_base = -sin_yaw * dx + cos_yaw * dy;
  const double distance = std::max(std::hypot(target_x_base, target_y_base), 0.001);

  double heading_error = std::atan2(target_y_base, target_x_base);

  geometry_msgs::msg::Twist cmd;

  if (target.reverse) {
    // 倒车时目标方向按车尾方向处理，linear.x 为负。
    heading_error = std::atan2(-target_y_base, -target_x_base);
    cmd.linear.x = -linear_speed_;
    cmd.angular.z = reverse_angular_gain_ * heading_error;
    cmd = apply_reverse_approach_nudge(cmd, target.waypoint_index, distance);
  } else {
    cmd.linear.x = linear_speed_;
    cmd.angular.z = turn_angular_gain_ * heading_error;
  }
  return cmd;
}

// 倒车接近指定航点时额外打一小段方向，用来修正倒车入点姿态。
geometry_msgs::msg::Twist SmoothPathFollower::apply_reverse_approach_nudge(
  geometry_msgs::msg::Twist cmd, std::size_t target_index, double target_distance) const
{
  const int waypoint_number = static_cast<int>(target_index) + 1;
  if (!enable_reverse_approach_nudge_ || waypoint_number != reverse_approach_nudge_waypoint_) {
    return cmd;
  }
  if (target_distance > reverse_approach_nudge_distance_) {
    return cmd;
  }

  cmd.angular.z = reverse_approach_nudge_angular_z_;
  return cmd;
}

// ********************************************************************************
// ******************************** 矩形通行区避障 ********************************
// ********************************************************************************

// 类似 visual_control 的反应式避障：
// 矩形通行区有障碍时固定往反方向打舵；路线进度仍由每帧线段投影持续更新。
geometry_msgs::msg::Twist SmoothPathFollower::apply_corridor_obstacle_avoidance(
  const SmoothPathFollower::RobotPose & pose, geometry_msgs::msg::Twist cmd, bool reverse)
{
  (void)pose;
  if (!enable_obstacle_avoidance_ || reverse) {
    obstacle_avoidance_active_ = false;
    obstacle_clear_count_ = 0;
    avoid_direction_ = 0;
    return cmd;
  }

  if (static_cast<int>(current_index_) + 1 < obstacle_enable_from_waypoint_) {
    obstacle_avoidance_active_ = false;
    obstacle_clear_count_ = 0;
    avoid_direction_ = 0;
    return cmd;
  }

  CorridorObstacle obstacle;
  const bool blocked = detect_corridor_obstacle(false, obstacle);
  if (blocked) {
    obstacle_avoidance_active_ = true;
    obstacle_clear_count_ = 0;
    avoid_direction_ = choose_avoid_direction(obstacle);
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Corridor obstacle: points=%d, center=(%.2f, %.2f), nearest=%.2fm, steering=%s.",
      obstacle.point_count, obstacle.center.x, obstacle.center.y, obstacle.nearest_range,
      avoid_direction_ > 0 ? "left" : "right");
  } else if (obstacle_avoidance_active_) {
    ++obstacle_clear_count_;
    if (obstacle_clear_count_ >= obstacle_clear_frames_) {
      obstacle_avoidance_active_ = false;
      obstacle_clear_count_ = 0;
      avoid_direction_ = 0;
      return cmd;
    }
  }

  if (!obstacle_avoidance_active_) {
    return cmd;
  }

  next_control_state_ = ControlState::OBSTACLE_AVOIDING;
  cmd.linear.x = avoid_linear_speed_;
  cmd.angular.z = static_cast<double>(avoid_direction_) * avoid_angular_z_;
  return cmd;
}

// 在车体前方定义矩形通行区：0 < x <= length, |y| <= half_width。
// 只要矩形内有效雷达点数量达到阈值，就认为需要避障。
bool SmoothPathFollower::detect_corridor_obstacle(
  bool reverse, SmoothPathFollower::CorridorObstacle & obstacle) const
{
  if (!latest_scan_) {
    return false;
  }

  const auto & scan = *latest_scan_;
  const double min_valid_range = std::max(static_cast<double>(scan.range_min), min_obstacle_range_);
  double angle = scan.angle_min;
  double sum_x = 0.0;
  double sum_y = 0.0;
  obstacle.nearest_range = std::numeric_limits<double>::infinity();

  for (const auto value : scan.ranges) {
    if (
      std::isfinite(value) && min_valid_range <= value && value <= scan.range_max)
    {
      const double base_x = static_cast<double>(value) * std::cos(angle);
      const double base_y = static_cast<double>(value) * std::sin(angle);
      const Point2D motion_point{reverse ? -base_x : base_x, reverse ? -base_y : base_y};
      if (
        motion_point.x > 0.0 && motion_point.x <= obstacle_corridor_length_ &&
        std::abs(motion_point.y) <= obstacle_corridor_half_width_)
      {
        ++obstacle.point_count;
        sum_x += motion_point.x;
        sum_y += motion_point.y;
        obstacle.nearest_range = std::min(
          obstacle.nearest_range, std::hypot(motion_point.x, motion_point.y));
        if (motion_point.y >= 0.0) {
          ++obstacle.left_count;
        } else {
          ++obstacle.right_count;
        }
      }
    }
    angle += scan.angle_increment;
  }

  if (obstacle.point_count < obstacle_min_points_) {
    return false;
  }

  obstacle.center.x = sum_x / static_cast<double>(obstacle.point_count);
  obstacle.center.y = sum_y / static_cast<double>(obstacle.point_count);
  return true;
}

// 正角速度向左，负角速度向右。障碍在左就向右打，障碍在右就向左打。
int SmoothPathFollower::choose_avoid_direction(const SmoothPathFollower::CorridorObstacle & obstacle) const
{
  constexpr double center_deadband = 0.03;
  if (obstacle.center.y > center_deadband) {
    return -1;
  }
  if (obstacle.center.y < -center_deadband) {
    return 1;
  }
  if (avoid_direction_ != 0) {
    return avoid_direction_;
  }
  if (obstacle.left_count != obstacle.right_count) {
    return obstacle.left_count > obstacle.right_count ? -1 : 1;
  }
  return 1;
}

// ********************************************************************************
// ******************************** QR 与图生文触发 ********************************
// ********************************************************************************

// QR 回调：
//   1. 根据 QR 数字判断顺/逆时针并切换路线；
//   2. 如果 enable_qr_skip_first_ 为 true，第一次收到 QR 时确认起步线段已可执行。
void SmoothPathFollower::qr_callback(const std_msgs::msg::String::ConstSharedPtr msg)
{
  const std::string direction = direction_from_qr(msg->data);
  if (!direction.empty()) {
    switch_route(direction);
  }

  if (enable_qr_skip_first_) {
    skip_first_waypoint_from_qr(msg->data);
  }
}

// 将 QR 字符串转换为路线方向。
// use_qr_parity_direction_=true 时：奇数=clockwise，偶数=counterclockwise。
// 否则使用 clockwise_qr_value_ 和 counterclockwise_qr_value_ 做精确匹配。
std::string SmoothPathFollower::direction_from_qr(const std::string & data) const
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

// 切换路线。current_index_ 不清零，而是继续沿用当前进度，避免 QR 重复触发后回到起点。
void SmoothPathFollower::switch_route(const std::string & direction)
{
  if (direction == selected_route_direction_) {
    return;
  }

  const auto new_waypoints = load_waypoints(direction);
  if (new_waypoints.empty()) {
    RCLCPP_WARN(get_logger(), "QR selected route \"%s\", but no waypoints were loaded.", direction.c_str());
    return;
  }

  waypoints_ = new_waypoints;
  selected_route_direction_ = direction;
  current_index_ = std::min(current_index_, waypoints_.size() - 1);
  first_waypoint_reached_ = current_index_ > 0;
  obstacle_avoidance_active_ = false;
  obstacle_clear_count_ = 0;
  avoid_direction_ = 0;
  RCLCPP_INFO(
    get_logger(), "QR selected %s route; continuing from waypoint %zu/%zu.",
    selected_route_direction_.c_str(), current_index_ + 1, waypoints_.size());
}

// 线段巡线里 current_index_=0 表示 point_01 -> point_02。
// QR 到来时只确认 point_01 已处理，让车开始执行这条起步线段。
void SmoothPathFollower::skip_first_waypoint_from_qr(const std::string & data)
{
  if (qr_skip_done_) {
    return;
  }
  if (waypoints_.size() < 2) {
    qr_skip_done_ = true;
    RCLCPP_WARN(get_logger(), "QR skip ignored: fewer than 2 waypoints loaded.");
    return;
  }
  if (current_index_ == 0) {
    first_waypoint_reached_ = true;
    qr_skip_done_ = true;
    RCLCPP_INFO(
      get_logger(), "QR info received \"%s\"; starting segment toward waypoint 2 (%s).",
      data.c_str(), is_segment_reverse(current_index_) ? "reverse" : "forward");
    return;
  }
  qr_skip_done_ = true;
  RCLCPP_INFO(get_logger(), "QR info received \"%s\", but waypoint 1 is already passed; no skip needed.", data.c_str());
}

// 到达指定航点时发布一次图像分析触发消息。
// waypoint_number 是人看的航点编号，从 1 开始；current_index_ 是数组下标，从 0 开始。
void SmoothPathFollower::trigger_image_analysis_if_needed(std::size_t waypoint_number)
{
  // 只触发一次，避免机器人在同一个航点附近反复发布。
  if (!enable_image_analysis_trigger_ || image_analysis_triggered_) {
    return;
  }
  if (waypoint_number != static_cast<std::size_t>(image_analysis_waypoint_)) {
    return;
  }

  std_msgs::msg::Int32 msg;
  msg.data = image_analysis_waypoint_;
  image_analysis_trigger_pub_->publish(msg);
  image_analysis_triggered_ = true;
  RCLCPP_INFO(
    get_logger(), "Reached waypoint %d; triggered Aliyun image analysis on %s.",
    image_analysis_waypoint_, image_analysis_trigger_topic_.c_str());
}

// ********************************************************************************
// ******************************** 航点加载与定位 ********************************
// ********************************************************************************

// 从 YAML 文件加载航点。
// 支持两种格式：
//   1. routes.clockwise / routes.counterclockwise：当前 91 点文件使用这种；
//   2. waypoints：没有路线分组时的兜底格式。
std::vector<Waypoint> SmoothPathFollower::load_waypoints(const std::string & requested_direction)
{
  std::vector<Waypoint> waypoints;
  std::string resolved_direction = requested_direction;
  if (waypoints_file_.empty()) {
    RCLCPP_ERROR(get_logger(), "Parameter waypoints_file is empty.");
    return waypoints;
  }
  try {
    const YAML::Node data = YAML::LoadFile(waypoints_file_);
    std::string direction = to_lower(requested_direction);
    if (direction.empty() || direction == "auto") {
      // auto 时使用 YAML 顶层 route_direction；如果没有，就默认 counterclockwise。
      direction = data["route_direction"] ? to_lower(data["route_direction"].as<std::string>()) : "counterclockwise";
    }
    resolved_direction = direction;
    const YAML::Node routes = data["routes"];
    const YAML::Node items = routes && routes[direction] ? routes[direction] : data["waypoints"];
    if (!items || !items.IsSequence()) {
      return waypoints;
    }
    for (std::size_t i = 0; i < items.size(); ++i) {
      try {
        Waypoint w;
        // 当前跟随器不使用 YAML yaw；方向由当前位姿和目标点实时计算。
        w.x = items[i]["x"].as<double>();
        w.y = items[i]["y"].as<double>();
        w.reverse = yaml_bool(items[i]["reverse"], false);
        w.pass_radius = items[i]["pass_radius"] ? items[i]["pass_radius"].as<double>() : pass_radius_;
        w.reverse_pass_radius =
          items[i]["reverse_pass_radius"] ? items[i]["reverse_pass_radius"].as<double>() : reverse_pass_radius_;
        waypoints.push_back(w);
      } catch (const YAML::Exception & exc) {
        RCLCPP_WARN(get_logger(), "Skipping invalid waypoint #%zu: %s", i, exc.what());
      }
    }
  } catch (const YAML::Exception & exc) {
    RCLCPP_ERROR(get_logger(), "Failed to read waypoint YAML: %s", exc.what());
  }
  if (!waypoints.empty()) {
    selected_route_direction_ = resolved_direction;
  }
  return waypoints;
}

// 从 TF 查询 frame_id_ -> base_frame_id_，默认 map -> base_link。
// 返回 x/y/yaw，其中 yaw 是机器人在 map 坐标系下的航向角。
bool SmoothPathFollower::lookup_robot_pose(double & x, double & y, double & yaw)
{
  try {
    const auto transform = tf_buffer_.lookupTransform(frame_id_, base_frame_id_, tf2::TimePointZero);
    const auto & translation = transform.transform.translation;
    const auto & rotation = transform.transform.rotation;
    x = translation.x;
    y = translation.y;
    yaw = std::atan2(
      2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
      1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z));
    return true;
  } catch (const tf2::TransformException & exc) {
    RCLCPP_WARN(get_logger(), "Waiting for TF %s->%s: %s", frame_id_.c_str(), base_frame_id_.c_str(), exc.what());
    return false;
  }
}

int main(int argc, char ** argv)
{
  // 标准 ROS 2 节点入口：初始化、创建节点、进入事件循环，退出前发布零速度。
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SmoothPathFollower>();
  rclcpp::spin(node);
  node->publish_stop();
  rclcpp::shutdown();
  return 0;
}
