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
 *   3. /scan：LaserScan，用于前方障碍转向避让，不再修改线速度。
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

// 将 value 限制在 [low, high] 区间内，常用于限速、限角速度和比例系数裁剪。
double clamp_value(double value, double low, double high)
{
  return std::max(low, std::min(high, value));
}

// 将任意角度归一化到 [-pi, pi]，方便比较雷达扇区和航向误差。
double normalize_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

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
  double front_angle_deg = front_angle_ * 180.0 / M_PI;
  double side_angle_deg = side_angle_ * 180.0 / M_PI;

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

  // 基础追点控制
  declare_parameter<double>("control_frequency", control_frequency);
  get_parameter("control_frequency", control_frequency);
  declare_parameter<double>("linear_speed", linear_speed_);
  get_parameter("linear_speed", linear_speed_);
  declare_parameter<double>("lookahead_distance", lookahead_distance_);
  get_parameter("lookahead_distance", lookahead_distance_);
  declare_parameter<bool>("enable_lookahead_waypoint_selection", enable_lookahead_waypoint_selection_);
  get_parameter("enable_lookahead_waypoint_selection", enable_lookahead_waypoint_selection_);
  declare_parameter<bool>("enable_lookahead_progress_commit", enable_lookahead_progress_commit_);
  get_parameter("enable_lookahead_progress_commit", enable_lookahead_progress_commit_);
  declare_parameter<double>("pass_radius", pass_radius_);
  get_parameter("pass_radius", pass_radius_);
  declare_parameter<double>("goal_tolerance", goal_tolerance_);
  get_parameter("goal_tolerance", goal_tolerance_);

  // 冲过头跳点保护
  declare_parameter<bool>("enable_skip_overshot_waypoints", enable_skip_overshot_waypoints_);
  get_parameter("enable_skip_overshot_waypoints", enable_skip_overshot_waypoints_);
  declare_parameter<double>("overshot_waypoint_margin", overshot_waypoint_margin_);
  get_parameter("overshot_waypoint_margin", overshot_waypoint_margin_);
  declare_parameter<double>("overshot_waypoint_max_distance", overshot_waypoint_max_distance_);
  get_parameter("overshot_waypoint_max_distance", overshot_waypoint_max_distance_);
  declare_parameter<double>("overshot_next_waypoint_max_distance", overshot_next_waypoint_max_distance_);
  get_parameter("overshot_next_waypoint_max_distance", overshot_next_waypoint_max_distance_);
  declare_parameter<int>("skip_overshot_min_waypoint", skip_overshot_min_waypoint_);
  get_parameter("skip_overshot_min_waypoint", skip_overshot_min_waypoint_);
  declare_parameter<int>("skip_overshot_max_waypoint", skip_overshot_max_waypoint_);
  get_parameter("skip_overshot_max_waypoint", skip_overshot_max_waypoint_);

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

  // 雷达避障
  declare_parameter<bool>("enable_obstacle_avoidance", enable_obstacle_avoidance_);
  get_parameter("enable_obstacle_avoidance", enable_obstacle_avoidance_);
  declare_parameter<int>("obstacle_enable_from_waypoint", obstacle_enable_from_waypoint_);
  get_parameter("obstacle_enable_from_waypoint", obstacle_enable_from_waypoint_);
  declare_parameter<double>("obstacle_avoid_distance", obstacle_avoid_distance_);
  get_parameter("obstacle_avoid_distance", obstacle_avoid_distance_);
  declare_parameter<double>("min_obstacle_range", min_obstacle_range_);
  get_parameter("min_obstacle_range", min_obstacle_range_);
  declare_parameter<double>("front_angle_deg", front_angle_deg);
  get_parameter("front_angle_deg", front_angle_deg);
  declare_parameter<double>("side_angle_deg", side_angle_deg);
  get_parameter("side_angle_deg", side_angle_deg);
  declare_parameter<double>("avoid_angular_z", avoid_angular_z_);
  get_parameter("avoid_angular_z", avoid_angular_z_);

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
  overshot_waypoint_margin_ = std::max(0.0, overshot_waypoint_margin_);
  overshot_waypoint_max_distance_ = std::max(0.0, overshot_waypoint_max_distance_);
  overshot_next_waypoint_max_distance_ = std::max(0.0, overshot_next_waypoint_max_distance_);
  skip_overshot_min_waypoint_ = std::max(1, skip_overshot_min_waypoint_);
  skip_overshot_max_waypoint_ = std::max(skip_overshot_min_waypoint_, skip_overshot_max_waypoint_);
  reverse_approach_nudge_waypoint_ = std::max(1, reverse_approach_nudge_waypoint_);
  reverse_approach_nudge_distance_ = std::max(0.0, reverse_approach_nudge_distance_);
  obstacle_enable_from_waypoint_ = std::max(0, obstacle_enable_from_waypoint_);
  image_analysis_waypoint_ = std::max(1, image_analysis_waypoint_);
  front_angle_ = front_angle_deg * M_PI / 180.0;
  side_angle_ = side_angle_deg * M_PI / 180.0;

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
    "Loaded %zu waypoint(s) (%zu reverse), route=%s, lookahead=%.2fm, waypoint_lookahead=%s, pass_radius=%.2fm, linear_speed=%.2fm/s, obstacle_from_waypoint=%d, obstacle_avoidance=%s.",
    waypoints_.size(), static_cast<std::size_t>(reverse_count), selected_route_direction_.c_str(), lookahead_distance_,
    enable_lookahead_waypoint_selection_ ? "true" : "false", pass_radius_,
    linear_speed_, obstacle_enable_from_waypoint_,
    enable_obstacle_avoidance_ ? "true" : "false");

  // 定时控制循环：每次读取 TF、更新航点进度、算 cmd_vel、应用避障，然后发布速度。
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

// 主控制循环：拿位姿 -> 判断终点 -> 推进航点 -> 选追踪目标 -> 算速度 -> 叠加避障 -> 发布。
// 这里保留完整顺序，避免为了几行赋值再跳到小包装函数里。
void SmoothPathFollower::control_once()
{
  RobotPose pose;
  if (!lookup_robot_pose(pose.x, pose.y, pose.yaw)) {
    set_control_state(ControlState::WAITING_FOR_POSE);
    publish_stop();
    return;
  }

  if (stop_if_final_goal_reached(pose)) {
    return;
  }

  next_control_state_ = ControlState::PATH_FOLLOWING;

  advance_passed_waypoints(pose.x, pose.y);

  const std::size_t target_index = select_lookahead_waypoint(pose.x, pose.y);
  commit_lookahead_progress(target_index);
  const auto & target = waypoints_[target_index];

  auto cmd = compute_tracking_command(pose, target, target_index);

  cmd = apply_obstacle_avoidance(cmd, target.reverse);
  set_control_state(next_control_state_);
  cmd_pub_->publish(cmd);
}

// 终点使用 goal_tolerance_ 判定，避免最后一个点因为 pass_radius 逻辑继续向后切。
bool SmoothPathFollower::stop_if_final_goal_reached(const SmoothPathFollower::RobotPose & pose)
{
  const auto & final = waypoints_.back();
  const double final_distance = std::hypot(final.x - pose.x, final.y - pose.y);
  if (current_index_ >= waypoints_.size() - 1 && final_distance <= goal_tolerance_) {
    set_control_state(ControlState::FINISHED);
    RCLCPP_INFO(get_logger(), "Final waypoint reached; stopping.");
    publish_stop();
    rclcpp::shutdown();
    return true;
  }
  return false;
}

// ********************************************************************************
// ******************************** 航点进度 ********************************
// ********************************************************************************

// 按 pass_radius 更新 current_index_。
// 只要机器人进入当前航点通过半径，就认为该点完成并切到下一个点。
void SmoothPathFollower::advance_passed_waypoints(double robot_x, double robot_y)
{
  while (current_index_ < waypoints_.size() - 1) {
    const auto & waypoint = waypoints_[current_index_];
    const double distance = std::hypot(waypoint.x - robot_x, waypoint.y - robot_y);
    const double pass_radius = waypoint.reverse ?
      std::max(waypoint.reverse_pass_radius, reverse_goal_lookahead_distance_) : waypoint.pass_radius;
    if (distance > pass_radius) {
      if (should_skip_overshot_waypoint(robot_x, robot_y, distance)) {
        const auto & next = waypoints_[current_index_ + 1];
        const double next_distance = std::hypot(next.x - robot_x, next.y - robot_y);
        RCLCPP_WARN(
          get_logger(),
          "Skipping overshot waypoint %zu/%zu: current_distance=%.2fm, next_distance=%.2fm.",
          current_index_ + 1, waypoints_.size(), distance, next_distance);
        trigger_image_analysis_if_needed(current_index_ + 1);
        ++current_index_;
        continue;
      }
      break;
    }
    RCLCPP_INFO(get_logger(), "Passing waypoint %zu/%zu at distance %.2fm.", current_index_ + 1, waypoints_.size(), distance);
    trigger_image_analysis_if_needed(current_index_ + 1);
    ++current_index_;
  }
}

// 判断机器人是否已经“冲过”当前航点。
// 典型场景：车因为速度/避障绕行没进入当前点 pass_radius，但已经离下一个点更近。
// 返回 true 时 advance_passed_waypoints() 会跳过当前点，避免车辆回头找点。
bool SmoothPathFollower::should_skip_overshot_waypoint(double robot_x, double robot_y, double current_distance) const
{
  // 没开功能，或当前点已经是最后一个点，就不能跳。
  if (!enable_skip_overshot_waypoints_ || current_index_ + 1 >= waypoints_.size()) {
    return false;
  }

  // 只允许在配置的航点编号范围内跳点，避免普通路段误跳。
  const int waypoint_number = static_cast<int>(current_index_) + 1;
  if (waypoint_number < skip_overshot_min_waypoint_ || waypoint_number > skip_overshot_max_waypoint_) {
    return false;
  }

  // 当前点太远时不认为是“刚刚错过”，否则可能把路径大段跳掉。
  if (current_distance > overshot_waypoint_max_distance_) {
    return false;
  }

  const auto & next = waypoints_[current_index_ + 1];
  const double next_distance = std::hypot(next.x - robot_x, next.y - robot_y);
  if (next_distance > overshot_next_waypoint_max_distance_) {
    return false;
  }
  return next_distance + overshot_waypoint_margin_ < current_distance;
}

// 选择本周期要追踪的航点下标。
// 如果不开前瞻，就追当前 current_index_。
// 如果开前瞻，就从 current_index_ 往后找第一个距离达到 lookahead_distance_ 的点，
// 让车提前朝远一点的点转向，从而减少折线感。
std::size_t SmoothPathFollower::select_lookahead_waypoint(double robot_x, double robot_y) const
{
  if (!enable_lookahead_waypoint_selection_) {
    return current_index_;
  }

  // 精准点需要真正贴近，例如起步点、二维码后倒车点和终点，不做前瞻跳过。
  if (current_index_ < waypoints_.size() && is_precision_waypoint(current_index_)) {
    return current_index_;
  }

  std::size_t target_index = current_index_;
  for (std::size_t index = current_index_; index < waypoints_.size(); ++index) {
    const auto & waypoint = waypoints_[index];
    const double distance = std::hypot(waypoint.x - robot_x, waypoint.y - robot_y);

    double lookahead = lookahead_distance_;
    if (waypoint.reverse && index == current_index_) {
      lookahead = std::min(lookahead_distance_, reverse_goal_lookahead_distance_);
    }

    // 前方如果遇到精准点，直接把它作为目标，让车辆认真到位。
    if (index > current_index_ && is_precision_waypoint(index)) {
      target_index = index;
      break;
    }

    if (distance >= lookahead) {
      target_index = index;
      break;
    }

    target_index = index;
  }
  return target_index;
}

// 哪些点必须精确通过，不允许被前瞻逻辑提前跳过。
// 当前策略：前两个点和最后一个点为精准点。
bool SmoothPathFollower::is_precision_waypoint(std::size_t index) const
{
  if (index >= waypoints_.size()) {
    return false;
  }
  return index < 2 || index == waypoints_.size() - 1;
}

// 当前瞻目标已经超过 current_index_ 时，把中间航点直接标记为完成。
// 这会让“看远一点”的目标也推进路线进度，否则车辆会一直被旧 current_index_ 拉回。
void SmoothPathFollower::commit_lookahead_progress(std::size_t target_index)
{
  if (!enable_lookahead_progress_commit_ || target_index <= current_index_) {
    return;
  }

  target_index = std::min(target_index, waypoints_.size() - 1);
  while (current_index_ < target_index) {
    RCLCPP_INFO(
      get_logger(), "Lookahead committed waypoint %zu/%zu; continuing toward waypoint %zu/%zu.",
      current_index_ + 1, waypoints_.size(), target_index + 1, waypoints_.size());
    trigger_image_analysis_if_needed(current_index_ + 1);
    ++current_index_;
  }
}

// ********************************************************************************
// ******************************** 追点控制 ********************************
// ********************************************************************************

// 根据目标航点计算原始跟随速度，不处理避障。
geometry_msgs::msg::Twist SmoothPathFollower::compute_tracking_command(
  const SmoothPathFollower::RobotPose & pose, const Waypoint & target, std::size_t target_index) const
{
  const double dx = target.x - pose.x;
  const double dy = target.y - pose.y;

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
    cmd = apply_reverse_approach_nudge(cmd, target_index, distance);
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
// ******************************** 障碍转向 ********************************
// ********************************************************************************

// 叠加真实雷达避障。
// 输入 cmd 是路径跟随算出的原始速度；返回值只会叠加角速度，不再改线速度。
// reverse 表示当前路径点是否倒车，倒车时“前方雷达扇区”会切到车尾方向。
geometry_msgs::msg::Twist SmoothPathFollower::apply_obstacle_avoidance(geometry_msgs::msg::Twist cmd, bool reverse)
{
  if (!enable_obstacle_avoidance_) {
    return cmd;
  }

  // obstacle_enable_from_waypoint_ 允许前几个点不启用避障，常用于起步/扫码等特殊动作。
  if (static_cast<int>(current_index_) + 1 < obstacle_enable_from_waypoint_) {
    return cmd;
  }

  if (!latest_scan_) {
    return cmd;
  }

  double front_min = 0.0;
  double left_min = 0.0;
  double right_min = 0.0;
  if (reverse) {
    // 倒车时，车尾方向才是“运动前方”，所以采样角度移动到 pi 附近。
    front_min = min_range(M_PI - front_angle_, M_PI + front_angle_);
    left_min = min_range(M_PI, M_PI + side_angle_);
    right_min = min_range(M_PI - side_angle_, M_PI);
  } else {
    // 正向时，0 度附近是车头前方，正角度为左侧，负角度为右侧。
    front_min = min_range(-front_angle_, front_angle_);
    left_min = min_range(0.0, side_angle_);
    right_min = min_range(-side_angle_, 0.0);
  }

  // 如果前方完全没有有效雷达点，就沿用原始跟随速度。
  if (!std::isfinite(front_min)) {
    return cmd;
  }

  // 避让区：在原始跟随角速度上叠加朝空旷侧转的角速度，线速度保持不变。
  if (front_min <= obstacle_avoid_distance_) {
    next_control_state_ = ControlState::OBSTACLE_AVOIDING;
    const double avoid_direction = avoid_direction_from_ranges(left_min, right_min);
    const double avoid_gain = clamp_value(
      (obstacle_avoid_distance_ - front_min) / std::max(obstacle_avoid_distance_, 0.01),
      0.0, 1.0);
    cmd.angular.z += avoid_direction * avoid_angular_z_ * avoid_gain;
    log_obstacle_warning(
      "Obstacle avoid: front=" + std::to_string(front_min) + "m, left=" +
      std::to_string(left_min) + "m, right=" + std::to_string(right_min) + "m.");
  }
  return cmd;
}

// 根据左右最近障碍距离选择避让方向。
// 返回值会乘到 avoid_angular_z_ 上：1.0 叠加正角速度，-1.0 叠加负角速度。
// 注意这里保留当前代码原有策略；若实车左右转向不符合预期，优先核查这几个分支。
double SmoothPathFollower::avoid_direction_from_ranges(double left_min, double right_min) const
{
  // 两边都没看到障碍时默认正角速度方向。
  if (!std::isfinite(left_min) && !std::isfinite(right_min)) {
    return 1.0;
  }
  // 只有左侧没有有效点时，按原策略返回负角速度方向。
  if (!std::isfinite(left_min)) {
    return -1.0;
  }
  // 只有右侧没有有效点时，按原策略返回正角速度方向。
  if (!std::isfinite(right_min)) {
    return 1.0;
  }
  // 两边都有读数时，左侧距离更大返回正角速度，否则返回负角速度。
  return left_min >= right_min ? 1.0 : -1.0;
}

// 读取 LaserScan 在 [min_angle, max_angle] 扇区内的最近有效距离。
// 角度单位是弧度，0 表示车头方向；返回 infinity 表示该扇区没有有效点。
double SmoothPathFollower::min_range(double min_angle, double max_angle) const
{
  if (!latest_scan_) {
    return std::numeric_limits<double>::infinity();
  }
  const auto & scan = *latest_scan_;
  double result = std::numeric_limits<double>::infinity();
  double angle = scan.angle_min;

  // 小于雷达自身 range_min 或 min_obstacle_range_ 的点都忽略，过滤近距离噪声。
  const double min_valid_range = std::max(static_cast<double>(scan.range_min), min_obstacle_range_);
  min_angle = normalize_angle(min_angle);
  max_angle = normalize_angle(max_angle);
  for (const auto value : scan.ranges) {
    const double scan_angle = normalize_angle(angle);

    // 扇区可能跨越 -pi/pi 边界；跨界时用 “>= min 或 <= max” 判断。
    const bool in_sector = min_angle <= max_angle ?
      (min_angle <= scan_angle && scan_angle <= max_angle) :
      (scan_angle >= min_angle || scan_angle <= max_angle);
    if (in_sector && std::isfinite(value) && min_valid_range <= value && value <= scan.range_max) {
      result = std::min(result, static_cast<double>(value));
    }
    angle += scan.angle_increment;
  }
  return result;
}

// 避障日志节流。控制循环频率较高，直接每周期 WARN 会刷屏，所以每 10 次打印 1 次。
void SmoothPathFollower::log_obstacle_warning(const std::string & message)
{
  ++obstacle_warning_count_;
  if (obstacle_warning_count_ % 10 == 1) {
    RCLCPP_WARN(get_logger(), "%s", message.c_str());
  }
}

// ********************************************************************************
// ******************************** QR 与图生文触发 ********************************
// ********************************************************************************

// QR 回调：
//   1. 根据 QR 数字判断顺/逆时针并切换路线；
//   2. 如果 enable_qr_skip_first_ 为 true，第一次收到 QR 时直接跳到当前路线第 2 点。
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
  RCLCPP_INFO(
    get_logger(), "QR selected %s route; continuing from waypoint %zu/%zu.",
    selected_route_direction_.c_str(), current_index_ + 1, waypoints_.size());
}

// 第一次收到 QR 时，如果还在第 1 个点，就直接把当前目标改为第 2 个点。
// 这个逻辑常用于比赛中“扫码后开始倒车去 2 号点”的动作。
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
    current_index_ = 1;
    qr_skip_done_ = true;
    RCLCPP_INFO(
      get_logger(), "QR info received \"%s\"; switching directly to waypoint 2 (%s).",
      data.c_str(), waypoints_[current_index_].reverse ? "reverse" : "forward");
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
