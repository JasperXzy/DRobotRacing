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
 *   这是 test/dev_ws 里的路径跟随节点。它不调用 Nav2 action，
 *   而是直接根据 map 坐标系下的航点、当前机器人位姿和雷达障碍物信息，
 *   计算 geometry_msgs/Twist 并发布到 /cmd_vel。
 *
 * 输入：
 *   1. YAML 航点文件：维护逆时针路线；顺时针由程序反向读取，每个点至少有 x、y。
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

std::string route_direction_name(const std::string & direction)
{
  const auto lower = to_lower(direction);
  if (lower == "clockwise") {
    return "顺时针";
  }
  if (lower == "counterclockwise") {
    return "逆时针";
  }
  if (lower == "auto") {
    return "自动";
  }
  return direction;
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
  declare_parameter<std::string>("route_waypoints_file", route_waypoints_file_);
  get_parameter("route_waypoints_file", route_waypoints_file_);
  declare_parameter<std::string>("frame_id", frame_id_);
  get_parameter("frame_id", frame_id_);
  declare_parameter<std::string>("base_frame_id", base_frame_id_);
  get_parameter("base_frame_id", base_frame_id_);
  declare_parameter<std::string>("cmd_vel_topic", cmd_vel_topic_);
  get_parameter("cmd_vel_topic", cmd_vel_topic_);
  declare_parameter<std::string>("scan_topic", scan_topic_);
  get_parameter("scan_topic", scan_topic_);
  declare_parameter<std::string>("map_topic", map_topic_);
  get_parameter("map_topic", map_topic_);
  declare_parameter<bool>("debug_control_logs", debug_control_logs_);
  get_parameter("debug_control_logs", debug_control_logs_);
  declare_parameter<bool>("wait_for_qr_before_start", wait_for_qr_before_start_);
  get_parameter("wait_for_qr_before_start", wait_for_qr_before_start_);

  // 基础巡线控制
  declare_parameter<double>("control_frequency", control_frequency);
  get_parameter("control_frequency", control_frequency);
  declare_parameter<double>("linear_speed", linear_speed_);
  get_parameter("linear_speed", linear_speed_);
  declare_parameter<double>("lookahead_distance", lookahead_distance_);
  get_parameter("lookahead_distance", lookahead_distance_);
  declare_parameter<double>("pass_radius", pass_radius_);
  get_parameter("pass_radius", pass_radius_);
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
  declare_parameter<double>("slow_corridor_length", slow_corridor_length_);
  get_parameter("slow_corridor_length", slow_corridor_length_);
  declare_parameter<double>("slow_corridor_half_width", slow_corridor_half_width_);
  get_parameter("slow_corridor_half_width", slow_corridor_half_width_);
  declare_parameter<int>("slow_obstacle_min_points", slow_obstacle_min_points_);
  get_parameter("slow_obstacle_min_points", slow_obstacle_min_points_);
  declare_parameter<double>("slow_linear_speed", slow_linear_speed_);
  get_parameter("slow_linear_speed", slow_linear_speed_);
  declare_parameter<double>("direction_corridor_length", direction_corridor_length_);
  get_parameter("direction_corridor_length", direction_corridor_length_);
  declare_parameter<double>("direction_corridor_half_width", direction_corridor_half_width_);
  get_parameter("direction_corridor_half_width", direction_corridor_half_width_);
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
  declare_parameter<bool>("enable_static_map_obstacle_filter", enable_static_map_obstacle_filter_);
  get_parameter("enable_static_map_obstacle_filter", enable_static_map_obstacle_filter_);
  declare_parameter<int>("static_map_occupied_threshold", static_map_occupied_threshold_);
  get_parameter("static_map_occupied_threshold", static_map_occupied_threshold_);
  declare_parameter<int>("static_map_inflation_cells", static_map_inflation_cells_);
  get_parameter("static_map_inflation_cells", static_map_inflation_cells_);

  // QR 与图生文触发
  declare_parameter<std::string>("qr_topic", qr_topic_);
  get_parameter("qr_topic", qr_topic_);
  declare_parameter<std::string>("route_direction", route_direction_);
  get_parameter("route_direction", route_direction_);
  declare_parameter<bool>("enable_qr_direction_select", enable_qr_direction_select_);
  get_parameter("enable_qr_direction_select", enable_qr_direction_select_);
  declare_parameter<bool>("use_qr_parity_direction", use_qr_parity_direction_);
  get_parameter("use_qr_parity_direction", use_qr_parity_direction_);
  declare_parameter<bool>("suspend_visual_control_on_qr", suspend_visual_control_on_qr_);
  get_parameter("suspend_visual_control_on_qr", suspend_visual_control_on_qr_);
  declare_parameter<bool>("resume_visual_control_on_finish", resume_visual_control_on_finish_);
  get_parameter("resume_visual_control_on_finish", resume_visual_control_on_finish_);
  declare_parameter<std::string>("visual_control_suspend_topic", visual_control_suspend_topic_);
  get_parameter("visual_control_suspend_topic", visual_control_suspend_topic_);
  declare_parameter<int>("visual_control_suspend_value", visual_control_suspend_value_);
  get_parameter("visual_control_suspend_value", visual_control_suspend_value_);
  declare_parameter<int>("visual_control_resume_value", visual_control_resume_value_);
  get_parameter("visual_control_resume_value", visual_control_resume_value_);
  declare_parameter<int>("clockwise_qr_value", clockwise_qr_value_);
  get_parameter("clockwise_qr_value", clockwise_qr_value_);
  declare_parameter<int>("counterclockwise_qr_value", counterclockwise_qr_value_);
  get_parameter("counterclockwise_qr_value", counterclockwise_qr_value_);
  declare_parameter<bool>("enable_image_analysis_trigger", enable_image_analysis_trigger_);
  get_parameter("enable_image_analysis_trigger", enable_image_analysis_trigger_);
  declare_parameter<std::string>("image_analysis_trigger_mode", image_analysis_trigger_mode_);
  get_parameter("image_analysis_trigger_mode", image_analysis_trigger_mode_);
  declare_parameter<int>("image_analysis_waypoint", image_analysis_waypoint_);
  get_parameter("image_analysis_waypoint", image_analysis_waypoint_);
  declare_parameter<double>("image_analysis_x", image_analysis_x_);
  get_parameter("image_analysis_x", image_analysis_x_);
  declare_parameter<double>("image_analysis_y", image_analysis_y_);
  get_parameter("image_analysis_y", image_analysis_y_);
  declare_parameter<double>("image_analysis_radius", image_analysis_radius_);
  get_parameter("image_analysis_radius", image_analysis_radius_);
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
  slow_corridor_length_ = std::max(0.20, slow_corridor_length_);
  slow_corridor_half_width_ = std::max(0.05, slow_corridor_half_width_);
  slow_obstacle_min_points_ = std::max(1, slow_obstacle_min_points_);
  slow_linear_speed_ = std::max(0.0, slow_linear_speed_);
  direction_corridor_length_ = std::max(0.20, direction_corridor_length_);
  direction_corridor_half_width_ = std::max(0.05, direction_corridor_half_width_);
  obstacle_corridor_length_ = std::max(0.20, obstacle_corridor_length_);
  obstacle_corridor_half_width_ = std::max(0.05, obstacle_corridor_half_width_);
  slow_corridor_length_ = std::max(slow_corridor_length_, obstacle_corridor_length_);
  slow_corridor_half_width_ = std::max(slow_corridor_half_width_, obstacle_corridor_half_width_);
  direction_corridor_length_ = std::max(direction_corridor_length_, obstacle_corridor_length_);
  direction_corridor_half_width_ = std::max(direction_corridor_half_width_, obstacle_corridor_half_width_);
  obstacle_min_points_ = std::max(1, obstacle_min_points_);
  avoid_angular_z_ = std::max(0.0, avoid_angular_z_);
  avoid_linear_speed_ = std::max(0.0, avoid_linear_speed_);
  static_map_occupied_threshold_ = std::max(1, std::min(100, static_map_occupied_threshold_));
  static_map_inflation_cells_ = std::max(0, static_map_inflation_cells_);
  image_analysis_trigger_mode_ = to_lower(image_analysis_trigger_mode_);
  image_analysis_waypoint_ = std::max(1, image_analysis_waypoint_);
  image_analysis_radius_ = std::max(0.01, image_analysis_radius_);
  if (route_waypoints_file_.empty()) {
    route_waypoints_file_ = waypoints_file_;
  }
  if (wait_for_qr_before_start_ && !enable_qr_direction_select_) {
    RCLCPP_WARN(get_logger(), "wait_for_qr_before_start=true 需要启用 QR 选路，已自动打开 QR 订阅。");
    enable_qr_direction_select_ = true;
  }

  if (wait_for_qr_before_start_) {
    waiting_for_qr_start_ = true;
    following_qr_path_ = false;
    selected_route_direction_ = "waiting_qr";
  } else {
    // 兼容旧流程：启动先加载二维码前轨迹；扫到 QR 后再切到环线。
    waypoints_ = load_waypoints(waypoints_file_, route_direction_);
    selected_route_direction_ = "qr_path";
    following_qr_path_ = true;
  }

  // 发布速度命令，最终由主线底盘驱动订阅 /cmd_vel 执行。
  cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

  // 发布图像分析触发。transient_local 让后启动的订阅者也能收到最后一次触发值。
  image_analysis_trigger_pub_ =
    create_publisher<std_msgs::msg::Int32>(image_analysis_trigger_topic_, rclcpp::QoS(10).transient_local());
  visual_control_suspend_pub_ = create_publisher<std_msgs::msg::Int32>(visual_control_suspend_topic_, 3);

  // 保存最近一帧雷达数据，控制循环中用于避障。
  // 避障只关心最新障碍物位置，队列深度设为 1，系统忙时直接丢弃旧雷达帧。
  rclcpp::SensorDataQoS scan_qos;
  scan_qos.keep_last(1);
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_, scan_qos,
    [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) { latest_scan_ = msg; });

  // 保存静态地图，用地图占用栅格过滤墙体点，只让临时障碍物参与避障。
  if (enable_static_map_obstacle_filter_) {
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, rclcpp::QoS(1).transient_local().reliable(),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) { latest_map_ = msg; });
  }

  // 只需要一个有效 QR；选定方向后会取消订阅，避免后续误切路线。
  if (enable_qr_direction_select_) {
    qr_sub_ = create_subscription<std_msgs::msg::String>(
      qr_topic_, 10, std::bind(&SmoothPathFollower::qr_callback, this, std::placeholders::_1));
  }

  if (waypoints_.empty() && !waiting_for_qr_start_) {
    RCLCPP_ERROR(get_logger(), "未加载到航点，巡线节点将停车。");
    publish_stop();
    return;
  }

  const auto reverse_count = std::count_if(
    waypoints_.begin(), waypoints_.end(), [](const Waypoint & w) { return w.reverse; });
  const std::string route_name = waiting_for_qr_start_ ? "等待 QR" :
    (following_qr_path_ ? "二维码前路径" : route_direction_name(selected_route_direction_));
  RCLCPP_INFO(
    get_logger(), "SmoothPathFollower 初始化完成：%zu 个航点，路线=%s。",
    waypoints_.size(), route_name.c_str());
  if (debug_control_logs_) {
    RCLCPP_INFO(
      get_logger(),
      "debug: 倒车点=%zu，扫码前=%s，扫码后=%s，前瞻=%.2f，线速度=%.2f，方向矩形=%.2fx%.2f，避障=%s，地图过滤=%s。",
      static_cast<std::size_t>(reverse_count), waypoints_file_.c_str(), route_waypoints_file_.c_str(),
      lookahead_distance_, linear_speed_, direction_corridor_length_, direction_corridor_half_width_ * 2.0,
      enable_obstacle_avoidance_ ? "开启" : "关闭",
      enable_static_map_obstacle_filter_ ? "开启" : "关闭");
  }

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

bool SmoothPathFollower::should_publish_stop_on_exit() const
{
  return publish_stop_on_exit_;
}

const char * SmoothPathFollower::control_state_name(SmoothPathFollower::ControlState state) const
{
  switch (state) {
    case ControlState::WAITING_FOR_POSE:
      return "等待定位";
    case ControlState::WAITING_FOR_QR:
      return "等待二维码";
    case ControlState::PATH_FOLLOWING:
      return "巡线";
    case ControlState::OBSTACLE_AVOIDING:
      return "避障";
    case ControlState::FINISHED:
      return "完成";
  }
  return "未知";
}

void SmoothPathFollower::set_control_state(SmoothPathFollower::ControlState next_state)
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

void SmoothPathFollower::set_obstacle_report_state(
  SmoothPathFollower::ObstacleReportState next_state)
{
  if (obstacle_report_state_ == next_state) {
    return;
  }
  obstacle_report_state_ = next_state;
}

// ********************************************************************************
// ******************************** 主控制流程 ********************************
// ********************************************************************************

// 主控制循环：拿位姿 -> 更新路线线段进度 -> 算离散前瞻 waypoint 速度 -> 必要时由矩形通行区避障接管 -> 发布。
// 这里保留完整顺序，避免为了几行赋值再跳到小包装函数里。
void SmoothPathFollower::control_once()
{
  if (waiting_for_qr_start_) {
    set_control_state(ControlState::WAITING_FOR_QR);
    return;
  }
  if (waypoints_.empty()) {
    publish_stop();
    return;
  }

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
      if (debug_control_logs_) {
        RCLCPP_INFO(get_logger(), "debug: 已到达起始航点。");
      }
    }
  }

  if (first_waypoint_reached_) {
    update_route_progress(pose);
  }

  if (stop_if_route_finished()) {
    return;
  }

  trigger_image_analysis_if_needed(pose);

  const auto target = compute_lookahead_target(pose);
  auto cmd = compute_tracking_command(pose, target);
  cmd = apply_corridor_obstacle_avoidance(pose, cmd, target.reverse);

  set_control_state(next_control_state_);
  cmd_pub_->publish(cmd);
}

bool SmoothPathFollower::stop_if_route_finished()
{
  if (waypoints_.empty() || !first_waypoint_reached_ ||
    current_index_ < waypoints_.size() - 1)
  {
    return false;
  }

  if (following_qr_path_) {
    if (!qr_path_finished_) {
      RCLCPP_INFO(get_logger(), "到达二维码前等待点，等待 QR。");
    }
    qr_path_finished_ = true;
    set_control_state(ControlState::WAITING_FOR_QR);
    publish_stop();
    return true;
  }

  set_control_state(ControlState::FINISHED);
  if (resume_visual_control_on_finish_) {
    RCLCPP_INFO(get_logger(), "环线追点完成，恢复 visual_control 接管停车。");
    resume_visual_control();
    publish_stop_on_exit_ = false;
  } else {
    RCLCPP_INFO(get_logger(), "路线追点完成，停车。");
    publish_stop();
  }
  rclcpp::shutdown();
  return true;
}

// ********************************************************************************
// ******************************** 路线线段进度 ********************************
// ********************************************************************************

// 正常巡线同时使用两种进度条件：
//   1. 车投影到当前线段末端附近；
//   2. 车进入下一个 waypoint 的 pass_radius。
// 投影仍只看当前线段，避免高速时提前跳线；pass_radius 每帧最多推进一个点。
// 避障时冻结路线进度，避免把绕障轨迹误认为原路线轨迹；避障结束时再单次重定位。
// current_index_ 表示当前参考线段起点：waypoint[i] -> waypoint[i + 1]。
void SmoothPathFollower::update_route_progress(const SmoothPathFollower::RobotPose & pose)
{
  if (waypoints_.size() < 2 || current_index_ >= waypoints_.size() - 1) {
    return;
  }
  if (obstacle_avoidance_active_) {
    return;
  }

  const std::size_t next_index = current_index_ + 1;
  const auto & next_waypoint = waypoints_[next_index];
  const double next_distance = std::hypot(next_waypoint.x - pose.x, next_waypoint.y - pose.y);
  const bool waypoint_radius_hit = next_distance <= waypoint_pass_radius(next_index);

  std::size_t candidate_index = current_index_;
  const char * advance_reason = nullptr;
  if (waypoint_radius_hit) {
    candidate_index = next_index;
    advance_reason = "航点半径";
  }

  RouteProjection projection;
  if (find_route_projection(pose, 0U, projection)) {
    std::size_t projection_candidate = projection.segment_index;
    if (projection.t >= segment_advance_t_) {
      projection_candidate = projection.segment_index + 1;
    }
    projection_candidate = std::min(projection_candidate, waypoints_.size() - 1);
    if (projection_candidate > candidate_index) {
      candidate_index = projection_candidate;
      advance_reason = "路线投影";
    }
  }

  if (candidate_index <= current_index_) {
    return;
  }
  if (advance_reason == nullptr) {
    advance_reason = "路线进度";
  }

  std::size_t allowed_index = current_index_;
  if (!can_advance_to_index(candidate_index, pose, allowed_index)) {
    return;
  }
  advance_current_index_to(allowed_index, advance_reason);
}

void SmoothPathFollower::relocalize_route_after_avoidance(const SmoothPathFollower::RobotPose & pose)
{
  if (waypoints_.size() < 2 || current_index_ >= waypoints_.size() - 1) {
    return;
  }

  RouteProjection projection;
  if (!find_route_projection(pose, static_cast<std::size_t>(route_projection_window_), projection)) {
    RCLCPP_WARN(get_logger(), "避障结束后未匹配到路线，保持当前航点。");
    if (debug_control_logs_) {
      const auto & waypoint = waypoints_[current_index_];
      RCLCPP_INFO(
        get_logger(), "debug: 保持航点 %zu/%zu（%s）。",
        current_index_ + 1, waypoints_.size(), waypoint.name.c_str());
    }
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
  advance_current_index_to(allowed_index, "避障回线");
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
    if (debug_control_logs_) {
      const auto & waypoint = waypoints_[current_index_];
      RCLCPP_INFO(
        get_logger(), "debug: %s 推进到航点 %zu/%zu（%s）。",
        reason, current_index_ + 1, waypoints_.size(), waypoint.name.c_str());
    }
    trigger_image_analysis_if_needed(current_index_ + 1);
  }
}

double SmoothPathFollower::waypoint_pass_radius(std::size_t index) const
{
  if (index >= waypoints_.size()) {
    return pass_radius_;
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
// 当前策略：按航点触发图生文时，该航点不能被投影跨过。
bool SmoothPathFollower::is_precision_waypoint(std::size_t index) const
{
  if (index >= waypoints_.size()) {
    return false;
  }
  return image_analysis_trigger_mode_ == "waypoint" && enable_image_analysis_trigger_ &&
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
// 小矩形有障碍时进入避障；方向矩形负责选左右；减速矩形只限速。
geometry_msgs::msg::Twist SmoothPathFollower::apply_corridor_obstacle_avoidance(
  const SmoothPathFollower::RobotPose & pose, geometry_msgs::msg::Twist cmd, bool reverse)
{
  if (!enable_obstacle_avoidance_ || reverse) {
    obstacle_avoidance_active_ = false;
    set_obstacle_report_state(ObstacleReportState::CLEAR);
    avoid_direction_ = 0;
    return cmd;
  }

  if (static_cast<int>(current_index_) + 1 < obstacle_enable_from_waypoint_) {
    obstacle_avoidance_active_ = false;
    set_obstacle_report_state(ObstacleReportState::CLEAR);
    avoid_direction_ = 0;
    return cmd;
  }

  CorridorObstacle obstacle;
  const bool blocked = detect_corridor_obstacle(
    pose, false, obstacle_corridor_length_, obstacle_corridor_half_width_, obstacle_min_points_, obstacle);
  if (blocked) {
    const bool entering_avoidance = obstacle_report_state_ != ObstacleReportState::AVOIDING;
    CorridorObstacle direction_obstacle;
    detect_corridor_obstacle(
      pose, false, direction_corridor_length_, direction_corridor_half_width_, 1, direction_obstacle);
    if (direction_obstacle.point_count <= 0 && direction_obstacle.static_ignored_count <= 0) {
      direction_obstacle = obstacle;
    }
    obstacle_avoidance_active_ = true;
    avoid_direction_ = choose_avoid_direction(direction_obstacle);
    set_obstacle_report_state(ObstacleReportState::AVOIDING);
    if (entering_avoidance) {
      RCLCPP_WARN(
        get_logger(), "障碍物过近，切换到避障：%s。",
        avoid_direction_ > 0 ? "向左" : "向右");
      if (debug_control_logs_) {
        RCLCPP_INFO(
          get_logger(),
          "debug: 障碍点=%d，墙体点=%d，中心=(%.2f, %.2f)，最近=%.2f，动态左/右=%d/%d，墙体左/右=%d/%d。",
          obstacle.point_count, obstacle.static_ignored_count, obstacle.center.x, obstacle.center.y,
          obstacle.nearest_range, direction_obstacle.left_count, direction_obstacle.right_count,
          direction_obstacle.static_left_count, direction_obstacle.static_right_count);
      }
    }
  } else if (obstacle_avoidance_active_) {
    relocalize_route_after_avoidance(pose);
    obstacle_avoidance_active_ = false;
    avoid_direction_ = 0;
    RCLCPP_WARN(get_logger(), "结束避障，回到巡线。");
  }

  if (!obstacle_avoidance_active_) {
    CorridorObstacle slow_obstacle;
    const bool should_slow = detect_corridor_obstacle(
      pose, false, slow_corridor_length_, slow_corridor_half_width_, slow_obstacle_min_points_, slow_obstacle);
    if (should_slow) {
      const bool entering_slowing = obstacle_report_state_ != ObstacleReportState::SLOWING;
      set_obstacle_report_state(ObstacleReportState::SLOWING);
      if (entering_slowing && debug_control_logs_) {
        RCLCPP_INFO(
          get_logger(),
          "debug: 减速区障碍点=%d，墙体点=%d，中心=(%.2f, %.2f)，最近=%.2f，限速=%.2f。",
          slow_obstacle.point_count, slow_obstacle.static_ignored_count, slow_obstacle.center.x,
          slow_obstacle.center.y, slow_obstacle.nearest_range, slow_linear_speed_);
      }
      if (cmd.linear.x > slow_linear_speed_) {
        cmd.linear.x = slow_linear_speed_;
      }
    } else {
      set_obstacle_report_state(ObstacleReportState::CLEAR);
    }
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
  const SmoothPathFollower::RobotPose & pose,
  bool reverse, double corridor_length, double corridor_half_width, int min_points,
  SmoothPathFollower::CorridorObstacle & obstacle) const
{
  if (!latest_scan_) {
    return false;
  }

  const auto & scan = *latest_scan_;
  RobotPose scan_pose = pose;
  if (enable_static_map_obstacle_filter_ && latest_map_) {
    const rclcpp::Time scan_time(scan.header.stamp);
    if (scan_time.nanoseconds() > 0) {
      lookup_robot_pose_at(scan_time, scan_pose);
    }
  }
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
      const Point2D base_point{base_x, base_y};
      const Point2D motion_point{reverse ? -base_x : base_x, reverse ? -base_y : base_y};
      if (
        motion_point.x > 0.0 && motion_point.x <= corridor_length &&
        std::abs(motion_point.y) <= corridor_half_width)
      {
        if (is_static_map_obstacle(scan_pose, base_point)) {
          ++obstacle.static_ignored_count;
          if (motion_point.y >= 0.0) {
            ++obstacle.static_left_count;
          } else {
            ++obstacle.static_right_count;
          }
        } else {
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
    }
    angle += scan.angle_increment;
  }

  if (obstacle.point_count < min_points) {
    return false;
  }

  obstacle.center.x = sum_x / static_cast<double>(obstacle.point_count);
  obstacle.center.y = sum_y / static_cast<double>(obstacle.point_count);
  return true;
}

bool SmoothPathFollower::is_static_map_obstacle(
  const SmoothPathFollower::RobotPose & pose, const SmoothPathFollower::Point2D & base_point) const
{
  if (!enable_static_map_obstacle_filter_ || !latest_map_) {
    return false;
  }

  const auto & map = *latest_map_;
  const int width = static_cast<int>(map.info.width);
  const int height = static_cast<int>(map.info.height);
  if (width <= 0 || height <= 0 || map.info.resolution <= 0.0) {
    return false;
  }
  if (map.data.size() < static_cast<std::size_t>(width) * static_cast<std::size_t>(height)) {
    return false;
  }

  // base_link 下的雷达点先转到 map，再转到 OccupancyGrid 栅格坐标。
  const double cos_yaw = std::cos(pose.yaw);
  const double sin_yaw = std::sin(pose.yaw);
  const double map_x = pose.x + cos_yaw * base_point.x - sin_yaw * base_point.y;
  const double map_y = pose.y + sin_yaw * base_point.x + cos_yaw * base_point.y;

  const auto & origin = map.info.origin;
  const auto & q = origin.orientation;
  const double origin_yaw = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  const double dx = map_x - origin.position.x;
  const double dy = map_y - origin.position.y;
  const double cos_origin = std::cos(origin_yaw);
  const double sin_origin = std::sin(origin_yaw);
  const double grid_x_m = cos_origin * dx + sin_origin * dy;
  const double grid_y_m = -sin_origin * dx + cos_origin * dy;
  const int grid_x = static_cast<int>(std::floor(grid_x_m / map.info.resolution));
  const int grid_y = static_cast<int>(std::floor(grid_y_m / map.info.resolution));
  if (grid_x < 0 || grid_y < 0 || grid_x >= width || grid_y >= height) {
    return false;
  }

  const int inflation_cells = static_map_inflation_cells_;
  for (int offset_y = -inflation_cells; offset_y <= inflation_cells; ++offset_y) {
    for (int offset_x = -inflation_cells; offset_x <= inflation_cells; ++offset_x) {
      const int check_x = grid_x + offset_x;
      const int check_y = grid_y + offset_y;
      if (check_x < 0 || check_y < 0 || check_x >= width || check_y >= height) {
        continue;
      }
      const auto value = map.data[static_cast<std::size_t>(check_y) * map.info.width + check_x];
      if (value >= static_map_occupied_threshold_) {
        return true;
      }
    }
  }

  return false;
}

// 正角速度向左，负角速度向右。
// 避障是否触发只看非静态障碍；真正选方向时，把墙体也作为左右占用依据，往更空的一边走。
int SmoothPathFollower::choose_avoid_direction(const SmoothPathFollower::CorridorObstacle & obstacle) const
{
  const int left_score = obstacle.left_count * 2 + obstacle.static_left_count;
  const int right_score = obstacle.right_count * 2 + obstacle.static_right_count;
  if (left_score != right_score) {
    return left_score < right_score ? 1 : -1;
  }

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
//   1. 根据 QR 数字判断顺/逆时针；
//   2. 从二维码前轨迹切换到 route_waypoints_file_ 中对应方向的环线。
void SmoothPathFollower::qr_callback(const std_msgs::msg::String::ConstSharedPtr msg)
{
  if (qr_route_selected_) {
    return;
  }

  const std::string direction = direction_from_qr(msg->data);
  if (!direction.empty()) {
    if (switch_route(direction)) {
      qr_route_selected_ = true;
      suspend_visual_control();
      if (qr_sub_) {
        qr_sub_.reset();
      }
      if (debug_control_logs_) {
        RCLCPP_INFO(get_logger(), "debug: 已停止订阅 QR 话题：%s。", qr_topic_.c_str());
      }
    }
    return;
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

// 切换路线：
//   扫码前在 waypoints_file_ 中跑二维码前轨迹；
//   扫到 QR 后加载 route_waypoints_file_，并从该方向环线的第一个航点开始追。
bool SmoothPathFollower::switch_route(const std::string & direction)
{
  if (!waiting_for_qr_start_ && !following_qr_path_ && direction == selected_route_direction_) {
    return true;
  }

  std::string resolved_direction;
  const auto new_waypoints = load_waypoints(route_waypoints_file_, direction, &resolved_direction);
  if (new_waypoints.empty()) {
    const auto route_name = route_direction_name(direction);
    RCLCPP_WARN(get_logger(), "QR 选择了路线“%s”，但未加载到对应航点。", route_name.c_str());
    return false;
  }

  const bool was_following_qr_path = following_qr_path_;
  const bool was_waiting_for_qr = waiting_for_qr_start_;
  waypoints_ = new_waypoints;
  selected_route_direction_ = resolved_direction.empty() ? direction : resolved_direction;
  waiting_for_qr_start_ = false;
  following_qr_path_ = false;
  qr_path_finished_ = false;
  image_analysis_triggered_ = false;
  if (was_following_qr_path || was_waiting_for_qr) {
    current_index_ = 0;
    first_waypoint_reached_ = false;
  } else {
    current_index_ = std::min(current_index_, waypoints_.size() - 1);
    first_waypoint_reached_ = current_index_ > 0;
  }
  obstacle_avoidance_active_ = false;
  avoid_direction_ = 0;
  const auto route_name = route_direction_name(selected_route_direction_);
  RCLCPP_INFO(get_logger(), "QR 选择%s路线，开始环线。", route_name.c_str());
  if (debug_control_logs_) {
    RCLCPP_INFO(
      get_logger(), "debug: 环线文件=%s，起点=%zu/%zu（%s）。",
      route_waypoints_file_.c_str(), current_index_ + 1, waypoints_.size(),
      waypoints_[current_index_].name.c_str());
  }
  return true;
}

void SmoothPathFollower::suspend_visual_control()
{
  if (!suspend_visual_control_on_qr_ || !visual_control_suspend_pub_) {
    return;
  }

  std_msgs::msg::Int32 msg;
  msg.data = visual_control_suspend_value_;
  for (int i = 0; i < 3; ++i) {
    visual_control_suspend_pub_->publish(msg);
  }
  RCLCPP_INFO(get_logger(), "已通知 visual_control 挂起，smooth_path_follower 接管。");
  if (debug_control_logs_) {
    RCLCPP_INFO(
      get_logger(), "debug: suspend_topic=%s, value=%d。",
      visual_control_suspend_topic_.c_str(), visual_control_suspend_value_);
  }
}

void SmoothPathFollower::resume_visual_control()
{
  if (!visual_control_suspend_pub_) {
    return;
  }

  std_msgs::msg::Int32 msg;
  msg.data = visual_control_resume_value_;
  for (int i = 0; i < 3; ++i) {
    visual_control_suspend_pub_->publish(msg);
  }
  RCLCPP_INFO(get_logger(), "已通知 visual_control 恢复，后续交给视觉停车。");
  if (debug_control_logs_) {
    RCLCPP_INFO(
      get_logger(), "debug: resume_topic=%s, value=%d。",
      visual_control_suspend_topic_.c_str(), visual_control_resume_value_);
  }
}

// 到达指定航点时发布一次图像分析触发消息。
// waypoint_number 是人看的航点编号，从 1 开始；current_index_ 是数组下标，从 0 开始。
void SmoothPathFollower::trigger_image_analysis_if_needed(std::size_t waypoint_number)
{
  // 只触发一次，避免机器人在同一个航点附近反复发布。
  if (following_qr_path_ || !enable_image_analysis_trigger_ || image_analysis_triggered_) {
    return;
  }
  if (image_analysis_trigger_mode_ != "waypoint") {
    return;
  }
  if (waypoint_number != static_cast<std::size_t>(image_analysis_waypoint_)) {
    return;
  }

  std_msgs::msg::Int32 msg;
  msg.data = image_analysis_waypoint_;
  image_analysis_trigger_pub_->publish(msg);
  image_analysis_triggered_ = true;
  RCLCPP_INFO(get_logger(), "图生文触发：航点 %d。", image_analysis_waypoint_);
  if (debug_control_logs_) {
    RCLCPP_INFO(get_logger(), "debug: 触发话题=%s。", image_analysis_trigger_topic_.c_str());
  }
}

void SmoothPathFollower::trigger_image_analysis_if_needed(const SmoothPathFollower::RobotPose & pose)
{
  if (following_qr_path_ || !enable_image_analysis_trigger_ || image_analysis_triggered_) {
    return;
  }
  if (image_analysis_trigger_mode_ != "region") {
    return;
  }

  const double distance = std::hypot(image_analysis_x_ - pose.x, image_analysis_y_ - pose.y);
  if (distance > image_analysis_radius_) {
    return;
  }

  std_msgs::msg::Int32 msg;
  msg.data = image_analysis_waypoint_;
  image_analysis_trigger_pub_->publish(msg);
  image_analysis_triggered_ = true;
  RCLCPP_INFO(get_logger(), "图生文触发：进入区域。");
  if (debug_control_logs_) {
    RCLCPP_INFO(
      get_logger(),
      "debug: 区域中心=(%.2f, %.2f)，半径=%.2f，距离=%.2f，触发话题=%s。",
      image_analysis_x_, image_analysis_y_, image_analysis_radius_, distance,
      image_analysis_trigger_topic_.c_str());
  }
}

// ********************************************************************************
// ******************************** 航点加载与定位 ********************************
// ********************************************************************************

// 从 YAML 文件加载航点。
// 支持三种格式：
//   1. 推荐格式：只维护 routes.counterclockwise；请求顺时针时自动反转；
//   2. 兼容格式：routes.clockwise / routes.counterclockwise 各自维护；
//   3. waypoints：没有路线分组时的兜底格式。
std::vector<Waypoint> SmoothPathFollower::load_waypoints(
  const std::string & waypoints_file, const std::string & requested_direction,
  std::string * resolved_direction_out)
{
  std::vector<Waypoint> waypoints;
  std::string resolved_direction = requested_direction;
  if (waypoints_file.empty()) {
    RCLCPP_ERROR(get_logger(), "航点文件路径为空。");
    return waypoints;
  }
  try {
    const YAML::Node data = YAML::LoadFile(waypoints_file);
    std::string direction = to_lower(requested_direction);
    if (direction.empty() || direction == "auto") {
      // auto 时使用 YAML 顶层 route_direction；如果没有，就默认 counterclockwise。
      direction = data["route_direction"] ? to_lower(data["route_direction"].as<std::string>()) : "counterclockwise";
    }
    resolved_direction = direction;
    const YAML::Node routes = data["routes"];
    YAML::Node items;
    bool reverse_items = false;
    if (routes && routes[direction]) {
      items = routes[direction];
      resolved_direction = direction;
    } else if (routes && direction == "clockwise" && routes["counterclockwise"]) {
      items = routes["counterclockwise"];
      reverse_items = true;
      resolved_direction = direction;
    } else if (routes && direction == "counterclockwise" && routes["clockwise"]) {
      items = routes["clockwise"];
      reverse_items = true;
      resolved_direction = direction;
    } else {
      items = data["waypoints"];
      if (items) {
        resolved_direction = "waypoints";
      }
    }
    if (!items || !items.IsSequence()) {
      RCLCPP_WARN(get_logger(), "航点文件 %s 中未找到可用航点序列。", waypoints_file.c_str());
      return waypoints;
    }
    for (std::size_t i = 0; i < items.size(); ++i) {
      try {
        Waypoint w;
        const YAML::Node item = items[i];
        // 当前跟随器不使用 YAML yaw；方向由当前位姿和目标点实时计算。
        if (item.IsMap()) {
          w.name = item["name"] ? item["name"].as<std::string>() : "航点_" + std::to_string(i + 1);
          w.x = item["x"].as<double>();
          w.y = item["y"].as<double>();
          w.reverse = yaml_bool(item["reverse"], false);
          w.pass_radius = item["pass_radius"] ? item["pass_radius"].as<double>() : pass_radius_;
          w.reverse_pass_radius =
            item["reverse_pass_radius"] ? item["reverse_pass_radius"].as<double>() : reverse_pass_radius_;
        } else if (item.IsSequence() && item.size() >= 2) {
          w.name = "航点_" + std::to_string(i + 1);
          w.x = item[0].as<double>();
          w.y = item[1].as<double>();
          w.reverse = false;
          w.pass_radius = pass_radius_;
          w.reverse_pass_radius = reverse_pass_radius_;
        } else {
          RCLCPP_WARN(get_logger(), "跳过无效航点 #%zu：格式不是 map，也不是 [x, y, yaw]。", i);
          continue;
        }
        waypoints.push_back(w);
      } catch (const YAML::Exception & exc) {
        RCLCPP_WARN(get_logger(), "跳过无效航点 #%zu：%s", i, exc.what());
      }
    }
    if (reverse_items) {
      std::reverse(waypoints.begin(), waypoints.end());
    }
  } catch (const YAML::Exception & exc) {
    RCLCPP_ERROR(get_logger(), "读取航点 YAML %s 失败：%s", waypoints_file.c_str(), exc.what());
  }
  if (resolved_direction_out) {
    *resolved_direction_out = resolved_direction;
  }
  return waypoints;
}

// 从 TF 查询指定时间的 frame_id_ -> base_frame_id_，默认 map -> base_link。
// 墙体过滤会用 scan.header.stamp 调这里，避免用“现在的车位姿”去解释“过去的雷达点”。
bool SmoothPathFollower::lookup_robot_pose_at(const rclcpp::Time & stamp, SmoothPathFollower::RobotPose & pose) const
{
  try {
    const auto transform = tf_buffer_.lookupTransform(frame_id_, base_frame_id_, stamp);
    const auto & translation = transform.transform.translation;
    const auto & rotation = transform.transform.rotation;
    pose.x = translation.x;
    pose.y = translation.y;
    pose.yaw = std::atan2(
      2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
      1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z));
    return true;
  } catch (const tf2::TransformException & exc) {
    RCLCPP_WARN_ONCE(
      get_logger(), "等待 TF %s->%s：%s", frame_id_.c_str(), base_frame_id_.c_str(), exc.what());
    return false;
  }
}

// 从 TF 查询 frame_id_ -> base_frame_id_ 的最新位姿。
// 返回 x/y/yaw，其中 yaw 是机器人在 map 坐标系下的航向角。
bool SmoothPathFollower::lookup_robot_pose(double & x, double & y, double & yaw)
{
  RobotPose pose;
  if (!lookup_robot_pose_at(rclcpp::Time(0, 0, get_clock()->get_clock_type()), pose)) {
    return false;
  }
  x = pose.x;
  y = pose.y;
  yaw = pose.yaw;
  return true;
}

int main(int argc, char ** argv)
{
  // 标准 ROS 2 节点入口：初始化、创建节点、进入事件循环，退出前发布零速度。
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SmoothPathFollower>();
  rclcpp::spin(node);
  if (node->should_publish_stop_on_exit()) {
    node->publish_stop();
  }
  rclcpp::shutdown();
  return 0;
}
