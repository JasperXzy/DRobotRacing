#include "origincar_system/visual_lidar_path_follower.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <exception>
#include <functional>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "tf2/exceptions.h"
#include "tf2/time.h"

/*
 * visual_lidar_path_follower.cpp
 *
 * 作用：
 *   这是 test/dev_ws 里的路径跟随节点。它不调用 Nav2 action，
 *   而是直接根据 map 坐标系下的航点和当前机器人位姿，
 *   计算 geometry_msgs/Twist 并发布到 /cmd_vel。
 *
 * 输入：
 *   1. YAML 航点文件：维护逆时针路线；顺时针由程序反向读取，每个点至少有 x、y。
 *   2. TF：frame_id -> base_frame_id，默认 map -> base_link。
 *   3. /qr_info 或 /qr_code：可选 QR 数字，用于选择顺/逆时针路线。
 *
 * 输出：
 *   1. /cmd_vel：底盘速度命令。
 *   2. /person_trigger 或 /sign4return：到独立拍照点时触发图像分析/图生文。
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
VisualLidarPathFollower::VisualLidarPathFollower()
: rclcpp::Node("visual_lidar_path_follower"),
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
  declare_parameter<std::string>("restriction_map_topic", restriction_map_topic_);
  get_parameter("restriction_map_topic", restriction_map_topic_);
  declare_parameter<bool>("debug_control_logs", debug_control_logs_);
  get_parameter("debug_control_logs", debug_control_logs_);
  declare_parameter<bool>("wait_for_qr_before_start", wait_for_qr_before_start_);
  get_parameter("wait_for_qr_before_start", wait_for_qr_before_start_);
  declare_parameter<bool>("enable_debug_visualization", enable_debug_visualization_);
  get_parameter("enable_debug_visualization", enable_debug_visualization_);

  // 基础巡线控制
  declare_parameter<double>("control_frequency", control_frequency);
  get_parameter("control_frequency", control_frequency);
  declare_parameter<double>("linear_speed", linear_speed_);
  get_parameter("linear_speed", linear_speed_);
  declare_parameter<double>("lookahead_distance", lookahead_distance_);
  get_parameter("lookahead_distance", lookahead_distance_);
  declare_parameter<double>("pass_radius", pass_radius_);
  get_parameter("pass_radius", pass_radius_);
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

  // waypoint_shift 点云簇与地图横截方向
  declare_parameter<double>("cluster_max_point_gap", cluster_max_point_gap_);
  get_parameter("cluster_max_point_gap", cluster_max_point_gap_);
  declare_parameter<double>("cluster_max_range_jump", cluster_max_range_jump_);
  get_parameter("cluster_max_range_jump", cluster_max_range_jump_);
  declare_parameter<int>("cluster_min_points", cluster_min_points_);
  get_parameter("cluster_min_points", cluster_min_points_);
  declare_parameter<double>("cone_circle_min_radius", cone_circle_min_radius_);
  get_parameter("cone_circle_min_radius", cone_circle_min_radius_);
  declare_parameter<double>("cone_circle_max_radius", cone_circle_max_radius_);
  get_parameter("cone_circle_max_radius", cone_circle_max_radius_);
  declare_parameter<double>("cone_circle_max_rmse", cone_circle_max_rmse_);
  get_parameter("cone_circle_max_rmse", cone_circle_max_rmse_);
  declare_parameter<double>(
    "waypoint_shift_clearance_radius", waypoint_shift_clearance_radius_);
  get_parameter("waypoint_shift_clearance_radius", waypoint_shift_clearance_radius_);
  declare_parameter<int>(
    "waypoint_shift_occupied_threshold", waypoint_shift_occupied_threshold_);
  get_parameter("waypoint_shift_occupied_threshold", waypoint_shift_occupied_threshold_);
  declare_parameter<int>("waypoint_shift_check_count", waypoint_shift_check_count_);
  get_parameter("waypoint_shift_check_count", waypoint_shift_check_count_);
  declare_parameter<double>("waypoint_cloud_check_radius", waypoint_cloud_check_radius_);
  get_parameter("waypoint_cloud_check_radius", waypoint_cloud_check_radius_);
  declare_parameter<double>(
    "waypoint_shift_target_clearance", waypoint_shift_target_clearance_);
  get_parameter("waypoint_shift_target_clearance", waypoint_shift_target_clearance_);
  declare_parameter<int>("waypoint_shift_points_before", waypoint_shift_points_before_);
  get_parameter("waypoint_shift_points_before", waypoint_shift_points_before_);
  declare_parameter<int>("waypoint_shift_points_after", waypoint_shift_points_after_);
  get_parameter("waypoint_shift_points_after", waypoint_shift_points_after_);
  declare_parameter<bool>("enable_static_map_filter", enable_static_map_filter_);
  get_parameter("enable_static_map_filter", enable_static_map_filter_);
  declare_parameter<bool>("require_map_for_lidar_assist", require_map_for_lidar_assist_);
  get_parameter("require_map_for_lidar_assist", require_map_for_lidar_assist_);
  declare_parameter<int>("static_map_occupied_threshold", static_map_occupied_threshold_);
  get_parameter("static_map_occupied_threshold", static_map_occupied_threshold_);
  declare_parameter<int>("static_map_inflation_cells", static_map_inflation_cells_);
  get_parameter("static_map_inflation_cells", static_map_inflation_cells_);

  // 扫码后开始追点前的点云后撤
  declare_parameter<double>("qr_escape_check_distance", qr_escape_check_distance_);
  get_parameter("qr_escape_check_distance", qr_escape_check_distance_);
  declare_parameter<double>("qr_escape_rear_extension", qr_escape_rear_extension_);
  get_parameter("qr_escape_rear_extension", qr_escape_rear_extension_);
  declare_parameter<double>("qr_escape_half_width", qr_escape_half_width_);
  get_parameter("qr_escape_half_width", qr_escape_half_width_);
  declare_parameter<int>("qr_escape_min_points", qr_escape_min_points_);
  get_parameter("qr_escape_min_points", qr_escape_min_points_);
  declare_parameter<double>("qr_reverse_linear_speed", qr_reverse_linear_speed_);
  get_parameter("qr_reverse_linear_speed", qr_reverse_linear_speed_);
  declare_parameter<double>("qr_reverse_angular_speed", qr_reverse_angular_speed_);
  get_parameter("qr_reverse_angular_speed", qr_reverse_angular_speed_);
  declare_parameter<double>(
    "qr_rear_warning_start_distance", qr_rear_warning_start_distance_);
  get_parameter("qr_rear_warning_start_distance", qr_rear_warning_start_distance_);
  declare_parameter<double>("qr_rear_check_distance", qr_rear_check_distance_);
  get_parameter("qr_rear_check_distance", qr_rear_check_distance_);
  declare_parameter<double>("qr_rear_half_width", qr_rear_half_width_);
  get_parameter("qr_rear_half_width", qr_rear_half_width_);
  declare_parameter<int>("qr_rear_min_points", qr_rear_min_points_);
  get_parameter("qr_rear_min_points", qr_rear_min_points_);
  declare_parameter<double>(
    "qr_protect_exit_waypoint_distance", qr_protect_exit_waypoint_distance_);
  get_parameter("qr_protect_exit_waypoint_distance", qr_protect_exit_waypoint_distance_);

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
  declare_parameter<double>(
    "image_capture_clockwise_align_x", image_capture_clockwise_align_x_);
  get_parameter("image_capture_clockwise_align_x", image_capture_clockwise_align_x_);
  declare_parameter<double>(
    "image_capture_clockwise_align_y", image_capture_clockwise_align_y_);
  get_parameter("image_capture_clockwise_align_y", image_capture_clockwise_align_y_);
  declare_parameter<double>("image_capture_clockwise_x", image_capture_clockwise_x_);
  get_parameter("image_capture_clockwise_x", image_capture_clockwise_x_);
  declare_parameter<double>("image_capture_clockwise_y", image_capture_clockwise_y_);
  get_parameter("image_capture_clockwise_y", image_capture_clockwise_y_);
  declare_parameter<double>(
    "image_capture_counterclockwise_align_x", image_capture_counterclockwise_align_x_);
  get_parameter(
    "image_capture_counterclockwise_align_x", image_capture_counterclockwise_align_x_);
  declare_parameter<double>(
    "image_capture_counterclockwise_align_y", image_capture_counterclockwise_align_y_);
  get_parameter(
    "image_capture_counterclockwise_align_y", image_capture_counterclockwise_align_y_);
  declare_parameter<double>(
    "image_capture_counterclockwise_x", image_capture_counterclockwise_x_);
  get_parameter("image_capture_counterclockwise_x", image_capture_counterclockwise_x_);
  declare_parameter<double>(
    "image_capture_counterclockwise_y", image_capture_counterclockwise_y_);
  get_parameter("image_capture_counterclockwise_y", image_capture_counterclockwise_y_);
  declare_parameter<double>("image_capture_radius", image_capture_radius_);
  get_parameter("image_capture_radius", image_capture_radius_);
  declare_parameter<double>("image_capture_slow_distance", image_capture_slow_distance_);
  get_parameter("image_capture_slow_distance", image_capture_slow_distance_);
  declare_parameter<double>("image_capture_slow_speed", image_capture_slow_speed_);
  get_parameter("image_capture_slow_speed", image_capture_slow_speed_);
  declare_parameter<double>("image_capture_stop_time", image_capture_stop_time_);
  get_parameter("image_capture_stop_time", image_capture_stop_time_);
  declare_parameter<int>("image_capture_trigger_value", image_capture_trigger_value_);
  get_parameter("image_capture_trigger_value", image_capture_trigger_value_);
  declare_parameter<std::string>("image_analysis_trigger_topic", image_analysis_trigger_topic_);
  get_parameter("image_analysis_trigger_topic", image_analysis_trigger_topic_);

  route_direction_ = to_lower(route_direction_);
  lookahead_distance_ = std::max(0.05, lookahead_distance_);
  route_projection_max_distance_ = std::max(0.10, route_projection_max_distance_);
  segment_advance_t_ = std::max(0.50, std::min(1.20, segment_advance_t_));
  cluster_max_point_gap_ = std::max(0.01, cluster_max_point_gap_);
  cluster_max_range_jump_ = std::max(0.01, cluster_max_range_jump_);
  cluster_min_points_ = std::max(3, cluster_min_points_);
  cone_circle_min_radius_ = std::max(0.001, cone_circle_min_radius_);
  cone_circle_max_radius_ = std::max(cone_circle_min_radius_, cone_circle_max_radius_);
  cone_circle_max_rmse_ = std::max(0.001, cone_circle_max_rmse_);
  waypoint_shift_clearance_radius_ = std::max(0.05, waypoint_shift_clearance_radius_);
  waypoint_shift_occupied_threshold_ = std::max(
    1, std::min(100, waypoint_shift_occupied_threshold_));
  waypoint_shift_check_count_ = std::max(1, waypoint_shift_check_count_);
  waypoint_cloud_check_radius_ = std::max(0.01, waypoint_cloud_check_radius_);
  waypoint_shift_target_clearance_ = std::max(0.0, waypoint_shift_target_clearance_);
  waypoint_shift_points_before_ = std::max(0, waypoint_shift_points_before_);
  waypoint_shift_points_after_ = std::max(0, waypoint_shift_points_after_);
  static_map_occupied_threshold_ = std::max(1, std::min(100, static_map_occupied_threshold_));
  static_map_inflation_cells_ = std::max(0, static_map_inflation_cells_);
  qr_escape_check_distance_ = std::max(0.05, qr_escape_check_distance_);
  qr_escape_rear_extension_ = std::max(0.0, qr_escape_rear_extension_);
  qr_escape_half_width_ = std::max(0.05, qr_escape_half_width_);
  qr_escape_min_points_ = std::max(1, qr_escape_min_points_);
  qr_reverse_linear_speed_ = std::max(0.0, qr_reverse_linear_speed_);
  qr_reverse_angular_speed_ = std::abs(qr_reverse_angular_speed_);
  qr_rear_warning_start_distance_ = std::max(
    qr_escape_rear_extension_, qr_rear_warning_start_distance_);
  qr_rear_check_distance_ = std::max(
    qr_rear_warning_start_distance_ + 0.05, qr_rear_check_distance_);
  qr_rear_half_width_ = std::max(0.05, qr_rear_half_width_);
  qr_rear_min_points_ = std::max(1, qr_rear_min_points_);
  qr_protect_exit_waypoint_distance_ = std::max(0.01, qr_protect_exit_waypoint_distance_);
  image_capture_radius_ = std::max(0.01, image_capture_radius_);
  image_capture_slow_distance_ = std::max(image_capture_radius_, image_capture_slow_distance_);
  image_capture_slow_speed_ = std::max(0.0, image_capture_slow_speed_);
  image_capture_stop_time_ = std::max(0.0, image_capture_stop_time_);
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
    reset_waypoint_shift_state();
    selected_route_direction_ = "qr_path";
    following_qr_path_ = true;
  }

  // 发布速度命令，最终由主线底盘驱动订阅 /cmd_vel 执行。
  cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

  // 发布图像分析触发。transient_local 让后启动的订阅者也能收到最后一次触发值。
  image_analysis_trigger_pub_ =
    create_publisher<std_msgs::msg::Int32>(image_analysis_trigger_topic_, rclcpp::QoS(10).transient_local());
  visual_control_suspend_pub_ = create_publisher<std_msgs::msg::Int32>(visual_control_suspend_topic_, 3);
  if (enable_debug_visualization_) {
    const auto transient_qos = rclcpp::QoS(1).transient_local().reliable();
    debug_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/visual_lidar_path_follower/debug_markers", transient_qos);
    debug_route_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/visual_lidar_path_follower/debug_route_markers", transient_qos);
    debug_path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/visual_lidar_path_follower/debug_path", transient_qos);
  }

  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_, rclcpp::SensorDataQoS().keep_last(1),
    [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
      latest_scan_ = msg;
      ++scan_sequence_;
    });
  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic_, rclcpp::QoS(1).transient_local().reliable(),
    [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {latest_map_ = msg;});
  restriction_map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    restriction_map_topic_, rclcpp::QoS(1).transient_local().reliable(),
    [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
      latest_restriction_map_ = msg;
    });

  // 只需要一个有效 QR；选定方向后会取消订阅，避免后续误切路线。
  if (enable_qr_direction_select_) {
    qr_sub_ = create_subscription<std_msgs::msg::String>(
      qr_topic_, 10, std::bind(&VisualLidarPathFollower::qr_callback, this, std::placeholders::_1));
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
    get_logger(), "VisualLidarPathFollower 初始化完成：%zu 个航点，路线=%s。",
    waypoints_.size(), route_name.c_str());
  if (debug_control_logs_) {
    RCLCPP_INFO(
      get_logger(),
      "debug: 倒车点=%zu，扫码前=%s，扫码后=%s，控制前瞻=%.2f，"
      "避障=waypoint_shift，线速度=%.2f。",
      static_cast<std::size_t>(reverse_count), waypoints_file_.c_str(), route_waypoints_file_.c_str(),
      lookahead_distance_, linear_speed_);
  }
  publish_debug_route();

  // 定时控制循环：每次读取 TF、更新路线进度并计算离散前瞻 waypoint 速度。
  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / std::max(control_frequency, 1.0)),
    std::bind(&VisualLidarPathFollower::control_once, this));
}

// 立即发布零速度。用于启动失败、定位暂不可用、到达终点或程序退出时停车。
void VisualLidarPathFollower::publish_stop()
{
  if (cmd_pub_) {
    cmd_pub_->publish(geometry_msgs::msg::Twist());
  }
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

// ********************************************************************************
// ******************************** 主控制流程 ********************************
// ********************************************************************************

// 主控制循环：拿位姿 -> 更新路线线段进度 -> 算离散前瞻 waypoint 速度 -> 发布。
// 这里保留完整顺序，避免为了几行赋值再跳到小包装函数里。
void VisualLidarPathFollower::control_once()
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

  geometry_msgs::msg::Twist qr_escape_cmd;
  if (apply_qr_escape(pose, qr_escape_cmd)) {
    publish_debug_visualization(pose);
    set_control_state(next_control_state_);
    cmd_pub_->publish(qr_escape_cmd);
    return;
  }

  // 独立拍照点是不可跳过的精确点。进入到达半径后先停车稳定，再触发一次图生文。
  if (handle_image_capture_stop(pose)) {
    publish_debug_visualization(pose);
    set_control_state(next_control_state_);
    publish_stop();
    return;
  }

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

  auto target = compute_lookahead_target(pose, lookahead_distance_);
  if (!target.reverse) {
    update_waypoint_shifts(pose, target.waypoint_index);
    target = compute_lookahead_target(pose, lookahead_distance_);
  }

  auto cmd = compute_tracking_command(pose, target);
  apply_image_capture_speed_limit(pose, cmd);

  publish_debug_visualization(pose);

  set_control_state(next_control_state_);
  cmd_pub_->publish(cmd);
}

// 发布静态航点。debug_path 发空路径，Foxglove 里只保留 waypoint 点。
void VisualLidarPathFollower::publish_debug_route()
{
  if (!enable_debug_visualization_ || !debug_route_marker_pub_ || !debug_path_pub_ ||
    waypoints_.empty())
  {
    return;
  }

  const auto stamp = now();
  nav_msgs::msg::Path path;
  path.header.frame_id = frame_id_;
  path.header.stamp = stamp;

  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker delete_all;
  delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(delete_all);

  visualization_msgs::msg::Marker waypoint_points;
  waypoint_points.header = path.header;
  waypoint_points.ns = "waypoints";
  waypoint_points.id = 1;
  waypoint_points.type = visualization_msgs::msg::Marker::POINTS;
  waypoint_points.action = visualization_msgs::msg::Marker::ADD;
  waypoint_points.pose.orientation.w = 1.0;
  waypoint_points.scale.x = 0.028;
  waypoint_points.scale.y = 0.028;
  waypoint_points.color.r = 0.26F;
  waypoint_points.color.g = 0.65F;
  waypoint_points.color.b = 0.96F;
  waypoint_points.color.a = 0.80F;

  for (std::size_t index = 0; index < waypoints_.size(); ++index) {
    const auto & waypoint = waypoints_[index];
    geometry_msgs::msg::Point point;
    point.x = waypoint.x;
    point.y = waypoint.y;
    point.z = 0.025;
    waypoint_points.points.push_back(point);
  }

  markers.markers.push_back(waypoint_points);
  debug_path_pub_->publish(path);
  debug_route_marker_pub_->publish(markers);
}

// 发布动态调试图层：只显示圆锥圆心和圆锥拟合圆。
void VisualLidarPathFollower::publish_debug_visualization(
  const VisualLidarPathFollower::RobotPose & pose)
{
  if (!enable_debug_visualization_ || !debug_marker_pub_) {
    return;
  }

  const auto stamp = now();
  const auto lifetime = rclcpp::Duration::from_seconds(0.30);
  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker delete_all;
  delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(delete_all);

  auto make_marker = [&](const std::string & ns, int id, int type) {
      visualization_msgs::msg::Marker result;
      result.header.frame_id = base_frame_id_;
      result.header.stamp = stamp;
      result.ns = ns;
      result.id = id;
      result.type = type;
      result.action = visualization_msgs::msg::Marker::ADD;
      result.pose.orientation.w = 1.0;
      result.lifetime = lifetime;
      return result;
    };
  auto make_point = [](double x, double y, double z) {
      geometry_msgs::msg::Point result;
      result.x = x;
      result.y = y;
      result.z = z;
      return result;
    };

  const auto cone_clusters = collect_cone_clusters(pose);

  for (std::size_t index = 0; index < cone_clusters.size(); ++index) {
    const auto & cone = cone_clusters[index];
    const int marker_id = static_cast<int>(index);

    auto center = make_marker(
      "all_cone_centers", marker_id, visualization_msgs::msg::Marker::SPHERE);
    center.pose.position.x = cone.center.x;
    center.pose.position.y = cone.center.y;
    center.pose.position.z = 0.070;
    center.scale.x = center.scale.y = center.scale.z = 0.060;
    center.color.r = 0.00F;
    center.color.g = 0.74F;
    center.color.b = 0.83F;
    center.color.a = 1.0F;
    markers.markers.push_back(center);

    auto circle = make_marker(
      "all_cone_fit", marker_id, visualization_msgs::msg::Marker::LINE_STRIP);
    circle.scale.x = 0.020;
    circle.color.r = 0.00F;
    circle.color.g = 0.74F;
    circle.color.b = 0.83F;
    circle.color.a = 0.95F;
    for (int point_index = 0; point_index <= 64; ++point_index) {
      const double angle =
        2.0 * M_PI * static_cast<double>(point_index) / 64.0;
      circle.points.push_back(make_point(
        cone.center.x + cone.fitted_radius * std::cos(angle),
        cone.center.y + cone.fitted_radius * std::sin(angle), 0.08));
    }
    markers.markers.push_back(circle);
  }

  debug_marker_pub_->publish(markers);
}

bool VisualLidarPathFollower::stop_if_route_finished()
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
// current_index_ 表示当前参考线段起点：waypoint[i] -> waypoint[i + 1]。
void VisualLidarPathFollower::update_route_progress(const VisualLidarPathFollower::RobotPose & pose)
{
  if (waypoints_.size() < 2 || current_index_ >= waypoints_.size() - 1) {
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

// 在 current_index_ 到 current_index_ + extra_segments 的线段窗口里找车的投影。
// 只向前找，不全局找，避免闭环/椭圆路线匹配到另一侧。
bool VisualLidarPathFollower::find_route_projection(
  const VisualLidarPathFollower::RobotPose & pose, std::size_t extra_segments,
  VisualLidarPathFollower::RouteProjection & projection) const
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

// 从 current_index_ + 1 往后找第一个离车达到指定前瞻距离的离散 waypoint。
VisualLidarPathFollower::TargetPoint VisualLidarPathFollower::compute_lookahead_target(
  const VisualLidarPathFollower::RobotPose & pose, double lookahead_distance) const
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
    std::min(lookahead_distance, reverse_goal_lookahead_distance_) : lookahead_distance;
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
bool VisualLidarPathFollower::can_advance_to_index(
  std::size_t candidate_index, const VisualLidarPathFollower::RobotPose & pose, std::size_t & allowed_index) const
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
  const auto & waypoint = waypoints_[index];
  if (waypoint.reverse) {
    return std::max(waypoint.reverse_pass_radius, reverse_goal_lookahead_distance_);
  }
  return waypoint.pass_radius;
}

bool VisualLidarPathFollower::is_segment_reverse(std::size_t segment_index) const
{
  return segment_index + 1 < waypoints_.size() && waypoints_[segment_index + 1].reverse;
}

// 独立拍照点必须精确通过，不允许被路线投影或控制前瞻跨过。
bool VisualLidarPathFollower::is_precision_waypoint(std::size_t index) const
{
  if (index >= waypoints_.size()) {
    return false;
  }
  return enable_image_analysis_trigger_ && waypoints_[index].image_capture;
}

// 根据所选路线方向插入“对齐点 -> 拍照点”直线段。
// 两个坐标独立于 waypoints_channel；中间原有圆弧点会在运行时被这条直线替换。
void VisualLidarPathFollower::insert_image_capture_waypoints(const std::string & direction)
{
  has_image_capture_waypoint_ = false;
  image_capture_stopping_ = false;
  if (!enable_image_analysis_trigger_ || waypoints_.size() < 2) {
    return;
  }

  const bool clockwise = to_lower(direction) == "clockwise";
  const Point2D align_point{
    clockwise ? image_capture_clockwise_align_x_ : image_capture_counterclockwise_align_x_,
    clockwise ? image_capture_clockwise_align_y_ : image_capture_counterclockwise_align_y_};
  const Point2D capture_point{
    clockwise ? image_capture_clockwise_x_ : image_capture_counterclockwise_x_,
    clockwise ? image_capture_clockwise_y_ : image_capture_counterclockwise_y_};

  auto nearest_segment_from = [&](const Point2D & point, std::size_t first_segment) {
      double best_distance_squared = std::numeric_limits<double>::infinity();
      std::size_t best_segment = std::min(first_segment, waypoints_.size() - 2);
      for (std::size_t index = best_segment; index + 1 < waypoints_.size(); ++index) {
        const auto & start = waypoints_[index];
        const auto & finish = waypoints_[index + 1];
        const double segment_x = finish.x - start.x;
        const double segment_y = finish.y - start.y;
        const double length_squared = segment_x * segment_x + segment_y * segment_y;
        if (length_squared < 1e-9) {
          continue;
        }
        const double raw_t =
          ((point.x - start.x) * segment_x + (point.y - start.y) * segment_y) /
          length_squared;
        const double t = std::clamp(raw_t, 0.0, 1.0);
        const double projection_x = start.x + t * segment_x;
        const double projection_y = start.y + t * segment_y;
        const double dx = point.x - projection_x;
        const double dy = point.y - projection_y;
        const double distance_squared = dx * dx + dy * dy;
        if (distance_squared < best_distance_squared) {
          best_distance_squared = distance_squared;
          best_segment = index;
        }
      }
      return best_segment;
    };

  const std::size_t align_segment = nearest_segment_from(align_point, 0);
  const std::size_t capture_segment = nearest_segment_from(capture_point, align_segment);

  Waypoint align_waypoint;
  align_waypoint.name = clockwise ? "image_align_right" : "image_align_left";
  align_waypoint.x = align_point.x;
  align_waypoint.y = align_point.y;
  align_waypoint.image_capture = true;
  align_waypoint.pass_radius = image_capture_radius_;
  align_waypoint.reverse_pass_radius = image_capture_radius_;

  Waypoint capture_waypoint;
  capture_waypoint.name = clockwise ? "image_capture_right" : "image_capture_left";
  capture_waypoint.x = capture_point.x;
  capture_waypoint.y = capture_point.y;
  capture_waypoint.image_capture = true;
  capture_waypoint.pass_radius = image_capture_radius_;
  capture_waypoint.reverse_pass_radius = image_capture_radius_;

  std::vector<Waypoint> route_with_capture;
  route_with_capture.reserve(waypoints_.size() + 2);
  route_with_capture.insert(
    route_with_capture.end(), waypoints_.begin(),
    waypoints_.begin() + static_cast<std::ptrdiff_t>(align_segment + 1));
  route_with_capture.push_back(align_waypoint);
  route_with_capture.push_back(capture_waypoint);
  route_with_capture.insert(
    route_with_capture.end(),
    waypoints_.begin() + static_cast<std::ptrdiff_t>(capture_segment + 1),
    waypoints_.end());
  waypoints_ = std::move(route_with_capture);

  image_capture_waypoint_index_ = align_segment + 2;
  has_image_capture_waypoint_ = true;

  RCLCPP_INFO(
    get_logger(),
    "%s路线插入%s拍照直线：对齐点=(%.3f, %.3f)，拍照点=(%.3f, %.3f)，"
    "替换中间%zu个普通航点，到达半径=%.2fm。",
    route_direction_name(direction).c_str(), clockwise ? "右侧" : "左侧",
    align_point.x, align_point.y, capture_point.x, capture_point.y,
    capture_segment - align_segment, image_capture_radius_);
}

// 返回 true 表示本周期保持停车；稳定时间结束后发布一次触发并恢复正常巡点。
bool VisualLidarPathFollower::handle_image_capture_stop(
  const VisualLidarPathFollower::RobotPose & pose)
{
  if (!has_image_capture_waypoint_ || image_analysis_triggered_ ||
    image_capture_waypoint_index_ >= waypoints_.size() ||
    current_index_ >= image_capture_waypoint_index_)
  {
    return false;
  }
  if (current_index_ + 1 != image_capture_waypoint_index_) {
    return false;
  }

  const auto & capture = waypoints_[image_capture_waypoint_index_];
  const double distance = std::hypot(capture.x - pose.x, capture.y - pose.y);
  if (distance > image_capture_radius_) {
    image_capture_stopping_ = false;
    return false;
  }

  const auto now = std::chrono::steady_clock::now();
  if (!image_capture_stopping_) {
    image_capture_stopping_ = true;
    image_capture_stop_started_at_ = now;
    RCLCPP_INFO(
      get_logger(), "到达图生文拍照点，停车稳定 %.2fs。", image_capture_stop_time_);
  }

  const double stopped_seconds =
    std::chrono::duration<double>(now - image_capture_stop_started_at_).count();
  if (stopped_seconds < image_capture_stop_time_) {
    return true;
  }

  publish_image_analysis_trigger();
  image_capture_stopping_ = false;
  return false;
}

void VisualLidarPathFollower::apply_image_capture_speed_limit(
  const VisualLidarPathFollower::RobotPose & pose,
  geometry_msgs::msg::Twist & cmd) const
{
  if (!has_image_capture_waypoint_ || image_analysis_triggered_ ||
    image_capture_waypoint_index_ >= waypoints_.size() ||
    current_index_ >= image_capture_waypoint_index_)
  {
    return;
  }
  const auto & capture = waypoints_[image_capture_waypoint_index_];
  const double distance = std::hypot(capture.x - pose.x, capture.y - pose.y);
  if (distance > image_capture_slow_distance_) {
    return;
  }
  const double limited_speed = std::min(std::abs(cmd.linear.x), image_capture_slow_speed_);
  cmd.linear.x = std::copysign(limited_speed, cmd.linear.x);
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

// 从点云簇中心沿路线横截方向逐格前进，返回遇到禁行区、未知区域或地图边界的距离。
bool VisualLidarPathFollower::map_distance_to_blocked_along_ray(
  const VisualLidarPathFollower::Point2D & ray_origin,
  const VisualLidarPathFollower::Point2D & ray_direction,
  double & distance) const
{
  if (!latest_restriction_map_) {
    return false;
  }

  const auto & map = *latest_restriction_map_;
  const int width = static_cast<int>(map.info.width);
  const int height = static_cast<int>(map.info.height);
  const double resolution = map.info.resolution;
  const double direction_length =
    std::hypot(ray_direction.x, ray_direction.y);
  if (width <= 0 || height <= 0 || resolution <= 0.0 ||
    direction_length < 1e-8 ||
    map.data.size() < static_cast<std::size_t>(width) * static_cast<std::size_t>(height))
  {
    return false;
  }

  const auto & origin = map.info.origin;
  const auto & q = origin.orientation;
  const double origin_yaw = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  auto map_to_grid = [&](const Point2D & point, int & grid_x, int & grid_y) {
      const double dx = point.x - origin.position.x;
      const double dy = point.y - origin.position.y;
      const double grid_x_m =
        std::cos(origin_yaw) * dx + std::sin(origin_yaw) * dy;
      const double grid_y_m =
        -std::sin(origin_yaw) * dx + std::cos(origin_yaw) * dy;
      grid_x = static_cast<int>(std::floor(grid_x_m / resolution));
      grid_y = static_cast<int>(std::floor(grid_y_m / resolution));
      return grid_x >= 0 && grid_y >= 0 && grid_x < width && grid_y < height;
    };
  auto is_blocked = [&](int grid_x, int grid_y) {
      const auto value = map.data[
        static_cast<std::size_t>(grid_y) * map.info.width + grid_x];
      return value < 0 || value >= waypoint_shift_occupied_threshold_;
    };

  int grid_x = 0;
  int grid_y = 0;
  if (!map_to_grid(ray_origin, grid_x, grid_y)) {
    return false;
  }
  if (is_blocked(grid_x, grid_y)) {
    distance = 0.0;
    return true;
  }

  const Point2D unit_direction{
    ray_direction.x / direction_length,
    ray_direction.y / direction_length};
  const double step = 0.5 * resolution;
  for (double travelled = step;
    travelled <= waypoint_shift_clearance_radius_; travelled += step)
  {
    const Point2D sample{
      ray_origin.x + travelled * unit_direction.x,
      ray_origin.y + travelled * unit_direction.y};
    if (!map_to_grid(sample, grid_x, grid_y) || is_blocked(grid_x, grid_y)) {
      distance = travelled;
      return true;
    }
  }

  distance = waypoint_shift_clearance_radius_;
  return true;
}

bool VisualLidarPathFollower::original_route_normal_at(
  std::size_t waypoint_index, VisualLidarPathFollower::Point2D & normal) const
{
  if (original_waypoints_.size() < 2 || waypoint_index >= original_waypoints_.size()) {
    return false;
  }

  const std::size_t previous_index = waypoint_index > 0 ? waypoint_index - 1 : 0;
  const std::size_t next_index = std::min(
    waypoint_index + 1, original_waypoints_.size() - 1);
  const double tangent_x =
    original_waypoints_[next_index].x - original_waypoints_[previous_index].x;
  const double tangent_y =
    original_waypoints_[next_index].y - original_waypoints_[previous_index].y;
  const double tangent_length = std::hypot(tangent_x, tangent_y);
  if (tangent_length < 1e-8) {
    return false;
  }
  normal = Point2D{-tangent_y / tangent_length, tangent_x / tangent_length};
  return true;
}

bool VisualLidarPathFollower::is_static_map_point(
  const VisualLidarPathFollower::RobotPose & robot_pose,
  const VisualLidarPathFollower::Point2D & base_point) const
{
  if (!enable_static_map_filter_) {
    return false;
  }

  const Point2D map_point = base_to_map(robot_pose, base_point);
  auto occupied_in_map = [&](
      const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & map_ptr,
      int occupied_threshold) {
      if (!map_ptr) {
        return false;
      }
      const auto & map = *map_ptr;
      const int width = static_cast<int>(map.info.width);
      const int height = static_cast<int>(map.info.height);
      if (width <= 0 || height <= 0 || map.info.resolution <= 0.0 ||
        map.data.size() < static_cast<std::size_t>(width) * static_cast<std::size_t>(height))
      {
        return false;
      }

      const auto & origin = map.info.origin;
      const auto & q = origin.orientation;
      const double origin_yaw = std::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      const double dx = map_point.x - origin.position.x;
      const double dy = map_point.y - origin.position.y;
      const double grid_x_m = std::cos(origin_yaw) * dx + std::sin(origin_yaw) * dy;
      const double grid_y_m = -std::sin(origin_yaw) * dx + std::cos(origin_yaw) * dy;
      const int grid_x = static_cast<int>(std::floor(grid_x_m / map.info.resolution));
      const int grid_y = static_cast<int>(std::floor(grid_y_m / map.info.resolution));
      if (grid_x < 0 || grid_y < 0 || grid_x >= width || grid_y >= height) {
        return false;
      }

      for (int offset_y = -static_map_inflation_cells_;
        offset_y <= static_map_inflation_cells_; ++offset_y)
      {
        for (int offset_x = -static_map_inflation_cells_;
          offset_x <= static_map_inflation_cells_; ++offset_x)
        {
          const int check_x = grid_x + offset_x;
          const int check_y = grid_y + offset_y;
          if (check_x < 0 || check_y < 0 || check_x >= width || check_y >= height) {
            continue;
          }
          const auto value = map.data[
            static_cast<std::size_t>(check_y) * map.info.width + check_x];
          if (value >= occupied_threshold) {
            return true;
          }
        }
      }
      return false;
    };

  return occupied_in_map(latest_map_, static_map_occupied_threshold_) ||
         occupied_in_map(latest_restriction_map_, waypoint_shift_occupied_threshold_);
}

std::vector<VisualLidarPathFollower::Point2D>
VisualLidarPathFollower::collect_non_static_scan_points_map(
  const VisualLidarPathFollower::RobotPose & robot_pose) const
{
  if (!latest_scan_) {
    return {};
  }
  if (enable_static_map_filter_ && require_map_for_lidar_assist_ &&
    (!latest_map_ || !latest_restriction_map_))
  {
    return {};
  }

  const auto & scan = *latest_scan_;
  RobotPose scan_to_base;
  if (!scan.header.frame_id.empty() && scan.header.frame_id != base_frame_id_ &&
    !lookup_pose(
      base_frame_id_, scan.header.frame_id,
      rclcpp::Time(0, 0, get_clock()->get_clock_type()), scan_to_base))
  {
    return {};
  }

  const double cos_scan = std::cos(scan_to_base.yaw);
  const double sin_scan = std::sin(scan_to_base.yaw);
  std::vector<Point2D> points;
  points.reserve(scan.ranges.size());
  double angle = scan.angle_min;
  for (const auto range : scan.ranges) {
    if (!std::isfinite(range) || range < scan.range_min || range > scan.range_max) {
      angle += scan.angle_increment;
      continue;
    }
    const double scan_x = static_cast<double>(range) * std::cos(angle);
    const double scan_y = static_cast<double>(range) * std::sin(angle);
    const Point2D base_point{
      scan_to_base.x + cos_scan * scan_x - sin_scan * scan_y,
      scan_to_base.y + sin_scan * scan_x + cos_scan * scan_y};
    if (!enable_static_map_filter_ || !is_static_map_point(robot_pose, base_point)) {
      points.push_back(base_to_map(robot_pose, base_point));
    }
    angle += scan.angle_increment;
  }
  return points;
}

// 扫码后、开始追通道点前统计车辆周围点云：
//   1. 左右比较矩形从车前延伸到车屁股后方，只统计地图过滤后的非墙点；
//   2. 更靠后的警戒区也只统计非墙点，进入该区域后恢复巡点和 waypoint_shift。
VisualLidarPathFollower::QrEscapeObservation
VisualLidarPathFollower::observe_qr_escape_point_cloud(
  const VisualLidarPathFollower::RobotPose & robot_pose) const
{
  QrEscapeObservation observation;
  if (!latest_scan_) {
    return observation;
  }
  if (enable_static_map_filter_ && require_map_for_lidar_assist_ &&
    (!latest_map_ || !latest_restriction_map_))
  {
    return observation;
  }

  const auto & scan = *latest_scan_;
  RobotPose scan_to_base;
  if (!scan.header.frame_id.empty() && scan.header.frame_id != base_frame_id_ &&
    !lookup_pose(
      base_frame_id_, scan.header.frame_id,
      rclcpp::Time(0, 0, get_clock()->get_clock_type()), scan_to_base))
  {
    return observation;
  }

  const double cos_scan = std::cos(scan_to_base.yaw);
  const double sin_scan = std::sin(scan_to_base.yaw);
  double angle = scan.angle_min;
  for (const auto value : scan.ranges) {
    if (!std::isfinite(value) || value < scan.range_min || value > scan.range_max) {
      angle += scan.angle_increment;
      continue;
    }

    const double scan_x = static_cast<double>(value) * std::cos(angle);
    const double scan_y = static_cast<double>(value) * std::sin(angle);
    const Point2D base_point{
      scan_to_base.x + cos_scan * scan_x - sin_scan * scan_y,
      scan_to_base.y + sin_scan * scan_x + cos_scan * scan_y};
    const bool in_escape_area =
      base_point.x >= -qr_escape_rear_extension_ &&
      base_point.x <= qr_escape_check_distance_ &&
      std::abs(base_point.y) <= qr_escape_half_width_;
    const bool in_rear_warning_area =
      base_point.x <= -qr_rear_warning_start_distance_ &&
      base_point.x >= -qr_rear_check_distance_ &&
      std::abs(base_point.y) <= qr_rear_half_width_;
    if ((in_escape_area || in_rear_warning_area) &&
      !is_static_map_point(robot_pose, base_point))
    {
      if (in_escape_area) {
        if (base_point.y >= 0.0) {
          ++observation.area_left_count;
        } else {
          ++observation.area_right_count;
        }
      } else if (base_point.y >= 0.0) {
        ++observation.rear_left_count;
      } else {
        ++observation.rear_right_count;
      }
    }
    angle += scan.angle_increment;
  }

  observation.ready = true;
  return observation;
}

// 返回 true 表示本周期由二维码后撤接管 cmd；返回 false 后继续正常追点。
bool VisualLidarPathFollower::apply_qr_escape(
  const VisualLidarPathFollower::RobotPose & robot_pose,
  geometry_msgs::msg::Twist & cmd)
{
  if (qr_escape_state_ == QrEscapeState::IDLE) {
    return false;
  }

  std::size_t nearest_waypoint_index = 0;
  double nearest_waypoint_distance = std::numeric_limits<double>::infinity();
  const bool has_nearest_waypoint = find_nearest_waypoint(
    robot_pose, nearest_waypoint_index, nearest_waypoint_distance);
  const bool reached_protected_waypoint =
    has_nearest_waypoint && nearest_waypoint_distance <= qr_protect_exit_waypoint_distance_;

  auto finish_qr_protection = [&](const char * reason) {
      qr_escape_state_ = QrEscapeState::IDLE;
      qr_escape_rear_blocked_ = false;
      first_waypoint_reached_ = true;
      RCLCPP_INFO(
        get_logger(),
        "二维码后撤保护结束：最近航点=%zu/%zu，距离=%.2fm，当前巡点=%zu/%zu，原因=%s。",
        nearest_waypoint_index + 1, waypoints_.size(), nearest_waypoint_distance,
        current_index_ + 1, waypoints_.size(), reason);
    };

  latest_qr_escape_observation_ = observe_qr_escape_point_cloud(robot_pose);
  const auto & observation = latest_qr_escape_observation_;
  if (!observation.ready) {
    if (reached_protected_waypoint) {
      finish_qr_protection("已进入航点距离范围");
      return false;
    }
    qr_escape_rear_blocked_ = true;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "二维码后撤等待有效雷达和地图数据，发布零速度。");
    return true;
  }

  const int area_count = observation.area_left_count + observation.area_right_count;
  const int rear_count = observation.rear_left_count + observation.rear_right_count;
  if (rear_count >= qr_rear_min_points_) {
    qr_escape_state_ = QrEscapeState::CHECKING;
    qr_escape_rear_blocked_ = true;
    if (reached_protected_waypoint) {
      finish_qr_protection("已进入航点距离范围");
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "二维码后方警戒区出现非墙点：左=%d，右=%d，暂停后撤并继续巡点 waypoint_shift；"
        "最近航点距离=%.2fm。",
        observation.rear_left_count, observation.rear_right_count, nearest_waypoint_distance);
    }
    return false;
  }

  if (area_count < qr_escape_min_points_) {
    qr_escape_state_ = QrEscapeState::CHECKING;
    qr_escape_rear_blocked_ = false;
    if (reached_protected_waypoint) {
      finish_qr_protection("已进入航点距离范围");
    } else if (debug_control_logs_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "debug: 二维码后撤保护保持：框内左/右=%d/%d，最近航点距离=%.2fm。",
        observation.area_left_count, observation.area_right_count, nearest_waypoint_distance);
    }
    return false;
  }

  qr_escape_state_ = QrEscapeState::BACKING_RIGHT;

  if (debug_control_logs_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(), 1000,
      "二维码后撤矩形内非墙点：左=%d，右=%d，默认向右后撤；最近航点距离=%.2fm。",
      observation.area_left_count, observation.area_right_count,
      nearest_waypoint_distance);
  }

  qr_escape_rear_blocked_ = false;
  cmd.linear.x = -qr_reverse_linear_speed_;
  cmd.angular.z = qr_escape_state_ == QrEscapeState::BACKING_RIGHT ?
    qr_reverse_angular_speed_ : -qr_reverse_angular_speed_;
  if (debug_control_logs_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 500,
      "debug: 二维码向%s后撤，框内左/右=%d/%d，后方警戒左/右=%d/%d，输出=%.2f/%.2f。",
      qr_escape_state_ == QrEscapeState::BACKING_RIGHT ? "右" : "左",
      observation.area_left_count, observation.area_right_count,
      observation.rear_left_count, observation.rear_right_count,
      cmd.linear.x, cmd.angular.z);
  }
  return true;
}

// 用簇内二维雷达点拟合圆：
//   2*u*cx + 2*v*cy + c = u^2 + v^2
// 先减去点集均值改善数值稳定性，再用 3x3 高斯消元解最小二乘法方程。
bool VisualLidarPathFollower::fit_cone_circle(
  VisualLidarPathFollower::LidarCluster & cluster) const
{
  if (cluster.points.size() < static_cast<std::size_t>(cluster_min_points_)) {
    return false;
  }

  double mean_x = 0.0;
  double mean_y = 0.0;
  for (const auto & point : cluster.points) {
    mean_x += point.x;
    mean_y += point.y;
  }
  const double point_count = static_cast<double>(cluster.points.size());
  mean_x /= point_count;
  mean_y /= point_count;

  double system[3][4] = {};
  for (const auto & point : cluster.points) {
    const double u = point.x - mean_x;
    const double v = point.y - mean_y;
    const double rhs = u * u + v * v;
    const double row[3] = {2.0 * u, 2.0 * v, 1.0};
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        system[r][c] += row[r] * row[c];
      }
      system[r][3] += row[r] * rhs;
    }
  }

  for (int column = 0; column < 3; ++column) {
    int pivot_row = column;
    for (int row = column + 1; row < 3; ++row) {
      if (std::abs(system[row][column]) > std::abs(system[pivot_row][column])) {
        pivot_row = row;
      }
    }
    if (std::abs(system[pivot_row][column]) < 1e-10) {
      return false;
    }
    if (pivot_row != column) {
      for (int c = column; c < 4; ++c) {
        std::swap(system[column][c], system[pivot_row][c]);
      }
    }

    const double pivot = system[column][column];
    for (int c = column; c < 4; ++c) {
      system[column][c] /= pivot;
    }
    for (int row = 0; row < 3; ++row) {
      if (row == column) {
        continue;
      }
      const double factor = system[row][column];
      for (int c = column; c < 4; ++c) {
        system[row][c] -= factor * system[column][c];
      }
    }
  }

  const double local_center_x = system[0][3];
  const double local_center_y = system[1][3];
  const double radius_squared =
    local_center_x * local_center_x + local_center_y * local_center_y + system[2][3];
  if (!std::isfinite(radius_squared) || radius_squared <= 0.0) {
    return false;
  }

  const double radius = std::sqrt(radius_squared);
  const Point2D center{mean_x + local_center_x, mean_y + local_center_y};
  if (!std::isfinite(center.x) || !std::isfinite(center.y) ||
    radius < cone_circle_min_radius_ || radius > cone_circle_max_radius_)
  {
    return false;
  }

  double squared_error_sum = 0.0;
  for (const auto & point : cluster.points) {
    const double radial_error = std::hypot(point.x - center.x, point.y - center.y) - radius;
    squared_error_sum += radial_error * radial_error;
  }
  const double rmse = std::sqrt(squared_error_sum / point_count);
  if (!std::isfinite(rmse) || rmse > cone_circle_max_rmse_) {
    return false;
  }

  cluster.center = center;
  cluster.fitted_radius = radius;
  cluster.fit_rmse = rmse;
  return true;
}

std::vector<VisualLidarPathFollower::LidarCluster>
VisualLidarPathFollower::collect_cone_clusters(
  const VisualLidarPathFollower::RobotPose & robot_pose) const
{
  if (!latest_scan_) {
    return {};
  }
  if (enable_static_map_filter_ && require_map_for_lidar_assist_ &&
    (!latest_map_ || !latest_restriction_map_))
  {
    return {};
  }

  const auto & scan = *latest_scan_;

  RobotPose scan_to_base;
  if (!scan.header.frame_id.empty() && scan.header.frame_id != base_frame_id_ &&
    !lookup_pose(
      base_frame_id_, scan.header.frame_id,
      rclcpp::Time(0, 0, get_clock()->get_clock_type()), scan_to_base))
  {
    return {};
  }

  std::vector<LidarCluster> clusters;
  bool cluster_active = false;
  LidarCluster current_cluster;
  Point2D previous_point;
  double previous_range = 0.0;

  auto finish_cluster = [&]() {
      if (cluster_active && current_cluster.point_count >= cluster_min_points_ &&
        fit_cone_circle(current_cluster))
      {
        const bool static_map_cluster =
          enable_static_map_filter_ && is_static_map_point(robot_pose, current_cluster.center);
        if (!static_map_cluster) {
          clusters.push_back(current_cluster);
        }
      }
      current_cluster = LidarCluster();
      cluster_active = false;
    };

  double angle = scan.angle_min;
  for (const auto value : scan.ranges) {
    bool valid = std::isfinite(value) && value >= scan.range_min && value <= scan.range_max;
    Point2D base_point;
    if (valid) {
      const double scan_x = static_cast<double>(value) * std::cos(angle);
      const double scan_y = static_cast<double>(value) * std::sin(angle);
      const double cos_scan = std::cos(scan_to_base.yaw);
      const double sin_scan = std::sin(scan_to_base.yaw);
      base_point.x = scan_to_base.x + cos_scan * scan_x - sin_scan * scan_y;
      base_point.y = scan_to_base.y + sin_scan * scan_x + cos_scan * scan_y;
    }
    if (!valid) {
      finish_cluster();
      angle += scan.angle_increment;
      continue;
    }

    const double base_range = std::hypot(base_point.x, base_point.y);
    const bool connected = cluster_active &&
      std::abs(base_range - previous_range) <= cluster_max_range_jump_ &&
      std::hypot(base_point.x - previous_point.x, base_point.y - previous_point.y) <=
      cluster_max_point_gap_;
    if (cluster_active && !connected) {
      finish_cluster();
    }
    if (!cluster_active) {
      cluster_active = true;
    }

    current_cluster.points.push_back(base_point);
    ++current_cluster.point_count;
    previous_point = base_point;
    previous_range = base_range;
    angle += scan.angle_increment;
  }
  finish_cluster();
  return clusters;
}

void VisualLidarPathFollower::reset_waypoint_shift_state()
{
  original_waypoints_ = waypoints_;
  waypoint_shift_offsets_.assign(waypoints_.size(), 0.0);
  processed_waypoint_shift_scan_sequence_ = scan_sequence_;
}

// waypoint_shift 模式：每帧新雷达提前检查当前目标之后的固定数量航点。
// 以障碍物最近路线点为中心，前后分别取配置数量的点，整段向地图更空旷侧横移。
// 横移量由路线到点云的期望间距计算并保存；重叠影响只保留绝对偏移更大的候选。
void VisualLidarPathFollower::update_waypoint_shifts(
  const VisualLidarPathFollower::RobotPose & robot_pose,
  std::size_t target_waypoint_index)
{
  if (!latest_scan_ ||
    processed_waypoint_shift_scan_sequence_ == scan_sequence_ ||
    original_waypoints_.size() != waypoints_.size() ||
    waypoint_shift_offsets_.size() != waypoints_.size() || waypoints_.empty())
  {
    return;
  }
  processed_waypoint_shift_scan_sequence_ = scan_sequence_;

  const auto scan_points = collect_non_static_scan_points_map(robot_pose);
  if (scan_points.empty()) {
    return;
  }

  const std::size_t first_index = std::min(target_waypoint_index, waypoints_.size() - 1);
  const std::size_t last_index = std::min(
    waypoints_.size(), first_index + static_cast<std::size_t>(waypoint_shift_check_count_));
  const double check_radius_squared =
    waypoint_cloud_check_radius_ * waypoint_cloud_check_radius_;

  struct ScanPointCluster
  {
    std::vector<Point2D> points;
    Point2D center;
  };
  std::vector<ScanPointCluster> clusters;
  ScanPointCluster current_cluster;
  Point2D previous_point;
  double previous_range = 0.0;
  auto finish_cluster = [&]() {
      if (current_cluster.points.size() >= static_cast<std::size_t>(cluster_min_points_)) {
        for (const auto & point : current_cluster.points) {
          current_cluster.center.x += point.x;
          current_cluster.center.y += point.y;
        }
        const double count = static_cast<double>(current_cluster.points.size());
        current_cluster.center.x /= count;
        current_cluster.center.y /= count;
        clusters.push_back(std::move(current_cluster));
      }
      current_cluster = ScanPointCluster();
    };

  for (const auto & point : scan_points) {
    const double point_range = std::hypot(point.x - robot_pose.x, point.y - robot_pose.y);
    const bool connected = !current_cluster.points.empty() &&
      std::abs(point_range - previous_range) <= cluster_max_range_jump_ &&
      std::hypot(point.x - previous_point.x, point.y - previous_point.y) <=
      cluster_max_point_gap_;
    if (!current_cluster.points.empty() && !connected) {
      finish_cluster();
    }
    current_cluster.points.push_back(point);
    previous_point = point;
    previous_range = point_range;
  }
  finish_cluster();

  bool route_changed = false;
  for (const auto & cluster : clusters) {
    const bool cluster_hits_window = std::any_of(
      cluster.points.begin(), cluster.points.end(), [&](const Point2D & point) {
        for (std::size_t index = first_index; index < last_index; ++index) {
          const double dx = point.x - original_waypoints_[index].x;
          const double dy = point.y - original_waypoints_[index].y;
          if (dx * dx + dy * dy <= check_radius_squared) {
            return true;
          }
        }
        return false;
      });
    if (!cluster_hits_window) {
      continue;
    }

    // 只在当前提前检查窗口附近找最近路线段，避免闭环另一侧的邻近路线抢走障碍物。
    const std::size_t segment_first = first_index > 0 ? first_index - 1 : first_index;
    const std::size_t segment_last = std::min(last_index, original_waypoints_.size() - 1);
    double best_distance_squared = std::numeric_limits<double>::infinity();
    std::size_t best_segment = segment_first;
    double best_t = 0.0;
    Point2D route_tangent;
    Point2D route_projection;
    for (std::size_t index = segment_first; index < segment_last; ++index) {
      const auto & start = original_waypoints_[index];
      const auto & finish = original_waypoints_[index + 1];
      const double segment_x = finish.x - start.x;
      const double segment_y = finish.y - start.y;
      const double length_squared = segment_x * segment_x + segment_y * segment_y;
      if (length_squared < 1e-8) {
        continue;
      }
      const double raw_t =
        ((cluster.center.x - start.x) * segment_x +
        (cluster.center.y - start.y) * segment_y) / length_squared;
      const double projection_t = std::max(0.0, std::min(1.0, raw_t));
      const double projection_x = start.x + projection_t * segment_x;
      const double projection_y = start.y + projection_t * segment_y;
      const double dx = cluster.center.x - projection_x;
      const double dy = cluster.center.y - projection_y;
      const double distance_squared = dx * dx + dy * dy;
      if (distance_squared < best_distance_squared) {
        const double length = std::sqrt(length_squared);
        best_distance_squared = distance_squared;
        best_segment = index;
        best_t = projection_t;
        route_tangent = Point2D{segment_x / length, segment_y / length};
        route_projection = Point2D{projection_x, projection_y};
      }
    }
    if (!std::isfinite(best_distance_squared)) {
      continue;
    }

    const Point2D & obstacle_point = cluster.center;
    const Point2D left_normal{-route_tangent.y, route_tangent.x};
    const Point2D right_normal{-left_normal.x, -left_normal.y};
    double left_distance = 0.0;
    double right_distance = 0.0;
    if (!map_distance_to_blocked_along_ray(
        obstacle_point, left_normal, left_distance) ||
      !map_distance_to_blocked_along_ray(
        obstacle_point, right_normal, right_distance))
    {
      continue;
    }
    const int shift_direction = left_distance > right_distance ? 1 : -1;
    const Point2D shift_normal = shift_direction > 0 ? left_normal : right_normal;

    // 直接计算点云簇在空旷侧横截方向上的最外沿，再越过它配置的目标净距。
    // required_offset 是从原路线投影点出发的位移，不设置最大偏移上限。
    double cloud_edge_along_shift = -std::numeric_limits<double>::infinity();
    for (const auto & point : cluster.points) {
      const double lateral_distance =
        (point.x - route_projection.x) * shift_normal.x +
        (point.y - route_projection.y) * shift_normal.y;
      cloud_edge_along_shift = std::max(cloud_edge_along_shift, lateral_distance);
    }
    const double required_offset = std::max(
      0.0, cloud_edge_along_shift + waypoint_shift_target_clearance_);
    if (required_offset <= 1e-9) {
      continue;
    }

    std::size_t obstacle_waypoint = best_t < 0.5 ? best_segment : best_segment + 1;
    obstacle_waypoint = std::max(
      first_index, std::min(obstacle_waypoint, original_waypoints_.size() - 1));
    const std::size_t points_before =
      static_cast<std::size_t>(waypoint_shift_points_before_);
    const std::size_t points_after =
      static_cast<std::size_t>(waypoint_shift_points_after_);
    const std::size_t influence_first = std::max(
      first_index,
      obstacle_waypoint > points_before ? obstacle_waypoint - points_before : 0);
    const std::size_t influence_last = std::min(
      original_waypoints_.size() - 1, obstacle_waypoint + points_after);
    std::size_t updated_count = 0;
    for (std::size_t index = influence_first; index <= influence_last; ++index) {
      if (original_waypoints_[index].image_capture) {
        continue;
      }
      const double candidate = static_cast<double>(shift_direction) * required_offset;
      // 重叠区不相加；只保存绝对偏移更大的障碍物影响。
      if (std::abs(candidate) > std::abs(waypoint_shift_offsets_[index]) + 1e-9) {
        waypoint_shift_offsets_[index] = candidate;
        ++updated_count;
        route_changed = true;
      }
    }

    if (updated_count > 0) {
      RCLCPP_WARN(
        get_logger(),
        "waypoint_shift：障碍物最近点=%zu，统一偏移点=%zu-%zu，"
        "横截线左/右空旷距离=%.3f/%.3fm，向%s偏移=%.3fm，"
        "与点云横截净距=%.3fm，更新%zu个点。",
        obstacle_waypoint + 1, influence_first + 1, influence_last + 1,
        left_distance, right_distance, shift_direction > 0 ? "左" : "右",
        required_offset, waypoint_shift_target_clearance_, updated_count);
    }
  }

  if (route_changed) {
    for (std::size_t index = 0; index < waypoints_.size(); ++index) {
      waypoints_[index] = original_waypoints_[index];
      if (std::abs(waypoint_shift_offsets_[index]) < 1e-9) {
        continue;
      }
      Point2D normal;
      if (!original_route_normal_at(index, normal)) {
        continue;
      }
      const double lateral_offset = waypoint_shift_offsets_[index];
      waypoints_[index].x += lateral_offset * normal.x;
      waypoints_[index].y += lateral_offset * normal.y;
    }
    publish_debug_route();
  }
}

// ********************************************************************************
// ******************************** QR 与图生文触发 ********************************
// ********************************************************************************

// QR 回调：
//   1. 根据 QR 数字判断顺/逆时针；
//   2. 从二维码前轨迹切换到 route_waypoints_file_ 中对应方向的环线。
void VisualLidarPathFollower::qr_callback(const std_msgs::msg::String::ConstSharedPtr msg)
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
//   扫码前在 waypoints_file_ 中跑二维码前轨迹；
//   扫到 QR 后加载 route_waypoints_file_，并从该方向环线的第一个航点开始追。
bool VisualLidarPathFollower::switch_route(const std::string & direction)
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
  insert_image_capture_waypoints(selected_route_direction_);
  reset_waypoint_shift_state();
  waiting_for_qr_start_ = false;
  following_qr_path_ = false;
  qr_path_finished_ = false;
  image_analysis_triggered_ = false;
  if (was_following_qr_path || was_waiting_for_qr) {
    current_index_ = 0;
    first_waypoint_reached_ = false;
    qr_escape_state_ = QrEscapeState::CHECKING;
    latest_qr_escape_observation_ = QrEscapeObservation();
    qr_escape_rear_blocked_ = false;
  } else {
    current_index_ = std::min(current_index_, waypoints_.size() - 1);
    first_waypoint_reached_ = current_index_ > 0;
  }
  const auto route_name = route_direction_name(selected_route_direction_);
  RCLCPP_INFO(get_logger(), "QR 选择%s路线，开始环线。", route_name.c_str());
  if (debug_control_logs_) {
    RCLCPP_INFO(
      get_logger(), "debug: 环线文件=%s，起点=%zu/%zu（%s）。",
      route_waypoints_file_.c_str(), current_index_ + 1, waypoints_.size(),
      waypoints_[current_index_].name.c_str());
  }
  publish_debug_route();
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
  RCLCPP_INFO(get_logger(), "已通知 visual_control 恢复，后续交给视觉停车。");
  if (debug_control_logs_) {
    RCLCPP_INFO(
      get_logger(), "debug: resume_topic=%s, value=%d。",
      visual_control_suspend_topic_.c_str(), visual_control_resume_value_);
  }
}

void VisualLidarPathFollower::publish_image_analysis_trigger()
{
  if (following_qr_path_ || !enable_image_analysis_trigger_ || image_analysis_triggered_) {
    return;
  }

  std_msgs::msg::Int32 msg;
  msg.data = image_capture_trigger_value_;
  image_analysis_trigger_pub_->publish(msg);
  image_analysis_triggered_ = true;
  RCLCPP_INFO(get_logger(), "图生文拍照触发：发布 %d。", image_capture_trigger_value_);
  if (debug_control_logs_) {
    RCLCPP_INFO(
      get_logger(), "debug: 触发话题=%s。", image_analysis_trigger_topic_.c_str());
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
std::vector<Waypoint> VisualLidarPathFollower::load_waypoints(
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
// 普通巡点传入时间 0，直接取得当前可用的最新车辆位姿。
bool VisualLidarPathFollower::lookup_robot_pose_at(const rclcpp::Time & stamp, VisualLidarPathFollower::RobotPose & pose) const
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
bool VisualLidarPathFollower::lookup_robot_pose(double & x, double & y, double & yaw)
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
  auto node = std::make_shared<VisualLidarPathFollower>();
  rclcpp::spin(node);
  if (node->should_publish_stop_on_exit()) {
    node->publish_stop();
  }
  rclcpp::shutdown();
  return 0;
}
