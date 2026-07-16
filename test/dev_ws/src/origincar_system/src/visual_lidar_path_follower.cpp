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

// 与阿克曼下位机 balance.c 保持一致的前轮转角到舵机目标 PWM 标定。
constexpr double kSteeringAngleLimit = 0.55;
constexpr double kServoPwmCenter = 1500.0;
constexpr double kServoPwmLeftLimit = 1050.0;
constexpr double kServoPwmRightLimit = 2200.0;
constexpr double kServoPwmPerRadian = 636.56;

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
  declare_parameter<std::string>(
    "route_waypoints_clockwise_file", route_waypoints_clockwise_file_);
  get_parameter("route_waypoints_clockwise_file", route_waypoints_clockwise_file_);
  declare_parameter<std::string>(
    "route_waypoints_counterclockwise_file", route_waypoints_counterclockwise_file_);
  get_parameter(
    "route_waypoints_counterclockwise_file", route_waypoints_counterclockwise_file_);
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
  declare_parameter<bool>("enable_turn_speed_slowdown", enable_turn_speed_slowdown_);
  get_parameter("enable_turn_speed_slowdown", enable_turn_speed_slowdown_);
  declare_parameter<double>(
    "max_turn_speed_reduction_percent", max_turn_speed_reduction_percent_);
  get_parameter(
    "max_turn_speed_reduction_percent", max_turn_speed_reduction_percent_);
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
  declare_parameter<int>("waypoint_shift_confirm_scans", waypoint_shift_confirm_scans_);
  get_parameter("waypoint_shift_confirm_scans", waypoint_shift_confirm_scans_);
  declare_parameter<int>("waypoint_shift_release_scans", waypoint_shift_release_scans_);
  get_parameter("waypoint_shift_release_scans", waypoint_shift_release_scans_);
  declare_parameter<double>("waypoint_shift_min_change", waypoint_shift_min_change_);
  get_parameter("waypoint_shift_min_change", waypoint_shift_min_change_);
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
    "qr_reverse_min_map_x", qr_reverse_min_map_x_);
  get_parameter("qr_reverse_min_map_x", qr_reverse_min_map_x_);
  declare_parameter<double>(
    "qr_visual_handoff_waypoint_distance", qr_visual_handoff_waypoint_distance_);
  get_parameter(
    "qr_visual_handoff_waypoint_distance", qr_visual_handoff_waypoint_distance_);

  // 只用于“扫码后 -> 进入 waypoint_channel 前”的独立视觉避障参数组。
  declare_parameter<std::string>(
    "qr_channel_visual_detection_topic", qr_channel_visual_detection_topic_);
  get_parameter("qr_channel_visual_detection_topic", qr_channel_visual_detection_topic_);
  declare_parameter<std::string>(
    "qr_channel_visual_obstacle_class_name", qr_channel_visual_obstacle_class_name_);
  get_parameter(
    "qr_channel_visual_obstacle_class_name", qr_channel_visual_obstacle_class_name_);
  declare_parameter<double>(
    "qr_channel_visual_obstacle_confidence_threshold",
    qr_channel_visual_obstacle_confidence_threshold_);
  get_parameter(
    "qr_channel_visual_obstacle_confidence_threshold",
    qr_channel_visual_obstacle_confidence_threshold_);
  declare_parameter<int>(
    "qr_channel_visual_obstacle_bottom_threshold",
    qr_channel_visual_obstacle_bottom_threshold_);
  get_parameter(
    "qr_channel_visual_obstacle_bottom_threshold",
    qr_channel_visual_obstacle_bottom_threshold_);
  declare_parameter<double>(
    "qr_channel_visual_obstacle_delta_x_ratio",
    qr_channel_visual_obstacle_delta_x_ratio_);
  get_parameter(
    "qr_channel_visual_obstacle_delta_x_ratio",
    qr_channel_visual_obstacle_delta_x_ratio_);
  declare_parameter<double>(
    "qr_channel_visual_avoid_linear_speed", qr_channel_visual_avoid_linear_speed_);
  get_parameter(
    "qr_channel_visual_avoid_linear_speed", qr_channel_visual_avoid_linear_speed_);
  declare_parameter<double>(
    "qr_channel_visual_avoid_angular_ratio", qr_channel_visual_avoid_angular_ratio_);
  get_parameter(
    "qr_channel_visual_avoid_angular_ratio", qr_channel_visual_avoid_angular_ratio_);
  declare_parameter<double>(
    "qr_channel_visual_detection_timeout", qr_channel_visual_detection_timeout_);
  get_parameter(
    "qr_channel_visual_detection_timeout", qr_channel_visual_detection_timeout_);
  declare_parameter<int>(
    "qr_channel_visual_detection_source_width", qr_channel_visual_detection_source_width_);
  get_parameter(
    "qr_channel_visual_detection_source_width", qr_channel_visual_detection_source_width_);
  declare_parameter<int>(
    "qr_channel_visual_detection_source_height", qr_channel_visual_detection_source_height_);
  get_parameter(
    "qr_channel_visual_detection_source_height", qr_channel_visual_detection_source_height_);
  declare_parameter<int>(
    "qr_channel_visual_control_image_width", qr_channel_visual_control_image_width_);
  get_parameter(
    "qr_channel_visual_control_image_width", qr_channel_visual_control_image_width_);
  declare_parameter<int>(
    "qr_channel_visual_control_image_height", qr_channel_visual_control_image_height_);
  get_parameter(
    "qr_channel_visual_control_image_height", qr_channel_visual_control_image_height_);

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
  declare_parameter<int>(
    "perception_obstacle_only_value", perception_obstacle_only_value_);
  get_parameter("perception_obstacle_only_value", perception_obstacle_only_value_);
  declare_parameter<int>("perception_full_resume_value", perception_full_resume_value_);
  get_parameter("perception_full_resume_value", perception_full_resume_value_);
  declare_parameter<int>("perception_pause_value", perception_pause_value_);
  get_parameter("perception_pause_value", perception_pause_value_);
  declare_parameter<int>("clockwise_qr_value", clockwise_qr_value_);
  get_parameter("clockwise_qr_value", clockwise_qr_value_);
  declare_parameter<int>("counterclockwise_qr_value", counterclockwise_qr_value_);
  get_parameter("counterclockwise_qr_value", counterclockwise_qr_value_);
  declare_parameter<bool>("enable_image_analysis_trigger", enable_image_analysis_trigger_);
  get_parameter("enable_image_analysis_trigger", enable_image_analysis_trigger_);
  declare_parameter<double>("image_capture_clockwise_x", image_capture_clockwise_x_);
  get_parameter("image_capture_clockwise_x", image_capture_clockwise_x_);
  declare_parameter<double>("image_capture_clockwise_y", image_capture_clockwise_y_);
  get_parameter("image_capture_clockwise_y", image_capture_clockwise_y_);
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
  declare_parameter<double>("image_capture_angular_gain", image_capture_angular_gain_);
  get_parameter("image_capture_angular_gain", image_capture_angular_gain_);
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
  max_turn_speed_reduction_percent_ = std::max(
    0.0, std::min(90.0, max_turn_speed_reduction_percent_));
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
  waypoint_shift_confirm_scans_ = std::max(1, waypoint_shift_confirm_scans_);
  waypoint_shift_release_scans_ = std::max(1, waypoint_shift_release_scans_);
  waypoint_shift_min_change_ = std::max(0.0, waypoint_shift_min_change_);
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
  qr_visual_handoff_waypoint_distance_ = std::max(
    0.01, qr_visual_handoff_waypoint_distance_);
  qr_channel_visual_obstacle_confidence_threshold_ = std::max(
    0.0, std::min(1.0, qr_channel_visual_obstacle_confidence_threshold_));
  qr_channel_visual_obstacle_bottom_threshold_ = std::max(
    0, qr_channel_visual_obstacle_bottom_threshold_);
  qr_channel_visual_obstacle_delta_x_ratio_ = std::max(
    0.0, qr_channel_visual_obstacle_delta_x_ratio_);
  qr_channel_visual_avoid_linear_speed_ = std::max(
    0.0, qr_channel_visual_avoid_linear_speed_);
  qr_channel_visual_detection_timeout_ = std::max(
    0.05, qr_channel_visual_detection_timeout_);
  qr_channel_visual_detection_source_width_ = std::max(
    1, qr_channel_visual_detection_source_width_);
  qr_channel_visual_detection_source_height_ = std::max(
    1, qr_channel_visual_detection_source_height_);
  qr_channel_visual_control_image_width_ = std::max(
    1, qr_channel_visual_control_image_width_);
  qr_channel_visual_control_image_height_ = std::max(
    1, qr_channel_visual_control_image_height_);
  if (qr_channel_visual_obstacle_class_name_.empty()) {
    qr_channel_visual_obstacle_class_name_ = "construction_cone";
  }
  image_capture_radius_ = std::max(0.01, image_capture_radius_);
  image_capture_slow_distance_ = std::max(image_capture_radius_, image_capture_slow_distance_);
  image_capture_slow_speed_ = std::max(0.0, image_capture_slow_speed_);
  image_capture_angular_gain_ = std::max(0.0, image_capture_angular_gain_);
  image_capture_stop_time_ = std::max(0.0, image_capture_stop_time_);
  if (route_waypoints_clockwise_file_.empty()) {
    route_waypoints_clockwise_file_ = waypoints_file_;
  }
  if (route_waypoints_counterclockwise_file_.empty()) {
    route_waypoints_counterclockwise_file_ = waypoints_file_;
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
  visual_detection_sub_ = create_subscription<ai_msgs::msg::PerceptionTargets>(
    qr_channel_visual_detection_topic_, rclcpp::SensorDataQoS().keep_last(1),
    [this](ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg) {
      latest_visual_detection_ = msg;
      latest_visual_detection_received_at_ = std::chrono::steady_clock::now();
      has_visual_detection_ = true;
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
      "debug: 倒车点=%zu，扫码前=%s，顺时针=%s，逆时针=%s，控制前瞻=%.2f，"
      "避障=进场视觉覆盖+channel waypoint_shift，线速度=%.2f。",
      static_cast<std::size_t>(reverse_count), waypoints_file_.c_str(),
      route_waypoints_clockwise_file_.c_str(), route_waypoints_counterclockwise_file_.c_str(),
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

  if (qr_visual_avoidance_active_) {
    update_qr_visual_avoidance_phase(pose);
    geometry_msgs::msg::Twist reverse_cmd;
    if (qr_visual_avoidance_active_ && apply_one_shot_qr_reverse(pose, reverse_cmd)) {
      publish_debug_visualization(pose);
      set_control_state(next_control_state_);
      cmd_pub_->publish(reverse_cmd);
      return;
    }
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
  if (!target.reverse && !qr_visual_avoidance_active_) {
    update_waypoint_shifts(pose, target.waypoint_index);
    target = compute_lookahead_target(pose, lookahead_distance_);
  }

  auto cmd = compute_tracking_command(pose, target);
  apply_turn_speed_limit(cmd);
  apply_image_capture_speed_limit(pose, cmd);
  if (qr_visual_avoidance_active_) {
    apply_qr_channel_visual_obstacle_avoidance(cmd);
  }

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
    resume_visual_control(true);
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

// 根据所选路线方向，在离配置坐标最近的路线段中插入唯一拍照点。
// 拍照点不能被前瞻跳过，但会和普通航点一样参与 waypoint_shift 动态避障。
void VisualLidarPathFollower::insert_image_capture_waypoint(const std::string & direction)
{
  has_image_capture_waypoint_ = false;
  image_capture_stopping_ = false;
  if (!enable_image_analysis_trigger_ || waypoints_.size() < 2) {
    return;
  }

  const bool clockwise = to_lower(direction) == "clockwise";
  const Point2D capture_point{
    clockwise ? image_capture_clockwise_x_ : image_capture_counterclockwise_x_,
    clockwise ? image_capture_clockwise_y_ : image_capture_counterclockwise_y_};

  auto nearest_segment = [&](const Point2D & point) {
      double best_distance_squared = std::numeric_limits<double>::infinity();
      std::size_t best_segment = 0;
      for (std::size_t index = 0; index + 1 < waypoints_.size(); ++index) {
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

  const std::size_t capture_segment = nearest_segment(capture_point);

  Waypoint capture_waypoint;
  capture_waypoint.name = clockwise ? "image_capture_right" : "image_capture_left";
  capture_waypoint.x = capture_point.x;
  capture_waypoint.y = capture_point.y;
  capture_waypoint.image_capture = true;
  capture_waypoint.pass_radius = image_capture_radius_;
  capture_waypoint.reverse_pass_radius = image_capture_radius_;

  std::vector<Waypoint> route_with_capture;
  route_with_capture.reserve(waypoints_.size() + 1);
  route_with_capture.insert(
    route_with_capture.end(), waypoints_.begin(),
    waypoints_.begin() + static_cast<std::ptrdiff_t>(capture_segment + 1));
  route_with_capture.push_back(capture_waypoint);
  route_with_capture.insert(
    route_with_capture.end(),
    waypoints_.begin() + static_cast<std::ptrdiff_t>(capture_segment + 1),
    waypoints_.end());
  waypoints_ = std::move(route_with_capture);

  image_capture_waypoint_index_ = capture_segment + 1;
  has_image_capture_waypoint_ = true;

  RCLCPP_INFO(
    get_logger(),
    "%s路线插入唯一%s拍照点：原始坐标=(%.3f, %.3f)，"
    "插入位置=%zu/%zu，到达半径=%.2fm，允许 waypoint_shift 平移。",
    route_direction_name(direction).c_str(), clockwise ? "右侧" : "左侧",
    capture_point.x, capture_point.y, image_capture_waypoint_index_ + 1, waypoints_.size(),
    image_capture_radius_);
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
    const bool tracking_image_capture =
      enable_image_analysis_trigger_ && target.waypoint_index < waypoints_.size() &&
      waypoints_[target.waypoint_index].image_capture;
    const double angular_gain =
      tracking_image_capture ? image_capture_angular_gain_ : turn_angular_gain_;
    cmd.linear.x = linear_speed_;
    cmd.angular.z = angular_gain * heading_error;
  }
  return cmd;
}

// 根据即将发送给下位机的前轮目标转角，复现 balance.c 的舵机 PWM 换算。
// 左右 PWM 行程分别归一化，因此 1050 和 2200 打满时都会使用相同的最大降速百分比。
void VisualLidarPathFollower::apply_turn_speed_limit(
  geometry_msgs::msg::Twist & cmd) const
{
  if (!enable_turn_speed_slowdown_ || std::abs(cmd.linear.x) <= 1.0e-9) {
    return;
  }

  const double steering_angle = std::max(
    -kSteeringAngleLimit, std::min(kSteeringAngleLimit, cmd.angular.z));
  const double angle_squared = steering_angle * steering_angle;
  const double angle_cubed = angle_squared * steering_angle;

  double servo_angle = 0.0;
  if (steering_angle < 0.0) {
    servo_angle =
      -0.628 * angle_cubed + 1.269 * angle_squared - 1.772 * steering_angle;
  } else {
    servo_angle =
      -0.628 * angle_cubed - 1.269 * angle_squared - 1.772 * steering_angle;
  }

  const double servo_pwm = std::max(
    kServoPwmLeftLimit,
    std::min(kServoPwmRightLimit, kServoPwmCenter + servo_angle * kServoPwmPerRadian));

  double turn_ratio = 0.0;
  if (servo_pwm <= kServoPwmCenter) {
    turn_ratio =
      (kServoPwmCenter - servo_pwm) / (kServoPwmCenter - kServoPwmLeftLimit);
  } else {
    turn_ratio =
      (servo_pwm - kServoPwmCenter) / (kServoPwmRightLimit - kServoPwmCenter);
  }
  turn_ratio = std::max(0.0, std::min(1.0, turn_ratio));

  const double reduction_fraction = max_turn_speed_reduction_percent_ / 100.0;
  const double speed_limit =
    std::abs(linear_speed_) * (1.0 - reduction_fraction * turn_ratio);
  const double limited_speed = std::min(std::abs(cmd.linear.x), speed_limit);
  cmd.linear.x = std::copysign(limited_speed, cmd.linear.x);
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

std::vector<VisualLidarPathFollower::ScanPointCluster>
VisualLidarPathFollower::collect_non_static_scan_clusters_map(
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
  std::vector<ScanPointCluster> clusters;
  bool cluster_active = false;
  ScanPointCluster current_cluster;
  Point2D previous_base_point;
  double previous_range = 0.0;

  auto finish_cluster = [&]() {
      if (cluster_active &&
        current_cluster.points.size() >= static_cast<std::size_t>(cluster_min_points_))
      {
        for (const auto & point : current_cluster.points) {
          current_cluster.center.x += point.x;
          current_cluster.center.y += point.y;
        }
        const double point_count = static_cast<double>(current_cluster.points.size());
        current_cluster.center.x /= point_count;
        current_cluster.center.y /= point_count;
        clusters.push_back(std::move(current_cluster));
      }
      current_cluster = ScanPointCluster();
      cluster_active = false;
    };

  double angle = scan.angle_min;
  for (const auto range : scan.ranges) {
    if (!std::isfinite(range) || range < scan.range_min || range > scan.range_max) {
      finish_cluster();
      angle += scan.angle_increment;
      continue;
    }
    const double scan_x = static_cast<double>(range) * std::cos(angle);
    const double scan_y = static_cast<double>(range) * std::sin(angle);
    const Point2D base_point{
      scan_to_base.x + cos_scan * scan_x - sin_scan * scan_y,
      scan_to_base.y + sin_scan * scan_x + cos_scan * scan_y};
    if (enable_static_map_filter_ && is_static_map_point(robot_pose, base_point)) {
      finish_cluster();
      angle += scan.angle_increment;
      continue;
    }

    const double point_range = std::hypot(base_point.x, base_point.y);
    const bool connected = cluster_active &&
      std::abs(point_range - previous_range) <= cluster_max_range_jump_ &&
      std::hypot(
      base_point.x - previous_base_point.x,
      base_point.y - previous_base_point.y) <= cluster_max_point_gap_;
    if (cluster_active && !connected) {
      finish_cluster();
    }
    cluster_active = true;
    current_cluster.points.push_back(base_to_map(robot_pose, base_point));
    previous_base_point = base_point;
    previous_range = point_range;
    angle += scan.angle_increment;
  }
  finish_cluster();
  return clusters;
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

// 扫码瞬间只判断一次 map.x。阈值左侧永久跳过本轮后撤，右侧才把后撤置为待触发。
void VisualLidarPathFollower::evaluate_qr_reverse_gate(
  const VisualLidarPathFollower::RobotPose & robot_pose)
{
  if (qr_reverse_gate_evaluated_) {
    return;
  }

  qr_reverse_gate_evaluated_ = true;
  qr_reverse_eligible_ = robot_pose.x > qr_reverse_min_map_x_;
  qr_escape_state_ = qr_reverse_eligible_ ? QrEscapeState::CHECKING : QrEscapeState::IDLE;
  RCLCPP_INFO(
    get_logger(),
    "二维码后撤 x 门限判定：扫码位姿 map.x=%.3f，门限=%.3f，结果=%s。",
    robot_pose.x, qr_reverse_min_map_x_,
    qr_reverse_eligible_ ? "允许最多一次后撤" : "位于门限左侧，跳过后撤");
}

// 进入 channel 航点邻域后结束视觉覆盖；基础控制从扫码起就始终是航点跟踪。
void VisualLidarPathFollower::update_qr_visual_avoidance_phase(
  const VisualLidarPathFollower::RobotPose & robot_pose)
{
  evaluate_qr_reverse_gate(robot_pose);

  // 扫码时位于门限右侧只代表“保留待触发资格”。倒车真正触发前，车辆一旦
  // 前进到门限左侧，本轮资格永久失效；已经开始的倒车仍只由点云解除条件控制。
  if (qr_reverse_eligible_ && !qr_reverse_triggered_ &&
    qr_escape_state_ == QrEscapeState::CHECKING &&
    robot_pose.x <= qr_reverse_min_map_x_)
  {
    qr_reverse_eligible_ = false;
    qr_escape_state_ = QrEscapeState::IDLE;
    RCLCPP_INFO(
      get_logger(),
      "二维码倒车待触发资格已取消：当前 map.x=%.3f 已进入门限 %.3f 左侧；"
      "本轮后续不再倒车。",
      robot_pose.x, qr_reverse_min_map_x_);
  }

  if (qr_escape_state_ == QrEscapeState::BACKING_RIGHT) {
    return;
  }

  std::size_t nearest_waypoint_index = 0;
  double nearest_waypoint_distance = std::numeric_limits<double>::infinity();
  if (!find_nearest_waypoint(
      robot_pose, nearest_waypoint_index, nearest_waypoint_distance) ||
    nearest_waypoint_distance > qr_visual_handoff_waypoint_distance_)
  {
    return;
  }

  qr_visual_avoidance_active_ = false;
  qr_escape_state_ = QrEscapeState::IDLE;
  current_index_ = std::min(nearest_waypoint_index, waypoints_.size() - 1);
  first_waypoint_reached_ = true;
  set_perception_mode(perception_pause_value_, "暂停");
  RCLCPP_INFO(
    get_logger(),
    "已进入 waypoint_channel：最近航点=%zu/%zu，距离=%.3fm；"
    "关闭视觉障碍物覆盖，继续 waypoint_shift 巡点。",
    current_index_ + 1, waypoints_.size(), nearest_waypoint_distance);
}

// 返回 true 时本周期由一次性二维码倒车覆盖航点指令；解除后下一周期直接恢复追航点。
bool VisualLidarPathFollower::apply_one_shot_qr_reverse(
  const VisualLidarPathFollower::RobotPose & robot_pose,
  geometry_msgs::msg::Twist & cmd)
{
  if (!qr_reverse_eligible_ ||
    (qr_reverse_triggered_ && qr_escape_state_ != QrEscapeState::BACKING_RIGHT))
  {
    return false;
  }

  latest_qr_escape_observation_ = observe_qr_escape_point_cloud(robot_pose);
  const auto & observation = latest_qr_escape_observation_;
  if (!observation.ready) {
    if (qr_escape_state_ == QrEscapeState::BACKING_RIGHT) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "二维码倒车期间缺少有效雷达或地图数据，停车等待数据恢复。");
      return true;
    }
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "二维码后撤待触发，但雷达或地图尚未就绪；继续追 waypoint_channel。");
    return false;
  }

  const int area_count = observation.area_left_count + observation.area_right_count;
  const int rear_count = observation.rear_left_count + observation.rear_right_count;
  if (qr_escape_state_ == QrEscapeState::CHECKING) {
    if (rear_count >= qr_rear_min_points_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "二维码后方警戒区有非墙点：左=%d，右=%d；暂不倒车，继续追航点。",
        observation.rear_left_count, observation.rear_right_count);
      return false;
    }
    if (area_count < qr_escape_min_points_) {
      return false;
    }

    qr_escape_state_ = QrEscapeState::BACKING_RIGHT;
    qr_reverse_triggered_ = true;
    RCLCPP_WARN(
      get_logger(),
      "二维码后撤首次且唯一一次触发：框内左/右=%d/%d，临时覆盖航点指令向右后撤。",
      observation.area_left_count, observation.area_right_count);
  }

  if (rear_count >= qr_rear_min_points_ || area_count < qr_escape_min_points_) {
    qr_escape_state_ = QrEscapeState::IDLE;
    RCLCPP_INFO(
      get_logger(),
      "二维码倒车已解除且锁止，不再二次触发：原因=%s；下一周期直接恢复追 waypoint_channel。",
      rear_count >= qr_rear_min_points_ ? "后方警戒命中" : "前方触发区已清空");
    return true;
  }

  cmd.linear.x = -qr_reverse_linear_speed_;
  cmd.angular.z = qr_reverse_angular_speed_;
  if (debug_control_logs_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 500,
      "debug: 二维码向右后撤，框内左/右=%d/%d，后方左/右=%d/%d，输出=%.2f/%.2f。",
      observation.area_left_count, observation.area_right_count,
      observation.rear_left_count, observation.rear_right_count,
      cmd.linear.x, cmd.angular.z);
  }
  return true;
}

int VisualLidarPathFollower::scale_qr_channel_visual_detection_x(int x) const
{
  return static_cast<int>(std::round(
    static_cast<double>(x) * qr_channel_visual_control_image_width_ /
    qr_channel_visual_detection_source_width_));
}

int VisualLidarPathFollower::scale_qr_channel_visual_detection_y(int y) const
{
  return static_cast<int>(std::round(
    static_cast<double>(y) * qr_channel_visual_control_image_height_ /
    qr_channel_visual_detection_source_height_));
}

// 有有效障碍物时只覆盖当前航点速度；障碍物失效后不保留任何回线/丢线状态。
bool VisualLidarPathFollower::apply_qr_channel_visual_obstacle_avoidance(
  geometry_msgs::msg::Twist & cmd)
{
  if (!has_visual_detection_ || !latest_visual_detection_) {
    return false;
  }
  const double detection_age = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - latest_visual_detection_received_at_).count();
  if (detection_age > qr_channel_visual_detection_timeout_) {
    return false;
  }

  const ai_msgs::msg::Target * obstacle = nullptr;
  int obstacle_center_x = 0;
  int obstacle_bottom_y = 0;
  const int image_center_x = qr_channel_visual_control_image_width_ / 2;
  for (const auto & target : latest_visual_detection_->targets) {
    if (target.type != qr_channel_visual_obstacle_class_name_ || target.rois.empty()) {
      continue;
    }
    const auto & roi = target.rois.front();
    const int center_x = scale_qr_channel_visual_detection_x(static_cast<int>(
      roi.rect.x_offset + roi.rect.width / 2));
    const int bottom_y = scale_qr_channel_visual_detection_y(static_cast<int>(
      roi.rect.y_offset + roi.rect.height));
    const int delta_x = std::abs(center_x - image_center_x);
    const double avoid_threshold =
      qr_channel_visual_obstacle_bottom_threshold_ +
      delta_x * qr_channel_visual_obstacle_delta_x_ratio_;
    if (roi.confidence >= qr_channel_visual_obstacle_confidence_threshold_ &&
      bottom_y >= avoid_threshold)
    {
      obstacle = &target;
      obstacle_center_x = center_x;
      obstacle_bottom_y = bottom_y;
      break;
    }
  }
  if (!obstacle) {
    return false;
  }

  double center_offset = static_cast<double>(obstacle_center_x - image_center_x);
  if (center_offset >= 0.0 && center_offset < 5.0) {
    center_offset = 5.0;
  } else if (center_offset < 0.0 && center_offset > -5.0) {
    center_offset = -5.0;
  }
  const double angular_command = center_offset > 0.0 ?
    -qr_channel_visual_avoid_angular_ratio_ * (image_center_x - center_offset) :
    -qr_channel_visual_avoid_angular_ratio_ * (-image_center_x - center_offset);
  cmd.linear.x = qr_channel_visual_avoid_linear_speed_;
  cmd.angular.z = angular_command;
  RCLCPP_WARN_THROTTLE(
    get_logger(), *get_clock(), 500,
    "视觉障碍物覆盖航点指令：center_x=%d，bottom_y=%d，confidence=%.2f，输出=%.2f/%.2f。",
    obstacle_center_x, obstacle_bottom_y, obstacle->rois.front().confidence,
    cmd.linear.x, cmd.angular.z);
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
  waypoint_shift_pending_offsets_.assign(waypoints_.size(), 0.0);
  waypoint_shift_pending_counts_.assign(waypoints_.size(), 0);
  waypoint_shift_missing_counts_.assign(waypoints_.size(), 0);
  processed_waypoint_shift_scan_sequence_ = scan_sequence_;
}

// waypoint_shift 模式：每帧新雷达提前检查当前目标之后的固定数量航点。
// 以障碍物最近路线点为中心，前后分别取配置数量的点，整段向地图更空旷侧横移。
// 横移量先作为本帧候选，连续确认后才提交；障碍消失或错误观测消失后会释放回原路线。
void VisualLidarPathFollower::update_waypoint_shifts(
  const VisualLidarPathFollower::RobotPose & robot_pose,
  std::size_t target_waypoint_index)
{
  if (!latest_scan_ ||
    processed_waypoint_shift_scan_sequence_ == scan_sequence_ ||
    original_waypoints_.size() != waypoints_.size() ||
    waypoint_shift_offsets_.size() != waypoints_.size() ||
    waypoint_shift_pending_offsets_.size() != waypoints_.size() ||
    waypoint_shift_pending_counts_.size() != waypoints_.size() ||
    waypoint_shift_missing_counts_.size() != waypoints_.size() || waypoints_.empty())
  {
    return;
  }
  processed_waypoint_shift_scan_sequence_ = scan_sequence_;

  if (enable_static_map_filter_ && require_map_for_lidar_assist_ &&
    (!latest_map_ || !latest_restriction_map_))
  {
    return;
  }

  const auto clusters = collect_non_static_scan_clusters_map(robot_pose);

  const std::size_t first_index = std::min(target_waypoint_index, waypoints_.size() - 1);
  const std::size_t last_index = std::min(
    waypoints_.size(), first_index + static_cast<std::size_t>(waypoint_shift_check_count_));
  const std::size_t managed_last_index = std::min(
    waypoints_.size() - 1,
    first_index + static_cast<std::size_t>(waypoint_shift_check_count_) +
    static_cast<std::size_t>(waypoint_shift_points_after_));
  const double check_radius_squared =
    waypoint_cloud_check_radius_ * waypoint_cloud_check_radius_;

  struct ShiftCandidateSegment
  {
    std::size_t first{0};
    std::size_t last{0};
    double left_distance{0.0};
    double right_distance{0.0};
    double left_required_offset{0.0};
    double right_required_offset{0.0};
  };
  std::vector<ShiftCandidateSegment> candidate_segments;

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
    // 左右两个方向都保留所需偏移量。等重叠段统一方向后，再选对应方向的值；
    // 这样不会沿用某个簇原先相反方向计算出来的偏移量。
    auto required_offset_along = [&](const Point2D & shift_normal) {
        double cloud_edge_along_shift = -std::numeric_limits<double>::infinity();
        for (const auto & point : cluster.points) {
          const double lateral_distance =
            (point.x - route_projection.x) * shift_normal.x +
            (point.y - route_projection.y) * shift_normal.y;
          cloud_edge_along_shift = std::max(cloud_edge_along_shift, lateral_distance);
        }
        return std::max(
          0.0, cloud_edge_along_shift + waypoint_shift_target_clearance_);
      };
    const double left_required_offset = required_offset_along(left_normal);
    const double right_required_offset = required_offset_along(right_normal);
    if (left_required_offset <= 1e-9 && right_required_offset <= 1e-9) {
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
    candidate_segments.push_back(ShiftCandidateSegment{
      influence_first, influence_last, left_distance, right_distance,
      left_required_offset, right_required_offset});
  }

  // 每个点云簇先拥有自己的偏移段。只有区间重叠的段才组成冲突组：
  // 冲突组统一左右方向，但各簇仍保留自己的影响区间和偏移量；
  // 最终仅在重叠 waypoint 上取同方向的较大偏移。
  std::sort(
    candidate_segments.begin(), candidate_segments.end(),
    [](const ShiftCandidateSegment & lhs, const ShiftCandidateSegment & rhs) {
      if (lhs.first != rhs.first) {
        return lhs.first < rhs.first;
      }
      return lhs.last < rhs.last;
    });

  std::vector<double> observed_offsets(waypoints_.size(), 0.0);
  std::vector<bool> observed_offsets_valid(waypoints_.size(), false);
  std::size_t group_begin = 0;
  while (group_begin < candidate_segments.size()) {
    std::size_t group_end = group_begin + 1;
    std::size_t group_first = candidate_segments[group_begin].first;
    std::size_t group_last = candidate_segments[group_begin].last;
    while (group_end < candidate_segments.size() &&
      candidate_segments[group_end].first <= group_last)
    {
      group_last = std::max(group_last, candidate_segments[group_end].last);
      ++group_end;
    }

    std::size_t committed_left_count = 0;
    std::size_t committed_right_count = 0;
    double committed_left_magnitude = 0.0;
    double committed_right_magnitude = 0.0;
    long pending_left_score = 0;
    long pending_right_score = 0;
    for (std::size_t index = group_first; index <= group_last; ++index) {
      const double committed = waypoint_shift_offsets_[index];
      if (committed > 1e-9) {
        ++committed_left_count;
        committed_left_magnitude += committed;
      } else if (committed < -1e-9) {
        ++committed_right_count;
        committed_right_magnitude += -committed;
      }

      const double pending = waypoint_shift_pending_offsets_[index];
      if (pending > 1e-9) {
        pending_left_score += waypoint_shift_pending_counts_[index];
      } else if (pending < -1e-9) {
        pending_right_score += waypoint_shift_pending_counts_[index];
      }
    }

    double group_left_distance = std::numeric_limits<double>::infinity();
    double group_right_distance = std::numeric_limits<double>::infinity();
    for (std::size_t segment_index = group_begin;
      segment_index < group_end; ++segment_index)
    {
      group_left_distance = std::min(
        group_left_distance, candidate_segments[segment_index].left_distance);
      group_right_distance = std::min(
        group_right_distance, candidate_segments[segment_index].right_distance);
    }

    int shift_direction = 0;
    const char * direction_source = "空旷侧";
    if (committed_left_count != committed_right_count) {
      shift_direction = committed_left_count > committed_right_count ? 1 : -1;
      direction_source = "已提交段锁定";
    } else if (committed_left_count > 0 &&
      std::abs(committed_left_magnitude - committed_right_magnitude) > 1e-9)
    {
      shift_direction = committed_left_magnitude > committed_right_magnitude ? 1 : -1;
      direction_source = "已提交段锁定";
    } else if (pending_left_score != pending_right_score) {
      shift_direction = pending_left_score > pending_right_score ? 1 : -1;
      direction_source = "待确认段锁定";
    } else {
      shift_direction = group_left_distance > group_right_distance ? 1 : -1;
    }

    std::size_t candidate_count = 0;
    double min_required_offset = std::numeric_limits<double>::infinity();
    double max_required_offset = 0.0;
    for (std::size_t segment_index = group_begin;
      segment_index < group_end; ++segment_index)
    {
      const auto & segment = candidate_segments[segment_index];
      const double required_offset = shift_direction > 0 ?
        segment.left_required_offset : segment.right_required_offset;
      if (required_offset <= 1e-9) {
        continue;
      }
      min_required_offset = std::min(min_required_offset, required_offset);
      max_required_offset = std::max(max_required_offset, required_offset);
      const double candidate = static_cast<double>(shift_direction) * required_offset;
      for (std::size_t index = segment.first; index <= segment.last; ++index) {
        if (!observed_offsets_valid[index]) {
          observed_offsets[index] = candidate;
          observed_offsets_valid[index] = true;
          ++candidate_count;
        } else if (std::abs(candidate) > std::abs(observed_offsets[index]) + 1e-9) {
          observed_offsets[index] = candidate;
        }
      }
    }

    if (debug_control_logs_ && candidate_count > 0) {
      RCLCPP_INFO(
        get_logger(),
        "debug: waypoint_shift 偏移组：簇段%zu个，覆盖点=%zu-%zu，"
        "横截线最小左/右空旷距离=%.3f/%.3fm，向%s（%s），"
        "各段偏移=%.3f-%.3fm，候选%zu个点。",
        group_end - group_begin, group_first + 1, group_last + 1,
        group_left_distance, group_right_distance,
        shift_direction > 0 ? "左" : "右", direction_source,
        min_required_offset, max_required_offset, candidate_count);
    }

    group_begin = group_end;
  }

  bool route_changed = false;

  std::size_t confirmed_count = 0;
  std::size_t released_count = 0;
  for (std::size_t index = first_index; index <= managed_last_index; ++index) {
    if (observed_offsets_valid[index]) {
      const double candidate = observed_offsets[index];
      waypoint_shift_missing_counts_[index] = 0;
      const bool same_direction =
        waypoint_shift_pending_counts_[index] > 0 &&
        waypoint_shift_pending_offsets_[index] * candidate > 0.0;

      if (!same_direction) {
        waypoint_shift_pending_offsets_[index] = candidate;
        waypoint_shift_pending_counts_[index] = 1;
      } else {
        waypoint_shift_pending_offsets_[index] = candidate;
        ++waypoint_shift_pending_counts_[index];
      }

      if (waypoint_shift_pending_counts_[index] >= waypoint_shift_confirm_scans_) {
        const double confirmed_offset = waypoint_shift_pending_offsets_[index];
        if (std::abs(confirmed_offset - waypoint_shift_offsets_[index]) >=
          waypoint_shift_min_change_)
        {
          waypoint_shift_offsets_[index] = confirmed_offset;
          route_changed = true;
          ++confirmed_count;
        }
      }
      continue;
    }

    waypoint_shift_pending_offsets_[index] = 0.0;
    waypoint_shift_pending_counts_[index] = 0;
    ++waypoint_shift_missing_counts_[index];
    if (waypoint_shift_missing_counts_[index] >= waypoint_shift_release_scans_ &&
      std::abs(waypoint_shift_offsets_[index]) > 1e-9)
    {
      waypoint_shift_offsets_[index] = 0.0;
      route_changed = true;
      ++released_count;
    }
  }

  if (confirmed_count > 0 || released_count > 0) {
    RCLCPP_WARN(
      get_logger(),
      "waypoint_shift：确认刷新%zu个点，释放%zu个点（确认帧=%d，释放帧=%d）。",
      confirmed_count, released_count, waypoint_shift_confirm_scans_,
      waypoint_shift_release_scans_);
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
//   2. 从二维码前轨迹切换到该方向独立的 channel 航点文件。
void VisualLidarPathFollower::qr_callback(const std_msgs::msg::String::ConstSharedPtr msg)
{
  if (qr_route_selected_) {
    return;
  }

  const std::string direction = direction_from_qr(msg->data);
  if (!direction.empty()) {
    if (switch_route(direction)) {
      qr_route_selected_ = true;
      RobotPose qr_pose;
      if (lookup_robot_pose(qr_pose.x, qr_pose.y, qr_pose.yaw)) {
        evaluate_qr_reverse_gate(qr_pose);
      } else {
        RCLCPP_WARN(
          get_logger(), "扫码回调暂未取得 map 位姿，将在第一帧有效定位时完成 x 门限判定。");
      }
      // visual_control 从此不再参与巡线/回线；YOLO 独立保持仅障碍物输出。
      suspend_visual_control();
      set_perception_mode(perception_obstacle_only_value_, "仅障碍物识别");
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
  waypoints_ = new_waypoints;
  selected_route_direction_ = resolved_direction.empty() ? direction : resolved_direction;
  insert_image_capture_waypoint(selected_route_direction_);
  reset_waypoint_shift_state();
  waiting_for_qr_start_ = false;
  following_qr_path_ = false;
  qr_path_finished_ = false;
  image_analysis_triggered_ = false;
  if (was_following_qr_path || was_waiting_for_qr) {
    current_index_ = 0;
    first_waypoint_reached_ = false;
    qr_visual_avoidance_active_ = true;
    qr_reverse_gate_evaluated_ = false;
    qr_reverse_eligible_ = false;
    qr_reverse_triggered_ = false;
    qr_escape_state_ = QrEscapeState::IDLE;
    latest_qr_escape_observation_ = QrEscapeObservation();
  } else {
    current_index_ = std::min(current_index_, waypoints_.size() - 1);
    first_waypoint_reached_ = current_index_ > 0;
  }
  const auto route_name = route_direction_name(selected_route_direction_);
  RCLCPP_INFO(
    get_logger(), "QR 选择%s路线，立即追 waypoint_channel；视觉仅临时覆盖有效障碍物。",
    route_name.c_str());
  if (debug_control_logs_) {
    RCLCPP_INFO(
      get_logger(), "debug: 环线文件=%s，起点=%zu/%zu（%s）。",
      route_waypoints_file.c_str(), current_index_ + 1, waypoints_.size(),
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

void VisualLidarPathFollower::set_perception_mode(int value, const char * mode_name)
{
  if (!visual_control_suspend_pub_) {
    return;
  }

  std_msgs::msg::Int32 msg;
  msg.data = value;
  for (int i = 0; i < 3; ++i) {
    visual_control_suspend_pub_->publish(msg);
  }
  RCLCPP_INFO(
    get_logger(), "YOLO 模式切换为%s：topic=%s, value=%d。",
    mode_name, visual_control_suspend_topic_.c_str(), value);
}

void VisualLidarPathFollower::resume_visual_control(bool restore_full_perception)
{
  if (!visual_control_suspend_pub_) {
    return;
  }

  std_msgs::msg::Int32 msg;
  if (restore_full_perception && perception_full_resume_value_ != visual_control_resume_value_) {
    msg.data = perception_full_resume_value_;
    for (int i = 0; i < 3; ++i) {
      visual_control_suspend_pub_->publish(msg);
    }
  }
  msg.data = visual_control_resume_value_;
  for (int i = 0; i < 3; ++i) {
    visual_control_suspend_pub_->publish(msg);
  }
  RCLCPP_INFO(
    get_logger(), "已通知 visual_control 恢复%s。",
    restore_full_perception ? "，YOLO 同时恢复完整识别" : "");
  if (debug_control_logs_) {
    RCLCPP_INFO(
      get_logger(), "debug: resume_topic=%s, visual_value=%d, perception_full_value=%d。",
      visual_control_suspend_topic_.c_str(), visual_control_resume_value_,
      perception_full_resume_value_);
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
// channel 的顺、逆时针必须各自提供对应 routes.<direction>，不再反向读取另一方向；
// waypoints 仍作为二维码前旧格式的兜底。
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
    if (routes && routes[direction]) {
      items = routes[direction];
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
