#include "origincar_system/visual_lidar_path_follower.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <ctime>
#include <exception>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "rcutils/error_handling.h"
#include "rcutils/logging.h"
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

double normalize_angle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

struct CircleIntersectionChord
{
  double first_x{0.0};
  double first_y{0.0};
  double second_x{0.0};
  double second_y{0.0};
};

// 计算两个真实规划圆的公共弦端点。半径由调用方直接传入，因此
// circle_envelope模式会自然使用target_clearance + numeric_margin。
// 相离、相切、同心或一圆包含另一圆时没有两个稳定交点，返回false。
bool circle_intersection_chord(
  double first_x, double first_y, double first_radius,
  double second_x, double second_y, double second_radius,
  CircleIntersectionChord & chord)
{
  const double center_x = second_x - first_x;
  const double center_y = second_y - first_y;
  const double center_distance = std::hypot(center_x, center_y);
  if (first_radius <= 1.0e-9 || second_radius <= 1.0e-9 ||
    center_distance <= 1.0e-9 ||
    center_distance + 1.0e-9 >= first_radius + second_radius ||
    center_distance <= std::abs(first_radius - second_radius) + 1.0e-9)
  {
    return false;
  }

  const double along_centers =
    (first_radius * first_radius - second_radius * second_radius +
    center_distance * center_distance) / (2.0 * center_distance);
  const double half_chord_squared =
    first_radius * first_radius - along_centers * along_centers;
  if (half_chord_squared <= 1.0e-12) {
    return false;
  }
  const double half_chord = std::sqrt(half_chord_squared);
  const double unit_x = center_x / center_distance;
  const double unit_y = center_y / center_distance;
  const double chord_center_x = first_x + along_centers * unit_x;
  const double chord_center_y = first_y + along_centers * unit_y;
  const double perpendicular_x = -unit_y;
  const double perpendicular_y = unit_x;
  chord.first_x = chord_center_x + half_chord * perpendicular_x;
  chord.first_y = chord_center_y + half_chord * perpendicular_y;
  chord.second_x = chord_center_x - half_chord * perpendicular_x;
  chord.second_y = chord_center_y - half_chord * perpendicular_y;
  return true;
}

bool normal_line_intersects_chord(
  double base_x, double base_y, double normal_x, double normal_y,
  const CircleIntersectionChord & chord, double & normal_offset,
  double & chord_ratio)
{
  const double chord_x = chord.second_x - chord.first_x;
  const double chord_y = chord.second_y - chord.first_y;
  const double chord_length_squared = chord_x * chord_x + chord_y * chord_y;
  const double denominator = normal_x * chord_y - normal_y * chord_x;
  if (chord_length_squared <= 1.0e-12 || std::abs(denominator) <= 1.0e-9) {
    return false;
  }
  normal_offset =
    ((chord.first_x - base_x) * chord_y -
    (chord.first_y - base_y) * chord_x) / denominator;
  const double intersection_x = base_x + normal_offset * normal_x;
  const double intersection_y = base_y + normal_offset * normal_y;
  chord_ratio =
    ((intersection_x - chord.first_x) * chord_x +
    (intersection_y - chord.first_y) * chord_y) / chord_length_squared;
  return std::isfinite(normal_offset) && std::isfinite(chord_ratio);
}

bool point_near_chord_segment(
  double point_x, double point_y, const CircleIntersectionChord & chord,
  double tolerance)
{
  const double chord_x = chord.second_x - chord.first_x;
  const double chord_y = chord.second_y - chord.first_y;
  const double chord_length_squared = chord_x * chord_x + chord_y * chord_y;
  if (chord_length_squared <= 1.0e-12) {
    return false;
  }
  const double chord_length = std::sqrt(chord_length_squared);
  const double ratio =
    ((point_x - chord.first_x) * chord_x +
    (point_y - chord.first_y) * chord_y) / chord_length_squared;
  const double ratio_tolerance = std::max(0.0, tolerance) / chord_length;
  if (ratio < -ratio_tolerance || ratio > 1.0 + ratio_tolerance) {
    return false;
  }
  const double clamped_ratio = std::clamp(ratio, 0.0, 1.0);
  const double closest_x = chord.first_x + clamped_ratio * chord_x;
  const double closest_y = chord.first_y + clamped_ratio * chord_y;
  return std::hypot(point_x - closest_x, point_y - closest_y) <= tolerance;
}

std::string json_escape(const std::string & text)
{
  std::ostringstream escaped;
  for (const char ch : text) {
    switch (ch) {
      case '\\': escaped << "\\\\"; break;
      case '"': escaped << "\\\""; break;
      case '\n': escaped << "\\n"; break;
      case '\r': escaped << "\\r"; break;
      case '\t': escaped << "\\t"; break;
      default: escaped << ch; break;
    }
  }
  return escaped.str();
}

}  // namespace

// 构造函数完成参数声明、参数读取、航点加载、话题订阅/发布和控制定时器创建。
VisualLidarPathFollower::VisualLidarPathFollower()
: rclcpp::Node("visual_lidar_path_follower"),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  // 坐标系与话题
  declare_parameter<std::string>("task1_waypoints_file", task1_waypoints_file_);
  get_parameter("task1_waypoints_file", task1_waypoints_file_);
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
  declare_parameter<std::string>(
    "logic3_static_restriction_map_topic",
    logic3_static_restriction_map_topic_);
  get_parameter(
    "logic3_static_restriction_map_topic",
    logic3_static_restriction_map_topic_);
  declare_parameter<std::string>(
    "logic3_runtime_restriction_map_topic",
    logic3_runtime_restriction_map_topic_);
  get_parameter(
    "logic3_runtime_restriction_map_topic",
    logic3_runtime_restriction_map_topic_);
  declare_parameter<std::string>(
    "logic3_match_score_topic", logic3_match_score_topic_);
  get_parameter("logic3_match_score_topic", logic3_match_score_topic_);
  declare_parameter<std::string>(
    "logic3_match_diagnostics_topic", logic3_match_diagnostics_topic_);
  get_parameter(
    "logic3_match_diagnostics_topic", logic3_match_diagnostics_topic_);
  declare_parameter<bool>("debug_control_logs", debug_control_logs_);
  get_parameter("debug_control_logs", debug_control_logs_);
  declare_parameter<bool>(
    "quiet_logs_after_static_freeze", quiet_logs_after_static_freeze_);
  get_parameter(
    "quiet_logs_after_static_freeze", quiet_logs_after_static_freeze_);
  declare_parameter<bool>("enable_debug_visualization", enable_debug_visualization_);
  get_parameter("enable_debug_visualization", enable_debug_visualization_);

  // 全阶段公共控制
  declare_parameter<double>("control_frequency", control_frequency_);
  get_parameter("control_frequency", control_frequency_);
  declare_parameter<bool>(
    "enable_start_acceleration_limit", enable_start_acceleration_limit_);
  get_parameter(
    "enable_start_acceleration_limit", enable_start_acceleration_limit_);
  declare_parameter<double>("start_speed", start_speed_);
  get_parameter("start_speed", start_speed_);
  declare_parameter<double>("start_speed_threshold", start_speed_threshold_);
  get_parameter("start_speed_threshold", start_speed_threshold_);
  declare_parameter<double>("global_max_accel", global_max_accel_);
  get_parameter("global_max_accel", global_max_accel_);
  declare_parameter<double>("global_max_decel", global_max_decel_);
  get_parameter("global_max_decel", global_max_decel_);
  declare_parameter<double>(
    "trusted_departure_start_speed", trusted_departure_start_speed_);
  get_parameter(
    "trusted_departure_start_speed", trusted_departure_start_speed_);
  declare_parameter<double>(
    "trusted_departure_start_max_accel", trusted_departure_start_max_accel_);
  get_parameter(
    "trusted_departure_start_max_accel", trusted_departure_start_max_accel_);
  declare_parameter<double>(
    "image_capture_start_speed", image_capture_start_speed_);
  get_parameter("image_capture_start_speed", image_capture_start_speed_);
  declare_parameter<double>(
    "image_capture_start_max_accel", image_capture_start_max_accel_);
  get_parameter(
    "image_capture_start_max_accel", image_capture_start_max_accel_);
  declare_parameter<double>(
    "post_reverse_start_speed", post_reverse_start_speed_);
  get_parameter("post_reverse_start_speed", post_reverse_start_speed_);
  declare_parameter<double>(
    "post_reverse_start_max_accel", post_reverse_start_max_accel_);
  get_parameter(
    "post_reverse_start_max_accel", post_reverse_start_max_accel_);
  declare_parameter<double>("lookahead_base", lookahead_base_);
  get_parameter("lookahead_base", lookahead_base_);
  declare_parameter<double>("lookahead_speed_gain", lookahead_speed_gain_);
  get_parameter("lookahead_speed_gain", lookahead_speed_gain_);
  declare_parameter<double>("lookahead_max", lookahead_max_);
  get_parameter("lookahead_max", lookahead_max_);
  declare_parameter<double>(
    "route_projection_search_backward_distance",
    route_projection_search_backward_distance_);
  get_parameter(
    "route_projection_search_backward_distance",
    route_projection_search_backward_distance_);
  declare_parameter<double>(
    "route_projection_search_forward_distance",
    route_projection_search_forward_distance_);
  get_parameter(
    "route_projection_search_forward_distance",
    route_projection_search_forward_distance_);
  declare_parameter<double>(
    "turn_speed_full_steering_angle", turn_speed_full_steering_angle_);
  get_parameter(
    "turn_speed_full_steering_angle", turn_speed_full_steering_angle_);
  declare_parameter<std::string>(
    "tracking_controller_mode", tracking_controller_mode_);
  get_parameter("tracking_controller_mode", tracking_controller_mode_);
  declare_parameter<double>("path_pid_yaw_scale_left", path_pid_yaw_scale_left_);
  get_parameter("path_pid_yaw_scale_left", path_pid_yaw_scale_left_);
  declare_parameter<double>("path_pid_yaw_scale_right", path_pid_yaw_scale_right_);
  get_parameter("path_pid_yaw_scale_right", path_pid_yaw_scale_right_);
  declare_parameter<double>("path_pid_heading_kp", path_pid_heading_kp_);
  get_parameter("path_pid_heading_kp", path_pid_heading_kp_);
  declare_parameter<double>("path_pid_lateral_kp", path_pid_lateral_kp_);
  get_parameter("path_pid_lateral_kp", path_pid_lateral_kp_);
  declare_parameter<double>("path_pid_lateral_ki", path_pid_lateral_ki_);
  get_parameter("path_pid_lateral_ki", path_pid_lateral_ki_);
  declare_parameter<double>("path_pid_lateral_kd", path_pid_lateral_kd_);
  get_parameter("path_pid_lateral_kd", path_pid_lateral_kd_);
  declare_parameter<double>("path_pid_integral_limit", path_pid_integral_limit_);
  get_parameter("path_pid_integral_limit", path_pid_integral_limit_);
  declare_parameter<double>(
    "path_pid_derivative_filter_alpha", path_pid_derivative_filter_alpha_);
  get_parameter(
    "path_pid_derivative_filter_alpha", path_pid_derivative_filter_alpha_);
  declare_parameter<double>("path_pid_preview_base", path_pid_preview_base_);
  get_parameter("path_pid_preview_base", path_pid_preview_base_);
  declare_parameter<double>("path_pid_preview_time", path_pid_preview_time_);
  get_parameter("path_pid_preview_time", path_pid_preview_time_);
  declare_parameter<double>("path_pid_preview_max", path_pid_preview_max_);
  get_parameter("path_pid_preview_max", path_pid_preview_max_);
  declare_parameter<double>(
    "path_pid_curvature_window_length", path_pid_curvature_window_length_);
  get_parameter(
    "path_pid_curvature_window_length", path_pid_curvature_window_length_);
  declare_parameter<double>("path_pid_max_curvature", path_pid_max_curvature_);
  get_parameter("path_pid_max_curvature", path_pid_max_curvature_);
  declare_parameter<double>(
    "path_pid_absolute_angular_limit", path_pid_absolute_angular_limit_);
  get_parameter("path_pid_absolute_angular_limit", path_pid_absolute_angular_limit_);
  declare_parameter<double>(
    "path_pid_max_angular_rate", path_pid_max_angular_rate_);
  get_parameter("path_pid_max_angular_rate", path_pid_max_angular_rate_);

  // 倒车控制
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
  declare_parameter<int>(
    "waypoint_shift_occupied_threshold", waypoint_shift_occupied_threshold_);
  get_parameter("waypoint_shift_occupied_threshold", waypoint_shift_occupied_threshold_);
  declare_parameter<int>("waypoint_shift_check_count", waypoint_shift_check_count_);
  get_parameter("waypoint_shift_check_count", waypoint_shift_check_count_);
  declare_parameter<double>("waypoint_cloud_check_radius", waypoint_cloud_check_radius_);
  get_parameter("waypoint_cloud_check_radius", waypoint_cloud_check_radius_);
  declare_parameter<double>(
    "task1_waypoint_shift_target_clearance", task1_waypoint_shift_target_clearance_);
  get_parameter(
    "task1_waypoint_shift_target_clearance", task1_waypoint_shift_target_clearance_);
  declare_parameter<double>(
    "transition_waypoint_shift_target_clearance",
    transition_waypoint_shift_target_clearance_);
  get_parameter(
    "transition_waypoint_shift_target_clearance",
    transition_waypoint_shift_target_clearance_);
  declare_parameter<bool>(
    "transition_dynamic_avoid_only", transition_dynamic_avoid_only_);
  get_parameter(
    "transition_dynamic_avoid_only", transition_dynamic_avoid_only_);
  declare_parameter<double>(
    "task2_waypoint_shift_target_clearance", task2_waypoint_shift_target_clearance_);
  get_parameter(
    "task2_waypoint_shift_target_clearance", task2_waypoint_shift_target_clearance_);
  declare_parameter<double>(
    "task3_waypoint_shift_target_clearance", task3_waypoint_shift_target_clearance_);
  get_parameter(
    "task3_waypoint_shift_target_clearance", task3_waypoint_shift_target_clearance_);
  declare_parameter<double>(
    "task1_waypoint_shift_min_y", task1_waypoint_shift_min_y_);
  get_parameter("task1_waypoint_shift_min_y", task1_waypoint_shift_min_y_);
  declare_parameter<std::string>(
    "route_deformation_mode", route_deformation_mode_);
  get_parameter("route_deformation_mode", route_deformation_mode_);
  declare_parameter<double>(
    "circle_envelope_target_clearance", circle_envelope_target_clearance_);
  get_parameter(
    "circle_envelope_target_clearance", circle_envelope_target_clearance_);
  declare_parameter<double>(
    "circle_envelope_numeric_margin", circle_envelope_numeric_margin_);
  get_parameter("circle_envelope_numeric_margin", circle_envelope_numeric_margin_);
  declare_parameter<double>(
    "circle_envelope_min_transition_length",
    circle_envelope_min_transition_length_);
  get_parameter(
    "circle_envelope_min_transition_length",
    circle_envelope_min_transition_length_);
  declare_parameter<double>(
    "circle_envelope_max_transition_length",
    circle_envelope_max_transition_length_);
  get_parameter(
    "circle_envelope_max_transition_length",
    circle_envelope_max_transition_length_);
  declare_parameter<double>(
    "circle_envelope_extension_step", circle_envelope_extension_step_);
  get_parameter("circle_envelope_extension_step", circle_envelope_extension_step_);
  declare_parameter<double>(
    "circle_envelope_merge_gap", circle_envelope_merge_gap_);
  get_parameter("circle_envelope_merge_gap", circle_envelope_merge_gap_);
  declare_parameter<double>(
    "circle_envelope_max_curvature", circle_envelope_max_curvature_);
  get_parameter("circle_envelope_max_curvature", circle_envelope_max_curvature_);
  declare_parameter<int>(
    "circle_envelope_smoothing_iterations",
    circle_envelope_smoothing_iterations_);
  get_parameter(
    "circle_envelope_smoothing_iterations",
    circle_envelope_smoothing_iterations_);
  declare_parameter<bool>(
    "waypoint_shift_enable_local_rebuild", waypoint_shift_enable_local_rebuild_);
  get_parameter(
    "waypoint_shift_enable_local_rebuild", waypoint_shift_enable_local_rebuild_);
  declare_parameter<double>(
    "waypoint_shift_rebuild_min_progress_ratio",
    waypoint_shift_rebuild_min_progress_ratio_);
  get_parameter(
    "waypoint_shift_rebuild_min_progress_ratio",
    waypoint_shift_rebuild_min_progress_ratio_);
  declare_parameter<double>(
    "waypoint_shift_rebuild_plateau_half_length",
    waypoint_shift_rebuild_plateau_half_length_);
  get_parameter(
    "waypoint_shift_rebuild_plateau_half_length",
    waypoint_shift_rebuild_plateau_half_length_);
  declare_parameter<double>(
    "waypoint_shift_rebuild_bezier_handle_ratio",
    waypoint_shift_rebuild_bezier_handle_ratio_);
  get_parameter(
    "waypoint_shift_rebuild_bezier_handle_ratio",
    waypoint_shift_rebuild_bezier_handle_ratio_);
  declare_parameter<int>(
    "task1_waypoint_shift_points_before",
    task1_waypoint_shift_points_before_);
  get_parameter(
    "task1_waypoint_shift_points_before",
    task1_waypoint_shift_points_before_);
  declare_parameter<int>(
    "task1_waypoint_shift_points_after",
    task1_waypoint_shift_points_after_);
  get_parameter(
    "task1_waypoint_shift_points_after",
    task1_waypoint_shift_points_after_);
  declare_parameter<int>(
    "transition_waypoint_shift_points_before",
    transition_waypoint_shift_points_before_);
  get_parameter(
    "transition_waypoint_shift_points_before",
    transition_waypoint_shift_points_before_);
  declare_parameter<int>(
    "transition_waypoint_shift_points_after",
    transition_waypoint_shift_points_after_);
  get_parameter(
    "transition_waypoint_shift_points_after",
    transition_waypoint_shift_points_after_);
  declare_parameter<int>(
    "task23_waypoint_shift_points_before",
    task23_waypoint_shift_points_before_);
  get_parameter(
    "task23_waypoint_shift_points_before",
    task23_waypoint_shift_points_before_);
  declare_parameter<int>(
    "task23_waypoint_shift_points_after",
    task23_waypoint_shift_points_after_);
  get_parameter(
    "task23_waypoint_shift_points_after",
    task23_waypoint_shift_points_after_);
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

  // 静态+动态方案参数。
  declare_parameter<std::string>(
    "logic3_static_route_direction", logic3_static_route_direction_);
  get_parameter("logic3_static_route_direction", logic3_static_route_direction_);
  declare_parameter<std::string>("logic3_odom_topic", logic3_odom_topic_);
  get_parameter("logic3_odom_topic", logic3_odom_topic_);
  declare_parameter<std::string>("logic3_odom_frame_id", logic3_odom_frame_id_);
  get_parameter("logic3_odom_frame_id", logic3_odom_frame_id_);
  declare_parameter<std::string>(
    "logic3_planning_pose_source", logic3_planning_pose_source_);
  get_parameter("logic3_planning_pose_source", logic3_planning_pose_source_);
  declare_parameter<std::string>("logic3_output_root", logic3_output_root_);
  get_parameter("logic3_output_root", logic3_output_root_);
  declare_parameter<std::string>(
    "logic3_restriction_map_yaml", logic3_restriction_map_yaml_);
  get_parameter("logic3_restriction_map_yaml", logic3_restriction_map_yaml_);
  declare_parameter<bool>(
    "logic3_actual_route_enabled", logic3_actual_route_enabled_);
  get_parameter("logic3_actual_route_enabled", logic3_actual_route_enabled_);
  declare_parameter<double>(
    "logic3_actual_route_min_distance", logic3_actual_route_min_distance_);
  get_parameter(
    "logic3_actual_route_min_distance", logic3_actual_route_min_distance_);
  declare_parameter<int>(
    "logic3_actual_route_line_width", logic3_actual_route_line_width_);
  get_parameter("logic3_actual_route_line_width", logic3_actual_route_line_width_);
  declare_parameter<std::string>(
    "logic3_actual_route_color", logic3_actual_route_color_);
  get_parameter("logic3_actual_route_color", logic3_actual_route_color_);
  declare_parameter<double>(
    "logic3_motion_speed_threshold", logic3_motion_speed_threshold_);
  get_parameter("logic3_motion_speed_threshold", logic3_motion_speed_threshold_);
  declare_parameter<double>(
    "logic3_motion_displacement_threshold", logic3_motion_displacement_threshold_);
  get_parameter(
    "logic3_motion_displacement_threshold", logic3_motion_displacement_threshold_);
  declare_parameter<int>(
    "logic3_motion_confirm_messages", logic3_motion_confirm_messages_);
  get_parameter("logic3_motion_confirm_messages", logic3_motion_confirm_messages_);
  declare_parameter<bool>(
    "logic3_ready_preplan_enabled", logic3_ready_preplan_enabled_);
  get_parameter("logic3_ready_preplan_enabled", logic3_ready_preplan_enabled_);
  declare_parameter<double>(
    "logic3_ready_preplan_min_center_change",
    logic3_ready_preplan_min_center_change_);
  get_parameter(
    "logic3_ready_preplan_min_center_change",
    logic3_ready_preplan_min_center_change_);
  declare_parameter<double>(
    "logic3_ready_preplan_min_interval", logic3_ready_preplan_min_interval_);
  get_parameter(
    "logic3_ready_preplan_min_interval", logic3_ready_preplan_min_interval_);
  declare_parameter<bool>(
    "logic3_start_zone_guard_enabled", logic3_start_zone_guard_enabled_);
  get_parameter("logic3_start_zone_guard_enabled", logic3_start_zone_guard_enabled_);
  declare_parameter<double>("logic3_start_zone_min_x", logic3_start_zone_min_x_);
  get_parameter("logic3_start_zone_min_x", logic3_start_zone_min_x_);
  declare_parameter<double>("logic3_start_zone_max_x", logic3_start_zone_max_x_);
  get_parameter("logic3_start_zone_max_x", logic3_start_zone_max_x_);
  declare_parameter<double>("logic3_start_zone_min_y", logic3_start_zone_min_y_);
  get_parameter("logic3_start_zone_min_y", logic3_start_zone_min_y_);
  declare_parameter<double>("logic3_start_zone_max_y", logic3_start_zone_max_y_);
  get_parameter("logic3_start_zone_max_y", logic3_start_zone_max_y_);
  declare_parameter<double>(
    "logic3_start_zone_guard_margin", logic3_start_zone_guard_margin_);
  get_parameter("logic3_start_zone_guard_margin", logic3_start_zone_guard_margin_);
  declare_parameter<double>("logic3_old_cluster_radius", logic3_old_cluster_radius_);
  get_parameter("logic3_old_cluster_radius", logic3_old_cluster_radius_);
  declare_parameter<double>(
    "logic3_green_red_filter_radius", logic3_green_red_filter_radius_);
  get_parameter(
    "logic3_green_red_filter_radius", logic3_green_red_filter_radius_);
  declare_parameter<double>("logic3_runtime_roi_width", logic3_runtime_roi_width_);
  get_parameter("logic3_runtime_roi_width", logic3_runtime_roi_width_);
  declare_parameter<double>("logic3_runtime_roi_height", logic3_runtime_roi_height_);
  get_parameter("logic3_runtime_roi_height", logic3_runtime_roi_height_);
  declare_parameter<double>(
    "logic3_runtime_channel_min_y", logic3_runtime_channel_min_y_);
  get_parameter("logic3_runtime_channel_min_y", logic3_runtime_channel_min_y_);
  declare_parameter<bool>(
    "logic3_runtime_green_update_enabled",
    logic3_runtime_green_update_enabled_);
  get_parameter(
    "logic3_runtime_green_update_enabled",
    logic3_runtime_green_update_enabled_);
  declare_parameter<bool>(
    "logic3_runtime_orange_only_update_enabled",
    logic3_runtime_orange_only_update_enabled_);
  get_parameter(
    "logic3_runtime_orange_only_update_enabled",
    logic3_runtime_orange_only_update_enabled_);
  declare_parameter<double>(
    "logic3_runtime_green_robot_exclusion_radius",
    logic3_runtime_green_robot_exclusion_radius_);
  get_parameter(
    "logic3_runtime_green_robot_exclusion_radius",
    logic3_runtime_green_robot_exclusion_radius_);
  declare_parameter<double>(
    "logic3_runtime_green_min_match_score",
    logic3_runtime_green_min_match_score_);
  get_parameter(
    "logic3_runtime_green_min_match_score",
    logic3_runtime_green_min_match_score_);
  declare_parameter<bool>(
    "logic3_runtime_green_motion_gate_enabled",
    logic3_runtime_green_motion_gate_enabled_);
  get_parameter(
    "logic3_runtime_green_motion_gate_enabled",
    logic3_runtime_green_motion_gate_enabled_);
  declare_parameter<double>(
    "logic3_runtime_green_max_steering_span",
    logic3_runtime_green_max_steering_span_);
  get_parameter(
    "logic3_runtime_green_max_steering_span",
    logic3_runtime_green_max_steering_span_);
  declare_parameter<double>(
    "logic3_runtime_green_min_speed",
    logic3_runtime_green_min_speed_);
  get_parameter(
    "logic3_runtime_green_min_speed",
    logic3_runtime_green_min_speed_);
  declare_parameter<double>(
    "logic3_runtime_green_max_speed_step",
    logic3_runtime_green_max_speed_step_);
  get_parameter(
    "logic3_runtime_green_max_speed_step",
    logic3_runtime_green_max_speed_step_);
  declare_parameter<double>(
    "logic3_runtime_green_max_map_odom_translation_correction",
    logic3_runtime_green_max_map_odom_translation_correction_);
  get_parameter(
    "logic3_runtime_green_max_map_odom_translation_correction",
    logic3_runtime_green_max_map_odom_translation_correction_);
  declare_parameter<double>(
    "logic3_runtime_green_max_map_odom_yaw_correction_deg",
    logic3_runtime_green_max_map_odom_yaw_correction_deg_);
  get_parameter(
    "logic3_runtime_green_max_map_odom_yaw_correction_deg",
    logic3_runtime_green_max_map_odom_yaw_correction_deg_);
  declare_parameter<double>(
    "logic3_runtime_green_gate_log_rate_hz",
    logic3_runtime_green_gate_log_rate_hz_);
  get_parameter(
    "logic3_runtime_green_gate_log_rate_hz",
    logic3_runtime_green_gate_log_rate_hz_);
  declare_parameter<std::vector<std::int64_t>>(
    "logic3_runtime_green_confirm_scans_by_point_count",
    logic3_runtime_green_confirm_scans_by_point_count_);
  get_parameter(
    "logic3_runtime_green_confirm_scans_by_point_count",
    logic3_runtime_green_confirm_scans_by_point_count_);
  declare_parameter<double>(
    "logic3_runtime_green_match_radius",
    logic3_runtime_green_match_radius_);
  get_parameter(
    "logic3_runtime_green_match_radius",
    logic3_runtime_green_match_radius_);
  declare_parameter<double>(
    "logic3_runtime_green_anchor_update_radius",
    logic3_runtime_green_anchor_update_radius_);
  get_parameter(
    "logic3_runtime_green_anchor_update_radius",
    logic3_runtime_green_anchor_update_radius_);
  declare_parameter<bool>(
    "logic3_runtime_green_allow_route_crossing",
    logic3_runtime_green_allow_route_crossing_);
  get_parameter(
    "logic3_runtime_green_allow_route_crossing",
    logic3_runtime_green_allow_route_crossing_);
  declare_parameter<int>(
    "logic3_runtime_green_average_samples",
    logic3_runtime_green_average_samples_);
  get_parameter(
    "logic3_runtime_green_average_samples",
    logic3_runtime_green_average_samples_);
  declare_parameter<double>(
    "logic3_runtime_green_update_min_distance",
    logic3_runtime_green_update_min_distance_);
  get_parameter(
    "logic3_runtime_green_update_min_distance",
    logic3_runtime_green_update_min_distance_);
  declare_parameter<double>(
    "logic3_runtime_green_replan_min_distance",
    logic3_runtime_green_replan_min_distance_);
  get_parameter(
    "logic3_runtime_green_replan_min_distance",
    logic3_runtime_green_replan_min_distance_);
  declare_parameter<double>(
    "logic3_runtime_green_replan_min_interval",
    logic3_runtime_green_replan_min_interval_);
  get_parameter(
    "logic3_runtime_green_replan_min_interval",
    logic3_runtime_green_replan_min_interval_);
  declare_parameter<int>(
    "logic3_runtime_green_candidate_release_scans",
    logic3_runtime_green_candidate_release_scans_);
  get_parameter(
    "logic3_runtime_green_candidate_release_scans",
    logic3_runtime_green_candidate_release_scans_);
  declare_parameter<int>(
    "logic3_static_cluster_min_points", logic3_static_cluster_min_points_);
  get_parameter("logic3_static_cluster_min_points", logic3_static_cluster_min_points_);
  declare_parameter<int>(
    "logic3_static_restriction_inflation_cells",
    logic3_static_restriction_inflation_cells_);
  get_parameter(
    "logic3_static_restriction_inflation_cells",
    logic3_static_restriction_inflation_cells_);
  declare_parameter<double>(
    "logic3_static_channel_max_x", logic3_static_channel_max_x_);
  get_parameter("logic3_static_channel_max_x", logic3_static_channel_max_x_);
  declare_parameter<double>(
    "logic3_static_channel_min_y", logic3_static_channel_min_y_);
  get_parameter("logic3_static_channel_min_y", logic3_static_channel_min_y_);
  declare_parameter<double>("task1_static_min_x", task1_static_min_x_);
  get_parameter("task1_static_min_x", task1_static_min_x_);
  declare_parameter<double>("task1_static_min_y", task1_static_min_y_);
  get_parameter("task1_static_min_y", task1_static_min_y_);
  declare_parameter<double>(
    "task1_static_avoid_min_x", task1_static_avoid_min_x_);
  get_parameter("task1_static_avoid_min_x", task1_static_avoid_min_x_);
  declare_parameter<double>(
    "task1_static_avoid_max_x", task1_static_avoid_max_x_);
  get_parameter("task1_static_avoid_max_x", task1_static_avoid_max_x_);
  declare_parameter<double>(
    "task1_static_avoid_min_y", task1_static_avoid_min_y_);
  get_parameter("task1_static_avoid_min_y", task1_static_avoid_min_y_);
  declare_parameter<double>(
    "task1_static_avoid_max_y", task1_static_avoid_max_y_);
  get_parameter("task1_static_avoid_max_y", task1_static_avoid_max_y_);
  declare_parameter<double>(
    "logic3_static_cluster_match_radius", logic3_static_cluster_match_radius_);
  get_parameter("logic3_static_cluster_match_radius", logic3_static_cluster_match_radius_);
  declare_parameter<double>(
    "logic3_static_same_frame_merge_radius",
    logic3_static_same_frame_merge_radius_);
  get_parameter(
    "logic3_static_same_frame_merge_radius",
    logic3_static_same_frame_merge_radius_);
  declare_parameter<double>(
    "logic3_static_confirmation_window_seconds",
    logic3_static_confirmation_window_seconds_);
  get_parameter(
    "logic3_static_confirmation_window_seconds",
    logic3_static_confirmation_window_seconds_);
  declare_parameter<int>(
    "logic3_static_confirmation_min_hits",
    logic3_static_confirmation_min_hits_);
  get_parameter(
    "logic3_static_confirmation_min_hits",
    logic3_static_confirmation_min_hits_);
  declare_parameter<double>(
    "logic3_static_collection_stable_seconds",
    logic3_static_collection_stable_seconds_);
  get_parameter(
    "logic3_static_collection_stable_seconds",
    logic3_static_collection_stable_seconds_);
  declare_parameter<double>(
    "logic3_static_stability_radius", logic3_static_stability_radius_);
  get_parameter("logic3_static_stability_radius", logic3_static_stability_radius_);
  declare_parameter<double>(
    "logic3_static_far_stability_radius", logic3_static_far_stability_radius_);
  get_parameter(
    "logic3_static_far_stability_radius", logic3_static_far_stability_radius_);
  declare_parameter<double>(
    "logic3_static_stability_near_distance",
    logic3_static_stability_near_distance_);
  get_parameter(
    "logic3_static_stability_near_distance",
    logic3_static_stability_near_distance_);
  declare_parameter<double>(
    "logic3_static_stability_far_distance",
    logic3_static_stability_far_distance_);
  get_parameter(
    "logic3_static_stability_far_distance",
    logic3_static_stability_far_distance_);
  declare_parameter<int>(
    "logic3_static_best_score_samples", logic3_static_best_score_samples_);
  get_parameter(
    "logic3_static_best_score_samples", logic3_static_best_score_samples_);
  declare_parameter<double>(
    "logic3_static_min_match_score", logic3_static_min_match_score_);
  get_parameter(
    "logic3_static_min_match_score", logic3_static_min_match_score_);
  declare_parameter<double>(
    "logic3_static_loss_grace_seconds", logic3_static_loss_grace_seconds_);
  get_parameter(
    "logic3_static_loss_grace_seconds", logic3_static_loss_grace_seconds_);
  declare_parameter<double>(
    "logic3_static_reacquire_radius", logic3_static_reacquire_radius_);
  get_parameter(
    "logic3_static_reacquire_radius", logic3_static_reacquire_radius_);
  declare_parameter<double>(
    "logic3_red_takeover_radius", logic3_red_takeover_radius_);
  get_parameter("logic3_red_takeover_radius", logic3_red_takeover_radius_);
  declare_parameter<double>(
    "logic3_pose_correction_deadband_xy", logic3_pose_correction_deadband_xy_);
  get_parameter("logic3_pose_correction_deadband_xy", logic3_pose_correction_deadband_xy_);
  declare_parameter<double>(
    "logic3_pose_correction_deadband_yaw", logic3_pose_correction_deadband_yaw_);
  get_parameter("logic3_pose_correction_deadband_yaw", logic3_pose_correction_deadband_yaw_);
  declare_parameter<double>(
    "logic3_pose_correction_rate_xy", logic3_pose_correction_rate_xy_);
  get_parameter("logic3_pose_correction_rate_xy", logic3_pose_correction_rate_xy_);
  declare_parameter<double>(
    "logic3_pose_correction_rate_yaw", logic3_pose_correction_rate_yaw_);
  get_parameter("logic3_pose_correction_rate_yaw", logic3_pose_correction_rate_yaw_);
  declare_parameter<double>("logic3_pose_outlier_xy", logic3_pose_outlier_xy_);
  get_parameter("logic3_pose_outlier_xy", logic3_pose_outlier_xy_);
  declare_parameter<double>("logic3_pose_outlier_yaw", logic3_pose_outlier_yaw_);
  get_parameter("logic3_pose_outlier_yaw", logic3_pose_outlier_yaw_);
  declare_parameter<double>(
    "logic3_static_status_log_rate_hz", logic3_static_status_log_rate_hz_);
  get_parameter(
    "logic3_static_status_log_rate_hz", logic3_static_status_log_rate_hz_);
  declare_parameter<double>(
    "task1_qr_timeout_seconds", task1_qr_timeout_seconds_);
  get_parameter("task1_qr_timeout_seconds", task1_qr_timeout_seconds_);
  declare_parameter<std::string>(
    "task1_timeout_qr_text", task1_timeout_qr_text_);
  get_parameter("task1_timeout_qr_text", task1_timeout_qr_text_);
  declare_parameter<double>(
    "task1_endpoint_qr_wait_seconds", task1_endpoint_qr_wait_seconds_);
  get_parameter(
    "task1_endpoint_qr_wait_seconds", task1_endpoint_qr_wait_seconds_);
  declare_parameter<double>(
    "task1_endpoint_arrival_radius", task1_endpoint_arrival_radius_);
  get_parameter(
    "task1_endpoint_arrival_radius", task1_endpoint_arrival_radius_);
  declare_parameter<double>(
    "task1_transition_min_x", task1_transition_min_x_);
  get_parameter("task1_transition_min_x", task1_transition_min_x_);

  // 扫码后持续武装到waypoint_channel或配置轮数上限的条件倒车。
  // 前后视野和走廊分别可调，检测区均为扇形与矩形的交集。
  declare_parameter<double>("qr_reverse_y_threshold", qr_reverse_y_threshold_);
  get_parameter("qr_reverse_y_threshold", qr_reverse_y_threshold_);
  declare_parameter<double>(
    "qr_reverse_force_qr_x_threshold", qr_reverse_force_qr_x_threshold_);
  get_parameter(
    "qr_reverse_force_qr_x_threshold", qr_reverse_force_qr_x_threshold_);
  declare_parameter<double>("qr_reverse_x_limit", qr_reverse_x_limit_);
  get_parameter("qr_reverse_x_limit", qr_reverse_x_limit_);
  declare_parameter<double>("qr_reverse_min_distance", qr_reverse_min_distance_);
  get_parameter("qr_reverse_min_distance", qr_reverse_min_distance_);
  declare_parameter<double>(
    "qr_reverse_min_distance_max", qr_reverse_min_distance_max_);
  get_parameter("qr_reverse_min_distance_max", qr_reverse_min_distance_max_);
  declare_parameter<double>(
    "qr_reverse_min_distance_ramp_start_x",
    qr_reverse_min_distance_ramp_start_x_);
  get_parameter(
    "qr_reverse_min_distance_ramp_start_x",
    qr_reverse_min_distance_ramp_start_x_);
  declare_parameter<double>(
    "qr_reverse_min_distance_ramp_end_x",
    qr_reverse_min_distance_ramp_end_x_);
  get_parameter(
    "qr_reverse_min_distance_ramp_end_x",
    qr_reverse_min_distance_ramp_end_x_);
  declare_parameter<double>("qr_reverse_max_distance", qr_reverse_max_distance_);
  get_parameter("qr_reverse_max_distance", qr_reverse_max_distance_);
  declare_parameter<double>(
    "qr_reverse_odom_timeout_seconds", qr_reverse_odom_timeout_seconds_);
  get_parameter(
    "qr_reverse_odom_timeout_seconds", qr_reverse_odom_timeout_seconds_);
  declare_parameter<double>(
    "qr_reverse_front_obstacle_distance", qr_reverse_front_obstacle_distance_);
  get_parameter(
    "qr_reverse_front_obstacle_distance", qr_reverse_front_obstacle_distance_);
  declare_parameter<double>("qr_reverse_front_fov_deg", qr_reverse_front_fov_deg_);
  get_parameter("qr_reverse_front_fov_deg", qr_reverse_front_fov_deg_);
  declare_parameter<double>(
    "qr_reverse_front_corridor_half_width", qr_reverse_front_corridor_half_width_);
  get_parameter(
    "qr_reverse_front_corridor_half_width", qr_reverse_front_corridor_half_width_);
  declare_parameter<double>(
    "qr_reverse_front_near_half_width", qr_reverse_front_near_half_width_);
  get_parameter(
    "qr_reverse_front_near_half_width", qr_reverse_front_near_half_width_);
  declare_parameter<double>(
    "qr_reverse_front_far_half_width", qr_reverse_front_far_half_width_);
  get_parameter(
    "qr_reverse_front_far_half_width", qr_reverse_front_far_half_width_);
  declare_parameter<double>(
    "qr_reverse_rear_obstacle_distance", qr_reverse_rear_obstacle_distance_);
  get_parameter(
    "qr_reverse_rear_obstacle_distance", qr_reverse_rear_obstacle_distance_);
  declare_parameter<double>("qr_reverse_rear_fov_deg", qr_reverse_rear_fov_deg_);
  get_parameter("qr_reverse_rear_fov_deg", qr_reverse_rear_fov_deg_);
  declare_parameter<double>(
    "qr_reverse_rear_corridor_half_width", qr_reverse_rear_corridor_half_width_);
  get_parameter(
    "qr_reverse_rear_corridor_half_width", qr_reverse_rear_corridor_half_width_);
  declare_parameter<double>(
    "qr_reverse_rear_arc_angular_speed", qr_reverse_rear_arc_angular_speed_);
  get_parameter(
    "qr_reverse_rear_arc_angular_speed", qr_reverse_rear_arc_angular_speed_);
  declare_parameter<double>(
    "qr_reverse_rear_arc_left_offset", qr_reverse_rear_arc_left_offset_);
  get_parameter(
    "qr_reverse_rear_arc_left_offset", qr_reverse_rear_arc_left_offset_);
  declare_parameter<double>("qr_reverse_linear_speed", qr_reverse_linear_speed_);
  get_parameter("qr_reverse_linear_speed", qr_reverse_linear_speed_);
  declare_parameter<double>("qr_reverse_angular_speed", qr_reverse_angular_speed_);
  get_parameter("qr_reverse_angular_speed", qr_reverse_angular_speed_);
  declare_parameter<double>(
    "qr_reverse_angular_speed_max", qr_reverse_angular_speed_max_);
  get_parameter("qr_reverse_angular_speed_max", qr_reverse_angular_speed_max_);
  declare_parameter<double>(
    "qr_reverse_angular_ramp_start_x", qr_reverse_angular_ramp_start_x_);
  get_parameter("qr_reverse_angular_ramp_start_x", qr_reverse_angular_ramp_start_x_);
  declare_parameter<double>(
    "qr_reverse_angular_ramp_end_x", qr_reverse_angular_ramp_end_x_);
  get_parameter("qr_reverse_angular_ramp_end_x", qr_reverse_angular_ramp_end_x_);
  declare_parameter<double>(
    "qr_reverse_post_stop_seconds", qr_reverse_post_stop_seconds_);
  get_parameter("qr_reverse_post_stop_seconds", qr_reverse_post_stop_seconds_);
  declare_parameter<int>("qr_reverse_max_count", qr_reverse_max_count_);
  get_parameter("qr_reverse_max_count", qr_reverse_max_count_);

  // A→C→B进场坐标、结束判定与入口基准路线圆滑。
  declare_parameter<double>(
    "channel_entry_heading_reference_distance",
    channel_entry_heading_reference_distance_);
  get_parameter(
    "channel_entry_heading_reference_distance",
    channel_entry_heading_reference_distance_);
  declare_parameter<double>(
    "channel_entry_rear_static_reference_forward_distance",
    channel_entry_rear_static_reference_forward_distance_);
  get_parameter(
    "channel_entry_rear_static_reference_forward_distance",
    channel_entry_rear_static_reference_forward_distance_);
  declare_parameter<double>("channel_entry_via_x", channel_entry_via_x_);
  get_parameter("channel_entry_via_x", channel_entry_via_x_);
  declare_parameter<double>("channel_entry_via_y", channel_entry_via_y_);
  get_parameter("channel_entry_via_y", channel_entry_via_y_);
  declare_parameter<double>(
    "channel_handoff_waypoint_distance", channel_handoff_waypoint_distance_);
  get_parameter(
    "channel_handoff_waypoint_distance", channel_handoff_waypoint_distance_);
  declare_parameter<bool>(
    "channel_ac_curve_enabled", channel_ac_curve_enabled_);
  get_parameter(
    "channel_ac_curve_enabled", channel_ac_curve_enabled_);
  declare_parameter<double>(
    "channel_ac_curve_start_handle_ratio",
    channel_ac_curve_start_handle_ratio_);
  get_parameter(
    "channel_ac_curve_start_handle_ratio",
    channel_ac_curve_start_handle_ratio_);
  declare_parameter<double>(
    "channel_ac_curve_finish_handle_ratio",
    channel_ac_curve_finish_handle_ratio_);
  get_parameter(
    "channel_ac_curve_finish_handle_ratio",
    channel_ac_curve_finish_handle_ratio_);
  declare_parameter<double>(
    "channel_ac_curve_max_curvature",
    channel_ac_curve_max_curvature_);
  get_parameter(
    "channel_ac_curve_max_curvature",
    channel_ac_curve_max_curvature_);
  declare_parameter<double>(
    "channel_entry_corner_smoothing_length",
    channel_entry_corner_smoothing_length_);
  get_parameter(
    "channel_entry_corner_smoothing_length",
    channel_entry_corner_smoothing_length_);

  // 四个任务阶段分别持有速度、转向和实时雷达避障参数；正向前瞻改为
  // 全阶段共用的实测速度动态前瞻，不再由阶段profile覆盖。
  task1_profile_ = StageControlProfile{};
  transition_profile_ = StageControlProfile{};
  task2_profile_ = StageControlProfile{};
  task3_profile_ = StageControlProfile{};

  auto declare_stage_profile =
    [this](const std::string & prefix, StageControlProfile & profile) {
      auto declare_double = [&](const std::string & suffix, double & value) {
          const std::string name = prefix + "_" + suffix;
          declare_parameter<double>(name, value);
          get_parameter(name, value);
      };
      declare_double("linear_speed", profile.linear_speed);
      declare_double("turn_angular_gain", profile.turn_angular_gain);
      const std::string slowdown_name =
        prefix + "_enable_turn_speed_slowdown";
      declare_parameter<bool>(
        slowdown_name, profile.enable_turn_speed_slowdown);
      get_parameter(
        slowdown_name, profile.enable_turn_speed_slowdown);
      declare_double(
        "max_turn_speed_reduction_percent",
        profile.max_turn_speed_reduction_percent);
      declare_double("pass_radius", profile.pass_radius);
      declare_double(
        "route_projection_max_distance",
        profile.route_projection_max_distance);
      declare_double("segment_advance_t", profile.segment_advance_t);
      declare_double(
        "lidar_avoid_distance", profile.lidar_avoid_distance);
      declare_double("lidar_avoid_fov_deg", profile.lidar_avoid_fov_deg);
      declare_double(
        "lidar_corridor_half_width", profile.lidar_corridor_half_width);
      declare_double(
        "lidar_avoid_linear_speed", profile.lidar_avoid_linear_speed);
      declare_double(
        "lidar_avoid_angular_gain", profile.lidar_avoid_angular_gain);
      declare_double(
        "lidar_direction_upper_angle_deg",
        profile.lidar_direction_upper_angle_deg);
      declare_double(
        "lidar_direction_lower_angle_deg",
        profile.lidar_direction_lower_angle_deg);
      const std::string heading_count_name =
        prefix + "_lidar_heading_direction_max_count";
      declare_parameter<int>(
        heading_count_name, profile.lidar_heading_direction_max_count);
      get_parameter(
        heading_count_name, profile.lidar_heading_direction_max_count);
      const std::string route_count_name =
        prefix + "_lidar_route_side_direction_max_count";
      declare_parameter<int>(
        route_count_name, profile.lidar_route_side_direction_max_count);
      get_parameter(
        route_count_name, profile.lidar_route_side_direction_max_count);
      const std::string route_enabled_name =
        prefix + "_lidar_route_side_direction_enabled";
      declare_parameter<bool>(
        route_enabled_name, profile.lidar_route_side_direction_enabled);
      get_parameter(
        route_enabled_name, profile.lidar_route_side_direction_enabled);
      declare_double(
        "lidar_route_side_deadband", profile.lidar_route_side_deadband);
      declare_double(
        "lidar_route_side_search_backward_distance",
        profile.lidar_route_side_search_backward_distance);
      declare_double(
        "lidar_route_side_search_forward_distance",
        profile.lidar_route_side_search_forward_distance);
      declare_double(
        "lidar_route_side_angular_gain",
        profile.lidar_route_side_angular_gain);
      const std::string reverse_phase_name =
        prefix + "_lidar_route_side_reverse_phase_enabled";
      declare_parameter<bool>(
        reverse_phase_name, profile.lidar_route_side_reverse_phase_enabled);
      get_parameter(
        reverse_phase_name, profile.lidar_route_side_reverse_phase_enabled);
      declare_double(
        "lidar_route_side_reverse_exit_hysteresis_deg",
        profile.lidar_route_side_reverse_exit_hysteresis_deg);
    };
  declare_stage_profile("task1", task1_profile_);
  declare_stage_profile("transition", transition_profile_);
  declare_stage_profile("task2", task2_profile_);
  declare_stage_profile("task3", task3_profile_);

  // 条件倒车和四阶段避障共用雷达新鲜度及物理地图墙体过滤。
  declare_parameter<double>(
    "lidar_scan_timeout", lidar_scan_timeout_);
  get_parameter(
    "lidar_scan_timeout", lidar_scan_timeout_);
  declare_parameter<bool>(
    "lidar_enable_static_map_filter", lidar_enable_static_map_filter_);
  get_parameter(
    "lidar_enable_static_map_filter", lidar_enable_static_map_filter_);
  declare_parameter<int>(
    "lidar_static_map_occupied_threshold", lidar_static_map_occupied_threshold_);
  get_parameter(
    "lidar_static_map_occupied_threshold", lidar_static_map_occupied_threshold_);
  declare_parameter<double>(
    "lidar_static_map_wall_padding", lidar_static_map_wall_padding_);
  get_parameter(
    "lidar_static_map_wall_padding", lidar_static_map_wall_padding_);

  // channel结束后的返回起点控制。视觉参数与当前visual_control实跑配置保持
  // 同名语义，但由本节点读取和执行，避免恢复visual_control后争抢/cmd_vel。
  declare_parameter<bool>("return_to_start_enabled", return_to_start_enabled_);
  get_parameter("return_to_start_enabled", return_to_start_enabled_);
  declare_parameter<double>("return_base_min_x", return_base_min_x_);
  get_parameter("return_base_min_x", return_base_min_x_);
  declare_parameter<double>("return_base_max_x", return_base_max_x_);
  get_parameter("return_base_max_x", return_base_max_x_);
  declare_parameter<double>("return_base_min_y", return_base_min_y_);
  get_parameter("return_base_min_y", return_base_min_y_);
  declare_parameter<double>("return_base_max_y", return_base_max_y_);
  get_parameter("return_base_max_y", return_base_max_y_);
  declare_parameter<double>("return_stop_zone_min_x", return_stop_zone_min_x_);
  get_parameter("return_stop_zone_min_x", return_stop_zone_min_x_);
  declare_parameter<double>(
    "return_stop_zone_top_min_x", return_stop_zone_top_min_x_);
  get_parameter(
    "return_stop_zone_top_min_x", return_stop_zone_top_min_x_);
  declare_parameter<double>(
    "return_stop_zone_top_max_x", return_stop_zone_top_max_x_);
  get_parameter(
    "return_stop_zone_top_max_x", return_stop_zone_top_max_x_);
  declare_parameter<double>("return_stop_zone_max_x", return_stop_zone_max_x_);
  get_parameter("return_stop_zone_max_x", return_stop_zone_max_x_);
  declare_parameter<double>("return_stop_zone_min_y", return_stop_zone_min_y_);
  get_parameter("return_stop_zone_min_y", return_stop_zone_min_y_);
  declare_parameter<double>(
    "return_stop_zone_right_top_y", return_stop_zone_right_top_y_);
  get_parameter(
    "return_stop_zone_right_top_y", return_stop_zone_right_top_y_);
  declare_parameter<double>("return_stop_zone_max_y", return_stop_zone_max_y_);
  get_parameter("return_stop_zone_max_y", return_stop_zone_max_y_);
  declare_parameter<double>(
    "return_stop_zone_left_curve_control_x",
    return_stop_zone_left_curve_control_x_);
  get_parameter(
    "return_stop_zone_left_curve_control_x",
    return_stop_zone_left_curve_control_x_);
  declare_parameter<double>(
    "return_stop_zone_left_curve_control_y",
    return_stop_zone_left_curve_control_y_);
  get_parameter(
    "return_stop_zone_left_curve_control_y",
    return_stop_zone_left_curve_control_y_);
  declare_parameter<double>(
    "return_stop_confirm_seconds", return_stop_confirm_seconds_);
  get_parameter("return_stop_confirm_seconds", return_stop_confirm_seconds_);
  declare_parameter<bool>(
    "task3_return_slowdown_enabled", task3_return_slowdown_enabled_);
  get_parameter(
    "task3_return_slowdown_enabled", task3_return_slowdown_enabled_);
  declare_parameter<double>(
    "task3_terminal_target_x", task3_terminal_target_x_);
  get_parameter(
    "task3_terminal_target_x", task3_terminal_target_x_);
  declare_parameter<double>(
    "task3_terminal_target_y", task3_terminal_target_y_);
  get_parameter(
    "task3_terminal_target_y", task3_terminal_target_y_);
  declare_parameter<double>(
    "task3_terminal_route_end_radius", task3_terminal_route_end_radius_);
  get_parameter(
    "task3_terminal_route_end_radius", task3_terminal_route_end_radius_);
  declare_parameter<double>(
    "task3_return_slowdown_radius", task3_return_slowdown_radius_);
  get_parameter(
    "task3_return_slowdown_radius", task3_return_slowdown_radius_);
  declare_parameter<double>(
    "task3_return_slowdown_min_speed", task3_return_slowdown_min_speed_);
  get_parameter(
    "task3_return_slowdown_min_speed", task3_return_slowdown_min_speed_);
  declare_parameter<double>(
    "return_brake_reverse_speed", return_brake_reverse_speed_);
  get_parameter("return_brake_reverse_speed", return_brake_reverse_speed_);
  declare_parameter<int>(
    "return_brake_reverse_cycles", return_brake_reverse_cycles_);
  get_parameter("return_brake_reverse_cycles", return_brake_reverse_cycles_);
  declare_parameter<int>(
    "temporary_stop_brake_reverse_cycles",
    temporary_stop_brake_reverse_cycles_);
  get_parameter(
    "temporary_stop_brake_reverse_cycles",
    temporary_stop_brake_reverse_cycles_);
  declare_parameter<double>(
    "temporary_stop_brake_release_speed_threshold",
    temporary_stop_brake_release_speed_threshold_);
  get_parameter(
    "temporary_stop_brake_release_speed_threshold",
    temporary_stop_brake_release_speed_threshold_);
  declare_parameter<int>("return_brake_zero_cycles", return_brake_zero_cycles_);
  get_parameter("return_brake_zero_cycles", return_brake_zero_cycles_);
  declare_parameter<double>("return_obstacle_split_y", return_obstacle_split_y_);
  get_parameter("return_obstacle_split_y", return_obstacle_split_y_);
  declare_parameter<std::string>(
    "return_visual_detection_topic", return_visual_detection_topic_);
  get_parameter("return_visual_detection_topic", return_visual_detection_topic_);
  declare_parameter<double>(
    "return_visual_detection_timeout", return_visual_detection_timeout_);
  get_parameter("return_visual_detection_timeout", return_visual_detection_timeout_);
  declare_parameter<int>("return_visual_source_width", return_visual_source_width_);
  get_parameter("return_visual_source_width", return_visual_source_width_);
  declare_parameter<int>("return_visual_source_height", return_visual_source_height_);
  get_parameter("return_visual_source_height", return_visual_source_height_);
  declare_parameter<int>("return_visual_control_width", return_visual_control_width_);
  get_parameter("return_visual_control_width", return_visual_control_width_);
  declare_parameter<int>("return_visual_control_height", return_visual_control_height_);
  get_parameter("return_visual_control_height", return_visual_control_height_);
  declare_parameter<double>(
    "return_visual_avoid_linear_speed", return_visual_avoid_linear_speed_);
  get_parameter(
    "return_visual_avoid_linear_speed", return_visual_avoid_linear_speed_);
  declare_parameter<double>(
    "return_visual_avoid_angular_ratio", return_visual_avoid_angular_ratio_);
  get_parameter(
    "return_visual_avoid_angular_ratio", return_visual_avoid_angular_ratio_);
  declare_parameter<double>(
    "return_visual_obstacle_confidence_threshold",
    return_visual_obstacle_confidence_threshold_);
  get_parameter(
    "return_visual_obstacle_confidence_threshold",
    return_visual_obstacle_confidence_threshold_);
  declare_parameter<int>(
    "return_visual_bottom_threshold_caution",
    return_visual_bottom_threshold_caution_);
  get_parameter(
    "return_visual_bottom_threshold_caution",
    return_visual_bottom_threshold_caution_);
  declare_parameter<int>(
    "return_visual_bottom_threshold_avoid", return_visual_bottom_threshold_avoid_);
  get_parameter(
    "return_visual_bottom_threshold_avoid", return_visual_bottom_threshold_avoid_);
  declare_parameter<double>(
    "return_visual_delta_x_ratio", return_visual_delta_x_ratio_);
  get_parameter("return_visual_delta_x_ratio", return_visual_delta_x_ratio_);
  declare_parameter<double>(
    "return_visual_parking_sign_confidence_threshold",
    return_visual_parking_sign_confidence_threshold_);
  get_parameter(
    "return_visual_parking_sign_confidence_threshold",
    return_visual_parking_sign_confidence_threshold_);
  declare_parameter<int>(
    "return_visual_parking_y_threshold", return_visual_parking_y_threshold_);
  get_parameter(
    "return_visual_parking_y_threshold", return_visual_parking_y_threshold_);

  // QR 与图生文触发
  declare_parameter<std::string>("qr_topic", qr_topic_);
  get_parameter("qr_topic", qr_topic_);
  declare_parameter<int>("qr_code_height_threshold", qr_code_height_threshold_);
  get_parameter("qr_code_height_threshold", qr_code_height_threshold_);
  declare_parameter<double>("qr_code_speed_limit", qr_code_speed_limit_);
  get_parameter("qr_code_speed_limit", qr_code_speed_limit_);
  declare_parameter<std::string>("tts_topic", tts_topic_);
  get_parameter("tts_topic", tts_topic_);
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
  declare_parameter<int>("perception_shutdown_value", perception_shutdown_value_);
  get_parameter("perception_shutdown_value", perception_shutdown_value_);
  declare_parameter<int>("clockwise_qr_value", clockwise_qr_value_);
  get_parameter("clockwise_qr_value", clockwise_qr_value_);
  declare_parameter<int>("counterclockwise_qr_value", counterclockwise_qr_value_);
  get_parameter("counterclockwise_qr_value", counterclockwise_qr_value_);
  declare_parameter<bool>("enable_image_analysis_trigger", enable_image_analysis_trigger_);
  get_parameter("enable_image_analysis_trigger", enable_image_analysis_trigger_);
  declare_parameter<double>(
    "image_capture_clockwise_enter_x", image_capture_clockwise_enter_x_);
  get_parameter("image_capture_clockwise_enter_x", image_capture_clockwise_enter_x_);
  declare_parameter<double>(
    "image_capture_clockwise_exit_x", image_capture_clockwise_exit_x_);
  get_parameter("image_capture_clockwise_exit_x", image_capture_clockwise_exit_x_);
  declare_parameter<double>(
    "image_capture_counterclockwise_enter_x", image_capture_counterclockwise_enter_x_);
  get_parameter(
    "image_capture_counterclockwise_enter_x", image_capture_counterclockwise_enter_x_);
  declare_parameter<double>(
    "image_capture_counterclockwise_exit_x", image_capture_counterclockwise_exit_x_);
  get_parameter(
    "image_capture_counterclockwise_exit_x", image_capture_counterclockwise_exit_x_);
  declare_parameter<double>(
    "image_capture_corridor_min_y", image_capture_corridor_min_y_);
  get_parameter("image_capture_corridor_min_y", image_capture_corridor_min_y_);
  declare_parameter<double>(
    "image_capture_corridor_max_y", image_capture_corridor_max_y_);
  get_parameter("image_capture_corridor_max_y", image_capture_corridor_max_y_);
  declare_parameter<double>(
    "image_capture_corridor_speed", image_capture_corridor_speed_);
  get_parameter("image_capture_corridor_speed", image_capture_corridor_speed_);
  declare_parameter<double>(
    "image_capture_stop_duration", image_capture_stop_duration_);
  get_parameter("image_capture_stop_duration", image_capture_stop_duration_);
  declare_parameter<double>(
    "image_capture_yaw_tolerance", image_capture_yaw_tolerance_);
  get_parameter("image_capture_yaw_tolerance", image_capture_yaw_tolerance_);
  declare_parameter<bool>(
    "image_capture_route_progress_guard_enabled",
    image_capture_route_progress_guard_enabled_);
  get_parameter(
    "image_capture_route_progress_guard_enabled",
    image_capture_route_progress_guard_enabled_);
  declare_parameter<double>(
    "image_capture_reprojection_backward_distance",
    image_capture_reprojection_backward_distance_);
  get_parameter(
    "image_capture_reprojection_backward_distance",
    image_capture_reprojection_backward_distance_);
  declare_parameter<double>(
    "image_capture_reprojection_forward_distance",
    image_capture_reprojection_forward_distance_);
  get_parameter(
    "image_capture_reprojection_forward_distance",
    image_capture_reprojection_forward_distance_);
  declare_parameter<int>("image_capture_trigger_value", image_capture_trigger_value_);
  get_parameter("image_capture_trigger_value", image_capture_trigger_value_);
  declare_parameter<int>("image_infer_trigger_value", image_infer_trigger_value_);
  get_parameter("image_infer_trigger_value", image_infer_trigger_value_);
  declare_parameter<std::string>("image_analysis_trigger_topic", image_analysis_trigger_topic_);
  get_parameter("image_analysis_trigger_topic", image_analysis_trigger_topic_);

  route_direction_ = to_lower(route_direction_);
  tracking_controller_mode_ = to_lower(tracking_controller_mode_);
  if (tracking_controller_mode_ != "legacy_lookahead" &&
    tracking_controller_mode_ != "smac_pid")
  {
    RCLCPP_WARN(
      get_logger(), "未知 tracking_controller_mode=%s，改用 legacy_lookahead。",
      tracking_controller_mode_.c_str());
    tracking_controller_mode_ = "legacy_lookahead";
  }
  route_deformation_mode_ = to_lower(route_deformation_mode_);
  if (route_deformation_mode_ != "legacy_offset" &&
    route_deformation_mode_ != "circle_envelope")
  {
    RCLCPP_WARN(
      get_logger(), "未知 route_deformation_mode=%s，改用 legacy_offset。",
      route_deformation_mode_.c_str());
    route_deformation_mode_ = "legacy_offset";
  }
  path_pid_yaw_scale_left_ = std::max(0.01, std::abs(path_pid_yaw_scale_left_));
  path_pid_yaw_scale_right_ = std::max(0.01, std::abs(path_pid_yaw_scale_right_));
  path_pid_heading_kp_ = std::max(0.0, path_pid_heading_kp_);
  path_pid_lateral_kp_ = std::max(0.0, path_pid_lateral_kp_);
  path_pid_lateral_ki_ = std::max(0.0, path_pid_lateral_ki_);
  path_pid_lateral_kd_ = std::max(0.0, path_pid_lateral_kd_);
  path_pid_integral_limit_ = std::max(0.0, path_pid_integral_limit_);
  path_pid_derivative_filter_alpha_ = std::clamp(
    path_pid_derivative_filter_alpha_, 0.0, 1.0);
  start_speed_ = std::max(
    0.0, std::isfinite(start_speed_) ? start_speed_ : 0.12);
  start_speed_threshold_ = std::max(
    0.0, std::isfinite(start_speed_threshold_) ? start_speed_threshold_ : 0.03);
  global_max_accel_ = std::max(
    0.0, std::isfinite(global_max_accel_) ? global_max_accel_ : 1.0);
  global_max_decel_ = std::max(
    0.0, std::isfinite(global_max_decel_) ? global_max_decel_ : 1.0);
  trusted_departure_start_speed_ = std::max(
    0.0, std::isfinite(trusted_departure_start_speed_) ?
    trusted_departure_start_speed_ : 0.12);
  trusted_departure_start_max_accel_ = std::max(
    0.0, std::isfinite(trusted_departure_start_max_accel_) ?
    trusted_departure_start_max_accel_ : 0.30);
  image_capture_start_speed_ = std::max(
    0.0, std::isfinite(image_capture_start_speed_) ?
    image_capture_start_speed_ : 0.12);
  image_capture_start_max_accel_ = std::max(
    0.0, std::isfinite(image_capture_start_max_accel_) ?
    image_capture_start_max_accel_ : 0.30);
  post_reverse_start_speed_ = std::max(
    0.0, std::isfinite(post_reverse_start_speed_) ?
    post_reverse_start_speed_ : 0.12);
  post_reverse_start_max_accel_ = std::max(
    0.0, std::isfinite(post_reverse_start_max_accel_) ?
    post_reverse_start_max_accel_ : 0.30);
  path_pid_preview_base_ = std::max(0.0, path_pid_preview_base_);
  path_pid_preview_time_ = std::max(0.0, path_pid_preview_time_);
  path_pid_preview_max_ = std::max(path_pid_preview_base_, path_pid_preview_max_);
  path_pid_curvature_window_length_ = std::max(
    0.04, path_pid_curvature_window_length_);
  path_pid_max_curvature_ = std::max(0.01, path_pid_max_curvature_);
  route_projection_search_backward_distance_ = std::max(
    0.0, route_projection_search_backward_distance_);
  route_projection_search_forward_distance_ = std::max(
    0.0, route_projection_search_forward_distance_);
  path_pid_absolute_angular_limit_ = std::max(
    0.01, path_pid_absolute_angular_limit_);
  path_pid_max_angular_rate_ = std::max(0.0, path_pid_max_angular_rate_);
  circle_envelope_target_clearance_ = std::max(
    0.0, circle_envelope_target_clearance_);
  circle_envelope_numeric_margin_ = std::max(0.0, circle_envelope_numeric_margin_);
  circle_envelope_min_transition_length_ = std::max(
    0.05, circle_envelope_min_transition_length_);
  circle_envelope_max_transition_length_ = std::max(
    circle_envelope_min_transition_length_, circle_envelope_max_transition_length_);
  circle_envelope_extension_step_ = std::max(0.01, circle_envelope_extension_step_);
  circle_envelope_merge_gap_ = std::max(0.0, circle_envelope_merge_gap_);
  circle_envelope_max_curvature_ = std::max(0.01, circle_envelope_max_curvature_);
  circle_envelope_smoothing_iterations_ = std::clamp(
    circle_envelope_smoothing_iterations_, 0, 100);
  turn_speed_full_steering_angle_ = std::max(
    0.01, std::abs(turn_speed_full_steering_angle_));
  cluster_max_point_gap_ = std::max(0.01, cluster_max_point_gap_);
  cluster_max_range_jump_ = std::max(0.01, cluster_max_range_jump_);
  cluster_min_points_ = std::max(3, cluster_min_points_);
  cone_circle_min_radius_ = std::max(0.001, cone_circle_min_radius_);
  cone_circle_max_radius_ = std::max(cone_circle_min_radius_, cone_circle_max_radius_);
  cone_circle_max_rmse_ = std::max(0.001, cone_circle_max_rmse_);
  waypoint_shift_occupied_threshold_ = std::max(
    1, std::min(100, waypoint_shift_occupied_threshold_));
  waypoint_shift_check_count_ = std::max(1, waypoint_shift_check_count_);
  waypoint_cloud_check_radius_ = std::max(0.01, waypoint_cloud_check_radius_);
  task1_waypoint_shift_target_clearance_ = std::max(
    0.0, task1_waypoint_shift_target_clearance_);
  transition_waypoint_shift_target_clearance_ = std::max(
    0.0, transition_waypoint_shift_target_clearance_);
  task2_waypoint_shift_target_clearance_ = std::max(
    0.0, task2_waypoint_shift_target_clearance_);
  task3_waypoint_shift_target_clearance_ = std::max(
    0.0, task3_waypoint_shift_target_clearance_);
  if (!std::isfinite(task1_waypoint_shift_min_y_)) {
    task1_waypoint_shift_min_y_ = -0.20;
  }
  waypoint_shift_rebuild_min_progress_ratio_ = std::clamp(
    waypoint_shift_rebuild_min_progress_ratio_, -0.50, 0.90);
  waypoint_shift_rebuild_plateau_half_length_ = std::max(
    0.0, waypoint_shift_rebuild_plateau_half_length_);
  waypoint_shift_rebuild_bezier_handle_ratio_ = std::clamp(
    waypoint_shift_rebuild_bezier_handle_ratio_, 0.0, 0.45);
  task1_waypoint_shift_points_before_ = std::max(
    0, task1_waypoint_shift_points_before_);
  task1_waypoint_shift_points_after_ = std::max(
    0, task1_waypoint_shift_points_after_);
  transition_waypoint_shift_points_before_ = std::max(
    0, transition_waypoint_shift_points_before_);
  transition_waypoint_shift_points_after_ = std::max(
    0, transition_waypoint_shift_points_after_);
  task23_waypoint_shift_points_before_ = std::max(
    0, task23_waypoint_shift_points_before_);
  task23_waypoint_shift_points_after_ = std::max(
    0, task23_waypoint_shift_points_after_);
  waypoint_shift_confirm_scans_ = std::max(1, waypoint_shift_confirm_scans_);
  waypoint_shift_release_scans_ = std::max(1, waypoint_shift_release_scans_);
  waypoint_shift_min_change_ = std::max(0.0, waypoint_shift_min_change_);
  static_map_occupied_threshold_ = std::max(1, std::min(100, static_map_occupied_threshold_));
  static_map_inflation_cells_ = std::max(0, static_map_inflation_cells_);
  logic3_static_route_direction_ = to_lower(logic3_static_route_direction_);
  if (logic3_static_route_direction_ != "clockwise" &&
    logic3_static_route_direction_ != "counterclockwise")
  {
    logic3_static_route_direction_ = "clockwise";
  }
  logic3_planning_pose_source_ = to_lower(logic3_planning_pose_source_);
  if (logic3_planning_pose_source_ != "live_lidar_location" &&
    logic3_planning_pose_source_ != "frozen_map_odom" &&
    logic3_planning_pose_source_ != "filtered_map_odom")
  {
    RCLCPP_WARN(
      get_logger(), "未知 logic3_planning_pose_source，改用 filtered_map_odom。");
    logic3_planning_pose_source_ = "filtered_map_odom";
  }
  logic3_actual_route_min_distance_ = std::max(
    0.001, std::isfinite(logic3_actual_route_min_distance_) ?
    logic3_actual_route_min_distance_ : 0.02);
  logic3_actual_route_line_width_ = std::clamp(
    logic3_actual_route_line_width_, 1, 30);
  logic3_motion_speed_threshold_ = std::max(0.001, logic3_motion_speed_threshold_);
  logic3_motion_displacement_threshold_ = std::max(
    0.001, logic3_motion_displacement_threshold_);
  logic3_motion_confirm_messages_ = std::max(1, logic3_motion_confirm_messages_);
  logic3_ready_preplan_min_center_change_ = std::max(
    0.001, std::isfinite(logic3_ready_preplan_min_center_change_) ?
    logic3_ready_preplan_min_center_change_ : 0.02);
  logic3_ready_preplan_min_interval_ = std::max(
    0.0, std::isfinite(logic3_ready_preplan_min_interval_) ?
    logic3_ready_preplan_min_interval_ : 0.50);
  if (!std::isfinite(logic3_start_zone_min_x_)) {
    logic3_start_zone_min_x_ = -0.15;
  }
  if (!std::isfinite(logic3_start_zone_max_x_)) {
    logic3_start_zone_max_x_ = 0.15;
  }
  if (!std::isfinite(logic3_start_zone_min_y_)) {
    logic3_start_zone_min_y_ = -0.05;
  }
  if (!std::isfinite(logic3_start_zone_max_y_)) {
    logic3_start_zone_max_y_ = 0.20;
  }
  if (logic3_start_zone_min_x_ > logic3_start_zone_max_x_) {
    std::swap(logic3_start_zone_min_x_, logic3_start_zone_max_x_);
  }
  if (logic3_start_zone_min_y_ > logic3_start_zone_max_y_) {
    std::swap(logic3_start_zone_min_y_, logic3_start_zone_max_y_);
  }
  logic3_start_zone_guard_margin_ = std::max(
    0.0, std::isfinite(logic3_start_zone_guard_margin_) ?
    logic3_start_zone_guard_margin_ : 0.12);
  logic3_old_cluster_radius_ = std::max(0.05, logic3_old_cluster_radius_);
  logic3_green_red_filter_radius_ = std::max(
    0.05, logic3_green_red_filter_radius_);
  logic3_runtime_roi_width_ =
    std::isfinite(logic3_runtime_roi_width_) && logic3_runtime_roi_width_ > 0.0 ?
    std::max(0.10, logic3_runtime_roi_width_) : 0.0;
  logic3_runtime_roi_height_ =
    std::isfinite(logic3_runtime_roi_height_) && logic3_runtime_roi_height_ > 0.0 ?
    std::max(0.10, logic3_runtime_roi_height_) : 0.0;
  if (!std::isfinite(logic3_runtime_channel_min_y_)) {
    logic3_runtime_channel_min_y_ = -0.20;
  }
  logic3_runtime_green_min_match_score_ = std::clamp(
    logic3_runtime_green_min_match_score_, 0.0, 1.0);
  logic3_runtime_green_max_steering_span_ = std::max(
    0.0, std::isfinite(logic3_runtime_green_max_steering_span_) ?
    logic3_runtime_green_max_steering_span_ : 0.08);
  logic3_runtime_green_min_speed_ = std::max(
    0.0, std::isfinite(logic3_runtime_green_min_speed_) ?
    logic3_runtime_green_min_speed_ : 0.40);
  logic3_runtime_green_max_speed_step_ = std::max(
    0.0, std::isfinite(logic3_runtime_green_max_speed_step_) ?
    logic3_runtime_green_max_speed_step_ : 0.30);
  logic3_runtime_green_max_map_odom_translation_correction_ = std::max(
    0.0, std::isfinite(logic3_runtime_green_max_map_odom_translation_correction_) ?
    logic3_runtime_green_max_map_odom_translation_correction_ : 0.05);
  logic3_runtime_green_max_map_odom_yaw_correction_deg_ = std::max(
    0.0, std::isfinite(logic3_runtime_green_max_map_odom_yaw_correction_deg_) ?
    logic3_runtime_green_max_map_odom_yaw_correction_deg_ : 2.0);
  logic3_runtime_green_gate_log_rate_hz_ = std::max(
    0.0, std::isfinite(logic3_runtime_green_gate_log_rate_hz_) ?
    logic3_runtime_green_gate_log_rate_hz_ : 1.0);
  logic3_runtime_green_robot_exclusion_radius_ = std::max(
    0.0, std::isfinite(logic3_runtime_green_robot_exclusion_radius_) ?
    logic3_runtime_green_robot_exclusion_radius_ : 1.0);
  if (logic3_runtime_green_confirm_scans_by_point_count_.size() != 5U) {
    RCLCPP_WARN(
      get_logger(),
      "logic3_runtime_green_confirm_scans_by_point_count必须恰好包含5个整数，"
      "分别对应1/2/3/4/5+点；当前配置无效，恢复为[7,6,5,4,3]。");
    logic3_runtime_green_confirm_scans_by_point_count_ = {7, 6, 5, 4, 3};
  }
  for (auto & confirm_scans : logic3_runtime_green_confirm_scans_by_point_count_) {
    // 用户要求连续帧数最低为3；参数即使误配得更小也不能突破这个下限。
    confirm_scans = std::max<std::int64_t>(3, confirm_scans);
  }
  // 身份关联、固定锚点更新边界、永久库写入门槛和过滤覆盖相互解耦。
  logic3_runtime_green_match_radius_ = std::clamp(
    logic3_runtime_green_match_radius_, 0.05, 0.50);
  logic3_runtime_green_anchor_update_radius_ = std::clamp(
    logic3_runtime_green_anchor_update_radius_, 0.05, 1.00);
  logic3_runtime_green_average_samples_ = std::max(
    1, logic3_runtime_green_average_samples_);
  logic3_runtime_green_update_min_distance_ = std::max(
    0.0, logic3_runtime_green_update_min_distance_);
  logic3_runtime_green_replan_min_distance_ = std::max(
    0.0, logic3_runtime_green_replan_min_distance_);
  logic3_runtime_green_replan_min_interval_ = std::max(
    0.0, logic3_runtime_green_replan_min_interval_);
  logic3_runtime_green_candidate_release_scans_ = std::max(
    1, logic3_runtime_green_candidate_release_scans_);
  logic3_static_cluster_min_points_ = std::max(1, logic3_static_cluster_min_points_);
  logic3_static_restriction_inflation_cells_ = std::max(
    0, logic3_static_restriction_inflation_cells_);
  if (!std::isfinite(logic3_static_channel_max_x_)) {
    logic3_static_channel_max_x_ = 1.80;
  }
  if (!std::isfinite(logic3_static_channel_min_y_)) {
    logic3_static_channel_min_y_ = 1.75;
  }
  if (!std::isfinite(task1_static_min_x_)) {
    task1_static_min_x_ = -0.20;
  }
  if (!std::isfinite(task1_static_min_y_)) {
    task1_static_min_y_ = -0.15;
  }
  if (!std::isfinite(task1_static_avoid_min_x_)) {
    task1_static_avoid_min_x_ = 0.0;
  }
  if (!std::isfinite(task1_static_avoid_max_x_)) {
    task1_static_avoid_max_x_ = 4.50;
  }
  if (!std::isfinite(task1_static_avoid_min_y_)) {
    task1_static_avoid_min_y_ = -0.15;
  }
  if (!std::isfinite(task1_static_avoid_max_y_)) {
    task1_static_avoid_max_y_ = 0.50;
  }
  if (task1_static_avoid_min_x_ > task1_static_avoid_max_x_) {
    std::swap(task1_static_avoid_min_x_, task1_static_avoid_max_x_);
  }
  if (task1_static_avoid_min_y_ > task1_static_avoid_max_y_) {
    std::swap(task1_static_avoid_min_y_, task1_static_avoid_max_y_);
  }
  task1_qr_timeout_seconds_ = std::max(
    0.1, task1_qr_timeout_seconds_);
  task1_endpoint_qr_wait_seconds_ = std::max(
    0.0, task1_endpoint_qr_wait_seconds_);
  task1_endpoint_arrival_radius_ = std::clamp(
    task1_endpoint_arrival_radius_, 0.05, 0.50);
  if (!std::isfinite(task1_transition_min_x_)) {
    task1_transition_min_x_ = 3.50;
  }
  logic3_static_cluster_match_radius_ = std::max(
    0.01, logic3_static_cluster_match_radius_);
  logic3_static_same_frame_merge_radius_ = std::max(
    0.01, logic3_static_same_frame_merge_radius_);
  logic3_static_confirmation_window_seconds_ = std::max(
    0.1, logic3_static_confirmation_window_seconds_);
  logic3_static_confirmation_min_hits_ = std::max(
    1, logic3_static_confirmation_min_hits_);
  logic3_static_collection_stable_seconds_ = std::max(
    0.0, logic3_static_collection_stable_seconds_);
  logic3_static_stability_radius_ = std::max(
    0.01, logic3_static_stability_radius_);
  logic3_static_far_stability_radius_ = std::max(
    logic3_static_stability_radius_, logic3_static_far_stability_radius_);
  logic3_static_stability_near_distance_ = std::max(
    0.0, logic3_static_stability_near_distance_);
  logic3_static_stability_far_distance_ = std::max(
    logic3_static_stability_near_distance_ + 0.01,
    logic3_static_stability_far_distance_);
  logic3_static_cluster_match_radius_ = std::max(
    logic3_static_far_stability_radius_, logic3_static_cluster_match_radius_);
  logic3_static_best_score_samples_ = std::max(
    1, logic3_static_best_score_samples_);
  logic3_static_min_match_score_ = std::clamp(
    logic3_static_min_match_score_, 0.0, 1.0);
  logic3_static_loss_grace_seconds_ = std::max(
    0.01, logic3_static_loss_grace_seconds_);
  logic3_static_reacquire_radius_ = std::clamp(
    logic3_static_reacquire_radius_, 0.01,
    logic3_static_cluster_match_radius_);
  logic3_red_takeover_radius_ = std::max(
    0.01, logic3_red_takeover_radius_);
  logic3_pose_correction_deadband_xy_ = std::max(
    0.0, logic3_pose_correction_deadband_xy_);
  logic3_pose_correction_deadband_yaw_ = std::max(
    0.0, logic3_pose_correction_deadband_yaw_);
  logic3_pose_correction_rate_xy_ = std::max(0.0, logic3_pose_correction_rate_xy_);
  logic3_pose_correction_rate_yaw_ = std::max(0.0, logic3_pose_correction_rate_yaw_);
  logic3_pose_outlier_xy_ = std::max(0.01, logic3_pose_outlier_xy_);
  logic3_pose_outlier_yaw_ = std::max(0.01, logic3_pose_outlier_yaw_);
  logic3_static_status_log_rate_hz_ = std::max(
    0.1, logic3_static_status_log_rate_hz_);

  if (!std::isfinite(qr_reverse_x_limit_)) {
    qr_reverse_x_limit_ = 4.50;
  }
  if (!std::isfinite(qr_reverse_force_qr_x_threshold_)) {
    qr_reverse_force_qr_x_threshold_ = 3.50;
  }
  qr_reverse_min_distance_ = std::max(
    0.0, std::isfinite(qr_reverse_min_distance_) ? qr_reverse_min_distance_ : 0.45);
  qr_reverse_min_distance_max_ = std::max(
    qr_reverse_min_distance_,
    std::isfinite(qr_reverse_min_distance_max_) ?
    qr_reverse_min_distance_max_ : 0.65);
  if (!std::isfinite(qr_reverse_min_distance_ramp_start_x_)) {
    qr_reverse_min_distance_ramp_start_x_ = 2.00;
  }
  if (!std::isfinite(qr_reverse_min_distance_ramp_end_x_) ||
    qr_reverse_min_distance_ramp_end_x_ <=
    qr_reverse_min_distance_ramp_start_x_)
  {
    qr_reverse_min_distance_ramp_end_x_ =
      qr_reverse_min_distance_ramp_start_x_ + 0.50;
  }
  qr_reverse_max_distance_ = std::max(
    qr_reverse_min_distance_max_,
    std::isfinite(qr_reverse_max_distance_) ? qr_reverse_max_distance_ : 0.80);
  qr_reverse_odom_timeout_seconds_ = std::max(
    0.05, std::isfinite(qr_reverse_odom_timeout_seconds_) ?
    qr_reverse_odom_timeout_seconds_ : 0.30);
  qr_reverse_front_obstacle_distance_ = std::max(
    0.0, qr_reverse_front_obstacle_distance_);
  qr_reverse_front_fov_deg_ = std::clamp(qr_reverse_front_fov_deg_, 1.0, 360.0);
  qr_reverse_front_corridor_half_width_ = std::max(
    0.01, qr_reverse_front_corridor_half_width_);
  qr_reverse_front_near_half_width_ = std::max(
    0.01, std::isfinite(qr_reverse_front_near_half_width_) ?
    qr_reverse_front_near_half_width_ : 0.30);
  qr_reverse_front_far_half_width_ = std::clamp(
    std::isfinite(qr_reverse_front_far_half_width_) ?
    qr_reverse_front_far_half_width_ : 0.08,
    0.01, qr_reverse_front_near_half_width_);
  qr_reverse_rear_obstacle_distance_ = std::max(
    0.0, qr_reverse_rear_obstacle_distance_);
  qr_reverse_rear_fov_deg_ = std::clamp(qr_reverse_rear_fov_deg_, 1.0, 360.0);
  qr_reverse_rear_corridor_half_width_ = std::max(
    0.01, qr_reverse_rear_corridor_half_width_);
  qr_reverse_rear_arc_angular_speed_ = std::max(
    0.0, std::isfinite(qr_reverse_rear_arc_angular_speed_) ?
    qr_reverse_rear_arc_angular_speed_ : 0.50);
  qr_reverse_rear_arc_left_offset_ = std::max(
    0.0, std::isfinite(qr_reverse_rear_arc_left_offset_) ?
    qr_reverse_rear_arc_left_offset_ : 0.107);
  qr_reverse_linear_speed_ = std::max(0.0, qr_reverse_linear_speed_);
  if (!std::isfinite(qr_reverse_angular_speed_)) {
    qr_reverse_angular_speed_ = 1.00;
  }
  if (!std::isfinite(qr_reverse_angular_speed_max_)) {
    qr_reverse_angular_speed_max_ = 0.70;
  }
  if (!std::isfinite(qr_reverse_angular_ramp_start_x_)) {
    qr_reverse_angular_ramp_start_x_ = 1.10;
  }
  if (!std::isfinite(qr_reverse_angular_ramp_end_x_) ||
    qr_reverse_angular_ramp_end_x_ <= qr_reverse_angular_ramp_start_x_)
  {
    qr_reverse_angular_ramp_end_x_ = qr_reverse_angular_ramp_start_x_ + 2.00;
  }
  qr_reverse_post_stop_seconds_ = std::max(
    0.0, std::isfinite(qr_reverse_post_stop_seconds_) ?
    qr_reverse_post_stop_seconds_ : 0.50);
  qr_reverse_max_count_ = std::max(0, qr_reverse_max_count_);
  channel_entry_heading_reference_distance_ = std::max(
    0.01, std::isfinite(channel_entry_heading_reference_distance_) ?
    channel_entry_heading_reference_distance_ : 0.30);
  channel_entry_rear_static_reference_forward_distance_ = std::max(
    0.0, std::isfinite(channel_entry_rear_static_reference_forward_distance_) ?
    channel_entry_rear_static_reference_forward_distance_ : 0.20);
  if (!std::isfinite(channel_entry_via_x_)) {
    channel_entry_via_x_ = 1.95;
  }
  if (!std::isfinite(channel_entry_via_y_)) {
    channel_entry_via_y_ = 1.60;
  }
  channel_handoff_waypoint_distance_ = std::max(
    0.01, channel_handoff_waypoint_distance_);
  channel_ac_curve_start_handle_ratio_ = std::clamp(
    std::isfinite(channel_ac_curve_start_handle_ratio_) ?
    channel_ac_curve_start_handle_ratio_ : 0.368, 0.01, 1.00);
  channel_ac_curve_finish_handle_ratio_ = std::clamp(
    std::isfinite(channel_ac_curve_finish_handle_ratio_) ?
    channel_ac_curve_finish_handle_ratio_ : 0.243, 0.01, 1.00);
  channel_ac_curve_max_curvature_ = std::max(
    0.01, std::isfinite(channel_ac_curve_max_curvature_) ?
    channel_ac_curve_max_curvature_ : 10.5);
  channel_entry_corner_smoothing_length_ = std::max(
    0.0, channel_entry_corner_smoothing_length_);
  auto normalize_stage_profile =
    [](StageControlProfile & profile) {
      profile.linear_speed = std::max(0.0, profile.linear_speed);
      profile.turn_angular_gain = std::max(
        0.0, profile.turn_angular_gain);
      profile.max_turn_speed_reduction_percent = std::clamp(
        profile.max_turn_speed_reduction_percent, 0.0, 90.0);
      profile.pass_radius = std::max(0.01, profile.pass_radius);
      profile.route_projection_max_distance = std::max(
        0.10, profile.route_projection_max_distance);
      profile.segment_advance_t = std::clamp(
        profile.segment_advance_t, 0.50, 1.20);
      profile.lidar_avoid_distance = std::max(
        0.0, profile.lidar_avoid_distance);
      profile.lidar_avoid_fov_deg = std::clamp(
        profile.lidar_avoid_fov_deg, 1.0, 360.0);
      profile.lidar_corridor_half_width = std::max(
        0.01, profile.lidar_corridor_half_width);
      profile.lidar_avoid_linear_speed = std::max(
        0.0, profile.lidar_avoid_linear_speed);
      profile.lidar_avoid_angular_gain = std::max(
        0.0, profile.lidar_avoid_angular_gain);
      const double half_fov = 0.5 * profile.lidar_avoid_fov_deg;
      profile.lidar_direction_upper_angle_deg = std::clamp(
        profile.lidar_direction_upper_angle_deg, 0.0, half_fov);
      profile.lidar_direction_lower_angle_deg = std::clamp(
        profile.lidar_direction_lower_angle_deg, -half_fov, 0.0);
      if (profile.lidar_direction_lower_angle_deg >=
        profile.lidar_direction_upper_angle_deg)
      {
        const double fallback = std::min(8.0, half_fov);
        profile.lidar_direction_upper_angle_deg = fallback;
        profile.lidar_direction_lower_angle_deg = -fallback;
      }
      profile.lidar_heading_direction_max_count = std::max(
        0, profile.lidar_heading_direction_max_count);
      profile.lidar_route_side_direction_max_count = std::max(
        0, profile.lidar_route_side_direction_max_count);
      profile.lidar_route_side_deadband = std::max(
        0.0, std::isfinite(profile.lidar_route_side_deadband) ?
        profile.lidar_route_side_deadband : 0.05);
      profile.lidar_route_side_search_backward_distance = std::max(
        0.0, std::isfinite(profile.lidar_route_side_search_backward_distance) ?
        profile.lidar_route_side_search_backward_distance : 0.30);
      profile.lidar_route_side_search_forward_distance = std::max(
        0.01, std::isfinite(profile.lidar_route_side_search_forward_distance) ?
        profile.lidar_route_side_search_forward_distance : 0.80);
      profile.lidar_route_side_angular_gain = std::max(
        0.0, std::isfinite(profile.lidar_route_side_angular_gain) ?
        profile.lidar_route_side_angular_gain : profile.lidar_avoid_angular_gain);
      profile.lidar_route_side_reverse_exit_hysteresis_deg = std::clamp(
        std::isfinite(profile.lidar_route_side_reverse_exit_hysteresis_deg) ?
        profile.lidar_route_side_reverse_exit_hysteresis_deg : 2.0,
        0.0, 45.0);
    };
  normalize_stage_profile(task1_profile_);
  normalize_stage_profile(transition_profile_);
  normalize_stage_profile(task2_profile_);
  normalize_stage_profile(task3_profile_);
  lidar_scan_timeout_ = std::max(0.05, lidar_scan_timeout_);
  lidar_static_map_occupied_threshold_ = std::clamp(
    lidar_static_map_occupied_threshold_, 1, 100);
  lidar_static_map_wall_padding_ = std::max(
    0.0, lidar_static_map_wall_padding_);
  if (return_base_min_x_ > return_base_max_x_) {
    std::swap(return_base_min_x_, return_base_max_x_);
  }
  if (return_base_min_y_ > return_base_max_y_) {
    std::swap(return_base_min_y_, return_base_max_y_);
  }
  if (return_stop_zone_min_x_ > return_stop_zone_max_x_) {
    std::swap(return_stop_zone_min_x_, return_stop_zone_max_x_);
  }
  if (return_stop_zone_min_y_ > return_stop_zone_max_y_) {
    std::swap(return_stop_zone_min_y_, return_stop_zone_max_y_);
  }
  if (!std::isfinite(return_stop_zone_top_min_x_)) {
    return_stop_zone_top_min_x_ = return_stop_zone_min_x_;
  }
  return_stop_zone_top_min_x_ = std::clamp(
    return_stop_zone_top_min_x_,
    return_stop_zone_min_x_, return_stop_zone_max_x_);
  if (!std::isfinite(return_stop_zone_top_max_x_)) {
    return_stop_zone_top_max_x_ = return_stop_zone_max_x_;
  }
  return_stop_zone_top_max_x_ = std::clamp(
    return_stop_zone_top_max_x_,
    return_stop_zone_top_min_x_, return_stop_zone_max_x_);
  if (!std::isfinite(return_stop_zone_right_top_y_)) {
    return_stop_zone_right_top_y_ = return_stop_zone_max_y_;
  }
  return_stop_zone_right_top_y_ = std::clamp(
    return_stop_zone_right_top_y_,
    return_stop_zone_min_y_, return_stop_zone_max_y_);
  if (!std::isfinite(return_stop_zone_left_curve_control_x_)) {
    return_stop_zone_left_curve_control_x_ = 0.5 *
      (return_stop_zone_min_x_ + return_stop_zone_top_min_x_);
  }
  return_stop_zone_left_curve_control_x_ = std::clamp(
    return_stop_zone_left_curve_control_x_,
    return_stop_zone_min_x_, return_stop_zone_top_min_x_);
  if (!std::isfinite(return_stop_zone_left_curve_control_y_)) {
    return_stop_zone_left_curve_control_y_ = return_stop_zone_min_y_;
  }
  return_stop_zone_left_curve_control_y_ = std::clamp(
    return_stop_zone_left_curve_control_y_,
    return_stop_zone_min_y_, return_stop_zone_max_y_);
  return_stop_confirm_seconds_ = std::max(0.0, return_stop_confirm_seconds_);
  if (!std::isfinite(task3_terminal_target_x_)) {
    task3_terminal_target_x_ = 0.0;
  }
  if (!std::isfinite(task3_terminal_target_y_)) {
    task3_terminal_target_y_ = 0.0;
  }
  task3_terminal_route_end_radius_ = std::max(
    0.01, std::isfinite(task3_terminal_route_end_radius_) ?
    task3_terminal_route_end_radius_ : 0.10);
  task3_return_slowdown_radius_ = std::max(
    0.01, std::isfinite(task3_return_slowdown_radius_) ?
    task3_return_slowdown_radius_ : 0.70);
  task3_return_slowdown_min_speed_ = std::max(
    0.0, std::isfinite(task3_return_slowdown_min_speed_) ?
    task3_return_slowdown_min_speed_ : 0.50);
  return_brake_reverse_speed_ = std::max(0.0, return_brake_reverse_speed_);
  return_brake_reverse_cycles_ = std::max(0, return_brake_reverse_cycles_);
  temporary_stop_brake_reverse_cycles_ = std::max(
    0, temporary_stop_brake_reverse_cycles_);
  temporary_stop_brake_release_speed_threshold_ = std::max(
    0.0, std::isfinite(temporary_stop_brake_release_speed_threshold_) ?
    temporary_stop_brake_release_speed_threshold_ : 0.03);
  return_brake_zero_cycles_ = std::max(0, return_brake_zero_cycles_);
  if (!std::isfinite(return_obstacle_split_y_)) {
    return_obstacle_split_y_ = 0.50;
  }
  return_visual_detection_timeout_ = std::max(
    0.05, return_visual_detection_timeout_);
  return_visual_source_width_ = std::max(1, return_visual_source_width_);
  return_visual_source_height_ = std::max(1, return_visual_source_height_);
  return_visual_control_width_ = std::max(1, return_visual_control_width_);
  return_visual_control_height_ = std::max(1, return_visual_control_height_);
  return_visual_avoid_linear_speed_ = std::max(
    0.0, return_visual_avoid_linear_speed_);
  return_visual_obstacle_confidence_threshold_ = std::clamp(
    return_visual_obstacle_confidence_threshold_, 0.0, 1.0);
  return_visual_bottom_threshold_caution_ = std::max(
    0, return_visual_bottom_threshold_caution_);
  return_visual_bottom_threshold_avoid_ = std::max(
    return_visual_bottom_threshold_caution_, return_visual_bottom_threshold_avoid_);
  return_visual_delta_x_ratio_ = std::max(0.0, return_visual_delta_x_ratio_);
  return_visual_parking_sign_confidence_threshold_ = std::clamp(
    return_visual_parking_sign_confidence_threshold_, 0.0, 1.0);
  return_visual_parking_y_threshold_ = std::max(
    0, return_visual_parking_y_threshold_);
  qr_code_height_threshold_ = std::max(0, qr_code_height_threshold_);
  qr_code_speed_limit_ = std::max(0.0, qr_code_speed_limit_);
  if (image_capture_corridor_min_y_ > image_capture_corridor_max_y_) {
    std::swap(image_capture_corridor_min_y_, image_capture_corridor_max_y_);
  }
  image_capture_corridor_speed_ = std::max(0.0, image_capture_corridor_speed_);
  image_capture_stop_duration_ = std::max(0.0, image_capture_stop_duration_);
  image_capture_yaw_tolerance_ = std::clamp(
    image_capture_yaw_tolerance_, 0.0, M_PI);
  image_capture_reprojection_backward_distance_ = std::max(
    0.0, image_capture_reprojection_backward_distance_);
  image_capture_reprojection_forward_distance_ = std::max(
    0.0, image_capture_reprojection_forward_distance_);
  if (!enable_qr_direction_select_) {
    RCLCPP_WARN(get_logger(), "四阶段流程需要 QR 选路，已自动打开 QR 订阅。");
    enable_qr_direction_select_ = true;
  }

  // 启动时始终预载 Task1 路线。物理底盘开关关闭期间节点会持续完成
  // 静态绿色候选确认；开关打开后同一条路线立即可用，不需要等待二维码。
  // 顺/逆时针文件同时各保留一份无偏移保护线，仅用于限制运行绿/橙点
  // 坐标更新不得跨越原始路线；它们不参与此刻的控制和路径偏移。
  logic3_clockwise_update_guard_route_ = load_waypoints(
    route_waypoints_clockwise_file_, "clockwise");
  logic3_counterclockwise_update_guard_route_ = load_waypoints(
    route_waypoints_counterclockwise_file_, "counterclockwise");
  waypoints_ = load_waypoints(task1_waypoints_file_, "task1");
  reset_waypoint_shift_state();
  task1_original_route_ = original_waypoints_;
  selected_route_direction_ = "task1";
  waiting_for_qr_start_ = false;
  following_qr_path_ = true;
  set_task_phase(TaskPhase::TASK1);

  // 发布速度命令，最终由主线底盘驱动订阅 /cmd_vel 执行。
  cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
  tts_pub_ = create_publisher<std_msgs::msg::String>(tts_topic_, 10);

  // 发布图像分析触发。transient_local 让后启动的订阅者也能收到最后一次触发值。
  image_analysis_trigger_pub_ =
    create_publisher<std_msgs::msg::Int32>(image_analysis_trigger_topic_, rclcpp::QoS(10).transient_local());
  visual_control_suspend_pub_ = create_publisher<std_msgs::msg::Int32>(visual_control_suspend_topic_, 3);
  logic3_snapshot_pub_ = create_publisher<std_msgs::msg::String>(
    "/visual_lidar_path_follower/route_snapshot",
    rclcpp::QoS(32).transient_local().reliable());
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
      latest_scan_received_at_ = std::chrono::steady_clock::now();
      ++scan_sequence_;
      // The diagnostic can arrive before or after this node receives /scan.
      // Latch it only when both messages carry the same scan timestamp.
      latch_logic3_match_diagnostics_to_latest_scan();
    });
  if (!logic3_match_diagnostics_topic_.empty()) {
    logic3_match_diagnostics_sub_ =
      create_subscription<geometry_msgs::msg::Vector3Stamped>(
      logic3_match_diagnostics_topic_, rclcpp::QoS(10),
      [this](geometry_msgs::msg::Vector3Stamped::ConstSharedPtr msg) {
        latest_match_diagnostics_stamp_nanoseconds_ = rclcpp::Time(
          msg->header.stamp, get_clock()->get_clock_type()).nanoseconds();
        latest_lidar_match_score_ = msg->vector.x;
        latest_map_odom_translation_correction_ = msg->vector.y;
        latest_map_odom_yaw_correction_ = msg->vector.z;
        latest_match_diagnostics_valid_ =
          std::isfinite(msg->vector.x) && std::isfinite(msg->vector.y) &&
          std::isfinite(msg->vector.z);
        // The values above are pending until their timestamp matches the
        // follower's current /scan. This also handles diagnostics arriving
        // before the corresponding scan callback.
        latest_lidar_match_score_valid_ = false;
        latch_logic3_match_diagnostics_to_latest_scan();
      });
  }
  logic3_match_score_sub_ = create_subscription<std_msgs::msg::Float32>(
    logic3_match_score_topic_, rclcpp::QoS(10),
    [this](std_msgs::msg::Float32::ConstSharedPtr msg) {
      // A configured stamped diagnostic is the authoritative score source.
      // Keep this legacy scalar path only for configurations that explicitly
      // disable the diagnostic topic.
      if (!logic3_match_diagnostics_topic_.empty()) {
        return;
      }
      latest_lidar_match_score_ = static_cast<double>(msg->data);
      latest_lidar_match_score_valid_ = std::isfinite(latest_lidar_match_score_);
      // match_score由lidar_location处理完一帧/scan后发布。消息本身没有
      // Header，因此锁存回调到达时最新的scan序号，让运行绿点只处理
      // 已经得到匹配分数的雷达帧，而不是抢先使用上一帧分数。
      latest_match_score_scan_sequence_ = scan_sequence_;
    });
  logic3_cmd_vel_history_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_topic_, rclcpp::QoS(100),
    [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
      const std::int64_t stamp_nanoseconds = now().nanoseconds();
      logic3_command_history_.push_back(
        Logic3CommandSample{stamp_nanoseconds, msg->angular.z, msg->linear.x});
      // Two seconds comfortably covers a lidar scan and inter-process
      // diagnostic latency. Keep one older sample as the command active at
      // the beginning of the retained interval.
      constexpr std::int64_t kHistoryDurationNanoseconds = 2000000000LL;
      const std::int64_t oldest_stamp =
        stamp_nanoseconds - kHistoryDurationNanoseconds;
      while (logic3_command_history_.size() > 1 &&
        logic3_command_history_[1].stamp_nanoseconds < oldest_stamp)
      {
        logic3_command_history_.pop_front();
      }
    });
  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic_, rclcpp::QoS(1).transient_local().reliable(),
    [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {latest_map_ = msg;});
  restriction_map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    restriction_map_topic_, rclcpp::QoS(1).transient_local().reliable(),
    [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
      latest_restriction_map_ = msg;
    });
  logic3_static_restriction_map_sub_ =
    create_subscription<nav_msgs::msg::OccupancyGrid>(
      logic3_static_restriction_map_topic_,
      rclcpp::QoS(1).transient_local().reliable(),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
        latest_logic3_static_restriction_map_ = msg;
      });
  logic3_runtime_restriction_map_sub_ =
    create_subscription<nav_msgs::msg::OccupancyGrid>(
      logic3_runtime_restriction_map_topic_,
      rclcpp::QoS(1).transient_local().reliable(),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
        latest_logic3_runtime_restriction_map_ = msg;
      });
  logic3_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    logic3_odom_topic_, rclcpp::SensorDataQoS().keep_last(10),
    std::bind(
      &VisualLidarPathFollower::logic3_odom_callback, this,
      std::placeholders::_1));

  return_visual_detection_sub_ =
    create_subscription<ai_msgs::msg::PerceptionTargets>(
    return_visual_detection_topic_, rclcpp::SensorDataQoS().keep_last(1),
    [this](ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg) {
      latest_return_visual_targets_ = msg;
      latest_return_visual_received_at_ = std::chrono::steady_clock::now();
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
  const std::string route_name = following_qr_path_ ? "Task1" :
    route_direction_name(selected_route_direction_);
  RCLCPP_INFO(
    get_logger(),
    "VisualLidarPathFollower 初始化完成：%zu 个航点，路线=%s，跟踪器=%s，路线变形=%s。",
    waypoints_.size(), route_name.c_str(), tracking_controller_mode_.c_str(),
    route_deformation_mode_.c_str());
  if (debug_control_logs_) {
    RCLCPP_INFO(
      get_logger(),
      "debug: 倒车点=%zu，Task1=%s，顺时针=%s，逆时针=%s，"
      "动态前瞻=%.2f+%.2f*|v|（上限%.2f），"
      "避障=全阶段静态+动态雷达控制，线速度=%.2f。",
      static_cast<std::size_t>(reverse_count), task1_waypoints_file_.c_str(),
      route_waypoints_clockwise_file_.c_str(), route_waypoints_counterclockwise_file_.c_str(),
      lookahead_base_, lookahead_speed_gain_, lookahead_max_, linear_speed_);
  }
  publish_debug_route();

  // 定时控制循环：每次读取 TF、更新路线进度并计算离散前瞻 waypoint 速度。
  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / std::max(control_frequency_, 1.0)),
    std::bind(&VisualLidarPathFollower::control_once, this));
}

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

// ********************************************************************************
// ******************************** 主控制流程 ********************************
// ********************************************************************************

// 主控制循环：拿位姿 -> 更新路线线段进度 -> 算离散前瞻 waypoint 速度 -> 发布。
// 这里保留完整顺序，避免为了几行赋值再跳到小包装函数里。
void VisualLidarPathFollower::control_once()
{
  if (waiting_for_qr_start_) {
    if (!logic3_planning_pose_anchored_ && !waypoints_.empty())
    {
      RobotPose planning_pose;
      if (lookup_robot_pose(planning_pose.x, planning_pose.y, planning_pose.yaw)) {
        if (!logic3_motion_candidate_ && !logic3_start_zone_motion_pending_) {
          logic3_force_full_route_ = true;
          update_waypoint_shifts(planning_pose, 0);
          logic3_force_full_route_ = false;
        }
        publish_debug_visualization(planning_pose);
      }
    }
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
    if (qr_escape_state_ == QrEscapeState::WAITING_FOR_PRE_REVERSE_SCAN) {
      // 真实二维码交接可能明确跳过反相制动；等待新雷达帧期间必须继续
      // 走同一条直接零速通道，不能被残余实测轮速重新武装反相脉冲。
      publish_translation_stop_without_reverse_brake(qr_reverse_hold_angular_command_);
      return;
    }
    if (qr_escape_state_ == QrEscapeState::PRE_REVERSE_BRAKE ||
      qr_escape_state_ == QrEscapeState::BACKING_ARMED ||
      qr_escape_state_ == QrEscapeState::BACKING_RIGHT ||
      qr_escape_state_ == QrEscapeState::POST_STOP ||
      qr_escape_state_ == QrEscapeState::WAITING_FOR_POST_STOP_SCAN ||
      channel_entry_route_rebuild_pending_)
    {
      // 正在倒车、倒车后静止或等待按新A点重建路线时不能盲目沿用旧指令。
      publish_translation_stop(qr_reverse_hold_angular_command_);
      return;
    }
    // 正向行驶时单次定位查询失败不能插入零速脉冲；本周期不刷新控制指令，
    // 由底盘继续执行上一条有效指令，避免50Hz控制中出现走走停停。
    return;
  }
  record_logic3_actual_route(pose);

  if (task_phase_ == TaskPhase::TASK1 && logic3_planning_pose_anchored_) {
    const auto steady_now = std::chrono::steady_clock::now();
    if (!task1_pending_direction_.empty()) {
      if (pose.x >= task1_transition_min_x_) {
        const std::string pending_direction = task1_pending_direction_;
        const std::string pending_reason = task1_pending_reason_;
        const bool pending_from_qr = task1_pending_from_qr_;
        if (complete_task1_and_select_route(
            pending_direction, pending_reason.c_str(), pending_from_qr))
        {
          // complete_task1_and_select_route已在路线切换前下发对应停车命令；
          // 本控制周期必须立刻结束，不能再计算Task1正向命令。
          return;
        }
        // 门槛已经满足但路线加载/规划暂未成功时，也不能在同一周期恢复
        // Task1正向巡线。真实二维码继续保持直接零速；默认流程保持原出口。
        set_control_state(ControlState::WAITING_FOR_QR);
        if (pending_from_qr) {
          publish_translation_stop_without_reverse_brake(
            qr_reverse_hold_angular_command_);
        } else {
          publish_translation_stop(qr_reverse_hold_angular_command_);
        }
        return;
      }
    } else if (task1_endpoint_waiting_) {
      if (!task1_endpoint_wait_timer_started_) {
        task1_endpoint_wait_timer_started_ = true;
        task1_endpoint_wait_started_at_ = steady_now;
        RCLCPP_INFO(
          get_logger(),
          "Task1末点已直接零速停车（不执行反相制动），开始计时等待二维码%.2f秒。",
          task1_endpoint_qr_wait_seconds_);
      }
      const double wait_seconds =
        std::chrono::duration<double>(
        steady_now - task1_endpoint_wait_started_at_).count();
      if (wait_seconds < task1_endpoint_qr_wait_seconds_) {
        set_control_state(ControlState::WAITING_FOR_QR);
        publish_translation_stop_without_reverse_brake(qr_reverse_hold_angular_command_);
        return;
      }
      if (!task1_endpoint_fallback_logged_) {
        task1_endpoint_fallback_logged_ = true;
        task1_pending_qr_text_ = task1_timeout_qr_text_;
        task1_pending_qr_is_fallback_ = true;
        task1_qr_speech_pending_ = true;
        task1_qr_speech_published_ = false;
        RCLCPP_WARN(
          get_logger(),
          "Task1末点二维码等待已结束：停车等待%.2f秒后仍未收到有效二维码，"
          "默认选择顺时针路线；进入下一阶段后按真实二维码流程播报%s。",
          wait_seconds, task1_timeout_qr_text_.c_str());
      }
      if (complete_task1_and_select_route(
          "clockwise", "Task1终点等待结束仍未识别到有效二维码", false))
      {
        return;
      }
    } else if (task1_departure_timer_started_) {
      const double elapsed_seconds =
        std::chrono::duration<double>(
        steady_now - task1_departure_started_at_).count();
      if (elapsed_seconds >= task1_qr_timeout_seconds_) {
        if (!task1_qr_timeout_logged_) {
          task1_qr_timeout_logged_ = true;
          task1_pending_qr_text_ = task1_timeout_qr_text_;
          task1_pending_qr_is_fallback_ = true;
          task1_qr_speech_pending_ = true;
          task1_qr_speech_published_ = false;
          RCLCPP_WARN(
            get_logger(),
            "Task1二维码识别已超时：可信发车后已等待%.2f秒，"
            "仍未收到有效二维码，默认选择顺时针路线；"
            "进入下一阶段后按真实二维码流程播报%s。",
            elapsed_seconds, task1_timeout_qr_text_.c_str());
        }
        if (complete_task1_and_select_route(
            "clockwise", "Task1可信发车后二维码识别超时", false))
        {
          return;
        }
      }
    }
  }

  if (task3_terminal_homing_active_) {
    control_task3_terminal_homing(pose);
    return;
  }

  if (return_to_start_active_) {
    control_return_to_start(pose);
    return;
  }

  next_control_state_ = ControlState::PATH_FOLLOWING;

  // 正式进入过渡且入口X要求强制倒车时，先完成一次反相停车制动。
  // 此状态位于路线重建和雷达判断之前：制动未结束时禁止继续正向巡线，
  // 也不允许用制动期间缓存的旧雷达帧决定倒车动作。
  if (qr_channel_approach_active_ &&
    qr_escape_state_ == QrEscapeState::PRE_REVERSE_BRAKE)
  {
    set_control_state(next_control_state_);
    publish_translation_stop(qr_reverse_hold_angular_command_);
    if (!stop_brake_sequence_active_) {
      qr_pre_reverse_scan_sequence_ = scan_sequence_;
      qr_pre_reverse_time_nanoseconds_ = now().nanoseconds();
      qr_escape_state_ = QrEscapeState::WAITING_FOR_PRE_REVERSE_SCAN;
      RCLCPP_INFO(
        get_logger(),
        "二维码倒车前反相制动完成：保持零速度，等待制动完成后的新雷达帧。"
      );
    }
    return;
  }

  if (qr_channel_approach_active_ &&
    qr_escape_state_ == QrEscapeState::WAITING_FOR_PRE_REVERSE_SCAN)
  {
    bool fresh_scan_ready =
      latest_scan_ && scan_sequence_ > qr_pre_reverse_scan_sequence_;
    if (fresh_scan_ready) {
      const std::int64_t scan_stamp_nanoseconds = rclcpp::Time(
        latest_scan_->header.stamp, get_clock()->get_clock_type()).nanoseconds();
      fresh_scan_ready = scan_stamp_nanoseconds == 0 ||
        scan_stamp_nanoseconds > qr_pre_reverse_time_nanoseconds_;
    }
    if (!fresh_scan_ready) {
      set_control_state(next_control_state_);
      publish_translation_stop_without_reverse_brake(qr_reverse_hold_angular_command_);
      return;
    }
    // 本周期的pose是停车命令后的最新位姿；PENDING直接使用刚确认的新
    // 雷达帧进入强制倒车状态。A→C→B规划必须等该状态退出后才执行。
    qr_escape_state_ = QrEscapeState::PENDING;
  }

  bool rebuilt_after_completed_reverse = false;
  auto rebuild_pending_channel_entry = [&]() {
      if (!channel_entry_route_rebuild_pending_) {
        return true;
      }
      const bool after_completed_reverse = channel_entry_route_rebuild_after_reverse_;
      const char * reason = after_completed_reverse ?
        "大厅倒车完成" : "二维码路线切换";
      if (rebuild_channel_entry_route(pose, reason)) {
        rebuilt_after_completed_reverse = after_completed_reverse;
        return true;
      }
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "%s后无法从当前base_link重建waypoint_channel进场路线，当前停车。",
        reason);
      if (after_completed_reverse) {
        publish_translation_stop(qr_reverse_hold_angular_command_);
      } else {
        publish_stop();
      }
      return false;
    };

  // 强制倒车状态必须先于任何A→C→B/circle_envelope规划执行。当前路线
  // 重建待办保留到倒车状态结束；否则一次马上会失效的规划会阻塞负速度。
  if (qr_channel_approach_active_) {
    geometry_msgs::msg::Twist reverse_cmd;
    if (apply_qr_reverse_maneuver(pose, reverse_cmd)) {
      set_control_state(next_control_state_);
      publish_velocity_command(reverse_cmd);
      publish_debug_visualization(pose);
      return;
    }
    // 每轮实际倒车退出后先由POST_STOP持续发布配置时长的零速度；只有
    // 停车结束，apply_qr_reverse_maneuver才会在这里重新置位重规划待办。
    // 随后用停车结束时的最新base_link重建A点；倒车机制可继续保持武装。
    if (!rebuild_pending_channel_entry()) {
      return;
    }
    if (rebuilt_after_completed_reverse) {
      // 停车结束的这一周期先完成重规划并明确下发零速；下一控制周期
      // 才沿新路线前进，避免重规划与正向指令在同一周期交接。
      set_control_state(next_control_state_);
      publish_translation_stop(qr_reverse_hold_angular_command_);
      publish_debug_visualization(pose);
      return;
    }
    update_qr_channel_approach_phase(pose);
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

  const bool freeze_image_capture_route_progress =
    image_capture_route_progress_guard_enabled_ &&
    image_capture_corridor_state_ == ImageCaptureCorridorState::STOPPING;
  if (image_capture_route_guard_active_ &&
    image_capture_corridor_state_ != ImageCaptureCorridorState::STOPPING)
  {
    // 拍照完成后的第一个起步控制周期先从停车锚点附近受限重投影；此时
    // find_route_projection()也受同一弧长窗口约束，不能被定位前跳带飞。
    reproject_image_capture_route_on_resume(pose);
  }
  if (first_waypoint_reached_ && !freeze_image_capture_route_progress) {
    update_route_progress(pose);
  }

  // 生成路线里的返程直线仍属于channel。这里只在进入该段后
  // 打开停车标志识别并执行停车判定，不切换追点或避障控制器。
  if (handle_task3_start(pose)) {
    return;
  }

  if (stop_if_route_finished(pose)) {
    return;
  }

  // Task2顶部直道不再追逐独立拍照点。进入当前方向的检测矩形后替换
  // Task2基础线速度；位置和航向合格后只停车一次。停车期间继续运行
  // 巡线和雷达避障，最终只覆盖平移速度。
  const bool image_capture_hold_translation = update_image_capture_corridor(pose);

  const double lookahead_distance = std::min(
    lookahead_max_,
    lookahead_base_ + lookahead_speed_gain_ * std::abs(latest_chassis_linear_speed_));
  auto target = compute_lookahead_target(pose, lookahead_distance);
  if (!target.reverse) {
    if (!logic3_planning_pose_anchored_) {
      logic3_force_full_route_ = true;
    }
    update_waypoint_shifts(pose, target.waypoint_index);
    logic3_force_full_route_ = false;
    target = compute_lookahead_target(pose, lookahead_distance);
  }

  geometry_msgs::msg::Twist cmd;
  bool path_tracking_failed = false;
  if (!target.reverse && tracking_controller_mode_ == "smac_pid") {
    if (!compute_path_pid_tracking_command(pose, cmd)) {
      // smac_pid失败时禁止用另一套控制器掩盖问题。保持路线和节点运行，
      // 但本周期明确输出零速；具体失败原因由对应ERROR日志给出。
      cmd = geometry_msgs::msg::Twist();
      path_tracking_failed = true;
    }
  } else {
    reset_path_pid_state();
    cmd = compute_tracking_command(pose, target);
  }
  const bool image_capture_corridor_speed_active =
    !path_tracking_failed && !target.reverse && cmd.linear.x > 0.0 &&
    image_capture_corridor_speed_ > 0.0 &&
    image_capture_corridor_contains(pose);
  if (image_capture_corridor_speed_active) {
    // 只替换Task2的基础正向速度，不改变巡线或避障计算出的转向量。
    cmd.linear.x = image_capture_corridor_speed_;
  }
  // 两套正向跟线控制器共用同一套转弯降速。检测矩形速度在这里之前
  // 替换基础速度，因此原有转弯减速仍继续生效。
  if (!path_tracking_failed) {
    apply_turn_speed_limit(cmd);
  }
  const double image_capture_corridor_turn_limited_cap =
    image_capture_corridor_speed_active && cmd.linear.x > 0.0 ?
    cmd.linear.x : std::numeric_limits<double>::infinity();
  const bool runtime_lidar_active = logic3_planning_pose_anchored_ &&
    task_phase_ != TaskPhase::FINISHED;
  if (runtime_lidar_active && !target.reverse && !path_tracking_failed) {
    if (apply_channel_lidar_obstacle_avoidance(cmd)) {
      // 雷达覆盖了本周期的路线控制命令，下次回到路线时重新建立PID状态。
      reset_path_pid_state();
    }
  }
  if (std::isfinite(image_capture_corridor_turn_limited_cap) && cmd.linear.x > 0.0) {
    // 雷达接管可以继续减速，但不能把矩形基础速度或转弯后的速度重新抬高。
    cmd.linear.x = std::min(cmd.linear.x, image_capture_corridor_turn_limited_cap);
  }
  // 只在Task1二维码尚未有效解码时，根据当前检测框高度封顶最终线速度。
  // 放在雷达控制之后，确保雷达分支不会把0.20m/s限制重新抬高。
  apply_qr_code_speed_limit(cmd);
  // Task3返程减速是最终正向速度上限，放在实时雷达接管之后，确保靠近
  // 基地时雷达避障也不会把速度重新抬高。零速和倒车不会被它改变。
  apply_task3_return_slowdown(pose, cmd);
  if (image_capture_hold_translation) {
    // 拍照停车只禁止车身平移，保留巡线或雷达避障最终算出的angular.z。
    // 不调用publish_stop()，避免舵机回中和巡线PID状态被拍照逻辑重置。
    cmd.linear.x = 0.0;
    cmd.linear.y = 0.0;
  }
  set_control_state(next_control_state_);
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
  // Foxglove Marker构建会重新聚类并创建大量圆弧点；始终排在本周期
  // 底盘控制发布和必要的红点事件锁存之后，不能延迟首条运动指令。
  publish_debug_visualization(pose);
  // 轮速真正离开静止阈值后才恢复普通局部投影；没有轮速反馈时则在
  // 完成一次受限重投影后解除，避免永久锁在停车锚点附近。
  maybe_release_image_capture_route_guard();
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

// 逐点法线偏移在圆弧内侧满足 kappa * d >= 1 时会退化成尖点并反相。
// 这里先生成旧算法候选，再检测离散曲率、沿原路线进度和局部自交；只有
// 检测到不可行的最小连续偏移点段才改用同侧局部曲线；同一障碍窗口内的
// 其余点保留逐点法线偏移。局部曲线只允许改写危险点本身，并且每个原本
// 要求偏移的点都必须保留完整偏移量；若无法同时消除反相/自交并满足该
// 约束，就保留原始直接偏移（包括局部反相），绝不扩大融合或回退原路线。
std::size_t VisualLidarPathFollower::rebuild_curvature_unsafe_shift_segments(
  std::vector<Waypoint> & shifted_route)
{
  const std::size_t route_size = original_waypoints_.size();
  if (!waypoint_shift_enable_local_rebuild_ || route_size < 4 ||
    shifted_route.size() != route_size ||
    waypoint_shift_offsets_.size() != route_size ||
    waypoint_shift_world_directions_.size() != route_size ||
    waypoint_shift_source_indices_.size() != route_size)
  {
    return 0;
  }

  auto normalize_vector = [](Point2D vector, Point2D & unit) {
      const double length = std::hypot(vector.x, vector.y);
      if (length < 1e-9) {
        return false;
      }
      unit = Point2D{vector.x / length, vector.y / length};
      return true;
    };
  auto route_tangent_at = [&](const std::vector<Waypoint> & route,
      std::size_t index, Point2D & tangent) {
      const std::size_t previous = index > 0 ? index - 1 : index;
      const std::size_t next = std::min(index + 1, route.size() - 1);
      return normalize_vector(
        Point2D{route[next].x - route[previous].x,
          route[next].y - route[previous].y}, tangent);
    };
  auto point_distance = [](const Waypoint & lhs, const Waypoint & rhs) {
      return std::hypot(lhs.x - rhs.x, lhs.y - rhs.y);
    };
  auto signed_area = [](const Waypoint & a, const Waypoint & b, const Waypoint & c) {
      return (b.x - a.x) * (c.y - a.y) -
             (b.y - a.y) * (c.x - a.x);
    };
  auto proper_segments_intersect = [&](const Waypoint & a, const Waypoint & b,
      const Waypoint & c, const Waypoint & d) {
      constexpr double intersection_epsilon = 1e-9;
      const double ab_c = signed_area(a, b, c);
      const double ab_d = signed_area(a, b, d);
      const double cd_a = signed_area(c, d, a);
      const double cd_b = signed_area(c, d, b);
      return ((ab_c > intersection_epsilon && ab_d < -intersection_epsilon) ||
             (ab_c < -intersection_epsilon && ab_d > intersection_epsilon)) &&
             ((cd_a > intersection_epsilon && cd_b < -intersection_epsilon) ||
             (cd_a < -intersection_epsilon && cd_b > intersection_epsilon));
    };
  auto local_geometry_valid = [&](const std::vector<Waypoint> & route,
      std::size_t first, std::size_t last, std::string & invalid_reason) {
      // first/last是本轮真正改写的两个锚点。锚点之外的直接偏移没有被
      // 本轮修改，不能因为相邻障碍物在所有权交界处原本就有偏移跳变，
      // 反过来否决当前这段独立圆弧的合法重建。
      for (std::size_t index = first; index < last; ++index) {
        const double new_length = point_distance(route[index], route[index + 1]);
        const double original_length = point_distance(
          original_waypoints_[index], original_waypoints_[index + 1]);
        if (new_length < 1e-6 ||
          new_length > std::max(0.10, 6.0 * original_length))
        {
          invalid_reason = "相邻点距离异常";
          return false;
        }
      }
      // 锚点本身也要检查接入角，因此转角检查比改写区各向外读取一个
      // 未修改点；这里只读邻点，不把外侧线段当作本轮候选重新验证。
      const std::size_t turn_first = std::max<std::size_t>(1, first);
      const std::size_t turn_last = std::min(last, route_size - 2);
      for (std::size_t index = turn_first; index <= turn_last; ++index) {
        const Point2D incoming{
          route[index].x - route[index - 1].x,
          route[index].y - route[index - 1].y};
        const Point2D outgoing{
          route[index + 1].x - route[index].x,
          route[index + 1].y - route[index].y};
        const double incoming_length = std::hypot(incoming.x, incoming.y);
        const double outgoing_length = std::hypot(outgoing.x, outgoing.y);
        if (incoming_length < 1e-9 || outgoing_length < 1e-9) {
          invalid_reason = "接入处存在零长度线段";
          return false;
        }
        const double turn_cosine =
          (incoming.x * outgoing.x + incoming.y * outgoing.y) /
          (incoming_length * outgoing_length);
        if (turn_cosine < -0.25) {
          invalid_reason = "接入处或曲线内部出现急剧反向";
          return false;
        }
      }
      const std::size_t intersection_first = first > 0 ? first - 1 : first;
      const std::size_t intersection_last = std::min(last + 1, route_size - 1);
      for (std::size_t first_segment = intersection_first;
        first_segment + 1 <= intersection_last; ++first_segment)
      {
        for (std::size_t second_segment = first_segment + 2;
          second_segment + 1 <= intersection_last; ++second_segment)
        {
          if (proper_segments_intersect(
              route[first_segment], route[first_segment + 1],
              route[second_segment], route[second_segment + 1]))
          {
            invalid_reason = "局部曲线自交";
            return false;
          }
        }
      }
      invalid_reason.clear();
      return true;
    };
  auto chord_progress_valid = [&](const std::vector<Waypoint> & route,
      std::size_t first, std::size_t last, const Point2D & start,
      const Point2D & finish, std::string & invalid_reason) {
      if (first >= last) {
        return true;
      }
      const Point2D chord{finish.x - start.x, finish.y - start.y};
      const double chord_length_squared =
        chord.x * chord.x + chord.y * chord.y;
      if (chord_length_squared < 1e-12) {
        invalid_reason = "过渡段两锚点重合";
        return false;
      }
      // 圆角切弦本来就不会逐点平行于原圆弧切线，因此不能再用“同索引
      // 原线段点积”否决合法的切角路线。改为要求Bezier采样沿自己的起终
      // 点弦严格单调前进；这能直接禁止先向前再向后，同时允许独立圆弧
      // 在不侵入相邻障碍窗口的前提下切角接回。
      for (std::size_t index = first; index < last; ++index) {
        const Point2D segment{
          route[index + 1].x - route[index].x,
          route[index + 1].y - route[index].y};
        const double chord_progress =
          (segment.x * chord.x + segment.y * chord.y) /
          chord_length_squared;
        if (chord_progress <= 1e-6) {
          invalid_reason = "Bezier过渡沿自身接回弦发生停滞或反向";
          return false;
        }
      }
      return true;
    };
  auto fill_bezier = [&](std::vector<Waypoint> & route,
      std::size_t first, std::size_t last, const Point2D & start,
      const Point2D & finish, const Point2D & start_tangent,
      const Point2D & finish_tangent, double handle_ratio) {
      if (first >= last) {
        route[first].x = start.x;
        route[first].y = start.y;
        return;
      }
      const double chord_length = std::hypot(
        finish.x - start.x, finish.y - start.y);
      const double handle_length = handle_ratio * chord_length;
      const Point2D control1{
        start.x + handle_length * start_tangent.x,
        start.y + handle_length * start_tangent.y};
      const Point2D control2{
        finish.x - handle_length * finish_tangent.x,
        finish.y - handle_length * finish_tangent.y};
      double total_arc_length = 0.0;
      for (std::size_t index = first; index < last; ++index) {
        total_arc_length += point_distance(
          original_waypoints_[index], original_waypoints_[index + 1]);
      }
      double travelled = 0.0;
      for (std::size_t index = first; index <= last; ++index) {
        const double t = total_arc_length > 1e-9 ?
          travelled / total_arc_length :
          static_cast<double>(index - first) / static_cast<double>(last - first);
        const double one_minus_t = 1.0 - t;
        route[index].x =
          one_minus_t * one_minus_t * one_minus_t * start.x +
          3.0 * one_minus_t * one_minus_t * t * control1.x +
          3.0 * one_minus_t * t * t * control2.x + t * t * t * finish.x;
        route[index].y =
          one_minus_t * one_minus_t * one_minus_t * start.y +
          3.0 * one_minus_t * one_minus_t * t * control1.y +
          3.0 * one_minus_t * t * t * control2.y + t * t * t * finish.y;
        if (index < last) {
          travelled += point_distance(
            original_waypoints_[index], original_waypoints_[index + 1]);
        }
      }
    };
  auto requested_offsets_preserved = [&](const std::vector<Waypoint> & route,
      std::size_t first, std::size_t last, std::string & invalid_reason) {
      for (std::size_t index = first; index <= last; ++index) {
        const double requested_offset = std::abs(waypoint_shift_offsets_[index]);
        if (requested_offset < 1e-9) {
          continue;
        }
        Point2D requested_direction;
        if (!normalize_vector(
            waypoint_shift_world_directions_[index], requested_direction))
        {
          Point2D normal;
          if (!original_route_normal_at(index, normal)) {
            invalid_reason = "无法计算偏移量验收方向";
            return false;
          }
          const double sign = waypoint_shift_offsets_[index] > 0.0 ? 1.0 : -1.0;
          requested_direction = Point2D{sign * normal.x, sign * normal.y};
        }
        const Point2D displacement{
          route[index].x - original_waypoints_[index].x,
          route[index].y - original_waypoints_[index].y};
        const double retained_offset =
          displacement.x * requested_direction.x +
          displacement.y * requested_direction.y;
        if (retained_offset + 1e-6 < requested_offset) {
          invalid_reason = "局部曲线削弱了前/后窗口设定的偏移量";
          return false;
        }
      }
      return true;
    };

  std::size_t rebuilt_count = 0;
  // 这些区间已经通过“沿自身接回弦严格前进 + 局部无折返/无自交”检查。
  // 最终守门不能再拿同索引原路线切线否决合法的切弯 Bezier；否则内弯
  // 虽然已经被安全切掉，仍会因为局部切线方向不同而被误判成反相。
  std::vector<std::pair<std::size_t, std::size_t>> chord_validated_spans;
  std::size_t run_first = 0;
  while (run_first < route_size) {
    if (std::abs(waypoint_shift_offsets_[run_first]) < 1e-9) {
      ++run_first;
      continue;
    }
    const int offset_sign = waypoint_shift_offsets_[run_first] > 0.0 ? 1 : -1;
    const std::size_t source_identity = waypoint_shift_source_indices_[run_first];
    std::size_t run_last = run_first;
    while (run_last + 1 < route_size &&
      waypoint_shift_offsets_[run_last + 1] * static_cast<double>(offset_sign) > 1e-9)
    {
      const std::size_t next_source = waypoint_shift_source_indices_[run_last + 1];
      if (source_identity < route_size && next_source < route_size &&
        next_source != source_identity)
      {
        break;
      }
      ++run_last;
    }

    // 风险判定保留逐点粒度：同一障碍物的整个直接偏移窗口中，只有真正
    // 发生曲率退化、前进反相或自交的点才进入后续局部曲线重建。
    const std::size_t run_length = run_last - run_first + 1;
    std::vector<bool> unsafe_points(run_length, false);
    std::vector<double> risk_factors(
      run_length, std::numeric_limits<double>::infinity());
    auto mark_unsafe_point = [&](std::size_t index, double risk_factor) {
        if (index < run_first || index > run_last) {
          return;
        }
        const std::size_t local_index = index - run_first;
        unsafe_points[local_index] = true;
        risk_factors[local_index] = std::min(
          risk_factors[local_index], risk_factor);
      };

    const std::size_t curvature_first = std::max<std::size_t>(1, run_first);
    const std::size_t curvature_last = std::min(run_last, route_size - 2);
    for (std::size_t index = curvature_first; index <= curvature_last; ++index) {
      const Point2D previous_segment{
        original_waypoints_[index].x - original_waypoints_[index - 1].x,
        original_waypoints_[index].y - original_waypoints_[index - 1].y};
      const Point2D next_segment{
        original_waypoints_[index + 1].x - original_waypoints_[index].x,
        original_waypoints_[index + 1].y - original_waypoints_[index].y};
      const double previous_length = std::hypot(previous_segment.x, previous_segment.y);
      const double next_length = std::hypot(next_segment.x, next_segment.y);
      if (previous_length < 1e-9 || next_length < 1e-9) {
        continue;
      }
      const Point2D previous_tangent{
        previous_segment.x / previous_length, previous_segment.y / previous_length};
      const Point2D next_tangent{
        next_segment.x / next_length, next_segment.y / next_length};
      const double turn_angle = std::atan2(
        previous_tangent.x * next_tangent.y -
        previous_tangent.y * next_tangent.x,
        previous_tangent.x * next_tangent.x +
        previous_tangent.y * next_tangent.y);
      const double curvature = turn_angle / (0.5 * (previous_length + next_length));
      const double feasibility =
        1.0 - curvature * waypoint_shift_offsets_[index];
      if (feasibility <= waypoint_shift_rebuild_min_progress_ratio_) {
        mark_unsafe_point(index, feasibility);
      }
    }

    const std::size_t progress_first = run_first > 0 ? run_first - 1 : run_first;
    const std::size_t progress_last = std::min(run_last, route_size - 2);
    for (std::size_t index = progress_first; index <= progress_last; ++index) {
      const Point2D original_segment{
        original_waypoints_[index + 1].x - original_waypoints_[index].x,
        original_waypoints_[index + 1].y - original_waypoints_[index].y};
      const double original_length_squared =
        original_segment.x * original_segment.x +
        original_segment.y * original_segment.y;
      if (original_length_squared < 1e-12) {
        continue;
      }
      const Point2D shifted_segment{
        shifted_route[index + 1].x - shifted_route[index].x,
        shifted_route[index + 1].y - shifted_route[index].y};
      const double progress_ratio =
        (original_segment.x * shifted_segment.x +
        original_segment.y * shifted_segment.y) / original_length_squared;
      if (progress_ratio <= waypoint_shift_rebuild_min_progress_ratio_) {
        // 反相发生在线段上；只标记该线段位于当前偏移窗口内的端点。
        mark_unsafe_point(index, progress_ratio);
        mark_unsafe_point(index + 1, progress_ratio);
      }
    }

    const std::size_t intersection_first = progress_first;
    const std::size_t intersection_last = std::min(run_last + 1, route_size - 1);
    for (std::size_t first_segment = intersection_first;
      first_segment + 1 <= intersection_last; ++first_segment)
    {
      for (std::size_t second_segment = first_segment + 2;
        second_segment + 1 <= intersection_last; ++second_segment)
      {
        if (!proper_segments_intersect(
            shifted_route[first_segment], shifted_route[first_segment + 1],
            shifted_route[second_segment], shifted_route[second_segment + 1]))
        {
          continue;
        }
        // 两条不相邻线段相交时，中间这段构成局部回环；仅标记回环内部点。
        const std::size_t loop_first = std::max(run_first, first_segment + 1);
        const std::size_t loop_last = std::min(run_last, second_segment);
        for (std::size_t index = loop_first; index <= loop_last; ++index) {
          mark_unsafe_point(index, 0.0);
        }
      }
    }

    std::size_t unsafe_first = run_first;
    while (unsafe_first <= run_last) {
      while (unsafe_first <= run_last &&
        !unsafe_points[unsafe_first - run_first])
      {
        ++unsafe_first;
      }
      if (unsafe_first > run_last) {
        break;
      }
      std::size_t unsafe_last = unsafe_first;
      while (unsafe_last < run_last &&
        unsafe_points[unsafe_last + 1 - run_first])
      {
        ++unsafe_last;
      }

      double minimum_risk_factor = std::numeric_limits<double>::infinity();
      std::size_t risk_index = unsafe_first;
      for (std::size_t index = unsafe_first; index <= unsafe_last; ++index) {
        const double risk_factor = risk_factors[index - run_first];
        if (risk_factor < minimum_risk_factor) {
          minimum_risk_factor = risk_factor;
          risk_index = index;
        }
      }

      // B前的轻微圆滑已经是偏移前基准路线的一部分。这一小段若再进入
      // 偏移后曲率兜底，就会破坏1/2—1/2直接偏移分配；仅跳过与B重叠的
      // 危险子段，不再连带跳过或重建同一障碍窗口的其他点。
      const std::size_t repair_context_first =
        unsafe_first > 0 ? unsafe_first - 1 : unsafe_first;
      const std::size_t repair_context_last =
        std::min(unsafe_last + 1, route_size - 1);
      const bool overlaps_pre_shift_entry_smoothing =
        channel_entry_waypoint_index_ > 0 &&
        channel_entry_smoothing_first_index_ < channel_entry_waypoint_index_ &&
        repair_context_first <= channel_entry_waypoint_index_ &&
        repair_context_last >= channel_entry_smoothing_first_index_;
      if (overlaps_pre_shift_entry_smoothing) {
        RCLCPP_INFO(
          get_logger(),
          "waypoint_shift：危险子段点%zu-%zu与B前基准圆滑区点%zu-%zu重叠；"
          "保留该处直接偏移，跳过偏移后局部曲线重构。",
          unsafe_first + 1, unsafe_last + 1,
          channel_entry_smoothing_first_index_ + 1,
          channel_entry_waypoint_index_ + 1);
        unsafe_first = unsafe_last + 1;
        continue;
      }

      // 只处理真正发生反相/自交的危险点。真实障碍来源点若不在危险子段
      // 内，不能为了让曲线经过它而把整段前/后偏移窗口一起拉成大圆弧。
      const std::size_t source_index =
        source_identity >= unsafe_first && source_identity <= unsafe_last ?
        source_identity : risk_index;
      Point2D fixed_direction = waypoint_shift_world_directions_[source_index];
      if (!normalize_vector(fixed_direction, fixed_direction)) {
        fixed_direction = Point2D();
        for (std::size_t index = unsafe_first; index <= unsafe_last; ++index) {
          fixed_direction.x += waypoint_shift_world_directions_[index].x;
          fixed_direction.y += waypoint_shift_world_directions_[index].y;
        }
        if (!normalize_vector(fixed_direction, fixed_direction)) {
          Point2D normal;
          if (!original_route_normal_at(source_index, normal)) {
            unsafe_first = unsafe_last + 1;
            continue;
          }
          fixed_direction = Point2D{
            static_cast<double>(offset_sign) * normal.x,
            static_cast<double>(offset_sign) * normal.y};
        }
      }
      double target_offset = 0.0;
      for (std::size_t index = unsafe_first; index <= unsafe_last; ++index) {
        target_offset = std::max(
          target_offset, std::abs(waypoint_shift_offsets_[index]));
      }
      if (target_offset < 1e-9) {
        unsafe_first = unsafe_last + 1;
        continue;
      }

      // 两个接回锚点严格取危险子段的相邻点：锚点不改写，Bezier只覆盖
      // unsafe_first..unsafe_last。若相邻点不足或这一最小局部桥不可行，
      // 就保留原始直接偏移；不再向障碍中心或窗口外扩展几十个路径点。
      const std::size_t repair_core_first = unsafe_first;
      const std::size_t repair_core_last = unsafe_last;
      const std::size_t anchor_search_first =
        repair_core_first > 0 ? repair_core_first - 1 : repair_core_first;
      const std::size_t anchor_search_last =
        std::min(repair_core_last + 1, route_size - 1);
      const std::size_t maximum_context = std::max(
        repair_core_first - anchor_search_first,
        anchor_search_last - repair_core_last);
      std::size_t accepted_anchor_first = unsafe_first;
      std::size_t accepted_anchor_last = unsafe_last;
      double accepted_handle_ratio = 0.0;
      double accepted_clearance = target_offset;
      const char * accepted_bridge_mode = "单段切角桥";
      std::string last_invalid_reason = "没有可用的双侧接回锚点";
      bool valid_rebuild = false;
      const double handle_ratios[] = {
        waypoint_shift_rebuild_bezier_handle_ratio_,
        0.75 * waypoint_shift_rebuild_bezier_handle_ratio_,
        0.50 * waypoint_shift_rebuild_bezier_handle_ratio_,
        0.25 * waypoint_shift_rebuild_bezier_handle_ratio_, 0.0};
      std::vector<Waypoint> accepted_candidate;
      std::size_t previous_anchor_first = route_size;
      std::size_t previous_anchor_last = route_size;
      for (std::size_t context = 1;
        context <= maximum_context && !valid_rebuild; ++context)
      {
        const std::size_t anchor_first =
          repair_core_first > anchor_search_first + context ?
          repair_core_first - context : anchor_search_first;
        const std::size_t anchor_last =
          std::min(anchor_search_last, repair_core_last + context);
        if (anchor_first == previous_anchor_first &&
          anchor_last == previous_anchor_last)
        {
          continue;
        }
        previous_anchor_first = anchor_first;
        previous_anchor_last = anchor_last;
        if (anchor_first >= repair_core_first || anchor_last <= repair_core_last ||
          source_index <= anchor_first || source_index >= anchor_last)
        {
          continue;
        }

        const Point2D start_point{
          shifted_route[anchor_first].x, shifted_route[anchor_first].y};
        const Point2D finish_point{
          shifted_route[anchor_last].x, shifted_route[anchor_last].y};
        double original_span_length = 0.0;
        for (std::size_t index = anchor_first; index < anchor_last; ++index) {
          original_span_length += point_distance(
            original_waypoints_[index], original_waypoints_[index + 1]);
        }
        const double bridge_chord_length = std::hypot(
          finish_point.x - start_point.x, finish_point.y - start_point.y);
        if (original_span_length < 1e-9 ||
          bridge_chord_length / original_span_length <=
          waypoint_shift_rebuild_min_progress_ratio_)
        {
          last_invalid_reason = "Bezier桥弦长过短，会把危险圆弧压缩成近似原地转向";
          continue;
        }
        Point2D start_tangent;
        Point2D finish_tangent;
        Point2D source_tangent;
        if (!route_tangent_at(original_waypoints_, anchor_first, start_tangent) ||
          !route_tangent_at(original_waypoints_, anchor_last, finish_tangent) ||
          !route_tangent_at(original_waypoints_, source_index, source_tangent))
        {
          last_invalid_reason = "无法计算接回切向";
          continue;
        }

        for (const double handle_ratio : handle_ratios) {
          std::vector<Waypoint> rebuilt_candidate = shifted_route;
          fill_bezier(
            rebuilt_candidate, anchor_first, anchor_last,
            start_point, finish_point, start_tangent, finish_tangent,
            handle_ratio);
          const Point2D source_displacement{
            rebuilt_candidate[source_index].x - original_waypoints_[source_index].x,
            rebuilt_candidate[source_index].y - original_waypoints_[source_index].y};
          const double source_clearance =
            source_displacement.x * fixed_direction.x +
            source_displacement.y * fixed_direction.y;
          std::string candidate_invalid_reason;
          const bool single_bridge_geometry_valid =
            chord_progress_valid(
              rebuilt_candidate, anchor_first, anchor_last,
              start_point, finish_point, candidate_invalid_reason) &&
            local_geometry_valid(
              rebuilt_candidate, anchor_first, anchor_last,
              candidate_invalid_reason) &&
            requested_offsets_preserved(
              rebuilt_candidate, repair_core_first, repair_core_last,
              candidate_invalid_reason);
          if (single_bridge_geometry_valid)
          {
            valid_rebuild = true;
            accepted_anchor_first = anchor_first;
            accepted_anchor_last = anchor_last;
            accepted_handle_ratio = handle_ratio;
            accepted_clearance = source_clearance;
            accepted_bridge_mode = "单段切角桥";
            accepted_candidate = std::move(rebuilt_candidate);
            break;
          }
          if (!single_bridge_geometry_valid) {
            last_invalid_reason = candidate_invalid_reason;
          }

          // 外弯的直接等距曲线本身不会发生曲率坍缩；单段切弦反而可能
          // 向内吃掉已经满足的净空。此时保留来源索引处的完整直接偏移点，
          // 只用两段Bezier消除局部折线/自交。内弯若使用这个中点会再次
          // 折返，下面的双弦单调、转角和自交检查会把它拒绝。
          const Point2D shifted_source{
            shifted_route[source_index].x, shifted_route[source_index].y};
          rebuilt_candidate = shifted_route;
          fill_bezier(
            rebuilt_candidate, anchor_first, source_index,
            start_point, shifted_source, start_tangent, source_tangent,
            handle_ratio);
          fill_bezier(
            rebuilt_candidate, source_index, anchor_last,
            shifted_source, finish_point, source_tangent, finish_tangent,
            handle_ratio);
          if (chord_progress_valid(
              rebuilt_candidate, anchor_first, source_index,
              start_point, shifted_source, last_invalid_reason) &&
            chord_progress_valid(
              rebuilt_candidate, source_index, anchor_last,
              shifted_source, finish_point, last_invalid_reason) &&
            local_geometry_valid(
              rebuilt_candidate, anchor_first, anchor_last, last_invalid_reason) &&
            requested_offsets_preserved(
              rebuilt_candidate, repair_core_first, repair_core_last,
              last_invalid_reason))
          {
            valid_rebuild = true;
            accepted_anchor_first = anchor_first;
            accepted_anchor_last = anchor_last;
            accepted_handle_ratio = handle_ratio;
            accepted_clearance = std::abs(waypoint_shift_offsets_[source_index]);
            accepted_bridge_mode = "双段外弯保净空桥";
            accepted_candidate = std::move(rebuilt_candidate);
            break;
          }
        }
      }
      // 最小局部桥若失败，原始直接偏移保持不动。允许巡点逻辑和到点容限
      // 消化这一处局部反相/自交，绝不扩大曲线范围或恢复原路线。
      if (!valid_rebuild) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "waypoint_shift：危险子段%zu-%zu在所属直接偏移窗口%zu-%zu内仍无法"
          "生成保持完整净距、严格前进且无自交的独立同侧曲线（最后原因：%s）；"
          "保留前/后窗口的原始完整偏移及该处局部反相/自交，不扩大融合、不回退原路线。",
          unsafe_first + 1, unsafe_last + 1, run_first + 1, run_last + 1,
          last_invalid_reason.c_str());
        unsafe_first = unsafe_last + 1;
        continue;
      }
      shifted_route = std::move(accepted_candidate);
      chord_validated_spans.emplace_back(
        accepted_anchor_first, accepted_anchor_last);
      ++rebuilt_count;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "waypoint_shift：检测到圆弧内侧偏移不可行（危险点%zu-%zu，"
        "所属直接偏移窗口%zu-%zu，最小进度/可行因子=%.3f），"
        "仅在点%zu-%zu内用%s独立重建危险圆弧，来源点=%zu，"
        "危险段最大设定偏移/来源点实际偏移=%.3f/%.3fm，"
        "且危险段逐点偏移验收通过；Bezier控制柄比例=%.3f。",
        unsafe_first + 1, unsafe_last + 1, run_first + 1, run_last + 1,
        minimum_risk_factor, accepted_anchor_first + 1, accepted_anchor_last + 1,
        accepted_bridge_mode, source_index + 1, target_offset,
        accepted_clearance, accepted_handle_ratio);
      unsafe_first = unsafe_last + 1;
    }
    run_first = run_last + 1;
  }

  // 最终整条路线只做诊断：上面的最小局部算法已经尝试处理反向、折返和
  // 自交；若仍有问题，按要求保留完整直接偏移和局部几何，不再扩大范围。
  auto route_safety_issue = [&](const std::vector<Waypoint> & route,
      std::vector<std::size_t> & involved_indices, std::string & reason) {
      involved_indices.clear();
      for (std::size_t index = 0; index + 1 < route_size; ++index) {
        const bool belongs_to_validated_chord = std::any_of(
          chord_validated_spans.begin(), chord_validated_spans.end(),
          [&](const auto & span) {
            return index >= span.first && index + 1 <= span.second;
          });
        if (belongs_to_validated_chord) {
          // 该段已经按Bezier自身的起终弦验证为严格前进。切弯后局部切线
          // 本来就可能不同于同索引原圆弧切线，不能重复按原切线判反相。
          continue;
        }
        const Point2D original_segment{
          original_waypoints_[index + 1].x - original_waypoints_[index].x,
          original_waypoints_[index + 1].y - original_waypoints_[index].y};
        const double original_length_squared =
          original_segment.x * original_segment.x +
          original_segment.y * original_segment.y;
        if (original_length_squared < 1e-12) {
          continue;
        }
        const Point2D shifted_segment{
          route[index + 1].x - route[index].x,
          route[index + 1].y - route[index].y};
        const double progress_ratio =
          (shifted_segment.x * original_segment.x +
          shifted_segment.y * original_segment.y) / original_length_squared;
        if (progress_ratio < -0.05) {
          involved_indices = {index, index + 1};
          reason = "出现相对原路线明确反向的线段";
          return true;
        }
      }
      for (std::size_t index = 1; index + 1 < route_size; ++index) {
        const Point2D incoming{
          route[index].x - route[index - 1].x,
          route[index].y - route[index - 1].y};
        const Point2D outgoing{
          route[index + 1].x - route[index].x,
          route[index + 1].y - route[index].y};
        const double incoming_length = std::hypot(incoming.x, incoming.y);
        const double outgoing_length = std::hypot(outgoing.x, outgoing.y);
        if (incoming_length < 1e-9 || outgoing_length < 1e-9) {
          continue;
        }
        const double turn_cosine =
          (incoming.x * outgoing.x + incoming.y * outgoing.y) /
          (incoming_length * outgoing_length);
        if (turn_cosine >= -0.25) {
          continue;
        }
        const Point2D original_incoming{
          original_waypoints_[index].x - original_waypoints_[index - 1].x,
          original_waypoints_[index].y - original_waypoints_[index - 1].y};
        const Point2D original_outgoing{
          original_waypoints_[index + 1].x - original_waypoints_[index].x,
          original_waypoints_[index + 1].y - original_waypoints_[index].y};
        const double original_denominator =
          std::hypot(original_incoming.x, original_incoming.y) *
          std::hypot(original_outgoing.x, original_outgoing.y);
        const double original_turn_cosine = original_denominator > 1e-12 ?
          (original_incoming.x * original_outgoing.x +
          original_incoming.y * original_outgoing.y) / original_denominator : 1.0;
        if (original_turn_cosine >= -0.25) {
          involved_indices = {index - 1, index, index + 1};
          reason = "静态偏移新引入急剧折返";
          return true;
        }
      }
      for (std::size_t first_segment = 0;
        first_segment + 1 < route_size; ++first_segment)
      {
        for (std::size_t second_segment = first_segment + 2;
          second_segment + 1 < route_size; ++second_segment)
        {
          if (!proper_segments_intersect(
              route[first_segment], route[first_segment + 1],
              route[second_segment], route[second_segment + 1]) ||
            proper_segments_intersect(
              original_waypoints_[first_segment],
              original_waypoints_[first_segment + 1],
              original_waypoints_[second_segment],
              original_waypoints_[second_segment + 1]))
          {
            continue;
          }
          involved_indices = {
            first_segment, first_segment + 1,
            second_segment, second_segment + 1};
          reason = "静态偏移新引入路线自交";
          return true;
        }
      }
      reason.clear();
      return false;
    };

  std::vector<std::size_t> unsafe_final_indices;
  std::string final_invalid_reason;
  if (route_safety_issue(
      shifted_route, unsafe_final_indices, final_invalid_reason))
  {
    const auto unsafe_bounds = std::minmax_element(
      unsafe_final_indices.begin(), unsafe_final_indices.end());
    if (!logic3_ready_preplanning_active_) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "waypoint_shift最终检查仍发现%s（涉及点%zu-%zu）；"
        "局部最小重建已失败或不适用，按要求保留前/后窗口的全部原始偏移"
        "和该处局部几何，不扩大融合、不降低偏移量、不恢复原路线。",
        final_invalid_reason.c_str(),
        *unsafe_bounds.first + 1, *unsafe_bounds.second + 1);
    }
  }
  return rebuilt_count;
}

void VisualLidarPathFollower::rebuild_route_from_confirmed_shifts()
{
  if (original_waypoints_.size() != waypoint_shift_offsets_.size()) {
    return;
  }
  waypoints_ = original_waypoints_;
  if (route_deformation_mode_ == "circle_envelope") {
    apply_circle_envelope_deformation(waypoints_);
  } else {
    for (std::size_t index = 0; index < waypoints_.size(); ++index) {
      if (std::abs(waypoint_shift_offsets_[index]) < 1e-9) {
        continue;
      }
      Point2D normal;
      if (!original_route_normal_at(index, normal)) {
        continue;
      }
      waypoints_[index].x += waypoint_shift_offsets_[index] * normal.x;
      waypoints_[index].y += waypoint_shift_offsets_[index] * normal.y;
    }
    rebuild_curvature_unsafe_shift_segments(waypoints_);
  }

  if (following_qr_path_ || task_phase_ == TaskPhase::TASK1) {
    for (auto & waypoint : waypoints_) {
      waypoint.y = std::max(waypoint.y, task1_waypoint_shift_min_y_);
    }
  }
  if (channel_entry_waypoint_index_ > 0 && !waypoints_.empty()) {
    waypoints_.front() = original_waypoints_.front();
  }
  validate_logic3_recorded_obstacle_route_clearance();
  rebuild_path_pid_route_cache();
  reset_path_pid_state();
}

void VisualLidarPathFollower::validate_logic3_recorded_obstacle_route_clearance()
{
  if (route_deformation_mode_ != "circle_envelope" || waypoints_.size() < 2 ||
    waypoint_shift_offsets_.size() != waypoints_.size() ||
    waypoint_shift_obstacle_centers_.size() != waypoints_.size() ||
    waypoint_shift_forbidden_radii_.size() != waypoints_.size())
  {
    return;
  }

  struct RecordedObstacleRouteCandidate
  {
    Point2D center;
    double forbidden_radius{0.0};
    std::size_t first_segment{0};
  };
  auto obstacle_first_segment = [&](const Point2D & center) {
      if (!channel_entry_rear_static_filter_active_) {
        return std::size_t{0};
      }
      const double relative_x =
        center.x - channel_entry_rear_static_filter_start_x_;
      const double relative_y =
        center.y - channel_entry_rear_static_filter_start_y_;
      const double forward_projection =
        relative_x * std::cos(channel_entry_rear_static_filter_start_yaw_) +
        relative_y * std::sin(channel_entry_rear_static_filter_start_yaw_);
      if (forward_projection >= 0.0) {
        return std::size_t{0};
      }
      return std::min(
        channel_entry_rear_static_filter_first_index_,
        waypoints_.size() - 1);
    };
  std::vector<RecordedObstacleRouteCandidate> candidates;
  for (std::size_t index = 0; index < waypoints_.size(); ++index) {
    if (std::abs(waypoint_shift_offsets_[index]) < 1.0e-9 ||
      waypoint_shift_forbidden_radii_[index] <= 1.0e-9)
    {
      continue;
    }
    const auto & center = waypoint_shift_obstacle_centers_[index];
    const std::size_t first_segment = obstacle_first_segment(center);
    auto existing = std::find_if(
      candidates.begin(), candidates.end(),
      [&](const RecordedObstacleRouteCandidate & candidate) {
        return std::hypot(
          candidate.center.x - center.x,
          candidate.center.y - center.y) <= 0.05;
      });
    if (existing == candidates.end()) {
      candidates.push_back(
        RecordedObstacleRouteCandidate{
          center, waypoint_shift_forbidden_radii_[index], first_segment});
    } else {
      existing->forbidden_radius = std::max(
        existing->forbidden_radius,
        waypoint_shift_forbidden_radii_[index]);
      existing->first_segment = std::max(
        existing->first_segment, first_segment);
    }
  }
  if (logic3_planning_pose_anchored_)
  {
    const double permanent_forbidden_radius =
      circle_envelope_target_clearance_ + circle_envelope_numeric_margin_;
    for (std::size_t obstacle_index = 0;
      obstacle_index < logic3_green_clusters_.size(); ++obstacle_index)
    {
      const bool dynamic_takeover =
        obstacle_index < logic3_green_dynamic_takeover_.size() &&
        logic3_green_dynamic_takeover_[obstacle_index];
      if (dynamic_takeover) {
        continue;
      }
      const auto & center = logic3_green_clusters_[obstacle_index].center;
      const std::size_t first_segment = obstacle_first_segment(center);
      auto existing = std::find_if(
        candidates.begin(), candidates.end(),
        [&](const RecordedObstacleRouteCandidate & candidate) {
          return std::hypot(
            candidate.center.x - center.x,
            candidate.center.y - center.y) <= 0.05;
        });
      if (existing == candidates.end()) {
        candidates.push_back(
          RecordedObstacleRouteCandidate{
            center, permanent_forbidden_radius, first_segment});
      } else {
        existing->forbidden_radius = std::max(
          existing->forbidden_radius, permanent_forbidden_radius);
        existing->first_segment = std::max(
          existing->first_segment, first_segment);
      }
    }
  }

  auto closest_point_on_segment = [](const Point2D & point,
      const Waypoint & start, const Waypoint & finish) {
      const double segment_x = finish.x - start.x;
      const double segment_y = finish.y - start.y;
      const double length_squared =
        segment_x * segment_x + segment_y * segment_y;
      if (length_squared < 1.0e-12) {
        return Point2D{start.x, start.y};
      }
      const double t = std::clamp(
        ((point.x - start.x) * segment_x +
        (point.y - start.y) * segment_y) / length_squared,
        0.0, 1.0);
      return Point2D{
        start.x + t * segment_x,
        start.y + t * segment_y};
    };

  for (std::size_t candidate_index = 0;
    candidate_index < candidates.size(); ++candidate_index)
  {
    const auto & candidate = candidates[candidate_index];
    double route_distance = std::numeric_limits<double>::infinity();
    Point2D closest_route_point;
    for (std::size_t index = candidate.first_segment;
      index + 1 < waypoints_.size(); ++index)
    {
      const Point2D closest = closest_point_on_segment(
        candidate.center, waypoints_[index], waypoints_[index + 1]);
      const double distance = std::hypot(
        closest.x - candidate.center.x,
        closest.y - candidate.center.y);
      if (distance < route_distance) {
        route_distance = distance;
        closest_route_point = closest;
      }
    }
    // 路线净距在apply_circle_envelope_deformation中已做硬验收；未通过
    // target_clearance的ID会解除绿/橙掩码并交给实时雷达。这里保留第二份
    // 诊断日志，不再修改路线或覆盖前面的失败标记。
    const double required_route_distance = std::max(
      0.0,
      candidate.forbidden_radius - circle_envelope_numeric_margin_);
    if (route_distance + 1.0e-6 >= required_route_distance) {
      continue;
    }

    bool intentional_overlap_corridor = false;
    if (route_deformation_mode_ == "circle_envelope") {
      for (std::size_t partner_index = 0;
        partner_index < candidates.size(); ++partner_index)
      {
        if (partner_index == candidate_index) {
          continue;
        }
        const auto & partner = candidates[partner_index];
        CircleIntersectionChord chord;
        if (circle_intersection_chord(
            candidate.center.x, candidate.center.y,
            candidate.forbidden_radius,
            partner.center.x, partner.center.y,
            partner.forbidden_radius,
            chord) &&
          point_near_chord_segment(
            closest_route_point.x, closest_route_point.y, chord,
            std::max(0.02, 0.5 * circle_envelope_numeric_margin_)))
        {
          intentional_overlap_corridor = true;
          break;
        }
      }
    }
    if (intentional_overlap_corridor) {
      continue;
    }
    if (!logic3_ready_preplanning_active_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "circle_envelope路线对已记录障碍(%.3f, %.3f)的最近距离%.3fm小于要求%.3fm；"
        "仅标记路线净距异常，不修改该永久点及当前控制器的障碍处理策略。",
        candidate.center.x, candidate.center.y,
        route_distance, required_route_distance);
    }
  }
}

// 把已确认的点云障碍转换成“固定安全圆 + 圆滑接线”。对每个障碍只做三件事：
// 1. 使用waypoint_shift已经确定的绕行方向，后面不得翻转；
// 2. 原路线被以circle_envelope_target_clearance为半径的圆覆盖时，
//    将整段路线限制在指定侧的圆边界外，不再只锁一个障碍锚点；
// 3. 从未偏移的original_waypoints_同时生成所有接线并整体圆滑。
// 两个安全圆相交时不合并障碍；若相反方向的圆约束已无法同时满足，
// 使用含numeric_margin的两个规划圆交点形成公共弦，并圆滑连接弦的两端。
void VisualLidarPathFollower::apply_circle_envelope_deformation(
  std::vector<Waypoint> & shifted_route)
{
  const std::size_t route_size = original_waypoints_.size();
  if (route_size < 3 || shifted_route.size() != route_size ||
    waypoint_shift_offsets_.size() != route_size ||
    waypoint_shift_world_directions_.size() != route_size ||
    waypoint_shift_source_indices_.size() != route_size ||
    waypoint_shift_obstacle_centers_.size() != route_size ||
    waypoint_shift_forbidden_radii_.size() != route_size)
  {
    return;
  }

  // 每次静态路线重算都重新验收全部永久点。最终仍不合格的ID会在本函数
  // 末尾单独标记并临时解除雷达掩码，不能沿用上一版路线的失败状态。
  logic3_green_route_clearance_failed_.assign(
    logic3_green_clusters_.size(), false);

  std::vector<double> route_s(route_size, 0.0);
  std::vector<Point2D> left_normals(route_size, Point2D());
  std::vector<bool> normal_valid(route_size, false);
  for (std::size_t index = 1; index < route_size; ++index) {
    route_s[index] = route_s[index - 1] + std::hypot(
      original_waypoints_[index].x - original_waypoints_[index - 1].x,
      original_waypoints_[index].y - original_waypoints_[index - 1].y);
  }
  for (std::size_t index = 0; index < route_size; ++index) {
    normal_valid[index] = original_route_normal_at(index, left_normals[index]);
  }

  // A→C→B已经在original_waypoints_中接到Task2首点B之前。静态偏移必须
  // 把这条组合路线视为一个整体，允许B附近障碍的before窗口自然跨过B，
  // 不能因为过渡段同时启用实时雷达就把A→C→B排除在可偏移范围之外。
  const std::size_t first_deformable_index = 0;
  const std::size_t last_deformable_index = route_size - 1;
  if (first_deformable_index > last_deformable_index) {
    return;
  }

  // 仅二维码倒车完成后的A->C->B重建会启用本过滤：用当时A沿yaw前移
  // 配置距离后的虚拟参考点和原yaw冻结一次车前/车后关系。车后永久点仍可
  // 作用于起始保护长度之后的路线，但circle_envelope接线和二次净距修复
  // 不得越过同一轮保存的最早可偏移索引，避免倒灌到A点附近。
  auto obstacle_first_deformable_index = [&](const Point2D & center) {
      if (!channel_entry_rear_static_filter_active_) {
        return first_deformable_index;
      }
      const double relative_x =
        center.x - channel_entry_rear_static_filter_start_x_;
      const double relative_y =
        center.y - channel_entry_rear_static_filter_start_y_;
      const double forward_projection =
        relative_x * std::cos(channel_entry_rear_static_filter_start_yaw_) +
        relative_y * std::sin(channel_entry_rear_static_filter_start_yaw_);
      if (forward_projection >= 0.0) {
        return first_deformable_index;
      }
      return std::clamp(
        channel_entry_rear_static_filter_first_index_,
        first_deformable_index, last_deformable_index);
    };

  struct CircleConstraint
  {
    std::size_t source{0};
    int direction{0};
    Point2D center;
    double radius{0.0};
    double peak_offset{0.0};
    double transition_extension{0.0};
    // 第一次路线生成完后，若某个此前未覆盖的永久绿/橙点被新路线侵入，
    // 只允许在真正侵圆的连续索引段使用“原路线法线与圆求交”补充边界。
    // 普通约束仍坚持原航点本身必须在圆内，不能退回无限长法线全局约束。
    bool secondary_repair{false};
    std::size_t repair_first{0};
    std::size_t repair_last{0};
    // 单个障碍允许影响的最早路线索引。通常为0；本轮入口A点车后的
    // 永久点从A后的保护长度边界开始，防止圆滑接线越过车前/车后过滤边界。
    std::size_t first_deformable{0};
    // 正反两次经过同一物理通道时，整个圆滑接入区间都必须保持已经选定的
    // 路线相对侧。只锁圆心附近的硬边界会让其他障碍的长接线尾巴在水平
    // 转弯段把符号平均翻转，出现“竖直段右绕、水平段反而向上”的形状。
    bool transition_side_locked{false};
  };
  std::vector<CircleConstraint> constraints;
  for (std::size_t index = 0; index < route_size; ++index) {
    if (std::abs(waypoint_shift_offsets_[index]) < 1.0e-9 ||
      waypoint_shift_forbidden_radii_[index] <= 1.0e-9 ||
      waypoint_shift_source_indices_[index] >= route_size)
    {
      continue;
    }

    const std::size_t source = waypoint_shift_source_indices_[index];
    if (source < first_deformable_index || source > last_deformable_index ||
      !normal_valid[source])
    {
      continue;
    }

    // 优先使用障碍物投影到原路线时保存的世界坐标绕行方向，再换算成
    // source处原路线左法向的正负号。后续任何圆滑和净距修正都不得改号。
    int direction = waypoint_shift_offsets_[index] > 0.0 ? 1 : -1;
    const auto & world_direction = waypoint_shift_world_directions_[index];
    const double world_direction_length =
      std::hypot(world_direction.x, world_direction.y);
    if (world_direction_length > 1.0e-9) {
      const double alignment =
        (world_direction.x * left_normals[source].x +
        world_direction.y * left_normals[source].y) / world_direction_length;
      if (std::abs(alignment) > 0.2) {
        direction = alignment > 0.0 ? 1 : -1;
      }
    }
    const auto & center = waypoint_shift_obstacle_centers_[index];
    const std::size_t constraint_first_deformable =
      obstacle_first_deformable_index(center);
    if (source < constraint_first_deformable) {
      continue;
    }
    auto existing = std::find_if(
      constraints.begin(), constraints.end(),
      [&](const CircleConstraint & candidate) {
        // 这里只去除同一障碍物、同一路线锚点的重复元数据；不同中心或
        // 不同路线经过位置始终保留为独立约束，绝不合并相邻障碍物。
        return candidate.source == source &&
               std::hypot(
          candidate.center.x - center.x,
          candidate.center.y - center.y) <= 0.02;
      });
    if (existing == constraints.end()) {
      constraints.push_back(
        CircleConstraint{
          source, direction, center,
          waypoint_shift_forbidden_radii_[index],
          std::abs(waypoint_shift_offsets_[index]), 0.0,
          false, 0, 0, constraint_first_deformable, false});
    } else {
      existing->radius = std::max(
        existing->radius, waypoint_shift_forbidden_radii_[index]);
      existing->peak_offset = std::max(
        existing->peak_offset, std::abs(waypoint_shift_offsets_[index]));
      existing->first_deformable = std::max(
        existing->first_deformable, constraint_first_deformable);
    }
  }
  if (constraints.empty()) {
    return;
  }
  std::sort(
    constraints.begin(), constraints.end(),
    [](const CircleConstraint & first, const CircleConstraint & second) {
      if (first.source != second.source) {
        return first.source < second.source;
      }
      if (first.center.x != second.center.x) {
        return first.center.x < second.center.x;
      }
      return first.center.y < second.center.y;
    });

  // 同一物理通道可能在路线序列中正向、反向各经过一次。同一个障碍会因此
  // 生成两个source相距很远、但世界坐标几乎重合且法向相反的独立约束。
  // 两次经过仍各自在完整路线s轴上连续画接线，只统一障碍的世界绕行侧、
  // 半径和偏移峰值；不能在偏移完成后强制同步坐标，否则会在同步边界造尖峰。
  // 这里只配对“同中心 + 同位置 + 反向共线”的重复经过；不同障碍、普通
  // 交叉点以及同方向近邻路线都不参与，仍保持原来的独立约束语义。
  struct SharedPassGeometry
  {
    std::size_t first_source{0};
    std::size_t second_source{0};
    Point2D center;
    int first_direction{0};
    int second_direction{0};
    std::vector<std::pair<std::size_t, std::size_t>> waypoint_pairs;
  };
  std::vector<SharedPassGeometry> shared_pass_geometries;
  std::size_t shared_pass_waypoint_pair_count = 0;
  constexpr double shared_center_tolerance = 0.03;
  constexpr double shared_source_tolerance = 0.05;
  constexpr double shared_waypoint_tolerance = 0.015;
  constexpr double minimum_shared_pass_length = 0.20;
  constexpr double opposite_normal_cosine = -0.80;
  for (std::size_t first_index = 0;
    first_index < constraints.size(); ++first_index)
  {
    for (std::size_t second_index = first_index + 1;
      second_index < constraints.size(); ++second_index)
    {
      auto & first = constraints[first_index];
      auto & second = constraints[second_index];
      if (std::hypot(
          first.center.x - second.center.x,
          first.center.y - second.center.y) > shared_center_tolerance)
      {
        continue;
      }
      const auto & first_source = original_waypoints_[first.source];
      const auto & second_source = original_waypoints_[second.source];
      if (std::hypot(
          first_source.x - second_source.x,
          first_source.y - second_source.y) > shared_source_tolerance)
      {
        continue;
      }
      const double normal_alignment =
        left_normals[first.source].x * left_normals[second.source].x +
        left_normals[first.source].y * left_normals[second.source].y;
      if (normal_alignment > opposite_normal_cosine) {
        continue;
      }

      std::vector<std::pair<std::size_t, std::size_t>> shared_waypoint_pairs;
      shared_waypoint_pairs.emplace_back(first.source, second.source);
      for (std::size_t step = 1;
        first.source + step < route_size && second.source >= step; ++step)
      {
        const std::size_t first_waypoint = first.source + step;
        const std::size_t second_waypoint = second.source - step;
        if (std::hypot(
            original_waypoints_[first_waypoint].x -
            original_waypoints_[second_waypoint].x,
            original_waypoints_[first_waypoint].y -
            original_waypoints_[second_waypoint].y) > shared_waypoint_tolerance)
        {
          break;
        }
        shared_waypoint_pairs.emplace_back(first_waypoint, second_waypoint);
      }
      for (std::size_t step = 1;
        first.source >= step && second.source + step < route_size; ++step)
      {
        const std::size_t first_waypoint = first.source - step;
        const std::size_t second_waypoint = second.source + step;
        if (std::hypot(
            original_waypoints_[first_waypoint].x -
            original_waypoints_[second_waypoint].x,
            original_waypoints_[first_waypoint].y -
            original_waypoints_[second_waypoint].y) > shared_waypoint_tolerance)
        {
          break;
        }
        shared_waypoint_pairs.emplace_back(first_waypoint, second_waypoint);
      }
      // 单点交叉或B/C附近很短的相向接线不属于共用通道。先保留最少
      // 3个连续匹配点的离散门槛，下面还会用实际弧长和两段各自已经
      // 判定的世界绕行方向复核，防止入口段把返回段方向覆盖掉。
      if (shared_waypoint_pairs.size() < 3) {
        continue;
      }

      std::sort(
        shared_waypoint_pairs.begin(), shared_waypoint_pairs.end(),
        [](const auto & lhs, const auto & rhs) {
          return lhs.first < rhs.first;
        });
      shared_waypoint_pairs.erase(
        std::unique(
          shared_waypoint_pairs.begin(), shared_waypoint_pairs.end()),
        shared_waypoint_pairs.end());

      const double first_shared_length =
        route_s[shared_waypoint_pairs.back().first] -
        route_s[shared_waypoint_pairs.front().first];
      std::size_t minimum_second_waypoint = route_size - 1;
      std::size_t maximum_second_waypoint = 0;
      for (const auto & pair : shared_waypoint_pairs) {
        minimum_second_waypoint = std::min(
          minimum_second_waypoint, pair.second);
        maximum_second_waypoint = std::max(
          maximum_second_waypoint, pair.second);
      }
      const double second_shared_length =
        route_s[maximum_second_waypoint] - route_s[minimum_second_waypoint];
      if (first_shared_length + 1.0e-9 < minimum_shared_pass_length ||
        second_shared_length + 1.0e-9 < minimum_shared_pass_length)
      {
        continue;
      }

      const Point2D first_world_direction{
        static_cast<double>(first.direction) * left_normals[first.source].x,
        static_cast<double>(first.direction) * left_normals[first.source].y};
      // “同中心、同世界位置、反向共线且共享至少0.20m”的几何已经足以
      // 证明这是同一物理通道。中心几乎压在线上时，两次独立判边可能因
      // 毫米级误差得出相反结果；不能再把这个相反结果当成拒绝配对的理由。
      // 使用路线序列中较早一遍已经确定的世界侧作为本次发车的规范方向，
      // 另一遍换算成自己的局部法向符号。这样既不复制最终坐标，也不会
      // 出现一遍右绕、一遍穿心或左绕。
      const double second_alignment =
        first_world_direction.x * left_normals[second.source].x +
        first_world_direction.y * left_normals[second.source].y;
      second.direction = second_alignment >= 0.0 ? 1 : -1;
      first.transition_side_locked = true;
      second.transition_side_locked = true;
      const double shared_radius = std::max(first.radius, second.radius);
      const double shared_peak = std::max(first.peak_offset, second.peak_offset);
      first.radius = shared_radius;
      second.radius = shared_radius;
      first.peak_offset = shared_peak;
      second.peak_offset = shared_peak;
      const auto duplicate_geometry = std::find_if(
        shared_pass_geometries.begin(), shared_pass_geometries.end(),
        [&](const SharedPassGeometry & geometry) {
          return geometry.first_source == first.source &&
                 geometry.second_source == second.source;
        });
      if (duplicate_geometry == shared_pass_geometries.end()) {
        shared_pass_waypoint_pair_count += shared_waypoint_pairs.size();
        shared_pass_geometries.push_back(
          SharedPassGeometry{
            first.source, second.source, first.center,
            first.direction, second.direction,
            std::move(shared_waypoint_pairs)});
      }
    }
  }

  // 二次净距审计可能在上面的首次配对之后才新增或升级同一永久点的约束。
  // 每轮重算前都用已确认的共享通道世界侧重新换算所有同中心约束，保证
  // 新增约束不会再次按接近零的横向量独立选到相反侧。
  auto apply_shared_pass_side_locks = [&](std::vector<CircleConstraint> & active) {
      for (auto & constraint : active) {
        for (const auto & shared : shared_pass_geometries) {
          if (std::hypot(
              constraint.center.x - shared.center.x,
              constraint.center.y - shared.center.y) > shared_center_tolerance ||
            constraint.source >= route_size || !normal_valid[constraint.source])
          {
            continue;
          }
          const double first_distance = std::abs(
            route_s[constraint.source] - route_s[shared.first_source]);
          const double second_distance = std::abs(
            route_s[constraint.source] - route_s[shared.second_source]);
          // 同一障碍还可能在转弯肩部新增二次约束。肩部法向已经旋转，
          // 与固定世界方向可能正交，不能因此丢掉侧向锁；按完整路线s轴
          // 归到更近的那次经过，并继承该次经过的有符号偏移方向。
          if (std::min(first_distance, second_distance) >
            circle_envelope_max_transition_length_ +
            circle_envelope_extension_step_)
          {
            continue;
          }
          constraint.direction = first_distance <= second_distance ?
            shared.first_direction : shared.second_direction;
          constraint.transition_side_locked = true;
          break;
        }
      }
    };
  apply_shared_pass_side_locks(constraints);

  // 先在每个独立锚点上求自己的最低有符号偏移量。这里读取的几何始终
  // 是original_waypoints_，不会拿前一个障碍物已经偏移后的路线判断后一个。
  for (auto & constraint : constraints) {
    const auto & base = original_waypoints_[constraint.source];
    const auto & normal = left_normals[constraint.source];
    const double qx = base.x - constraint.center.x;
    const double qy = base.y - constraint.center.y;
    const double along_normal = qx * normal.x + qy * normal.y;
    const double perpendicular_squared = std::max(
      0.0, qx * qx + qy * qy - along_normal * along_normal);
    const double discriminant =
      constraint.radius * constraint.radius - perpendicular_squared;
    if (discriminant >= 0.0) {
      const double root = std::sqrt(discriminant);
      const double signed_boundary = -along_normal +
        static_cast<double>(constraint.direction) * root;
      constraint.peak_offset = std::max(
        constraint.peak_offset,
        std::max(0.0, static_cast<double>(constraint.direction) * signed_boundary));
    }
  }

  auto closest_point_on_segment = [](const Point2D & point,
      const Waypoint & start, const Waypoint & finish) {
      const double segment_x = finish.x - start.x;
      const double segment_y = finish.y - start.y;
      const double length_squared =
        segment_x * segment_x + segment_y * segment_y;
      if (length_squared < 1.0e-12) {
        return Point2D{start.x, start.y};
      }
      const double ratio = std::clamp(
        ((point.x - start.x) * segment_x +
        (point.y - start.y) * segment_y) / length_squared,
        0.0, 1.0);
      return Point2D{
        start.x + ratio * segment_x,
        start.y + ratio * segment_y};
    };

  auto build_candidate = [&](const std::vector<CircleConstraint> & active_constraints,
      double common_extension, std::vector<Waypoint> & candidate,
      std::size_t & constraint_conflicts,
      std::size_t & overlap_corridor_points,
      std::size_t & local_bridge_count) {
      struct ConstraintProfile
      {
        std::size_t owner{0};
        std::size_t source{0};
        std::size_t first_deformable{0};
        double entry_s{0.0};
        double center_s{0.0};
        double exit_s{0.0};
        double signed_peak{0.0};
      };
      std::vector<ConstraintProfile> profiles;
      profiles.reserve(active_constraints.size());
      // 以原来的逐障碍smoothstep偏移作为默认路线。它的接入距离长、
      // 不会为了贴住每个安全圆而强制经过圆边界，因此在普通弯道和
      // 单障碍场景下通常比连续锚点曲线更自然。后面只对它真正出现
      // 进度反向的小区间做局部Bezier修复。
      std::vector<double> weighted_offsets(route_size, 0.0);
      std::vector<double> weights(route_size, 0.0);
      std::vector<double> lower_bounds(
        route_size, -std::numeric_limits<double>::infinity());
      std::vector<double> upper_bounds(
        route_size, std::numeric_limits<double>::infinity());
      const std::size_t no_constraint = active_constraints.size();
      std::vector<std::size_t> lower_owners(route_size, no_constraint);
      std::vector<std::size_t> upper_owners(route_size, no_constraint);

      for (std::size_t index = 0; index < route_size; ++index) {
        if (!normal_valid[index] || index > last_deformable_index)
        {
          lower_bounds[index] = 0.0;
          upper_bounds[index] = 0.0;
          continue;
        }
        if (following_qr_path_ || task_phase_ == TaskPhase::TASK1) {
          const auto & base = original_waypoints_[index];
          const auto & normal = left_normals[index];
          const double minimum_y_offset = task1_waypoint_shift_min_y_ - base.y;
          if (normal.y > 1.0e-9) {
            lower_bounds[index] = std::max(
              lower_bounds[index], minimum_y_offset / normal.y);
          } else if (normal.y < -1.0e-9) {
            upper_bounds[index] = std::min(
              upper_bounds[index], minimum_y_offset / normal.y);
          } else if (base.y < task1_waypoint_shift_min_y_) {
            lower_bounds[index] = 0.0;
            upper_bounds[index] = 0.0;
          }
        }
      }

      auto circle_boundary_offset = [&](std::size_t index,
          const CircleConstraint & constraint, double & boundary) {
          if (!normal_valid[index]) {
            return false;
          }
          const auto & base = original_waypoints_[index];
          const auto & normal = left_normals[index];
          const double center_x = constraint.center.x - base.x;
          const double center_y = constraint.center.y - base.y;
          const double center_normal =
            center_x * normal.x + center_y * normal.y;
          const double tangent_squared = std::max(
            0.0,
            center_x * center_x + center_y * center_y -
            center_normal * center_normal);
          const double radius_squared =
            constraint.radius * constraint.radius;
          const double center_distance_squared =
            center_x * center_x + center_y * center_y;
          const bool in_secondary_repair_core =
            constraint.secondary_repair &&
            index >= constraint.repair_first &&
            index <= constraint.repair_last;
          // 第一次生成仍只替换原路线真正落入安全圆的连续点段，避免
          // “无限长法线穿圆”让远处弯道受到假约束。唯一例外是二次审计
          // 已经用第一次生成后的有限路线段证实侵圆；此时也只在那一小段
          // 索引内用法线求圆边界，绝不把约束扩散到整条路线。
          if ((!in_secondary_repair_core &&
            center_distance_squared > radius_squared) ||
            tangent_squared > radius_squared)
          {
            return false;
          }
          const double half_chord = std::sqrt(std::max(
            0.0, radius_squared - tangent_squared));
          // d是原路线左法向的偏移量。圆在该法线上占据
          // [center_normal-half_chord, center_normal+half_chord]。
          boundary = constraint.direction > 0 ?
            std::max(0.0, center_normal + half_chord) :
            std::min(0.0, center_normal - half_chord);
          return true;
        };

      auto smoothstep_at = [&](double current_s, double center_s,
          double entry_s, double exit_s) {
          double progress = 1.0;
          if (current_s < center_s) {
            const double length = center_s - entry_s;
            progress = length > 1.0e-9 ?
              (current_s - entry_s) / length : 1.0;
          } else if (current_s > center_s) {
            const double length = exit_s - center_s;
            progress = length > 1.0e-9 ?
              (exit_s - current_s) / length : 1.0;
          }
          progress = std::clamp(progress, 0.0, 1.0);
          return progress * progress * progress *
                 (10.0 + progress * (-15.0 + 6.0 * progress));
        };

      for (std::size_t constraint_index = 0;
        constraint_index < active_constraints.size(); ++constraint_index)
      {
        const auto & constraint = active_constraints[constraint_index];
        const std::size_t constraint_first_deformable = std::clamp(
          constraint.first_deformable,
          first_deformable_index, last_deformable_index);
        auto transition_length_for_peak = [&](double peak) {
            const double curvature_transition = std::sqrt(
              6.0 * std::max(0.001, peak) /
              circle_envelope_max_curvature_);
            return std::clamp(
              std::max(
                circle_envelope_min_transition_length_ +
                0.5 * circle_envelope_merge_gap_, curvature_transition) +
              common_extension + constraint.transition_extension,
              circle_envelope_min_transition_length_,
              circle_envelope_max_transition_length_);
          };

        const double maximum_reasonable_peak =
          waypoint_cloud_check_radius_ + cone_circle_max_radius_ +
          circle_envelope_target_clearance_ + circle_envelope_numeric_margin_;
        double effective_peak = std::min(
          constraint.peak_offset, maximum_reasonable_peak);
        double transition_length = transition_length_for_peak(effective_peak);
        const double center_s = route_s[constraint.source];
        double entry_s = std::max(
          center_s - transition_length,
          route_s[constraint_first_deformable]);
        double exit_s = std::min(
          center_s + transition_length, route_s[last_deformable_index]);

        // 峰值只取安全圆几何上真正需要的最大偏移。不得用
        // boundary/smoothstep反推峰值：接线边缘的smoothstep接近0，
        // 这种除法会把整条路线放大到地图外。圆滑后的局部净距
        // 由下面整段circle boundary约束保证。
        for (std::size_t index = constraint_first_deformable;
          index <= last_deformable_index; ++index)
        {
          if (route_s[index] < entry_s || route_s[index] > exit_s) {
            continue;
          }
          double boundary = 0.0;
          if (!circle_boundary_offset(index, constraint, boundary)) {
            continue;
          }
          effective_peak = std::min(
            maximum_reasonable_peak,
            std::max(effective_peak, std::abs(boundary)));
        }

        transition_length = transition_length_for_peak(effective_peak);
        entry_s = std::max(
          center_s - transition_length,
          route_s[constraint_first_deformable]);
        exit_s = std::min(
          center_s + transition_length, route_s[last_deformable_index]);
        profiles.push_back(
          ConstraintProfile{
            constraint_index, constraint.source, constraint_first_deformable,
            entry_s, center_s, exit_s,
            static_cast<double>(constraint.direction) * effective_peak});
        for (std::size_t index = constraint_first_deformable;
          index <= last_deformable_index; ++index)
        {
          if (!normal_valid[index] ||
            route_s[index] < entry_s || route_s[index] > exit_s)
          {
            continue;
          }
          const double smoothstep = smoothstep_at(
            route_s[index], center_s, entry_s, exit_s);
          const double proposal =
            static_cast<double>(constraint.direction) *
            effective_peak * smoothstep;
          weighted_offsets[index] += smoothstep * proposal;
          weights[index] += smoothstep;

          // 共享通道的正反两遍已选定同一世界绕行侧。锁定整个smoothstep
          // 接入区间的符号，而不只是安全圆核心：以本次顺时针返程为例，
          // 同一个正偏移在向左水平段表示向下，在随后向下竖直段表示向右，
          // 正好形成连续的“水平向下→竖直向右”路线。其他障碍接线只允许
          // 改变偏移大小，不能通过加权平均把它翻到原路线另一侧。
          if (constraint.transition_side_locked) {
            if (constraint.direction > 0 && lower_bounds[index] < 0.0) {
              lower_bounds[index] = 0.0;
              lower_owners[index] = constraint_index;
            } else if (constraint.direction < 0 && upper_bounds[index] > 0.0) {
              upper_bounds[index] = 0.0;
              upper_owners[index] = constraint_index;
            }
          }

          double boundary = 0.0;
          if (!circle_boundary_offset(index, constraint, boundary)) {
            continue;
          }
          if (constraint.direction > 0 && boundary > lower_bounds[index]) {
            lower_bounds[index] = boundary;
            lower_owners[index] = constraint_index;
          } else if (constraint.direction < 0 &&
            boundary < upper_bounds[index])
          {
            upper_bounds[index] = boundary;
            upper_owners[index] = constraint_index;
          }
        }
      }

      std::vector<double> offsets(route_size, 0.0);
      for (std::size_t index = 0; index < route_size; ++index) {
        if (weights[index] > 1.0e-12) {
          offsets[index] = weighted_offsets[index] / weights[index];
        }
      }
      local_bridge_count = 0;

      struct ActiveOverlapChord
      {
        std::size_t lower_owner{0};
        std::size_t upper_owner{0};
        std::size_t core_first{0};
        std::size_t core_last{0};
        CircleIntersectionChord chord;
      };
      std::vector<ActiveOverlapChord> active_overlap_chords;
      std::vector<bool> overlap_corridor(route_size, false);

      // 两个方向相反的规划圆发生冲突时，不再逐航点平均上下边界。
      // 平均值依赖每个航点自己的原路线法线，生成的世界坐标并不共线，
      // 后续一维平滑便会在重叠区形成不规则折弯。这里直接使用实际规划
      // 半径（target_clearance + numeric_margin）计算两个交点，并把局部
      // 冲突段锁到两个交点之间的公共弦上。
      std::size_t conflict_index = 0;
      while (conflict_index < route_size) {
        if (lower_bounds[conflict_index] <= upper_bounds[conflict_index] ||
          lower_owners[conflict_index] == no_constraint ||
          upper_owners[conflict_index] == no_constraint)
        {
          ++conflict_index;
          continue;
        }

        const std::size_t lower_owner = lower_owners[conflict_index];
        const std::size_t upper_owner = upper_owners[conflict_index];
        const auto & lower_circle = active_constraints[lower_owner];
        const auto & upper_circle = active_constraints[upper_owner];
        std::size_t run_last = conflict_index;
        while (run_last + 1 < route_size &&
          lower_bounds[run_last + 1] > upper_bounds[run_last + 1] &&
          lower_owners[run_last + 1] == lower_owner &&
          upper_owners[run_last + 1] == upper_owner)
        {
          ++run_last;
        }

        CircleIntersectionChord chord;
        if (!circle_intersection_chord(
            lower_circle.center.x, lower_circle.center.y, lower_circle.radius,
            upper_circle.center.x, upper_circle.center.y, upper_circle.radius,
            chord))
        {
          // 同心、内含或数值上接近相切时没有稳定的两个交点。保留旧的
          // 中值退化处理，避免无交点几何把路线留在互相矛盾的硬边界中。
          const double center_distance = std::hypot(
            lower_circle.center.x - upper_circle.center.x,
            lower_circle.center.y - upper_circle.center.y);
          if (center_distance + 1.0e-9 <
            lower_circle.radius + upper_circle.radius)
          {
            for (std::size_t index = conflict_index;
              index <= run_last; ++index)
            {
              const double corridor_offset =
                0.5 * (lower_bounds[index] + upper_bounds[index]);
              lower_bounds[index] = corridor_offset;
              upper_bounds[index] = corridor_offset;
              overlap_corridor[index] = true;
            }
          }
          conflict_index = run_last + 1;
          continue;
        }

        auto chord_offset_at = [&](std::size_t index, bool require_segment,
            double & chord_offset) {
            if (index >= route_size || !normal_valid[index]) {
              return false;
            }
            double chord_ratio = 0.0;
            const auto & base = original_waypoints_[index];
            const auto & normal = left_normals[index];
            if (!normal_line_intersects_chord(
                base.x, base.y, normal.x, normal.y,
                chord, chord_offset, chord_ratio))
            {
              return false;
            }
            return !require_segment ||
                   (chord_ratio >= -1.0e-6 && chord_ratio <= 1.0 + 1.0e-6);
          };

        // 先确认冲突段确实能由原路线各点的局部法线投影到公共弦。
        // 若极端几何下法线与弦平行，则只对该段使用中值退化，不把约束
        // 扩散到路线其他位置。
        std::size_t first_chord_index = route_size;
        std::size_t last_chord_index = 0;
        for (std::size_t index = conflict_index; index <= run_last; ++index) {
          double chord_offset = 0.0;
          if (chord_offset_at(index, true, chord_offset)) {
            first_chord_index = std::min(first_chord_index, index);
            last_chord_index = std::max(last_chord_index, index);
          }
        }
        if (first_chord_index == route_size) {
          for (std::size_t index = conflict_index; index <= run_last; ++index) {
            const double corridor_offset =
              0.5 * (lower_bounds[index] + upper_bounds[index]);
            lower_bounds[index] = corridor_offset;
            upper_bounds[index] = corridor_offset;
            overlap_corridor[index] = true;
          }
          conflict_index = run_last + 1;
          continue;
        }

        const std::size_t corridor_first_deformable = std::max(
          lower_circle.first_deformable, upper_circle.first_deformable);
        while (first_chord_index > corridor_first_deformable) {
          double chord_offset = 0.0;
          if (!chord_offset_at(first_chord_index - 1, true, chord_offset)) {
            break;
          }
          --first_chord_index;
        }
        while (last_chord_index < last_deformable_index) {
          double chord_offset = 0.0;
          if (!chord_offset_at(last_chord_index + 1, true, chord_offset)) {
            break;
          }
          ++last_chord_index;
        }

        for (std::size_t index = first_chord_index;
          index <= last_chord_index; ++index)
        {
          double chord_offset = 0.0;
          if (!chord_offset_at(index, true, chord_offset)) {
            continue;
          }
          const bool foreign_lower_bound =
            std::isfinite(lower_bounds[index]) &&
            lower_owners[index] != lower_owner &&
            lower_owners[index] != upper_owner;
          const bool foreign_upper_bound =
            std::isfinite(upper_bounds[index]) &&
            upper_owners[index] != lower_owner &&
            upper_owners[index] != upper_owner;
          if ((foreign_lower_bound &&
            chord_offset + 1.0e-9 < lower_bounds[index]) ||
            (foreign_upper_bound &&
            chord_offset - 1.0e-9 > upper_bounds[index]))
          {
            // Task1最低Y或第三个独立圆仍然是硬约束；两圆公共弦不能
            // 静默覆盖它。保留原边界，让质量检查把真正的多圆无解记录出来。
            continue;
          }
          lower_bounds[index] = chord_offset;
          upper_bounds[index] = chord_offset;
          overlap_corridor[index] = true;
        }
        active_overlap_chords.push_back(
          ActiveOverlapChord{
            lower_owner, upper_owner,
            first_chord_index, last_chord_index, chord});
        conflict_index = run_last + 1;
      }

      // 公共弦核心是精确硬约束；两端则沿公共弦延长线用五次smoothstep
      // 逐渐混回原来的整体偏移场。这样交点之间保持严格共线，接入处的
      // 一、二阶变化同时趋零，随后十轮全局平滑只需消除离散采样误差。
      const double chord_shoulder_length = std::max(
        circle_envelope_merge_gap_,
        0.5 * (circle_envelope_target_clearance_ +
        circle_envelope_numeric_margin_));
      const double maximum_reasonable_chord_offset =
        waypoint_cloud_check_radius_ + cone_circle_max_radius_ +
        circle_envelope_target_clearance_ + circle_envelope_numeric_margin_;
      auto quintic_blend = [](double progress) {
          const double t = std::clamp(progress, 0.0, 1.0);
          return t * t * t * (10.0 + t * (-15.0 + 6.0 * t));
      };
      for (const auto & corridor : active_overlap_chords) {
        const std::size_t corridor_first_deformable = std::max(
          active_constraints[corridor.lower_owner].first_deformable,
          active_constraints[corridor.upper_owner].first_deformable);
        auto blend_to_chord = [&](std::size_t index, double blend) {
            if (overlap_corridor[index] || !normal_valid[index]) {
              return;
            }
            double chord_offset = 0.0;
            double chord_ratio = 0.0;
            const auto & base = original_waypoints_[index];
            const auto & normal = left_normals[index];
            if (!normal_line_intersects_chord(
                base.x, base.y, normal.x, normal.y,
                corridor.chord, chord_offset, chord_ratio) ||
              std::abs(chord_offset) > maximum_reasonable_chord_offset)
            {
              return;
            }
            offsets[index] =
              (1.0 - blend) * offsets[index] + blend * chord_offset;
          };

        for (std::size_t index = corridor.core_first;
          index > corridor_first_deformable; )
        {
          --index;
          const double distance_to_core =
            route_s[corridor.core_first] - route_s[index];
          if (distance_to_core > chord_shoulder_length) {
            break;
          }
          blend_to_chord(
            index,
            quintic_blend(1.0 - distance_to_core / chord_shoulder_length));
        }
        for (std::size_t index = corridor.core_last + 1;
          index <= last_deformable_index; ++index)
        {
          const double distance_to_core =
            route_s[index] - route_s[corridor.core_last];
          if (distance_to_core > chord_shoulder_length) {
            break;
          }
          blend_to_chord(
            index,
            quintic_blend(1.0 - distance_to_core / chord_shoulder_length));
        }
      }
      overlap_corridor_points = static_cast<std::size_t>(std::count(
        overlap_corridor.begin(), overlap_corridor.end(), true));

      auto enforce_geometric_bounds = [&]() {
          std::size_t conflicts = 0;
          for (std::size_t index = 0; index < route_size; ++index) {
            if (lower_bounds[index] <= upper_bounds[index]) {
              offsets[index] = std::clamp(
                offsets[index], lower_bounds[index], upper_bounds[index]);
              continue;
            }
            // 非相交圆仍出现相反约束才到这里。不合并、不换向，
            // 保留离整体圆滑结果更近的一侧并详细记录。
            ++conflicts;
            offsets[index] =
              std::abs(offsets[index] - lower_bounds[index]) <=
              std::abs(offsets[index] - upper_bounds[index]) ?
              lower_bounds[index] : upper_bounds[index];
          }
          return conflicts;
        };

      constraint_conflicts = enforce_geometric_bounds();
      for (int iteration = 0;
        iteration < circle_envelope_smoothing_iterations_; ++iteration)
      {
        std::vector<double> smoothed = offsets;
        for (std::size_t index = 1; index + 1 < route_size; ++index) {
          smoothed[index] =
            0.25 * offsets[index - 1] +
            0.50 * offsets[index] +
            0.25 * offsets[index + 1];
        }
        offsets = std::move(smoothed);
        constraint_conflicts = std::max(
          constraint_conflicts, enforce_geometric_bounds());
      }

      candidate = original_waypoints_;
      for (std::size_t index = 0; index < route_size; ++index) {
        if (!normal_valid[index] || std::abs(offsets[index]) < 1.0e-12) {
          continue;
        }
        candidate[index].x += offsets[index] * left_normals[index].x;
        candidate[index].y += offsets[index] * left_normals[index].y;
      }

      // 相交安全圆允许路线沿公共弦附近的窄通道通过。这个例外只服务于
      // 后面的局部折返桥校验：桥上的点必须属于产生冲突的那一对圆，且
      // 仍贴近它们的有限公共弦，不能借“圆相交”放宽到整片区域。
      auto overlap_bridge_allows = [&](const Point2D & point,
          std::size_t constraint_owner) {
          const double corridor_tolerance = std::max(
            0.03, circle_envelope_merge_gap_);
          for (const auto & corridor : active_overlap_chords) {
            if (constraint_owner != corridor.lower_owner &&
              constraint_owner != corridor.upper_owner)
            {
              continue;
            }
            if (point_near_chord_segment(
                point.x, point.y, corridor.chord, corridor_tolerance))
            {
              return true;
            }
          }
          return false;
        };

      // 原路线在C/B一类高曲率接缝附近转向较快时，即使每个点都沿自己的
      // 法线落在安全圆外，也可能因为相邻法线交叉而把航点次序反过来，
      // 表现为小回环或尖峰。这里不重新选障碍方向，也不合并圆：只找最终
      // 橙色基线中真正出现进度反向或近停滞的最小连续段，使用更长的三次Bezier
      // 桥接两端。相交圆公共弦不再成为“禁止修复区”；桥可以沿公共弦的
      // 窄通道通过，但仍必须单向前进且不得自交。普通安全区保留橙线，不再
      // 为了贴住每个圆而重画。桥接结果必须逐圆通过target_clearance净距、
      // 保持各障碍原定绕行侧并消除整段进度反向，否则原样保留当前候选。
      auto segment_has_insufficient_original_progress = [&](
          const std::vector<Waypoint> & route,
          std::size_t segment) {
          if (segment + 1 >= route_size) {
            return false;
          }
          const double original_x =
            original_waypoints_[segment + 1].x - original_waypoints_[segment].x;
          const double original_y =
            original_waypoints_[segment + 1].y - original_waypoints_[segment].y;
          if (original_x * original_x + original_y * original_y < 1.0e-12) {
            return false;
          }
          const double route_x = route[segment + 1].x - route[segment].x;
          const double route_y = route[segment + 1].y - route[segment].y;
          const double progress_ratio =
            (route_x * original_x + route_y * original_y) /
            (original_x * original_x + original_y * original_y);
          // 不能只在已经倒退时才修复。圆偏移经过原路线急弯时，
          // 1-kappa*d可能仍为正但已经接近0，离散路线会表现成几乎
          // 原地转向的尖角。复用legacy局部重建的前进比例门槛，提前
          // 把这种近停滞段交给同一条Bezier桥处理。
          return progress_ratio <= waypoint_shift_rebuild_min_progress_ratio_;
        };
      const double progress_bridge_shoulder = std::min(
        circle_envelope_max_transition_length_,
        // 只包住真正反向段，并在两侧各留约0.40m相切接回。此前0.60m
        // 会把本来已经很顺的下一个障碍圆弧也一起重画。
        std::max(0.30, 0.25 * circle_envelope_min_transition_length_));
      constexpr double bridge_handle_ratio = 0.40;
      std::size_t progress_segment = 0;
      while (progress_segment + 1 < route_size) {
        if (!segment_has_insufficient_original_progress(
            candidate, progress_segment))
        {
          ++progress_segment;
          continue;
        }
        const std::size_t violation_first = progress_segment;
        std::size_t violation_last = progress_segment;
        while (violation_last + 2 < route_size &&
          segment_has_insufficient_original_progress(
            candidate, violation_last + 1))
        {
          ++violation_last;
        }

        const double requested_first_s = std::max(
          route_s.front(), route_s[violation_first] - progress_bridge_shoulder);
        const double requested_last_s = std::min(
          route_s.back(), route_s[violation_last + 1] + progress_bridge_shoulder);
        std::size_t anchor_first = static_cast<std::size_t>(std::distance(
          route_s.begin(), std::lower_bound(
            route_s.begin(), route_s.end(), requested_first_s)));
        std::size_t anchor_last = static_cast<std::size_t>(std::distance(
          route_s.begin(), std::lower_bound(
            route_s.begin(), route_s.end(), requested_last_s)));
        anchor_first = std::clamp<std::size_t>(anchor_first, 1, route_size - 2);
        // Task3停车点本身固定不动，但其前一段也可能产生反向尖峰。
        // 允许Bezier以固定末点为接回锚点；末端切向改从原路线单侧取得。
        anchor_last = std::clamp<std::size_t>(anchor_last, 1, route_size - 1);

        // 正反两次经过共用通道时，它们在完整路线s轴上是两段独立路线，
        // 允许各自修复自己的反向尖峰。锚点可以跨在公共弦两侧；是否仍在
        // 允许的两圆中间窄通道内，由下面逐线段的公共弦校验决定。
        if (anchor_last <= anchor_first + 3 ||
          anchor_first >= violation_first ||
          anchor_last <= violation_last + 1)
        {
          progress_segment = violation_last + 1;
          continue;
        }

        Point2D start_tangent{
          candidate[anchor_first].x - candidate[anchor_first - 1].x,
          candidate[anchor_first].y - candidate[anchor_first - 1].y};
        Point2D finish_tangent;
        if (anchor_last + 1 < route_size) {
          finish_tangent = Point2D{
            candidate[anchor_last + 1].x - candidate[anchor_last].x,
            candidate[anchor_last + 1].y - candidate[anchor_last].y};
        } else {
          finish_tangent = Point2D{
            original_waypoints_[anchor_last].x -
            original_waypoints_[anchor_last - 1].x,
            original_waypoints_[anchor_last].y -
            original_waypoints_[anchor_last - 1].y};
        }
        const double start_tangent_length = std::hypot(
          start_tangent.x, start_tangent.y);
        const double finish_tangent_length = std::hypot(
          finish_tangent.x, finish_tangent.y);
        const double chord_length = std::hypot(
          candidate[anchor_last].x - candidate[anchor_first].x,
          candidate[anchor_last].y - candidate[anchor_first].y);
        if (start_tangent_length < 1.0e-9 ||
          finish_tangent_length < 1.0e-9 || chord_length < 1.0e-6)
        {
          progress_segment = violation_last + 1;
          continue;
        }
        start_tangent.x /= start_tangent_length;
        start_tangent.y /= start_tangent_length;
        finish_tangent.x /= finish_tangent_length;
        finish_tangent.y /= finish_tangent_length;

        std::vector<Waypoint> trial = candidate;
        const Point2D p0{
          candidate[anchor_first].x, candidate[anchor_first].y};
        const Point2D p1{
          p0.x + bridge_handle_ratio * chord_length * start_tangent.x,
          p0.y + bridge_handle_ratio * chord_length * start_tangent.y};
        const Point2D p3{
          candidate[anchor_last].x, candidate[anchor_last].y};
        const Point2D p2{
          p3.x - bridge_handle_ratio * chord_length * finish_tangent.x,
          p3.y - bridge_handle_ratio * chord_length * finish_tangent.y};
        const double bridge_s_length =
          route_s[anchor_last] - route_s[anchor_first];
        for (std::size_t index = anchor_first + 1;
          index < anchor_last; ++index)
        {
          const double t = std::clamp(
            (route_s[index] - route_s[anchor_first]) / bridge_s_length,
            0.0, 1.0);
          const double u = 1.0 - t;
          const double w0 = u * u * u;
          const double w1 = 3.0 * u * u * t;
          const double w2 = 3.0 * u * t * t;
          const double w3 = t * t * t;
          trial[index].x =
            w0 * p0.x + w1 * p1.x + w2 * p2.x + w3 * p3.x;
          trial[index].y =
            w0 * p0.y + w1 * p1.y + w2 * p2.y + w3 * p3.y;
        }

        bool valid_bridge = true;
        for (std::size_t index = anchor_first;
          index < anchor_last; ++index)
        {
          if (segment_has_insufficient_original_progress(trial, index)) {
            valid_bridge = false;
            break;
          }
        }
        // “对原路线保持前进”之外，再直接禁止局部桥自身相交，避免急弯
        // 附近每一小段点积都为正、组合起来却仍形成一个小圈。
        auto signed_area = [](const Waypoint & a, const Waypoint & b,
            const Waypoint & c) {
            return (b.x - a.x) * (c.y - a.y) -
                   (b.y - a.y) * (c.x - a.x);
          };
        auto proper_segments_intersect = [&](const Waypoint & a,
            const Waypoint & b, const Waypoint & c, const Waypoint & d) {
            constexpr double epsilon = 1.0e-9;
            const double ab_c = signed_area(a, b, c);
            const double ab_d = signed_area(a, b, d);
            const double cd_a = signed_area(c, d, a);
            const double cd_b = signed_area(c, d, b);
            return ((ab_c > epsilon && ab_d < -epsilon) ||
                   (ab_c < -epsilon && ab_d > epsilon)) &&
                   ((cd_a > epsilon && cd_b < -epsilon) ||
                   (cd_a < -epsilon && cd_b > epsilon));
          };
        for (std::size_t first_segment = anchor_first;
          valid_bridge && first_segment + 1 < anchor_last; ++first_segment)
        {
          for (std::size_t second_segment = first_segment + 2;
            second_segment + 1 <= anchor_last; ++second_segment)
          {
            if (proper_segments_intersect(
                trial[first_segment], trial[first_segment + 1],
                trial[second_segment], trial[second_segment + 1]))
            {
              valid_bridge = false;
              break;
            }
          }
        }
        for (const auto & profile : profiles) {
          if (!valid_bridge ||
            profile.exit_s < route_s[anchor_first] ||
            profile.entry_s > route_s[anchor_last])
          {
            continue;
          }
          const auto & constraint = active_constraints[profile.owner];
          const double required_radius = std::max(
            0.0, constraint.radius - circle_envelope_numeric_margin_);
          for (std::size_t index = anchor_first;
            index < anchor_last; ++index)
          {
            const Point2D closest = closest_point_on_segment(
              constraint.center, trial[index], trial[index + 1]);
            if (std::hypot(
                closest.x - constraint.center.x,
                closest.y - constraint.center.y) + 1.0e-7 < required_radius)
            {
              if (!overlap_bridge_allows(closest, profile.owner)) {
                valid_bridge = false;
                break;
              }
            }
          }
          if (valid_bridge && profile.source >= anchor_first &&
            profile.source <= anchor_last)
          {
            const Point2D source_point{
              trial[profile.source].x, trial[profile.source].y};
            if (!overlap_bridge_allows(source_point, profile.owner)) {
              const double signed_side =
                static_cast<double>(constraint.direction) *
                ((trial[profile.source].x -
                original_waypoints_[profile.source].x) *
                left_normals[profile.source].x +
                (trial[profile.source].y -
                original_waypoints_[profile.source].y) *
                left_normals[profile.source].y);
              valid_bridge = signed_side >= -1.0e-6;
            }
          }
        }
        if (valid_bridge) {
          candidate.swap(trial);
          ++local_bridge_count;
          progress_segment = anchor_last;
        } else {
          progress_segment = violation_last + 1;
        }
      }

    };

  auto overlap_corridor_allows = [&](const Point2D & point,
      std::size_t constraint_index,
      const std::vector<CircleConstraint> & active_constraints) {
      const auto & constraint = active_constraints[constraint_index];
      for (std::size_t partner_index = 0;
        partner_index < active_constraints.size(); ++partner_index)
      {
        if (partner_index == constraint_index) {
          continue;
        }
        const auto & partner = active_constraints[partner_index];
        if (constraint.direction == partner.direction) {
          continue;
        }
        CircleIntersectionChord chord;
        if (circle_intersection_chord(
            constraint.center.x, constraint.center.y, constraint.radius,
            partner.center.x, partner.center.y, partner.radius,
            chord) &&
          point_near_chord_segment(
            point.x, point.y, chord,
            std::max(0.02, 0.5 * circle_envelope_numeric_margin_)))
        {
          return true;
        }
      }
      return false;
    };

  auto clearance_margins = [&](const std::vector<Waypoint> & candidate,
      const std::vector<CircleConstraint> & active_constraints) {
      std::vector<double> margins(
        active_constraints.size(), std::numeric_limits<double>::infinity());
      for (std::size_t constraint_index = 0;
        constraint_index < active_constraints.size(); ++constraint_index)
      {
        const auto & constraint = active_constraints[constraint_index];
        const double required_radius = std::max(
          0.0, constraint.radius - circle_envelope_numeric_margin_);
        for (std::size_t index = 0; index + 1 < candidate.size(); ++index) {
          const Point2D closest = closest_point_on_segment(
            constraint.center, candidate[index], candidate[index + 1]);
          const double margin = std::hypot(
            closest.x - constraint.center.x,
            closest.y - constraint.center.y) - required_radius;
          if (margin < 0.0 && overlap_corridor_allows(
              closest, constraint_index, active_constraints))
          {
            continue;
          }
          margins[constraint_index] = std::min(
            margins[constraint_index], margin);
        }
      }
      return margins;
    };

  auto discrete_curvature_at = [](const std::vector<Waypoint> & route,
      std::size_t index) {
      if (index == 0 || index + 1 >= route.size()) {
        return 0.0;
      }
      const double ab = std::hypot(
        route[index].x - route[index - 1].x,
        route[index].y - route[index - 1].y);
      const double bc = std::hypot(
        route[index + 1].x - route[index].x,
        route[index + 1].y - route[index].y);
      const double ac = std::hypot(
        route[index + 1].x - route[index - 1].x,
        route[index + 1].y - route[index - 1].y);
      const double denominator = ab * bc * ac;
      if (denominator < 1.0e-9) {
        return 0.0;
      }
      const double cross =
        (route[index].x - route[index - 1].x) *
        (route[index + 1].y - route[index - 1].y) -
        (route[index].y - route[index - 1].y) *
        (route[index + 1].x - route[index - 1].x);
      return std::abs(2.0 * cross / denominator);
    };
  auto maximum_discrete_curvature = [&](const std::vector<Waypoint> & route) {
      double maximum = 0.0;
      for (std::size_t index = 1; index + 1 < route.size(); ++index) {
        maximum = std::max(maximum, discrete_curvature_at(route, index));
      }
      return maximum;
    };

  auto progress_violation_count = [&](const std::vector<Waypoint> & route) {
      std::size_t count = 0;
      for (std::size_t index = 0; index + 1 < route_size; ++index) {
        const double original_x =
          original_waypoints_[index + 1].x - original_waypoints_[index].x;
        const double original_y =
          original_waypoints_[index + 1].y - original_waypoints_[index].y;
        const double original_length_squared =
          original_x * original_x + original_y * original_y;
        if (original_length_squared < 1.0e-12) {
          continue;
        }
        const double candidate_x = route[index + 1].x - route[index].x;
        const double candidate_y = route[index + 1].y - route[index].y;
        const double progress_ratio =
          (candidate_x * original_x + candidate_y * original_y) /
          original_length_squared;
        if (progress_ratio <= waypoint_shift_rebuild_min_progress_ratio_) {
          ++count;
        }
      }
      return count;
    };

  struct CandidateQuality
  {
    std::size_t clearance_failures{std::numeric_limits<std::size_t>::max()};
    std::size_t anchor_conflicts{std::numeric_limits<std::size_t>::max()};
    std::size_t progress_violations{std::numeric_limits<std::size_t>::max()};
    std::size_t overlap_corridor_points{0};
    std::size_t local_bridge_count{0};
    double minimum_clearance_margin{-std::numeric_limits<double>::infinity()};
    double maximum_curvature{std::numeric_limits<double>::infinity()};
  };
  auto quality_better = [](const CandidateQuality & first,
      const CandidateQuality & second) {
      if (first.clearance_failures != second.clearance_failures) {
        return first.clearance_failures < second.clearance_failures;
      }
      if (first.anchor_conflicts != second.anchor_conflicts) {
        return first.anchor_conflicts < second.anchor_conflicts;
      }
      if (first.progress_violations != second.progress_violations) {
        return first.progress_violations < second.progress_violations;
      }
      if (first.clearance_failures == 0 &&
        std::abs(first.maximum_curvature - second.maximum_curvature) > 1.0e-9)
      {
        return first.maximum_curvature < second.maximum_curvature;
      }
      if (std::abs(
          first.minimum_clearance_margin -
          second.minimum_clearance_margin) > 1.0e-9)
      {
        return first.minimum_clearance_margin >
               second.minimum_clearance_margin;
      }
      return first.maximum_curvature < second.maximum_curvature;
    };

  const double extension_range = std::max(
    0.0,
    circle_envelope_max_transition_length_ -
    circle_envelope_min_transition_length_);
  // 旧版会按extension_step把整个长度区间逐格穷举；当前0.80m区间会生成
  // 17条完整路线，并在每轮二次审计中重复同一批全路线圆滑。这里恢复为
  // 有界候选：只保留最短和最长两档，仍覆盖完整配置范围。
  // 净距不足由后面的局部连续段修复处理，不再依靠0.05m密集扫参。
  constexpr int maximum_attempt = 1;
  auto search_best_route = [&](const std::vector<CircleConstraint> & active_constraints,
      std::vector<Waypoint> & selected_route,
      CandidateQuality & selected_quality, bool & selected_accepted) {
      selected_route = original_waypoints_;
      selected_quality = CandidateQuality();
      selected_accepted = false;
      for (int attempt = 0; attempt <= maximum_attempt; ++attempt) {
        const double attempt_ratio = maximum_attempt > 0 ?
          static_cast<double>(attempt) / static_cast<double>(maximum_attempt) : 0.0;
        const double common_extension = extension_range * attempt_ratio;
        std::vector<Waypoint> candidate;
        std::size_t anchor_conflicts = 0;
        std::size_t overlap_corridor_points = 0;
        std::size_t local_bridge_count = 0;
        build_candidate(
          active_constraints, common_extension, candidate, anchor_conflicts,
          overlap_corridor_points, local_bridge_count);
        const std::vector<double> margins = clearance_margins(
          candidate, active_constraints);

        CandidateQuality quality;
        quality.anchor_conflicts = anchor_conflicts;
        quality.progress_violations = progress_violation_count(candidate);
        quality.overlap_corridor_points = overlap_corridor_points;
        quality.local_bridge_count = local_bridge_count;
        quality.maximum_curvature = maximum_discrete_curvature(candidate);
        quality.clearance_failures = 0;
        quality.minimum_clearance_margin =
          std::numeric_limits<double>::infinity();
        for (const double margin : margins) {
          quality.minimum_clearance_margin = std::min(
            quality.minimum_clearance_margin, margin);
          if (margin < -1.0e-7) {
            ++quality.clearance_failures;
          }
        }

        if (quality_better(quality, selected_quality)) {
          selected_quality = quality;
          selected_route = candidate;
        }
        if (quality.clearance_failures == 0 &&
          quality.anchor_conflicts == 0 &&
          quality.progress_violations == 0 &&
          quality.maximum_curvature <= circle_envelope_max_curvature_)
        {
          selected_accepted = true;
          break;
        }
      }
    };

  std::vector<Waypoint> best_route;
  CandidateQuality best_quality;
  bool accepted = false;
  search_best_route(constraints, best_route, best_quality, accepted);

  // 第一次路线只由原路线真正命中的障碍生成。它可能被别的障碍的圆滑尾段
  // 推入一个“原路线本来在圆外、因此此前没有偏移元数据”的永久绿/橙点。
  // 这里审计有限的最终路线段，只把真实侵入circle_envelope_target_clearance
  // 的连续段补成局部约束，然后仍交给同一个批量生成器从原路线统一重算。
  // 不逐航点硬推、不把零偏移点注册成全局约束；用有限轮次处理修复后新出现
  // 的另一个侵圆点。已有同ID主约束若仍然侵圆，必须升级为有限区间修复，
  // 不能把“已有约束”误当成“约束已经生效”。
  const std::size_t primary_constraint_count = constraints.size();
  std::size_t secondary_repair_count = 0;
  std::size_t promoted_primary_repair_count = 0;
  std::size_t expanded_secondary_repair_count = 0;
  std::size_t reinforced_secondary_repair_count = 0;
  std::size_t secondary_repair_round_count = 0;
  std::size_t rejected_secondary_replan_count = 0;
  std::size_t route_fairing_accept_count = 0;
  std::size_t route_fairing_reject_count = 0;
  std::size_t residual_permanent_clearance_failures = 0;
  const double planning_clearance_radius =
    circle_envelope_target_clearance_ + circle_envelope_numeric_margin_;
  // 恢复5518413的有界策略。二次审计只修真正小于target_clearance的
  // 连续段；numeric_margin是生成和候选排序的软余量，不值得为它阻塞
  // 50Hz控制循环十几轮。
  constexpr int maximum_secondary_repair_rounds = 4;
  constexpr double permanent_center_tolerance = 0.03;
  const bool audit_permanent_obstacles =
    logic3_planning_pose_anchored_ && !logic3_green_clusters_.empty();
  if (audit_permanent_obstacles) {
    auto intentional_permanent_overlap_corridor = [&](const Point2D & point,
        std::size_t obstacle_index) {
        const auto & obstacle = logic3_green_clusters_[obstacle_index].center;
        for (std::size_t partner_index = 0;
          partner_index < logic3_green_clusters_.size(); ++partner_index)
        {
          if (partner_index == obstacle_index) {
            continue;
          }
          const auto & partner = logic3_green_clusters_[partner_index].center;
          CircleIntersectionChord chord;
          if (circle_intersection_chord(
              obstacle.x, obstacle.y, planning_clearance_radius,
              partner.x, partner.y, planning_clearance_radius,
              chord) &&
            point_near_chord_segment(
              point.x, point.y, chord,
              std::max(0.02, 0.5 * circle_envelope_numeric_margin_)))
          {
            return true;
          }
        }
        return false;
      };

    const std::size_t last_auditable_segment = std::min(
      last_deformable_index, route_size - 2);
    struct PermanentClearanceQuality
    {
      std::size_t failing_obstacles{0};
      std::size_t violating_segments{0};
      double maximum_penetration{0.0};
      double total_penetration{0.0};
      double minimum_clearance{std::numeric_limits<double>::infinity()};
      std::vector<double> maximum_penetration_by_obstacle;
      std::vector<double> total_penetration_by_obstacle;
    };
    auto evaluate_permanent_clearance = [&](const std::vector<Waypoint> & route,
        double required_clearance) {
        PermanentClearanceQuality quality;
        quality.maximum_penetration_by_obstacle.assign(
          logic3_green_clusters_.size(), 0.0);
        quality.total_penetration_by_obstacle.assign(
          logic3_green_clusters_.size(), 0.0);
        for (std::size_t obstacle_index = 0;
          obstacle_index < logic3_green_clusters_.size(); ++obstacle_index)
        {
          const bool dynamic_takeover =
            obstacle_index < logic3_green_dynamic_takeover_.size() &&
            logic3_green_dynamic_takeover_[obstacle_index];
          if (dynamic_takeover) {
            continue;
          }
          const auto & center = logic3_green_clusters_[obstacle_index].center;
          const std::size_t obstacle_first_deformable =
            obstacle_first_deformable_index(center);
          bool obstacle_failed = false;
          for (std::size_t segment = obstacle_first_deformable;
            segment <= last_auditable_segment; ++segment)
          {
            const Point2D closest = closest_point_on_segment(
              center, route[segment], route[segment + 1]);
            const double clearance = std::hypot(
              closest.x - center.x, closest.y - center.y);
            if (clearance + 1.0e-7 < required_clearance &&
              intentional_permanent_overlap_corridor(closest, obstacle_index))
            {
              continue;
            }
            quality.minimum_clearance = std::min(
              quality.minimum_clearance, clearance);
            const double penetration = std::max(
              0.0, required_clearance - clearance);
            if (penetration <= 1.0e-7) {
              continue;
            }
            obstacle_failed = true;
            ++quality.violating_segments;
            quality.maximum_penetration = std::max(
              quality.maximum_penetration, penetration);
            quality.total_penetration += penetration;
            quality.maximum_penetration_by_obstacle[obstacle_index] = std::max(
              quality.maximum_penetration_by_obstacle[obstacle_index],
              penetration);
            quality.total_penetration_by_obstacle[obstacle_index] += penetration;
          }
          if (obstacle_failed) {
            ++quality.failing_obstacles;
          }
        }
        return quality;
      };
    auto permanent_quality_better = [](
        const PermanentClearanceQuality & candidate,
        const PermanentClearanceQuality & retained) {
        constexpr double tolerance = 1.0e-7;
        if (candidate.failing_obstacles != retained.failing_obstacles) {
          return candidate.failing_obstacles < retained.failing_obstacles;
        }
        if (std::abs(
            candidate.maximum_penetration -
            retained.maximum_penetration) > tolerance)
        {
          return candidate.maximum_penetration < retained.maximum_penetration;
        }
        if (std::abs(
            candidate.total_penetration - retained.total_penetration) > tolerance)
        {
          return candidate.total_penetration < retained.total_penetration;
        }
        if (candidate.violating_segments != retained.violating_segments) {
          return candidate.violating_segments < retained.violating_segments;
        }
        return candidate.minimum_clearance >
               retained.minimum_clearance + tolerance;
      };
    auto preserves_existing_hard_clearance = [](
        const PermanentClearanceQuality & candidate,
        const PermanentClearanceQuality & retained) {
        constexpr double tolerance = 1.0e-7;
        if (candidate.maximum_penetration_by_obstacle.size() !=
          retained.maximum_penetration_by_obstacle.size())
        {
          return false;
        }
        for (std::size_t obstacle_index = 0;
          obstacle_index < candidate.maximum_penetration_by_obstacle.size();
          ++obstacle_index)
        {
          // 已经达到target_clearance的永久点是不可回退的硬底线。
          // numeric_margin仍可在不同障碍间作为优化余量重新分配。
          if (retained.maximum_penetration_by_obstacle[obstacle_index] <= tolerance &&
            candidate.maximum_penetration_by_obstacle[obstacle_index] > tolerance)
          {
            return false;
          }
        }
        return true;
      };

    for (int repair_round = 0;
      repair_round < maximum_secondary_repair_rounds; ++repair_round)
    {
      // 本轮候选若没有改善，约束也必须一起回退；否则下一轮会在一组已经
      // 被拒绝的强化约束上继续做相同的全路线重算。
      const std::vector<CircleConstraint> retained_constraints = constraints;
      std::size_t added_this_round = 0;
      for (std::size_t obstacle_index = 0;
        obstacle_index < logic3_green_clusters_.size(); ++obstacle_index)
      {
        const bool dynamic_takeover =
          obstacle_index < logic3_green_dynamic_takeover_.size() &&
          logic3_green_dynamic_takeover_[obstacle_index];
        if (dynamic_takeover) {
          // 该ID已经明确切换为本次发车实时雷达接管，不能由二次审计
          // 偷偷恢复静态偏移；它的绿/橙雷达掩码仍按原逻辑保留。
          continue;
        }
        const auto & center = logic3_green_clusters_[obstacle_index].center;
        const std::size_t obstacle_first_deformable =
          obstacle_first_deformable_index(center);
        std::vector<bool> violating_segments(route_size - 1, false);
        for (std::size_t segment = obstacle_first_deformable;
          segment <= last_auditable_segment; ++segment)
        {
          const Point2D closest = closest_point_on_segment(
            center, best_route[segment], best_route[segment + 1]);
          if (std::hypot(
              closest.x - center.x, closest.y - center.y) + 1.0e-7 >=
            circle_envelope_target_clearance_ ||
            intentional_permanent_overlap_corridor(closest, obstacle_index))
          {
            continue;
          }
          violating_segments[segment] = true;
        }

        std::size_t segment = obstacle_first_deformable;
        while (segment <= last_auditable_segment) {
          if (!violating_segments[segment]) {
            ++segment;
            continue;
          }
          const std::size_t run_first_segment = segment;
          std::size_t run_last_segment = segment;
          while (run_last_segment + 1 <= last_auditable_segment &&
            violating_segments[run_last_segment + 1])
          {
            ++run_last_segment;
          }
          const std::size_t repair_first = run_first_segment;
          const std::size_t repair_last = run_last_segment + 1;
          const std::size_t represented_first =
            repair_first > obstacle_first_deformable + 2 ?
            repair_first - 2 : obstacle_first_deformable;
          const std::size_t represented_last = std::min(
            last_deformable_index, repair_last + 2);
          auto represented = std::find_if(
            constraints.begin(), constraints.end(),
            [&](const CircleConstraint & constraint) {
              if (std::hypot(
                  constraint.center.x - center.x,
                  constraint.center.y - center.y) > permanent_center_tolerance)
              {
                return false;
              }
              if (constraint.secondary_repair) {
                return constraint.repair_first <= repair_last &&
                       constraint.repair_last >= repair_first;
              }
              return constraint.source >= represented_first &&
                     constraint.source <= represented_last;
            });
          if (represented != constraints.end()) {
            if (!represented->secondary_repair) {
              // 主约束只会约束“原路线航点自身在圆内”的点。如果安全圆
              // 恰好落在两个航点之间，最终线段仍可能穿圆；二次审计已经
              // 用有限线段证实了这次侵入，必须把该主约束升级为有限区间
              // 修复，而不是因为同ID约束存在就直接跳过。
              represented->secondary_repair = true;
              represented->repair_first = repair_first;
              represented->repair_last = repair_last;
              represented->radius = std::max(
                represented->radius,
                circle_envelope_target_clearance_ +
                circle_envelope_numeric_margin_);
              represented->first_deformable = std::max(
                represented->first_deformable,
                obstacle_first_deformable);
              ++added_this_round;
              ++promoted_primary_repair_count;
            } else if (repair_first < represented->repair_first ||
              repair_last > represented->repair_last)
            {
              // 第一轮重算后侵圆连续段可能沿路线前后移动少量索引；只扩展
              // 已有局部核心，不新建同一ID的全局约束，也不改变其方向。
              represented->repair_first = std::min(
                represented->repair_first, repair_first);
              represented->repair_last = std::max(
                represented->repair_last, repair_last);
              ++added_this_round;
              ++expanded_secondary_repair_count;
            } else {
              // “已有二次约束且索引范围相同”不代表最终路线已经避开。
              // 若重算后同一连续段仍侵圆，继续向两侧扩展一个离散步长，
              // 同时增加接线长度；否则added_this_round会保持0，审计会把
              // 一条仍然穿圆的路线误判成无法再处理并提前结束。
              const double requested_first_s = std::max(
                route_s[obstacle_first_deformable],
                route_s[repair_first] - circle_envelope_extension_step_);
              const double requested_last_s = std::min(
                route_s[last_deformable_index],
                route_s[repair_last] + circle_envelope_extension_step_);
              const std::size_t reinforced_first = std::clamp<std::size_t>(
                static_cast<std::size_t>(std::distance(
                  route_s.begin(), std::lower_bound(
                    route_s.begin(), route_s.end(), requested_first_s))),
                obstacle_first_deformable, last_deformable_index);
              const std::size_t reinforced_last = std::clamp<std::size_t>(
                static_cast<std::size_t>(std::distance(
                  route_s.begin(), std::lower_bound(
                    route_s.begin(), route_s.end(), requested_last_s))),
                obstacle_first_deformable, last_deformable_index);
              const std::size_t previous_repair_first =
                represented->repair_first;
              const std::size_t previous_repair_last =
                represented->repair_last;
              const double previous_extension =
                represented->transition_extension;
              represented->repair_first = std::min(
                represented->repair_first, reinforced_first);
              represented->repair_last = std::max(
                represented->repair_last, reinforced_last);
              represented->transition_extension = std::min(
                circle_envelope_max_transition_length_ -
                circle_envelope_min_transition_length_,
                represented->transition_extension +
                circle_envelope_extension_step_);
              represented->radius = std::max(
                represented->radius,
                circle_envelope_target_clearance_ +
                circle_envelope_numeric_margin_);
              if (represented->repair_first < previous_repair_first ||
                represented->repair_last > previous_repair_last ||
                represented->transition_extension > previous_extension + 1.0e-12)
              {
                ++added_this_round;
                ++reinforced_secondary_repair_count;
              }
            }
            segment = run_last_segment + 1;
            continue;
          }

          std::size_t source_segment = run_first_segment;
          Point2D source_projection;
          double source_projection_ratio = 0.0;
          double minimum_original_distance_squared =
            std::numeric_limits<double>::infinity();
          for (std::size_t original_segment = run_first_segment;
            original_segment <= run_last_segment; ++original_segment)
          {
            const auto & start = original_waypoints_[original_segment];
            const auto & finish = original_waypoints_[original_segment + 1];
            const double segment_x = finish.x - start.x;
            const double segment_y = finish.y - start.y;
            const double length_squared =
              segment_x * segment_x + segment_y * segment_y;
            if (length_squared < 1.0e-12) {
              continue;
            }
            const double ratio = std::clamp(
              ((center.x - start.x) * segment_x +
              (center.y - start.y) * segment_y) / length_squared,
              0.0, 1.0);
            const Point2D projection{
              start.x + ratio * segment_x,
              start.y + ratio * segment_y};
            const double distance_squared =
              (projection.x - center.x) * (projection.x - center.x) +
              (projection.y - center.y) * (projection.y - center.y);
            if (distance_squared < minimum_original_distance_squared) {
              minimum_original_distance_squared = distance_squared;
              source_segment = original_segment;
              source_projection = projection;
              source_projection_ratio = ratio;
            }
          }
          std::size_t source = source_projection_ratio < 0.5 ?
            source_segment : source_segment + 1;
          source = std::clamp(
            source, obstacle_first_deformable, last_deformable_index);
          if (!std::isfinite(minimum_original_distance_squared) ||
            !normal_valid[source])
          {
            segment = run_last_segment + 1;
            continue;
          }

          const auto & source_normal = left_normals[source];
          const double obstacle_lateral =
            (center.x - source_projection.x) * source_normal.x +
            (center.y - source_projection.y) * source_normal.y;
          const int direction = obstacle_lateral >= 0.0 ? -1 : 1;
          const double repair_radius =
            circle_envelope_target_clearance_ +
            circle_envelope_numeric_margin_;
          const auto & source_base = original_waypoints_[source];
          const double center_x = center.x - source_base.x;
          const double center_y = center.y - source_base.y;
          const double center_normal =
            center_x * source_normal.x + center_y * source_normal.y;
          const double tangent_squared = std::max(
            0.0,
            center_x * center_x + center_y * center_y -
            center_normal * center_normal);
          double peak_offset = 0.0;
          if (tangent_squared <= repair_radius * repair_radius) {
            const double half_chord = std::sqrt(std::max(
              0.0, repair_radius * repair_radius - tangent_squared));
            const double signed_boundary = direction > 0 ?
              center_normal + half_chord : center_normal - half_chord;
            peak_offset = std::max(
              0.0, static_cast<double>(direction) * signed_boundary);
          }
          constraints.push_back(
            CircleConstraint{
              source, direction, center, repair_radius, peak_offset, 0.0,
              true, repair_first, repair_last,
              obstacle_first_deformable, false});
          ++added_this_round;
          ++secondary_repair_count;
          segment = run_last_segment + 1;
        }
      }

      if (added_this_round == 0) {
        break;
      }
      ++secondary_repair_round_count;
      std::sort(
        constraints.begin(), constraints.end(),
        [](const CircleConstraint & first, const CircleConstraint & second) {
          if (first.source != second.source) {
            return first.source < second.source;
          }
          if (first.center.x != second.center.x) {
            return first.center.x < second.center.x;
          }
          return first.center.y < second.center.y;
        });
      apply_shared_pass_side_locks(constraints);

      // 新一轮仍从未偏移路线批量生成候选，但必须保持每个已经通过
      // target_clearance的永久点继续合格，并且0.35m硬净距质量严格改善，
      // 才替换上一轮。0.42m规划余量和纯曲率改善都不能维持二次修复，
      // 避免安全已经等价时继续重画整条路线。失败候选直接丢弃，因此
      // 增加修复轮数也不会把已经合格的蓝线退回灰色原路线。
      const std::vector<Waypoint> retained_route = best_route;
      const CandidateQuality retained_quality = best_quality;
      const bool retained_accepted = accepted;
      const PermanentClearanceQuality retained_hard =
        evaluate_permanent_clearance(
        retained_route, circle_envelope_target_clearance_);
      std::vector<Waypoint> trial_route;
      CandidateQuality trial_quality;
      bool trial_accepted = false;
      search_best_route(
        constraints, trial_route, trial_quality, trial_accepted);
      const PermanentClearanceQuality trial_hard =
        evaluate_permanent_clearance(
        trial_route, circle_envelope_target_clearance_);
      const bool hard_clearance_preserved =
        preserves_existing_hard_clearance(trial_hard, retained_hard);
      if (hard_clearance_preserved &&
        permanent_quality_better(trial_hard, retained_hard))
      {
        best_route = std::move(trial_route);
        best_quality = trial_quality;
        accepted = trial_accepted;
      } else {
        constraints = retained_constraints;
        best_route = retained_route;
        best_quality = retained_quality;
        accepted = retained_accepted;
        ++rejected_secondary_replan_count;
      }
      RCLCPP_WARN(
        get_logger(),
        "circle_envelope二次净距审计第%zu轮：处理%zu个真实侵圆连续段；"
        "已按固定侧加入局部圆约束并统一圆滑重算；累计拒绝%zu次净距未改善候选。",
        secondary_repair_round_count, added_this_round,
        rejected_secondary_replan_count);
      // 一次无改善就说明当前固定绕行侧和接线模型无法通过继续强化获得
      // 更好路线。保留上一条有效偏移线，立即结束，禁止再空转十几轮。
      if (rejected_secondary_replan_count > 0) {
        break;
      }
    }

    // 圆边界与原路线法线在急弯处接近相切时，路线虽然仍沿原路线前进，
    // 相邻航点却可能被压得过密，形成离散曲率尖峰。这里把尖峰两侧锚点
    // 之间的坏采样点“逻辑删除”，用与当前航点数一致的等弧长三次Bezier
    // 点回填原索引；这样不破坏障碍来源、停车点等按索引保存的状态。
    // 每条回填候选都独立验收硬净距、0.15前进比例及原定绕行侧，不合格
    // 就封锁该局部并继续检查其他尖峰。因此它只能改善已保留的最优路线，
    // 不能把路线退回灰线，也不能用圆滑换取安全净距。
    const PermanentClearanceQuality fairing_reference_planning =
      evaluate_permanent_clearance(best_route, planning_clearance_radius);
    const PermanentClearanceQuality fairing_reference_hard =
      evaluate_permanent_clearance(
      best_route, circle_envelope_target_clearance_);
    std::vector<double> fairing_reference_side(route_size, 0.0);
    std::vector<double> original_curvature(route_size, 0.0);
    for (std::size_t index = 0; index < route_size; ++index) {
      if (normal_valid[index]) {
        fairing_reference_side[index] =
          (best_route[index].x - original_waypoints_[index].x) *
          left_normals[index].x +
          (best_route[index].y - original_waypoints_[index].y) *
          left_normals[index].y;
      }
      original_curvature[index] = discrete_curvature_at(
        original_waypoints_, index);
    }
    struct CurvatureExcessQuality
    {
      double total_squared_excess{0.0};
      double maximum_excess{0.0};
    };
    auto curvature_excess_quality = [&](const std::vector<Waypoint> & route) {
        CurvatureExcessQuality quality;
        for (std::size_t index = 1; index + 1 < route_size; ++index) {
          // 原路线本身的急弯不是本轮障碍规划造成的，不能为了追逐一个
          // 绝对2.2/m目标而改写整条基准路线；只压制明显高于原曲率的峰。
          const double allowed_curvature = std::max(
            circle_envelope_max_curvature_,
            1.5 * original_curvature[index] + 0.25);
          const double excess = std::max(
            0.0, discrete_curvature_at(route, index) - allowed_curvature);
          quality.total_squared_excess += excess * excess;
          quality.maximum_excess = std::max(quality.maximum_excess, excess);
        }
        return quality;
      };
    auto fairing_side_preserved = [&](const std::vector<Waypoint> & route) {
        constexpr double side_reference_threshold = 0.005;
        for (std::size_t index = 0; index < route_size; ++index) {
          if (!normal_valid[index] ||
            std::abs(fairing_reference_side[index]) <=
            side_reference_threshold)
          {
            continue;
          }
          const double candidate_side =
            (route[index].x - original_waypoints_[index].x) *
            left_normals[index].x +
            (route[index].y - original_waypoints_[index].y) *
            left_normals[index].y;
          if (candidate_side * fairing_reference_side[index] < -1.0e-8) {
            return false;
          }
        }
        for (const auto & constraint : constraints) {
          if (constraint.source >= route_size ||
            !normal_valid[constraint.source])
          {
            continue;
          }
          const double signed_side =
            static_cast<double>(constraint.direction) *
            ((route[constraint.source].x -
            original_waypoints_[constraint.source].x) *
            left_normals[constraint.source].x +
            (route[constraint.source].y -
            original_waypoints_[constraint.source].y) *
            left_normals[constraint.source].y);
          // 相交圆公共弦内，已验收路线在某个圆的局部法线定义下可能
          // 天然呈负值；不能要求回填候选突然跳回该圆的普通外侧。
          // 这种情况只允许保持或改善当前值，绝不允许进一步恶化。
          const double reference_signed_side =
            static_cast<double>(constraint.direction) *
            fairing_reference_side[constraint.source];
          const double minimum_signed_side = reference_signed_side < -1.0e-6 ?
            reference_signed_side - 1.0e-6 : -1.0e-6;
          if (signed_side < minimum_signed_side) {
            return false;
          }
        }
        return true;
      };
    const double fairing_clearance_allowance = std::min(
      0.002, 0.10 * std::max(0.0, circle_envelope_numeric_margin_));
    auto fairing_planning_clearance_preserved = [&] (
        const PermanentClearanceQuality & candidate) {
        if (candidate.maximum_penetration_by_obstacle.size() !=
          fairing_reference_planning.maximum_penetration_by_obstacle.size())
        {
          return false;
        }
        for (std::size_t obstacle_index = 0;
          obstacle_index < candidate.maximum_penetration_by_obstacle.size();
          ++obstacle_index)
        {
          if (candidate.maximum_penetration_by_obstacle[obstacle_index] >
            fairing_reference_planning.
            maximum_penetration_by_obstacle[obstacle_index] +
            fairing_clearance_allowance + 1.0e-7)
          {
            return false;
          }
        }
        return true;
      };

    // 尖峰只做少量局部“删除后等弧回填”。每个候选都要重新检查硬净距、
    // 固定侧和前进性，因此必须限制候选数，不能重新退化成全路线优化器。
    constexpr std::size_t maximum_fairing_hotspots = 4;
    const std::size_t maximum_fairing_iterations = std::min<std::size_t>(
      maximum_fairing_hotspots, route_size);
    const double fairing_bridge_shoulder = std::clamp(
      0.15 * circle_envelope_min_transition_length_, 0.20, 0.30);
    constexpr std::size_t fairing_block_radius = 2;
    auto build_equal_arc_fairing_bridge = [&] (
        const std::vector<Waypoint> & source_route,
        std::size_t hotspot, double shoulder, double handle_ratio,
        std::vector<Waypoint> & trial_route,
        std::size_t & fairing_first, std::size_t & fairing_last) {
        fairing_first = hotspot;
        while (fairing_first > 1 &&
          route_s[hotspot] - route_s[fairing_first] < shoulder)
        {
          --fairing_first;
        }
        fairing_last = hotspot;
        while (fairing_last + 1 < route_size - 1 &&
          route_s[fairing_last] - route_s[hotspot] < shoulder)
        {
          ++fairing_last;
        }
        if (fairing_last <= fairing_first + 3) {
          return false;
        }

        Point2D start_tangent{
          source_route[fairing_first].x -
          source_route[fairing_first - 1].x,
          source_route[fairing_first].y -
          source_route[fairing_first - 1].y};
        Point2D finish_tangent{
          source_route[fairing_last + 1].x -
          source_route[fairing_last].x,
          source_route[fairing_last + 1].y -
          source_route[fairing_last].y};
        const double start_tangent_length = std::hypot(
          start_tangent.x, start_tangent.y);
        const double finish_tangent_length = std::hypot(
          finish_tangent.x, finish_tangent.y);
        const double chord_length = std::hypot(
          source_route[fairing_last].x - source_route[fairing_first].x,
          source_route[fairing_last].y - source_route[fairing_first].y);
        if (start_tangent_length < 1.0e-9 ||
          finish_tangent_length < 1.0e-9 || chord_length < 1.0e-6)
        {
          return false;
        }
        start_tangent.x /= start_tangent_length;
        start_tangent.y /= start_tangent_length;
        finish_tangent.x /= finish_tangent_length;
        finish_tangent.y /= finish_tangent_length;

        const Point2D p0{
          source_route[fairing_first].x, source_route[fairing_first].y};
        const double handle_length = handle_ratio * chord_length;
        const Point2D p1{
          p0.x + handle_length * start_tangent.x,
          p0.y + handle_length * start_tangent.y};
        const Point2D p3{
          source_route[fairing_last].x, source_route[fairing_last].y};
        const Point2D p2{
          p3.x - handle_length * finish_tangent.x,
          p3.y - handle_length * finish_tangent.y};
        const std::size_t sample_count = std::max<std::size_t>(
          64, 8 * (fairing_last - fairing_first));
        std::vector<Point2D> bridge_samples(sample_count + 1);
        std::vector<double> bridge_sample_s(sample_count + 1, 0.0);
        for (std::size_t sample = 0; sample <= sample_count; ++sample) {
          const double t = static_cast<double>(sample) /
            static_cast<double>(sample_count);
          const double u = 1.0 - t;
          const double w0 = u * u * u;
          const double w1 = 3.0 * u * u * t;
          const double w2 = 3.0 * u * t * t;
          const double w3 = t * t * t;
          bridge_samples[sample] = Point2D{
            w0 * p0.x + w1 * p1.x + w2 * p2.x + w3 * p3.x,
            w0 * p0.y + w1 * p1.y + w2 * p2.y + w3 * p3.y};
          if (sample > 0) {
            bridge_sample_s[sample] = bridge_sample_s[sample - 1] +
              std::hypot(
              bridge_samples[sample].x - bridge_samples[sample - 1].x,
              bridge_samples[sample].y - bridge_samples[sample - 1].y);
          }
        }
        if (bridge_sample_s.back() < 1.0e-6) {
          return false;
        }

        trial_route = source_route;
        const double bridge_interval_count = static_cast<double>(
          fairing_last - fairing_first);
        for (std::size_t index = fairing_first + 1;
          index < fairing_last; ++index)
        {
          const double target_s = bridge_sample_s.back() *
            static_cast<double>(index - fairing_first) /
            bridge_interval_count;
          auto upper = std::lower_bound(
            bridge_sample_s.begin(), bridge_sample_s.end(), target_s);
          std::size_t upper_index = static_cast<std::size_t>(std::distance(
            bridge_sample_s.begin(), upper));
          upper_index = std::clamp<std::size_t>(
            upper_index, 1, sample_count);
          const std::size_t lower_index = upper_index - 1;
          const double sample_span =
            bridge_sample_s[upper_index] - bridge_sample_s[lower_index];
          const double sample_ratio = sample_span > 1.0e-12 ?
            std::clamp(
            (target_s - bridge_sample_s[lower_index]) / sample_span,
            0.0, 1.0) : 0.0;
          trial_route[index].x =
            (1.0 - sample_ratio) * bridge_samples[lower_index].x +
            sample_ratio * bridge_samples[upper_index].x;
          trial_route[index].y =
            (1.0 - sample_ratio) * bridge_samples[lower_index].y +
            sample_ratio * bridge_samples[upper_index].y;
        }
        return true;
      };

    // 主偏移继续做16轮一维圆滑；这里只补最多4个残余尖峰，避免重复优化。
    {
      std::vector<bool> fairing_blocked(route_size, false);
      for (std::size_t iteration = 0;
        iteration < maximum_fairing_iterations; ++iteration)
      {
        std::size_t hotspot = route_size;
        double hotspot_excess = 1.0e-6;
        for (std::size_t index = 1; index + 1 < route_size; ++index) {
          if (fairing_blocked[index]) {
            continue;
          }
          const double allowed_curvature = std::max(
            circle_envelope_max_curvature_,
            1.5 * original_curvature[index] + 0.25);
          const double excess =
            discrete_curvature_at(best_route, index) - allowed_curvature;
          if (excess > hotspot_excess) {
            hotspot_excess = excess;
            hotspot = index;
          }
        }
        if (hotspot == route_size) {
          break;
        }

        const std::vector<std::pair<double, double>> bridge_variants{
          {fairing_bridge_shoulder, 0.35}};

        const CurvatureExcessQuality retained_shape =
          curvature_excess_quality(best_route);
        bool selected = false;
        std::vector<Waypoint> selected_route;
        CurvatureExcessQuality selected_shape;
        selected_shape.total_squared_excess =
          std::numeric_limits<double>::infinity();
        selected_shape.maximum_excess =
          std::numeric_limits<double>::infinity();
        for (const auto & variant : bridge_variants) {
          std::vector<Waypoint> trial_route;
          std::size_t fairing_first = 0;
          std::size_t fairing_last = 0;
          if (!build_equal_arc_fairing_bridge(
              best_route, hotspot, variant.first, variant.second,
              trial_route, fairing_first, fairing_last))
          {
            continue;
          }

          const PermanentClearanceQuality trial_hard =
            evaluate_permanent_clearance(
            trial_route, circle_envelope_target_clearance_);
          const PermanentClearanceQuality trial_planning =
            evaluate_permanent_clearance(
            trial_route, planning_clearance_radius);
          const CurvatureExcessQuality trial_shape =
            curvature_excess_quality(trial_route);
          const std::size_t region_first =
            fairing_first > 1 ? fairing_first - 1 : 1;
          const std::size_t region_last = std::min(
            route_size - 2, fairing_last + 1);
          double retained_region_excess = 0.0;
          double trial_region_excess = 0.0;
          for (std::size_t index = region_first;
            index <= region_last; ++index)
          {
            const double allowed_curvature = std::max(
              circle_envelope_max_curvature_,
              1.5 * original_curvature[index] + 0.25);
            retained_region_excess = std::max(
              retained_region_excess,
              discrete_curvature_at(best_route, index) - allowed_curvature);
            trial_region_excess = std::max(
              trial_region_excess,
              discrete_curvature_at(trial_route, index) - allowed_curvature);
          }
          const bool valid_fairing =
            progress_violation_count(trial_route) == 0 &&
            fairing_side_preserved(trial_route) &&
            preserves_existing_hard_clearance(
              trial_hard, fairing_reference_hard) &&
            fairing_planning_clearance_preserved(trial_planning) &&
            trial_region_excess + 1.0e-7 < retained_region_excess &&
            trial_shape.maximum_excess <=
            retained_shape.maximum_excess + 1.0e-7 &&
            trial_shape.total_squared_excess + 1.0e-9 <
            retained_shape.total_squared_excess;
          if (!valid_fairing) {
            continue;
          }
          if (!selected ||
            trial_shape.maximum_excess + 1.0e-7 <
            selected_shape.maximum_excess ||
            (std::abs(
              trial_shape.maximum_excess - selected_shape.maximum_excess) <=
            1.0e-7 &&
            trial_shape.total_squared_excess + 1.0e-9 <
            selected_shape.total_squared_excess))
          {
            selected = true;
            selected_route = std::move(trial_route);
            selected_shape = trial_shape;
          }
        }
        if (selected) {
          best_route = std::move(selected_route);
          ++route_fairing_accept_count;
          continue;
        }

        const std::size_t blocked_first =
          hotspot > fairing_block_radius ? hotspot - fairing_block_radius : 1;
        const std::size_t blocked_last = std::min(
          route_size - 2, hotspot + fairing_block_radius);
        for (std::size_t index = blocked_first;
          index <= blocked_last; ++index)
        {
          fairing_blocked[index] = true;
        }
        ++route_fairing_reject_count;
      }
    }

    // 最后只按真实绿/橙点逐线段验收，不再用“约束已经存在”代替净距。
    // 数值裕量用于生成，验收仍严格采用用户配置的target_clearance。
    for (std::size_t obstacle_index = 0;
      obstacle_index < logic3_green_clusters_.size(); ++obstacle_index)
    {
      const bool dynamic_takeover =
        obstacle_index < logic3_green_dynamic_takeover_.size() &&
        logic3_green_dynamic_takeover_[obstacle_index];
      if (dynamic_takeover) {
        continue;
      }
      const auto & center = logic3_green_clusters_[obstacle_index].center;
      const std::size_t obstacle_first_deformable =
        obstacle_first_deformable_index(center);
      bool obstacle_failed = false;
      for (std::size_t segment = obstacle_first_deformable;
        segment < route_size - 1; ++segment)
      {
        const Point2D closest = closest_point_on_segment(
          center, best_route[segment], best_route[segment + 1]);
        if (std::hypot(
            closest.x - center.x, closest.y - center.y) + 1.0e-7 <
          circle_envelope_target_clearance_ &&
          !intentional_permanent_overlap_corridor(closest, obstacle_index))
        {
          obstacle_failed = true;
          break;
        }
      }
      if (obstacle_failed) {
        ++residual_permanent_clearance_failures;
        if (obstacle_index < logic3_green_route_clearance_failed_.size()) {
          logic3_green_route_clearance_failed_[obstacle_index] = true;
        }
      }
    }

    // 二次审计可能补入新约束，最终按真实永久点再做一次硬净距统计。
    const std::vector<double> final_margins = clearance_margins(
      best_route, constraints);
    best_quality.clearance_failures = 0;
    best_quality.minimum_clearance_margin =
      std::numeric_limits<double>::infinity();
    for (const double margin : final_margins) {
      best_quality.minimum_clearance_margin = std::min(
        best_quality.minimum_clearance_margin, margin);
      if (margin < -1.0e-7) {
        ++best_quality.clearance_failures;
      }
    }
    best_quality.progress_violations = progress_violation_count(best_route);
    best_quality.maximum_curvature = maximum_discrete_curvature(best_route);
    accepted = residual_permanent_clearance_failures == 0 &&
      best_quality.progress_violations == 0 &&
      best_quality.maximum_curvature <= circle_envelope_max_curvature_;
  }

  shifted_route = std::move(best_route);
  // READY预规划是发车前的纯计算候选：不发布调试图，也不用这些
  // 过程信息刷屏。真正冻结或运行中重规划仍保留原有日志。
  if (!logic3_ready_preplanning_active_ && !shared_pass_geometries.empty()) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "circle_envelope共用通道：识别%zu组正反重复经过、%zu对同位航点；"
      "仅统一障碍世界绕行侧，不再强制平均两次路线坐标。",
      shared_pass_geometries.size(), shared_pass_waypoint_pair_count);
  }
  if (!logic3_ready_preplanning_active_ && best_quality.clearance_failures > 0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "circle_envelope第一次生成及局部复查后仍有%zu个独立安全圆净距不足；"
      "没有合并障碍物或翻转方向，继续输出当前最优路线。",
      best_quality.clearance_failures);
  }
  if (!logic3_ready_preplanning_active_ && residual_permanent_clearance_failures > 0) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "circle_envelope二次净距审计后仍有%zu个永久障碍不满足target_clearance；"
      "保留当前最外推路线且不停车、不回退原路线；这些ID本轮解除绿/橙掩码，"
      "继续由实时雷达执行偏移。",
      residual_permanent_clearance_failures);
  }
  if (!logic3_ready_preplanning_active_ && best_quality.anchor_conflicts > 0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "circle_envelope有%zu个非相交圆路线点同时受到相反约束；"
      "障碍物保持独立，未换向。",
      best_quality.anchor_conflicts);
  }
  if (!logic3_ready_preplanning_active_ && best_quality.progress_violations > 0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "circle_envelope路线仍有%zu个局部点对原路线进度反向；"
      "局部Bezier桥未通过净距或方向校验，保留橙色基线路线。",
      best_quality.progress_violations);
  }
  if (!logic3_ready_preplanning_active_ && !accepted &&
    best_quality.maximum_curvature > circle_envelope_max_curvature_)
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "circle_envelope最大离散曲率%.3f仍高于软目标%.3f；"
      "安全圆、独立方向和原路线进度优先，继续使用该路线。",
      best_quality.maximum_curvature, circle_envelope_max_curvature_);
  }
  if (!logic3_ready_preplanning_active_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "circle_envelope生成：第一次约束=%zu，二次新增=%zu，主约束升级=%zu，"
      "区间扩展=%zu，同段强化=%zu(轮次=%zu)，"
      "最终约束=%zu，进度反向局部桥=%zu，尖峰等距回填=%zu/%zu，净距不足=%zu，"
      "非相交冲突=%zu，相交圆公共弦点=%zu，进度反向=%zu，"
      "最大曲率=%.3f。",
      primary_constraint_count, secondary_repair_count,
      promoted_primary_repair_count, expanded_secondary_repair_count,
      reinforced_secondary_repair_count, secondary_repair_round_count,
      constraints.size(),
      best_quality.local_bridge_count,
      route_fairing_accept_count, route_fairing_reject_count,
      best_quality.clearance_failures,
      best_quality.anchor_conflicts, best_quality.overlap_corridor_points,
      best_quality.progress_violations,
      best_quality.maximum_curvature);
  }
}

bool VisualLidarPathFollower::is_static_map_point(
  const VisualLidarPathFollower::RobotPose & robot_pose,
  const VisualLidarPathFollower::Point2D & base_point) const
{
  return is_static_map_point(
    robot_pose, base_point, latest_restriction_map_,
    static_map_inflation_cells_);
}

bool VisualLidarPathFollower::is_static_map_point(
  const VisualLidarPathFollower::RobotPose & robot_pose,
  const VisualLidarPathFollower::Point2D & base_point,
  const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & restriction_map,
  int restriction_inflation_cells) const
{
  if (!enable_static_map_filter_) {
    return false;
  }

  const Point2D map_point = base_to_map(robot_pose, base_point);
  auto occupied_in_map = [&](
      const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & map_ptr,
      int occupied_threshold, int inflation_cells,
      bool outside_is_occupied) {
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
        return outside_is_occupied;
      }

      for (int offset_y = -inflation_cells;
        offset_y <= inflation_cells; ++offset_y)
      {
        for (int offset_x = -inflation_cells;
          offset_x <= inflation_cells; ++offset_x)
        {
          if (offset_x * offset_x + offset_y * offset_y >
            inflation_cells * inflation_cells)
          {
            continue;
          }
          const int check_x = grid_x + offset_x;
          const int check_y = grid_y + offset_y;
          if (check_x < 0 || check_y < 0 || check_x >= width || check_y >= height) {
            if (outside_is_occupied) {
              return true;
            }
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

  const bool occupied_in_physical_map = occupied_in_map(
    latest_map_, static_map_occupied_threshold_, static_map_inflation_cells_, true);
  const bool occupied_in_restriction_map = occupied_in_map(
    restriction_map, waypoint_shift_occupied_threshold_,
    restriction_inflation_cells, false);
  return occupied_in_physical_map || occupied_in_restriction_map;
}

std::vector<VisualLidarPathFollower::ScanPointCluster>
VisualLidarPathFollower::collect_non_static_scan_clusters_map(
  const VisualLidarPathFollower::RobotPose & robot_pose,
  int minimum_cluster_points,
  bool use_logic3_static_restriction_map,
  bool filter_logic3_green_mask,
  VisualLidarPathFollower::RobotPose * observation_pose_out) const
{
  if (!latest_scan_) {
    return {};
  }
  const bool use_logic3_runtime_restriction_map =
    logic3_planning_pose_anchored_ && !use_logic3_static_restriction_map;
  const auto & active_restriction_map = use_logic3_static_restriction_map ?
    latest_logic3_static_restriction_map_ :
    (use_logic3_runtime_restriction_map ?
    latest_logic3_runtime_restriction_map_ : latest_restriction_map_);
  const int active_restriction_inflation_cells =
    use_logic3_static_restriction_map ?
    logic3_static_restriction_inflation_cells_ : 0;
  if (enable_static_map_filter_ && require_map_for_lidar_assist_ &&
    (!latest_map_ || !active_restriction_map))
  {
    return {};
  }

  const auto & scan = *latest_scan_;
  const rclcpp::Time scan_stamp(scan.header.stamp, get_clock()->get_clock_type());
  const rclcpp::Time lookup_stamp = scan_stamp.nanoseconds() == 0 ?
    rclcpp::Time(0, 0, get_clock()->get_clock_type()) : scan_stamp;
  RobotPose observation_pose = robot_pose;
  // 发车后车辆在运动，不能用控制周期“现在”的base_link位姿给稍早的
  // 雷达帧投影。按scan采样时刻取得map->base_link，避免12Hz雷达在高速
  // 行驶和转弯时给障碍坐标附加一段车身位移。
  if (logic3_planning_pose_anchored_ &&
    !lookup_pose(frame_id_, base_frame_id_, lookup_stamp, observation_pose))
  {
    return {};
  }
  if (observation_pose_out) {
    *observation_pose_out = observation_pose;
  }
  RobotPose scan_to_base;
  if (!scan.header.frame_id.empty() && scan.header.frame_id != base_frame_id_ &&
    !lookup_pose(
      base_frame_id_, scan.header.frame_id,
      lookup_stamp, scan_to_base))
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
  const std::size_t required_points = static_cast<std::size_t>(
    std::max(1, minimum_cluster_points));

  auto finish_cluster = [&]() {
      if (cluster_active &&
        current_cluster.points.size() >= required_points)
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
    if (enable_static_map_filter_ && is_static_map_point(
        observation_pose, base_point, active_restriction_map,
        active_restriction_inflation_cells))
    {
      finish_cluster();
      angle += scan.angle_increment;
      continue;
    }
    const Point2D map_point = base_to_map(observation_pose, base_point);
    if (filter_logic3_green_mask && logic3_green_mask_contains(map_point)) {
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
    current_cluster.points.push_back(map_point);
    previous_base_point = base_point;
    previous_range = point_range;
    angle += scan.angle_increment;
  }
  finish_cluster();
  return clusters;
}

bool VisualLidarPathFollower::latest_lidar_scan_is_fresh() const
{
  if (!latest_scan_) {
    return false;
  }
  const double age = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - latest_scan_received_at_).count();
  return age >= 0.0 && age <= lidar_scan_timeout_;
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

bool VisualLidarPathFollower::logic3_runtime_green_steering_is_stable(
  const sensor_msgs::msg::LaserScan & scan, double * steering_span) const
{
  if (steering_span) {
    *steering_span = std::numeric_limits<double>::infinity();
  }
  if (logic3_command_history_.empty()) {
    return false;
  }

  const std::int64_t scan_start_nanoseconds = rclcpp::Time(
    scan.header.stamp, get_clock()->get_clock_type()).nanoseconds();
  double scan_duration_seconds = 0.0;
  if (scan.ranges.size() > 1 && std::isfinite(scan.time_increment) &&
    scan.time_increment > 0.0F)
  {
    scan_duration_seconds = static_cast<double>(scan.ranges.size() - 1) *
      static_cast<double>(scan.time_increment);
  } else if (std::isfinite(scan.scan_time) && scan.scan_time > 0.0F) {
    scan_duration_seconds = static_cast<double>(scan.scan_time);
  } else {
    return false;
  }
  const std::int64_t scan_end_nanoseconds = scan_start_nanoseconds +
    static_cast<std::int64_t>(std::llround(scan_duration_seconds * 1.0e9));

  bool have_start_command = false;
  double minimum_steering = 0.0;
  double maximum_steering = 0.0;
  for (const auto & sample : logic3_command_history_) {
    if (sample.stamp_nanoseconds <= scan_start_nanoseconds) {
      if (!std::isfinite(sample.steering_angle)) {
        return false;
      }
      minimum_steering = sample.steering_angle;
      maximum_steering = sample.steering_angle;
      have_start_command = true;
      continue;
    }
    if (sample.stamp_nanoseconds > scan_end_nanoseconds) {
      break;
    }
    if (!have_start_command || !std::isfinite(sample.steering_angle)) {
      return false;
    }
    minimum_steering = std::min(minimum_steering, sample.steering_angle);
    maximum_steering = std::max(maximum_steering, sample.steering_angle);
  }
  if (!have_start_command) {
    return false;
  }

  const double measured_span = maximum_steering - minimum_steering;
  if (steering_span) {
    *steering_span = measured_span;
  }
  // Only the change is limited: a constant non-zero steering command (a
  // stable arc) passes exactly like a constant zero command (a straight line).
  return measured_span <= logic3_runtime_green_max_steering_span_;
}

bool VisualLidarPathFollower::logic3_runtime_green_speed_is_stable(
  const sensor_msgs::msg::LaserScan & scan,
  double * minimum_abs_speed,
  double * maximum_speed_step) const
{
  if (minimum_abs_speed) {
    *minimum_abs_speed = std::numeric_limits<double>::infinity();
  }
  if (maximum_speed_step) {
    *maximum_speed_step = std::numeric_limits<double>::infinity();
  }
  if (logic3_command_history_.empty()) {
    return false;
  }

  const std::int64_t scan_start_nanoseconds = rclcpp::Time(
    scan.header.stamp, get_clock()->get_clock_type()).nanoseconds();
  double scan_duration_seconds = 0.0;
  if (scan.ranges.size() > 1 && std::isfinite(scan.time_increment) &&
    scan.time_increment > 0.0F)
  {
    scan_duration_seconds = static_cast<double>(scan.ranges.size() - 1) *
      static_cast<double>(scan.time_increment);
  } else if (std::isfinite(scan.scan_time) && scan.scan_time > 0.0F) {
    scan_duration_seconds = static_cast<double>(scan.scan_time);
  } else {
    return false;
  }
  const std::int64_t scan_end_nanoseconds = scan_start_nanoseconds +
    static_cast<std::int64_t>(std::llround(scan_duration_seconds * 1.0e9));

  bool have_start_command = false;
  double previous_speed = 0.0;
  double measured_minimum_abs_speed = std::numeric_limits<double>::infinity();
  double measured_maximum_speed_step = 0.0;
  for (const auto & sample : logic3_command_history_) {
    if (sample.stamp_nanoseconds <= scan_start_nanoseconds) {
      if (!std::isfinite(sample.linear_speed)) {
        return false;
      }
      previous_speed = sample.linear_speed;
      have_start_command = true;
      continue;
    }
    if (sample.stamp_nanoseconds > scan_end_nanoseconds) {
      break;
    }
    if (!have_start_command || !std::isfinite(sample.linear_speed)) {
      return false;
    }
    measured_minimum_abs_speed = std::min(
      measured_minimum_abs_speed, std::abs(previous_speed));
    measured_maximum_speed_step = std::max(
      measured_maximum_speed_step,
      std::abs(sample.linear_speed - previous_speed));
    previous_speed = sample.linear_speed;
    measured_minimum_abs_speed = std::min(
      measured_minimum_abs_speed, std::abs(previous_speed));
  }
  if (!have_start_command) {
    return false;
  }
  if (!std::isfinite(measured_minimum_abs_speed)) {
    // 一圈雷达内没有新的速度消息时，使用扫描开始时仍在生效的指令；
    // 相邻发布跳变量为0。
    measured_minimum_abs_speed = std::abs(previous_speed);
  }

  if (minimum_abs_speed) {
    *minimum_abs_speed = measured_minimum_abs_speed;
  }
  if (maximum_speed_step) {
    *maximum_speed_step = measured_maximum_speed_step;
  }
  return measured_minimum_abs_speed + 1.0e-9 >=
         logic3_runtime_green_min_speed_ &&
         measured_maximum_speed_step <=
         logic3_runtime_green_max_speed_step_ + 1.0e-9;
}

void VisualLidarPathFollower::record_logic3_runtime_green_gate_log_sample(
  double steering_span, bool steering_measurement_valid,
  double minimum_abs_speed, double maximum_speed_step,
  bool speed_measurement_valid,
  double map_odom_translation_correction_abs,
  double map_odom_yaw_correction_abs_deg,
  bool steering_within_limit, bool speed_within_limits,
  bool translation_within_limit, bool yaw_within_limit)
{
  if (logic3_runtime_green_gate_log_rate_hz_ <= 0.0) {
    return;
  }

  const auto sample_time = std::chrono::steady_clock::now();
  auto & stats = logic3_runtime_green_gate_log_stats_;
  if (!stats.window_started) {
    stats.window_started = true;
    stats.window_started_at = sample_time;
  }

  ++stats.scan_count;
  stats.translation_correction_abs_sum += map_odom_translation_correction_abs;
  stats.yaw_correction_abs_deg_sum += map_odom_yaw_correction_abs_deg;
  if (!steering_measurement_valid) {
    ++stats.steering_invalid_count;
  } else {
    ++stats.steering_valid_count;
    stats.steering_span_sum += steering_span;
  }
  if (!speed_measurement_valid) {
    ++stats.speed_invalid_count;
  } else {
    ++stats.speed_valid_count;
    stats.minimum_abs_speed_sum += minimum_abs_speed;
    stats.maximum_speed_step_sum += maximum_speed_step;
  }
  if (steering_measurement_valid && speed_measurement_valid) {
    const int rejection_count =
      static_cast<int>(!steering_within_limit) +
      static_cast<int>(!speed_within_limits) +
      static_cast<int>(!translation_within_limit) +
      static_cast<int>(!yaw_within_limit);
    if (rejection_count == 0) {
      ++stats.motion_within_limits_count;
    } else if (rejection_count > 1) {
      ++stats.both_reject_count;
    } else if (!steering_within_limit) {
      ++stats.steering_reject_count;
    } else if (!speed_within_limits) {
      ++stats.speed_reject_count;
    } else if (!translation_within_limit) {
      ++stats.translation_reject_count;
    } else {
      ++stats.yaw_reject_count;
    }
  }

  const double elapsed_seconds =
    std::chrono::duration<double>(sample_time - stats.window_started_at).count();
  const double log_period_seconds = 1.0 / logic3_runtime_green_gate_log_rate_hz_;
  if (elapsed_seconds < log_period_seconds) {
    return;
  }

  const double average_steering_span = stats.steering_valid_count > 0 ?
    stats.steering_span_sum / static_cast<double>(stats.steering_valid_count) :
    std::numeric_limits<double>::quiet_NaN();
  const double average_minimum_abs_speed = stats.speed_valid_count > 0 ?
    stats.minimum_abs_speed_sum / static_cast<double>(stats.speed_valid_count) :
    std::numeric_limits<double>::quiet_NaN();
  const double average_maximum_speed_step = stats.speed_valid_count > 0 ?
    stats.maximum_speed_step_sum / static_cast<double>(stats.speed_valid_count) :
    std::numeric_limits<double>::quiet_NaN();
  const double average_translation_correction_abs =
    stats.translation_correction_abs_sum / static_cast<double>(stats.scan_count);
  const double average_yaw_correction_abs_deg =
    stats.yaw_correction_abs_deg_sum / static_cast<double>(stats.scan_count);
  const char * steering_gate_result = !steering_measurement_valid ? "无数据" :
    (steering_within_limit ? "通过" : "拒绝");
  const char * speed_gate_result = !speed_measurement_valid ? "无数据" :
    (speed_within_limits ? "通过" : "拒绝");
  RCLCPP_INFO(
    get_logger(),
    "运行绿/橙点运动统计[%.2fs]：阶段=%s，雷达帧=%llu，"
    "转角跨度当前/平均=%.3f/%.3frad（上限%.3f），"
    "最低速度幅值当前/平均=%.3f/%.3fm/s（下限%.3f），"
    "相邻发布速度跳变当前/平均=%.3f/%.3fm/s（上限%.3f），"
    "map->odom平移修正当前/平均=%.3f/%.3fm（上限%.3f），"
    "map->odom航向修正当前/平均=%.2f/%.2fdeg（上限%.2f），"
    "本帧[转向=%s，速度=%s，平移=%s，航向=%s]，"
    "阈值内=%llu/%llu，拒绝[转向=%llu，速度=%llu，平移=%llu，"
    "航向=%llu，多项=%llu，转向无数据=%llu，速度无数据=%llu]，"
    "运动门槛启用=%s。",
    elapsed_seconds, task_phase_name(task_phase_),
    static_cast<unsigned long long>(stats.scan_count),
    steering_span, average_steering_span,
    logic3_runtime_green_max_steering_span_,
    minimum_abs_speed, average_minimum_abs_speed,
    logic3_runtime_green_min_speed_,
    maximum_speed_step, average_maximum_speed_step,
    logic3_runtime_green_max_speed_step_,
    map_odom_translation_correction_abs, average_translation_correction_abs,
    logic3_runtime_green_max_map_odom_translation_correction_,
    map_odom_yaw_correction_abs_deg, average_yaw_correction_abs_deg,
    logic3_runtime_green_max_map_odom_yaw_correction_deg_,
    steering_gate_result,
    speed_gate_result,
    translation_within_limit ? "通过" : "拒绝",
    yaw_within_limit ? "通过" : "拒绝",
    static_cast<unsigned long long>(stats.motion_within_limits_count),
    static_cast<unsigned long long>(stats.scan_count),
    static_cast<unsigned long long>(stats.steering_reject_count),
    static_cast<unsigned long long>(stats.speed_reject_count),
    static_cast<unsigned long long>(stats.translation_reject_count),
    static_cast<unsigned long long>(stats.yaw_reject_count),
    static_cast<unsigned long long>(stats.both_reject_count),
    static_cast<unsigned long long>(stats.steering_invalid_count),
    static_cast<unsigned long long>(stats.speed_invalid_count),
    logic3_runtime_green_motion_gate_enabled_ ? "是" : "否");

  stats = Logic3RuntimeGreenGateLogStats{};
  // Keep the rate-limit clock continuous across statistic windows. Otherwise
  // the first scan after every log only restarts the clock and high log rates
  // can never reach the actual diagnostic scan rate.
  stats.window_started = true;
  stats.window_started_at = sample_time;
}

// 为 QR 倒车和 channel 全程避障准备同一帧雷达的坐标变换。
// 墙体、静态+动态方案运行限制图或绿色掩码需要 map 坐标时，采样时刻 TF 缺失视为数据未就绪。
bool VisualLidarPathFollower::prepare_lidar_scan(
  VisualLidarPathFollower::RobotPose & scan_to_base,
  VisualLidarPathFollower::RobotPose & scan_to_map,
  bool require_map_transform,
  bool require_logic3_runtime_map) const
{
  scan_to_base = RobotPose();
  scan_to_map = RobotPose();
  if (!latest_lidar_scan_is_fresh()) {
    return false;
  }

  const auto & scan = *latest_scan_;
  const std::string source_frame =
    scan.header.frame_id.empty() ? base_frame_id_ : scan.header.frame_id;
  const rclcpp::Time scan_stamp(scan.header.stamp, get_clock()->get_clock_type());
  const rclcpp::Time lookup_stamp = scan_stamp.nanoseconds() == 0 ?
    rclcpp::Time(0, 0, get_clock()->get_clock_type()) : scan_stamp;

  if (source_frame != base_frame_id_ &&
    !lookup_pose(base_frame_id_, source_frame, lookup_stamp, scan_to_base))
  {
    return false;
  }
  const bool needs_map_transform =
    lidar_enable_static_map_filter_ || require_map_transform;
  if (!needs_map_transform) {
    return true;
  }
  if (lidar_enable_static_map_filter_) {
    if (!latest_map_) {
      return false;
    }
    const auto & map = *latest_map_;
    if (map.info.width == 0 || map.info.height == 0 || map.info.resolution <= 0.0 ||
      map.data.size() < static_cast<std::size_t>(map.info.width) * map.info.height)
    {
      return false;
    }
  }
  if (require_logic3_runtime_map) {
    if (!latest_logic3_runtime_restriction_map_) {
      return false;
    }
    const auto & map = *latest_logic3_runtime_restriction_map_;
    if (map.info.width == 0 || map.info.height == 0 || map.info.resolution <= 0.0 ||
      map.data.size() < static_cast<std::size_t>(map.info.width) * map.info.height)
    {
      return false;
    }
  }
  if (source_frame == frame_id_) {
    return true;
  }
  return lookup_pose(frame_id_, source_frame, lookup_stamp, scan_to_map);
}

bool VisualLidarPathFollower::task1_static_avoid_region_contains(
  const VisualLidarPathFollower::Point2D & map_point) const
{
  return map_point.x > task1_static_avoid_min_x_ &&
         map_point.x < task1_static_avoid_max_x_ &&
         map_point.y > task1_static_avoid_min_y_ &&
         map_point.y < task1_static_avoid_max_y_;
}

bool VisualLidarPathFollower::logic3_green_mask_contains(
  const VisualLidarPathFollower::Point2D & map_point) const
{
  if (tracking_controller_mode_ == "smac_pid") {
    // smac_pid障碍策略下，正常完成静态偏移的绿/橙永久点保持完整掩码。
    // 只有circle_envelope最终硬验收仍失败的ID临时解除掩码，让实时雷达
    // 接续偏移；静态路线本身继续保留最后一次最外推结果，不回退原路线。
    const double radius_squared =
      logic3_green_red_filter_radius_ * logic3_green_red_filter_radius_;
    for (std::size_t index = 0; index < logic3_green_clusters_.size(); ++index) {
      if (index < logic3_green_route_clearance_failed_.size() &&
        logic3_green_route_clearance_failed_[index])
      {
        continue;
      }
      const auto & recorded_cluster = logic3_green_clusters_[index];
      const double dx = map_point.x - recorded_cluster.center.x;
      const double dy = map_point.y - recorded_cluster.center.y;
      if (dx * dx + dy * dy <= radius_squared) {
        return true;
      }
    }
    return false;
  }
  const double radius_squared =
    logic3_green_red_filter_radius_ * logic3_green_red_filter_radius_;
  for (std::size_t index = 0; index < logic3_green_clusters_.size(); ++index) {
    if (index < logic3_green_route_clearance_failed_.size() &&
      logic3_green_route_clearance_failed_[index])
    {
      continue;
    }
    const auto & green_cluster = logic3_green_clusters_[index];
    // Task1静态矩形外的冻结绿色簇不做静态路线偏移，因此也不能被
    // 绿圈掩码过滤；保留给实时雷达动态避障。A→C→B动态专用阶段由调用方
    // 完全关闭本掩码，进入Task2后再恢复。
    if ((following_qr_path_ || task_phase_ == TaskPhase::TASK1) &&
      !task1_static_avoid_region_contains(green_cluster.center))
    {
      continue;
    }
    // 进入返程段后，分界线以下的整个绿/橙永久障碍簇交给实时雷达，不能继续
    // 被冻结绿色掩码吃掉。按簇中心分类，避免同一个锥桶跨线时被拆开。
    // 上方障碍仍完整保留掩码，即使其点云或偏移尾段延伸到分界线以下。
    if (task3_route_active_ &&
      green_cluster.center.y <= return_obstacle_split_y_)
    {
      continue;
    }
    const double dx = map_point.x - green_cluster.center.x;
    const double dy = map_point.y - green_cluster.center.y;
    if (dx * dx + dy * dy <= radius_squared) {
      return true;
    }
  }
  return false;
}

// 静态+动态方案运行限制图已直接画出需排除的区域，这里只查询点所在栅格，
// 不再叠加物理地图的15cm墙体膨胀。限制图外仍交给物理地图和绿色掩码判断。
bool VisualLidarPathFollower::logic3_runtime_map_point_is_filtered(
  const VisualLidarPathFollower::Point2D & map_point) const
{
  if (!latest_logic3_runtime_restriction_map_) {
    return false;
  }

  const auto & map = *latest_logic3_runtime_restriction_map_;
  const int width = static_cast<int>(map.info.width);
  const int height = static_cast<int>(map.info.height);
  const double resolution = map.info.resolution;
  if (width <= 0 || height <= 0 || resolution <= 0.0 ||
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
  const int cell_x = static_cast<int>(std::floor(grid_x_m / resolution));
  const int cell_y = static_cast<int>(std::floor(grid_y_m / resolution));
  if (cell_x < 0 || cell_y < 0 || cell_x >= width || cell_y >= height) {
    return false;
  }

  const auto index = static_cast<std::size_t>(cell_y) * map.info.width +
    static_cast<std::size_t>(cell_x);
  return map.data[index] >= waypoint_shift_occupied_threshold_;
}

// 只用物理定位地图过滤墙体，不把 restriction_map 的软禁行区混入这两段判断。
bool VisualLidarPathFollower::lidar_map_point_is_wall(
  const VisualLidarPathFollower::Point2D & map_point) const
{
  if (!lidar_enable_static_map_filter_ || !latest_map_) {
    return false;
  }

  const auto & map = *latest_map_;
  const int width = static_cast<int>(map.info.width);
  const int height = static_cast<int>(map.info.height);
  const double resolution = map.info.resolution;
  if (width <= 0 || height <= 0 || resolution <= 0.0 ||
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
  const int cell_x = static_cast<int>(std::floor(grid_x_m / resolution));
  const int cell_y = static_cast<int>(std::floor(grid_y_m / resolution));
  const int padding_cells = static_cast<int>(
    std::ceil(lidar_static_map_wall_padding_ / resolution));
  if (cell_x < 0 || cell_y < 0 || cell_x >= width || cell_y >= height) {
    return true;
  }

  for (int y = cell_y - padding_cells; y <= cell_y + padding_cells; ++y) {
    for (int x = cell_x - padding_cells; x <= cell_x + padding_cells; ++x) {
      const int offset_x = x - cell_x;
      const int offset_y = y - cell_y;
      if (std::hypot(offset_x, offset_y) * resolution >
        lidar_static_map_wall_padding_ + 1.0e-9)
      {
        continue;
      }
      if (x < 0 || y < 0 || x >= width || y >= height) {
        return true;
      }
      const auto index = static_cast<std::size_t>(y) * map.info.width +
        static_cast<std::size_t>(x);
      if (map.data[index] >= lidar_static_map_occupied_threshold_) {
        return true;
      }
    }
  }
  return false;
}

// 在指定朝向上取最近障碍。常规模式要求总视野扇形、最大半径和矩形
// 走廊同时命中；二维码倒车前方模式改用近宽远窄的直边梯形。
// center_angle=0 检查车前，center_angle=pi 检查车后。
// restrict_to_qr_reverse_front_trapezoid=true时，maximum_distance作为梯形
// 纵向长度，near/far半宽沿forward线性插值，不再使用fov或固定走廊。
// restrict_to_qr_reverse_arc=true时，额外用二维码倒车的固定参考曲率生成
// 车体左外缘圆弧，只保留圆弧以下、也就是右后倒实际扫过一侧的点。
VisualLidarPathFollower::LidarObstacle
VisualLidarPathFollower::find_lidar_obstacle(
  double center_angle, double maximum_distance, double fov_deg,
  double corridor_half_width,
  const VisualLidarPathFollower::RobotPose & scan_to_base,
  const VisualLidarPathFollower::RobotPose & scan_to_map,
  bool filter_logic3_green_mask,
  bool filter_logic3_runtime_map,
  bool restrict_to_qr_reverse_front_trapezoid,
  bool restrict_to_qr_reverse_arc,
  const VisualLidarPathFollower::Point2D * locked_target_map_point,
  double locked_target_radius) const
{
  LidarObstacle result;
  if (!latest_scan_ || maximum_distance <= 0.0) {
    return result;
  }

  const auto & scan = *latest_scan_;
  const double half_fov = std::max(fov_deg * M_PI / 360.0, 0.01);
  const double cos_base = std::cos(scan_to_base.yaw);
  const double sin_base = std::sin(scan_to_base.yaw);
  const double cos_map = std::cos(scan_to_map.yaw);
  const double sin_map = std::sin(scan_to_map.yaw);
  double angle = scan.angle_min;
  for (const auto value : scan.ranges) {
    const double scan_angle = angle;
    angle += scan.angle_increment;
    if (!std::isfinite(value) || value < scan.range_min || value > scan.range_max) {
      continue;
    }

    const double scan_x = static_cast<double>(value) * std::cos(scan_angle);
    const double scan_y = static_cast<double>(value) * std::sin(scan_angle);
    const Point2D base_point{
      scan_to_base.x + cos_base * scan_x - sin_base * scan_y,
      scan_to_base.y + sin_base * scan_x + cos_base * scan_y};
    const double point_range = std::hypot(base_point.x, base_point.y);
    const double relative_angle = normalize_angle(
      std::atan2(base_point.y, base_point.x) - center_angle);
    const double forward = point_range * std::cos(relative_angle);
    const double lateral = point_range * std::sin(relative_angle);
    if (restrict_to_qr_reverse_front_trapezoid) {
      if (forward <= 0.0 || forward > maximum_distance) {
        continue;
      }
      const double distance_ratio = std::clamp(
        forward / maximum_distance, 0.0, 1.0);
      const double allowed_half_width =
        qr_reverse_front_near_half_width_ +
        distance_ratio *
        (qr_reverse_front_far_half_width_ - qr_reverse_front_near_half_width_);
      if (std::abs(lateral) > allowed_half_width) {
        continue;
      }
    } else {
      if (point_range > maximum_distance ||
        std::abs(relative_angle) > half_fov || forward <= 0.0 ||
        std::abs(lateral) > corridor_half_width)
      {
        continue;
      }
    }

    if (restrict_to_qr_reverse_arc) {
      // 当前倒车为linear.x<0、angular.z>0，瞬时转动中心位于车体右侧
      // (base_link.y<0)。车体左外缘因此扫过以(0,-R)为圆心、
      // R+left_offset为半径的外圆弧：
      //   y_arc(x) = -R + sqrt((R+left_offset)^2 - x^2)
      // 后方硬保护只保留y<=y_arc的点。参考角速度为0时退化为
      // y<=left_offset的直线裁剪。
      double arc_boundary_y = qr_reverse_rear_arc_left_offset_;
      if (qr_reverse_rear_arc_angular_speed_ > 1.0e-6 &&
        qr_reverse_linear_speed_ > 1.0e-6)
      {
        const double turn_radius =
          qr_reverse_linear_speed_ / qr_reverse_rear_arc_angular_speed_;
        const double outer_radius =
          turn_radius + qr_reverse_rear_arc_left_offset_;
        const double radicand =
          outer_radius * outer_radius - base_point.x * base_point.x;
        if (radicand < 0.0) {
          continue;
        }
        arc_boundary_y = -turn_radius + std::sqrt(std::max(0.0, radicand));
      }
      if (base_point.y > arc_boundary_y) {
        continue;
      }
    }

    const Point2D map_point{
      scan_to_map.x + cos_map * scan_x - sin_map * scan_y,
      scan_to_map.y + sin_map * scan_x + cos_map * scan_y};
    const bool locked_target_active = locked_target_map_point != nullptr;
    const bool belongs_to_locked_target = locked_target_active &&
      std::hypot(
      map_point.x - locked_target_map_point->x,
      map_point.y - locked_target_map_point->y) <= locked_target_radius;
    // 一次动作开始后只允许首次锁定目标继续提供控制点。其他实时障碍和
    // 永久锥桶即使随后进入扇形，也不能把本轮避障无限续命；待本轮目标
    // 真正离开检测区后，它们才可在下一轮作为新目标重新判断方向。
    if (locked_target_active && !belongs_to_locked_target) {
      continue;
    }
    if (lidar_enable_static_map_filter_ || filter_logic3_runtime_map ||
      filter_logic3_green_mask)
    {
      if ((lidar_enable_static_map_filter_ &&
        lidar_map_point_is_wall(map_point)) ||
        (filter_logic3_runtime_map &&
        logic3_runtime_map_point_is_filtered(map_point)) ||
        (filter_logic3_green_mask && !belongs_to_locked_target &&
        logic3_green_mask_contains(map_point)))
      {
        continue;
      }
    }

    if (!result.found || point_range < result.range) {
      result.found = true;
      result.range = point_range;
      result.relative_angle = relative_angle;
      result.map_point = map_point;
    }
  }
  return result;
}

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

// 为四个阶段的第x+1轮及以后提供“障碍相对无偏移路线方位”。
// 障碍在路线左侧时右绕、右侧时左绕；若几乎压在线上，则与上一轮反向，
// 避免连续同方向动作把车越推越远。投影只搜索current_index附近的局部弧长，
// 防止闭环路线把障碍匹配到空间接近但进度很远的另一条支路。
bool VisualLidarPathFollower::find_lidar_route_side_direction(
  const VisualLidarPathFollower::Point2D & obstacle_map_point,
  const VisualLidarPathFollower::StageControlProfile & profile,
  int last_avoid_direction, int & avoid_direction, double & route_heading,
  double & signed_lateral_distance) const
{
  avoid_direction = 0;
  route_heading = 0.0;
  signed_lateral_distance = 0.0;
  if (original_waypoints_.size() < 2 ||
    current_index_ >= original_waypoints_.size() - 1)
  {
    return false;
  }

  const std::size_t last_segment = original_waypoints_.size() - 2;
  std::size_t start_segment = std::min(current_index_, last_segment);
  double backward_length = 0.0;
  while (start_segment > 0 &&
    backward_length < profile.lidar_route_side_search_backward_distance)
  {
    --start_segment;
    backward_length += std::hypot(
      original_waypoints_[start_segment + 1].x -
      original_waypoints_[start_segment].x,
      original_waypoints_[start_segment + 1].y -
      original_waypoints_[start_segment].y);
  }

  std::size_t end_segment = std::min(current_index_, last_segment);
  double forward_length = 0.0;
  while (end_segment < last_segment &&
    forward_length < profile.lidar_route_side_search_forward_distance)
  {
    forward_length += std::hypot(
      original_waypoints_[end_segment + 1].x -
      original_waypoints_[end_segment].x,
      original_waypoints_[end_segment + 1].y -
      original_waypoints_[end_segment].y);
    ++end_segment;
  }

  bool projection_valid = false;
  double best_distance = std::numeric_limits<double>::infinity();
  double best_segment_x = 0.0;
  double best_segment_y = 0.0;
  double best_segment_length = 0.0;
  const Waypoint * best_segment_start = nullptr;
  for (std::size_t index = start_segment; index <= end_segment; ++index) {
    const auto & start = original_waypoints_[index];
    const auto & finish = original_waypoints_[index + 1];
    const double segment_x = finish.x - start.x;
    const double segment_y = finish.y - start.y;
    const double segment_length_squared =
      segment_x * segment_x + segment_y * segment_y;
    if (segment_length_squared < 1.0e-8) {
      continue;
    }
    const double point_x = obstacle_map_point.x - start.x;
    const double point_y = obstacle_map_point.y - start.y;
    const double t = std::clamp(
      (point_x * segment_x + point_y * segment_y) /
      segment_length_squared, 0.0, 1.0);
    const double projected_x = start.x + t * segment_x;
    const double projected_y = start.y + t * segment_y;
    const double distance = std::hypot(
      obstacle_map_point.x - projected_x,
      obstacle_map_point.y - projected_y);
    if (distance < best_distance) {
      projection_valid = true;
      best_distance = distance;
      best_segment_x = segment_x;
      best_segment_y = segment_y;
      best_segment_length = std::sqrt(segment_length_squared);
      best_segment_start = &start;
    }
  }
  if (!projection_valid || best_segment_start == nullptr ||
    best_segment_length < 1.0e-8)
  {
    return false;
  }

  route_heading = std::atan2(best_segment_y, best_segment_x);
  signed_lateral_distance =
    (best_segment_x * (obstacle_map_point.y - best_segment_start->y) -
    best_segment_y * (obstacle_map_point.x - best_segment_start->x)) /
    best_segment_length;
  if (signed_lateral_distance > profile.lidar_route_side_deadband) {
    // 障碍在路线行进方向左侧：向右避。
    avoid_direction = -1;
  } else if (
    signed_lateral_distance < -profile.lidar_route_side_deadband)
  {
    // 障碍在路线行进方向右侧：向左避。
    avoid_direction = 1;
  } else {
    // 障碍几乎压在线上时没有稳定左右方位：优先与上一轮反向。
    avoid_direction = last_avoid_direction != 0 ? -last_avoid_direction : 1;
  }
  return true;
}

// 四阶段实时雷达避障：smac_pid策略始终屏蔽全部已确认的绿/橙永久点，
// 只让掩码外未记录新障碍进入实时控制；legacy_lookahead在配置
// transition_dynamic_avoid_only时允许过渡段雷达覆盖全部障碍，但这只影响
// 实时雷达过滤，不再清除已经在完整A→C→B→Task2路线中计算出的静态偏移。
bool VisualLidarPathFollower::apply_channel_lidar_obstacle_avoidance(
  geometry_msgs::msg::Twist & cmd, bool * observation_valid,
  bool * obstacle_found)
{
  if (observation_valid) {
    *observation_valid = false;
  }
  if (obstacle_found) {
    *obstacle_found = false;
  }
  const bool filter_logic3_runtime_map = logic3_planning_pose_anchored_;
  const bool before_or_at_channel_entry = !waypoints_.empty() &&
    channel_entry_waypoint_index_ < waypoints_.size() &&
    current_index_ <= channel_entry_waypoint_index_;
  const bool transition_lidar_scope =
    task_phase_ == TaskPhase::TRANSITION ||
    (task_phase_ == TaskPhase::TASK2 && before_or_at_channel_entry);
  // B点附近即使Task2参数已经提前接管，仍按过渡阶段计数与策略继续，
  // 直到真正越过A→C→B入口段；其余三个阶段各自独立计数。
  const TaskPhase policy_phase = transition_lidar_scope ?
    TaskPhase::TRANSITION : task_phase_;
  const StageControlProfile * policy_profile = nullptr;
  std::size_t * policy_avoid_count = nullptr;
  int * policy_last_direction = nullptr;
  switch (policy_phase) {
    case TaskPhase::TASK1:
      policy_profile = &task1_profile_;
      policy_avoid_count = &task1_lidar_avoid_count_;
      policy_last_direction = &task1_lidar_last_avoid_direction_;
      break;
    case TaskPhase::TRANSITION:
      policy_profile = &transition_profile_;
      policy_avoid_count = &transition_lidar_avoid_count_;
      policy_last_direction = &transition_lidar_last_avoid_direction_;
      break;
    case TaskPhase::TASK2:
      policy_profile = &task2_profile_;
      policy_avoid_count = &task2_lidar_avoid_count_;
      policy_last_direction = &task2_lidar_last_avoid_direction_;
      break;
    case TaskPhase::TASK3:
      policy_profile = &task3_profile_;
      policy_avoid_count = &task3_lidar_avoid_count_;
      policy_last_direction = &task3_lidar_last_avoid_direction_;
      break;
    case TaskPhase::FINISHED:
    case TaskPhase::UNKNOWN:
      break;
  }
  if (!policy_profile || !policy_avoid_count || !policy_last_direction) {
    return false;
  }
  const std::size_t heading_direction_max_count = static_cast<std::size_t>(
    policy_profile->lidar_heading_direction_max_count);
  const std::size_t route_side_direction_max_count = static_cast<std::size_t>(
    policy_profile->lidar_route_side_direction_enabled ?
    policy_profile->lidar_route_side_direction_max_count : 0);
  const std::size_t policy_total_max_count =
    heading_direction_max_count + route_side_direction_max_count;
  if (!channel_lidar_obstacle_active_ &&
    (*policy_avoid_count >= policy_total_max_count))
  {
    return false;
  }
  // 倒车结束时立下scan_sequence栅栏。必须收到严格晚于倒车退出时刻的
  // 新雷达帧，才允许首次正向动态避障判断；禁止复用倒车转向过程中的旧帧。
  if (channel_lidar_waiting_for_post_reverse_scan_) {
    if (!latest_scan_ || scan_sequence_ <= channel_lidar_post_reverse_scan_sequence_) {
      return false;
    }
    const std::int64_t scan_stamp_nanoseconds = rclcpp::Time(
      latest_scan_->header.stamp, get_clock()->get_clock_type()).nanoseconds();
    // 正常带时间戳的雷达必须确实采样于倒车退出之后；时间戳为0的驱动无法
    // 做采样时刻比较，只退化为上面的新回调序号闸门。
    if (scan_stamp_nanoseconds != 0 &&
      scan_stamp_nanoseconds <= channel_lidar_post_reverse_time_nanoseconds_)
    {
      return false;
    }
    channel_lidar_waiting_for_post_reverse_scan_ = false;
  }
  const bool transition_dynamic_only =
    tracking_controller_mode_ != "smac_pid" && transition_dynamic_avoid_only_ &&
    transition_lidar_scope;
  const bool track_locked_target =
    channel_lidar_obstacle_active_ && channel_lidar_locked_target_valid_;
  const double locked_target_tracking_radius =
    channel_lidar_locked_target_is_permanent_ ?
    logic3_green_red_filter_radius_ : logic3_red_takeover_radius_;
  const bool filter_logic3_green_mask =
    logic3_planning_pose_anchored_ &&
    (!transition_dynamic_only || channel_lidar_obstacle_active_) &&
    !logic3_green_clusters_.empty();
  // 同一次避障动作锁定触发阶段的检测区、控制参数和首次目标。这样即使
  // 动作中越过B点切换Task2参数，也只会在首次目标离开原检测区后结束。
  const double lidar_avoid_distance = channel_lidar_obstacle_active_ ?
    channel_lidar_action_profile_.lidar_avoid_distance :
    channel_lidar_avoid_distance_;
  const double lidar_avoid_fov_deg = channel_lidar_obstacle_active_ ?
    channel_lidar_action_profile_.lidar_avoid_fov_deg :
    channel_lidar_avoid_fov_deg_;
  const double lidar_corridor_half_width = channel_lidar_obstacle_active_ ?
    channel_lidar_action_profile_.lidar_corridor_half_width :
    channel_lidar_corridor_half_width_;
  const double lidar_avoid_linear_speed = channel_lidar_obstacle_active_ ?
    channel_lidar_action_profile_.lidar_avoid_linear_speed :
    channel_lidar_avoid_linear_speed_;
  double lidar_avoid_angular_gain = channel_lidar_obstacle_active_ ?
    channel_lidar_action_profile_.lidar_avoid_angular_gain :
    channel_lidar_avoid_angular_gain_;
  const double lidar_direction_upper_angle_deg = channel_lidar_obstacle_active_ ?
    channel_lidar_action_profile_.lidar_direction_upper_angle_deg :
    channel_lidar_direction_upper_angle_deg_;
  const double lidar_direction_lower_angle_deg = channel_lidar_obstacle_active_ ?
    channel_lidar_action_profile_.lidar_direction_lower_angle_deg :
    channel_lidar_direction_lower_angle_deg_;
  const double lidar_route_side_reverse_exit_hysteresis_deg =
    channel_lidar_obstacle_active_ ?
    channel_lidar_action_profile_.lidar_route_side_reverse_exit_hysteresis_deg :
    policy_profile->lidar_route_side_reverse_exit_hysteresis_deg;
  RobotPose scan_to_base;
  RobotPose scan_to_map;
  if (!prepare_lidar_scan(
      scan_to_base, scan_to_map,
      filter_logic3_runtime_map || filter_logic3_green_mask ||
      track_locked_target,
      filter_logic3_runtime_map))
  {
    if (!channel_lidar_waiting_for_inputs_) {
      RCLCPP_WARN(
        get_logger(),
        "channel雷达避障输入暂不可用：跳过本周期雷达覆盖，继续当前路线跟踪，不停车。");
      channel_lidar_waiting_for_inputs_ = true;
    }
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
    return false;
  }
  channel_lidar_waiting_for_inputs_ = false;
  if (observation_valid) {
    *observation_valid = true;
  }

  const auto obstacle = find_lidar_obstacle(
    0.0, lidar_avoid_distance, lidar_avoid_fov_deg,
    lidar_corridor_half_width, scan_to_base, scan_to_map,
    filter_logic3_green_mask, filter_logic3_runtime_map, false, false,
    track_locked_target ? &channel_lidar_locked_target_map_point_ : nullptr,
    locked_target_tracking_radius);
  // 二次防护：即使上游过滤开关未按预期生效，PID模式也绝不允许
  // 已记录绿/橙掩码内的雷达点覆盖路线跟踪命令。
  if (obstacle.found && !track_locked_target &&
    tracking_controller_mode_ == "smac_pid" &&
    logic3_green_mask_contains(obstacle.map_point))
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "实时雷达候选map(%.3f, %.3f)位于已记录绿/橙点%.2fm掩码内；"
      "二次防护已拦截，保持smac_pid路线跟踪。",
      obstacle.map_point.x, obstacle.map_point.y, logic3_green_red_filter_radius_);
    return false;
  }
  if (!obstacle.found) {
    if (channel_lidar_obstacle_active_) {
      const char * completed_target_type =
        channel_lidar_locked_target_is_task1_clearance_fallback_ ?
        "Task1静态净距失败专用兜底" :
        (channel_lidar_locked_target_is_permanent_ ?
        "绿/橙永久点动态接管" : "普通实时动态障碍");
      RCLCPP_INFO(
        get_logger(),
        "channel雷达避障结束：首次锁定目标=%s，中心=(%.3f, %.3f)"
        "已离开检测区；其他障碍未参与本轮续接，恢复当前阶段路线跟踪。",
        completed_target_type,
        channel_lidar_locked_target_map_point_.x,
        channel_lidar_locked_target_map_point_.y);
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
    return false;
  }
  if (obstacle_found) {
    *obstacle_found = true;
  }

  const double half_fov = std::max(
    lidar_avoid_fov_deg * M_PI / 360.0, 0.01);
  const double center_weight = std::clamp(
    1.0 - std::abs(obstacle.relative_angle) / half_fov, 0.0, 1.0);
  const double upper_angle =
    lidar_direction_upper_angle_deg * M_PI / 180.0;
  const double lower_angle =
    lidar_direction_lower_angle_deg * M_PI / 180.0;
  // 由同一雷达采样时刻的map<-scan和base_link<-scan恢复map<-base_link，
  // 供路线方位动作判断车头是否已经越过“本轮触发障碍”的固定map方向。
  const double vehicle_map_yaw = normalize_angle(
    scan_to_map.yaw - scan_to_base.yaw);
  const double vehicle_map_cos = std::cos(vehicle_map_yaw);
  const double vehicle_map_sin = std::sin(vehicle_map_yaw);
  const Point2D vehicle_map_point{
    scan_to_map.x -
    (vehicle_map_cos * scan_to_base.x - vehicle_map_sin * scan_to_base.y),
    scan_to_map.y -
    (vehicle_map_sin * scan_to_base.x + vehicle_map_cos * scan_to_base.y)};
  const bool avoidance_action_started = !channel_lidar_obstacle_active_;
  double route_signed_lateral_distance = 0.0;
  if (avoidance_action_started) {
    channel_lidar_route_side_action_active_ = false;
    channel_lidar_route_reverse_phase_active_ = false;
    channel_lidar_route_toward_direction_ = 0;
    channel_lidar_route_trigger_obstacle_valid_ = false;
    const std::size_t policy_action_number = *policy_avoid_count + 1;
    const bool request_route_side_direction =
      policy_profile->lidar_route_side_direction_enabled &&
      policy_action_number > heading_direction_max_count;
    if (request_route_side_direction &&
      find_lidar_route_side_direction(
        obstacle.map_point, *policy_profile, *policy_last_direction,
        channel_lidar_avoid_direction_,
        channel_lidar_route_heading_, route_signed_lateral_distance))
    {
      channel_lidar_route_side_action_active_ = true;
      channel_lidar_route_trigger_obstacle_map_point_ = obstacle.map_point;
      channel_lidar_route_trigger_obstacle_valid_ = true;
      lidar_avoid_angular_gain = policy_profile->lidar_route_side_angular_gain;

      // T_map_scan = T_map_base * T_base_scan，由两份同帧TF恢复车头map角。
      const double route_relative_angle = normalize_angle(
        channel_lidar_route_heading_ - vehicle_map_yaw);
      if (std::abs(route_relative_angle) > 1.0e-6) {
        channel_lidar_route_toward_direction_ =
          route_relative_angle > 0.0 ? 1 : -1;
      }
      const bool obstacle_between_vehicle_and_route =
        channel_lidar_route_toward_direction_ != 0 &&
        obstacle.relative_angle * route_relative_angle > 0.0 &&
        std::abs(obstacle.relative_angle) <=
        std::abs(route_relative_angle) + 1.0e-9;
      channel_lidar_route_reverse_phase_active_ =
        policy_profile->lidar_route_side_reverse_phase_enabled &&
        obstacle_between_vehicle_and_route;
      if (channel_lidar_route_reverse_phase_active_) {
        // 精确落实反相规则：障碍角夹在车头与路线之间时，不沿普通障碍
        // 左右侧继续外绕，而是直接锁定朝路线方向转，并使用最大转角。
        channel_lidar_avoid_direction_ = channel_lidar_route_toward_direction_;
      }
    } else {
      // 第1轮以及路线局部投影失败时，保持原有车头相对角方向算法。
      if (obstacle.relative_angle > upper_angle) {
        // 障碍明确在左侧：右绕。
        channel_lidar_avoid_direction_ = -1;
      } else if (obstacle.relative_angle < lower_angle) {
        // 障碍明确在右侧：左绕。
        channel_lidar_avoid_direction_ = 1;
      } else {
        // 首次遇到正中障碍时沿用原策略：默认左绕。
        channel_lidar_avoid_direction_ = 1;
      }
    }
    ++(*policy_avoid_count);
    *policy_last_direction = channel_lidar_avoid_direction_;
  }
  // 路线方位动作的方向从触发到清空始终锁定。若触发时障碍方向夹在
  // 车头与路线方向之间，就先按最大转角穿过障碍方向；车头沿锁定方向
  // 越过障碍至少配置迟滞角后，才恢复原扇形中心权重。切换时障碍几乎
  // 位于车头中心，因此最大值可平滑衔接到原公式，不会出现先小后大的反相。
  if (channel_lidar_route_side_action_active_ &&
    channel_lidar_route_reverse_phase_active_)
  {
    const double exit_hysteresis =
      lidar_route_side_reverse_exit_hysteresis_deg * M_PI / 180.0;
    const double trigger_obstacle_relative_angle =
      channel_lidar_route_trigger_obstacle_valid_ ? normalize_angle(
      std::atan2(
        channel_lidar_route_trigger_obstacle_map_point_.y - vehicle_map_point.y,
        channel_lidar_route_trigger_obstacle_map_point_.x - vehicle_map_point.x) -
      vehicle_map_yaw) : obstacle.relative_angle;
    const bool crossed_obstacle_direction =
      (channel_lidar_route_toward_direction_ > 0 &&
      trigger_obstacle_relative_angle <= -exit_hysteresis) ||
      (channel_lidar_route_toward_direction_ < 0 &&
      trigger_obstacle_relative_angle >= exit_hysteresis);
    if (crossed_obstacle_direction) {
      channel_lidar_route_reverse_phase_active_ = false;
    }
  }
  const double turn_weight =
    channel_lidar_route_side_action_active_ &&
    channel_lidar_route_reverse_phase_active_ ? 1.0 : center_weight;
  const double avoid_direction =
    static_cast<double>(channel_lidar_avoid_direction_);
  cmd.linear.x = lidar_avoid_linear_speed;
  cmd.angular.z = avoid_direction * lidar_avoid_angular_gain * turn_weight;
  if (avoidance_action_started) {
    // 固定本轮唯一目标。优先关联绿/橙永久ID并锁定其稳定中心；未关联时
    // 锁定首次红点。中心在整轮动作中绝不逐帧更新，避免雷达抖动把目标
    // 沿相邻点云拖走，或由其他锥桶接力延长本轮动作。
    channel_lidar_locked_target_map_point_ = obstacle.map_point;
    channel_lidar_locked_target_valid_ = true;
    channel_lidar_locked_target_is_permanent_ = false;
    channel_lidar_locked_target_is_task1_clearance_fallback_ = false;
    channel_lidar_locked_target_permanent_index_ = logic3_green_clusters_.size();
    double nearest_permanent_distance = logic3_red_takeover_radius_;
    for (std::size_t index = 0; index < logic3_green_clusters_.size(); ++index) {
      const double distance = std::hypot(
        obstacle.map_point.x - logic3_green_clusters_[index].center.x,
        obstacle.map_point.y - logic3_green_clusters_[index].center.y);
      if (distance <= nearest_permanent_distance) {
        nearest_permanent_distance = distance;
        channel_lidar_locked_target_permanent_index_ = index;
      }
    }
    if (channel_lidar_locked_target_permanent_index_ <
      logic3_green_clusters_.size())
    {
      const std::size_t permanent_index =
        channel_lidar_locked_target_permanent_index_;
      channel_lidar_locked_target_map_point_ =
        logic3_green_clusters_[permanent_index].center;
      channel_lidar_locked_target_is_permanent_ = true;
      channel_lidar_locked_target_is_task1_clearance_fallback_ =
        task_phase_ == TaskPhase::TASK1 &&
        task1_static_avoid_region_contains(
        logic3_green_clusters_[permanent_index].center) &&
        permanent_index < logic3_green_route_clearance_failed_.size() &&
        logic3_green_route_clearance_failed_[permanent_index];
    }
    const char * locked_target_type =
      channel_lidar_locked_target_is_task1_clearance_fallback_ ?
      "Task1静态净距失败专用兜底" :
      (channel_lidar_locked_target_is_permanent_ ?
      "绿/橙永久点动态接管" : "普通实时动态障碍");

    // 每次完整动作只在首次真正覆盖路线命令时记录一个红点；连续控制周期
    // 不重复生成抖动红点，也不重复生成before_red图片。
    record_logic3_lidar_avoidance_obstacle(obstacle, scan_to_base, scan_to_map);
    channel_lidar_action_profile_.lidar_avoid_distance = lidar_avoid_distance;
    channel_lidar_action_profile_.lidar_avoid_fov_deg = lidar_avoid_fov_deg;
    channel_lidar_action_profile_.lidar_corridor_half_width = lidar_corridor_half_width;
    channel_lidar_action_profile_.lidar_avoid_linear_speed = lidar_avoid_linear_speed;
    channel_lidar_action_profile_.lidar_avoid_angular_gain = lidar_avoid_angular_gain;
    channel_lidar_action_profile_.lidar_direction_upper_angle_deg =
      lidar_direction_upper_angle_deg;
    channel_lidar_action_profile_.lidar_direction_lower_angle_deg =
      lidar_direction_lower_angle_deg;
    channel_lidar_action_profile_.lidar_route_side_reverse_exit_hysteresis_deg =
      lidar_route_side_reverse_exit_hysteresis_deg;
    // 只在本次避障真正触发的这一刻关联永久绿/橙身份：从原始路线重算并
    // 取消该ID的静态偏移；绿/橙雷达掩码始终保留。后续只对上面锁定的
    // 唯一目标开放掩码例外，并在该目标离开检测区时结束本轮。
    if (activate_logic3_dynamic_takeover(obstacle.map_point)) {
      rebuild_logic3_route_from_green_clusters("实际雷达避障接管永久点");
    }
    channel_lidar_obstacle_active_ = true;
    channel_lidar_logged_direction_ = channel_lidar_avoid_direction_;

    // 每次完整避障动作只在false->true的触发瞬间留一张图；动作中的连续
    // 控制周期以及方向保持不重复出图。
    if (task_phase_ == TaskPhase::TASK1) {
      ++logic3_task1_before_red_snapshot_count_;
      pending_control_snapshot_event_ = "task1_before_red";
    } else if (task_phase_ == TaskPhase::TRANSITION ||
      task_phase_ == TaskPhase::TASK2 || task_phase_ == TaskPhase::TASK3)
    {
      ++logic3_final_before_red_snapshot_count_;
      pending_control_snapshot_event_ = "final_before_red";
    }

    if (channel_lidar_route_side_action_active_) {
      RCLCPP_WARN(
        get_logger(),
        "channel雷达避障开始：阶段=%s，第%zu/%zu轮[x=%zu,y=%zu]，"
        "目标=%s，锁定中心=(%.3f, %.3f)，"
        "方向算法=路线方位，障碍map=(%.3f, %.3f)，"
        "相对无偏移路线横距=%+.3fm，路线角=%.1fdeg，"
        "车头相对障碍角=%.1fdeg，反相最大转角=%s，保持%s绕，输出=%.2f/%.2f。",
        task_phase_name(policy_phase), *policy_avoid_count, policy_total_max_count,
        heading_direction_max_count, route_side_direction_max_count,
        locked_target_type,
        channel_lidar_locked_target_map_point_.x,
        channel_lidar_locked_target_map_point_.y,
        obstacle.map_point.x, obstacle.map_point.y,
        route_signed_lateral_distance,
        channel_lidar_route_heading_ * 180.0 / M_PI,
        obstacle.relative_angle * 180.0 / M_PI,
        channel_lidar_route_reverse_phase_active_ ? "是" : "否",
        channel_lidar_avoid_direction_ > 0 ? "左" : "右",
        cmd.linear.x, cmd.angular.z);
    } else {
      RCLCPP_WARN(
        get_logger(),
        "channel雷达避障开始：阶段=%s，第%zu/%zu轮[x=%zu,y=%zu]，"
        "目标=%s，锁定中心=(%.3f, %.3f)，"
        "方向算法=车头相对角，距离=%.3fm，相对角=%.1fdeg，"
        "滞环=[%.1f, %.1f]deg，保持%s绕，输出=%.2f/%.2f。",
        task_phase_name(policy_phase), *policy_avoid_count, policy_total_max_count,
        heading_direction_max_count, route_side_direction_max_count,
        locked_target_type,
        channel_lidar_locked_target_map_point_.x,
        channel_lidar_locked_target_map_point_.y,
        obstacle.range, obstacle.relative_angle * 180.0 / M_PI,
        lidar_direction_lower_angle_deg,
        lidar_direction_upper_angle_deg,
        channel_lidar_avoid_direction_ > 0 ? "左" : "右",
        cmd.linear.x, cmd.angular.z);
    }
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
  task3_route_available_ = false;
  task3_route_active_ = false;
  return_stop_zone_entry_active_ = false;
  task3_start_index_ = waypoints_.size();
  for (std::size_t index = 0; index < waypoints_.size(); ++index) {
    if (waypoints_[index].task3_start) {
      task3_route_available_ = true;
      task3_start_index_ = index;
      break;
    }
  }
  // 保留航点文件原本的Task3末点。路线外(0,0)保底点不写入waypoints_，
  // 不参与下面的静态偏移、circle_envelope或路径PID缓存。
  original_waypoints_ = waypoints_;
  waypoint_shift_offsets_.assign(waypoints_.size(), 0.0);
  waypoint_shift_pending_offsets_.assign(waypoints_.size(), 0.0);
  waypoint_shift_pending_counts_.assign(waypoints_.size(), 0);
  waypoint_shift_missing_counts_.assign(waypoints_.size(), 0);
  waypoint_shift_dynamic_directions_.assign(waypoints_.size(), 0);
  waypoint_shift_world_directions_.assign(waypoints_.size(), Point2D());
  waypoint_shift_source_indices_.assign(waypoints_.size(), waypoints_.size());
  waypoint_shift_obstacle_centers_.assign(waypoints_.size(), Point2D());
  waypoint_shift_forbidden_radii_.assign(waypoints_.size(), 0.0);
  logic3_frozen_base_offsets_.clear();
  logic3_frozen_base_shift_directions_.clear();
  logic3_frozen_base_source_indices_.clear();
  logic3_frozen_base_obstacle_centers_.clear();
  logic3_frozen_base_forbidden_radii_.clear();
  logic3_runtime_green_tracks_.clear();
  if (!channel_lidar_obstacle_active_) {
    channel_lidar_avoid_direction_ = 0;
  }
  processed_waypoint_shift_scan_sequence_ = scan_sequence_;
  rebuild_path_pid_route_cache();
  reset_path_pid_state();
}

void VisualLidarPathFollower::update_logic3_static_cluster_tracks(
  const std::vector<VisualLidarPathFollower::ScanPointCluster> & clusters,
  const VisualLidarPathFollower::RobotPose & robot_pose,
  double match_score, std::uint64_t score_scan_sequence)
{
  const double now_seconds = now().seconds();
  bool formal_collection_changed = false;
  const double window_start =
    now_seconds - logic3_static_confirmation_window_seconds_;
  logic3_static_scan_times_.push_back(now_seconds);
  while (!logic3_static_scan_times_.empty() &&
    logic3_static_scan_times_.front() < window_start)
  {
    logic3_static_scan_times_.pop_front();
  }
  for (auto & track : logic3_static_cluster_tracks_) {
    while (!track.rolling_hit_times.empty() &&
      track.rolling_hit_times.front() < window_start)
    {
      track.rolling_hit_times.pop_front();
      track.rolling_hit_centers.pop_front();
    }
  }

  std::vector<bool> track_matched(logic3_static_cluster_tracks_.size(), false);
  for (const auto & cluster : clusters) {
    std::size_t best_track = logic3_static_cluster_tracks_.size();
    double best_distance = logic3_static_cluster_match_radius_;
    for (std::size_t index = 0; index < logic3_static_cluster_tracks_.size(); ++index) {
      if (track_matched[index]) {
        continue;
      }
      const auto & track = logic3_static_cluster_tracks_[index];
      if (track.entered_state1 && track.loss_pending &&
        now_seconds - track.loss_started_seconds > logic3_static_loss_grace_seconds_)
      {
        continue;
      }
      const double match_radius = track.entered_state1 && track.loss_pending ?
        logic3_static_reacquire_radius_ : logic3_static_cluster_match_radius_;
      const auto & center = track.stability_anchor;
      const double distance = std::hypot(
        cluster.center.x - center.x, cluster.center.y - center.y);
      if (distance <= match_radius && distance <= best_distance) {
        best_distance = distance;
        best_track = index;
      }
    }
    if (best_track == logic3_static_cluster_tracks_.size()) {
      // 一个障碍在同一帧仍可能残留两个碎片。若最近ID已经被本帧第一个
      // 碎片占用，旧代码会跳过它并为第二个碎片创建新ID。这里允许第二个
      // 碎片在身份半径内并回本帧已匹配ID，但同一雷达帧仍只计一次命中。
      std::size_t matched_track = logic3_static_cluster_tracks_.size();
      double matched_distance = logic3_static_cluster_match_radius_;
      for (std::size_t index = 0; index < logic3_static_cluster_tracks_.size(); ++index) {
        if (!track_matched[index]) {
          continue;
        }
        const auto & current_center = logic3_static_cluster_tracks_[index].cluster.center;
        const double distance = std::hypot(
          cluster.center.x - current_center.x,
          cluster.center.y - current_center.y);
        if (distance <= matched_distance) {
          matched_distance = distance;
          matched_track = index;
        }
      }
      if (matched_track < logic3_static_cluster_tracks_.size()) {
        auto & track = logic3_static_cluster_tracks_[matched_track];
        const Point2D existing_center = track.cluster.center;
        track.cluster.points.insert(
          track.cluster.points.end(), cluster.points.begin(), cluster.points.end());
        track.cluster.center = Point2D();
        if (track.cluster.points.empty()) {
          track.cluster.center.x = 0.5 * (existing_center.x + cluster.center.x);
          track.cluster.center.y = 0.5 * (existing_center.y + cluster.center.y);
        } else {
          for (const auto & point : track.cluster.points) {
            track.cluster.center.x += point.x;
            track.cluster.center.y += point.y;
          }
          const double point_count = static_cast<double>(track.cluster.points.size());
          track.cluster.center.x /= point_count;
          track.cluster.center.y /= point_count;
        }
        track.last_seen_seconds = now_seconds;
        if (!track.rolling_hit_centers.empty()) {
          track.rolling_hit_centers.back() = track.cluster.center;
        }
        continue;
      }

      Logic3StaticClusterTrack track;
      track.cluster = cluster;
      track.stability_anchor = cluster.center;
      track.last_seen_seconds = now_seconds;
      track.rolling_hit_times.push_back(now_seconds);
      track.rolling_hit_centers.push_back(cluster.center);
      logic3_static_cluster_tracks_.push_back(std::move(track));
      track_matched.push_back(true);
      continue;
    }

    auto & track = logic3_static_cluster_tracks_[best_track];
    const double stability_distance = std::hypot(
      cluster.center.x - track.stability_anchor.x,
      cluster.center.y - track.stability_anchor.y);
    const double obstacle_range = std::hypot(
      cluster.center.x - robot_pose.x, cluster.center.y - robot_pose.y);
    const double effective_stability_radius =
      logic3_static_stability_radius_for_range(obstacle_range);
    const bool recovered_during_grace = track.entered_state1 && track.loss_pending;

    // 状态1后的失联重捕只检查固定锚点0.20m；成功出现一次就恢复原ID，
    // 不再要求该时刻的2秒窗口立刻重新达到10帧。
    if (recovered_during_grace) {
      track.loss_pending = false;
      track.loss_started_seconds = 0.0;
    } else if (stability_distance > effective_stability_radius) {
      // 坐标越出其距离自适应稳定半径，视为该ID位置发生更新。只让该ID
      // 作废并从新位置重新进行2秒窗口确认，其他ID继续采样或保持READY。
      const std::uint64_t invalidated_id = track.identity_id;
      const bool invalidated_formal_track = track.confirmed;
      track.identity_id = 0;
      track.confirmed = false;
      track.entered_state1 = false;
      track.loss_pending = false;
      track.ready = false;
      track.rolling_hit_times.clear();
      track.rolling_hit_centers.clear();
      track.best_score_samples.clear();
      track.ready_obstacle = Logic3StaticObstacle();
      track.stability_anchor = cluster.center;
      if (invalidated_formal_track) {
        formal_collection_changed = true;
        if (debug_control_logs_) {
          RCLCPP_INFO(
            get_logger(),
            "debug: 静态ID%llu位置越出稳定半径，回到2秒窗口重新确认。",
            static_cast<unsigned long long>(invalidated_id));
        }
      }
    }
    track.cluster = cluster;
    track.last_seen_seconds = now_seconds;
    track.rolling_hit_times.push_back(now_seconds);
    track.rolling_hit_centers.push_back(cluster.center);
    track_matched[best_track] = true;
  }

  std::vector<bool> remove_track(logic3_static_cluster_tracks_.size(), false);
  for (std::size_t index = 0; index < logic3_static_cluster_tracks_.size(); ++index) {
    auto & track = logic3_static_cluster_tracks_[index];
    const int rolling_hits = static_cast<int>(track.rolling_hit_times.size());

    if (!track.confirmed &&
      rolling_hits >= logic3_static_confirmation_min_hits_)
    {
      Point2D averaged_center;
      for (const auto & center : track.rolling_hit_centers) {
        averaged_center.x += center.x;
        averaged_center.y += center.y;
      }
      const double hit_count =
        static_cast<double>(track.rolling_hit_centers.size());
      averaged_center.x /= hit_count;
      averaged_center.y /= hit_count;
      track.stability_anchor = averaged_center;
      track.identity_id = logic3_static_next_identity_id_++;
      track.confirmed = true;
      track.entered_state1 = false;
      track.loss_pending = false;
      track.loss_started_seconds = 0.0;
      track.ready = false;
      track.best_score_samples.clear();
      track.ready_obstacle = Logic3StaticObstacle();
      formal_collection_changed = true;
    }

    if (track.confirmed && track.entered_state1) {
      if (!track_matched[index] &&
        rolling_hits < logic3_static_confirmation_min_hits_ &&
        !track.loss_pending)
      {
        track.loss_pending = true;
        track.loss_started_seconds = now_seconds;
      }
      if (track.loss_pending &&
        now_seconds - track.loss_started_seconds > logic3_static_loss_grace_seconds_)
      {
        // 状态1后每个正式ID独立使用1秒失联宽限；超时才真正删除该ID
        // 并让整体退出状态1，但不清空其他ID的数据。
        remove_track[index] = true;
        formal_collection_changed = true;
        RCLCPP_WARN(
          get_logger(),
          "静态+动态方案静态ID%llu失联超过%.2fs：删除该ID并退出整体状态1；"
          "其他ID的命中窗口、采样和READY结果保持不变。",
          static_cast<unsigned long long>(track.identity_id),
          logic3_static_loss_grace_seconds_);
      }
    } else if (track.confirmed &&
      rolling_hits < logic3_static_confirmation_min_hits_)
    {
      // 正式ID还没参加过状态1时，不启用1秒失联宽限；它仍必须持续满足
      // 状态1前的2秒/10帧规则。
      remove_track[index] = true;
      formal_collection_changed = true;
      RCLCPP_INFO(
        get_logger(),
        "静态+动态方案静态ID%llu在进入状态1前不再满足2秒/%d帧：删除该ID。",
        static_cast<unsigned long long>(track.identity_id),
        logic3_static_confirmation_min_hits_);
    } else if (!track.confirmed && track.rolling_hit_times.empty()) {
      // 状态1前的临时候选完全由2秒窗口维持，不另设1秒内部ID保留。
      remove_track[index] = true;
    }
  }

  for (std::size_t index = remove_track.size(); index > 0; --index) {
    if (remove_track[index - 1]) {
      logic3_static_cluster_tracks_.erase(
        logic3_static_cluster_tracks_.begin() + static_cast<std::ptrdiff_t>(index - 1));
      track_matched.erase(
        track_matched.begin() + static_cast<std::ptrdiff_t>(index - 1));
    }
  }

  // 第三道硬保护：已有静态轨迹也必须满足身份半径内唯一。绿色坐标是
  // 最高分样本平均值，两个早期相距较远的碎片ID可能在稳定后收敛到一起；
  // 因此不能只在“新建ID”瞬间检查，还要在每帧READY计算前两两去重。
  auto representative_center = [](const Logic3StaticClusterTrack & track) {
      if (track.ready) {
        return track.ready_obstacle.cluster.center;
      }
      if (!track.best_score_samples.empty()) {
        Point2D center;
        for (const auto & sample : track.best_score_samples) {
          center.x += sample.cluster.center.x;
          center.y += sample.cluster.center.y;
        }
        const double count = static_cast<double>(track.best_score_samples.size());
        center.x /= count;
        center.y /= count;
        return center;
      }
      if (!track.rolling_hit_centers.empty()) {
        return track.rolling_hit_centers.back();
      }
      return track.cluster.center;
    };
  for (std::size_t first = 0; first < logic3_static_cluster_tracks_.size(); ++first) {
    std::size_t second = first + 1;
    while (second < logic3_static_cluster_tracks_.size()) {
      const Point2D first_center = representative_center(
        logic3_static_cluster_tracks_[first]);
      const Point2D second_center = representative_center(
        logic3_static_cluster_tracks_[second]);
      const double duplicate_distance = std::hypot(
        first_center.x - second_center.x, first_center.y - second_center.y);
      if (duplicate_distance > logic3_static_cluster_match_radius_) {
        ++second;
        continue;
      }

      auto & first_track = logic3_static_cluster_tracks_[first];
      auto & second_track = logic3_static_cluster_tracks_[second];
      const bool second_is_better_keeper =
        (!first_track.confirmed && second_track.confirmed) ||
        (first_track.confirmed == second_track.confirmed &&
        first_track.identity_id != 0 && second_track.identity_id != 0 &&
        second_track.identity_id < first_track.identity_id);
      if (second_is_better_keeper) {
        std::swap(first_track, second_track);
        std::swap(track_matched[first], track_matched[second]);
      }

      auto & keeper = logic3_static_cluster_tracks_[first];
      auto & duplicate = logic3_static_cluster_tracks_[second];
      const bool duplicate_was_formal = duplicate.confirmed;
      const bool either_matched = track_matched[first] || track_matched[second];

      if (duplicate.last_seen_seconds > keeper.last_seen_seconds ||
        (std::abs(duplicate.last_seen_seconds - keeper.last_seen_seconds) <= 1e-9 &&
        duplicate.cluster.points.size() > keeper.cluster.points.size()))
      {
        keeper.cluster = duplicate.cluster;
      }
      keeper.last_seen_seconds = std::max(
        keeper.last_seen_seconds, duplicate.last_seen_seconds);

      std::vector<std::pair<double, Point2D>> combined_hits;
      combined_hits.reserve(
        keeper.rolling_hit_times.size() + duplicate.rolling_hit_times.size());
      for (std::size_t index = 0;
        index < keeper.rolling_hit_times.size() &&
        index < keeper.rolling_hit_centers.size(); ++index)
      {
        combined_hits.emplace_back(
          keeper.rolling_hit_times[index], keeper.rolling_hit_centers[index]);
      }
      for (std::size_t index = 0;
        index < duplicate.rolling_hit_times.size() &&
        index < duplicate.rolling_hit_centers.size(); ++index)
      {
        combined_hits.emplace_back(
          duplicate.rolling_hit_times[index], duplicate.rolling_hit_centers[index]);
      }
      std::sort(
        combined_hits.begin(), combined_hits.end(),
        [](const auto & lhs, const auto & rhs) {return lhs.first < rhs.first;});
      keeper.rolling_hit_times.clear();
      keeper.rolling_hit_centers.clear();
      for (const auto & hit : combined_hits) {
        if (!keeper.rolling_hit_times.empty() &&
          std::abs(hit.first - keeper.rolling_hit_times.back()) <= 1e-6)
        {
          auto & center = keeper.rolling_hit_centers.back();
          center.x = 0.5 * (center.x + hit.second.x);
          center.y = 0.5 * (center.y + hit.second.y);
          continue;
        }
        if (hit.first >= window_start) {
          keeper.rolling_hit_times.push_back(hit.first);
          keeper.rolling_hit_centers.push_back(hit.second);
        }
      }

      std::vector<Logic3StaticScoredSample> combined_samples =
        keeper.best_score_samples;
      combined_samples.insert(
        combined_samples.end(), duplicate.best_score_samples.begin(),
        duplicate.best_score_samples.end());
      std::sort(
        combined_samples.begin(), combined_samples.end(),
        [](const Logic3StaticScoredSample & lhs,
        const Logic3StaticScoredSample & rhs) {
          if (std::abs(lhs.match_score - rhs.match_score) > 1e-9) {
            return lhs.match_score > rhs.match_score;
          }
          return lhs.scan_sequence > rhs.scan_sequence;
        });
      keeper.best_score_samples.clear();
      for (const auto & sample : combined_samples) {
        const bool duplicate_scan = std::any_of(
          keeper.best_score_samples.begin(), keeper.best_score_samples.end(),
          [&](const Logic3StaticScoredSample & selected) {
            return selected.scan_sequence == sample.scan_sequence;
          });
        if (duplicate_scan) {
          continue;
        }
        keeper.best_score_samples.push_back(sample);
        if (keeper.best_score_samples.size() >=
          static_cast<std::size_t>(logic3_static_best_score_samples_))
        {
          break;
        }
      }

      keeper.confirmed = keeper.confirmed || duplicate.confirmed;
      keeper.entered_state1 = keeper.entered_state1 || duplicate.entered_state1;
      keeper.loss_pending = either_matched ? false :
        (keeper.loss_pending && duplicate.loss_pending);
      if (!keeper.loss_pending) {
        keeper.loss_started_seconds = 0.0;
      }
      track_matched[first] = either_matched;

      if (keeper.best_score_samples.size() >=
        static_cast<std::size_t>(logic3_static_best_score_samples_))
      {
        ScanPointCluster averaged_cluster;
        for (const auto & sample : keeper.best_score_samples) {
          averaged_cluster.center.x += sample.cluster.center.x;
          averaged_cluster.center.y += sample.cluster.center.y;
        }
        const double sample_count =
          static_cast<double>(keeper.best_score_samples.size());
        averaged_cluster.center.x /= sample_count;
        averaged_cluster.center.y /= sample_count;
        const auto strongest_sample = std::max_element(
          keeper.best_score_samples.begin(), keeper.best_score_samples.end(),
          [](const Logic3StaticScoredSample & lhs,
          const Logic3StaticScoredSample & rhs) {
            if (std::abs(lhs.match_score - rhs.match_score) > 1e-9) {
              return lhs.match_score < rhs.match_score;
            }
            return lhs.scan_sequence < rhs.scan_sequence;
          });
        averaged_cluster.points = strongest_sample->cluster.points;
        if (averaged_cluster.points.empty()) {
          averaged_cluster.points.push_back(averaged_cluster.center);
        } else {
          const double translate_x =
            averaged_cluster.center.x - strongest_sample->cluster.center.x;
          const double translate_y =
            averaged_cluster.center.y - strongest_sample->cluster.center.y;
          for (auto & point : averaged_cluster.points) {
            point.x += translate_x;
            point.y += translate_y;
          }
        }
        keeper.ready_obstacle = Logic3StaticObstacle{averaged_cluster};
        keeper.ready = true;
      } else if (!keeper.ready && duplicate.ready) {
        keeper.ready_obstacle = duplicate.ready_obstacle;
        keeper.ready = true;
      }

      formal_collection_changed = formal_collection_changed || duplicate_was_formal;
      logic3_static_cluster_tracks_.erase(
        logic3_static_cluster_tracks_.begin() + static_cast<std::ptrdiff_t>(second));
      track_matched.erase(
        track_matched.begin() + static_cast<std::ptrdiff_t>(second));
    }
  }

  const bool has_formal_id = std::any_of(
    logic3_static_cluster_tracks_.begin(), logic3_static_cluster_tracks_.end(),
    [](const Logic3StaticClusterTrack & track) {
      return track.confirmed;
    });

  // 状态1是整个正式ID集合的统一状态，不再由单个ID独立进入。正式ID
  // 新增、删除或位置越界更新都会重新开始0.5秒稳定计时并退出状态1；
  // 未受影响ID的采样与READY平均结果全部保留。
  if (!has_formal_id) {
    logic3_static_state1_active_ = false;
    logic3_static_ready_ = false;
    logic3_static_collection_change_time_valid_ = false;
  } else if (formal_collection_changed) {
    logic3_static_state1_active_ = false;
    logic3_static_ready_ = false;
    logic3_static_collection_change_time_valid_ = true;
    logic3_static_collection_last_change_seconds_ = now_seconds;
  } else if (!logic3_static_collection_change_time_valid_) {
    logic3_static_collection_change_time_valid_ = true;
    logic3_static_collection_last_change_seconds_ = now_seconds;
  } else if (!logic3_static_state1_active_ &&
    now_seconds - logic3_static_collection_last_change_seconds_ >=
    logic3_static_collection_stable_seconds_)
  {
    logic3_static_state1_active_ = true;
    for (auto & track : logic3_static_cluster_tracks_) {
      if (track.confirmed) {
        track.entered_state1 = true;
      }
    }
    RCLCPP_INFO(
      get_logger(),
      "静态+动态方案静态正式ID数量和位置连续%.2fs无更新：整体进入状态1。",
      logic3_static_collection_stable_seconds_);
  }

  // 只有整体状态1有效时才维护最高置信度样本池。当前帧未匹配、失联
  // 宽限或整帧地图匹配分数不足时只暂停采样，不影响ID的存在性判断。
  // READY之后仍允许更高分的新帧替换池内最低分帧，因此绿色中心会继续
  // 变准；identity_id和stability_anchor始终不随该平均中心移动。
  for (std::size_t index = 0; index < logic3_static_cluster_tracks_.size(); ++index) {
    auto & track = logic3_static_cluster_tracks_[index];
    if (!logic3_static_state1_active_ || !track.entered_state1 ||
      !track.confirmed || !track_matched[index] || track.loss_pending ||
      !std::isfinite(match_score) ||
      match_score + 1.0e-9 < logic3_static_min_match_score_)
    {
      continue;
    }

    Logic3StaticScoredSample new_sample;
    new_sample.match_score = match_score;
    new_sample.scan_sequence = score_scan_sequence;
    new_sample.cluster = track.cluster;
    bool sample_pool_changed = false;
    if (track.best_score_samples.size() <
      static_cast<std::size_t>(logic3_static_best_score_samples_))
    {
      track.best_score_samples.push_back(std::move(new_sample));
      sample_pool_changed = true;
    } else {
      const auto weakest_sample = std::min_element(
        track.best_score_samples.begin(), track.best_score_samples.end(),
        [](const Logic3StaticScoredSample & left,
        const Logic3StaticScoredSample & right) {
          if (std::abs(left.match_score - right.match_score) > 1.0e-9) {
            return left.match_score < right.match_score;
          }
          return left.scan_sequence < right.scan_sequence;
        });
      const bool higher_score =
        match_score > weakest_sample->match_score + 1.0e-9;
      const bool equal_score_but_newer =
        std::abs(match_score - weakest_sample->match_score) <= 1.0e-9 &&
        score_scan_sequence > weakest_sample->scan_sequence;
      if (higher_score || equal_score_but_newer) {
        *weakest_sample = std::move(new_sample);
        sample_pool_changed = true;
      }
    }

    if (!sample_pool_changed || track.best_score_samples.size() <
      static_cast<std::size_t>(logic3_static_best_score_samples_))
    {
      continue;
    }

    ScanPointCluster averaged_cluster;
    for (const auto & sample : track.best_score_samples) {
      averaged_cluster.center.x += sample.cluster.center.x;
      averaged_cluster.center.y += sample.cluster.center.y;
    }
    const double sample_count =
      static_cast<double>(track.best_score_samples.size());
    averaged_cluster.center.x /= sample_count;
    averaged_cluster.center.y /= sample_count;

    // 障碍几何只取最高分（同分取最新）帧，再整体平移到10帧平均中心；
    // 不把多帧原始点叠成一个虚假的宽障碍。
    const auto strongest_sample = std::max_element(
      track.best_score_samples.begin(), track.best_score_samples.end(),
      [](const Logic3StaticScoredSample & left,
      const Logic3StaticScoredSample & right) {
        if (std::abs(left.match_score - right.match_score) > 1.0e-9) {
          return left.match_score < right.match_score;
        }
        return left.scan_sequence < right.scan_sequence;
      });
    averaged_cluster.points = strongest_sample->cluster.points;
    if (averaged_cluster.points.empty()) {
      averaged_cluster.points.push_back(averaged_cluster.center);
    } else {
      const double translate_x =
        averaged_cluster.center.x - strongest_sample->cluster.center.x;
      const double translate_y =
        averaged_cluster.center.y - strongest_sample->cluster.center.y;
      for (auto & point : averaged_cluster.points) {
        point.x += translate_x;
        point.y += translate_y;
      }
    }

    // 静态阶段只冻结障碍身份和几何，不把某条参考路线的世界方向固化到
    // 障碍物。扫码选路后，每个连续局部路线命中段独立决定偏移方向。
    track.ready_obstacle = Logic3StaticObstacle{averaged_cluster};
    track.ready = true;
  }

  logic3_static_ready_ = logic3_static_state1_active_ && std::all_of(
    logic3_static_cluster_tracks_.begin(), logic3_static_cluster_tracks_.end(),
    [](const Logic3StaticClusterTrack & track) {
      return !track.confirmed || track.ready;
    });
}

double VisualLidarPathFollower::logic3_static_stability_radius_for_range(
  double obstacle_range) const
{
  if (obstacle_range <= logic3_static_stability_near_distance_) {
    return logic3_static_stability_radius_;
  }
  if (obstacle_range >= logic3_static_stability_far_distance_) {
    return logic3_static_far_stability_radius_;
  }
  const double interpolation =
    (obstacle_range - logic3_static_stability_near_distance_) /
    (logic3_static_stability_far_distance_ -
    logic3_static_stability_near_distance_);
  return logic3_static_stability_radius_ + interpolation *
         (logic3_static_far_stability_radius_ - logic3_static_stability_radius_);
}

std::vector<VisualLidarPathFollower::Logic3StaticObstacle>
VisualLidarPathFollower::logic3_ready_static_obstacles() const
{
  std::vector<Logic3StaticObstacle> obstacles;
  for (const auto & track : logic3_static_cluster_tracks_) {
    if (track.confirmed && track.entered_state1 && track.ready) {
      obstacles.push_back(track.ready_obstacle);
    }
  }
  return obstacles;
}

bool VisualLidarPathFollower::logic3_ready_snapshot_needs_preplan(
  const Logic3PlanningSnapshot & snapshot) const
{
  if (!snapshot.valid) {
    return false;
  }
  const Logic3PlanningSnapshot * reference_snapshot = nullptr;
  if (logic3_ready_preplan_candidate_pending_ &&
    logic3_ready_preplan_candidate_snapshot_.valid)
  {
    reference_snapshot = &logic3_ready_preplan_candidate_snapshot_;
  } else if (logic3_ready_preplan_valid_ &&
    logic3_ready_preplanned_snapshot_.valid)
  {
    reference_snapshot = &logic3_ready_preplanned_snapshot_;
  }
  if (!reference_snapshot ||
    snapshot.obstacles.size() != reference_snapshot->obstacles.size())
  {
    return true;
  }

  // READY障碍按稳定身份顺序导出。数量相同时，只在任一可靠中心发生
  // 配置量级的实际移动后重算；点云细碎抖动不会重复触发完整路线规划。
  for (std::size_t index = 0; index < snapshot.obstacles.size(); ++index) {
    const auto & current = snapshot.obstacles[index].cluster.center;
    const auto & planned = reference_snapshot->obstacles[index].cluster.center;
    if (std::hypot(current.x - planned.x, current.y - planned.y) + 1.0e-9 >=
      logic3_ready_preplan_min_center_change_)
    {
      return true;
    }
  }
  return false;
}

void VisualLidarPathFollower::preplan_logic3_ready_route(
  const Logic3PlanningSnapshot & snapshot)
{
  if (!logic3_ready_preplan_enabled_ || !snapshot.valid ||
    logic3_planning_pose_anchored_ || logic3_motion_candidate_ ||
    logic3_start_zone_motion_pending_ || logic3_ready_preplanning_active_ ||
    waypoints_.empty() || original_waypoints_.size() != waypoints_.size() ||
    !logic3_ready_snapshot_needs_preplan(snapshot))
  {
    return;
  }

  const auto planning_started_at = std::chrono::steady_clock::now();
  if (logic3_ready_preplan_last_at_valid_ &&
    std::chrono::duration<double>(
      planning_started_at - logic3_ready_preplan_last_at_).count() + 1.0e-9 <
    logic3_ready_preplan_min_interval_)
  {
    // 冷却期间logic3_last_complete_ready_snapshot_仍会继续刷新。后续新
    // 雷达帧到来且满0.50s后，直接对最新快照规划，不排队处理中间版本。
    return;
  }

  // 规划只发生在可信发车前的静止READY窗口。先保存上一次已提交路线，
  // 再借用冻结簇回放算法生成候选；候选完成后恢复旧路线，绝不直接覆盖。
  // 只有后续新里程计再次确认静止才会提交；若先检测到发车则丢弃候选。
  auto saved_green_clusters = std::move(logic3_green_clusters_);
  auto saved_identity_anchors = std::move(logic3_green_identity_anchors_);
  auto saved_dynamic_takeover = std::move(logic3_green_dynamic_takeover_);
  auto saved_clearance_failed = std::move(logic3_green_route_clearance_failed_);
  auto saved_waypoints = std::move(waypoints_);
  auto saved_offsets = std::move(waypoint_shift_offsets_);
  auto saved_pending_offsets = std::move(waypoint_shift_pending_offsets_);
  auto saved_pending_counts = std::move(waypoint_shift_pending_counts_);
  auto saved_missing_counts = std::move(waypoint_shift_missing_counts_);
  auto saved_dynamic_directions = std::move(waypoint_shift_dynamic_directions_);
  auto saved_shift_directions = std::move(waypoint_shift_world_directions_);
  auto saved_source_indices = std::move(waypoint_shift_source_indices_);
  auto saved_obstacle_centers = std::move(waypoint_shift_obstacle_centers_);
  auto saved_forbidden_radii = std::move(waypoint_shift_forbidden_radii_);
  const bool saved_replay = logic3_replay_frozen_clusters_;
  const bool saved_force_full_route = logic3_force_full_route_;
  const bool saved_force_confirm = logic3_force_confirm_;

  logic3_green_clusters_.clear();
  logic3_green_clusters_.reserve(snapshot.obstacles.size());
  for (const auto & obstacle : snapshot.obstacles) {
    logic3_green_clusters_.push_back(obstacle.cluster);
  }
  logic3_green_identity_anchors_.clear();
  logic3_green_dynamic_takeover_.assign(logic3_green_clusters_.size(), false);
  logic3_green_route_clearance_failed_.assign(logic3_green_clusters_.size(), false);

  waypoints_ = original_waypoints_;
  waypoint_shift_offsets_.assign(waypoints_.size(), 0.0);
  waypoint_shift_pending_offsets_.assign(waypoints_.size(), 0.0);
  waypoint_shift_pending_counts_.assign(waypoints_.size(), 0);
  waypoint_shift_missing_counts_.assign(waypoints_.size(), 0);
  waypoint_shift_dynamic_directions_.assign(waypoints_.size(), 0);
  waypoint_shift_world_directions_.assign(waypoints_.size(), Point2D());
  waypoint_shift_source_indices_.assign(waypoints_.size(), waypoints_.size());
  waypoint_shift_obstacle_centers_.assign(waypoints_.size(), Point2D());
  waypoint_shift_forbidden_radii_.assign(waypoints_.size(), 0.0);

  logic3_ready_preplanning_active_ = true;
  logic3_replay_frozen_clusters_ = true;
  logic3_force_full_route_ = true;
  logic3_force_confirm_ = true;
  update_waypoint_shifts(RobotPose(), 0);
  logic3_ready_preplan_candidate_waypoints_ = std::move(waypoints_);
  logic3_ready_preplan_candidate_offsets_ = std::move(waypoint_shift_offsets_);
  logic3_ready_preplan_candidate_shift_directions_ =
    std::move(waypoint_shift_world_directions_);
  logic3_ready_preplan_candidate_source_indices_ =
    std::move(waypoint_shift_source_indices_);
  logic3_ready_preplan_candidate_obstacle_centers_ =
    std::move(waypoint_shift_obstacle_centers_);
  logic3_ready_preplan_candidate_forbidden_radii_ =
    std::move(waypoint_shift_forbidden_radii_);
  logic3_ready_preplan_candidate_clearance_failed_ =
    logic3_green_route_clearance_failed_;
  logic3_replay_frozen_clusters_ = saved_replay;
  logic3_force_full_route_ = saved_force_full_route;
  logic3_force_confirm_ = saved_force_confirm;
  logic3_ready_preplanning_active_ = false;

  logic3_green_clusters_ = std::move(saved_green_clusters);
  logic3_green_identity_anchors_ = std::move(saved_identity_anchors);
  logic3_green_dynamic_takeover_ = std::move(saved_dynamic_takeover);
  logic3_green_route_clearance_failed_ = std::move(saved_clearance_failed);
  waypoints_ = std::move(saved_waypoints);
  waypoint_shift_offsets_ = std::move(saved_offsets);
  waypoint_shift_pending_offsets_ = std::move(saved_pending_offsets);
  waypoint_shift_pending_counts_ = std::move(saved_pending_counts);
  waypoint_shift_missing_counts_ = std::move(saved_missing_counts);
  waypoint_shift_dynamic_directions_ = std::move(saved_dynamic_directions);
  waypoint_shift_world_directions_ = std::move(saved_shift_directions);
  waypoint_shift_source_indices_ = std::move(saved_source_indices);
  waypoint_shift_obstacle_centers_ = std::move(saved_obstacle_centers);
  waypoint_shift_forbidden_radii_ = std::move(saved_forbidden_radii);
  rebuild_path_pid_route_cache();
  reset_path_pid_state();

  logic3_ready_preplan_candidate_snapshot_ = snapshot;
  logic3_ready_preplan_candidate_pending_ =
    !logic3_ready_preplan_candidate_waypoints_.empty();
  logic3_ready_preplan_candidate_odom_sequence_ = chassis_odom_sequence_;
  logic3_ready_preplan_last_at_ = planning_started_at;
  logic3_ready_preplan_last_at_valid_ = true;
}

void VisualLidarPathFollower::resolve_logic3_ready_preplan_candidate(bool moving)
{
  if (!logic3_ready_preplan_candidate_pending_) {
    return;
  }

  const auto clear_candidate = [&]() {
      logic3_ready_preplan_candidate_pending_ = false;
      logic3_ready_preplan_candidate_snapshot_ = Logic3PlanningSnapshot();
      logic3_ready_preplan_candidate_waypoints_.clear();
      logic3_ready_preplan_candidate_offsets_.clear();
      logic3_ready_preplan_candidate_shift_directions_.clear();
      logic3_ready_preplan_candidate_source_indices_.clear();
      logic3_ready_preplan_candidate_obstacle_centers_.clear();
      logic3_ready_preplan_candidate_forbidden_radii_.clear();
      logic3_ready_preplan_candidate_clearance_failed_.clear();
    };

  // 规划开始后出现任何可信运动迹象，都把本次候选视为过期。现行单线程
  // executor会在规划函数返回后处理这条里程计；由于候选从未覆盖活动路线，
  // 此时清掉它即可严格继续使用上一次完整路线。
  if (moving || logic3_motion_candidate_ || logic3_start_zone_motion_pending_) {
    clear_candidate();
    return;
  }
  if (chassis_odom_sequence_ <= logic3_ready_preplan_candidate_odom_sequence_) {
    return;
  }

  waypoints_ = std::move(logic3_ready_preplan_candidate_waypoints_);
  waypoint_shift_offsets_ = std::move(logic3_ready_preplan_candidate_offsets_);
  waypoint_shift_pending_offsets_.assign(waypoints_.size(), 0.0);
  waypoint_shift_pending_counts_.assign(waypoints_.size(), 0);
  waypoint_shift_missing_counts_.assign(waypoints_.size(), 0);
  waypoint_shift_dynamic_directions_.assign(waypoints_.size(), 0);
  waypoint_shift_world_directions_ =
    std::move(logic3_ready_preplan_candidate_shift_directions_);
  waypoint_shift_source_indices_ =
    std::move(logic3_ready_preplan_candidate_source_indices_);
  waypoint_shift_obstacle_centers_ =
    std::move(logic3_ready_preplan_candidate_obstacle_centers_);
  waypoint_shift_forbidden_radii_ =
    std::move(logic3_ready_preplan_candidate_forbidden_radii_);
  logic3_ready_preplanned_route_clearance_failed_ =
    std::move(logic3_ready_preplan_candidate_clearance_failed_);
  logic3_ready_preplanned_snapshot_ = logic3_ready_preplan_candidate_snapshot_;
  logic3_ready_preplan_valid_ = true;
  rebuild_path_pid_route_cache();
  reset_path_pid_state();
  clear_candidate();
}

void VisualLidarPathFollower::maybe_log_logic3_static_obstacles(
  std::size_t current_frame_cluster_count,
  const std::vector<Logic3StaticObstacle> & obstacles)
{
  (void)current_frame_cluster_count;
  (void)obstacles;
  if (logic3_planning_pose_anchored_ || logic3_motion_candidate_ ||
    logic3_start_zone_motion_pending_)
  {
    return;
  }

  const double now_seconds = now().seconds();
  const double log_interval_seconds = 1.0 / logic3_static_status_log_rate_hz_;
  if (logic3_static_status_log_started_ &&
    now_seconds - logic3_last_static_status_log_seconds_ < log_interval_seconds)
  {
    return;
  }
  logic3_static_status_log_started_ = true;
  logic3_last_static_status_log_seconds_ = now_seconds;

  std::vector<const Logic3StaticClusterTrack *> fixed_tracks;
  for (const auto & track : logic3_static_cluster_tracks_) {
    if (track.entered_state1 && track.confirmed) {
      fixed_tracks.push_back(&track);
    }
  }
  std::sort(
    fixed_tracks.begin(), fixed_tracks.end(),
    [](const Logic3StaticClusterTrack * left,
    const Logic3StaticClusterTrack * right) {
      return left->identity_id < right->identity_id;
    });

  std::ostringstream status;
  status << "状态1=" << (logic3_static_state1_active_ ? 1 : 0)
         << "\n静态ID坐标：";
  status << std::fixed << std::setprecision(3);
  const std::size_t rolling_scan_count = logic3_static_scan_times_.size();
  std::size_t ready_track_count = 0;
  for (const auto * track : fixed_tracks) {
    if (track->ready) {
      ++ready_track_count;
    }
    const Point2D & display_center = track->ready ?
      track->ready_obstacle.cluster.center : track->stability_anchor;
    status << "\nID" << track->identity_id << ":("
           << display_center.x << ',' << display_center.y << ") 命中="
           << track->rolling_hit_times.size() << '/' << rolling_scan_count
           << " 最高分采样=" << track->best_score_samples.size()
           << '/' << logic3_static_best_score_samples_ << "；";
  }
  status << "\nREADY=" << (logic3_static_ready_ ? 1 : 0)
         << "（独立就绪=" << ready_track_count << '/' << fixed_tracks.size()
         << "；可信发车冻结最后一次完整READY快照）";
  RCLCPP_INFO(get_logger(), "%s", status.str().c_str());
}

void VisualLidarPathFollower::record_logic3_lidar_avoidance_obstacle(
  const VisualLidarPathFollower::LidarObstacle & obstacle,
  const VisualLidarPathFollower::RobotPose & scan_to_base,
  const VisualLidarPathFollower::RobotPose & scan_to_map)
{
  if (!obstacle.found || !std::isfinite(obstacle.map_point.x) ||
    !std::isfinite(obstacle.map_point.y))
  {
    return;
  }
  // 红点入库的最后一道不变式：PID模式下已记录绿/橙掩码内
  // 不得产生实时避障红点。正常情况应已被find_lidar_obstacle过滤。
  if (tracking_controller_mode_ == "smac_pid" &&
    logic3_green_mask_contains(obstacle.map_point))
  {
    return;
  }

  // 保留原有红图ROI语义。scan_to_map=T(map<-scan)，
  // scan_to_base=T(base<-scan)，由两者还原采样时刻的base_link地图位置。
  const double map_to_base_yaw = scan_to_map.yaw - scan_to_base.yaw;
  const double cos_map_to_base = std::cos(map_to_base_yaw);
  const double sin_map_to_base = std::sin(map_to_base_yaw);
  const Point2D robot_map_point{
    scan_to_map.x -
    (cos_map_to_base * scan_to_base.x - sin_map_to_base * scan_to_base.y),
    scan_to_map.y -
    (sin_map_to_base * scan_to_base.x + cos_map_to_base * scan_to_base.y)};
  const double half_width = 0.5 * logic3_runtime_roi_width_;
  const double half_height = 0.5 * logic3_runtime_roi_height_;
  if ((logic3_runtime_roi_width_ > 0.0 &&
    std::abs(obstacle.map_point.x - robot_map_point.x) > half_width) ||
    (logic3_runtime_roi_height_ > 0.0 &&
    std::abs(obstacle.map_point.y - robot_map_point.y) > half_height) ||
    obstacle.map_point.y < logic3_runtime_channel_min_y_)
  {
    return;
  }

  // 红点只忠实记录实际雷达避障动作，不建立身份、不更新坐标、不去重。
  // 本函数只在避障动作由未激活切换为激活时调用，因此一次动作只记录一次；
  // 即使多次动作命中了相同位置，也分别保留并绘制。
  Logic3RedAvoidanceRecord record;
  record.record_id = logic3_next_red_record_id_++;
  record.center = obstacle.map_point;
  record.seen_in_task1 = task_phase_ == TaskPhase::TASK1;
  record.seen_after_task1 = !record.seen_in_task1;
  logic3_red_avoidance_records_.push_back(record);
}

bool VisualLidarPathFollower::activate_logic3_dynamic_takeover(
  const VisualLidarPathFollower::Point2D & red_point)
{
  if (!std::isfinite(red_point.x) || !std::isfinite(red_point.y) ||
    logic3_green_clusters_.empty())
  {
    return false;
  }
  if (logic3_green_dynamic_takeover_.size() > logic3_green_clusters_.size()) {
    logic3_green_dynamic_takeover_.resize(logic3_green_clusters_.size());
  }
  logic3_green_dynamic_takeover_.resize(logic3_green_clusters_.size(), false);
  if (logic3_green_route_clearance_failed_.size() > logic3_green_clusters_.size()) {
    logic3_green_route_clearance_failed_.resize(logic3_green_clusters_.size());
  }
  logic3_green_route_clearance_failed_.resize(
    logic3_green_clusters_.size(), false);

  std::size_t nearest_index = logic3_green_clusters_.size();
  double nearest_distance = logic3_red_takeover_radius_;
  for (std::size_t index = 0; index < logic3_green_clusters_.size(); ++index) {
    const auto & center = logic3_green_clusters_[index].center;
    const double distance = std::hypot(
      red_point.x - center.x, red_point.y - center.y);
    if (distance <= nearest_distance) {
      nearest_distance = distance;
      nearest_index = index;
    }
  }
  if (nearest_index >= logic3_green_clusters_.size() ||
    logic3_green_dynamic_takeover_[nearest_index])
  {
    return false;
  }

  logic3_green_dynamic_takeover_[nearest_index] = true;
  const bool is_orange = logic3_orange_green_boundary_locked_ &&
    nearest_index >= logic3_orange_green_begin_index_;
  RCLCPP_WARN(
    get_logger(),
    "实际雷达避障动态接管%s点ID%zu：红点map(%.3f, %.3f)，"
    "永久点map(%.3f, %.3f)，距离=%.3fm<=%.2fm；"
    "本次发车只取消该ID静态偏移，绿/橙雷达掩码继续保留。",
    is_orange ? "橙" : "绿", nearest_index + 1,
    red_point.x, red_point.y,
    logic3_green_clusters_[nearest_index].center.x,
    logic3_green_clusters_[nearest_index].center.y,
    nearest_distance, logic3_red_takeover_radius_);
  return true;
}

bool VisualLidarPathFollower::logic3_identity_update_crosses_protected_route(
  const Point2D & identity_anchor, const Point2D & candidate_center) const
{
  const double move_x = candidate_center.x - identity_anchor.x;
  const double move_y = candidate_center.y - identity_anchor.y;
  if (move_x * move_x + move_y * move_y <= 1.0e-12) {
    return false;
  }

  // 检查“固定锚点→候选中心”是否真正穿过保护折线。只使用有限路线段，
  // 不把局部切线无限延伸，避免在弯角外侧误拒绝更新。
  auto crosses_route = [&](const std::vector<Waypoint> & route) {
      if (route.size() < 2) {
        return false;
      }
      constexpr double kSideTolerance = 1.0e-8;
      constexpr double kSegmentTolerance = 1.0e-7;
      for (std::size_t index = 0; index + 1 < route.size(); ++index) {
        const double route_x = route[index + 1].x - route[index].x;
        const double route_y = route[index + 1].y - route[index].y;
        const double route_length_squared = route_x * route_x + route_y * route_y;
        if (route_length_squared <= 1.0e-12) {
          continue;
        }
        const double anchor_side =
          route_x * (identity_anchor.y - route[index].y) -
          route_y * (identity_anchor.x - route[index].x);
        const double candidate_side =
          route_x * (candidate_center.y - route[index].y) -
          route_y * (candidate_center.x - route[index].x);

        // 锚点恰在线上时允许观测离开，否则该ID将永远无法更新。
        if (std::abs(anchor_side) <= kSideTolerance) {
          continue;
        }
        if (std::abs(candidate_side) > kSideTolerance &&
          anchor_side * candidate_side >= 0.0)
        {
          continue;
        }
        const double denominator = anchor_side - candidate_side;
        if (std::abs(denominator) <= 1.0e-12) {
          continue;
        }
        const double move_ratio = anchor_side / denominator;
        if (move_ratio <= kSegmentTolerance ||
          move_ratio > 1.0 + kSegmentTolerance)
        {
          continue;
        }
        const double intersection_x = identity_anchor.x + move_ratio * move_x;
        const double intersection_y = identity_anchor.y + move_ratio * move_y;
        const double route_ratio =
          ((intersection_x - route[index].x) * route_x +
          (intersection_y - route[index].y) * route_y) / route_length_squared;
        if (route_ratio >= -kSegmentTolerance &&
          route_ratio <= 1.0 + kSegmentTolerance)
        {
          return true;
        }
      }
      return false;
    };

  // Task1阶段也保护其未偏移基准路线，避免绿/橙/红永久中心因更新穿过
  // Task1巡线轨迹换到另一侧。
  if (task_phase_ == TaskPhase::TASK1) {
    return crosses_route(task1_original_route_);
  }

  const std::string direction = to_lower(selected_route_direction_);
  if (direction == "clockwise") {
    if (crosses_route(logic3_clockwise_update_guard_route_)) {
      return true;
    }
  } else if (direction == "counterclockwise") {
    if (crosses_route(logic3_counterclockwise_update_guard_route_)) {
      return true;
    }
  } else if (crosses_route(logic3_clockwise_update_guard_route_) ||
    crosses_route(logic3_counterclockwise_update_guard_route_))
  {
    // 二维码方向尚未确定时，两条原始路线都作为保护线。
    return true;
  }

  // 每次进入过渡或倒车后重规划都会替换这条最新无偏移ACB前缀。
  return crosses_route(logic3_latest_channel_entry_update_guard_route_);
}

int VisualLidarPathFollower::logic3_runtime_green_required_confirm_scans(
  std::size_t point_count) const
{
  if (logic3_runtime_green_confirm_scans_by_point_count_.empty()) {
    return 7;
  }
  // 参数的第5项同时覆盖5个及以上点；空簇按最严格的1点处理，避免
  // 异常输入绕过确认门槛。
  const std::size_t normalized_point_count = std::max<std::size_t>(1U, point_count);
  const std::size_t parameter_index = std::min<std::size_t>(
    normalized_point_count, logic3_runtime_green_confirm_scans_by_point_count_.size()) - 1U;
  return static_cast<int>(std::max<std::int64_t>(
      3, logic3_runtime_green_confirm_scans_by_point_count_[parameter_index]));
}

bool VisualLidarPathFollower::update_logic3_runtime_green_tracks(
  const std::vector<VisualLidarPathFollower::ScanPointCluster> & clusters,
  double speed, double speed_step,
  double map_odom_translation_correction,
  double map_odom_yaw_correction_deg)
{
  // 身份锚点与logic3_green_clusters_永远按索引对齐。旧状态或
  // 异常路径缺失锚点时，仅用当前中心补齐一次；后续绝不随
  // cluster.center更新。
  if (logic3_green_identity_anchors_.size() > logic3_green_clusters_.size()) {
    logic3_green_identity_anchors_.resize(logic3_green_clusters_.size());
  }
  while (logic3_green_identity_anchors_.size() < logic3_green_clusters_.size()) {
    logic3_green_identity_anchors_.push_back(
      logic3_green_clusters_[logic3_green_identity_anchors_.size()].center);
  }
  if (logic3_green_dynamic_takeover_.size() > logic3_green_clusters_.size()) {
    logic3_green_dynamic_takeover_.resize(logic3_green_clusters_.size());
  }
  logic3_green_dynamic_takeover_.resize(logic3_green_clusters_.size(), false);
  if (logic3_green_route_clearance_failed_.size() > logic3_green_clusters_.size()) {
    logic3_green_route_clearance_failed_.resize(logic3_green_clusters_.size());
  }
  logic3_green_route_clearance_failed_.resize(
    logic3_green_clusters_.size(), false);

  std::vector<bool> track_matched(logic3_runtime_green_tracks_.size(), false);
  bool route_replan_required = false;

  auto refresh_recent_cluster = [&](Logic3RuntimeGreenTrack & track) {
      Point2D averaged_center;
      for (const auto & sample : track.recent_samples) {
        averaged_center.x += sample.center.x;
        averaged_center.y += sample.center.y;
      }
      const double sample_count = static_cast<double>(track.recent_samples.size());
      averaged_center.x /= sample_count;
      averaged_center.y /= sample_count;

      // 位置使用最近高分帧的中心平均，点云形状只取最新帧并整体平移；
      // 这样不会把车辆运动期间的多帧点云拖成一个很宽的障碍物。
      track.cluster = track.recent_samples.back();
      const double translate_x = averaged_center.x - track.cluster.center.x;
      const double translate_y = averaged_center.y - track.cluster.center.y;
      for (auto & point : track.cluster.points) {
        point.x += translate_x;
        point.y += translate_y;
      }
      track.cluster.center = averaged_center;
    };

  auto find_green_identity = [&](const Point2D & center) {
      std::size_t best_index = logic3_green_clusters_.size();
      double best_distance = logic3_runtime_green_match_radius_;
      for (std::size_t index = 0; index < logic3_green_clusters_.size(); ++index) {
        // 身份ID关联只和永久库当前中心比较；固定锚点在身份确定后再
        // 独立决定这个ID是否允许接受新坐标。
        const auto & current_center = logic3_green_clusters_[index].center;
        const double distance = std::hypot(
          center.x - current_center.x,
          center.y - current_center.y);
        if (distance <= best_distance) {
          best_distance = distance;
          best_index = index;
        }
      }
      return best_index;
    };
  auto find_current_permanent_coverage = [&](const Point2D & center) {
      std::size_t best_index = logic3_green_clusters_.size();
      double best_distance = logic3_old_cluster_radius_;
      for (std::size_t index = 0; index < logic3_green_clusters_.size(); ++index) {
        const auto & current_center = logic3_green_clusters_[index].center;
        const double distance = std::hypot(
          center.x - current_center.x, center.y - current_center.y);
        if (distance <= best_distance) {
          best_distance = distance;
          best_index = index;
        }
      }
      return best_index;
  };

  for (const auto & cluster : clusters) {
    const int observation_required_confirm_scans =
      logic3_runtime_green_required_confirm_scans(cluster.points.size());
    std::size_t best_track = logic3_runtime_green_tracks_.size();
    double best_distance = logic3_runtime_green_match_radius_;
    for (std::size_t index = 0; index < logic3_runtime_green_tracks_.size(); ++index) {
      if (track_matched[index]) {
        continue;
      }
      const auto & candidate_track = logic3_runtime_green_tracks_[index];
      const bool has_permanent_identity = candidate_track.linked_to_green &&
        candidate_track.green_index < logic3_green_clusters_.size();
      const auto & center = has_permanent_identity ?
        logic3_green_clusters_[candidate_track.green_index].center :
        candidate_track.latest_center;
      const double distance = std::hypot(
        cluster.center.x - center.x, cluster.center.y - center.y);
      if (distance <= best_distance) {
        best_distance = distance;
        best_track = index;
      }
    }

    if (best_track == logic3_runtime_green_tracks_.size()) {
      Logic3RuntimeGreenTrack track;
      track.cluster = cluster;
      track.latest_center = cluster.center;
      track.consecutive_hits = 1;
      track.required_confirm_scans = observation_required_confirm_scans;
      track.recent_samples.push_back(cluster);
      const std::size_t green_index = find_green_identity(cluster.center);
      if (green_index < logic3_green_clusters_.size()) {
        track.linked_to_green = true;
        track.green_index = green_index;
      }
      logic3_runtime_green_tracks_.push_back(std::move(track));
      track_matched.push_back(true);
      best_track = logic3_runtime_green_tracks_.size() - 1;
    } else {
      auto & track = logic3_runtime_green_tracks_[best_track];
      if (track.missing_count > 0) {
        // 中断后重新出现必须重新积累连续高分帧；永久绿点本身不受影响。
        track.confirmed = false;
        track.consecutive_hits = 0;
        track.required_confirm_scans = observation_required_confirm_scans;
        track.recent_samples.clear();
      } else {
        // 同一段连续观测内，点数偶尔变少时采用见过的最严格门槛，不能
        // 因后一帧点数重新变多就立刻放宽。
        track.required_confirm_scans = std::max(
          track.required_confirm_scans, observation_required_confirm_scans);
      }
      track.latest_center = cluster.center;
      track.missing_count = 0;
      ++track.consecutive_hits;
      track.recent_samples.push_back(cluster);
      const int retained_sample_count = std::max(
        logic3_runtime_green_average_samples_, track.required_confirm_scans);
      while (track.recent_samples.size() >
        static_cast<std::size_t>(retained_sample_count))
      {
        track.recent_samples.pop_front();
      }
      track_matched[best_track] = true;
    }

    auto & track = logic3_runtime_green_tracks_[best_track];
    // 若连续段后半程才出现更严格的低点数观测，缓存还需要补满新的完整
    // 确认窗口；不能只凭已经很大的consecutive_hits立即采用该帧。
    if (track.consecutive_hits < track.required_confirm_scans ||
      track.recent_samples.size() <
      static_cast<std::size_t>(track.required_confirm_scans))
    {
      continue;
    }
    refresh_recent_cluster(track);
    track.confirmed = true;

    // 候选确认时再查一次全局绿色集合，防止两个临时候选先后确认后
    // 为同一实体追加两个永久绿点。
    if (!track.linked_to_green ||
      track.green_index >= logic3_green_clusters_.size())
    {
      const std::size_t green_index = find_green_identity(track.cluster.center);
      if (green_index < logic3_green_clusters_.size()) {
        track.linked_to_green = true;
        track.green_index = green_index;
      } else {
        // READY锚点只负责约束既有绿色身份不能被连续小步拖走。判断一个
        // 候选能否成为“新增橙点”时，必须再与所有绿/橙永久点的当前更新
        // 中心比较。0.25m只负责严格身份关联；无法关联身份后，只要候选仍在
        // 任一永久点配置过滤覆盖范围内，就视为该永久障碍的点云抖动，禁止
        // 新增橙点，也不把原ID拉出固定身份锚点。
        const std::size_t current_center_index =
          find_current_permanent_coverage(track.cluster.center);
        if (current_center_index < logic3_green_clusters_.size()) {
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "运行绿/橙点候选map(%.3f, %.3f)无法通过%.2fm身份半径关联，"
            "但位于永久点%zu当前中心%.2fm过滤覆盖内，不新增橙点。",
            track.cluster.center.x, track.cluster.center.y,
            logic3_runtime_green_match_radius_, current_center_index + 1,
            logic3_old_cluster_radius_);
          continue;
        }
        track.green_index = logic3_green_clusters_.size();
        track.linked_to_green = true;
        logic3_green_clusters_.push_back(track.cluster);
        // 发车后新橙点没有READY坐标，以首次连续确认的
        // 中心作为它的固定身份锚点。
        logic3_green_identity_anchors_.push_back(track.cluster.center);
        logic3_green_dynamic_takeover_.push_back(false);
        logic3_green_route_clearance_failed_.push_back(false);
        route_replan_required = true;
        const double seconds_since_trusted_departure =
          task1_departure_timer_started_ ?
          std::max(
            0.0,
            std::chrono::duration<double>(
              std::chrono::steady_clock::now() - task1_departure_started_at_).count()) :
          0.0;
        RCLCPP_WARN(
          get_logger(),
          "@@@@@@@橙点：(%.3f，%.3f)@@@@@@@此时匹配score为：%.3f；"
          "此时距离可信发车的时间为：%.3f秒；"
          "speed=%.3fm/s；speed_step=%.3fm/s；"
          "map_odom_translation_correction=%.3fm；"
          "map_odom_yaw_correction_deg=%.3fdeg。",
          track.cluster.center.x, track.cluster.center.y,
          latest_lidar_match_score_, seconds_since_trusted_departure,
          speed, speed_step, map_odom_translation_correction,
          map_odom_yaw_correction_deg);
        continue;
      }
    }

    const std::size_t green_index = track.green_index;
    const std::size_t departure_green_count =
      logic3_orange_green_boundary_locked_ ?
      std::min(logic3_orange_green_begin_index_, logic3_green_clusters_.size()) :
      logic3_green_clusters_.size();
    if (logic3_runtime_orange_only_update_enabled_ &&
      green_index < departure_green_count)
    {
      // 橙点专用更新模式：可信发车时的READY绿色身份、坐标和点云保持冻结。
      // 它们发车时已经产生的静态绕行及配置半径雷达掩码继续有效；这里只禁止
      // 运行高分观测移动绿色中心并触发新的绿色路线偏移。
      continue;
    }

    // 身份已经确定后才进入固定锚点更新约束。锚点只保存READY绿色或
    // 橙点首次确认的位置，绝不随永久库中心更新；越界只拒绝本次坐标，
    // 不解除ID关联，也绝不能因此新增一个橙点。
    const auto & identity_anchor = logic3_green_identity_anchors_[green_index];
    const double anchor_distance = std::hypot(
      track.cluster.center.x - identity_anchor.x,
      track.cluster.center.y - identity_anchor.y);
    if (anchor_distance > logic3_runtime_green_anchor_update_radius_ + 1.0e-9) {
      continue;
    }

    // 可选的路线换边保护。允许跨线时完全跳过几何检查，身份关联半径、
    // 固定锚点更新半径、永久库写入阈值和重规划阈值仍照常生效。
    if (!logic3_runtime_green_allow_route_crossing_ &&
      logic3_identity_update_crosses_protected_route(
        identity_anchor, track.cluster.center))
    {
      continue;
    }

    const auto & previous_center = logic3_green_clusters_[green_index].center;
    const double distance_from_permanent = std::hypot(
      track.cluster.center.x - previous_center.x,
      track.cluster.center.y - previous_center.y);
    // 小于写入阈值时永久库保持原值；临时高分轨迹仍持续观察，因此多帧
    // 小幅同向变化可以累计，最终达到阈值后再一次性写入。
    if (distance_from_permanent + 1.0e-9 <
      logic3_runtime_green_update_min_distance_)
    {
      continue;
    }
    logic3_green_clusters_[green_index] = track.cluster;
    const bool has_planned_center =
      green_index < logic3_green_last_planned_centers_.size();
    const double distance_from_planned = has_planned_center ? std::hypot(
      track.cluster.center.x - logic3_green_last_planned_centers_[green_index].x,
      track.cluster.center.y - logic3_green_last_planned_centers_[green_index].y) :
      std::numeric_limits<double>::infinity();
    if (!has_planned_center ||
      distance_from_planned + 1.0e-9 >= logic3_runtime_green_replan_min_distance_)
    {
      route_replan_required = true;
    }
  }

  for (std::size_t index = 0; index < logic3_runtime_green_tracks_.size(); ++index) {
    if (!track_matched[index]) {
      auto & track = logic3_runtime_green_tracks_[index];
      ++track.missing_count;
      track.consecutive_hits = 0;
    }
  }
  logic3_runtime_green_tracks_.erase(
    std::remove_if(
      logic3_runtime_green_tracks_.begin(), logic3_runtime_green_tracks_.end(),
      [&](const Logic3RuntimeGreenTrack & track) {
        // 这里只清除短期关联轨迹；已经写入的永久绿/橙点不会因失联删除。
        return track.missing_count >=
        logic3_runtime_green_candidate_release_scans_;
      }),
    logic3_runtime_green_tracks_.end());
  // 历史红点只用于绘图，绝不因绿/橙点后来更新靠近而反向补做接管。
  // 路径接管只能由apply_channel_lidar_obstacle_avoidance在动作触发瞬间决定。
  return route_replan_required;
}

void VisualLidarPathFollower::rebuild_logic3_route_from_green_clusters(
  const char * reason)
{
  if (!logic3_planning_pose_anchored_ || waypoints_.empty() ||
    original_waypoints_.size() != waypoints_.size() ||
    task_phase_ == TaskPhase::UNKNOWN ||
    task_phase_ == TaskPhase::FINISHED)
  {
    return;
  }
  if (enable_static_map_filter_ && require_map_for_lidar_assist_ &&
    (!latest_map_ || !latest_restriction_map_))
  {
    return;
  }
  const auto planning_started_at = std::chrono::steady_clock::now();

  // 每次都从当前阶段的原始路线重算。未被动态接管的永久点继续贡献静态
  // 偏移；接管点仍留在点库和图片中，但本次发车不再影响静态路线。
  waypoints_ = original_waypoints_;
  waypoint_shift_offsets_.assign(waypoints_.size(), 0.0);
  waypoint_shift_pending_offsets_.assign(waypoints_.size(), 0.0);
  waypoint_shift_pending_counts_.assign(waypoints_.size(), 0);
  waypoint_shift_missing_counts_.assign(waypoints_.size(), 0);
  waypoint_shift_dynamic_directions_.assign(waypoints_.size(), 0);
  waypoint_shift_world_directions_.assign(waypoints_.size(), Point2D());
  waypoint_shift_source_indices_.assign(waypoints_.size(), waypoints_.size());
  waypoint_shift_obstacle_centers_.assign(waypoints_.size(), Point2D());
  waypoint_shift_forbidden_radii_.assign(waypoints_.size(), 0.0);

  logic3_replay_frozen_clusters_ = true;
  logic3_force_full_route_ = true;
  logic3_force_confirm_ = true;
  update_waypoint_shifts(RobotPose(), 0);
  logic3_force_confirm_ = false;
  logic3_force_full_route_ = false;
  logic3_replay_frozen_clusters_ = false;

  logic3_frozen_base_offsets_ = waypoint_shift_offsets_;
  logic3_frozen_base_shift_directions_ = waypoint_shift_world_directions_;
  logic3_frozen_base_source_indices_ = waypoint_shift_source_indices_;
  logic3_frozen_base_obstacle_centers_ = waypoint_shift_obstacle_centers_;
  logic3_frozen_base_forbidden_radii_ = waypoint_shift_forbidden_radii_;
  logic3_green_last_planned_centers_.clear();
  logic3_green_last_planned_centers_.reserve(logic3_green_clusters_.size());
  for (const auto & green_cluster : logic3_green_clusters_) {
    logic3_green_last_planned_centers_.push_back(green_cluster.center);
  }
  logic3_green_route_replan_pending_ = false;

  logic3_baseline_original_route_ = original_waypoints_;
  logic3_baseline_route_ = waypoints_;
  logic3_visualization_route_ = waypoints_;
  if (task_phase_ == TaskPhase::TASK1) {
    task1_frozen_route_ = waypoints_;
  } else {
    final_frozen_route_ = waypoints_;
  }
  reset_path_pid_state();
  publish_debug_route();
  const auto planning_finished_at = std::chrono::steady_clock::now();
  logic3_last_route_replan_duration_ms_ = 1000.0 *
    std::chrono::duration<double>(
    planning_finished_at - planning_started_at).count();
  logic3_last_runtime_route_replan_at_ = planning_finished_at;
  logic3_last_runtime_route_replan_at_valid_ = true;
  RCLCPP_WARN(
    get_logger(), "%s：用%zu个永久绿点重新计算%s路线；耗时=%.2fms。",
    reason ? reason : "运行绿点更新", logic3_green_clusters_.size(),
    task_phase_name(task_phase_), logic3_last_route_replan_duration_ms_);
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

std::string VisualLidarPathFollower::make_logic3_session_id() const
{
  const std::time_t timestamp = std::time(nullptr);
  std::tm local_time;
  localtime_r(&timestamp, &local_time);
  std::ostringstream stream;
  stream << std::put_time(&local_time, "%Y.%m.%d_%H:%M:%S");
  return stream.str();
}

void VisualLidarPathFollower::freeze_logic3_route()
{
  const Logic3PlanningSnapshot & snapshot = logic3_motion_pending_snapshot_;
  logic3_green_clusters_.clear();
  logic3_green_identity_anchors_.clear();
  logic3_green_dynamic_takeover_.clear();
  logic3_green_route_clearance_failed_.clear();
  if (snapshot.valid) {
    for (const auto & obstacle : snapshot.obstacles) {
      logic3_green_clusters_.push_back(obstacle.cluster);
      // 可信发车后绿色身份始终围绕这个READY坐标关联；
      // obstacle.cluster.center本身仍允许由运行高分观测修正。
      logic3_green_identity_anchors_.push_back(obstacle.cluster.center);
      logic3_green_dynamic_takeover_.push_back(false);
      logic3_green_route_clearance_failed_.push_back(false);
    }
    if (snapshot.obstacles.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "静态+动态方案可信发车时没有已独立完成状态1和10次平均的绿色簇；"
        "按规则不使用原始帧兜底，将冻结空快照。");
    }
  } else {
    RCLCPP_WARN(
      get_logger(),
      "静态+动态方案发车候选时状态1或READY无效；按规则不使用原始帧兜底，将记录为空。");
  }
  logic3_orange_green_begin_index_ = logic3_green_clusters_.size();
  // 颜色边界就在可信发车冻结这一刻锁定：READY快照保持绿色，之后由
  // match_score高分跨帧逻辑新增的永久点统一画成橙色，与二维码无关。
  logic3_orange_green_boundary_locked_ = true;
  logic3_green_last_planned_centers_.clear();
  logic3_green_last_planned_centers_.reserve(logic3_green_clusters_.size());
  for (const auto & green_cluster : logic3_green_clusters_) {
    logic3_green_last_planned_centers_.push_back(green_cluster.center);
  }
  logic3_green_route_replan_pending_ = false;

  logic3_planning_pose_anchored_ = true;
  logic3_motion_candidate_ = false;
  logic3_start_zone_motion_pending_ = false;
  logic3_runtime_green_tracks_.clear();
  logic3_red_avoidance_records_.clear();
  logic3_next_red_record_id_ = 1;
  logic3_task1_before_red_snapshot_count_ = 0;
  logic3_final_before_red_snapshot_count_ = 0;
  channel_lidar_waiting_for_post_reverse_scan_ = false;
  channel_lidar_post_reverse_scan_sequence_ = 0;
  channel_lidar_post_reverse_time_nanoseconds_ = 0;
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
  task1_lidar_avoid_count_ = 0;
  transition_lidar_avoid_count_ = 0;
  task2_lidar_avoid_count_ = 0;
  task3_lidar_avoid_count_ = 0;
  task1_lidar_last_avoid_direction_ = 0;
  transition_lidar_last_avoid_direction_ = 0;
  task2_lidar_last_avoid_direction_ = 0;
  task3_lidar_last_avoid_direction_ = 0;
  qr_reverse_rearm_after_post_stop_ = false;
  logic3_final_snapshot_published_ = false;
  logic3_final_pose_valid_ = false;
  logic3_actual_route_recording_ = false;
  logic3_actual_route_.clear();
  logic3_session_id_ = make_logic3_session_id();
  logic3_last_pose_correction_at_ = std::chrono::steady_clock::now();
  task1_departure_timer_started_ = true;
  task1_departure_started_at_ = std::chrono::steady_clock::now();
  task1_qr_timeout_logged_ = false;
  task1_endpoint_fallback_logged_ = false;
  task1_endpoint_wait_timer_started_ = false;
  task1_endpoint_wait_started_at_ = std::chrono::steady_clock::time_point{};
  task1_pending_qr_is_fallback_ = false;

  if (logic3_planning_pose_source_ != "live_lidar_location") {
    RobotPose live_map_odom;
    if (lookup_pose(
        frame_id_, logic3_odom_frame_id_,
        rclcpp::Time(0, 0, get_clock()->get_clock_type()), live_map_odom))
    {
      logic3_map_odom_plan_ = live_map_odom;
      logic3_map_odom_plan_valid_ = true;
    }
  }

  RCLCPP_WARN(
    get_logger(),
    "静态障碍记录完毕：冻结%zu个绿色障碍坐标及点云，"
    "发车后新增永久点将画成橙色；开始Task1计时，输出批次=%s。",
    logic3_green_clusters_.size(),
    logic3_session_id_.c_str());

  if (quiet_logs_after_static_freeze_) {
    const rcutils_ret_t result = rcutils_logging_set_logger_level(
      get_logger().get_name(), RCUTILS_LOG_SEVERITY_ERROR);
    if (result != RCUTILS_RET_OK) {
      RCLCPP_ERROR(
        get_logger(), "静态冻结后切换终端静默模式失败，继续保留原日志级别。");
      rcutils_reset_error();
    }
  }

  // 可信发车优先直接冻结与本次snapshot完全一致的上一次完整预规划路线。
  // 规划中的候选已经在第一条运动里程计到来时丢弃，绝不在这里等待它。
  if (task_phase_ == TaskPhase::TASK1 && !waypoints_.empty()) {
    const bool reuse_ready_preplan = logic3_ready_preplan_enabled_ &&
      logic3_ready_preplan_valid_ && snapshot.valid &&
      logic3_ready_preplanned_snapshot_.valid &&
      std::abs(
      snapshot.stamp_seconds -
      logic3_ready_preplanned_snapshot_.stamp_seconds) <= 1.0e-9;
    if (reuse_ready_preplan) {
      logic3_green_route_clearance_failed_ =
        logic3_ready_preplanned_route_clearance_failed_;
      logic3_green_route_clearance_failed_.resize(
        logic3_green_clusters_.size(), false);
      commit_logic3_current_route_as_frozen("task1_frozen");
    } else {
      apply_logic3_frozen_clusters_to_current_route("task1_frozen");
    }
  }
}

void VisualLidarPathFollower::commit_logic3_current_route_as_frozen(
  const std::string & snapshot_event)
{
  logic3_frozen_base_offsets_ = waypoint_shift_offsets_;
  logic3_frozen_base_shift_directions_ = waypoint_shift_world_directions_;
  logic3_frozen_base_source_indices_ = waypoint_shift_source_indices_;
  logic3_frozen_base_obstacle_centers_ = waypoint_shift_obstacle_centers_;
  logic3_frozen_base_forbidden_radii_ = waypoint_shift_forbidden_radii_;
  logic3_green_last_planned_centers_.clear();
  logic3_green_last_planned_centers_.reserve(logic3_green_clusters_.size());
  for (const auto & green_cluster : logic3_green_clusters_) {
    logic3_green_last_planned_centers_.push_back(green_cluster.center);
  }
  logic3_green_route_replan_pending_ = false;
  // 静态回放只留下当前永久绿点集合的基础偏移；清空规划确认计数。
  // 运行中新增/修正绿点时会从原始路线和完整绿色集合重新计算。
  std::fill(waypoint_shift_pending_offsets_.begin(), waypoint_shift_pending_offsets_.end(), 0.0);
  std::fill(waypoint_shift_pending_counts_.begin(), waypoint_shift_pending_counts_.end(), 0);
  std::fill(waypoint_shift_missing_counts_.begin(), waypoint_shift_missing_counts_.end(), 0);
  std::fill(
    waypoint_shift_dynamic_directions_.begin(),
    waypoint_shift_dynamic_directions_.end(), 0);
  logic3_baseline_original_route_ = original_waypoints_;
  logic3_baseline_route_ = waypoints_;
  logic3_visualization_route_ = waypoints_;
  logic3_final_snapshot_published_ = false;
  logic3_final_pose_valid_ = false;
  if (snapshot_event == "task1_frozen") {
    task1_original_route_ = logic3_baseline_original_route_;
    task1_frozen_route_ = logic3_baseline_route_;
    task1_dynamic_original_route_.clear();
    task1_dynamic_route_.clear();
  } else if (snapshot_event == "final_frozen") {
    final_original_route_ = logic3_baseline_original_route_;
    final_frozen_route_ = logic3_baseline_route_;
  }
  publish_logic3_snapshot(snapshot_event);
  RCLCPP_WARN(
    get_logger(),
    "路线已规划完毕：已将%zu个冻结绿色障碍按局部路线段独立方向应用到%s路线。",
    logic3_green_clusters_.size(), selected_route_direction_.c_str());
}

void VisualLidarPathFollower::apply_logic3_frozen_clusters_to_current_route(
  const std::string & snapshot_event)
{
  if (!logic3_planning_pose_anchored_) {
    return;
  }
  // 没有可复用预规划版本时，也必须从当前阶段原始路线重新开始，避免
  // READY旧候选或上一次静态偏移残留到本次冻结结果。
  waypoints_ = original_waypoints_;
  waypoint_shift_offsets_.assign(waypoints_.size(), 0.0);
  waypoint_shift_pending_offsets_.assign(waypoints_.size(), 0.0);
  waypoint_shift_pending_counts_.assign(waypoints_.size(), 0);
  waypoint_shift_missing_counts_.assign(waypoints_.size(), 0);
  waypoint_shift_dynamic_directions_.assign(waypoints_.size(), 0);
  waypoint_shift_world_directions_.assign(waypoints_.size(), Point2D());
  waypoint_shift_source_indices_.assign(waypoints_.size(), waypoints_.size());
  waypoint_shift_obstacle_centers_.assign(waypoints_.size(), Point2D());
  waypoint_shift_forbidden_radii_.assign(waypoints_.size(), 0.0);
  logic3_green_route_clearance_failed_.assign(logic3_green_clusters_.size(), false);
  logic3_replay_frozen_clusters_ = true;
  logic3_force_full_route_ = true;
  logic3_force_confirm_ = true;
  update_waypoint_shifts(RobotPose(), 0);
  logic3_force_confirm_ = false;
  logic3_force_full_route_ = false;
  logic3_replay_frozen_clusters_ = false;
  commit_logic3_current_route_as_frozen(snapshot_event);
}

void VisualLidarPathFollower::start_logic3_actual_route(
  const VisualLidarPathFollower::RobotPose & pose)
{
  logic3_actual_route_.clear();
  logic3_actual_route_recording_ = logic3_actual_route_enabled_;
  record_logic3_actual_route(pose, true);
}

void VisualLidarPathFollower::record_logic3_actual_route(
  const VisualLidarPathFollower::RobotPose & pose, bool force)
{
  if (!logic3_actual_route_recording_ ||
    !std::isfinite(pose.x) || !std::isfinite(pose.y))
  {
    return;
  }
  if (!force && !logic3_actual_route_.empty()) {
    const Point2D & previous = logic3_actual_route_.back();
    if (std::hypot(pose.x - previous.x, pose.y - previous.y) <
      logic3_actual_route_min_distance_)
    {
      return;
    }
  }
  if (force && !logic3_actual_route_.empty()) {
    Point2D & previous = logic3_actual_route_.back();
    if (std::hypot(pose.x - previous.x, pose.y - previous.y) < 1.0e-6) {
      previous = Point2D{pose.x, pose.y};
      return;
    }
  }
  logic3_actual_route_.push_back(Point2D{pose.x, pose.y});
}

void VisualLidarPathFollower::publish_logic3_snapshot(const std::string & event)
{
  if (!logic3_snapshot_pub_ || logic3_session_id_.empty()) {
    return;
  }
  const bool task1_before_red_event = event == "task1_before_red";
  const bool final_before_red_event = event == "final_before_red";
  const bool image_capture_event = event == "image_capture";
  const bool final_transition_marker_event = event == "final_transition_marker";
  const bool task1_event = event == "task1_frozen" ||
    event == "task1_dynamic" || task1_before_red_event;
  const bool dynamic_event = event == "task1_dynamic" ||
    event == "final_dynamic" || task1_before_red_event ||
    final_before_red_event;
  const bool task1_dynamic_event = event == "task1_dynamic" ||
    task1_before_red_event;
  const std::vector<Waypoint> & original_route =
    image_capture_event ?
    (final_original_route_.empty() ? original_waypoints_ : final_original_route_) :
    (task1_dynamic_event && !task1_dynamic_original_route_.empty() ?
    task1_dynamic_original_route_ :
    (task1_event ? task1_original_route_ : final_original_route_));
  const std::vector<Waypoint> & frozen_route =
    image_capture_event ?
    (final_frozen_route_.empty() ? waypoints_ : final_frozen_route_) :
    (task1_dynamic_event && !task1_dynamic_route_.empty() ?
    task1_dynamic_route_ :
    (task1_event ? task1_frozen_route_ : final_frozen_route_));
  // 04图严格以最新03冻结路线为基线，只额外叠加红色历史。
  const std::vector<Waypoint> & plotted_route = frozen_route;
  const bool include_qr_marker =
    task1_transition_marker_valid_ &&
    (event == "task1_dynamic" || final_transition_marker_event ||
    event == "final_frozen" ||
    event == "final_dynamic" || task1_before_red_event ||
    final_before_red_event);
  const bool include_route_start_marker =
    final_frozen_marker_valid_ &&
    (event == "task1_dynamic" || final_transition_marker_event ||
    event == "final_frozen" ||
    event == "final_dynamic" || task1_before_red_event ||
    final_before_red_event);
  auto append_route = [](std::ostringstream & stream, const std::vector<Waypoint> & route) {
      stream << '[';
      for (std::size_t index = 0; index < route.size(); ++index) {
        if (index > 0) {
          stream << ',';
        }
        stream << '[' << route[index].x << ',' << route[index].y << ']';
      }
      stream << ']';
    };
  auto append_clusters = [](
    std::ostringstream & stream, const std::vector<ScanPointCluster> & clusters) {
      stream << '[';
      for (std::size_t index = 0; index < clusters.size(); ++index) {
        if (index > 0) {
          stream << ',';
        }
        stream << '[' << clusters[index].center.x << ',' << clusters[index].center.y << ']';
      }
      stream << ']';
    };
  auto append_points = [](std::ostringstream & stream, const std::vector<Point2D> & points) {
      stream << '[';
      for (std::size_t index = 0; index < points.size(); ++index) {
        if (index > 0) {
          stream << ',';
        }
        stream << '[' << points[index].x << ',' << points[index].y << ']';
      }
      stream << ']';
    };
  std::vector<Point2D> plotted_red_clusters;
  if (event == "task1_dynamic" || task1_before_red_event) {
    // 02图逐次保留Task1期间真正触发过的雷达避障动作，不去重。
    for (const auto & record : logic3_red_avoidance_records_) {
      if (record.seen_in_task1) {
        plotted_red_clusters.push_back(record.center);
      }
    }
  } else if (event == "final_dynamic" || final_before_red_event) {
    // 04图逐次保留Task1结束后真正触发过的雷达避障动作，不去重。
    for (const auto & record : logic3_red_avoidance_records_) {
      if (record.seen_after_task1) {
        plotted_red_clusters.push_back(record.center);
      }
    }
  }
  const std::size_t orange_green_begin_index =
    logic3_orange_green_boundary_locked_ ?
    std::min(logic3_orange_green_begin_index_, logic3_green_clusters_.size()) :
    logic3_green_clusters_.size();

  std::ostringstream json;
  json << std::setprecision(10);
  json << "{\"event\":\"" << json_escape(event)
       << "\",\"session\":\"" << json_escape(logic3_session_id_)
       << "\",\"frozen_revision\":" << channel_completed_reverse_count_
       << ",\"output_root\":\"" << json_escape(logic3_output_root_)
       << "\",\"map_yaml\":\"" << json_escape(logic3_restriction_map_yaml_)
       << "\",\"red_radius\":" << logic3_red_takeover_radius_
       << ",\"green_radius\":" << logic3_old_cluster_radius_
       << ",\"green_red_filter_radius\":" << logic3_green_red_filter_radius_
       << ",\"green_identity_radius\":" << logic3_runtime_green_match_radius_
       << ",\"green_anchor_update_radius\":" <<
    logic3_runtime_green_anchor_update_radius_
       << ",\"green_update_min_distance\":" <<
    logic3_runtime_green_update_min_distance_
       << ",\"red_trigger_index\":" <<
    (task1_before_red_event ? logic3_task1_before_red_snapshot_count_ :
    (final_before_red_event ? logic3_final_before_red_snapshot_count_ : 0))
       << ",\"orange_green_begin_index\":" << orange_green_begin_index
       << ",\"base_zone\":{\"min_x\":" << return_base_min_x_
       << ",\"max_x\":" << return_base_max_x_
       << ",\"min_y\":" << return_base_min_y_
       << ",\"max_y\":" << return_base_max_y_ << '}'
       << ",\"parking_zone\":{\"min_x\":" << return_stop_zone_min_x_
       << ",\"top_min_x\":" << return_stop_zone_top_min_x_
       << ",\"top_max_x\":" << return_stop_zone_top_max_x_
       << ",\"max_x\":" << return_stop_zone_max_x_
       << ",\"min_y\":" << return_stop_zone_min_y_
       << ",\"right_top_y\":" << return_stop_zone_right_top_y_
       << ",\"max_y\":" << return_stop_zone_max_y_
       << ",\"left_curve_control_x\":" <<
    return_stop_zone_left_curve_control_x_
       << ",\"left_curve_control_y\":" <<
    return_stop_zone_left_curve_control_y_
       << ",\"confirm_seconds\":" << return_stop_confirm_seconds_ << '}'
       << ",\"task3_slowdown_zone\":{\"enabled\":"
       << (task3_return_slowdown_enabled_ ? "true" : "false")
       << ",\"center_x\":" << task3_terminal_target_x_
       << ",\"center_y\":" << task3_terminal_target_y_
       << ",\"radius\":" << task3_return_slowdown_radius_
       << ",\"start_speed\":" << task3_profile_.linear_speed
       << ",\"min_speed\":" << task3_return_slowdown_min_speed_ << '}'
       << ",\"qr_marker\":";
  if (include_qr_marker) {
    json << "{\"point\":[" << task1_transition_marker_.x << ','
         << task1_transition_marker_.y << "],\"label\":\""
         << json_escape(task1_transition_marker_label_) << "\"}";
  } else {
    json << "null";
  }
  json << ",\"route_start_marker\":";
  if (include_route_start_marker) {
    json << "{\"point\":[" << final_frozen_marker_.x << ','
         << final_frozen_marker_.y << "],\"yaw\":"
         << final_frozen_marker_yaw_ << ",\"heading_distance\":"
         << channel_entry_heading_reference_distance_ << ",\"label\":\""
         << json_escape(final_frozen_marker_label_) << "\"}";
  } else {
    json << "null";
  }
  json << ",\"task3_terminal_target_marker\":";
  if (task3_terminal_homing_used_) {
    json << "{\"point\":[" << task3_terminal_target_x_ << ','
         << task3_terminal_target_y_
         << "],\"label\":\"TASK3 FALLBACK\"}";
  } else {
    json << "null";
  }
  json << ",\"image_capture_corridor\":";
  if (image_capture_event) {
    const std::string direction = to_lower(selected_route_direction_);
    const bool clockwise = direction == "clockwise";
    const bool counterclockwise = direction == "counterclockwise";
    if (clockwise || counterclockwise) {
      const double enter_x = clockwise ?
        image_capture_clockwise_enter_x_ : image_capture_counterclockwise_enter_x_;
      const double exit_x = clockwise ?
        image_capture_clockwise_exit_x_ : image_capture_counterclockwise_exit_x_;
      json << "{\"enabled\":true,\"direction\":\""
           << json_escape(direction)
           << "\",\"min_x\":" << std::min(enter_x, exit_x)
           << ",\"max_x\":" << std::max(enter_x, exit_x)
           << ",\"min_y\":" << image_capture_corridor_min_y_
           << ",\"max_y\":" << image_capture_corridor_max_y_
           << ",\"speed\":" << image_capture_corridor_speed_ << '}';
    } else {
      json << "null";
    }
  } else {
    json << "null";
  }
  json << ",\"image_capture_stop_marker\":";
  if (image_capture_event && image_capture_stop_point_valid_) {
    json << "{\"point\":[" << image_capture_stop_point_.x << ','
         << image_capture_stop_point_.y << "],\"yaw\":"
         << image_capture_stop_point_yaw_ << ",\"label\":\"STOP\"}";
  } else {
    json << "null";
  }
  json << ",\"image_capture_marker\":";
  if (image_capture_event && image_capture_point_valid_) {
    json << "{\"point\":[" << image_capture_point_.x << ','
         << image_capture_point_.y << "],\"yaw\":"
         << image_capture_point_yaw_ << ",\"label\":\"CAPTURE\"}";
  } else {
    json << "null";
  }
  json << ",\"final_pose\":";
  if (event == "final_dynamic" && logic3_final_pose_valid_) {
    json << '[' << logic3_final_pose_.x << ',' << logic3_final_pose_.y << ','
         << logic3_final_pose_.yaw << ']';
  } else {
    json << "null";
  }
  json << ",\"original\":";
  append_route(json, original_route);
  json << ",\"baseline\":";
  append_route(json, frozen_route);
  json << ",\"route\":";
  append_route(json, plotted_route);
  json << ",\"actual_route_enabled\":"
       << (logic3_actual_route_enabled_ ? "true" : "false")
       << ",\"actual_route_line_width\":" << logic3_actual_route_line_width_
       << ",\"actual_route_color\":\""
       << json_escape(logic3_actual_route_color_) << "\""
       << ",\"actual_route\":";
  append_points(
    json,
    (event == "final_dynamic") ? logic3_actual_route_ : std::vector<Point2D>());
  json << ",\"green\":";
  append_clusters(json, logic3_green_clusters_);
  json << ",\"red\":";
  append_points(json, dynamic_event ? plotted_red_clusters : std::vector<Point2D>());
  json << '}';

  std_msgs::msg::String message;
  message.data = json.str();
  logic3_snapshot_pub_->publish(message);
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

// 静态+动态方案 waypoint_shift：每帧新雷达提前检查当前目标之后的固定数量航点。
// 以障碍物最近路线点为中心，前后分别取配置数量的点并横移。
// 静态绿色障碍使路线向其相对位置的反侧绕行。
// 横移量先作为本帧候选，连续确认后才提交；障碍消失或错误观测消失后会释放回原路线。
void VisualLidarPathFollower::update_waypoint_shifts(
  const VisualLidarPathFollower::RobotPose & robot_pose,
  std::size_t target_waypoint_index)
{
  const bool replay_frozen_clusters = logic3_replay_frozen_clusters_;
  const bool runtime_green_score_pending =
    logic3_planning_pose_anchored_ && !replay_frozen_clusters &&
    logic3_runtime_green_update_enabled_ &&
    (!latest_lidar_match_score_valid_ ||
    latest_match_score_scan_sequence_ != scan_sequence_);
  const bool runtime_green_diagnostics_pending =
    logic3_planning_pose_anchored_ && !replay_frozen_clusters &&
    logic3_runtime_green_update_enabled_ &&
    logic3_runtime_green_motion_gate_enabled_ &&
    (!latest_match_diagnostics_valid_ ||
    latest_match_diagnostics_scan_sequence_ != scan_sequence_);
  const bool static_green_score_pending =
    !logic3_planning_pose_anchored_ && !replay_frozen_clusters &&
    (!latest_lidar_match_score_valid_ ||
    latest_match_score_scan_sequence_ != scan_sequence_);
  if ((!latest_scan_ && !replay_frozen_clusters) ||
    (processed_waypoint_shift_scan_sequence_ == scan_sequence_ && !replay_frozen_clusters) ||
    runtime_green_score_pending || runtime_green_diagnostics_pending ||
    static_green_score_pending ||
    original_waypoints_.size() != waypoints_.size() ||
    waypoint_shift_offsets_.size() != waypoints_.size() ||
    waypoint_shift_pending_offsets_.size() != waypoints_.size() ||
    waypoint_shift_pending_counts_.size() != waypoints_.size() ||
    waypoint_shift_missing_counts_.size() != waypoints_.size() ||
    waypoint_shift_dynamic_directions_.size() != waypoints_.size() ||
    waypoint_shift_world_directions_.size() != waypoints_.size() ||
    waypoint_shift_source_indices_.size() != waypoints_.size() ||
    waypoint_shift_obstacle_centers_.size() != waypoints_.size() ||
    waypoint_shift_forbidden_radii_.size() != waypoints_.size() || waypoints_.empty())
  {
    return;
  }
  if (!replay_frozen_clusters) {
    processed_waypoint_shift_scan_sequence_ = scan_sequence_;
  }

  if (enable_static_map_filter_ && require_map_for_lidar_assist_ &&
    (!latest_map_ || !latest_restriction_map_))
  {
    return;
  }

  // 发车后的全图更新允许单点候选进入跨帧轨迹，再由点数对应的连续
  // 雷达帧门槛过滤；READY前静态点库仍沿用独立的静态最小点数参数。
  const int minimum_cluster_points = !logic3_planning_pose_anchored_ ?
    logic3_static_cluster_min_points_ : 1;
  const bool use_logic3_static_restriction_map =
    !logic3_planning_pose_anchored_ && !replay_frozen_clusters;
  // 运行中必须保留旧绿点圈内的点云，才能在高match_score时重新测量其
  // 位置。实时避障的绿色掩码在控制分支单独执行。
  const bool filter_logic3_green_mask =
    logic3_planning_pose_anchored_ && !replay_frozen_clusters &&
    !logic3_runtime_green_update_enabled_;
  RobotPose cluster_observation_pose = robot_pose;
  std::vector<ScanPointCluster> clusters = replay_frozen_clusters ?
    logic3_green_clusters_ : collect_non_static_scan_clusters_map(
    robot_pose, minimum_cluster_points, use_logic3_static_restriction_map,
    filter_logic3_green_mask, &cluster_observation_pose);
  if (replay_frozen_clusters && !logic3_green_dynamic_takeover_.empty()) {
    std::vector<ScanPointCluster> static_offset_clusters;
    static_offset_clusters.reserve(clusters.size());
    for (std::size_t index = 0; index < clusters.size(); ++index) {
      const bool dynamic_takeover =
        index < logic3_green_dynamic_takeover_.size() &&
        logic3_green_dynamic_takeover_[index];
      if (!dynamic_takeover) {
        static_offset_clusters.push_back(clusters[index]);
      }
    }
    clusters = std::move(static_offset_clusters);
  }

  // 远距离时同一障碍物的1~3个回波可能被无效量程或地图过滤切成多个
  // 单点段。静止阶段先按0.20m做传递连通合并：A靠近B、B靠近C时三者
  // 必须属于同一分量，不能受输入顺序或逐次更新质心影响而残留第二个簇。
  // 跨帧ID关联仍使用logic3_static_cluster_match_radius，不使用绿色冻结圆。
  if (!logic3_planning_pose_anchored_ && !replay_frozen_clusters)
  {
    std::vector<bool> consumed(clusters.size(), false);
    std::vector<ScanPointCluster> merged_clusters;
    merged_clusters.reserve(clusters.size());
    for (std::size_t seed = 0; seed < clusters.size(); ++seed) {
      if (consumed[seed]) {
        continue;
      }
      consumed[seed] = true;
      std::vector<std::size_t> component{seed};
      for (std::size_t cursor = 0; cursor < component.size(); ++cursor) {
        const std::size_t current = component[cursor];
        for (std::size_t candidate = 0; candidate < clusters.size(); ++candidate) {
          if (consumed[candidate]) {
            continue;
          }
          if (std::hypot(
              clusters[current].center.x - clusters[candidate].center.x,
              clusters[current].center.y - clusters[candidate].center.y) >
            logic3_static_same_frame_merge_radius_)
          {
            continue;
          }
          consumed[candidate] = true;
          component.push_back(candidate);
        }
      }

      ScanPointCluster merged;
      for (const std::size_t index : component) {
        merged.points.insert(
          merged.points.end(), clusters[index].points.begin(), clusters[index].points.end());
      }
      if (merged.points.empty()) {
        for (const std::size_t index : component) {
          merged.center.x += clusters[index].center.x;
          merged.center.y += clusters[index].center.y;
        }
        const double component_count = static_cast<double>(component.size());
        merged.center.x /= component_count;
        merged.center.y /= component_count;
      } else {
        for (const auto & point : merged.points) {
          merged.center.x += point.x;
          merged.center.y += point.y;
        }
        const double point_count = static_cast<double>(merged.points.size());
        merged.center.x /= point_count;
        merged.center.y /= point_count;
      }
      merged_clusters.push_back(std::move(merged));
    }
    clusters = std::move(merged_clusters);
  }

  if (use_logic3_static_restriction_map) {
    // Task1发车前只收集用户指定map矩形内的绿色候选。该坐标门限
    // 不影响运行红点记录、实时避障或后续Task2/Task3。
    clusters.erase(
      std::remove_if(
        clusters.begin(), clusters.end(), [&](const ScanPointCluster & cluster) {
          const bool in_static_green_region =
            cluster.center.x > task1_static_min_x_ &&
            cluster.center.y > task1_static_min_y_;
          return !in_static_green_region;
        }),
      clusters.end());
  }

  if (logic3_planning_pose_anchored_ && !replay_frozen_clusters)
  {
    if (logic3_runtime_green_update_enabled_) {
      std::vector<ScanPointCluster> green_observations;
      const bool should_measure_motion =
        logic3_runtime_green_motion_gate_enabled_ ||
        logic3_runtime_green_gate_log_rate_hz_ > 0.0;
      double steering_span = std::numeric_limits<double>::infinity();
      bool measured_steering_within_limit = true;
      bool steering_measurement_valid = false;
      double minimum_abs_speed = std::numeric_limits<double>::infinity();
      double maximum_speed_step = std::numeric_limits<double>::infinity();
      bool measured_speed_within_limits = true;
      bool speed_measurement_valid = false;
      if (should_measure_motion) {
        measured_steering_within_limit =
          logic3_runtime_green_steering_is_stable(*latest_scan_, &steering_span);
        steering_measurement_valid = std::isfinite(steering_span);
        measured_speed_within_limits = logic3_runtime_green_speed_is_stable(
          *latest_scan_, &minimum_abs_speed, &maximum_speed_step);
        speed_measurement_valid = std::isfinite(minimum_abs_speed) &&
          std::isfinite(maximum_speed_step);
      }
      const double map_odom_translation_correction_abs =
        std::abs(latest_map_odom_translation_correction_);
      const bool measured_translation_within_limit =
        map_odom_translation_correction_abs <=
        logic3_runtime_green_max_map_odom_translation_correction_;
      const double map_odom_yaw_correction_abs_deg =
        std::abs(latest_map_odom_yaw_correction_) * 180.0 / M_PI;
      const bool measured_yaw_within_limit =
        map_odom_yaw_correction_abs_deg <=
        logic3_runtime_green_max_map_odom_yaw_correction_deg_;
      record_logic3_runtime_green_gate_log_sample(
        steering_span, steering_measurement_valid,
        minimum_abs_speed, maximum_speed_step, speed_measurement_valid,
        map_odom_translation_correction_abs,
        map_odom_yaw_correction_abs_deg,
        measured_steering_within_limit, measured_speed_within_limits,
        measured_translation_within_limit, measured_yaw_within_limit);
      const bool steering_is_stable =
        !logic3_runtime_green_motion_gate_enabled_ ||
        (steering_measurement_valid && measured_steering_within_limit);
      const bool speed_is_stable =
        !logic3_runtime_green_motion_gate_enabled_ ||
        (speed_measurement_valid && measured_speed_within_limits);
      const bool map_odom_translation_is_stable =
        !logic3_runtime_green_motion_gate_enabled_ || measured_translation_within_limit;
      const bool map_odom_yaw_is_stable =
        !logic3_runtime_green_motion_gate_enabled_ || measured_yaw_within_limit;
      const bool motion_is_stable =
        steering_is_stable && speed_is_stable &&
        map_odom_translation_is_stable && map_odom_yaw_is_stable;
      if (latest_lidar_match_score_ >= logic3_runtime_green_min_match_score_ &&
        motion_is_stable)
      {
        // 全局绿点库由同圈匹配分数、发布速度/转向稳定度、map->odom
        // 平移/航向修正和跨帧确认共同决定，不受当前任务阶段、Task1
        // 静态矩形、Task3高度分界或运行红点ROI限制。
        green_observations.reserve(clusters.size());
        for (const auto & cluster : clusters) {
          const double distance_from_robot = std::hypot(
            cluster.center.x - cluster_observation_pose.x,
            cluster.center.y - cluster_observation_pose.y);
          // 近车点云最容易混入车体、基地边缘或运动畸变。可信发车后，
          // 以本雷达帧采样时刻的base_link为圆心统一拦截绿/橙候选：
          // 边界也属于排除区。过滤后短期轨迹会按“观测中断”清零，
          // 因此车辆离开后必须重新完整累计连续可信帧。
          if (logic3_runtime_green_robot_exclusion_radius_ > 0.0 &&
            distance_from_robot <=
            logic3_runtime_green_robot_exclusion_radius_ + 1.0e-9)
          {
            continue;
          }
          green_observations.push_back(cluster);
        }
      } else if (latest_lidar_match_score_ >= logic3_runtime_green_min_match_score_ &&
        logic3_runtime_green_motion_gate_enabled_)
      {
        RCLCPP_DEBUG_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "高分雷达帧不更新绿/橙点：转向跨度=%.3frad(上限%.3f)，"
          "最低速度幅值=%.3fm/s(下限%.3f)，"
          "相邻发布速度跳变=%.3fm/s(上限%.3f)，"
          "map->odom平移修正=%.3fm(上限%.3fm)，"
          "map->odom航向修正=%.2fdeg(上限%.2fdeg)。",
          steering_span, logic3_runtime_green_max_steering_span_,
          minimum_abs_speed, logic3_runtime_green_min_speed_,
          maximum_speed_step, logic3_runtime_green_max_speed_step_,
          map_odom_translation_correction_abs,
          logic3_runtime_green_max_map_odom_translation_correction_,
          map_odom_yaw_correction_abs_deg,
          logic3_runtime_green_max_map_odom_yaw_correction_deg_);
      }
      if (update_logic3_runtime_green_tracks(
          green_observations,
          minimum_abs_speed, maximum_speed_step,
          map_odom_translation_correction_abs,
          map_odom_yaw_correction_abs_deg))
      {
        logic3_green_route_replan_pending_ = true;
      }
      const auto runtime_replan_now = std::chrono::steady_clock::now();
      const bool runtime_replan_interval_elapsed =
        !logic3_last_runtime_route_replan_at_valid_ ||
        std::chrono::duration<double>(
        runtime_replan_now - logic3_last_runtime_route_replan_at_).count() >=
        logic3_runtime_green_replan_min_interval_;
      if (logic3_green_route_replan_pending_ && runtime_replan_interval_elapsed &&
        (task_phase_ != TaskPhase::TRANSITION ||
        tracking_controller_mode_ == "smac_pid"))
      {
        // smac_pid障碍策略在所有正向阶段都及时把确认点写入路线；
        // legacy_lookahead仍保留过渡段只积累、进入Task2后统一应用的旧行为。
        // 冷却期间点库继续更新，pending保持为true；到期后直接使用最新点库，
        // 不会丢失第一次不准的橙点后续修正。
        rebuild_logic3_route_from_green_clusters("高置信度全局绿点更新");
      }
    }

    // 红点不再由这里的全部运行簇生成；只有实时雷达真正覆盖控制命令时，
    // 才由apply_channel_lidar_obstacle_avoidance记录对应命中点。
    return;
  }

  // Task1只让静态矩形内的冻结绿色簇参与静态路径偏移；矩形外的冻结
  // 绿色簇不偏移，并由上面的绿圈放行逻辑交给实时动态雷达。
  if (replay_frozen_clusters &&
    (following_qr_path_ || task_phase_ == TaskPhase::TASK1) &&
    tracking_controller_mode_ != "smac_pid")
  {
    clusters.erase(
      std::remove_if(
        clusters.begin(), clusters.end(),
        [&](const ScanPointCluster & cluster) {
          return !task1_static_avoid_region_contains(cluster.center);
        }),
      clusters.end());
  }

  const bool full_route = logic3_force_full_route_ || replay_frozen_clusters;
  const std::size_t first_index = full_route ? 0 :
    std::min(target_waypoint_index, waypoints_.size() - 1);
  const std::size_t last_index = full_route ? waypoints_.size() : std::min(
    waypoints_.size(), first_index + static_cast<std::size_t>(waypoint_shift_check_count_));
  const std::size_t managed_last_index = full_route ? waypoints_.size() - 1 : std::min(
    waypoints_.size() - 1,
    first_index + static_cast<std::size_t>(waypoint_shift_check_count_) +
    static_cast<std::size_t>(std::max({
      task1_waypoint_shift_points_after_,
      transition_waypoint_shift_points_after_,
      task23_waypoint_shift_points_after_})));
  const double check_radius_squared =
    waypoint_cloud_check_radius_ * waypoint_cloud_check_radius_;

  struct ShiftCandidateSegment
  {
    std::size_t obstacle_waypoint{0};
    std::size_t first{0};
    std::size_t last{0};
    double left_required_offset{0.0};
    double right_required_offset{0.0};
    int preferred_direction{0};
    Point2D left_normal;
    Point2D obstacle_center;
    double forbidden_radius{0.0};
  };
  struct RouteSegmentMatch
  {
    bool valid{false};
    std::size_t segment{0};
    double t{0.0};
    double distance_squared{std::numeric_limits<double>::infinity()};
    Point2D tangent;
    Point2D projection;
  };
  std::vector<ShiftCandidateSegment> candidate_segments;

  auto match_logic3_route_segment = [&](
    const ScanPointCluster & cluster, std::size_t segment_index,
    RouteSegmentMatch & match)
    {
      match = RouteSegmentMatch();
      if (segment_index + 1 >= original_waypoints_.size()) {
        return false;
      }
      const auto & start = original_waypoints_[segment_index];
      const auto & finish = original_waypoints_[segment_index + 1];
      const double segment_x = finish.x - start.x;
      const double segment_y = finish.y - start.y;
      const double length_squared = segment_x * segment_x + segment_y * segment_y;
      if (length_squared < 1e-8) {
        return false;
      }

      auto projection = [&](const Point2D & point) {
          const double raw_t =
            ((point.x - start.x) * segment_x +
            (point.y - start.y) * segment_y) / length_squared;
          return std::max(0.0, std::min(1.0, raw_t));
        };
      const double center_t = projection(cluster.center);
      const double projection_x = start.x + center_t * segment_x;
      const double projection_y = start.y + center_t * segment_y;
      double cloud_distance_squared = std::numeric_limits<double>::infinity();
      auto accumulate_distance = [&](const Point2D & point) {
          const double point_t = projection(point);
          const double point_projection_x = start.x + point_t * segment_x;
          const double point_projection_y = start.y + point_t * segment_y;
          const double dx = point.x - point_projection_x;
          const double dy = point.y - point_projection_y;
          cloud_distance_squared = std::min(cloud_distance_squared, dx * dx + dy * dy);
        };
      if (cluster.points.empty()) {
        accumulate_distance(cluster.center);
      } else {
        for (const auto & point : cluster.points) {
          accumulate_distance(point);
        }
      }

      const double length = std::sqrt(length_squared);
      match.valid = std::isfinite(cloud_distance_squared);
      match.segment = segment_index;
      match.t = center_t;
      match.distance_squared = cloud_distance_squared;
      match.tangent = Point2D{segment_x / length, segment_y / length};
      match.projection = Point2D{projection_x, projection_y};
      return match.valid;
    };

  auto append_shift_candidate = [&](
    const ScanPointCluster & cluster, const RouteSegmentMatch & route_match,
    std::size_t cluster_route_first, std::size_t cluster_route_last,
    int preferred_direction)
    {
      if (!route_match.valid || cluster_route_first >= cluster_route_last) {
        return;
      }
      const Point2D left_normal{-route_match.tangent.y, route_match.tangent.x};
      const Point2D right_normal{-left_normal.x, -left_normal.y};
      double target_clearance = task1_waypoint_shift_target_clearance_;
      if (route_deformation_mode_ == "circle_envelope") {
        target_clearance = circle_envelope_target_clearance_;
      } else {
        switch (task_phase_) {
          case TaskPhase::TRANSITION:
            target_clearance = transition_waypoint_shift_target_clearance_;
            break;
          case TaskPhase::TASK2:
            target_clearance = task2_waypoint_shift_target_clearance_;
            break;
          case TaskPhase::TASK3:
            target_clearance = task3_waypoint_shift_target_clearance_;
            break;
          case TaskPhase::TASK1:
          case TaskPhase::UNKNOWN:
          case TaskPhase::FINISHED:
            break;
        }
      }

      auto required_offset_along = [&](const Point2D & shift_normal) {
          if (route_deformation_mode_ == "circle_envelope") {
            // circle_envelope按点云中心的固定禁入圆直接构造几何包络。
            // target_clearance就是圆半径，不再叠加点云自身拟合半径。
            const double center_along_shift =
              (cluster.center.x - route_match.projection.x) * shift_normal.x +
              (cluster.center.y - route_match.projection.y) * shift_normal.y;
            return std::max(
              0.0, center_along_shift + circle_envelope_target_clearance_);
          }
          double cloud_edge_along_shift = -std::numeric_limits<double>::infinity();
          if (cluster.points.empty()) {
            cloud_edge_along_shift =
              (cluster.center.x - route_match.projection.x) * shift_normal.x +
              (cluster.center.y - route_match.projection.y) * shift_normal.y;
          } else {
            for (const auto & point : cluster.points) {
              const double lateral_distance =
                (point.x - route_match.projection.x) * shift_normal.x +
                (point.y - route_match.projection.y) * shift_normal.y;
              cloud_edge_along_shift = std::max(cloud_edge_along_shift, lateral_distance);
            }
          }
          return std::max(
            0.0, cloud_edge_along_shift + target_clearance);
        };
      const double left_required_offset = required_offset_along(left_normal);
      const double right_required_offset = required_offset_along(right_normal);
      if (left_required_offset <= 1e-9 && right_required_offset <= 1e-9) {
        return;
      }
      double cloud_radius = 0.0;
      for (const auto & point : cluster.points) {
        cloud_radius = std::max(
          cloud_radius,
          std::hypot(point.x - cluster.center.x, point.y - cluster.center.y));
      }
      const double forbidden_radius = route_deformation_mode_ == "circle_envelope" ?
        circle_envelope_target_clearance_ + circle_envelope_numeric_margin_ :
        cloud_radius + target_clearance + circle_envelope_numeric_margin_;

      std::size_t obstacle_waypoint = route_match.t < 0.5 ?
        route_match.segment : route_match.segment + 1;
      obstacle_waypoint = std::max(
        cluster_route_first,
        std::min(obstacle_waypoint, original_waypoints_.size() - 1));
      int configured_points_before = task23_waypoint_shift_points_before_;
      int configured_points_after = task23_waypoint_shift_points_after_;
      if (following_qr_path_ || task_phase_ == TaskPhase::TASK1) {
        configured_points_before = task1_waypoint_shift_points_before_;
        configured_points_after = task1_waypoint_shift_points_after_;
      } else if (channel_entry_waypoint_index_ > 0 &&
        obstacle_waypoint < channel_entry_waypoint_index_)
      {
        configured_points_before = transition_waypoint_shift_points_before_;
        configured_points_after = transition_waypoint_shift_points_after_;
      }
      const std::size_t points_before =
        static_cast<std::size_t>(configured_points_before);
      const std::size_t points_after =
        static_cast<std::size_t>(configured_points_after);
      const std::size_t influence_first = std::max(
        cluster_route_first,
        obstacle_waypoint > points_before ? obstacle_waypoint - points_before : 0);
      const std::size_t influence_last = std::min(
        std::min(original_waypoints_.size() - 1, cluster_route_last - 1),
        obstacle_waypoint + points_after);
      candidate_segments.push_back(
        ShiftCandidateSegment{
          obstacle_waypoint, influence_first, influence_last, left_required_offset,
          right_required_offset, preferred_direction, left_normal,
          cluster.center, forbidden_radius});
    };

  if (!logic3_planning_pose_anchored_ && !replay_frozen_clusters)
  {
    // 第一条发车候选已经保存READY平均快照后，停止接收后续可能包含底盘
    // 震动的静态帧；候选若取消，下一帧会恢复观察并按连续性规则重建。
    if (logic3_motion_candidate_ || logic3_start_zone_motion_pending_) {
      return;
    }
    // 新channel路线会从扫码/倒车后的base_link向正式首点延长，因此此时
    // 不能再拿旧channel路线预筛大厅静态簇。保留原ID、稳定帧和最高分采样，
    // 规划阶段再由新路线的检查半径决定哪些绿色簇真正产生偏移。
    update_logic3_static_cluster_tracks(
      clusters, robot_pose, latest_lidar_match_score_,
      latest_match_score_scan_sequence_);
    const auto ready_obstacles = logic3_ready_static_obstacles();
    if (logic3_static_ready_ && !ready_obstacles.empty()) {
      // 只在完整READY状态下刷新。可信发车一旦出现运动预兆，静态更新会
      // 先暂停，因此这里保存的始终是运动前最后一次完整READY快照。
      logic3_last_complete_ready_snapshot_.valid = true;
      logic3_last_complete_ready_snapshot_.stamp_seconds = now().seconds();
      logic3_last_complete_ready_snapshot_.obstacles = ready_obstacles;
      preplan_logic3_ready_route(logic3_last_complete_ready_snapshot_);
    }
    maybe_log_logic3_static_obstacles(clusters.size(), ready_obstacles);
    // READY期间只提交已经完整算完、并由后一帧静止里程计确认的路线；
    // 尚在计算或尚未确认的候选绝不覆盖当前活动路线。
    return;
  }

  {
    struct Logic3SegmentAssignment
    {
      bool valid{false};
      std::size_t cluster_index{0};
      std::size_t route_first{0};
      std::size_t route_last{0};
      RouteSegmentMatch match;
    };
    std::vector<Logic3SegmentAssignment> assignments(original_waypoints_.size() - 1);

    // 每条路线段只归属检查半径内最近的障碍点云。相同障碍在路线索引上
    // 不连续时会自然形成多个局部命中段，不需要入口/出口高度特判。
    for (std::size_t cluster_index = 0; cluster_index < clusters.size(); ++cluster_index) {
      // 只在二维码倒车完成后的进场路线重生成时，用当时A沿yaw前移配置
      // 距离后的虚拟参考点和原yaw对永久点做一次车前车后分类。
      // 车后点仍可匹配A后保护长度之外及
      // 后续任务路线，但其直接命中及before窗口不得倒灌到A点附近。
      const double relative_x = clusters[cluster_index].center.x -
        channel_entry_rear_static_filter_start_x_;
      const double relative_y = clusters[cluster_index].center.y -
        channel_entry_rear_static_filter_start_y_;
      const bool behind_channel_entry_start =
        channel_entry_rear_static_filter_active_ &&
        relative_x * std::cos(channel_entry_rear_static_filter_start_yaw_) +
        relative_y * std::sin(channel_entry_rear_static_filter_start_yaw_) < 0.0;
      // 分界高度只给“整个障碍簇”选择处理器，不能拿它裁剪路线点。
      // 因此这里只阻止下方障碍匹配返程段；上方障碍一旦匹配成功，后续
      // influence_first..influence_last仍可完整跨过return_obstacle_split_y。
      const bool return_lidar_only = tracking_controller_mode_ != "smac_pid" &&
        clusters[cluster_index].center.y <= return_obstacle_split_y_;
      const std::size_t cluster_route_first = behind_channel_entry_start ?
        std::max(
        first_index,
        std::min(
          channel_entry_rear_static_filter_first_index_,
          original_waypoints_.size() - 1)) : first_index;
      const std::size_t cluster_route_last = last_index;
      const std::size_t segment_first = cluster_route_first > 0 ?
        cluster_route_first - 1 : cluster_route_first;
      const std::size_t segment_last = std::min(
        cluster_route_last, original_waypoints_.size() - 1);
      for (std::size_t segment_index = segment_first;
        segment_index < segment_last; ++segment_index)
      {
        if (return_lidar_only && task3_route_available_ &&
          segment_index >= task3_start_index_)
        {
          continue;
        }
        RouteSegmentMatch match;
        if (!match_logic3_route_segment(clusters[cluster_index], segment_index, match) ||
          match.distance_squared > check_radius_squared)
        {
          continue;
        }
        auto & assignment = assignments[segment_index];
        if (!assignment.valid ||
          match.distance_squared + 1e-12 < assignment.match.distance_squared)
        {
          assignment.valid = true;
          assignment.cluster_index = cluster_index;
          assignment.route_first = cluster_route_first;
          assignment.route_last = cluster_route_last;
          assignment.match = match;
        }
      }
    }

    std::size_t segment_index = 0;
    while (segment_index < assignments.size()) {
      if (!assignments[segment_index].valid) {
        ++segment_index;
        continue;
      }
      const std::size_t run_first = segment_index;
      const std::size_t cluster_index = assignments[segment_index].cluster_index;
      std::size_t run_last = segment_index;
      while (run_last + 1 < assignments.size() &&
        assignments[run_last + 1].valid &&
        assignments[run_last + 1].cluster_index == cluster_index)
      {
        ++run_last;
      }

      const Logic3SegmentAssignment * anchor = &assignments[run_first];
      for (std::size_t index = run_first + 1; index <= run_last; ++index) {
        if (assignments[index].match.distance_squared < anchor->match.distance_squared) {
          anchor = &assignments[index];
        }
      }
      const auto & cluster = clusters[cluster_index];
      const Point2D left_normal{-anchor->match.tangent.y, anchor->match.tangent.x};
      const double obstacle_lateral =
        (cluster.center.x - anchor->match.projection.x) * left_normal.x +
        (cluster.center.y - anchor->match.projection.y) * left_normal.y;
      // +1表示路线左移，-1表示路线右移。每个连续局部命中段只由自己的
      // 最近锚点决定一次方向，随后扩展before/after，避免逐点方向抖动。
      const int preferred_direction = obstacle_lateral >= 0.0 ? -1 : 1;
      append_shift_candidate(
        cluster, anchor->match, anchor->route_first, anchor->route_last,
        preferred_direction);
      if (debug_control_logs_ && !logic3_ready_preplanning_active_) {
        RCLCPP_INFO(
          get_logger(),
          "debug: 静态+动态方案局部路线命中：绿色簇(%.2f, %.2f)，路线段=%zu-%zu，"
          "锚点=%zu，向%s。",
          cluster.center.x, cluster.center.y, run_first + 1, run_last + 1,
          anchor->match.segment + 1, preferred_direction > 0 ? "左" : "右");
      }
      segment_index = run_last + 1;
    }
  }

  // 所有候选都先基于未偏移的original_waypoints_独立确定锚点、方向和距离。
  // circle_envelope只确认这些独立锚点，绝不让前一个障碍物的影响窗口覆盖
  // 后一个障碍物；legacy_offset仍保留原来的分段窗口写入行为。
  std::sort(
    candidate_segments.begin(), candidate_segments.end(),
    [&](const ShiftCandidateSegment & lhs, const ShiftCandidateSegment & rhs) {
      if (lhs.obstacle_waypoint != rhs.obstacle_waypoint)
      {
        return lhs.obstacle_waypoint < rhs.obstacle_waypoint;
      }
      if (lhs.first != rhs.first) {
        return lhs.first < rhs.first;
      }
      return lhs.last < rhs.last;
    });

  std::vector<double> observed_offsets(waypoints_.size(), 0.0);
  std::vector<bool> observed_offsets_valid(waypoints_.size(), false);
  std::vector<Point2D> observed_world_directions(waypoints_.size(), Point2D());
  std::vector<std::size_t> observed_source_indices(
    waypoints_.size(), waypoints_.size());
  std::vector<Point2D> observed_obstacle_centers(waypoints_.size(), Point2D());
  std::vector<double> observed_forbidden_radii(waypoints_.size(), 0.0);
  std::vector<bool> observed_geometry_metadata_valid(waypoints_.size(), false);
  if (route_deformation_mode_ == "circle_envelope") {
    for (const auto & segment : candidate_segments) {
      if (segment.preferred_direction == 0) {
        continue;
      }
      const double required_offset = segment.preferred_direction > 0 ?
        segment.left_required_offset : segment.right_required_offset;
      if (required_offset <= 1.0e-9) {
        continue;
      }

      // 通常每个障碍物都有不同锚点。极少数锚点重合时，只借用其影响窗内
      // 最近的空元数据槽；source仍保存真实投影锚点，不合并障碍物身份。
      std::size_t metadata_index = segment.obstacle_waypoint;
      if (observed_geometry_metadata_valid[metadata_index]) {
        bool found_slot = false;
        const std::size_t maximum_distance = std::max(
          metadata_index - segment.first, segment.last - metadata_index);
        for (std::size_t distance = 1; distance <= maximum_distance; ++distance) {
          if (metadata_index >= segment.first + distance &&
            !observed_geometry_metadata_valid[metadata_index - distance])
          {
            metadata_index -= distance;
            found_slot = true;
            break;
          }
          if (metadata_index + distance <= segment.last &&
            !observed_geometry_metadata_valid[metadata_index + distance])
          {
            metadata_index += distance;
            found_slot = true;
            break;
          }
        }
        if (!found_slot) {
          if (!logic3_ready_preplanning_active_) {
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 2000,
              "circle_envelope独立锚点槽不足：障碍(%.3f, %.3f)锚点=%zu；"
              "本轮保留已有锚点，不把两个障碍物合并。",
              segment.obstacle_center.x, segment.obstacle_center.y,
              segment.obstacle_waypoint + 1);
          }
          continue;
        }
      }

      observed_offsets[metadata_index] =
        static_cast<double>(segment.preferred_direction) * required_offset;
      observed_offsets_valid[metadata_index] = true;
      observed_world_directions[metadata_index] =
        segment.preferred_direction > 0 ?
        segment.left_normal : Point2D{-segment.left_normal.x, -segment.left_normal.y};
      observed_source_indices[metadata_index] = segment.obstacle_waypoint;
      observed_obstacle_centers[metadata_index] = segment.obstacle_center;
      observed_forbidden_radii[metadata_index] = segment.forbidden_radius;
      observed_geometry_metadata_valid[metadata_index] = true;
      if (debug_control_logs_ && !logic3_ready_preplanning_active_) {
        RCLCPP_INFO(
          get_logger(),
          "debug: circle_envelope独立锚点：障碍(%.2f, %.2f)，锚点=%zu，"
          "元数据槽=%zu，向%s偏移%.3fm。",
          segment.obstacle_center.x, segment.obstacle_center.y,
          segment.obstacle_waypoint + 1, metadata_index + 1,
          segment.preferred_direction > 0 ? "左" : "右", required_offset);
      }
    }
  } else {
    const ShiftCandidateSegment * previous_active_segment = nullptr;
    for (const auto & segment : candidate_segments) {
      if (segment.preferred_direction == 0) {
        continue;
      }
      const double required_offset = segment.preferred_direction > 0 ?
        segment.left_required_offset : segment.right_required_offset;
      if (required_offset <= 1e-9) {
        continue;
      }
      const double candidate =
        static_cast<double>(segment.preferred_direction) * required_offset;
      std::size_t applied_first = segment.first;
      std::size_t overlap_count = 0;
      if (previous_active_segment) {
        const std::size_t overlap_first = std::max(
          previous_active_segment->first, segment.first);
        const std::size_t overlap_last = std::min(
          previous_active_segment->last, segment.last);
        if (overlap_first <= overlap_last) {
          overlap_count = overlap_last - overlap_first + 1;
          // 整数除法将不能二等分的余数交给后一个障碍物。
          applied_first = overlap_first + overlap_count / 2;
        }
      }
      std::size_t overwritten_count = 0;
      for (std::size_t index = applied_first; index <= segment.last; ++index) {
        if (observed_offsets_valid[index]) {
          ++overwritten_count;
        }
        // 候选已按路线进度排序；后一个障碍物从重叠区1/2位置开始取得控制权。
        observed_offsets[index] = candidate;
        observed_offsets_valid[index] = true;
        observed_world_directions[index] = segment.preferred_direction > 0 ?
          segment.left_normal : Point2D{-segment.left_normal.x, -segment.left_normal.y};
        observed_source_indices[index] = segment.obstacle_waypoint;
        observed_obstacle_centers[index] = segment.obstacle_center;
        observed_forbidden_radii[index] = segment.forbidden_radius;
        observed_geometry_metadata_valid[index] = true;
      }
      previous_active_segment = &segment;
      if (debug_control_logs_ && !logic3_ready_preplanning_active_) {
        RCLCPP_INFO(
          get_logger(),
          "debug: 静态+动态方案二分之一覆盖：障碍点=%zu，原窗口=%zu-%zu，实际写入=%zu-%zu，"
          "向%s偏移%.3fm，重叠=%zu个，覆盖前障碍=%zu个。",
          segment.obstacle_waypoint + 1, segment.first + 1, segment.last + 1,
          applied_first + 1, segment.last + 1,
          segment.preferred_direction > 0 ? "左" : "右", required_offset,
          overlap_count, overwritten_count);
      }
    }
  }

  // 延长路线的第0点就是本次规划瞬间的base_link位置A。静态障碍可以
  // 改造其后的连接段，但A必须固定，不能被首个绿色簇横移。
  if (channel_entry_waypoint_index_ > 0 && !observed_offsets_valid.empty()) {
    observed_offsets_valid.front() = false;
    observed_geometry_metadata_valid.front() = false;
    observed_offsets.front() = 0.0;
    observed_obstacle_centers.front() = Point2D();
    observed_forbidden_radii.front() = 0.0;
  }
  bool route_changed = false;
  auto observed_geometry_changed = [&](std::size_t index) {
      if (!observed_geometry_metadata_valid[index]) {
        return false;
      }
      if (waypoint_shift_source_indices_[index] >= waypoints_.size()) {
        return true;
      }
      const std::size_t source_delta = waypoint_shift_source_indices_[index] >
        observed_source_indices[index] ?
        waypoint_shift_source_indices_[index] - observed_source_indices[index] :
        observed_source_indices[index] - waypoint_shift_source_indices_[index];
      const auto & current_direction = waypoint_shift_world_directions_[index];
      const auto & observed_direction = observed_world_directions[index];
      const double direction_alignment =
        current_direction.x * observed_direction.x +
        current_direction.y * observed_direction.y;
      const double center_change = std::hypot(
        waypoint_shift_obstacle_centers_[index].x -
        observed_obstacle_centers[index].x,
        waypoint_shift_obstacle_centers_[index].y -
        observed_obstacle_centers[index].y);
      const double radius_change = std::abs(
        waypoint_shift_forbidden_radii_[index] -
        observed_forbidden_radii[index]);
      // 0.02m航点下允许来源点在2点内轻微抖动；方向变化约2.6度以上才重建。
      const double geometry_change_threshold = std::max(
        0.005, 0.5 * waypoint_shift_min_change_);
      return source_delta > 2 || direction_alignment < 0.999 ||
             center_change >= geometry_change_threshold ||
             radius_change >= geometry_change_threshold;
    };

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

      const int required_confirm_scans = logic3_force_confirm_ ?
        1 : waypoint_shift_confirm_scans_;
      if (waypoint_shift_pending_counts_[index] >= required_confirm_scans) {
        const double confirmed_offset = waypoint_shift_pending_offsets_[index];
        if (std::abs(confirmed_offset - waypoint_shift_offsets_[index]) >=
          waypoint_shift_min_change_)
        {
          waypoint_shift_offsets_[index] = confirmed_offset;
          if (observed_geometry_metadata_valid[index]) {
            waypoint_shift_world_directions_[index] = observed_world_directions[index];
            waypoint_shift_source_indices_[index] = observed_source_indices[index];
            waypoint_shift_obstacle_centers_[index] = observed_obstacle_centers[index];
            waypoint_shift_forbidden_radii_[index] = observed_forbidden_radii[index];
          }
          route_changed = true;
          ++confirmed_count;
        } else if (std::abs(waypoint_shift_offsets_[index]) > 1e-9 &&
          observed_geometry_changed(index))
        {
          waypoint_shift_world_directions_[index] = observed_world_directions[index];
          waypoint_shift_source_indices_[index] = observed_source_indices[index];
          waypoint_shift_obstacle_centers_[index] = observed_obstacle_centers[index];
          waypoint_shift_forbidden_radii_[index] = observed_forbidden_radii[index];
          route_changed = true;
        }
      }
      continue;
    }

    waypoint_shift_pending_offsets_[index] = 0.0;
    waypoint_shift_pending_counts_[index] = 0;
    ++waypoint_shift_missing_counts_[index];
    const double base_offset = logic3_planning_pose_anchored_ &&
      logic3_frozen_base_offsets_.size() == waypoint_shift_offsets_.size() ?
      logic3_frozen_base_offsets_[index] : 0.0;
    if (waypoint_shift_missing_counts_[index] >= waypoint_shift_release_scans_) {
      waypoint_shift_dynamic_directions_[index] = 0;
      Point2D base_direction;
      std::size_t base_source = waypoints_.size();
      Point2D base_center;
      double base_radius = 0.0;
      if (std::abs(base_offset) > 1e-9 &&
        logic3_frozen_base_shift_directions_.size() == waypoints_.size() &&
        logic3_frozen_base_source_indices_.size() == waypoints_.size() &&
        logic3_frozen_base_obstacle_centers_.size() == waypoints_.size() &&
        logic3_frozen_base_forbidden_radii_.size() == waypoints_.size())
      {
        base_direction = logic3_frozen_base_shift_directions_[index];
        base_source = logic3_frozen_base_source_indices_[index];
        base_center = logic3_frozen_base_obstacle_centers_[index];
        base_radius = logic3_frozen_base_forbidden_radii_[index];
      }
      const bool release_changed =
        std::abs(waypoint_shift_offsets_[index] - base_offset) > 1e-9 ||
        waypoint_shift_source_indices_[index] != base_source ||
        std::hypot(
          waypoint_shift_obstacle_centers_[index].x - base_center.x,
          waypoint_shift_obstacle_centers_[index].y - base_center.y) > 1e-9 ||
        std::abs(waypoint_shift_forbidden_radii_[index] - base_radius) > 1e-9;
      if (release_changed) {
        waypoint_shift_offsets_[index] = base_offset;
        waypoint_shift_world_directions_[index] = base_direction;
        waypoint_shift_source_indices_[index] = base_source;
        waypoint_shift_obstacle_centers_[index] = base_center;
        waypoint_shift_forbidden_radii_[index] = base_radius;
        route_changed = true;
        ++released_count;
      }
    }
  }

  if (!logic3_ready_preplanning_active_ &&
    (confirmed_count > 0 || released_count > 0))
  {
    RCLCPP_WARN(
      get_logger(),
      "waypoint_shift：确认刷新%zu个点，释放%zu个点（确认帧=%d，释放帧=%d）。",
      confirmed_count, released_count, waypoint_shift_confirm_scans_,
      waypoint_shift_release_scans_);
  }

  if (route_changed) {
    rebuild_route_from_confirmed_shifts();
    if (!logic3_ready_preplanning_active_) {
      publish_debug_route();
    }
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
          w.task3_start = yaml_bool(item["task3_start"], false);
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
