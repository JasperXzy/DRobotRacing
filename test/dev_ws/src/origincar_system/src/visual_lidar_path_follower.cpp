#include "origincar_system/visual_lidar_path_follower.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <ctime>
#include <exception>
#include <functional>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <thread>
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
  std::string task1_route_variant{"default"};
  std::string task1_waypoints_backup_file;
  declare_parameter<std::string>("task1_route_variant", task1_route_variant);
  get_parameter("task1_route_variant", task1_route_variant);
  declare_parameter<std::string>(
    "task1_waypoints_backup_file", task1_waypoints_backup_file);
  get_parameter("task1_waypoints_backup_file", task1_waypoints_backup_file);
  task1_route_variant = to_lower(task1_route_variant);
  if (task1_route_variant == "backup" || task1_route_variant == "backup_45deg") {
    task1_route_variant = "backup_45deg";
    if (task1_waypoints_backup_file.empty()) {
      RCLCPP_ERROR(
        get_logger(), "Task1选择了备用45°路线，但task1_waypoints_backup_file为空。");
    } else {
      task1_waypoints_file_ = task1_waypoints_backup_file;
    }
  } else if (task1_route_variant == "default" || task1_route_variant == "original") {
    task1_route_variant = "default";
  } else {
    RCLCPP_WARN(
      get_logger(), "未知Task1路线选项“%s”，回退到default。可选：default、backup_45deg。",
      task1_route_variant.c_str());
    task1_route_variant = "default";
  }
  RCLCPP_INFO(
    get_logger(), "Task1路线选择：%s（%s）。",
    task1_route_variant.c_str(), task1_waypoints_file_.c_str());
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
  declare_parameter<bool>("debug_control_logs", debug_control_logs_);
  get_parameter("debug_control_logs", debug_control_logs_);
  declare_parameter<bool>("enable_debug_visualization", enable_debug_visualization_);
  get_parameter("enable_debug_visualization", enable_debug_visualization_);

  // 全阶段公共控制
  declare_parameter<double>("control_frequency", control_frequency_);
  get_parameter("control_frequency", control_frequency_);
  declare_parameter<double>(
    "turn_speed_full_steering_angle", turn_speed_full_steering_angle_);
  get_parameter(
    "turn_speed_full_steering_angle", turn_speed_full_steering_angle_);

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
  declare_parameter<bool>("enable_static_avoidance", enable_static_avoidance_);
  declare_parameter<bool>("enable_lidar_avoidance", enable_lidar_avoidance_);
  get_parameter("enable_static_map_filter", enable_static_map_filter_);
  get_parameter("enable_static_avoidance", enable_static_avoidance_);
  get_parameter("enable_lidar_avoidance", enable_lidar_avoidance_);
  RCLCPP_INFO(
    get_logger(), "避障开关：静态避障=%s，实时雷达避障=%s。",
    enable_static_avoidance_ ? "开启" : "关闭",
    enable_lidar_avoidance_ ? "开启" : "关闭");
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
  declare_parameter<double>("logic3_runtime_roi_width", logic3_runtime_roi_width_);
  get_parameter("logic3_runtime_roi_width", logic3_runtime_roi_width_);
  declare_parameter<double>("logic3_runtime_roi_height", logic3_runtime_roi_height_);
  get_parameter("logic3_runtime_roi_height", logic3_runtime_roi_height_);
  declare_parameter<double>(
    "logic3_runtime_channel_min_y", logic3_runtime_channel_min_y_);
  get_parameter("logic3_runtime_channel_min_y", logic3_runtime_channel_min_y_);
  declare_parameter<double>(
    "logic3_runtime_cluster_match_radius", logic3_runtime_cluster_match_radius_);
  get_parameter(
    "logic3_runtime_cluster_match_radius", logic3_runtime_cluster_match_radius_);
  declare_parameter<int>(
    "logic3_runtime_average_samples", logic3_runtime_average_samples_);
  get_parameter("logic3_runtime_average_samples", logic3_runtime_average_samples_);
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
    "logic3_static_average_samples", logic3_static_average_samples_);
  get_parameter("logic3_static_average_samples", logic3_static_average_samples_);
  declare_parameter<double>(
    "logic3_static_loss_grace_seconds", logic3_static_loss_grace_seconds_);
  get_parameter(
    "logic3_static_loss_grace_seconds", logic3_static_loss_grace_seconds_);
  declare_parameter<double>(
    "logic3_static_reacquire_radius", logic3_static_reacquire_radius_);
  get_parameter(
    "logic3_static_reacquire_radius", logic3_static_reacquire_radius_);
  declare_parameter<double>(
    "logic3_red_history_merge_radius", logic3_red_history_merge_radius_);
  get_parameter("logic3_red_history_merge_radius", logic3_red_history_merge_radius_);
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
  declare_parameter<double>("qr_reverse_x_limit", qr_reverse_x_limit_);
  get_parameter("qr_reverse_x_limit", qr_reverse_x_limit_);
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
    "qr_reverse_rear_obstacle_distance", qr_reverse_rear_obstacle_distance_);
  get_parameter(
    "qr_reverse_rear_obstacle_distance", qr_reverse_rear_obstacle_distance_);
  declare_parameter<double>("qr_reverse_rear_fov_deg", qr_reverse_rear_fov_deg_);
  get_parameter("qr_reverse_rear_fov_deg", qr_reverse_rear_fov_deg_);
  declare_parameter<double>(
    "qr_reverse_rear_corridor_half_width", qr_reverse_rear_corridor_half_width_);
  get_parameter(
    "qr_reverse_rear_corridor_half_width", qr_reverse_rear_corridor_half_width_);
  declare_parameter<double>("qr_reverse_linear_speed", qr_reverse_linear_speed_);
  get_parameter("qr_reverse_linear_speed", qr_reverse_linear_speed_);
  declare_parameter<double>("qr_reverse_angular_speed", qr_reverse_angular_speed_);
  get_parameter("qr_reverse_angular_speed", qr_reverse_angular_speed_);
  declare_parameter<int>("qr_reverse_max_count", qr_reverse_max_count_);
  get_parameter("qr_reverse_max_count", qr_reverse_max_count_);

  // A→B进场结束判定与入口基准路线圆滑。
  declare_parameter<double>(
    "channel_handoff_waypoint_distance", channel_handoff_waypoint_distance_);
  get_parameter(
    "channel_handoff_waypoint_distance", channel_handoff_waypoint_distance_);
  declare_parameter<double>(
    "channel_entry_corner_smoothing_length",
    channel_entry_corner_smoothing_length_);
  get_parameter(
    "channel_entry_corner_smoothing_length",
    channel_entry_corner_smoothing_length_);

  // 四个任务阶段分别持有完整的追点和实时雷达避障参数，不再读取旧的
  // 全局追点、channel避障或兼容返程速度作为默认入口。
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
      declare_double("lookahead_distance", profile.lookahead_distance);
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
  declare_parameter<double>("return_stop_zone_max_x", return_stop_zone_max_x_);
  get_parameter("return_stop_zone_max_x", return_stop_zone_max_x_);
  declare_parameter<double>("return_stop_zone_min_y", return_stop_zone_min_y_);
  get_parameter("return_stop_zone_min_y", return_stop_zone_min_y_);
  declare_parameter<double>("return_stop_zone_max_y", return_stop_zone_max_y_);
  get_parameter("return_stop_zone_max_y", return_stop_zone_max_y_);
  declare_parameter<double>(
    "return_stop_confirm_seconds", return_stop_confirm_seconds_);
  get_parameter("return_stop_confirm_seconds", return_stop_confirm_seconds_);
  declare_parameter<double>(
    "return_brake_reverse_speed", return_brake_reverse_speed_);
  get_parameter("return_brake_reverse_speed", return_brake_reverse_speed_);
  declare_parameter<int>(
    "return_brake_reverse_cycles", return_brake_reverse_cycles_);
  get_parameter("return_brake_reverse_cycles", return_brake_reverse_cycles_);
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
  declare_parameter<double>("image_sign_clockwise_x", image_sign_clockwise_x_);
  get_parameter("image_sign_clockwise_x", image_sign_clockwise_x_);
  declare_parameter<double>("image_sign_clockwise_y", image_sign_clockwise_y_);
  get_parameter("image_sign_clockwise_y", image_sign_clockwise_y_);
  declare_parameter<double>(
    "image_sign_counterclockwise_x", image_sign_counterclockwise_x_);
  get_parameter("image_sign_counterclockwise_x", image_sign_counterclockwise_x_);
  declare_parameter<double>(
    "image_sign_counterclockwise_y", image_sign_counterclockwise_y_);
  get_parameter("image_sign_counterclockwise_y", image_sign_counterclockwise_y_);
  declare_parameter<double>(
    "image_capture_zone_min_distance", image_capture_zone_min_distance_);
  get_parameter("image_capture_zone_min_distance", image_capture_zone_min_distance_);
  declare_parameter<double>(
    "image_capture_zone_max_distance", image_capture_zone_max_distance_);
  get_parameter("image_capture_zone_max_distance", image_capture_zone_max_distance_);
  declare_parameter<double>(
    "image_capture_zone_half_angle_deg", image_capture_zone_half_angle_deg_);
  get_parameter("image_capture_zone_half_angle_deg", image_capture_zone_half_angle_deg_);
  declare_parameter<double>(
    "image_capture_heading_tolerance_deg", image_capture_heading_tolerance_deg_);
  get_parameter(
    "image_capture_heading_tolerance_deg", image_capture_heading_tolerance_deg_);
  declare_parameter<double>(
    "image_capture_approach_distance", image_capture_approach_distance_);
  get_parameter("image_capture_approach_distance", image_capture_approach_distance_);
  declare_parameter<double>("image_capture_slow_speed", image_capture_slow_speed_);
  get_parameter("image_capture_slow_speed", image_capture_slow_speed_);
  declare_parameter<double>("image_capture_angular_gain", image_capture_angular_gain_);
  get_parameter("image_capture_angular_gain", image_capture_angular_gain_);
  declare_parameter<double>("image_capture_stop_time", image_capture_stop_time_);
  get_parameter("image_capture_stop_time", image_capture_stop_time_);
  declare_parameter<bool>("image_capture_recovery_enabled", image_capture_recovery_enabled_);
  get_parameter("image_capture_recovery_enabled", image_capture_recovery_enabled_);
  declare_parameter<double>(
    "image_capture_recovery_reverse_distance", image_capture_recovery_reverse_distance_);
  get_parameter(
    "image_capture_recovery_reverse_distance", image_capture_recovery_reverse_distance_);
  declare_parameter<double>(
    "image_capture_recovery_reverse_speed", image_capture_recovery_reverse_speed_);
  get_parameter(
    "image_capture_recovery_reverse_speed", image_capture_recovery_reverse_speed_);
  declare_parameter<double>(
    "image_capture_recovery_heading_gain", image_capture_recovery_heading_gain_);
  get_parameter(
    "image_capture_recovery_heading_gain", image_capture_recovery_heading_gain_);
  declare_parameter<double>(
    "image_capture_recovery_max_angular_speed", image_capture_recovery_max_angular_speed_);
  get_parameter(
    "image_capture_recovery_max_angular_speed", image_capture_recovery_max_angular_speed_);
  declare_parameter<double>(
    "image_capture_recovery_rear_obstacle_distance",
    image_capture_recovery_rear_obstacle_distance_);
  get_parameter(
    "image_capture_recovery_rear_obstacle_distance",
    image_capture_recovery_rear_obstacle_distance_);
  declare_parameter<double>(
    "image_capture_recovery_rear_fov_deg", image_capture_recovery_rear_fov_deg_);
  get_parameter(
    "image_capture_recovery_rear_fov_deg", image_capture_recovery_rear_fov_deg_);
  declare_parameter<double>(
    "image_capture_recovery_rear_corridor_half_width",
    image_capture_recovery_rear_corridor_half_width_);
  get_parameter(
    "image_capture_recovery_rear_corridor_half_width",
    image_capture_recovery_rear_corridor_half_width_);
  declare_parameter<int>("image_capture_trigger_value", image_capture_trigger_value_);
  get_parameter("image_capture_trigger_value", image_capture_trigger_value_);
  declare_parameter<std::string>("image_analysis_trigger_topic", image_analysis_trigger_topic_);
  get_parameter("image_analysis_trigger_topic", image_analysis_trigger_topic_);

  route_direction_ = to_lower(route_direction_);
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
    task1_waypoint_shift_min_y_ = -0.10;
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
  logic3_motion_speed_threshold_ = std::max(0.001, logic3_motion_speed_threshold_);
  logic3_motion_displacement_threshold_ = std::max(
    0.001, logic3_motion_displacement_threshold_);
  logic3_motion_confirm_messages_ = std::max(1, logic3_motion_confirm_messages_);
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
  logic3_runtime_roi_width_ = std::max(0.10, logic3_runtime_roi_width_);
  logic3_runtime_roi_height_ = std::max(0.10, logic3_runtime_roi_height_);
  if (!std::isfinite(logic3_runtime_channel_min_y_)) {
    logic3_runtime_channel_min_y_ = -0.20;
  }
  // 场内障碍物中心间距至少1m；运行身份半径限制在0.49m以内，避免两个
  // 实体被关联成同一轨迹。0.75m红圈仍只用于最终图片的历史归并。
  logic3_runtime_cluster_match_radius_ = std::clamp(
    logic3_runtime_cluster_match_radius_, 0.05, 0.49);
  logic3_runtime_average_samples_ = std::max(1, logic3_runtime_average_samples_);
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
    task1_static_min_x_ = 0.0;
  }
  if (!std::isfinite(task1_static_min_y_)) {
    task1_static_min_y_ = 0.0;
  }
  if (!std::isfinite(task1_static_avoid_min_x_)) {
    task1_static_avoid_min_x_ = 0.0;
  }
  if (!std::isfinite(task1_static_avoid_max_x_)) {
    task1_static_avoid_max_x_ = 0.60;
  }
  if (!std::isfinite(task1_static_avoid_min_y_)) {
    task1_static_avoid_min_y_ = 0.0;
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
    task1_transition_min_x_ = 1.80;
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
  logic3_static_average_samples_ = std::max(
    1, logic3_static_average_samples_);
  logic3_static_loss_grace_seconds_ = std::max(
    0.01, logic3_static_loss_grace_seconds_);
  logic3_static_reacquire_radius_ = std::clamp(
    logic3_static_reacquire_radius_, 0.01,
    logic3_static_cluster_match_radius_);
  logic3_red_history_merge_radius_ = std::max(
    0.01, logic3_red_history_merge_radius_);
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
    qr_reverse_x_limit_ = 4.30;
  }
  qr_reverse_front_obstacle_distance_ = std::max(
    0.0, qr_reverse_front_obstacle_distance_);
  qr_reverse_front_fov_deg_ = std::clamp(qr_reverse_front_fov_deg_, 1.0, 360.0);
  qr_reverse_front_corridor_half_width_ = std::max(
    0.01, qr_reverse_front_corridor_half_width_);
  qr_reverse_rear_obstacle_distance_ = std::max(
    0.0, qr_reverse_rear_obstacle_distance_);
  qr_reverse_rear_fov_deg_ = std::clamp(qr_reverse_rear_fov_deg_, 1.0, 360.0);
  qr_reverse_rear_corridor_half_width_ = std::max(
    0.01, qr_reverse_rear_corridor_half_width_);
  qr_reverse_linear_speed_ = std::max(0.0, qr_reverse_linear_speed_);
  qr_reverse_max_count_ = std::max(0, qr_reverse_max_count_);
  channel_handoff_waypoint_distance_ = std::max(
    0.01, channel_handoff_waypoint_distance_);
  channel_entry_corner_smoothing_length_ = std::max(
    0.0, channel_entry_corner_smoothing_length_);
  auto normalize_stage_profile =
    [](StageControlProfile & profile) {
      profile.linear_speed = std::max(0.0, profile.linear_speed);
      profile.lookahead_distance = std::max(
        0.05, profile.lookahead_distance);
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
  // 紫色停车边界直接采用YAML参数；Task3虚拟末点自动取紫色矩形中心。
  return_target_x_ = 0.5 * (return_stop_zone_min_x_ + return_stop_zone_max_x_);
  return_target_y_ = 0.5 * (return_stop_zone_min_y_ + return_stop_zone_max_y_);
  return_stop_confirm_seconds_ = std::max(0.0, return_stop_confirm_seconds_);
  return_brake_reverse_speed_ = std::max(0.0, return_brake_reverse_speed_);
  return_brake_reverse_cycles_ = std::max(0, return_brake_reverse_cycles_);
  return_brake_zero_cycles_ = std::max(0, return_brake_zero_cycles_);
  if (!std::isfinite(return_obstacle_split_y_)) {
    return_obstacle_split_y_ = -0.20;
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
  image_capture_slow_speed_ = std::max(0.0, image_capture_slow_speed_);
  image_capture_angular_gain_ = std::max(0.0, image_capture_angular_gain_);
  image_capture_stop_time_ = std::max(0.0, image_capture_stop_time_);
  image_capture_zone_min_distance_ = std::max(0.01, image_capture_zone_min_distance_);
  image_capture_zone_max_distance_ = std::max(
    image_capture_zone_min_distance_, image_capture_zone_max_distance_);
  image_capture_zone_half_angle_deg_ = std::clamp(
    image_capture_zone_half_angle_deg_, 1.0, 89.0);
  image_capture_heading_tolerance_deg_ = std::clamp(
    image_capture_heading_tolerance_deg_, 1.0, 89.0);
  image_capture_approach_distance_ = std::max(
    image_capture_zone_max_distance_, image_capture_approach_distance_);
  image_capture_recovery_reverse_distance_ = std::max(
    0.01, image_capture_recovery_reverse_distance_);
  image_capture_recovery_reverse_speed_ = std::max(
    0.0, image_capture_recovery_reverse_speed_);
  image_capture_recovery_heading_gain_ = std::max(
    0.0, image_capture_recovery_heading_gain_);
  image_capture_recovery_max_angular_speed_ = std::max(
    0.01, std::abs(image_capture_recovery_max_angular_speed_));
  image_capture_recovery_rear_obstacle_distance_ = std::max(
    0.0, image_capture_recovery_rear_obstacle_distance_);
  image_capture_recovery_rear_fov_deg_ = std::clamp(
    image_capture_recovery_rear_fov_deg_, 1.0, 360.0);
  image_capture_recovery_rear_corridor_half_width_ = std::max(
    0.01, image_capture_recovery_rear_corridor_half_width_);
  if (!enable_qr_direction_select_) {
    RCLCPP_WARN(get_logger(), "四阶段流程需要 QR 选路，已自动打开 QR 订阅。");
    enable_qr_direction_select_ = true;
  }

  // 启动时始终预载 Task1 路线。物理底盘开关关闭期间节点会持续完成
  // 静态绿色候选确认；开关打开后同一条路线立即可用，不需要等待二维码。
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
  manual_route_freeze_srv_ = create_service<std_srvs::srv::SetBool>(
    "/visual_lidar_path_follower/manual_route_freeze",
    std::bind(
      &VisualLidarPathFollower::manual_route_freeze_callback, this,
      std::placeholders::_1, std::placeholders::_2));

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
    get_logger(), "VisualLidarPathFollower 初始化完成：%zu 个航点，路线=%s。",
    waypoints_.size(), route_name.c_str());
  if (debug_control_logs_) {
    RCLCPP_INFO(
      get_logger(),
      "debug: 倒车点=%zu，Task1=%s，顺时针=%s，逆时针=%s，控制前瞻=%.2f，"
      "避障=全阶段静态+动态雷达控制，线速度=%.2f。",
      static_cast<std::size_t>(reverse_count), task1_waypoints_file_.c_str(),
      route_waypoints_clockwise_file_.c_str(), route_waypoints_counterclockwise_file_.c_str(),
      lookahead_distance_, linear_speed_);
  }
  publish_debug_route();

  // 定时控制循环：每次读取 TF、更新路线进度并计算离散前瞻 waypoint 速度。
  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / std::max(control_frequency_, 1.0)),
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
  lookahead_distance_ = profile.lookahead_distance;
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
  task_phase_ = phase;
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
  // 输入重新有效后会清除此标志；阶段切换也视为进入一轮新的等待状态，
  // 因而允许新阶段最多再报告一次对应的“等待输入”信息。
  qr_reverse_waiting_for_inputs_ = false;
  channel_lidar_waiting_for_inputs_ = false;
  channel_lidar_obstacle_active_ = false;
  channel_lidar_logged_direction_ = 0;
  RCLCPP_INFO(
    get_logger(), "当前任务状态：%s。", task_phase_name(task_phase_));
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
    publish_stop();
    return;
  }

  record_trajectory_sample(pose);

  if (task_phase_ == TaskPhase::TASK1 && logic3_planning_pose_anchored_) {
    const auto steady_now = std::chrono::steady_clock::now();
    if (!task1_pending_direction_.empty()) {
      if (pose.x > task1_transition_min_x_) {
        const std::string pending_direction = task1_pending_direction_;
        const std::string pending_reason = task1_pending_reason_;
        const bool pending_from_qr = task1_pending_from_qr_;
        if (complete_task1_and_select_route(
            pending_direction, pending_reason.c_str(), pending_from_qr))
        {
          // 本周期直接沿新建过渡路线继续，避免人为插入停车命令。
        }
      }
    } else if (task1_endpoint_waiting_) {
      const double wait_seconds =
        std::chrono::duration<double>(
        steady_now - task1_endpoint_wait_started_at_).count();
      if (wait_seconds < task1_endpoint_qr_wait_seconds_) {
        set_control_state(ControlState::WAITING_FOR_QR);
        publish_stop();
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
          "clockwise", "Task1终点等待0.3秒仍未识别到有效二维码", false))
      {
        // 本周期继续执行下方重建，用同一份最新pose无停车衔接过渡路线。
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
          // 本周期继续执行下方重建，避免沿用上一周期速度却没有新指令。
        }
      }
    }
  }

  if (return_to_start_active_) {
    control_return_to_start(pose);
    return;
  }

  next_control_state_ = ControlState::PATH_FOLLOWING;

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
      publish_stop();
      return false;
    };

  // QR回调没有可靠的同步位姿；切线只记录待办，在控制周期拿到当前
  // map->base_link 后才把A点和正式channel首点连接起来。
  if (!rebuild_pending_channel_entry()) {
    return;
  }
  if (rebuilt_after_completed_reverse) {
    publish_debug_visualization(pose);
    set_control_state(next_control_state_);
    publish_stop();
    return;
  }

  if (qr_channel_approach_active_) {
    geometry_msgs::msg::Twist reverse_cmd;
    if (apply_qr_reverse_maneuver(pose, reverse_cmd)) {
      publish_debug_visualization(pose);
      set_control_state(next_control_state_);
      cmd_pub_->publish(reverse_cmd);
      return;
    }
    // 每轮倒车满足“前方清空且map.y回到阈值内”后会再次置位待办。
    // 必须在同一周期用最新base_link重建A点；倒车机制本身仍保持武装。
    if (!rebuild_pending_channel_entry()) {
      return;
    }
    if (rebuilt_after_completed_reverse) {
      // 倒车结束的这一周期先完成重规划并明确下发零速；下一控制周期
      // 才沿新路线前进，避免倒车指令与正向指令在同一周期交接。
      publish_debug_visualization(pose);
      set_control_state(next_control_state_);
      publish_stop();
      return;
    }
    update_qr_channel_approach_phase(pose);
  }

  // 图生文拍照区首次以不理想朝向进入时，允许一次安全的低速倒车复位；
  // 倒车过程优先于普通追点和前向雷达绕障，且会先检查后方雷达安全距离。
  geometry_msgs::msg::Twist image_capture_recovery_cmd;
  if (apply_image_capture_recovery(pose, image_capture_recovery_cmd)) {
    publish_debug_visualization(pose);
    set_control_state(next_control_state_);
    cmd_pub_->publish(image_capture_recovery_cmd);
    return;
  }

  // 独立拍照区是不可跳过的精确任务边界。进入扇形区后，只有朝向立牌且稳定才触发。
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

  // 生成路线里的返程直线仍属于channel。这里只在进入该段后
  // 打开停车标志识别并执行停车判定，不切换追点或避障控制器。
  if (handle_task3_start(pose)) {
    return;
  }

  if (stop_if_route_finished(pose)) {
    return;
  }

  auto target = compute_lookahead_target(pose, lookahead_distance_);
  if (!target.reverse) {
    if (!logic3_planning_pose_anchored_) {
      logic3_force_full_route_ = true;
    }
    update_waypoint_shifts(pose, target.waypoint_index);
    logic3_force_full_route_ = false;
    target = compute_lookahead_target(pose, lookahead_distance_);
  }
  commit_channel_lookahead_progress(target);

  auto cmd = compute_tracking_command(pose, target);
  apply_turn_speed_limit(cmd);
  apply_image_capture_speed_limit(pose, cmd);
  const bool runtime_lidar_active = logic3_planning_pose_anchored_ &&
    task_phase_ != TaskPhase::FINISHED;
  if (runtime_lidar_active && !target.reverse && enable_lidar_avoidance_) {
    apply_channel_lidar_obstacle_avoidance(cmd);
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
      task1_endpoint_wait_started_at_ = std::chrono::steady_clock::now();
      RCLCPP_INFO(
        get_logger(),
        "Task1已到达最后一个waypoint：停车等待二维码%.2f秒。",
        task1_endpoint_qr_wait_seconds_);
    }
    qr_path_finished_ = true;
    set_control_state(ControlState::WAITING_FOR_QR);
    publish_stop();
    return true;
  }

  // 新版channel YAML已把返程直线放在同一条路线里。到达最后一个
  // waypoint不能再切换到旧的直指目标点+视觉避障控制；若因
  // pass_radius提前提交了末点，仍继续追末点直到停车条件成立。
  if (return_to_start_enabled_ && task3_route_available_) {
    return false;
  }

  if (return_to_start_enabled_) {
    start_return_to_start();
    return true;
  }

  set_control_state(ControlState::FINISHED);
  if (logic3_planning_pose_anchored_ &&
    !logic3_final_snapshot_published_)
  {
    logic3_final_pose_ = pose;
    logic3_final_pose_valid_ = std::isfinite(pose.x) &&
      std::isfinite(pose.y) && std::isfinite(pose.yaw);
    publish_logic3_snapshot("final_dynamic");
    logic3_final_snapshot_published_ = true;
  }
  if (resume_visual_control_on_finish_) {
    RCLCPP_INFO(get_logger(), "环线追点完成，恢复 visual_control 接管停车。");
    resume_visual_control(true);
    publish_stop_on_exit_ = false;
  } else {
    RCLCPP_INFO(get_logger(), "路线追点完成，停车。");
    publish_stop();
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
    // 过早停车并恢复YOLO。
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
      "实时雷达避障，虚拟牵引点=(%.2f,%.2f)，base_link进入有限紫色"
      "停车矩形后立即停车。障碍簇中心Y>%.2f走完整静态偏移，"
      "Y<=%.2f走实时雷达。",
      task3_start_index_ + 1, waypoints_.size(),
      return_target_x_, return_target_y_,
      return_obstacle_split_y_, return_obstacle_split_y_);
    return false;
  }

  // Task3停车只看map定位；无论当前是跟线还是实时绕障，都独立检查
  // 有限紫色矩形，base_link一进入便在当前控制周期停车。
  if (return_stop_zone_confirmed(pose)) {
    finish_return_to_start(pose, "base_link中心进入有限紫色停车矩形");
    return true;
  }
  return false;
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

  // QR成功后YOLO曾被暂停。这里只恢复完整检测，使本节点能收到锥桶和
  // parking_sign；不发送visual_control_resume_value，视觉控制节点仍保持挂起。
  set_perception_mode(perception_full_resume_value_, "完整识别（返回起点）");
  publish_stop();
  RCLCPP_INFO(
    get_logger(),
    "channel巡点完成，开始返回虚拟牵引点(%.2f, %.2f)；"
    "base_link进入有限紫色停车矩形后立即停车。",
    return_target_x_, return_target_y_);
}

void VisualLidarPathFollower::control_return_to_start(
  const VisualLidarPathFollower::RobotPose & pose)
{
  set_control_state(ControlState::RETURNING_TO_START);

  // 有限停车矩形是唯一停车判据，且不能被同帧避障覆盖。
  if (return_stop_zone_confirmed(pose)) {
    finish_return_to_start(pose, "base_link中心进入有限紫色停车矩形");
    return;
  }

  // 没有新鲜检测时保持停车，避免YOLO尚未恢复或异常退出时无视觉避障返程。
  if (!return_visual_detection_is_fresh()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "返回起点等待新鲜视觉检测：topic=%s，超时=%.2fs，当前停车。",
      return_visual_detection_topic_.c_str(), return_visual_detection_timeout_);
    publish_stop();
    return;
  }

  const double target_yaw = std::atan2(
    return_target_y_ - pose.y, return_target_x_ - pose.x);
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

  publish_debug_visualization(pose);
  cmd_pub_->publish(cmd);
}

void VisualLidarPathFollower::finish_return_to_start(
  const VisualLidarPathFollower::RobotPose & pose, const char * reason)
{
  task3_route_active_ = false;
  return_to_start_active_ = false;
  return_stop_zone_entry_active_ = false;
  set_control_state(ControlState::FINISHED);
  if (logic3_planning_pose_anchored_ &&
    !logic3_final_snapshot_published_)
  {
    logic3_final_pose_ = pose;
    logic3_final_pose_valid_ = std::isfinite(pose.x) &&
      std::isfinite(pose.y) && std::isfinite(pose.yaw);
    publish_logic3_snapshot("final_dynamic");
    logic3_final_snapshot_published_ = true;
  }
  set_task_phase(TaskPhase::FINISHED);
  RCLCPP_INFO(
    get_logger(),
    "到达map起终点：%s；最终base_link map坐标=(%.3f, %.3f)，yaw=%.3frad，执行反向制动。",
    reason, pose.x, pose.y, pose.yaw);
  perform_return_brake_sequence();
  rclcpp::shutdown();
}

void VisualLidarPathFollower::perform_return_brake_sequence()
{
  if (!cmd_pub_) {
    return;
  }

  const auto command_period = std::chrono::duration<double>(
    1.0 / std::max(control_frequency_, 1.0));

  if (return_brake_reverse_speed_ > 0.0 && return_brake_reverse_cycles_ > 0) {
    geometry_msgs::msg::Twist reverse_cmd;
    reverse_cmd.linear.x = -return_brake_reverse_speed_;
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

  RCLCPP_INFO(
    get_logger(),
    "Task3停车制动完成：反向速度=-%.2fm/s，反向=%d周期，零速保持=%d周期，控制频率=%.1fHz。",
    return_brake_reverse_speed_, return_brake_reverse_cycles_,
    return_brake_zero_cycles_, control_frequency_);
}

bool VisualLidarPathFollower::return_stop_zone_contains(
  const VisualLidarPathFollower::RobotPose & pose) const
{
  return pose.x >= return_stop_zone_min_x_ &&
         pose.x <= return_stop_zone_max_x_ &&
         pose.y >= return_stop_zone_min_y_ &&
         pose.y <= return_stop_zone_max_y_;
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

// channel 前瞻已经选定 target 后，将 target 之前的普通点正式提交。
// current_index_ 是线段起点，因此只推进到 target - 1，仍保留 target 作为下一追踪点。
// 拍照点是硬边界；即使未来目标选择逻辑变化，也不允许提交越过它。
void VisualLidarPathFollower::commit_channel_lookahead_progress(
  const VisualLidarPathFollower::TargetPoint & target)
{
  if (following_qr_path_ || !first_waypoint_reached_ || waypoints_.size() < 2) {
    return;
  }

  const std::size_t target_index = std::min(target.waypoint_index, waypoints_.size() - 1);
  if (target_index == 0 || target_index - 1 <= current_index_) {
    return;
  }

  std::size_t committed_index = target_index - 1;
  for (std::size_t index = current_index_ + 1; index <= committed_index; ++index) {
    if (is_precision_waypoint(index)) {
      committed_index = index - 1;
      break;
    }
  }

  if (committed_index > current_index_) {
    advance_current_index_to(committed_index, "前瞻提交");
  }
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
  // Task1最后一点正对二维码。它不能沿用普通航点0.60m通过半径，否则
  // 小车会在摄像头尚未获得足够正视时间时提前结束Task1。
  if (following_qr_path_ && index + 1 == waypoints_.size()) {
    return task1_endpoint_arrival_radius_;
  }
  const auto & waypoint = waypoints_[index];
  if (waypoint.reverse) {
    return std::max(waypoint.reverse_pass_radius, reverse_goal_lookahead_distance_);
  }
  if (waypoint.image_capture) {
    return waypoint.pass_radius;
  }
  return pass_radius_;
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
  // Task1末点必须真正进入专用到达半径：既禁止路线投影提前提交，也禁止
  // 离散前瞻跨过。到点后仍由原有0.30s二维码等待逻辑接管。
  if (following_qr_path_ && index + 1 == waypoints_.size()) {
    return true;
  }
  return enable_image_analysis_trigger_ && waypoints_[index].image_capture;
}

// 根据所选路线方向，在离配置坐标最近的路线段中插入唯一拍照点。
// 拍照点不能被前瞻跳过，但会和普通航点一样参与 waypoint_shift 动态避障。
void VisualLidarPathFollower::insert_image_capture_waypoint(const std::string & direction)
{
  has_image_capture_waypoint_ = false;
  image_capture_stopping_ = false;
  image_capture_recovery_active_ = false;
  image_capture_recovery_used_ = false;
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
  // 这是路线进度的内部防跳过阈值，直接由拍照区最小距离派生，
  // 不再单独暴露旧的“单点拍照半径”参数。
  const double capture_anchor_pass_radius = std::clamp(
    image_capture_zone_min_distance_ * 0.45, 0.05, 0.20);
  capture_waypoint.name = clockwise ? "image_capture_right" : "image_capture_left";
  capture_waypoint.x = capture_point.x;
  capture_waypoint.y = capture_point.y;
  capture_waypoint.image_capture = true;
  capture_waypoint.pass_radius = capture_anchor_pass_radius;
  capture_waypoint.reverse_pass_radius = capture_anchor_pass_radius;

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
    "插入位置=%zu/%zu，路线防跳过半径=%.2fm；拍照扇形区禁止 waypoint_shift。",
    route_direction_name(direction).c_str(), clockwise ? "右侧" : "左侧",
    capture_point.x, capture_point.y, image_capture_waypoint_index_ + 1, waypoints_.size(),
    capture_anchor_pass_radius);
}

// 返回 true 表示本周期保持停车。进入扇形区且姿态合格后锁存停车状态；即使底盘惯性
// 或定位抖动让位姿暂时离开扇形区，也不会重新追逐已经位于车后的拍照点。
VisualLidarPathFollower::Point2D VisualLidarPathFollower::image_capture_sign_point() const
{
  const bool clockwise = to_lower(selected_route_direction_) == "clockwise";
  return Point2D{
    clockwise ? image_sign_clockwise_x_ : image_sign_counterclockwise_x_,
    clockwise ? image_sign_clockwise_y_ : image_sign_counterclockwise_y_};
}

bool VisualLidarPathFollower::image_capture_zone_contains(const Point2D & point) const
{
  if (!has_image_capture_waypoint_ || following_qr_path_) {
    return false;
  }
  const Point2D sign = image_capture_sign_point();
  const double distance = std::hypot(point.x - sign.x, point.y - sign.y);
  if (distance < image_capture_zone_min_distance_ ||
    distance > image_capture_zone_max_distance_)
  {
    return false;
  }

  // 顺时针车辆从右侧立牌西侧驶来；逆时针车辆从左侧立牌东侧驶来。
  const bool clockwise = to_lower(selected_route_direction_) == "clockwise";
  const double expected_vehicle_bearing = clockwise ? M_PI : 0.0;
  const double vehicle_bearing = std::atan2(point.y - sign.y, point.x - sign.x);
  const double sector_error = std::abs(normalize_angle(
    vehicle_bearing - expected_vehicle_bearing));
  return sector_error <= image_capture_zone_half_angle_deg_ * M_PI / 180.0;
}

bool VisualLidarPathFollower::image_capture_pose_is_acceptable(
  const VisualLidarPathFollower::RobotPose & pose, double * distance_to_sign,
  double * heading_error) const
{
  const Point2D position{pose.x, pose.y};
  if (!image_capture_zone_contains(position)) {
    return false;
  }
  const Point2D sign = image_capture_sign_point();
  const double distance = std::hypot(sign.x - pose.x, sign.y - pose.y);
  const double target_yaw = std::atan2(sign.y - pose.y, sign.x - pose.x);
  const double yaw_error = normalize_angle(pose.yaw - target_yaw);
  if (distance_to_sign) {
    *distance_to_sign = distance;
  }
  if (heading_error) {
    *heading_error = yaw_error;
  }
  return std::abs(yaw_error) <= image_capture_heading_tolerance_deg_ * M_PI / 180.0;
}

bool VisualLidarPathFollower::is_image_capture_protected_waypoint(std::size_t index) const
{
  if (!has_image_capture_waypoint_ || index >= original_waypoints_.size()) {
    return false;
  }
  if (index == image_capture_waypoint_index_) {
    return true;
  }
  const auto & waypoint = original_waypoints_[index];
  return image_capture_zone_contains(Point2D{waypoint.x, waypoint.y});
}

bool VisualLidarPathFollower::apply_image_capture_recovery(
  const VisualLidarPathFollower::RobotPose & pose, geometry_msgs::msg::Twist & cmd)
{
  if (!image_capture_recovery_active_) {
    return false;
  }

  const double backed_distance = std::hypot(
    pose.x - image_capture_recovery_start_pose_.x,
    pose.y - image_capture_recovery_start_pose_.y);
  if (backed_distance >= image_capture_recovery_reverse_distance_) {
    image_capture_recovery_active_ = false;
    RCLCPP_INFO(
      get_logger(),
      "图生文拍照复位倒车完成：后退 %.3fm，恢复低速精确接近。",
      backed_distance);
    return false;
  }

  RobotPose scan_to_base;
  RobotPose scan_to_map;
  if (!prepare_lidar_scan(scan_to_base, scan_to_map)) {
    cmd = geometry_msgs::msg::Twist();
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "图生文拍照复位等待新鲜雷达与TF，当前保持停车。");
    return true;
  }
  const auto rear_obstacle = find_lidar_obstacle(
    M_PI, image_capture_recovery_rear_obstacle_distance_,
    image_capture_recovery_rear_fov_deg_,
    image_capture_recovery_rear_corridor_half_width_, scan_to_base, scan_to_map);
  if (rear_obstacle.found) {
    cmd = geometry_msgs::msg::Twist();
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "图生文拍照复位后方有障碍：距离=%.3fm，停止倒车并等待后方清空。",
      rear_obstacle.range);
    return true;
  }

  const Point2D sign = image_capture_sign_point();
  const double target_yaw = std::atan2(sign.y - pose.y, sign.x - pose.x);
  const double yaw_error = normalize_angle(target_yaw - pose.yaw);
  cmd.linear.x = -image_capture_recovery_reverse_speed_;
  cmd.angular.z = std::clamp(
    image_capture_recovery_heading_gain_ * yaw_error,
    -image_capture_recovery_max_angular_speed_,
    image_capture_recovery_max_angular_speed_);
  return true;
}

bool VisualLidarPathFollower::handle_image_capture_stop(
  const VisualLidarPathFollower::RobotPose & pose)
{
  // 拍照触发以固定立牌的物理扇形区为准，而不是以离散路线索引为准。
  // 动态避障或曲线重建可能使 current_index_ 已经跨过插入锚点；继续以
  // “必须在锚点前一位”为条件会导致车辆实际经过立牌却完全不触发。
  if (!has_image_capture_waypoint_ || image_analysis_triggered_ ||
    task_phase_ != TaskPhase::TASK2)
  {
    return false;
  }

  double distance_to_sign = 0.0;
  double heading_error = 0.0;
  const bool acceptable = image_capture_pose_is_acceptable(
    pose, &distance_to_sign, &heading_error);
  const Point2D position{pose.x, pose.y};
  if (!acceptable) {
    if (image_capture_zone_contains(position) && image_capture_recovery_enabled_ &&
      !image_capture_recovery_used_)
    {
      image_capture_recovery_used_ = true;
      image_capture_recovery_active_ = true;
      image_capture_recovery_start_pose_ = pose;
      image_capture_stopping_ = false;
      RCLCPP_WARN(
        get_logger(),
        "图生文拍照区姿态不理想：距牌 %.3fm，航向误差 %.1fdeg；"
        "执行唯一一次低速倒车复位 %.2fm。",
        distance_to_sign, heading_error * 180.0 / M_PI,
        image_capture_recovery_reverse_distance_);
      return true;
    }
    const Point2D sign = image_capture_sign_point();
    const double distance = std::hypot(sign.x - pose.x, sign.y - pose.y);
    if (distance <= image_capture_approach_distance_) {
      const double bearing = std::atan2(pose.y - sign.y, pose.x - sign.x);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "图生文接近立牌但尚未进入可拍扇区：距牌 %.3fm，方位 %.1fdeg；"
        "继续低速朝立牌收正。",
        distance, bearing * 180.0 / M_PI);
    }
    return false;
  }

  const auto now = std::chrono::steady_clock::now();
  if (!image_capture_stopping_) {
    image_capture_stopping_ = true;
    image_capture_stop_started_at_ = now;
    RCLCPP_INFO(
      get_logger(),
      "到达图生文扇形拍照区：距牌 %.3fm，航向误差 %.1fdeg，停车稳定 %.2fs。",
      distance_to_sign, heading_error * 180.0 / M_PI, image_capture_stop_time_);
  }

  const double stopped_seconds =
    std::chrono::duration<double>(now - image_capture_stop_started_at_).count();
  if (stopped_seconds < image_capture_stop_time_) {
    return true;
  }

  publish_image_analysis_trigger();
  if (image_analysis_triggered_ && current_index_ < image_capture_waypoint_index_) {
    advance_current_index_to(image_capture_waypoint_index_, "图生文拍照完成");
  }
  image_capture_stopping_ = false;
  return false;
}

void VisualLidarPathFollower::apply_image_capture_speed_limit(
  const VisualLidarPathFollower::RobotPose & pose,
  geometry_msgs::msg::Twist & cmd) const
{
  if (!has_image_capture_waypoint_ || image_analysis_triggered_ ||
    task_phase_ != TaskPhase::TASK2)
  {
    return;
  }
  const Point2D sign = image_capture_sign_point();
  const double distance = std::hypot(sign.x - pose.x, sign.y - pose.y);
  if (distance > image_capture_approach_distance_) {
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
  const bool tracking_image_capture =
    enable_image_analysis_trigger_ && target.waypoint_index < waypoints_.size() &&
    waypoints_[target.waypoint_index].image_capture;
  const Point2D sign = image_capture_sign_point();
  const bool image_approach_active =
    enable_image_analysis_trigger_ && has_image_capture_waypoint_ &&
    !image_analysis_triggered_ && !following_qr_path_ &&
    task_phase_ == TaskPhase::TASK2 &&
    std::hypot(sign.x - pose.x, sign.y - pose.y) <= image_capture_approach_distance_;
  Point2D steering_point = target.point;
  // 进入接近区后直接把车头收向固定立牌。这里不再依赖前瞻目标仍恰好是
  // 插入锚点，避免路线进度跨过锚点后失去朝向控制。
  if (!target.reverse && image_approach_active) {
    steering_point = sign;
  }
  const double dx = steering_point.x - pose.x;
  const double dy = steering_point.y - pose.y;

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
    const double angular_gain =
      (tracking_image_capture || image_approach_active) ?
      image_capture_angular_gain_ : turn_angular_gain_;
    cmd.linear.x = linear_speed_;
    cmd.angular.z = angular_gain * heading_error;
  }
  return cmd;
}

// 根据即将下发的目标前轮转角绝对值线性降速。左右相同目标转角使用相同
// 降速比例；舵机 PWM 的 1050/2200 非对称映射仍由下位机处理。
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
// 其余点保留逐点法线偏移。内弯偏移超过曲率半径时，不再强迫路线经过
// “来源点沿单一法向平移”的平台点，而是在危险圆弧两侧仍然有效的直接
// 偏移支路之间架一条三次Bezier桥。这相当于切掉已经坍缩的内侧等距圆弧。
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

  std::size_t rebuilt_count = 0;
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
      // 偏移后曲率兜底，就会破坏1/4—3/4直接偏移分配；仅跳过与B重叠的
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

      // 障碍来源点落在危险子段内时以它为局部平台中心；否则使用子段中
      // 风险最大的点，避免把平台重新扩展到整个障碍影响窗口。
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

      // before/after仍是实车严格调试后的绝对影响边界。先只取危险圆弧
      // 两侧一个点作为接回锚点；若空间不够，再在同一障碍物实际取得的
      // 1/4—3/4所有权窗口内逐点扩展。这样两个相邻圆角分别修复，中间
      // 的直线重叠分配不会被一次跨窗口Bezier重新改写。
      const std::size_t maximum_context = std::max(
        unsafe_first - run_first, run_last - unsafe_last);
      std::size_t accepted_anchor_first = unsafe_first;
      std::size_t accepted_anchor_last = unsafe_last;
      double accepted_handle_ratio = 0.0;
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
          unsafe_first > run_first + context ?
          unsafe_first - context : run_first;
        const std::size_t anchor_last =
          std::min(run_last, unsafe_last + context);
        if (anchor_first == previous_anchor_first &&
          anchor_last == previous_anchor_last)
        {
          continue;
        }
        previous_anchor_first = anchor_first;
        previous_anchor_last = anchor_last;
        if (anchor_first >= unsafe_first || anchor_last <= unsafe_last) {
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
          if (source_clearance + 1e-6 >= target_offset &&
            chord_progress_valid(
              rebuilt_candidate, anchor_first, anchor_last,
              start_point, finish_point, last_invalid_reason) &&
            local_geometry_valid(
              rebuilt_candidate, anchor_first, anchor_last, last_invalid_reason))
          {
            valid_rebuild = true;
            accepted_anchor_first = anchor_first;
            accepted_anchor_last = anchor_last;
            accepted_handle_ratio = handle_ratio;
            accepted_bridge_mode = "单段切角桥";
            accepted_candidate = std::move(rebuilt_candidate);
            break;
          }
          if (source_clearance + 1e-6 < target_offset) {
            last_invalid_reason = "Bezier桥在障碍来源点未达到目标同侧净偏移";
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
              rebuilt_candidate, anchor_first, anchor_last, last_invalid_reason))
          {
            valid_rebuild = true;
            accepted_anchor_first = anchor_first;
            accepted_anchor_last = anchor_last;
            accepted_handle_ratio = handle_ratio;
            accepted_bridge_mode = "双段外弯保净空桥";
            accepted_candidate = std::move(rebuilt_candidate);
            break;
          }
        }
      }
      // 所有Bezier候选都必须通过前进比例、局部转角和自交检查。旧实现
      // 会无条件提交最后一个零控制柄候选，正是圆角处出现前进后折返的
      // 原因。若仍不合格，宁可保留本轮进入函数前的路线并报告错误，也
      // 不能把已知会倒退的路径交给追点器。
      if (!valid_rebuild) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "waypoint_shift：危险子段%zu-%zu在所属直接偏移窗口%zu-%zu内仍无法"
          "生成严格前进且无自交的独立同侧曲线（最后原因：%s）；"
          "拒绝提交无效兜底路径。",
          unsafe_first + 1, unsafe_last + 1, run_first + 1, run_last + 1,
          last_invalid_reason.c_str());
        unsafe_first = unsafe_last + 1;
        continue;
      }
      shifted_route = std::move(accepted_candidate);
      ++rebuilt_count;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "waypoint_shift：检测到圆弧内侧偏移不可行（危险点%zu-%zu，"
        "所属直接偏移窗口%zu-%zu，最小进度/可行因子=%.3f），"
        "仅在点%zu-%zu内用%s独立重建危险圆弧，来源点=%zu，"
        "目标偏移=%.3fm，Bezier控制柄比例=%.3f。",
        unsafe_first + 1, unsafe_last + 1, run_first + 1, run_last + 1,
        minimum_risk_factor, accepted_anchor_first + 1, accepted_anchor_last + 1,
        accepted_bridge_mode, source_index + 1, target_offset, accepted_handle_ratio);
      unsafe_first = unsafe_last + 1;
    }
    run_first = run_last + 1;
  }
  return rebuilt_count;
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
  bool filter_logic3_green_mask) const
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
        robot_pose, base_point, active_restriction_map,
        active_restriction_inflation_cells))
    {
      finish_cluster();
      angle += scan.angle_increment;
      continue;
    }
    const Point2D map_point = base_to_map(robot_pose, base_point);
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
  const double radius_squared = logic3_old_cluster_radius_ * logic3_old_cluster_radius_;
  return std::any_of(
    logic3_green_clusters_.begin(), logic3_green_clusters_.end(),
    [&](const ScanPointCluster & green_cluster) {
      // Task1静态矩形外的冻结绿色簇不做静态路线偏移，因此也不能被
      // 绿圈掩码过滤；保留给实时雷达动态避障。进入过渡后恢复正常掩码。
      if ((following_qr_path_ || task_phase_ == TaskPhase::TASK1) &&
        !task1_static_avoid_region_contains(green_cluster.center))
      {
        return false;
      }
      // 进入返程段后，分界线以下的整个障碍簇交给实时雷达，不能继续
      // 被冻结绿色掩码吃掉。按簇中心分类，避免同一个锥桶跨线时被拆开。
      // 上方障碍仍完整保留掩码，即使其点云或偏移尾段延伸到分界线以下。
      if (task3_route_active_ &&
        green_cluster.center.y <= return_obstacle_split_y_)
      {
        return false;
      }
      const double dx = map_point.x - green_cluster.center.x;
      const double dy = map_point.y - green_cluster.center.y;
      return dx * dx + dy * dy <= radius_squared;
    });
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

// 在指定朝向上取最近障碍：总视野扇形、最大距离和矩形走廊必须同时命中。
// center_angle=0 检查车前，center_angle=pi 检查车后。
VisualLidarPathFollower::LidarObstacle
VisualLidarPathFollower::find_lidar_obstacle(
  double center_angle, double maximum_distance, double fov_deg,
  double corridor_half_width,
  const VisualLidarPathFollower::RobotPose & scan_to_base,
  const VisualLidarPathFollower::RobotPose & scan_to_map,
  bool filter_logic3_green_mask,
  bool filter_logic3_runtime_map) const
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
    if (point_range > maximum_distance || std::abs(relative_angle) > half_fov) {
      continue;
    }

    const double forward = point_range * std::cos(relative_angle);
    const double lateral = point_range * std::sin(relative_angle);
    if (forward <= 0.0 || std::abs(lateral) > corridor_half_width) {
      continue;
    }

    if (lidar_enable_static_map_filter_ || filter_logic3_runtime_map ||
      filter_logic3_green_mask)
    {
      const Point2D map_point{
        scan_to_map.x + cos_map * scan_x - sin_map * scan_y,
        scan_to_map.y + sin_map * scan_x + cos_map * scan_y};
      if ((lidar_enable_static_map_filter_ &&
        lidar_map_point_is_wall(map_point)) ||
        (filter_logic3_runtime_map &&
        logic3_runtime_map_point_is_filtered(map_point)) ||
        (filter_logic3_green_mask && logic3_green_mask_contains(map_point)))
      {
        continue;
      }
    }

    if (!result.found || point_range < result.range) {
      result.found = true;
      result.range = point_range;
      result.relative_angle = relative_angle;
    }
  }
  return result;
}

// 扫码后到进入waypoint_channel或达到配置轮数上限前，PENDING始终表示
// “倒车机制已武装”。qr_reverse_max_count为0时不武装、完全禁用倒车。
// 新一轮倒车只需前方命中即可触发；map.y和map.x仅作为倒车中的或关系
// 退出保护。本轮结束后回到PENDING，不关闭机制。前后同时命中时仍倒车，
// 并把右转量提升到配置的满舵角。
bool VisualLidarPathFollower::apply_qr_reverse_maneuver(
  const VisualLidarPathFollower::RobotPose & robot_pose,
  geometry_msgs::msg::Twist & cmd)
{
  if (qr_escape_state_ == QrEscapeState::IDLE) {
    return false;
  }

  RobotPose scan_to_base;
  RobotPose scan_to_map;
  if (!prepare_lidar_scan(scan_to_base, scan_to_map)) {
    cmd = geometry_msgs::msg::Twist();
    if (!qr_reverse_waiting_for_inputs_) {
      RCLCPP_WARN(
        get_logger(),
        "二维码倒车等待新鲜雷达、物理地图及采样时刻TF，当前停车。");
      qr_reverse_waiting_for_inputs_ = true;
    }
    return true;
  }
  qr_reverse_waiting_for_inputs_ = false;

  const auto front_obstacle = find_lidar_obstacle(
    0.0, qr_reverse_front_obstacle_distance_, qr_reverse_front_fov_deg_,
    qr_reverse_front_corridor_half_width_, scan_to_base, scan_to_map);
  const auto rear_obstacle = find_lidar_obstacle(
    M_PI, qr_reverse_rear_obstacle_distance_, qr_reverse_rear_fov_deg_,
    qr_reverse_rear_corridor_half_width_, scan_to_base, scan_to_map);

  // 当前底盘在负线速度下使用与qr_reverse_angular_speed相同的符号向右后倒。
  // turn_speed_full_steering_angle是本节点统一使用的满舵绝对值。
  const double full_right_angular_speed = std::copysign(
    turn_speed_full_steering_angle_,
    std::abs(qr_reverse_angular_speed_) > 1.0e-9 ?
    qr_reverse_angular_speed_ : 1.0);
  const auto set_reverse_command = [&](bool full_right) {
      cmd.linear.x = -qr_reverse_linear_speed_;
      cmd.angular.z = full_right ?
        full_right_angular_speed : qr_reverse_angular_speed_;
    };

  if (qr_escape_state_ == QrEscapeState::PENDING) {
    const bool reverse_triggered = front_obstacle.found;
    if (reverse_triggered) {
      qr_escape_state_ = QrEscapeState::BACKING_RIGHT;
      const bool full_right = rear_obstacle.found;
      set_reverse_command(full_right);
      RCLCPP_WARN(
        get_logger(),
        "开始二维码条件倒车：前方障碍=是，map.y=%.3f（仅作退出保护%.3f），"
        "后方障碍=%s%s；输出 linear.x=%.2f, angular.z=%.2f。",
        robot_pose.y, qr_reverse_y_threshold_, rear_obstacle.found ? "是" : "否",
        full_right ? "，执行右打满后倒" : "",
        cmd.linear.x, cmd.angular.z);
      return true;
    }
    if (debug_control_logs_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 500,
        "debug: 二维码倒车机制保持武装，前障碍=%s，后障碍=%s，"
        "map.y=%.3f（仅作退出保护%.3f）。",
        front_obstacle.found ? "是" : "否",
        rear_obstacle.found ? "是" : "否",
        robot_pose.y, qr_reverse_y_threshold_);
    }
    return false;
  }

  const bool front_clear = !front_obstacle.found;
  const bool y_limit_reached = robot_pose.y <= qr_reverse_y_threshold_;
  const bool x_limit_reached = robot_pose.x >= qr_reverse_x_limit_;
  if (front_clear || y_limit_reached || x_limit_reached) {
    // 只解除当前这一轮倒车；完成配置轮数后停止再次触发。
    ++channel_completed_reverse_count_;
    const bool reverse_count_limit_reached =
      qr_reverse_max_count_ <= 0 ||
      channel_completed_reverse_count_ >=
      static_cast<std::size_t>(qr_reverse_max_count_);
    qr_escape_state_ = reverse_count_limit_reached ?
      QrEscapeState::IDLE : QrEscapeState::PENDING;
    channel_entry_route_rebuild_pending_ = true;
    channel_entry_route_rebuild_after_reverse_ = true;
    if (reverse_count_limit_reached) {
      RCLCPP_WARN(
        get_logger(),
        "二维码倒车第%zu轮结束：或条件命中[前方清空=%s，"
        "map.y=%.3f<=%.3f=%s，map.x=%.3f>=%.3f=%s]；"
        "立即以当前base_link重规划；已达到配置上限%d轮，本次扫码不再触发倒车。",
        channel_completed_reverse_count_, front_clear ? "是" : "否",
        robot_pose.y, qr_reverse_y_threshold_, y_limit_reached ? "是" : "否",
        robot_pose.x, qr_reverse_x_limit_, x_limit_reached ? "是" : "否",
        qr_reverse_max_count_);
    } else {
      RCLCPP_INFO(
        get_logger(),
        "二维码倒车第%zu轮结束：或条件命中[前方清空=%s，"
        "map.y=%.3f<=%.3f=%s，map.x=%.3f>=%.3f=%s]；"
        "立即以当前base_link重规划；倒车机制继续保持武装%s。",
        channel_completed_reverse_count_, front_clear ? "是" : "否",
        robot_pose.y, qr_reverse_y_threshold_, y_limit_reached ? "是" : "否",
        robot_pose.x, qr_reverse_x_limit_, x_limit_reached ? "是" : "否",
        "");
    }
    return false;
  }

  const bool full_right = front_obstacle.found && rear_obstacle.found;
  set_reverse_command(full_right);
  if (debug_control_logs_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 500,
      "debug: 二维码条件倒车，map.y=%.3f，前障碍=%s，后障碍=%s%s，"
      "输出=%.2f/%.2f。",
      robot_pose.y, front_obstacle.found ? "是" : "否",
      rear_obstacle.found ? "是" : "否",
      full_right ? "，右打满后倒" : "", cmd.linear.x, cmd.angular.z);
  }
  return true;
}

// 进入 channel 航点邻域后结束进场速度覆盖；静态+动态方案继续沿用雷达避障，
// 但正常速度恢复为 channel 路线跟踪速度。
void VisualLidarPathFollower::update_qr_channel_approach_phase(
  const VisualLidarPathFollower::RobotPose & robot_pose)
{
  if (qr_escape_state_ == QrEscapeState::BACKING_RIGHT) {
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
  set_task_phase(TaskPhase::TASK2);
  current_index_ = std::min(
    std::max(current_index_, entry_index), waypoints_.size() - 1);
  first_waypoint_reached_ = true;
  RCLCPP_INFO(
    get_logger(),
    "已进入 waypoint_channel：延长段=%zu点，入口首点距离=%.3fm，入口截面穿越=%s；"
    "切换到Task2参数，继续使用运行限制图和绿圈掩码后的雷达避障。",
    entry_index, entry_waypoint_distance, crossed_entry_section ? "是" : "否");
}

// channel 雷达避障：逐点经过物理墙体、静态+动态方案运行限制图和冻结绿圈过滤后，
// 取前方扇形走廊内最近的新障碍。障碍在左则右绕，障碍在右则左绕。
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
  const bool filter_logic3_green_mask =
    logic3_planning_pose_anchored_ && !logic3_green_clusters_.empty();
  RobotPose scan_to_base;
  RobotPose scan_to_map;
  if (!prepare_lidar_scan(
      scan_to_base, scan_to_map,
      filter_logic3_runtime_map || filter_logic3_green_mask,
      filter_logic3_runtime_map))
  {
    cmd = geometry_msgs::msg::Twist();
    if (!channel_lidar_waiting_for_inputs_) {
      RCLCPP_WARN(
        get_logger(),
        "channel雷达避障等待新鲜雷达、物理地图、静态+动态方案运行限制图及采样时刻TF，当前停车。");
      channel_lidar_waiting_for_inputs_ = true;
    }
    channel_lidar_obstacle_active_ = false;
    channel_lidar_logged_direction_ = 0;
    return true;
  }
  channel_lidar_waiting_for_inputs_ = false;
  if (observation_valid) {
    *observation_valid = true;
  }

  const auto obstacle = find_lidar_obstacle(
    0.0, channel_lidar_avoid_distance_, channel_lidar_avoid_fov_deg_,
    channel_lidar_corridor_half_width_, scan_to_base, scan_to_map,
    filter_logic3_green_mask, filter_logic3_runtime_map);
  if (!obstacle.found) {
    if (channel_lidar_obstacle_active_) {
      RCLCPP_INFO(
        get_logger(), "channel雷达避障结束：前方检测区已清空，恢复当前阶段路线跟踪。");
      channel_lidar_obstacle_active_ = false;
      channel_lidar_logged_direction_ = 0;
    }
    return false;
  }
  if (obstacle_found) {
    *obstacle_found = true;
  }

  const double half_fov = std::max(
    channel_lidar_avoid_fov_deg_ * M_PI / 360.0, 0.01);
  const double center_weight = std::clamp(
    1.0 - std::abs(obstacle.relative_angle) / half_fov, 0.0, 1.0);
  const double upper_angle =
    channel_lidar_direction_upper_angle_deg_ * M_PI / 180.0;
  const double lower_angle =
    channel_lidar_direction_lower_angle_deg_ * M_PI / 180.0;
  if (obstacle.relative_angle > upper_angle) {
    // 障碍明确在左侧：右绕。
    channel_lidar_avoid_direction_ = -1;
  } else if (obstacle.relative_angle < lower_angle) {
    // 障碍明确在右侧：左绕。
    channel_lidar_avoid_direction_ = 1;
  } else if (channel_lidar_avoid_direction_ == 0) {
    // 首次遇到正中障碍时沿用原策略：默认左绕。滞环内后续保持该方向。
    channel_lidar_avoid_direction_ = 1;
  }
  const double avoid_direction =
    static_cast<double>(channel_lidar_avoid_direction_);
  cmd.linear.x = channel_lidar_avoid_linear_speed_;
  cmd.angular.z = avoid_direction * channel_lidar_avoid_angular_gain_ * center_weight;
  if (!channel_lidar_obstacle_active_ ||
    channel_lidar_logged_direction_ != channel_lidar_avoid_direction_)
  {
    RCLCPP_WARN(
      get_logger(),
      "channel雷达避障开始%s：距离=%.3fm，相对角=%.1fdeg，"
      "滞环=[%.1f, %.1f]deg，保持%s绕，输出=%.2f/%.2f。",
      channel_lidar_obstacle_active_ ? "（方向切换）" : "",
      obstacle.range, obstacle.relative_angle * 180.0 / M_PI,
      channel_lidar_direction_lower_angle_deg_,
      channel_lidar_direction_upper_angle_deg_,
      channel_lidar_avoid_direction_ > 0 ? "左" : "右",
      cmd.linear.x, cmd.angular.z);
    channel_lidar_obstacle_active_ = true;
    channel_lidar_logged_direction_ = channel_lidar_avoid_direction_;
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
  // Task3到达离散路线末端后不会按索引结束，而会持续追踪这个中心点，
  // 直至base_link进入同尺寸的紫色/蓝色基地矩形并立即停车。
  // 这里再做一次运行时校正，避免旧航点文件仍把车辆牵引到旧左下角。
  if (task3_route_available_ && !waypoints_.empty()) {
    waypoints_.back().x = return_target_x_;
    waypoints_.back().y = return_target_y_;
  }
  original_waypoints_ = waypoints_;
  waypoint_shift_offsets_.assign(waypoints_.size(), 0.0);
  waypoint_shift_pending_offsets_.assign(waypoints_.size(), 0.0);
  waypoint_shift_pending_counts_.assign(waypoints_.size(), 0);
  waypoint_shift_missing_counts_.assign(waypoints_.size(), 0);
  waypoint_shift_dynamic_directions_.assign(waypoints_.size(), 0);
  waypoint_shift_world_directions_.assign(waypoints_.size(), Point2D());
  waypoint_shift_source_indices_.assign(waypoints_.size(), waypoints_.size());
  logic3_frozen_base_shift_directions_.clear();
  logic3_frozen_base_source_indices_.clear();
  logic3_runtime_cluster_tracks_.clear();
  channel_lidar_avoid_direction_ = 0;
  processed_waypoint_shift_scan_sequence_ = scan_sequence_;
}

void VisualLidarPathFollower::update_logic3_static_cluster_tracks(
  const std::vector<VisualLidarPathFollower::ScanPointCluster> & clusters,
  const VisualLidarPathFollower::RobotPose & robot_pose)
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
      track.state1_sample_count = 0;
      track.sample_center_sum_x = 0.0;
      track.sample_center_sum_y = 0.0;
      track.sampled_points.clear();
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
      track.state1_sample_count = 0;
      track.sample_center_sum_x = 0.0;
      track.sample_center_sum_y = 0.0;
      track.sampled_points.clear();
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

  // 只有整体状态1有效时才累计10个有效坐标。当前帧未匹配只暂停该ID；
  // 失联宽限中的ID也不采样。其他ID原有采样与READY结果不受影响。
  for (std::size_t index = 0; index < logic3_static_cluster_tracks_.size(); ++index) {
    auto & track = logic3_static_cluster_tracks_[index];
    if (!logic3_static_state1_active_ || !track.entered_state1 ||
      !track.confirmed || !track_matched[index] || track.loss_pending ||
      track.ready)
    {
      continue;
    }

    ++track.state1_sample_count;
    track.sample_center_sum_x += track.cluster.center.x;
    track.sample_center_sum_y += track.cluster.center.y;
    track.sampled_points.insert(
      track.sampled_points.end(),
      track.cluster.points.begin(), track.cluster.points.end());

    if (track.state1_sample_count < logic3_static_average_samples_) {
      continue;
    }

    ScanPointCluster averaged_cluster;
    const double sample_count = static_cast<double>(track.state1_sample_count);
    averaged_cluster.center.x = track.sample_center_sum_x / sample_count;
    averaged_cluster.center.y = track.sample_center_sum_y / sample_count;
    averaged_cluster.points = track.sampled_points;
    if (averaged_cluster.points.empty()) {
      averaged_cluster.points.push_back(averaged_cluster.center);
    } else {
      Point2D points_center;
      for (const auto & point : averaged_cluster.points) {
        points_center.x += point.x;
        points_center.y += point.y;
      }
      const double point_count =
        static_cast<double>(averaged_cluster.points.size());
      points_center.x /= point_count;
      points_center.y /= point_count;
      const double translate_x = averaged_cluster.center.x - points_center.x;
      const double translate_y = averaged_cluster.center.y - points_center.y;
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
         << "\n固定坐标：";
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
           << " 采样=" << std::min(
      track->state1_sample_count, logic3_static_average_samples_)
           << '/' << logic3_static_average_samples_ << "；";
  }
  status << "\nREADY=" << (logic3_static_ready_ ? 1 : 0)
         << "（独立就绪=" << ready_track_count << '/' << fixed_tracks.size()
         << "；可信发车冻结最后一次完整READY快照）";
  RCLCPP_INFO(get_logger(), "%s", status.str().c_str());
}

void VisualLidarPathFollower::record_logic3_red_cluster(
  const VisualLidarPathFollower::ScanPointCluster & cluster)
{
  const auto recorded = std::find_if(
    logic3_red_cluster_history_.begin(), logic3_red_cluster_history_.end(),
    [&](const Point2D & point) {
      return std::hypot(
        point.x - cluster.center.x, point.y - cluster.center.y) <=
      logic3_red_history_merge_radius_;
    });
  if (recorded != logic3_red_cluster_history_.end()) {
    // 历史圆只负责绘图身份；中心持续跟随该身份的平滑坐标，避免最终图片
    // 永远停在第一次单帧观测的位置。
    *recorded = cluster.center;
    return;
  }
  logic3_red_cluster_history_.push_back(cluster.center);
  RCLCPP_WARN(
    get_logger(),
    "静态+动态方案记录通道内红点：map(%.3f, %.3f)，图片历史半径=%.2fm；"
    "不参与路径偏移。",
    cluster.center.x, cluster.center.y, logic3_red_history_merge_radius_);
}

void VisualLidarPathFollower::update_logic3_runtime_red_tracks(
  const std::vector<VisualLidarPathFollower::ScanPointCluster> & clusters)
{
  std::vector<bool> track_matched(logic3_runtime_cluster_tracks_.size(), false);

  auto refresh_smoothed_cluster = [&](Logic3RuntimeClusterTrack & track) {
      Point2D averaged_center;
      for (const auto & sample : track.recent_samples) {
        averaged_center.x += sample.center.x;
        averaged_center.y += sample.center.y;
      }
      const double sample_count = static_cast<double>(track.recent_samples.size());
      averaged_center.x /= sample_count;
      averaged_center.y /= sample_count;

      // 只平滑中心，不叠加运动过程中各帧点云，避免把一段运动轨迹误当成
      // 一个膨胀障碍。几何外形取最新帧并平移到平均中心。
      track.cluster = track.recent_samples.back();
      const double translate_x = averaged_center.x - track.cluster.center.x;
      const double translate_y = averaged_center.y - track.cluster.center.y;
      for (auto & point : track.cluster.points) {
        point.x += translate_x;
        point.y += translate_y;
      }
      track.cluster.center = averaged_center;
    };

  for (const auto & cluster : clusters) {
    std::size_t best_track = logic3_runtime_cluster_tracks_.size();
    double best_distance = logic3_runtime_cluster_match_radius_;
    for (std::size_t index = 0; index < logic3_runtime_cluster_tracks_.size(); ++index) {
      if (track_matched[index]) {
        continue;
      }
      const auto & center = logic3_runtime_cluster_tracks_[index].latest_center;
      const double distance = std::hypot(
        cluster.center.x - center.x, cluster.center.y - center.y);
      if (distance <= best_distance) {
        best_distance = distance;
        best_track = index;
      }
    }

    if (best_track == logic3_runtime_cluster_tracks_.size()) {
      Logic3RuntimeClusterTrack track;
      track.cluster = cluster;
      track.latest_center = cluster.center;
      track.consecutive_hits = 1;
      track.recent_samples.push_back(cluster);
      logic3_runtime_cluster_tracks_.push_back(std::move(track));
      track_matched.push_back(true);
      if (waypoint_shift_confirm_scans_ == 1) {
        auto & new_track = logic3_runtime_cluster_tracks_.back();
        new_track.confirmed = true;
        RCLCPP_WARN(
          get_logger(),
          "静态+动态方案红点记录确认：map(%.3f, %.3f)，仅用于绘图，不参与路径偏移。",
          new_track.cluster.center.x, new_track.cluster.center.y);
        record_logic3_red_cluster(new_track.cluster);
      }
      continue;
    }

    auto & track = logic3_runtime_cluster_tracks_[best_track];
    if (track.missing_count > 0) {
      track.consecutive_hits = 0;
      track.recent_samples.clear();
    }
    track.latest_center = cluster.center;
    track.missing_count = 0;
    ++track.consecutive_hits;
    track.recent_samples.push_back(cluster);
    while (track.recent_samples.size() >
      static_cast<std::size_t>(logic3_runtime_average_samples_))
    {
      track.recent_samples.pop_front();
    }
    refresh_smoothed_cluster(track);
    track_matched[best_track] = true;

    if (!track.confirmed &&
      track.consecutive_hits >= waypoint_shift_confirm_scans_)
    {
      track.confirmed = true;
      RCLCPP_WARN(
        get_logger(),
        "静态+动态方案红点记录确认：map(%.3f, %.3f)，连续%d帧，身份半径=%.2fm；"
        "仅用于绘图，不参与路径偏移。",
        track.cluster.center.x, track.cluster.center.y,
        track.consecutive_hits, logic3_runtime_cluster_match_radius_);
    }

    if (track.confirmed) {
      record_logic3_red_cluster(track.cluster);
    }
  }

  for (std::size_t index = 0; index < logic3_runtime_cluster_tracks_.size(); ++index) {
    if (!track_matched[index]) {
      auto & track = logic3_runtime_cluster_tracks_[index];
      ++track.missing_count;
      track.consecutive_hits = 0;
    }
  }
  logic3_runtime_cluster_tracks_.erase(
    std::remove_if(
      logic3_runtime_cluster_tracks_.begin(), logic3_runtime_cluster_tracks_.end(),
      [&](const Logic3RuntimeClusterTrack & track) {
        return track.missing_count >= waypoint_shift_release_scans_;
      }),
    logic3_runtime_cluster_tracks_.end());
}

void VisualLidarPathFollower::logic3_odom_callback(
  const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  if (logic3_planning_pose_anchored_) {
    // 人工冻结后，仍继续监测一次运动证据。这样发车后即便车又停住，
    // 数字键2也不能解除已经用于比赛行驶的静态规划路线。
    if (logic3_manual_freeze_active_) {
      const double speed = std::hypot(
        msg->twist.twist.linear.x, msg->twist.twist.linear.y);
      if (speed >= logic3_motion_speed_threshold_) {
        logic3_manual_unfreeze_locked_ = true;
      }
    }
    return;
  }

  const double odom_x = msg->pose.pose.position.x;
  const double odom_y = msg->pose.pose.position.y;
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
  auto capture_departure_snapshot = [&]() {
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

void VisualLidarPathFollower::manual_route_freeze_callback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  if (request->data) {
    if (logic3_planning_pose_anchored_) {
      response->success = false;
      response->message = "路线已经冻结；若是人工冻结且尚未发车，可按2解冻。";
      return;
    }

    // 与自动发车保持相同的证据标准：只使用最近一份完整 READY 快照，
    // 从未形成 READY 快照时按既有安全规则冻结空集，绝不取单帧原始雷达。
    logic3_motion_pending_snapshot_ = logic3_last_complete_ready_snapshot_;
    if (!logic3_motion_pending_snapshot_.valid) {
      logic3_motion_pending_snapshot_.valid = true;
      logic3_motion_pending_snapshot_.stamp_seconds = now().seconds();
      RCLCPP_WARN(
        get_logger(),
        "人工冻结时尚未形成完整READY快照：按安全规则冻结空静态障碍集。\n"
        "请按2解冻，等待静态探测稳定后再按1。");
    }
    logic3_motion_candidate_ = false;
    logic3_motion_candidate_count_ = 0;
    logic3_start_zone_motion_pending_ = false;
    logic3_manual_freeze_active_ = true;
    logic3_manual_unfreeze_locked_ = false;
    freeze_logic3_route();
    response->success = true;
    response->message = "已人工冻结路线；发车将直接使用此静态避障路线。";
    return;
  }

  std::string reason;
  response->success = unfreeze_manual_logic3_route(reason);
  response->message = reason;
}

bool VisualLidarPathFollower::unfreeze_manual_logic3_route(std::string & reason)
{
  if (!logic3_planning_pose_anchored_) {
    reason = "路线当前未冻结，无需解冻。";
    return true;
  }
  if (!logic3_manual_freeze_active_) {
    reason = "当前路线由发车自动冻结，不能通过数字键2解除。";
    return false;
  }
  if (logic3_manual_unfreeze_locked_) {
    reason = "已检测到小车发车运动，为避免行驶中改变路线，拒绝解冻。";
    return false;
  }
  if (task_phase_ != TaskPhase::TASK1 || current_index_ != 0) {
    reason = "路线已进入任务流程，拒绝解冻。";
    return false;
  }
  if (logic3_baseline_original_route_.empty()) {
    reason = "缺少冻结前基线路线，拒绝解冻。";
    return false;
  }

  // 还原未偏移的基线路线，清除冻结偏移及运行期绘图状态；随后就绪期会
  // 继续用实时雷达更新 READY 静态快照，等待下一次人工或自动冻结。
  original_waypoints_ = logic3_baseline_original_route_;
  waypoints_ = logic3_baseline_original_route_;
  current_index_ = 0;
  reset_waypoint_shift_state();
  logic3_green_clusters_.clear();
  logic3_frozen_base_offsets_.clear();
  logic3_frozen_base_shift_directions_.clear();
  logic3_frozen_base_source_indices_.clear();
  logic3_baseline_route_.clear();
  logic3_visualization_route_.clear();
  task1_frozen_route_.clear();
  task1_dynamic_original_route_.clear();
  task1_dynamic_route_.clear();
  task1_trajectory_.clear();
  full_trajectory_.clear();
  logic3_runtime_cluster_tracks_.clear();
  logic3_red_cluster_history_.clear();
  logic3_final_red_history_begin_index_ = 0;
  logic3_replay_frozen_clusters_ = false;
  logic3_force_full_route_ = false;
  logic3_force_confirm_ = false;
  logic3_planning_pose_anchored_ = false;
  logic3_manual_freeze_active_ = false;
  logic3_manual_unfreeze_locked_ = false;
  logic3_motion_candidate_ = false;
  logic3_motion_candidate_count_ = 0;
  logic3_start_zone_motion_pending_ = false;
  logic3_motion_pending_snapshot_ = Logic3PlanningSnapshot();
  logic3_odom_reference_valid_ = false;
  logic3_map_odom_plan_valid_ = false;
  logic3_session_id_.clear();
  task1_departure_timer_started_ = false;
  task1_endpoint_waiting_ = false;
  publish_debug_route();
  reason = "已解冻并恢复静态READY快照更新；可等待稳定后再次按1冻结。";
  RCLCPP_WARN(get_logger(), "%s", reason.c_str());
  return true;
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
  if (snapshot.valid) {
    for (const auto & obstacle : snapshot.obstacles) {
      logic3_green_clusters_.push_back(obstacle.cluster);
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

  logic3_planning_pose_anchored_ = true;
  logic3_motion_candidate_ = false;
  logic3_start_zone_motion_pending_ = false;
  logic3_runtime_cluster_tracks_.clear();
  logic3_red_cluster_history_.clear();
  logic3_final_red_history_begin_index_ = 0;
  logic3_final_snapshot_published_ = false;
  logic3_final_pose_valid_ = false;
  logic3_session_id_ = make_logic3_session_id();
  logic3_last_pose_correction_at_ = std::chrono::steady_clock::now();
  task1_departure_timer_started_ = true;
  task1_departure_started_at_ = std::chrono::steady_clock::now();
  task1_qr_timeout_logged_ = false;
  task1_endpoint_fallback_logged_ = false;
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
    "开始Task1计时，输出批次=%s。",
    logic3_green_clusters_.size(),
    logic3_session_id_.c_str());

  // 可信发车确认后，立即把最后一次READY绿色快照应用到Task1路径。
  if (task_phase_ == TaskPhase::TASK1 && !waypoints_.empty()) {
    apply_logic3_frozen_clusters_to_current_route("task1_frozen");
  }
}

void VisualLidarPathFollower::apply_logic3_frozen_clusters_to_current_route(
  const std::string & snapshot_event)
{
  if (!logic3_planning_pose_anchored_) {
    return;
  }
  logic3_frozen_base_offsets_.assign(waypoints_.size(), 0.0);
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
  // 静态回放只留下冻结基础偏移；清空规划确认计数。运行期红点只记录，
  // 不再读取这些计数，也不会修改冻结路线。
  std::fill(waypoint_shift_pending_offsets_.begin(), waypoint_shift_pending_offsets_.end(), 0.0);
  std::fill(waypoint_shift_pending_counts_.begin(), waypoint_shift_pending_counts_.end(), 0);
  std::fill(waypoint_shift_missing_counts_.begin(), waypoint_shift_missing_counts_.end(), 0);
  std::fill(
    waypoint_shift_dynamic_directions_.begin(),
    waypoint_shift_dynamic_directions_.end(), 0);
  logic3_baseline_original_route_ = original_waypoints_;
  logic3_baseline_route_ = waypoints_;
  logic3_visualization_route_ = waypoints_;
  logic3_runtime_cluster_tracks_.clear();
  logic3_final_snapshot_published_ = false;
  logic3_final_pose_valid_ = false;
  if (snapshot_event == "task1_frozen") {
    task1_original_route_ = logic3_baseline_original_route_;
    task1_frozen_route_ = logic3_baseline_route_;
    task1_dynamic_original_route_.clear();
    task1_dynamic_route_.clear();
    task1_trajectory_.clear();
    full_trajectory_.clear();
    last_trajectory_sample_at_ = std::chrono::steady_clock::now();
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

void VisualLidarPathFollower::publish_logic3_snapshot(const std::string & event)
{
  if (!logic3_snapshot_pub_ || logic3_session_id_.empty()) {
    return;
  }
  const bool task1_event = event == "task1_frozen" ||
    event == "task1_dynamic";
  const bool dynamic_event = event == "task1_dynamic" ||
    event == "final_dynamic";
  const bool task1_dynamic_event = event == "task1_dynamic";
  const std::vector<Waypoint> & original_route =
    task1_dynamic_event && !task1_dynamic_original_route_.empty() ?
    task1_dynamic_original_route_ :
    (task1_event ? task1_original_route_ : final_original_route_);
  const std::vector<Waypoint> & frozen_route =
    task1_dynamic_event && !task1_dynamic_route_.empty() ?
    task1_dynamic_route_ :
    (task1_event ? task1_frozen_route_ : final_frozen_route_);
  // 04图严格以最新03冻结路线为基线，只额外叠加红色历史。
  const std::vector<Waypoint> & plotted_route = frozen_route;
  const bool include_qr_marker =
    task1_transition_marker_valid_ &&
    (event == "task1_dynamic" || event == "final_frozen" ||
    event == "final_dynamic");
  const bool include_route_start_marker =
    final_frozen_marker_valid_ &&
    (event == "task1_dynamic" || event == "final_frozen" ||
    event == "final_dynamic");
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
  if (event == "task1_dynamic") {
    // 02图保留Task1期间积累的完整红簇历史。
    plotted_red_clusters = logic3_red_cluster_history_;
  } else if (event == "final_dynamic") {
    // 04图只画Task1正式结束后新加入全局身份历史的红簇。
    // 全局历史本身不清空，因此跨阶段0.65m身份去重仍然有效。
    const std::size_t begin_index = std::min(
      logic3_final_red_history_begin_index_,
      logic3_red_cluster_history_.size());
    plotted_red_clusters.assign(
      logic3_red_cluster_history_.begin() +
      static_cast<std::ptrdiff_t>(begin_index),
      logic3_red_cluster_history_.end());
  }

  std::ostringstream json;
  json << std::setprecision(10);
  json << "{\"event\":\"" << json_escape(event)
       << "\",\"session\":\"" << json_escape(logic3_session_id_)
       << "\",\"frozen_revision\":" << channel_completed_reverse_count_
       << ",\"output_root\":\"" << json_escape(logic3_output_root_)
       << "\",\"map_yaml\":\"" << json_escape(logic3_restriction_map_yaml_)
       << "\",\"red_identity_radius\":" << logic3_red_history_merge_radius_
       << ",\"green_radius\":" << logic3_old_cluster_radius_
       << ",\"base_zone\":{\"min_x\":" << return_base_min_x_
       << ",\"max_x\":" << return_base_max_x_
       << ",\"min_y\":" << return_base_min_y_
       << ",\"max_y\":" << return_base_max_y_ << '}'
       << ",\"parking_zone\":{\"min_x\":" << return_stop_zone_min_x_
       << ",\"max_x\":" << return_stop_zone_max_x_
       << ",\"min_y\":" << return_stop_zone_min_y_
       << ",\"max_y\":" << return_stop_zone_max_y_
       << ",\"confirm_seconds\":" << return_stop_confirm_seconds_ << '}'
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
         << final_frozen_marker_.y << "],\"label\":\""
         << json_escape(final_frozen_marker_label_) << "\"}";
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
  json << ",\"green\":";
  append_clusters(json, logic3_green_clusters_);
  json << ",\"red\":";
  append_points(json, dynamic_event ? plotted_red_clusters : std::vector<Point2D>());
  json << ",\"trajectory\":";
  if (task1_dynamic_event) {
    append_points(json, task1_trajectory_);
  } else if (event == "final_dynamic") {
    append_points(json, full_trajectory_);
  } else {
    json << "[]";
  }
  json << '}';

  std_msgs::msg::String message;
  message.data = json.str();
  logic3_snapshot_pub_->publish(message);
}

void VisualLidarPathFollower::record_trajectory_sample(
  const VisualLidarPathFollower::RobotPose & pose)
{
  const auto steady_now = std::chrono::steady_clock::now();
  const double elapsed = std::chrono::duration<double>(
    steady_now - last_trajectory_sample_at_).count();
  if (elapsed < 0.1) {
    return;
  }
  last_trajectory_sample_at_ = steady_now;
  full_trajectory_.push_back(Point2D{pose.x, pose.y});
  if (task_phase_ == TaskPhase::TASK1) {
    task1_trajectory_.push_back(Point2D{pose.x, pose.y});
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

// 静态+动态方案 waypoint_shift：每帧新雷达提前检查当前目标之后的固定数量航点。
// 以障碍物最近路线点为中心，前后分别取配置数量的点并横移。
// 静态绿色障碍使路线向其相对位置的反侧绕行。
// 横移量先作为本帧候选，连续确认后才提交；障碍消失或错误观测消失后会释放回原路线。
void VisualLidarPathFollower::update_waypoint_shifts(
  const VisualLidarPathFollower::RobotPose & robot_pose,
  std::size_t target_waypoint_index)
{
  const bool replay_frozen_clusters = logic3_replay_frozen_clusters_;
  if ((!latest_scan_ && !replay_frozen_clusters) ||
    (processed_waypoint_shift_scan_sequence_ == scan_sequence_ && !replay_frozen_clusters) ||
    original_waypoints_.size() != waypoints_.size() ||
    waypoint_shift_offsets_.size() != waypoints_.size() ||
    waypoint_shift_pending_offsets_.size() != waypoints_.size() ||
    waypoint_shift_pending_counts_.size() != waypoints_.size() ||
    waypoint_shift_missing_counts_.size() != waypoints_.size() ||
    waypoint_shift_dynamic_directions_.size() != waypoints_.size() ||
    waypoint_shift_world_directions_.size() != waypoints_.size() ||
    waypoint_shift_source_indices_.size() != waypoints_.size() || waypoints_.empty())
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

  const int minimum_cluster_points = !logic3_planning_pose_anchored_ ?
    logic3_static_cluster_min_points_ : cluster_min_points_;
  const bool use_logic3_static_restriction_map =
    !logic3_planning_pose_anchored_ && !replay_frozen_clusters;
  const bool filter_logic3_green_mask =
    logic3_planning_pose_anchored_ && !replay_frozen_clusters;
  std::vector<ScanPointCluster> clusters;
  if (replay_frozen_clusters) {
    clusters = logic3_green_clusters_;
  } else if (logic3_planning_pose_anchored_ || enable_static_avoidance_) {
    // 静态避障关闭时只禁止发车前绿色簇收集；可信发车后的运行期聚类仍要
    // 保留，用于动态红点确认和最终路线图片。实时控制仍由独立雷达开关控制。
    clusters = collect_non_static_scan_clusters_map(
      robot_pose, minimum_cluster_points, use_logic3_static_restriction_map,
      filter_logic3_green_mask);
  }

  // 远距离时同一障碍物的1~3个回波可能被无效量程切成多个单点段。
  // 仅在静态+动态方案静止阶段用独立的当前帧0.20m半径合并候选；跨帧ID关联
  // 仍使用logic3_static_cluster_match_radius，不使用绿色冻结圆。
  if (!logic3_planning_pose_anchored_ && !replay_frozen_clusters)
  {
    std::vector<ScanPointCluster> merged_clusters;
    for (const auto & cluster : clusters) {
      auto merged = std::find_if(
        merged_clusters.begin(), merged_clusters.end(),
        [&](const ScanPointCluster & candidate) {
          return std::hypot(
            candidate.center.x - cluster.center.x,
            candidate.center.y - cluster.center.y) <=
          logic3_static_same_frame_merge_radius_;
        });
      if (merged == merged_clusters.end()) {
        merged_clusters.push_back(cluster);
        continue;
      }
      merged->points.insert(
        merged->points.end(), cluster.points.begin(), cluster.points.end());
      merged->center = Point2D();
      for (const auto & point : merged->points) {
        merged->center.x += point.x;
        merged->center.y += point.y;
      }
      const double point_count = static_cast<double>(merged->points.size());
      merged->center.x /= point_count;
      merged->center.y /= point_count;
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

  // 静态+动态方案运行阶段只处理配置的车体中心ROI。冻结绿色圆已经
  // 在逐点聚类前过滤；这里保留动态画图Y下限及以上的圈外红点记录候选。
  if (logic3_planning_pose_anchored_ && !replay_frozen_clusters)
  {
    const double half_width = 0.5 * logic3_runtime_roi_width_;
    const double half_height = 0.5 * logic3_runtime_roi_height_;
    clusters.erase(
      std::remove_if(
        clusters.begin(), clusters.end(), [&](const ScanPointCluster & cluster) {
          if (std::abs(cluster.center.x - robot_pose.x) > half_width ||
          std::abs(cluster.center.y - robot_pose.y) > half_height)
          {
            return true;
          }
          return cluster.center.y < logic3_runtime_channel_min_y_;
        }),
      clusters.end());
  }

  if (logic3_planning_pose_anchored_ && !replay_frozen_clusters)
  {
    // Task3会有意让分界线下方的旧绿色障碍重新参与实时雷达避障，但
    // “参与控制”不等于“获得新红簇身份”。红点历史必须始终以完整的
    // 冻结绿色集合做身份排除，否则同一障碍会在最终图上同时出现绿圈
    // 和红圈。这里只过滤绘图/红簇跟踪，不改变前面的实时避障点集。
    const double green_identity_radius_squared =
      logic3_old_cluster_radius_ * logic3_old_cluster_radius_;
    clusters.erase(
      std::remove_if(
        clusters.begin(), clusters.end(),
        [&](const ScanPointCluster & cluster) {
          return std::any_of(
            logic3_green_clusters_.begin(), logic3_green_clusters_.end(),
            [&](const ScanPointCluster & green_cluster) {
              if ((following_qr_path_ || task_phase_ == TaskPhase::TASK1) &&
                !task1_static_avoid_region_contains(green_cluster.center))
              {
                return false;
              }
              const double dx = cluster.center.x - green_cluster.center.x;
              const double dy = cluster.center.y - green_cluster.center.y;
              return dx * dx + dy * dy <= green_identity_radius_squared;
            });
        }),
      clusters.end());
    // 运行期圈外点只更新红点记录。路径保持扫码后冻结的绿色规划结果，
    // 实时控制由 control_once 中的雷达避障直接处理。
    update_logic3_runtime_red_tracks(clusters);
    return;
  }

  // Task1只让静态矩形内的冻结绿色簇参与静态路径偏移；矩形外的冻结
  // 绿色簇不偏移，并由上面的绿圈放行逻辑交给实时动态雷达。
  if (replay_frozen_clusters &&
    (following_qr_path_ || task_phase_ == TaskPhase::TASK1))
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

      auto required_offset_along = [&](const Point2D & shift_normal) {
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
          right_required_offset, preferred_direction, left_normal});
    };

  if (!logic3_planning_pose_anchored_ && !replay_frozen_clusters)
  {
    // 第一条发车候选已经保存READY平均快照后，停止接收后续可能包含底盘
    // 震动的静态帧；候选若取消，下一帧会恢复观察并按连续性规则重建。
    if (logic3_motion_candidate_ || logic3_start_zone_motion_pending_) {
      return;
    }
    // 新channel路线会从扫码/倒车后的base_link向正式首点延长，因此此时
    // 不能再拿旧channel路线预筛大厅静态簇。保留原ID、稳定帧和平均方法，
    // 规划阶段再由新路线的检查半径决定哪些绿色簇真正产生偏移。
    if (enable_static_avoidance_) {
      update_logic3_static_cluster_tracks(clusters, robot_pose);
    }
    const auto ready_obstacles = logic3_ready_static_obstacles();
    if (logic3_static_ready_ && !ready_obstacles.empty()) {
      // 只在完整READY状态下刷新。可信发车一旦出现运动预兆，静态更新会
      // 先暂停，因此这里保存的始终是运动前最后一次完整READY快照。
      logic3_last_complete_ready_snapshot_.valid = true;
      logic3_last_complete_ready_snapshot_.stamp_seconds = now().seconds();
      logic3_last_complete_ready_snapshot_.obstacles = ready_obstacles;
    }
    maybe_log_logic3_static_obstacles(clusters.size(), ready_obstacles);
    // 未扫码时只记录绿色障碍身份和点云，不改动任何正式路径点或偏移方向。
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
      // 分界高度只给“整个障碍簇”选择处理器，不能拿它裁剪路线点。
      // 因此这里只阻止下方障碍匹配返程段；上方障碍一旦匹配成功，后续
      // influence_first..influence_last仍可完整跨过return_obstacle_split_y。
      const bool return_lidar_only =
        clusters[cluster_index].center.y <= return_obstacle_split_y_;
      const std::size_t cluster_route_first = first_index;
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
      if (debug_control_logs_) {
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

  // 静态+动态方案按路线进度依次应用每个局部命中段。相邻障碍物窗口重叠时，
  // 前1/4保留前一个，后3/4由后一个覆盖。
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
  std::vector<bool> observed_geometry_metadata_valid(waypoints_.size(), false);
  {
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
          // 整数除法将不能四等分的余数交给后一个障碍物。
          applied_first = overlap_first + overlap_count / 4;
        }
      }
      std::size_t overwritten_count = 0;
      for (std::size_t index = applied_first; index <= segment.last; ++index) {
        if (observed_offsets_valid[index]) {
          ++overwritten_count;
        }
        // 候选已按路线进度排序；后一个障碍物从重叠区1/4位置开始取得控制权。
        observed_offsets[index] = candidate;
        observed_offsets_valid[index] = true;
        observed_world_directions[index] = segment.preferred_direction > 0 ?
          segment.left_normal : Point2D{-segment.left_normal.x, -segment.left_normal.y};
        observed_source_indices[index] = segment.obstacle_waypoint;
        observed_geometry_metadata_valid[index] = true;
      }
      previous_active_segment = &segment;
      if (debug_control_logs_) {
        RCLCPP_INFO(
          get_logger(),
          "debug: 静态+动态方案四分之一覆盖：障碍点=%zu，原窗口=%zu-%zu，实际写入=%zu-%zu，"
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
  }
  // 延长后的路线末点是不可移动的方向目标，不属于任何障碍物的偏移窗口。
  // 上方障碍的偏移尾段可以一直延伸到低于分界线，但必须在固定终点前
  // 自然回零，不能把末点本身横移。
  if (task3_route_available_ && !observed_offsets_valid.empty()) {
    observed_offsets_valid.back() = false;
    observed_geometry_metadata_valid.back() = false;
    observed_offsets.back() = 0.0;
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
      // 0.02m航点下允许来源点在2点内轻微抖动；方向变化约2.6度以上才重建。
      return source_delta > 2 || direction_alignment < 0.999;
    };

  std::size_t confirmed_count = 0;
  std::size_t released_count = 0;
  for (std::size_t index = first_index; index <= managed_last_index; ++index) {
    // 拍照扇形区域和插入的精确拍照锚点属于视觉任务的固定几何，禁止被
    // 静态 waypoint_shift 或运行中同一套偏移状态横向移动。
    if (is_image_capture_protected_waypoint(index)) {
      waypoint_shift_pending_offsets_[index] = 0.0;
      waypoint_shift_pending_counts_[index] = 0;
      waypoint_shift_missing_counts_[index] = 0;
      waypoint_shift_dynamic_directions_[index] = 0;
      if (std::abs(waypoint_shift_offsets_[index]) > 1e-9) {
        waypoint_shift_offsets_[index] = 0.0;
        waypoint_shift_world_directions_[index] = Point2D();
        waypoint_shift_source_indices_[index] = waypoints_.size();
        route_changed = true;
        ++released_count;
      }
      continue;
    }
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
          }
          route_changed = true;
          ++confirmed_count;
        } else if (std::abs(waypoint_shift_offsets_[index]) > 1e-9 &&
          observed_geometry_changed(index))
        {
          waypoint_shift_world_directions_[index] = observed_world_directions[index];
          waypoint_shift_source_indices_[index] = observed_source_indices[index];
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
      if (std::abs(waypoint_shift_offsets_[index] - base_offset) > 1e-9) {
        waypoint_shift_offsets_[index] = base_offset;
        if (std::abs(base_offset) > 1e-9 &&
          logic3_frozen_base_shift_directions_.size() == waypoints_.size() &&
          logic3_frozen_base_source_indices_.size() == waypoints_.size())
        {
          waypoint_shift_world_directions_[index] =
            logic3_frozen_base_shift_directions_[index];
          waypoint_shift_source_indices_[index] =
            logic3_frozen_base_source_indices_[index];
        } else {
          waypoint_shift_world_directions_[index] = Point2D();
          waypoint_shift_source_indices_[index] = waypoints_.size();
        }
        route_changed = true;
        ++released_count;
      }
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
    rebuild_curvature_unsafe_shift_segments(waypoints_);
    // 曲率不安全段的 Bezier 重建可能跨过已锁定的边界点；重建完成后再次
    // 还原拍照扇形内的原始点，确保该区域的实际路径几何绝不参与平移。
    for (std::size_t index = first_index; index <= managed_last_index; ++index) {
      if (is_image_capture_protected_waypoint(index)) {
        waypoints_[index] = original_waypoints_[index];
      }
    }
    if (following_qr_path_ || task_phase_ == TaskPhase::TASK1) {
      for (auto & waypoint : waypoints_) {
        waypoint.y = std::max(waypoint.y, task1_waypoint_shift_min_y_);
      }
    }
    if (channel_entry_waypoint_index_ > 0 && !waypoints_.empty()) {
      waypoints_.front() = original_waypoints_.front();
    }
    if (task3_route_available_ && !waypoints_.empty()) {
      waypoint_shift_offsets_.back() = 0.0;
      waypoint_shift_world_directions_.back() = Point2D();
      waypoint_shift_source_indices_.back() = waypoints_.size();
      waypoints_.back() = original_waypoints_.back();
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
    // 二维码可在x门槛前先识别并缓存；路线切换必须等可信发车已冻结，
    // 且base_link真正驶过大厅半场门槛后才生效。
    complete_task1_and_select_route(direction, "识别到有效二维码", true);
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

  // 第一份有效二维码的实际识别位置单独保存。即使当时尚未可信发车或
  // 尚未越过x门槛，也不把之后真正切换路线的位置冒充二维码位置。
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
    selection_pose.x > task1_transition_min_x_;
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
          "Task1方向已缓存：%s；等待base_link.x>%.2fm后再进入过渡阶段"
          "（当前x=%.3f）。",
          route_direction_name(task1_pending_direction_).c_str(),
          task1_transition_min_x_, selection_pose.x);
      } else {
        RCLCPP_INFO(
          get_logger(),
          "Task1方向已缓存：%s；等待可信发车且base_link.x>%.2fm后再进入过渡阶段。",
          route_direction_name(task1_pending_direction_).c_str(),
          task1_transition_min_x_);
      }
      task1_pending_gate_logged_ = true;
    }
    return false;
  }

  // 02图与01图使用完全相同的完整Task1原始/冻结路线；02额外叠加
  // 红点历史、实际二维码位置，以及正式进入Task1→Task2过渡时的
  // base_link蓝色发车点，不再裁成已经行驶的前缀。默认顺时针没有
  // 真实二维码坐标，因此不伪造橙色点。
  task1_dynamic_original_route_ = task1_original_route_;
  task1_dynamic_route_ = task1_frozen_route_;
  if (!switch_route(direction)) {
    return false;
  }

  qr_route_selected_ = true;
  task1_endpoint_waiting_ = false;
  task1_pending_direction_.clear();
  task1_pending_reason_.clear();
  task1_pending_from_qr_ = false;
  task1_pending_gate_logged_ = false;
  // 此处就是Task1正式结束、进入下一阶段的唯一边界。02图的蓝色
  // 发车点必须记录这一刻的base_link；下一控制周期重建A→B路线时，
  // 03图会再以实际重建使用的A点刷新同一标记。
  final_frozen_marker_ = Point2D{selection_pose.x, selection_pose.y};
  final_frozen_marker_label_ = "ROUTE START";
  final_frozen_marker_valid_ = true;
  // 02图到此收口；04图从下一条新红簇身份开始绘制。不能清空全局历史，
  // 否则Task1见过的障碍会在后续阶段被再次识别为新红簇。
  logic3_final_red_history_begin_index_ =
    logic3_red_cluster_history_.size();
  set_task_phase(TaskPhase::TRANSITION);
  publish_logic3_snapshot("task1_dynamic");
  if (qr_sub_) {
    qr_sub_.reset();
  }
  RCLCPP_INFO(
    get_logger(),
    "Task1结束：%s；选择%s路线并进入过渡阶段。",
    reason_text.c_str(),
    route_direction_name(direction).c_str());
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

// 以当前map->base_link位置为A，以原始waypoint_channel首点为B，按channel
// 中位点距在AB直线上插点。B处的轻微圆滑只修改这条“偏移前基准路线”；
// 基准路线固化后才回放冻结绿色静态簇，完全交给原waypoint_shift算法偏移，
// 偏移完成后不再针对B点做任何额外拟合或回拉。
bool VisualLidarPathFollower::rebuild_channel_entry_route(
  const VisualLidarPathFollower::RobotPose & start_pose, const char * reason)
{
  if (channel_route_template_.empty() ||
    !std::isfinite(start_pose.x) || !std::isfinite(start_pose.y))
  {
    return false;
  }
  const bool rebuild_after_reverse = channel_entry_route_rebuild_after_reverse_;

  const auto & channel_entry = channel_route_template_.front();
  const double entry_distance = std::hypot(
    channel_entry.x - start_pose.x, channel_entry.y - start_pose.y);
  const double spacing = std::max(
    1.0e-3, estimate_channel_waypoint_spacing(channel_route_template_));
  const std::size_t extension_point_count = entry_distance > 1.0e-6 ?
    std::max<std::size_t>(
    1, static_cast<std::size_t>(std::llround(entry_distance / spacing))) : 0;

  std::vector<Waypoint> extended_route;
  extended_route.reserve(channel_route_template_.size() + extension_point_count + 1);
  for (std::size_t index = 0; index < extension_point_count; ++index) {
    const double ratio = static_cast<double>(index) /
      static_cast<double>(extension_point_count);
    Waypoint extension = channel_entry;
    extension.name = "channel_entry_extension_" + std::to_string(index + 1);
    extension.x = start_pose.x + ratio * (channel_entry.x - start_pose.x);
    extension.y = start_pose.y + ratio * (channel_entry.y - start_pose.y);
    extension.reverse = false;
    extension.image_capture = false;
    extension.pass_radius = pass_radius_;
    extension.reverse_pass_radius = reverse_pass_radius_;
    extended_route.push_back(extension);
  }
  channel_entry_waypoint_index_ = extended_route.size();
  // 默认与B重合，表示还没有生成需要保护的偏移前圆滑段。
  channel_entry_smoothing_first_index_ = channel_entry_waypoint_index_;
  extended_route.insert(
    extended_route.end(), channel_route_template_.begin(), channel_route_template_.end());

  // 阶段1（偏移前）：A→B主体保持直线，只把靠近B的末段改造成
  // 等弧长三次Bezier。起点切线沿AB，终点切线沿channel首段；
  // 路线仍精确经过B，但不会在B处突然折向。
  if (channel_entry_waypoint_index_ >= 2 &&
    channel_route_template_.size() >= 2 &&
    channel_entry_corner_smoothing_length_ > spacing)
  {
    const Point2D line_vector{
      channel_entry.x - start_pose.x, channel_entry.y - start_pose.y};
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
          channel_entry_corner_smoothing_length_, entry_distance);
        const std::size_t blend_segments = std::min(
          channel_entry_waypoint_index_,
          std::max<std::size_t>(
            2, static_cast<std::size_t>(
              std::llround(requested_blend_length / spacing))));
        const std::size_t anchor_index =
          channel_entry_waypoint_index_ - blend_segments;
        channel_entry_smoothing_first_index_ = anchor_index;
        const Point2D start{
          extended_route[anchor_index].x, extended_route[anchor_index].y};
        const Point2D finish{channel_entry.x, channel_entry.y};
        const double chord_length = std::hypot(
          finish.x - start.x, finish.y - start.y);
        const double handle_length = chord_length / 3.0;
        const Point2D control1{
          start.x + handle_length * line_tangent.x,
          start.y + handle_length * line_tangent.y};
        const Point2D control2{
          finish.x - handle_length * channel_tangent.x,
          finish.y - handle_length * channel_tangent.y};
        auto bezier_point = [&](double t) {
            const double one_minus_t = 1.0 - t;
            const double start_weight =
              one_minus_t * one_minus_t * one_minus_t;
            const double control1_weight =
              3.0 * one_minus_t * one_minus_t * t;
            const double control2_weight =
              3.0 * one_minus_t * t * t;
            const double finish_weight = t * t * t;
            return Point2D{
              start_weight * start.x + control1_weight * control1.x +
              control2_weight * control2.x + finish_weight * finish.x,
              start_weight * start.y + control1_weight * control1.y +
              control2_weight * control2.y + finish_weight * finish.y};
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
          "A→B偏移前圆滑：在B前%.3fm内使用%zu段等弧长Bezier，"
          "转角=%.1f°；偏移后不再针对B拟合。",
          curve_length, blend_segments,
          std::acos(tangent_alignment) * 180.0 / M_PI);
      }
    }
  }
  waypoints_ = std::move(extended_route);

  insert_image_capture_waypoint(selected_route_direction_);
  if (has_image_capture_waypoint_ &&
    image_capture_waypoint_index_ <= channel_entry_waypoint_index_)
  {
    if (image_capture_waypoint_index_ <= channel_entry_smoothing_first_index_) {
      ++channel_entry_smoothing_first_index_;
    }
    ++channel_entry_waypoint_index_;
  }
  // 阶段边界：从这里开始，original_waypoints_保存的就是已在B处圆滑的
  // 偏移前基准路线。后续只允许通用waypoint_shift读取该基准路线并偏移。
  reset_waypoint_shift_state();
  current_index_ = 0;
  first_waypoint_reached_ = false;
  image_analysis_triggered_ = false;
  channel_entry_route_rebuild_pending_ = false;
  channel_entry_route_rebuild_after_reverse_ = false;

  if (logic3_planning_pose_anchored_) {
    final_frozen_marker_ = Point2D{start_pose.x, start_pose.y};
    if (rebuild_after_reverse) {
      final_frozen_marker_label_ =
        "REPLAN " + std::to_string(channel_completed_reverse_count_);
    } else {
      final_frozen_marker_label_ = "ROUTE START";
    }
    final_frozen_marker_valid_ = true;
    apply_logic3_frozen_clusters_to_current_route("final_frozen");
    // 二维码内容只在第一次正式进场的蓝色起点和03图均已更新后播报。
    // 倒车重规划只刷新蓝色点和03图，不重复语音。
    if (!rebuild_after_reverse) {
      publish_pending_qr_speech();
    }
  }
  // 即使冻结绿色簇为空、偏移量没有变化，倒车后的A点也已经改变，
  // 所以必须无条件刷新调试路线。
  publish_debug_route();

  RCLCPP_WARN(
    get_logger(),
    "%s：以base_link A=(%.3f, %.3f)直连waypoint_channel首点B=(%.3f, %.3f)，"
    "AB=%.3fm，channel中位点距=%.4fm，新增%zu个等间距点；"
    "%s。",
    reason ? reason : "channel进场重建", start_pose.x, start_pose.y,
    channel_entry.x, channel_entry.y, entry_distance, spacing,
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
  has_image_capture_waypoint_ = false;
  image_capture_stopping_ = false;
  image_capture_recovery_active_ = false;
  image_capture_recovery_used_ = false;
  reset_waypoint_shift_state();
  channel_entry_route_rebuild_pending_ = true;
  channel_entry_route_rebuild_after_reverse_ = false;
  waiting_for_qr_start_ = false;
  following_qr_path_ = false;
  qr_path_finished_ = false;
  image_analysis_triggered_ = false;
  if (was_following_qr_path || was_waiting_for_qr) {
    current_index_ = 0;
    first_waypoint_reached_ = false;
    qr_channel_approach_active_ = true;
    qr_escape_state_ = qr_reverse_max_count_ > 0 ?
      QrEscapeState::PENDING : QrEscapeState::IDLE;
  } else {
    current_index_ = std::min(current_index_, waypoints_.size() - 1);
    first_waypoint_reached_ = current_index_ > 0;
  }
  const auto route_name = route_direction_name(selected_route_direction_);
  RCLCPP_INFO(
    get_logger(),
    "QR 选择%s路线；下一控制周期先从当前base_link直连waypoint_channel首点并等距插点，"
    "再按冻结静态坐标规划；每轮条件倒车结束后还会从新base_link重复一次；"
    "静态+动态方案在 channel 全程使用运行限制图和绿圈掩码后的雷达避障。",
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
