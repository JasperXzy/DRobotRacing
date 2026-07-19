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
    "turn_speed_full_steering_angle", turn_speed_full_steering_angle_);
  get_parameter(
    "turn_speed_full_steering_angle", turn_speed_full_steering_angle_);
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

  // 逻辑2全部使用 logic2_ 前缀，waypoint_shift_logic=1 时以下参数不参与控制。
  declare_parameter<int>("waypoint_shift_logic", waypoint_shift_logic_);
  get_parameter("waypoint_shift_logic", waypoint_shift_logic_);
  declare_parameter<std::string>("logic2_odom_topic", logic2_odom_topic_);
  get_parameter("logic2_odom_topic", logic2_odom_topic_);
  declare_parameter<std::string>("logic2_odom_frame_id", logic2_odom_frame_id_);
  get_parameter("logic2_odom_frame_id", logic2_odom_frame_id_);
  declare_parameter<std::string>(
    "logic2_planning_pose_source", logic2_planning_pose_source_);
  get_parameter("logic2_planning_pose_source", logic2_planning_pose_source_);
  declare_parameter<std::string>("logic2_output_root", logic2_output_root_);
  get_parameter("logic2_output_root", logic2_output_root_);
  declare_parameter<std::string>(
    "logic2_restriction_map_yaml", logic2_restriction_map_yaml_);
  get_parameter("logic2_restriction_map_yaml", logic2_restriction_map_yaml_);
  declare_parameter<double>(
    "logic2_motion_speed_threshold", logic2_motion_speed_threshold_);
  get_parameter("logic2_motion_speed_threshold", logic2_motion_speed_threshold_);
  declare_parameter<double>(
    "logic2_motion_displacement_threshold", logic2_motion_displacement_threshold_);
  get_parameter(
    "logic2_motion_displacement_threshold", logic2_motion_displacement_threshold_);
  declare_parameter<int>(
    "logic2_motion_confirm_messages", logic2_motion_confirm_messages_);
  get_parameter("logic2_motion_confirm_messages", logic2_motion_confirm_messages_);
  declare_parameter<double>("logic2_runtime_roi_width", logic2_runtime_roi_width_);
  get_parameter("logic2_runtime_roi_width", logic2_runtime_roi_width_);
  declare_parameter<double>("logic2_runtime_roi_height", logic2_runtime_roi_height_);
  get_parameter("logic2_runtime_roi_height", logic2_runtime_roi_height_);
  declare_parameter<double>(
    "logic2_red_history_merge_radius", logic2_red_history_merge_radius_);
  get_parameter("logic2_red_history_merge_radius", logic2_red_history_merge_radius_);
  declare_parameter<double>(
    "logic2_pose_correction_deadband_xy", logic2_pose_correction_deadband_xy_);
  get_parameter("logic2_pose_correction_deadband_xy", logic2_pose_correction_deadband_xy_);
  declare_parameter<double>(
    "logic2_pose_correction_deadband_yaw", logic2_pose_correction_deadband_yaw_);
  get_parameter("logic2_pose_correction_deadband_yaw", logic2_pose_correction_deadband_yaw_);
  declare_parameter<double>(
    "logic2_pose_correction_rate_xy", logic2_pose_correction_rate_xy_);
  get_parameter("logic2_pose_correction_rate_xy", logic2_pose_correction_rate_xy_);
  declare_parameter<double>(
    "logic2_pose_correction_rate_yaw", logic2_pose_correction_rate_yaw_);
  get_parameter("logic2_pose_correction_rate_yaw", logic2_pose_correction_rate_yaw_);
  declare_parameter<double>("logic2_pose_outlier_xy", logic2_pose_outlier_xy_);
  get_parameter("logic2_pose_outlier_xy", logic2_pose_outlier_xy_);
  declare_parameter<double>("logic2_pose_outlier_yaw", logic2_pose_outlier_yaw_);
  get_parameter("logic2_pose_outlier_yaw", logic2_pose_outlier_yaw_);

  // 逻辑3参数完整独立；选择逻辑3后会复制到共用运行管线，不改写逻辑2配置。
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
  declare_parameter<std::string>("logic3_tts_topic", logic3_tts_topic_);
  get_parameter("logic3_tts_topic", logic3_tts_topic_);
  declare_parameter<std::string>(
    "logic3_static_record_ready_text", logic3_static_record_ready_text_);
  get_parameter("logic3_static_record_ready_text", logic3_static_record_ready_text_);
  declare_parameter<std::string>("logic3_route_ready_text", logic3_route_ready_text_);
  get_parameter("logic3_route_ready_text", logic3_route_ready_text_);
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
  declare_parameter<double>(
    "logic3_runtime_stability_duration_seconds",
    logic3_runtime_stability_duration_seconds_);
  get_parameter(
    "logic3_runtime_stability_duration_seconds",
    logic3_runtime_stability_duration_seconds_);
  declare_parameter<double>(
    "logic3_runtime_stability_radius", logic3_runtime_stability_radius_);
  get_parameter("logic3_runtime_stability_radius", logic3_runtime_stability_radius_);
  declare_parameter<double>(
    "logic3_runtime_offset_max_step", logic3_runtime_offset_max_step_);
  get_parameter("logic3_runtime_offset_max_step", logic3_runtime_offset_max_step_);
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
    "logic3_static_channel_min_y", logic3_static_channel_min_y_);
  get_parameter("logic3_static_channel_min_y", logic3_static_channel_min_y_);
  declare_parameter<double>(
    "logic3_entry_exit_overlap_max_y", logic3_entry_exit_overlap_max_y_);
  get_parameter(
    "logic3_entry_exit_overlap_max_y", logic3_entry_exit_overlap_max_y_);
  declare_parameter<double>(
    "logic3_static_cluster_match_radius", logic3_static_cluster_match_radius_);
  get_parameter("logic3_static_cluster_match_radius", logic3_static_cluster_match_radius_);
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
  declare_parameter<bool>("logic3_enable_tts", logic3_enable_tts_);
  get_parameter("logic3_enable_tts", logic3_enable_tts_);
  declare_parameter<double>(
    "logic3_static_status_log_rate_hz", logic3_static_status_log_rate_hz_);
  get_parameter(
    "logic3_static_status_log_rate_hz", logic3_static_status_log_rate_hz_);

  // 扫码后的 zhy 条件倒车。前后视野和走廊分别可调，检测区均为扇形与矩形的交集。
  declare_parameter<double>("qr_reverse_y_threshold", qr_reverse_y_threshold_);
  get_parameter("qr_reverse_y_threshold", qr_reverse_y_threshold_);
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

  // 倒车结束后到进入 channel 前，普通航点跟踪叠加 zhy 雷达避障。
  declare_parameter<double>(
    "qr_channel_handoff_waypoint_distance", qr_channel_handoff_waypoint_distance_);
  get_parameter(
    "qr_channel_handoff_waypoint_distance", qr_channel_handoff_waypoint_distance_);
  declare_parameter<double>(
    "qr_channel_lidar_lookahead_distance", qr_channel_lidar_lookahead_distance_);
  get_parameter(
    "qr_channel_lidar_lookahead_distance", qr_channel_lidar_lookahead_distance_);
  declare_parameter<double>(
    "qr_channel_lidar_cruise_linear_speed", qr_channel_lidar_cruise_linear_speed_);
  get_parameter(
    "qr_channel_lidar_cruise_linear_speed", qr_channel_lidar_cruise_linear_speed_);
  declare_parameter<double>(
    "qr_channel_lidar_turn_angular_gain", qr_channel_lidar_turn_angular_gain_);
  get_parameter(
    "qr_channel_lidar_turn_angular_gain", qr_channel_lidar_turn_angular_gain_);
  declare_parameter<double>(
    "qr_channel_lidar_min_turn_speed_percent",
    qr_channel_lidar_min_turn_speed_percent_);
  get_parameter(
    "qr_channel_lidar_min_turn_speed_percent",
    qr_channel_lidar_min_turn_speed_percent_);
  declare_parameter<double>(
    "qr_channel_lidar_avoid_distance", qr_channel_lidar_avoid_distance_);
  get_parameter(
    "qr_channel_lidar_avoid_distance", qr_channel_lidar_avoid_distance_);
  declare_parameter<double>(
    "qr_channel_lidar_avoid_fov_deg", qr_channel_lidar_avoid_fov_deg_);
  get_parameter(
    "qr_channel_lidar_avoid_fov_deg", qr_channel_lidar_avoid_fov_deg_);
  declare_parameter<double>(
    "qr_channel_lidar_corridor_half_width", qr_channel_lidar_corridor_half_width_);
  get_parameter(
    "qr_channel_lidar_corridor_half_width", qr_channel_lidar_corridor_half_width_);
  declare_parameter<double>(
    "qr_channel_lidar_avoid_linear_speed", qr_channel_lidar_avoid_linear_speed_);
  get_parameter(
    "qr_channel_lidar_avoid_linear_speed", qr_channel_lidar_avoid_linear_speed_);
  declare_parameter<double>(
    "qr_channel_lidar_avoid_angular_gain", qr_channel_lidar_avoid_angular_gain_);
  get_parameter(
    "qr_channel_lidar_avoid_angular_gain", qr_channel_lidar_avoid_angular_gain_);

  // 上述两段共用雷达新鲜度和物理地图墙体过滤，不使用 restriction_map 过滤。
  declare_parameter<double>(
    "qr_lidar_scan_timeout", qr_lidar_scan_timeout_);
  get_parameter(
    "qr_lidar_scan_timeout", qr_lidar_scan_timeout_);
  declare_parameter<bool>(
    "qr_lidar_enable_static_map_filter", qr_lidar_enable_static_map_filter_);
  get_parameter(
    "qr_lidar_enable_static_map_filter", qr_lidar_enable_static_map_filter_);
  declare_parameter<int>(
    "qr_lidar_static_map_occupied_threshold", qr_lidar_static_map_occupied_threshold_);
  get_parameter(
    "qr_lidar_static_map_occupied_threshold", qr_lidar_static_map_occupied_threshold_);
  declare_parameter<double>(
    "qr_lidar_static_map_wall_padding", qr_lidar_static_map_wall_padding_);
  get_parameter(
    "qr_lidar_static_map_wall_padding", qr_lidar_static_map_wall_padding_);

  // channel结束后的返回起点控制。视觉参数与当前visual_control实跑配置保持
  // 同名语义，但由本节点读取和执行，避免恢复visual_control后争抢/cmd_vel。
  declare_parameter<bool>("return_to_start_enabled", return_to_start_enabled_);
  get_parameter("return_to_start_enabled", return_to_start_enabled_);
  declare_parameter<double>("return_target_x", return_target_x_);
  get_parameter("return_target_x", return_target_x_);
  declare_parameter<double>("return_target_y", return_target_y_);
  get_parameter("return_target_y", return_target_y_);
  declare_parameter<double>("return_stop_zone_length", return_stop_zone_length_);
  get_parameter("return_stop_zone_length", return_stop_zone_length_);
  declare_parameter<double>(
    "return_stop_zone_upper_half_width", return_stop_zone_upper_half_width_);
  get_parameter(
    "return_stop_zone_upper_half_width", return_stop_zone_upper_half_width_);
  declare_parameter<double>(
    "return_stop_zone_lower_half_width", return_stop_zone_lower_half_width_);
  get_parameter(
    "return_stop_zone_lower_half_width", return_stop_zone_lower_half_width_);
  declare_parameter<double>(
    "return_follow_linear_speed", return_follow_linear_speed_);
  get_parameter("return_follow_linear_speed", return_follow_linear_speed_);
  declare_parameter<double>("return_turn_angular_gain", return_turn_angular_gain_);
  get_parameter("return_turn_angular_gain", return_turn_angular_gain_);
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
  turn_speed_full_steering_angle_ = std::max(
    0.01, std::abs(turn_speed_full_steering_angle_));
  max_turn_speed_reduction_percent_ = std::clamp(
    max_turn_speed_reduction_percent_, 0.0, 90.0);
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
  waypoint_shift_rebuild_min_progress_ratio_ = std::clamp(
    waypoint_shift_rebuild_min_progress_ratio_, -0.50, 0.90);
  waypoint_shift_rebuild_plateau_half_length_ = std::max(
    0.0, waypoint_shift_rebuild_plateau_half_length_);
  waypoint_shift_rebuild_bezier_handle_ratio_ = std::clamp(
    waypoint_shift_rebuild_bezier_handle_ratio_, 0.0, 0.45);
  waypoint_shift_points_before_ = std::max(0, waypoint_shift_points_before_);
  waypoint_shift_points_after_ = std::max(0, waypoint_shift_points_after_);
  waypoint_shift_confirm_scans_ = std::max(1, waypoint_shift_confirm_scans_);
  waypoint_shift_release_scans_ = std::max(1, waypoint_shift_release_scans_);
  waypoint_shift_min_change_ = std::max(0.0, waypoint_shift_min_change_);
  static_map_occupied_threshold_ = std::max(1, std::min(100, static_map_occupied_threshold_));
  static_map_inflation_cells_ = std::max(0, static_map_inflation_cells_);
  waypoint_shift_logic_ = waypoint_shift_logic_ == 2 || waypoint_shift_logic_ == 3 ?
    waypoint_shift_logic_ : 1;
  logic2_planning_pose_source_ = to_lower(logic2_planning_pose_source_);
  if (logic2_planning_pose_source_ != "live_lidar_location" &&
    logic2_planning_pose_source_ != "frozen_map_odom" &&
    logic2_planning_pose_source_ != "filtered_map_odom")
  {
    RCLCPP_WARN(
      get_logger(), "未知 logic2_planning_pose_source，改用 filtered_map_odom。");
    logic2_planning_pose_source_ = "filtered_map_odom";
  }
  logic2_motion_speed_threshold_ = std::max(0.001, logic2_motion_speed_threshold_);
  logic2_motion_displacement_threshold_ = std::max(
    0.001, logic2_motion_displacement_threshold_);
  logic2_motion_confirm_messages_ = std::max(1, logic2_motion_confirm_messages_);
  logic2_runtime_roi_width_ = std::max(0.10, logic2_runtime_roi_width_);
  logic2_runtime_roi_height_ = std::max(0.10, logic2_runtime_roi_height_);
  logic2_red_history_merge_radius_ = std::max(0.01, logic2_red_history_merge_radius_);
  logic2_pose_correction_deadband_xy_ = std::max(
    0.0, logic2_pose_correction_deadband_xy_);
  logic2_pose_correction_deadband_yaw_ = std::max(
    0.0, logic2_pose_correction_deadband_yaw_);
  logic2_pose_correction_rate_xy_ = std::max(0.0, logic2_pose_correction_rate_xy_);
  logic2_pose_correction_rate_yaw_ = std::max(0.0, logic2_pose_correction_rate_yaw_);
  logic2_pose_outlier_xy_ = std::max(0.01, logic2_pose_outlier_xy_);
  logic2_pose_outlier_yaw_ = std::max(0.01, logic2_pose_outlier_yaw_);

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
  logic3_old_cluster_radius_ = std::max(0.05, logic3_old_cluster_radius_);
  logic3_runtime_roi_width_ = std::max(0.10, logic3_runtime_roi_width_);
  logic3_runtime_roi_height_ = std::max(0.10, logic3_runtime_roi_height_);
  if (!std::isfinite(logic3_runtime_channel_min_y_)) {
    logic3_runtime_channel_min_y_ = 1.60;
  }
  // 场内障碍物中心间距至少1m；运行身份半径限制在0.49m以内，避免两个
  // 实体被关联成同一轨迹。0.75m红圈仍只用于最终图片的历史归并。
  logic3_runtime_cluster_match_radius_ = std::clamp(
    logic3_runtime_cluster_match_radius_, 0.05, 0.49);
  logic3_runtime_average_samples_ = std::max(1, logic3_runtime_average_samples_);
  logic3_runtime_stability_duration_seconds_ = std::max(
    0.0, logic3_runtime_stability_duration_seconds_);
  logic3_runtime_stability_radius_ = std::clamp(
    logic3_runtime_stability_radius_, 0.01,
    logic3_runtime_cluster_match_radius_);
  logic3_runtime_offset_max_step_ = std::max(
    0.0, logic3_runtime_offset_max_step_);
  logic3_static_cluster_min_points_ = std::max(1, logic3_static_cluster_min_points_);
  logic3_static_restriction_inflation_cells_ = std::max(
    0, logic3_static_restriction_inflation_cells_);
  if (!std::isfinite(logic3_static_channel_min_y_)) {
    logic3_static_channel_min_y_ = 1.60;
  }
  if (!std::isfinite(logic3_entry_exit_overlap_max_y_)) {
    logic3_entry_exit_overlap_max_y_ = 2.50;
  }
  logic3_entry_exit_overlap_max_y_ = std::max(
    logic3_static_channel_min_y_ + 0.01, logic3_entry_exit_overlap_max_y_);
  logic3_static_cluster_match_radius_ = std::max(
    0.01, logic3_static_cluster_match_radius_);
  logic3_static_confirmation_window_seconds_ = std::max(
    0.1, logic3_static_confirmation_window_seconds_);
  logic3_static_confirmation_min_hits_ = std::max(
    1, logic3_static_confirmation_min_hits_);
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

  if (waypoint_shift_logic_ == 3) {
    // 共用动态管线读取logic2_活动值；这里只复制选中的logic3配置。
    logic2_odom_topic_ = logic3_odom_topic_;
    logic2_odom_frame_id_ = logic3_odom_frame_id_;
    logic2_planning_pose_source_ = logic3_planning_pose_source_;
    logic2_output_root_ = logic3_output_root_;
    logic2_restriction_map_yaml_ = logic3_restriction_map_yaml_;
    logic2_motion_speed_threshold_ = logic3_motion_speed_threshold_;
    logic2_motion_displacement_threshold_ = logic3_motion_displacement_threshold_;
    logic2_motion_confirm_messages_ = logic3_motion_confirm_messages_;
    logic2_runtime_roi_width_ = logic3_runtime_roi_width_;
    logic2_runtime_roi_height_ = logic3_runtime_roi_height_;
    logic2_red_history_merge_radius_ = logic3_red_history_merge_radius_;
    logic2_pose_correction_deadband_xy_ = logic3_pose_correction_deadband_xy_;
    logic2_pose_correction_deadband_yaw_ = logic3_pose_correction_deadband_yaw_;
    logic2_pose_correction_rate_xy_ = logic3_pose_correction_rate_xy_;
    logic2_pose_correction_rate_yaw_ = logic3_pose_correction_rate_yaw_;
    logic2_pose_outlier_xy_ = logic3_pose_outlier_xy_;
    logic2_pose_outlier_yaw_ = logic3_pose_outlier_yaw_;
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
  qr_channel_handoff_waypoint_distance_ = std::max(
    0.01, qr_channel_handoff_waypoint_distance_);
  qr_channel_lidar_lookahead_distance_ = std::max(
    0.01, qr_channel_lidar_lookahead_distance_);
  qr_channel_lidar_cruise_linear_speed_ = std::max(
    0.0, qr_channel_lidar_cruise_linear_speed_);
  qr_channel_lidar_turn_angular_gain_ = std::max(
    0.0, qr_channel_lidar_turn_angular_gain_);
  qr_channel_lidar_min_turn_speed_percent_ = std::clamp(
    qr_channel_lidar_min_turn_speed_percent_, 0.0, 100.0);
  qr_channel_lidar_avoid_distance_ = std::max(0.0, qr_channel_lidar_avoid_distance_);
  qr_channel_lidar_avoid_fov_deg_ = std::clamp(
    qr_channel_lidar_avoid_fov_deg_, 1.0, 360.0);
  qr_channel_lidar_corridor_half_width_ = std::max(
    0.01, qr_channel_lidar_corridor_half_width_);
  qr_channel_lidar_avoid_linear_speed_ = std::max(
    0.0, qr_channel_lidar_avoid_linear_speed_);
  qr_channel_lidar_avoid_angular_gain_ = std::max(
    0.0, qr_channel_lidar_avoid_angular_gain_);
  qr_lidar_scan_timeout_ = std::max(0.05, qr_lidar_scan_timeout_);
  qr_lidar_static_map_occupied_threshold_ = std::clamp(
    qr_lidar_static_map_occupied_threshold_, 1, 100);
  qr_lidar_static_map_wall_padding_ = std::max(
    0.0, qr_lidar_static_map_wall_padding_);
  return_stop_zone_length_ = std::max(0.0, return_stop_zone_length_);
  return_stop_zone_upper_half_width_ = std::max(
    0.0, return_stop_zone_upper_half_width_);
  return_stop_zone_lower_half_width_ = std::max(
    0.0, return_stop_zone_lower_half_width_);
  return_follow_linear_speed_ = std::max(0.0, return_follow_linear_speed_);
  return_turn_angular_gain_ = std::max(0.0, return_turn_angular_gain_);
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
    if (waypoint_shift_logic_ == 3) {
      const std::string static_route_file = logic3_static_route_direction_ == "clockwise" ?
        route_waypoints_clockwise_file_ : route_waypoints_counterclockwise_file_;
      waypoints_ = load_waypoints(static_route_file, logic3_static_route_direction_);
      reset_waypoint_shift_state();
      if (waypoints_.empty()) {
        RCLCPP_ERROR(
          get_logger(), "逻辑3无法加载静止规划channel路线：%s。",
          static_route_file.c_str());
      }
    }
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
  if (waypoint_shift_logic_ >= 2) {
    logic2_snapshot_pub_ = create_publisher<std_msgs::msg::String>(
      "/visual_lidar_path_follower/route_snapshot",
      rclcpp::QoS(3).transient_local().reliable());
  }
  if (waypoint_shift_logic_ == 3 && logic3_enable_tts_) {
    logic3_tts_pub_ = create_publisher<std_msgs::msg::String>(logic3_tts_topic_, 10);
  }
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
  if (return_to_start_enabled_) {
    return_visual_detection_sub_ =
      create_subscription<ai_msgs::msg::PerceptionTargets>(
        return_visual_detection_topic_, rclcpp::SensorDataQoS().keep_last(1),
        [this](ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg) {
          latest_return_visual_targets_ = msg;
          latest_return_visual_received_at_ = std::chrono::steady_clock::now();
        });
  }
  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic_, rclcpp::QoS(1).transient_local().reliable(),
    [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {latest_map_ = msg;});
  restriction_map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    restriction_map_topic_, rclcpp::QoS(1).transient_local().reliable(),
    [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
      latest_restriction_map_ = msg;
    });
  if (waypoint_shift_logic_ == 3) {
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
  }
  if (waypoint_shift_logic_ >= 2) {
    logic2_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      logic2_odom_topic_, rclcpp::SensorDataQoS().keep_last(10),
      std::bind(
        &VisualLidarPathFollower::logic2_odom_callback, this,
        std::placeholders::_1));
  }

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
    get_logger(), "VisualLidarPathFollower 初始化完成：%zu 个航点，路线=%s，waypoint_shift逻辑=%d。",
    waypoints_.size(), route_name.c_str(), waypoint_shift_logic_);
  if (debug_control_logs_) {
    RCLCPP_INFO(
      get_logger(),
      "debug: 倒车点=%zu，扫码前=%s，顺时针=%s，逆时针=%s，控制前瞻=%.2f，"
      "避障=QR后zhy雷达覆盖+channel waypoint_shift，线速度=%.2f。",
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

// ********************************************************************************
// ******************************** 主控制流程 ********************************
// ********************************************************************************

// 主控制循环：拿位姿 -> 更新路线线段进度 -> 算离散前瞻 waypoint 速度 -> 发布。
// 这里保留完整顺序，避免为了几行赋值再跳到小包装函数里。
void VisualLidarPathFollower::control_once()
{
  if (waiting_for_qr_start_) {
    if (waypoint_shift_logic_ == 3 && !logic2_planning_pose_anchored_ &&
      !waypoints_.empty())
    {
      RobotPose planning_pose;
      if (lookup_robot_pose(planning_pose.x, planning_pose.y, planning_pose.yaw)) {
        if (!logic2_motion_candidate_) {
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

  if (return_to_start_active_) {
    control_return_to_start(pose);
    return;
  }

  next_control_state_ = ControlState::PATH_FOLLOWING;

  if (qr_channel_approach_active_) {
    geometry_msgs::msg::Twist reverse_cmd;
    if (apply_qr_reverse_maneuver(pose, reverse_cmd)) {
      publish_debug_visualization(pose);
      set_control_state(next_control_state_);
      cmd_pub_->publish(reverse_cmd);
      return;
    }
    update_qr_channel_approach_phase(pose);
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

  const double active_lookahead_distance = qr_channel_approach_active_ ?
    qr_channel_lidar_lookahead_distance_ : lookahead_distance_;
  auto target = compute_lookahead_target(pose, active_lookahead_distance);
  if (!target.reverse && !qr_channel_approach_active_) {
    if (waypoint_shift_logic_ == 3 && !logic2_planning_pose_anchored_) {
      logic3_force_full_route_ = true;
    }
    update_waypoint_shifts(pose, target.waypoint_index);
    logic3_force_full_route_ = false;
    target = compute_lookahead_target(pose, active_lookahead_distance);
  }
  commit_channel_lookahead_progress(target);

  auto cmd = compute_tracking_command(pose, target);
  if (qr_channel_approach_active_ && !target.reverse) {
    const double target_yaw = std::atan2(
      target.point.y - pose.y, target.point.x - pose.x);
    const double heading_error = normalize_angle(target_yaw - pose.yaw);
    const double minimum_speed_ratio = qr_channel_lidar_min_turn_speed_percent_ / 100.0;
    const double speed_ratio = minimum_speed_ratio +
      (1.0 - minimum_speed_ratio) * 0.5 * (1.0 + std::cos(std::abs(heading_error)));
    cmd.linear.x = qr_channel_lidar_cruise_linear_speed_ * speed_ratio;
    cmd.angular.z = heading_error * qr_channel_lidar_turn_angular_gain_;
  } else {
    apply_turn_speed_limit(cmd);
  }
  apply_image_capture_speed_limit(pose, cmd);
  if (qr_channel_approach_active_ && !target.reverse) {
    apply_qr_channel_lidar_obstacle_avoidance(cmd);
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

  if (return_to_start_enabled_) {
    start_return_to_start();
    return true;
  }

  set_control_state(ControlState::FINISHED);
  if (waypoint_shift_logic_ >= 2 && logic2_planning_pose_anchored_ &&
    !logic2_final_snapshot_published_)
  {
    publish_logic2_snapshot("final");
    logic2_final_snapshot_published_ = true;
  }
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

void VisualLidarPathFollower::start_return_to_start()
{
  if (return_to_start_active_) {
    return;
  }

  return_to_start_active_ = true;
  latest_return_visual_targets_.reset();
  latest_return_visual_received_at_ = std::chrono::steady_clock::time_point();
  set_control_state(ControlState::RETURNING_TO_START);

  // QR成功后YOLO曾被暂停。这里只恢复完整检测，使本节点能收到锥桶和
  // parking_sign；不发送visual_control_resume_value，视觉控制节点仍保持挂起。
  set_perception_mode(perception_full_resume_value_, "完整识别（返回起点）");
  publish_stop();
  RCLCPP_INFO(
    get_logger(),
    "channel巡点完成，开始返回map起点(%.2f, %.2f)；"
    "base_link进入停车矩形或视觉满足停车条件时结束。",
    return_target_x_, return_target_y_);
}

void VisualLidarPathFollower::control_return_to_start(
  const VisualLidarPathFollower::RobotPose & pose)
{
  set_control_state(ControlState::RETURNING_TO_START);

  // 地图矩形与视觉停车都是最高优先级；停车判定不能被同帧避障覆盖。
  if (return_stop_zone_contains(pose)) {
    finish_return_to_start("base_link中心进入ZHY停车矩形");
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

  if (return_visual_parking_condition_met()) {
    finish_return_to_start("visual_control停车标志条件满足");
    return;
  }

  const double target_yaw = std::atan2(
    return_target_y_ - pose.y, return_target_x_ - pose.x);
  const double heading_error = normalize_angle(target_yaw - pose.yaw);
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = return_follow_linear_speed_;
  cmd.angular.z = return_turn_angular_gain_ * heading_error;
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

void VisualLidarPathFollower::finish_return_to_start(const char * reason)
{
  return_to_start_active_ = false;
  set_control_state(ControlState::FINISHED);
  if (waypoint_shift_logic_ >= 2 && logic2_planning_pose_anchored_ &&
    !logic2_final_snapshot_published_)
  {
    publish_logic2_snapshot("final");
    logic2_final_snapshot_published_ = true;
  }
  RCLCPP_INFO(get_logger(), "返回起点阶段结束：%s，停车。", reason);
  publish_stop();
  rclcpp::shutdown();
}

bool VisualLidarPathFollower::return_stop_zone_contains(
  const VisualLidarPathFollower::RobotPose & pose) const
{
  const double relative_x = pose.x - return_target_x_;
  const double relative_y = pose.y - return_target_y_;
  return std::abs(relative_x) <= return_stop_zone_length_ * 0.5 &&
         relative_y <= return_stop_zone_upper_half_width_ &&
         relative_y >= -return_stop_zone_lower_half_width_;
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

// 逐点法线偏移在圆弧内侧满足 kappa * d >= 1 时会退化成尖点并反相。
// 这里先生成旧算法候选，再检测离散曲率、沿原路线进度和局部自交；只有
// 检测到不可行的连续偏移段才改用同侧局部曲线。局部曲线在障碍对应点附近
// 采用固定世界方向整体平移，并用两段三次Bezier接回原偏移段的前后锚点。
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
      std::size_t first, std::size_t last) {
      first = first > 0 ? first - 1 : first;
      last = std::min(last + 1, route_size - 1);
      for (std::size_t index = first; index < last; ++index) {
        const double new_length = point_distance(route[index], route[index + 1]);
        const double original_length = point_distance(
          original_waypoints_[index], original_waypoints_[index + 1]);
        if (new_length < 1e-6 ||
          new_length > std::max(0.10, 6.0 * original_length))
        {
          return false;
        }
      }
      for (std::size_t index = first + 1; index < last; ++index) {
        const Point2D incoming{
          route[index].x - route[index - 1].x,
          route[index].y - route[index - 1].y};
        const Point2D outgoing{
          route[index + 1].x - route[index].x,
          route[index + 1].y - route[index].y};
        const double incoming_length = std::hypot(incoming.x, incoming.y);
        const double outgoing_length = std::hypot(outgoing.x, outgoing.y);
        if (incoming_length < 1e-9 || outgoing_length < 1e-9) {
          return false;
        }
        const double turn_cosine =
          (incoming.x * outgoing.x + incoming.y * outgoing.y) /
          (incoming_length * outgoing_length);
        if (turn_cosine < -0.25) {
          return false;
        }
      }
      for (std::size_t first_segment = first;
        first_segment + 1 <= last; ++first_segment)
      {
        for (std::size_t second_segment = first_segment + 2;
          second_segment + 1 <= last; ++second_segment)
        {
          if (proper_segments_intersect(
              route[first_segment], route[first_segment + 1],
              route[second_segment], route[second_segment + 1]))
          {
            return false;
          }
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

    bool unsafe = false;
    double minimum_feasibility = std::numeric_limits<double>::infinity();
    std::size_t risk_index = run_first;
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
      if (feasibility < minimum_feasibility) {
        minimum_feasibility = feasibility;
        risk_index = index;
      }
      if (feasibility <= waypoint_shift_rebuild_min_progress_ratio_) {
        unsafe = true;
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
        unsafe = true;
        if (!std::isfinite(minimum_feasibility)) {
          risk_index = std::min(index + 1, run_last);
        }
      }
    }

    const std::size_t intersection_first = progress_first;
    const std::size_t intersection_last = std::min(run_last + 1, route_size - 1);
    for (std::size_t first_segment = intersection_first;
      !unsafe && first_segment + 1 <= intersection_last; ++first_segment)
    {
      for (std::size_t second_segment = first_segment + 2;
        second_segment + 1 <= intersection_last; ++second_segment)
      {
        if (proper_segments_intersect(
            shifted_route[first_segment], shifted_route[first_segment + 1],
            shifted_route[second_segment], shifted_route[second_segment + 1]))
        {
          unsafe = true;
          risk_index = std::min(
            run_last, (first_segment + second_segment + 1) / 2);
          break;
        }
      }
    }

    if (!unsafe) {
      run_first = run_last + 1;
      continue;
    }

    std::size_t source_index = source_identity < route_size ?
      source_identity : risk_index;
    source_index = std::clamp(source_index, run_first, run_last);
    Point2D fixed_direction = waypoint_shift_world_directions_[source_index];
    if (!normalize_vector(fixed_direction, fixed_direction)) {
      fixed_direction = Point2D();
      for (std::size_t index = run_first; index <= run_last; ++index) {
        fixed_direction.x += waypoint_shift_world_directions_[index].x;
        fixed_direction.y += waypoint_shift_world_directions_[index].y;
      }
      if (!normalize_vector(fixed_direction, fixed_direction)) {
        Point2D normal;
        if (!original_route_normal_at(source_index, normal)) {
          run_first = run_last + 1;
          continue;
        }
        fixed_direction = Point2D{
          static_cast<double>(offset_sign) * normal.x,
          static_cast<double>(offset_sign) * normal.y};
      }
    }
    double target_offset = 0.0;
    for (std::size_t index = run_first; index <= run_last; ++index) {
      target_offset = std::max(target_offset, std::abs(waypoint_shift_offsets_[index]));
    }
    if (target_offset < 1e-9) {
      run_first = run_last + 1;
      continue;
    }

    std::size_t plateau_first = source_index;
    double plateau_distance = 0.0;
    while (plateau_first > run_first &&
      plateau_distance < waypoint_shift_rebuild_plateau_half_length_)
    {
      plateau_distance += point_distance(
        original_waypoints_[plateau_first], original_waypoints_[plateau_first - 1]);
      --plateau_first;
    }
    std::size_t plateau_last = source_index;
    plateau_distance = 0.0;
    while (plateau_last < run_last &&
      plateau_distance < waypoint_shift_rebuild_plateau_half_length_)
    {
      plateau_distance += point_distance(
        original_waypoints_[plateau_last], original_waypoints_[plateau_last + 1]);
      ++plateau_last;
    }

    const std::size_t anchor_first = run_first > 0 ? run_first - 1 : run_first;
    const std::size_t anchor_last = std::min(run_last + 1, route_size - 1);
    const Point2D start_point{shifted_route[anchor_first].x, shifted_route[anchor_first].y};
    const Point2D finish_point{shifted_route[anchor_last].x, shifted_route[anchor_last].y};
    const Point2D plateau_start{
      original_waypoints_[plateau_first].x + target_offset * fixed_direction.x,
      original_waypoints_[plateau_first].y + target_offset * fixed_direction.y};
    const Point2D plateau_finish{
      original_waypoints_[plateau_last].x + target_offset * fixed_direction.x,
      original_waypoints_[plateau_last].y + target_offset * fixed_direction.y};
    Point2D start_tangent;
    Point2D plateau_start_tangent;
    Point2D plateau_finish_tangent;
    Point2D finish_tangent;
    if (!route_tangent_at(original_waypoints_, anchor_first, start_tangent) ||
      !route_tangent_at(original_waypoints_, plateau_first, plateau_start_tangent) ||
      !route_tangent_at(original_waypoints_, plateau_last, plateau_finish_tangent) ||
      !route_tangent_at(original_waypoints_, anchor_last, finish_tangent))
    {
      run_first = run_last + 1;
      continue;
    }

    std::vector<Waypoint> rebuilt_candidate = shifted_route;
    bool valid_rebuild = false;
    const double handle_ratios[] = {
      waypoint_shift_rebuild_bezier_handle_ratio_,
      0.5 * waypoint_shift_rebuild_bezier_handle_ratio_, 0.0};
    for (const double handle_ratio : handle_ratios) {
      rebuilt_candidate = shifted_route;
      fill_bezier(
        rebuilt_candidate, anchor_first, plateau_first,
        start_point, plateau_start, start_tangent, plateau_start_tangent,
        handle_ratio);
      for (std::size_t index = plateau_first; index <= plateau_last; ++index) {
        rebuilt_candidate[index].x =
          original_waypoints_[index].x + target_offset * fixed_direction.x;
        rebuilt_candidate[index].y =
          original_waypoints_[index].y + target_offset * fixed_direction.y;
      }
      fill_bezier(
        rebuilt_candidate, plateau_last, anchor_last,
        plateau_finish, finish_point, plateau_finish_tangent, finish_tangent,
        handle_ratio);
      if (local_geometry_valid(rebuilt_candidate, anchor_first, anchor_last)) {
        valid_rebuild = true;
        break;
      }
    }
    // 不新增停车或换侧分支。极端情况下仍使用零控制柄的同侧折中曲线，
    // 而不是退回已经确认会反相的逐点法线偏移。
    shifted_route = std::move(rebuilt_candidate);
    ++rebuilt_count;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "waypoint_shift：检测到圆弧内侧偏移不可行（点%zu-%zu，最小可行因子=%.3f），"
      "已保持原方向并重建局部段，来源点=%zu，目标偏移=%.3fm%s。",
      run_first + 1, run_last + 1, minimum_feasibility,
      source_index + 1, target_offset, valid_rebuild ? "" : "，使用零控制柄兜底");
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
      int occupied_threshold, int inflation_cells) {
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

      for (int offset_y = -inflation_cells;
        offset_y <= inflation_cells; ++offset_y)
      {
        for (int offset_x = -inflation_cells;
          offset_x <= inflation_cells; ++offset_x)
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

  const bool occupied_in_physical_map = occupied_in_map(
    latest_map_, static_map_occupied_threshold_, static_map_inflation_cells_);
  const bool occupied_in_restriction_map = occupied_in_map(
    restriction_map, waypoint_shift_occupied_threshold_,
    restriction_inflation_cells);
  return occupied_in_physical_map || occupied_in_restriction_map;
}

std::vector<VisualLidarPathFollower::ScanPointCluster>
VisualLidarPathFollower::collect_non_static_scan_clusters_map(
  const VisualLidarPathFollower::RobotPose & robot_pose,
  int minimum_cluster_points,
  bool use_logic3_static_restriction_map) const
{
  if (!latest_scan_) {
    return {};
  }
  const bool use_logic3_runtime_restriction_map =
    waypoint_shift_logic_ == 3 && logic2_planning_pose_anchored_ &&
    !use_logic3_static_restriction_map;
  const auto & active_restriction_map = use_logic3_static_restriction_map ?
    latest_logic3_static_restriction_map_ :
    (use_logic3_runtime_restriction_map ?
    latest_logic3_runtime_restriction_map_ : latest_restriction_map_);
  const int active_restriction_inflation_cells =
    use_logic3_static_restriction_map ?
    logic3_static_restriction_inflation_cells_ : static_map_inflation_cells_;
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

bool VisualLidarPathFollower::latest_qr_lidar_scan_is_fresh() const
{
  if (!latest_scan_) {
    return false;
  }
  const double age = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - latest_scan_received_at_).count();
  return age >= 0.0 && age <= qr_lidar_scan_timeout_;
}

// 为 QR 倒车和到 channel 避障准备同一帧雷达的坐标变换。
// 开启墙体过滤时，地图或采样时刻 TF 缺失都视为数据未就绪，避免降级后把墙当障碍。
bool VisualLidarPathFollower::prepare_qr_lidar_scan(
  VisualLidarPathFollower::RobotPose & scan_to_base,
  VisualLidarPathFollower::RobotPose & scan_to_map) const
{
  scan_to_base = RobotPose();
  scan_to_map = RobotPose();
  if (!latest_qr_lidar_scan_is_fresh()) {
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
  if (!qr_lidar_enable_static_map_filter_) {
    return true;
  }
  if (!latest_map_) {
    return false;
  }
  const auto & map = *latest_map_;
  if (map.info.width == 0 || map.info.height == 0 || map.info.resolution <= 0.0 ||
    map.data.size() < static_cast<std::size_t>(map.info.width) * map.info.height)
  {
    return false;
  }
  if (source_frame == frame_id_) {
    return true;
  }
  return lookup_pose(frame_id_, source_frame, lookup_stamp, scan_to_map);
}

// 只用物理定位地图过滤墙体，不把 restriction_map 的软禁行区混入这两段判断。
bool VisualLidarPathFollower::qr_lidar_map_point_is_wall(
  const VisualLidarPathFollower::Point2D & map_point) const
{
  if (!qr_lidar_enable_static_map_filter_ || !latest_map_) {
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
    std::ceil(qr_lidar_static_map_wall_padding_ / resolution));

  for (int y = cell_y - padding_cells; y <= cell_y + padding_cells; ++y) {
    for (int x = cell_x - padding_cells; x <= cell_x + padding_cells; ++x) {
      if (x < 0 || y < 0 || x >= width || y >= height) {
        continue;
      }
      const auto index = static_cast<std::size_t>(y) * map.info.width +
        static_cast<std::size_t>(x);
      if (map.data[index] >= qr_lidar_static_map_occupied_threshold_) {
        return true;
      }
    }
  }
  return false;
}

// 在指定朝向上取最近障碍：总视野扇形、最大距离和矩形走廊必须同时命中。
// center_angle=0 检查车前，center_angle=pi 检查车后。
VisualLidarPathFollower::QrLidarObstacle
VisualLidarPathFollower::find_qr_lidar_obstacle(
  double center_angle, double maximum_distance, double fov_deg,
  double corridor_half_width,
  const VisualLidarPathFollower::RobotPose & scan_to_base,
  const VisualLidarPathFollower::RobotPose & scan_to_map) const
{
  QrLidarObstacle result;
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

    if (qr_lidar_enable_static_map_filter_) {
      const Point2D map_point{
        scan_to_map.x + cos_map * scan_x - sin_map * scan_y,
        scan_to_map.y + sin_map * scan_x + cos_map * scan_y};
      if (qr_lidar_map_point_is_wall(map_point)) {
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

// zhy 条件倒车：扫码后只判定一次；触发后固定向右后撤，直到后方命中，
// 或 map.y 回到阈值内且前方已经清空。前后点云均过滤墙体并使用扇形+矩形。
bool VisualLidarPathFollower::apply_qr_reverse_maneuver(
  const VisualLidarPathFollower::RobotPose & robot_pose,
  geometry_msgs::msg::Twist & cmd)
{
  if (qr_escape_state_ == QrEscapeState::IDLE) {
    return false;
  }

  RobotPose scan_to_base;
  RobotPose scan_to_map;
  if (!prepare_qr_lidar_scan(scan_to_base, scan_to_map)) {
    cmd = geometry_msgs::msg::Twist();
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "二维码倒车等待新鲜雷达、物理地图及采样时刻TF，当前停车。");
    return true;
  }

  const auto front_obstacle = find_qr_lidar_obstacle(
    0.0, qr_reverse_front_obstacle_distance_, qr_reverse_front_fov_deg_,
    qr_reverse_front_corridor_half_width_, scan_to_base, scan_to_map);
  const auto rear_obstacle = find_qr_lidar_obstacle(
    M_PI, qr_reverse_rear_obstacle_distance_, qr_reverse_rear_fov_deg_,
    qr_reverse_rear_corridor_half_width_, scan_to_base, scan_to_map);

  if (qr_escape_state_ == QrEscapeState::PENDING) {
    if (robot_pose.y > qr_reverse_y_threshold_ || front_obstacle.found) {
      qr_escape_state_ = QrEscapeState::BACKING_RIGHT;
      RCLCPP_WARN(
        get_logger(),
        "开始 zhy 二维码倒车：map.y=%.3f（阈值 %.3f），前方障碍=%s；"
        "输出 linear.x=-%.2f, angular.z=%.2f。",
        robot_pose.y, qr_reverse_y_threshold_, front_obstacle.found ? "是" : "否",
        qr_reverse_linear_speed_, qr_reverse_angular_speed_);
    } else {
      qr_escape_state_ = QrEscapeState::IDLE;
      RCLCPP_INFO(
        get_logger(),
        "跳过二维码倒车：map.y=%.3f 不大于阈值 %.3f，且前方扇形走廊已清空。",
        robot_pose.y, qr_reverse_y_threshold_);
      return false;
    }
  }

  if (rear_obstacle.found) {
    qr_escape_state_ = QrEscapeState::IDLE;
    RCLCPP_INFO(
      get_logger(), "二维码倒车结束：后方非墙障碍进入 %.2fm 停车区。",
      qr_reverse_rear_obstacle_distance_);
    return false;
  }
  if (!front_obstacle.found && robot_pose.y <= qr_reverse_y_threshold_) {
    qr_escape_state_ = QrEscapeState::IDLE;
    RCLCPP_INFO(
      get_logger(),
      "二维码倒车结束：前方扇形走廊已清空，map.y=%.3f 已回到阈值 %.3f 内。",
      robot_pose.y, qr_reverse_y_threshold_);
    return false;
  }

  cmd.linear.x = -qr_reverse_linear_speed_;
  cmd.angular.z = qr_reverse_angular_speed_;
  if (debug_control_logs_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 500,
      "debug: zhy二维码倒车，map.y=%.3f，前障碍=%s，后障碍=%s，输出=%.2f/%.2f。",
      robot_pose.y, front_obstacle.found ? "是" : "否",
      rear_obstacle.found ? "是" : "否", cmd.linear.x, cmd.angular.z);
  }
  return true;
}

// 进入 channel 航点邻域后关闭 zhy 进场雷达覆盖，恢复原有 waypoint_shift。
void VisualLidarPathFollower::update_qr_channel_approach_phase(
  const VisualLidarPathFollower::RobotPose & robot_pose)
{
  if (qr_escape_state_ == QrEscapeState::BACKING_RIGHT) {
    return;
  }

  if (waypoints_.empty()) {
    return;
  }
  const double entry_waypoint_distance = std::hypot(
    waypoints_.front().x - robot_pose.x, waypoints_.front().y - robot_pose.y);
  bool crossed_entry_section = false;
  if (waypoints_.size() >= 2) {
    const double tangent_x = waypoints_[1].x - waypoints_[0].x;
    const double tangent_y = waypoints_[1].y - waypoints_[0].y;
    const double tangent_length = std::hypot(tangent_x, tangent_y);
    if (tangent_length > 1.0e-8) {
      const double unit_x = tangent_x / tangent_length;
      const double unit_y = tangent_y / tangent_length;
      const double relative_x = robot_pose.x - waypoints_[0].x;
      const double relative_y = robot_pose.y - waypoints_[0].y;
      const double along_route = relative_x * unit_x + relative_y * unit_y;
      const double cross_track = std::abs(-relative_x * unit_y + relative_y * unit_x);
      crossed_entry_section =
        along_route >= 0.0 && cross_track <= waypoint_pass_radius(0);
    }
  }
  if (entry_waypoint_distance > qr_channel_handoff_waypoint_distance_ &&
    !crossed_entry_section)
  {
    return;
  }

  qr_channel_approach_active_ = false;
  qr_escape_state_ = QrEscapeState::IDLE;
  current_index_ = std::min(current_index_, waypoints_.size() - 1);
  first_waypoint_reached_ = true;
  RCLCPP_INFO(
    get_logger(),
    "已进入 waypoint_channel：入口首点距离=%.3fm，入口截面穿越=%s；"
    "关闭 zhy 进场雷达覆盖，恢复原有 waypoint_shift。",
    entry_waypoint_distance, crossed_entry_section ? "是" : "否");
}

// zhy 到 channel 雷达避障：取前方扇形走廊内最近的非墙点，障碍在左则右绕，
// 障碍在右则左绕；转向量从中心线向扇形边缘线性衰减。
bool VisualLidarPathFollower::apply_qr_channel_lidar_obstacle_avoidance(
  geometry_msgs::msg::Twist & cmd)
{
  RobotPose scan_to_base;
  RobotPose scan_to_map;
  if (!prepare_qr_lidar_scan(scan_to_base, scan_to_map)) {
    cmd = geometry_msgs::msg::Twist();
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "QR到channel等待新鲜雷达、物理地图及采样时刻TF，当前停车。");
    return true;
  }

  const auto obstacle = find_qr_lidar_obstacle(
    0.0, qr_channel_lidar_avoid_distance_, qr_channel_lidar_avoid_fov_deg_,
    qr_channel_lidar_corridor_half_width_, scan_to_base, scan_to_map);
  if (!obstacle.found) {
    return false;
  }

  const double half_fov = std::max(
    qr_channel_lidar_avoid_fov_deg_ * M_PI / 360.0, 0.01);
  const double center_weight = std::clamp(
    1.0 - std::abs(obstacle.relative_angle) / half_fov, 0.0, 1.0);
  const double avoid_direction = obstacle.relative_angle <= 0.0 ? 1.0 : -1.0;
  cmd.linear.x = qr_channel_lidar_avoid_linear_speed_;
  cmd.angular.z = avoid_direction * qr_channel_lidar_avoid_angular_gain_ * center_weight;
  RCLCPP_WARN_THROTTLE(
    get_logger(), *get_clock(), 500,
    "QR到channel雷达避障：距离=%.3fm，相对角=%.1fdeg，输出=%.2f/%.2f。",
    obstacle.range, obstacle.relative_angle * 180.0 / M_PI,
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
  waypoint_shift_dynamic_directions_.assign(waypoints_.size(), 0);
  waypoint_shift_world_directions_.assign(waypoints_.size(), Point2D());
  waypoint_shift_source_indices_.assign(waypoints_.size(), waypoints_.size());
  logic3_frozen_base_shift_directions_.clear();
  logic3_frozen_base_source_indices_.clear();
  logic3_runtime_cluster_tracks_.clear();
  processed_waypoint_shift_scan_sequence_ = scan_sequence_;
}

bool VisualLidarPathFollower::logic2_cluster_hits_route_window(
  const VisualLidarPathFollower::ScanPointCluster & cluster,
  std::size_t first_index, std::size_t last_index) const
{
  if (original_waypoints_.empty() || first_index >= last_index) {
    return false;
  }
  const std::size_t bounded_last = std::min(last_index, original_waypoints_.size());
  const double radius_squared =
    waypoint_cloud_check_radius_ * waypoint_cloud_check_radius_;
  auto point_hits_route = [&](const Point2D & point, const std::vector<Waypoint> & route) {
      if (route.size() != original_waypoints_.size()) {
        return false;
      }
      for (std::size_t index = first_index; index < bounded_last; ++index) {
        const double dx = point.x - route[index].x;
        const double dy = point.y - route[index].y;
        if (dx * dx + dy * dy <= radius_squared) {
          return true;
        }
      }
      return false;
    };
  return std::any_of(
    cluster.points.begin(), cluster.points.end(), [&](const Point2D & point) {
      if (point_hits_route(point, original_waypoints_)) {
        return true;
      }
      // 逻辑3运行时车辆实际沿冻结/动态路线行驶；同时检查当前路线，避免
      // 只贴近已偏移路线的新障碍被“原始路线窗口”漏掉。
      return waypoint_shift_logic_ == 3 && logic2_planning_pose_anchored_ &&
      point_hits_route(point, waypoints_);
    });
}

void VisualLidarPathFollower::update_logic3_static_cluster_tracks(
  const std::vector<VisualLidarPathFollower::ScanPointCluster> & clusters,
  const VisualLidarPathFollower::RobotPose & robot_pose)
{
  const double now_seconds = now().seconds();
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

  bool confirmed_set_changed = false;
  bool state1_invalidated = false;
  std::vector<bool> track_matched(logic3_static_cluster_tracks_.size(), false);
  for (const auto & cluster : clusters) {
    std::size_t best_track = logic3_static_cluster_tracks_.size();
    double best_distance = logic3_static_cluster_match_radius_;
    for (std::size_t index = 0; index < logic3_static_cluster_tracks_.size(); ++index) {
      if (track_matched[index]) {
        continue;
      }
      const auto & track = logic3_static_cluster_tracks_[index];
      if (logic3_static_state1_active_ && track.fixed_in_state1 &&
        track.loss_pending &&
        now_seconds - track.loss_started_seconds > logic3_static_loss_grace_seconds_)
      {
        continue;
      }
      const double match_radius = logic3_static_state1_active_ &&
        track.fixed_in_state1 && track.loss_pending ?
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
    const bool recovered_during_grace = logic3_static_state1_active_ &&
      track.fixed_in_state1 && track.loss_pending;

    // 状态1后的失联重捕只检查固定锚点0.20m；成功出现一次就恢复原ID，
    // 不再要求该时刻的2秒窗口立刻重新达到10帧。
    if (recovered_during_grace) {
      track.loss_pending = false;
      track.loss_started_seconds = 0.0;
    } else if (stability_distance > effective_stability_radius) {
      // 坐标越出其距离自适应稳定半径，视为位置发生更新。正式ID作废并
      // 从这个新位置重新进行2秒窗口确认；若已在状态1则本轮整体失效。
      if (track.confirmed) {
        confirmed_set_changed = true;
      }
      if (logic3_static_state1_active_ && track.fixed_in_state1) {
        state1_invalidated = true;
      }
      track.identity_id = 0;
      track.confirmed = false;
      track.fixed_in_state1 = false;
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
      confirmed_set_changed = true;
      if (logic3_static_state1_active_) {
        state1_invalidated = true;
      }
    }

    if (track.confirmed && !logic3_static_state1_active_ &&
      rolling_hits < logic3_static_confirmation_min_hits_)
    {
      // 状态1前没有额外失联宽限：一旦2秒窗口不再满足10帧，正式坐标
      // 立即撤销，剩余观测继续仅作为滚动窗口候选。
      track.identity_id = 0;
      track.confirmed = false;
      track.ready = false;
      confirmed_set_changed = true;
    }

    if (logic3_static_state1_active_ && track.fixed_in_state1) {
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
        // 只有状态1后的正式ID才使用1秒宽限；超时才删除该ID并结束本轮。
        remove_track[index] = true;
        confirmed_set_changed = true;
        state1_invalidated = true;
      }
    } else if (!track.confirmed && track.rolling_hit_times.empty()) {
      // 状态1前的临时候选完全由2秒窗口维持，不另设1秒内部ID保留。
      remove_track[index] = true;
    }
  }

  if (state1_invalidated) {
    reset_logic3_static_epoch();
    RCLCPP_WARN(
      get_logger(),
      "逻辑3静态状态1失效：固定坐标新增、删除或位置更新；保留其余ID的2秒窗口数据。");
  }

  for (std::size_t index = remove_track.size(); index > 0; --index) {
    if (remove_track[index - 1]) {
      logic3_static_cluster_tracks_.erase(
        logic3_static_cluster_tracks_.begin() + static_cast<std::ptrdiff_t>(index - 1));
      track_matched.erase(
        track_matched.begin() + static_cast<std::ptrdiff_t>(index - 1));
    }
  }

  const bool has_unconfirmed_candidate = std::any_of(
    logic3_static_cluster_tracks_.begin(), logic3_static_cluster_tracks_.end(),
    [](const Logic3StaticClusterTrack & track) {
      return !track.confirmed && !track.rolling_hit_times.empty();
    });
  if (!logic3_static_state1_active_ && !confirmed_set_changed &&
    !has_unconfirmed_candidate)
  {
    // 不再额外等待1秒：所有滚动候选均已确认或自然过期，且正式ID集合
    // 在本次更新没有数量/位置变化，就立即进入状态1。此后才启用1秒
    // 失联重捕和最终10个有效观测采样。
    begin_logic3_static_epoch();
    RCLCPP_INFO(get_logger(), "逻辑3静态进入状态1：固定ID集合已保持不变。");
  }

  if (logic3_static_state1_active_) {
    for (std::size_t index = 0; index < logic3_static_cluster_tracks_.size(); ++index) {
      auto & track = logic3_static_cluster_tracks_[index];
      if (!track.fixed_in_state1 || !track.confirmed || !track_matched[index] ||
        track.ready)
      {
        continue;
      }

      // 状态1后按有效观测累计，不要求所有ID在同一全局帧同时出现。
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

      Point2D shift_direction;
      if (logic3_static_shift_direction(averaged_cluster, shift_direction)) {
        track.ready_obstacle = Logic3StaticObstacle{averaged_cluster, shift_direction};
        track.ready = true;
      } else {
        // 坐标样本有效但暂时无法从通道路线得到方向时，只重采该ID，
        // 不影响其他固定ID已经完成的样本。
        track.state1_sample_count = 0;
        track.sample_center_sum_x = 0.0;
        track.sample_center_sum_y = 0.0;
        track.sampled_points.clear();
      }
    }

    logic3_static_ready_ = std::all_of(
      logic3_static_cluster_tracks_.begin(), logic3_static_cluster_tracks_.end(),
      [](const Logic3StaticClusterTrack & track) {
        return !track.fixed_in_state1 || track.ready;
      });
  }
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

void VisualLidarPathFollower::reset_logic3_static_epoch()
{
  logic3_static_state1_active_ = false;
  logic3_static_ready_ = false;
  for (auto & track : logic3_static_cluster_tracks_) {
    track.fixed_in_state1 = false;
    track.loss_pending = false;
    track.loss_started_seconds = 0.0;
    track.ready = false;
    track.state1_sample_count = 0;
    track.sample_center_sum_x = 0.0;
    track.sample_center_sum_y = 0.0;
    track.sampled_points.clear();
    track.ready_obstacle = Logic3StaticObstacle();
  }
}

void VisualLidarPathFollower::begin_logic3_static_epoch()
{
  reset_logic3_static_epoch();
  logic3_static_state1_active_ = true;
  for (auto & track : logic3_static_cluster_tracks_) {
    track.fixed_in_state1 = track.confirmed;
  }
  logic3_static_ready_ = std::none_of(
    logic3_static_cluster_tracks_.begin(), logic3_static_cluster_tracks_.end(),
    [](const Logic3StaticClusterTrack & track) {
      return track.fixed_in_state1;
    });
}

bool VisualLidarPathFollower::logic3_static_shift_direction(
  const VisualLidarPathFollower::ScanPointCluster & cluster,
  VisualLidarPathFollower::Point2D & shift_direction) const
{
  return logic3_shift_direction_on_route_range(
    cluster, 0, original_waypoints_.size(), shift_direction);
}

bool VisualLidarPathFollower::logic3_shift_direction_on_route_range(
  const VisualLidarPathFollower::ScanPointCluster & cluster,
  std::size_t route_first_index, std::size_t route_last_index,
  VisualLidarPathFollower::Point2D & shift_direction) const
{
  if (original_waypoints_.size() < 2) {
    return false;
  }

  route_first_index = std::min(
    route_first_index, original_waypoints_.size() - 1);
  route_last_index = std::min(route_last_index, original_waypoints_.size());
  if (route_first_index + 1 >= route_last_index) {
    return false;
  }

  double best_distance_squared = std::numeric_limits<double>::infinity();
  Point2D route_tangent;
  Point2D route_projection;
  for (std::size_t index = route_first_index; index + 1 < route_last_index; ++index) {
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
      route_tangent = Point2D{segment_x / length, segment_y / length};
      route_projection = Point2D{projection_x, projection_y};
    }
  }
  if (!std::isfinite(best_distance_squared)) {
    return false;
  }

  const Point2D left_normal{-route_tangent.y, route_tangent.x};
  const Point2D right_normal{-left_normal.x, -left_normal.y};
  const double obstacle_lateral =
    (cluster.center.x - route_projection.x) * left_normal.x +
    (cluster.center.y - route_projection.y) * left_normal.y;
  // 障碍在路线左侧时向右绕，在右侧时向左绕。理论上的中心线零值
  // 固定向路线右侧，保证静态快照结果确定且不依赖restriction_map。
  shift_direction = obstacle_lateral >= 0.0 ? right_normal : left_normal;
  return true;
}

std::vector<VisualLidarPathFollower::Logic3StaticObstacle>
VisualLidarPathFollower::logic3_ready_static_obstacles() const
{
  std::vector<Logic3StaticObstacle> obstacles;
  if (!logic3_static_state1_active_ || !logic3_static_ready_) {
    return obstacles;
  }
  for (const auto & track : logic3_static_cluster_tracks_) {
    if (track.fixed_in_state1 && track.ready) {
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
  if (waypoint_shift_logic_ != 3 || logic2_planning_pose_anchored_ ||
    logic2_motion_candidate_)
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
  if (logic3_static_state1_active_) {
    for (const auto & track : logic3_static_cluster_tracks_) {
      if (track.fixed_in_state1 && track.confirmed) {
        fixed_tracks.push_back(&track);
      }
    }
    std::sort(
      fixed_tracks.begin(), fixed_tracks.end(),
      [](const Logic3StaticClusterTrack * left,
        const Logic3StaticClusterTrack * right) {
        return left->identity_id < right->identity_id;
      });
  }

  std::ostringstream status;
  status << "状态1=" << (logic3_static_state1_active_ ? 1 : 0)
         << "\n固定坐标：";
  status << std::fixed << std::setprecision(3);
  const std::size_t rolling_scan_count = logic3_static_scan_times_.size();
  for (const auto * track : fixed_tracks) {
    const Point2D & display_center = track->ready ?
      track->ready_obstacle.cluster.center : track->stability_anchor;
    status << "\nID" << track->identity_id << ":("
           << display_center.x << ',' << display_center.y << ") 命中="
           << track->rolling_hit_times.size() << '/' << rolling_scan_count
           << " 采样=" << std::min(
      track->state1_sample_count, logic3_static_average_samples_)
           << '/' << logic3_static_average_samples_ << "；";
  }
  status << "\nREADY=" << (logic3_static_ready_ ? 1 : 0);
  RCLCPP_INFO(get_logger(), "%s", status.str().c_str());
}

void VisualLidarPathFollower::record_logic2_red_cluster(
  const VisualLidarPathFollower::ScanPointCluster & cluster)
{
  const auto recorded = std::find_if(
    logic2_red_cluster_history_.begin(), logic2_red_cluster_history_.end(),
    [&](const Point2D & point) {
      return std::hypot(
        point.x - cluster.center.x, point.y - cluster.center.y) <=
      logic2_red_history_merge_radius_;
    });
  if (recorded != logic2_red_cluster_history_.end()) {
    // 历史圆只负责绘图身份；中心持续跟随该身份的平滑坐标，避免最终图片
    // 永远停在第一次单帧观测的位置。
    *recorded = cluster.center;
    return;
  }
  logic2_red_cluster_history_.push_back(cluster.center);
  RCLCPP_WARN(
    get_logger(),
    "逻辑%d记录通道内动态红簇：map(%.3f, %.3f)，%s半径=%.2fm。",
    waypoint_shift_logic_, cluster.center.x, cluster.center.y,
    waypoint_shift_logic_ == 3 ? "图片历史" : "身份",
    logic2_red_history_merge_radius_);
}

void VisualLidarPathFollower::update_logic2_runtime_red_tracks(
  const std::vector<VisualLidarPathFollower::ScanPointCluster> & clusters)
{
  const int record_confirm_hits = waypoint_shift_confirm_scans_;
  std::vector<bool> track_matched(logic2_runtime_red_tracks_.size(), false);
  for (const auto & cluster : clusters) {
    std::size_t best_track = logic2_runtime_red_tracks_.size();
    double best_distance = logic2_red_history_merge_radius_;
    for (std::size_t index = 0; index < logic2_runtime_red_tracks_.size(); ++index) {
      if (track_matched[index]) {
        continue;
      }
      const auto & center = logic2_runtime_red_tracks_[index].cluster.center;
      const double distance = std::hypot(
        cluster.center.x - center.x, cluster.center.y - center.y);
      if (distance <= best_distance) {
        best_distance = distance;
        best_track = index;
      }
    }
    if (best_track == logic2_runtime_red_tracks_.size()) {
      Logic2TrackedCluster track;
      track.cluster = cluster;
      track.hit_count = 1;
      logic2_runtime_red_tracks_.push_back(std::move(track));
      track_matched.push_back(true);
      if (record_confirm_hits == 1) {
        record_logic2_red_cluster(cluster);
      }
      continue;
    }
    auto & track = logic2_runtime_red_tracks_[best_track];
    track.cluster = cluster;
    ++track.hit_count;
    track.missing_count = 0;
    track_matched[best_track] = true;
    if (track.hit_count == record_confirm_hits) {
      record_logic2_red_cluster(cluster);
    }
  }
  for (std::size_t index = 0; index < logic2_runtime_red_tracks_.size(); ++index) {
    if (!track_matched[index]) {
      ++logic2_runtime_red_tracks_[index].missing_count;
      logic2_runtime_red_tracks_[index].hit_count = 0;
    }
  }
  logic2_runtime_red_tracks_.erase(
    std::remove_if(
      logic2_runtime_red_tracks_.begin(), logic2_runtime_red_tracks_.end(),
      [&](const Logic2TrackedCluster & track) {
        return track.missing_count >= waypoint_shift_release_scans_;
      }),
      logic2_runtime_red_tracks_.end());
}

void VisualLidarPathFollower::update_logic3_runtime_red_tracks(
  const std::vector<VisualLidarPathFollower::ScanPointCluster> & clusters,
  std::size_t route_first_index, std::size_t route_last_index)
{
  const double now_seconds = now().seconds();
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
      track.stability_anchor = cluster.center;
      track.consecutive_hits = 1;
      track.stable_since_seconds = now_seconds;
      track.last_seen_seconds = now_seconds;
      track.recent_samples.push_back(cluster);
      logic3_runtime_cluster_tracks_.push_back(std::move(track));
      track_matched.push_back(true);
      if (waypoint_shift_confirm_scans_ == 1) {
        auto & new_track = logic3_runtime_cluster_tracks_.back();
        Point2D shift_direction;
        if (logic3_shift_direction_on_route_range(
            new_track.cluster, route_first_index, route_last_index, shift_direction))
        {
          new_track.shift_direction = shift_direction;
          new_track.direction_valid = true;
          new_track.confirmed = true;
          record_logic2_red_cluster(new_track.cluster);
        }
      }
      continue;
    }

    auto & track = logic3_runtime_cluster_tracks_[best_track];
    if (track.missing_count > 0) {
      track.consecutive_hits = 0;
      track.recent_samples.clear();
      if (!track.persistent) {
        track.stability_anchor = cluster.center;
        track.stable_since_seconds = now_seconds;
      }
    }
    const double anchor_distance = std::hypot(
      cluster.center.x - track.stability_anchor.x,
      cluster.center.y - track.stability_anchor.y);
    if (anchor_distance > logic3_runtime_stability_radius_) {
      if (track.persistent) {
        RCLCPP_WARN(
          get_logger(),
          "逻辑3持久红簇重新移动：map(%.3f, %.3f)，降级为移动红簇但保持已锁定绕行方向。",
          cluster.center.x, cluster.center.y);
      }
      track.persistent = false;
      track.stability_anchor = cluster.center;
      track.stable_since_seconds = now_seconds;
    }
    track.latest_center = cluster.center;
    track.last_seen_seconds = now_seconds;
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
      Point2D shift_direction;
      if (logic3_shift_direction_on_route_range(
          track.cluster, route_first_index, route_last_index, shift_direction))
      {
        track.shift_direction = shift_direction;
        track.direction_valid = true;
        track.confirmed = true;
        RCLCPP_WARN(
          get_logger(),
          "逻辑3动态红簇确认：map(%.3f, %.3f)，连续%d帧，身份半径=%.2fm。",
          track.cluster.center.x, track.cluster.center.y,
          track.consecutive_hits, logic3_runtime_cluster_match_radius_);
      }
    }

    if (track.confirmed && !track.persistent &&
      now_seconds - track.stable_since_seconds >=
      logic3_runtime_stability_duration_seconds_)
    {
      track.persistent = true;
      RCLCPP_WARN(
        get_logger(),
        "逻辑3动态红簇已稳定：map(%.3f, %.3f)，升级为任务内持久红簇；绘图仍为红色。",
        track.cluster.center.x, track.cluster.center.y);
    }
    if (track.confirmed) {
      record_logic2_red_cluster(track.cluster);
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
        return !track.persistent &&
        track.missing_count >= waypoint_shift_release_scans_;
      }),
    logic3_runtime_cluster_tracks_.end());
}

void VisualLidarPathFollower::logic2_odom_callback(
  const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  if (waypoint_shift_logic_ < 2 || logic2_planning_pose_anchored_) {
    return;
  }

  const double odom_x = msg->pose.pose.position.x;
  const double odom_y = msg->pose.pose.position.y;
  if (!logic2_odom_reference_valid_) {
    logic2_odom_reference_valid_ = true;
    logic2_odom_reference_x_ = odom_x;
    logic2_odom_reference_y_ = odom_y;
    return;
  }

  const double speed = std::hypot(
    msg->twist.twist.linear.x, msg->twist.twist.linear.y);
  const double displacement = std::hypot(
    odom_x - logic2_odom_reference_x_, odom_y - logic2_odom_reference_y_);
  const bool moving = speed >= logic2_motion_speed_threshold_ ||
    displacement >= logic2_motion_displacement_threshold_;
  if (!moving) {
    const bool motion_candidate_was_active = logic2_motion_candidate_;
    if (logic2_motion_candidate_) {
      RCLCPP_INFO(
        get_logger(), "逻辑%d发车候选取消：速度%.3fm/s，累计位移%.3fm。",
        waypoint_shift_logic_, speed, displacement);
    }
    logic2_motion_candidate_ = false;
    logic2_motion_candidate_count_ = 0;
    if (waypoint_shift_logic_ == 3) {
      logic3_motion_pending_snapshot_ = Logic3PlanningSnapshot();
      if (motion_candidate_was_active) {
        // 候选期间没有把起步震动帧写入静态状态机；候选取消后继续用
        // 原2秒窗口和状态1有效样本，下一帧会按时间窗口正常淘汰旧数据。
        RCLCPP_INFO(
          get_logger(),
          "逻辑3发车候选取消：恢复静态绿色点更新，不清空既有ID与有效样本。");
      }
    }
    return;
  }

  if (!logic2_motion_candidate_) {
    logic2_motion_candidate_ = true;
    logic2_motion_candidate_count_ = 1;
    if (waypoint_shift_logic_ == 3) {
      // 第一条可信发车候选到来时立刻停止采用后续震动帧，只冻结此刻仍处于
      // 状态1且全部ID完成10个有效观测的READY结果，绝不回退到最后原始帧。
      logic3_motion_pending_snapshot_.valid =
        logic3_static_state1_active_ && logic3_static_ready_;
      logic3_motion_pending_snapshot_.stamp_seconds = now().seconds();
      logic3_motion_pending_snapshot_.obstacles = logic3_ready_static_obstacles();
    }
    RCLCPP_INFO(
      get_logger(),
      "逻辑%d检测到第1条发车候选：速度%.3fm/s，累计位移%.3fm。",
      waypoint_shift_logic_, speed, displacement);
  } else {
    ++logic2_motion_candidate_count_;
  }

  if (logic2_motion_candidate_count_ >= logic2_motion_confirm_messages_) {
    if (waypoint_shift_logic_ == 3) {
      freeze_logic3_route();
    } else {
      anchor_logic2_planning_pose();
    }
  }
}

std::string VisualLidarPathFollower::make_logic2_session_id() const
{
  const std::time_t timestamp = std::time(nullptr);
  std::tm local_time;
  localtime_r(&timestamp, &local_time);
  std::ostringstream stream;
  stream << std::put_time(&local_time, "%Y%m%d_%H%M%S");
  return stream.str();
}

void VisualLidarPathFollower::anchor_logic2_planning_pose()
{
  logic2_planning_pose_anchored_ = true;
  logic2_motion_candidate_ = false;
  logic2_session_id_ = make_logic2_session_id();
  logic2_last_pose_correction_at_ = std::chrono::steady_clock::now();

  if (logic2_planning_pose_source_ != "live_lidar_location") {
    RobotPose live_map_odom;
    if (lookup_pose(
        frame_id_, logic2_odom_frame_id_,
        rclcpp::Time(0, 0, get_clock()->get_clock_type()), live_map_odom))
    {
      logic2_map_odom_plan_ = live_map_odom;
      logic2_map_odom_plan_valid_ = true;
    }
  }

  if (!waiting_for_qr_start_ && !following_qr_path_ && !waypoints_.empty()) {
    initialize_logic2_route_visualization();
  }
  if (logic2_planning_pose_source_ == "live_lidar_location") {
    RCLCPP_WARN(
      get_logger(),
      "逻辑2可信发车已确认：后续继续使用实时全局定位并做通道内动态红簇避障，"
      "红簇身份半径=%.2fm，输出批次=%s。",
      logic2_red_history_merge_radius_, logic2_session_id_.c_str());
  } else {
    RCLCPP_WARN(
      get_logger(),
      "逻辑2可信发车已确认：规划位姿锚定完成，后续做通道内动态红簇避障，"
      "红簇身份半径=%.2fm，输出批次=%s。",
      logic2_red_history_merge_radius_, logic2_session_id_.c_str());
  }
}

void VisualLidarPathFollower::freeze_logic3_route()
{
  const Logic3PlanningSnapshot & snapshot = logic3_motion_pending_snapshot_;
  logic3_green_clusters_.clear();
  logic3_green_shift_directions_.clear();
  if (snapshot.valid) {
    for (const auto & obstacle : snapshot.obstacles) {
      logic3_green_clusters_.push_back(obstacle.cluster);
      logic3_green_shift_directions_.push_back(obstacle.shift_direction);
    }
    if (snapshot.obstacles.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "逻辑3可信发车时状态1/READY有效，但本轮确认的绿色簇数量为0；"
        "按规则不使用原始帧兜底，将冻结空快照。");
    }
  } else {
    RCLCPP_WARN(
      get_logger(),
      "逻辑3发车候选时状态1或READY无效；按规则不使用原始帧兜底，将记录为空。");
  }

  logic2_planning_pose_anchored_ = true;
  logic2_motion_candidate_ = false;
  logic2_runtime_red_tracks_.clear();
  logic3_runtime_cluster_tracks_.clear();
  logic2_red_cluster_history_.clear();
  logic2_final_snapshot_published_ = false;
  logic2_session_id_ = make_logic2_session_id();
  logic2_last_pose_correction_at_ = std::chrono::steady_clock::now();

  if (logic2_planning_pose_source_ != "live_lidar_location") {
    RobotPose live_map_odom;
    if (lookup_pose(
        frame_id_, logic2_odom_frame_id_,
        rclcpp::Time(0, 0, get_clock()->get_clock_type()), live_map_odom))
    {
      logic2_map_odom_plan_ = live_map_odom;
      logic2_map_odom_plan_valid_ = true;
    }
  }

  if (logic3_enable_tts_ && logic3_tts_pub_) {
    std_msgs::msg::String speech;
    speech.data = logic3_static_record_ready_text_ + "，共" +
      std::to_string(logic3_green_clusters_.size()) + "个";
    logic3_tts_pub_->publish(speech);
  }
  RCLCPP_WARN(
    get_logger(),
    "逻辑3静态障碍与偏移记录完毕：冻结%zu个绿色坐标及世界偏移方向，"
    "尚未生成方向路线和图片，输出批次=%s。",
    logic3_green_clusters_.size(),
    logic2_session_id_.c_str());

  // 兼容极少数先扫码后确认运动的时序：已有正式方向路线时立即完成规划。
  if (!waiting_for_qr_start_ && !following_qr_path_ && !waypoints_.empty()) {
    apply_logic3_frozen_clusters_to_current_route();
  }
}

void VisualLidarPathFollower::apply_logic3_frozen_clusters_to_current_route()
{
  if (waypoint_shift_logic_ != 3 || !logic2_planning_pose_anchored_) {
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
  // 静态回放只留下冻结基础偏移；清空其确认计数，确保后续新动态点云
  // 必须独立满足 waypoint_shift_confirm_scans，不能继承绿色簇的计数。
  std::fill(waypoint_shift_pending_offsets_.begin(), waypoint_shift_pending_offsets_.end(), 0.0);
  std::fill(waypoint_shift_pending_counts_.begin(), waypoint_shift_pending_counts_.end(), 0);
  std::fill(waypoint_shift_missing_counts_.begin(), waypoint_shift_missing_counts_.end(), 0);
  std::fill(
    waypoint_shift_dynamic_directions_.begin(),
    waypoint_shift_dynamic_directions_.end(), 0);
  logic2_baseline_original_route_ = original_waypoints_;
  logic2_baseline_route_ = waypoints_;
  logic2_visualization_route_ = waypoints_;
  logic2_visualization_offsets_ = waypoint_shift_offsets_;
  logic2_runtime_red_tracks_.clear();
  logic3_runtime_cluster_tracks_.clear();
  logic2_red_cluster_history_.clear();
  logic2_final_snapshot_published_ = false;
  publish_logic2_snapshot("selected");
  if (logic3_enable_tts_ && logic3_tts_pub_) {
    std_msgs::msg::String speech;
    speech.data = logic3_route_ready_text_;
    logic3_tts_pub_->publish(speech);
  }
  RCLCPP_WARN(
    get_logger(),
    "逻辑3路线已规划完毕：已将%zu个冻结绿色障碍按记录的世界偏移方向应用到%s路线。",
    logic3_green_clusters_.size(), selected_route_direction_.c_str());
}

void VisualLidarPathFollower::initialize_logic2_route_visualization()
{
  if (waypoint_shift_logic_ != 2 || !logic2_planning_pose_anchored_ ||
    waypoints_.empty())
  {
    return;
  }
  logic2_runtime_red_tracks_.clear();
  logic2_red_cluster_history_.clear();
  logic2_final_snapshot_published_ = false;
  logic2_baseline_original_route_ = original_waypoints_;
  logic2_baseline_route_ = waypoints_;
  logic2_visualization_route_ = waypoints_;
  logic2_visualization_offsets_ = waypoint_shift_offsets_;
  publish_logic2_snapshot("selected");
}

void VisualLidarPathFollower::update_logic2_visualization_route()
{
  if (logic2_visualization_route_.size() != waypoints_.size() ||
    logic2_visualization_offsets_.size() != waypoint_shift_offsets_.size())
  {
    logic2_visualization_route_ = waypoints_;
    logic2_visualization_offsets_ = waypoint_shift_offsets_;
    return;
  }
  double current_dynamic_score = 0.0;
  double saved_dynamic_score = 0.0;
  for (std::size_t index = 0; index < waypoints_.size(); ++index) {
    const double base = waypoint_shift_logic_ == 3 &&
      logic3_frozen_base_offsets_.size() == waypoints_.size() ?
      logic3_frozen_base_offsets_[index] : 0.0;
    current_dynamic_score += std::abs(waypoint_shift_offsets_[index] - base);
    saved_dynamic_score += std::abs(logic2_visualization_offsets_[index] - base);
  }
  // 最终图保留“动态偏移总量最大”的一个完整时刻。禁止逐航点分别取
  // 历史最大值，否则会把不同帧、甚至不同绕行方向拼成从未存在过的交叉路线。
  if (current_dynamic_score > saved_dynamic_score + 1e-9) {
    logic2_visualization_route_ = waypoints_;
    logic2_visualization_offsets_ = waypoint_shift_offsets_;
  }
}

void VisualLidarPathFollower::publish_logic2_snapshot(const std::string & event)
{
  if (!logic2_snapshot_pub_ || logic2_session_id_.empty()) {
    return;
  }
  const std::vector<Waypoint> & plotted_route = event == "final" ?
    logic2_visualization_route_ : logic2_baseline_route_;
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

  std::ostringstream json;
  json << std::setprecision(10);
  json << "{\"logic\":" << waypoint_shift_logic_
       << ",\"event\":\"" << json_escape(event)
       << "\",\"session\":\"" << json_escape(logic2_session_id_)
       << "\",\"output_root\":\"" << json_escape(logic2_output_root_)
       << "\",\"map_yaml\":\"" << json_escape(logic2_restriction_map_yaml_)
       << "\",\"red_identity_radius\":" << logic2_red_history_merge_radius_
       << ",\"green_radius\":" << logic3_old_cluster_radius_
       << ",\"original\":";
  append_route(json, logic2_baseline_original_route_);
  json << ",\"baseline\":";
  append_route(json, logic2_baseline_route_);
  json << ",\"route\":";
  append_route(json, plotted_route);
  json << ",\"green\":";
  append_clusters(
    json, waypoint_shift_logic_ == 3 ? logic3_green_clusters_ :
    std::vector<ScanPointCluster>());
  json << ",\"red\":";
  append_points(json, event == "final" ? logic2_red_cluster_history_ : std::vector<Point2D>());
  json << '}';

  std_msgs::msg::String message;
  message.data = json.str();
  logic2_snapshot_pub_->publish(message);
}

bool VisualLidarPathFollower::lookup_logic2_planning_pose(
  VisualLidarPathFollower::RobotPose & pose)
{
  const auto latest_stamp = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  if (!logic2_planning_pose_anchored_ ||
    logic2_planning_pose_source_ == "live_lidar_location")
  {
    return lookup_pose(frame_id_, base_frame_id_, latest_stamp, pose);
  }

  RobotPose odom_to_base;
  if (!lookup_pose(logic2_odom_frame_id_, base_frame_id_, latest_stamp, odom_to_base)) {
    return false;
  }
  RobotPose live_map_odom;
  const bool has_live_map_odom = lookup_pose(
    frame_id_, logic2_odom_frame_id_, latest_stamp, live_map_odom);
  if (!logic2_map_odom_plan_valid_) {
    if (!has_live_map_odom) {
      return false;
    }
    logic2_map_odom_plan_ = live_map_odom;
    logic2_map_odom_plan_valid_ = true;
    logic2_last_pose_correction_at_ = std::chrono::steady_clock::now();
  }

  if (logic2_planning_pose_source_ == "filtered_map_odom" && has_live_map_odom) {
    const auto current_time = std::chrono::steady_clock::now();
    const double dt = std::max(
      0.0, std::chrono::duration<double>(
        current_time - logic2_last_pose_correction_at_).count());
    logic2_last_pose_correction_at_ = current_time;
    const double dx = live_map_odom.x - logic2_map_odom_plan_.x;
    const double dy = live_map_odom.y - logic2_map_odom_plan_.y;
    const double translation_error = std::hypot(dx, dy);
    const double yaw_error = normalize_angle(
      live_map_odom.yaw - logic2_map_odom_plan_.yaw);
    if (translation_error <= logic2_pose_outlier_xy_ &&
      std::abs(yaw_error) <= logic2_pose_outlier_yaw_)
    {
      if (translation_error > logic2_pose_correction_deadband_xy_ && dt > 0.0) {
        const double step = std::min(
          translation_error, logic2_pose_correction_rate_xy_ * dt);
        logic2_map_odom_plan_.x += step * dx / translation_error;
        logic2_map_odom_plan_.y += step * dy / translation_error;
      }
      if (std::abs(yaw_error) > logic2_pose_correction_deadband_yaw_ && dt > 0.0) {
        const double step = std::min(
          std::abs(yaw_error), logic2_pose_correction_rate_yaw_ * dt);
        logic2_map_odom_plan_.yaw = normalize_angle(
          logic2_map_odom_plan_.yaw + std::copysign(step, yaw_error));
      }
    }
  }

  const double cos_yaw = std::cos(logic2_map_odom_plan_.yaw);
  const double sin_yaw = std::sin(logic2_map_odom_plan_.yaw);
  pose.x = logic2_map_odom_plan_.x +
    cos_yaw * odom_to_base.x - sin_yaw * odom_to_base.y;
  pose.y = logic2_map_odom_plan_.y +
    sin_yaw * odom_to_base.x + cos_yaw * odom_to_base.y;
  pose.yaw = normalize_angle(logic2_map_odom_plan_.yaw + odom_to_base.yaw);
  return true;
}

// waypoint_shift 模式：每帧新雷达提前检查当前目标之后的固定数量航点。
// 以障碍物最近路线点为中心，前后分别取配置数量的点并横移。
// 逻辑3向障碍物相对路线位置的反侧绕行；逻辑1/2保留地图空旷侧规则。
// 横移量先作为本帧候选，连续确认后才提交；障碍消失或错误观测消失后会释放回原路线。
void VisualLidarPathFollower::update_waypoint_shifts(
  const VisualLidarPathFollower::RobotPose & robot_pose,
  std::size_t target_waypoint_index)
{
  const bool replay_frozen_clusters =
    waypoint_shift_logic_ == 3 && logic3_replay_frozen_clusters_;
  if (waypoint_shift_logic_ == 2 && !logic2_planning_pose_anchored_) {
    return;
  }
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

  const int minimum_cluster_points = waypoint_shift_logic_ == 3 &&
    !logic2_planning_pose_anchored_ ? logic3_static_cluster_min_points_ :
    cluster_min_points_;
  const bool use_logic3_static_restriction_map =
    waypoint_shift_logic_ == 3 && !logic2_planning_pose_anchored_ &&
    !replay_frozen_clusters;
  std::vector<ScanPointCluster> clusters = replay_frozen_clusters ?
    logic3_green_clusters_ : collect_non_static_scan_clusters_map(
    robot_pose, minimum_cluster_points, use_logic3_static_restriction_map);

  // 远距离时同一障碍物的1~3个回波可能被无效量程切成多个单点段。
  // 仅在逻辑3静止阶段用较小的静态匹配半径合并当前帧候选；不使用0.5m冻结圆。
  if (waypoint_shift_logic_ == 3 && !logic2_planning_pose_anchored_ &&
    !replay_frozen_clusters)
  {
    std::vector<ScanPointCluster> merged_clusters;
    for (const auto & cluster : clusters) {
      auto merged = std::find_if(
        merged_clusters.begin(), merged_clusters.end(),
        [&](const ScanPointCluster & candidate) {
          return std::hypot(
            candidate.center.x - cluster.center.x,
            candidate.center.y - cluster.center.y) <= logic3_static_cluster_match_radius_;
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
    // 静态障碍通道从左右黑色禁行块底边开始；更低的白色区域属于大厅。
    // 该门限只约束发车前绿色候选，不影响入口追点和运行中动态红簇。
    clusters.erase(
      std::remove_if(
        clusters.begin(), clusters.end(), [&](const ScanPointCluster & cluster) {
          return cluster.center.y <= logic3_static_channel_min_y_;
        }),
      clusters.end());
  }

  // 逻辑2/3运行阶段只处理各自配置的车体中心ROI。逻辑3只保留通道y下限以上
  // 的动态红簇，并抑制冻结绿色圆内的旧簇；红色图片历史半径仅合并记录，
  // 不屏蔽障碍物逐帧参与动态避障。
  if (waypoint_shift_logic_ >= 2 && logic2_planning_pose_anchored_ &&
    !replay_frozen_clusters)
  {
    const double half_width = 0.5 * logic2_runtime_roi_width_;
    const double half_height = 0.5 * logic2_runtime_roi_height_;
    const double old_radius_squared =
      logic3_old_cluster_radius_ * logic3_old_cluster_radius_;
    clusters.erase(
      std::remove_if(
        clusters.begin(), clusters.end(), [&](const ScanPointCluster & cluster) {
          if (std::abs(cluster.center.x - robot_pose.x) > half_width ||
          std::abs(cluster.center.y - robot_pose.y) > half_height)
          {
            return true;
          }
          if (waypoint_shift_logic_ != 3) {
            return false;
          }
          if (cluster.center.y <= logic3_runtime_channel_min_y_) {
            return true;
          }
          return std::any_of(
            logic3_green_clusters_.begin(), logic3_green_clusters_.end(),
            [&](const ScanPointCluster & old_cluster) {
              const double dx = cluster.center.x - old_cluster.center.x;
              const double dy = cluster.center.y - old_cluster.center.y;
              return dx * dx + dy * dy <= old_radius_squared;
            });
        }),
      clusters.end());
  }

  const bool logic3_runtime_dynamic = waypoint_shift_logic_ == 3 &&
    logic2_planning_pose_anchored_ && !replay_frozen_clusters;
  // lookahead目标在车前方；从已经提交的路线进度开始检查，不能把目标点
  // 之前、但车辆尚未真正驶过的障碍误判为“已通过”。
  const std::size_t runtime_first_index = std::min(
    std::min(current_index_, target_waypoint_index), waypoints_.size() - 1);
  const std::size_t runtime_last_index = std::min(
    waypoints_.size(), runtime_first_index +
    static_cast<std::size_t>(waypoint_shift_check_count_));

  // 逻辑3运行阶段先在障碍物身份层完成关联、确认和中心平滑。控制只消费
  // 已确认轨迹；绿色冻结约束与红色运行约束随后进入同一次全路线求解。
  std::vector<Point2D> planning_shift_directions;
  std::vector<bool> planning_direction_valid;
  std::vector<bool> planning_allow_entry_exit_split;
  std::vector<bool> planning_is_green;
  if (replay_frozen_clusters) {
    planning_shift_directions.resize(clusters.size());
    planning_direction_valid.assign(clusters.size(), false);
    for (std::size_t index = 0;
      index < clusters.size() && index < logic3_green_shift_directions_.size(); ++index)
    {
      planning_shift_directions[index] = logic3_green_shift_directions_[index];
      planning_direction_valid[index] = true;
    }
    planning_allow_entry_exit_split.assign(clusters.size(), true);
    planning_is_green.assign(clusters.size(), true);
  } else if (logic3_runtime_dynamic) {
    std::vector<ScanPointCluster> route_clusters;
    for (const auto & cluster : clusters) {
      if (logic2_cluster_hits_route_window(
          cluster, runtime_first_index, runtime_last_index))
      {
        route_clusters.push_back(cluster);
      }
    }
    update_logic3_runtime_red_tracks(
      route_clusters, runtime_first_index, runtime_last_index);

    clusters.clear();
    for (std::size_t index = 0; index < logic3_green_clusters_.size(); ++index) {
      clusters.push_back(logic3_green_clusters_[index]);
      planning_shift_directions.push_back(
        index < logic3_green_shift_directions_.size() ?
        logic3_green_shift_directions_[index] : Point2D());
      planning_direction_valid.push_back(
        index < logic3_green_shift_directions_.size());
      planning_allow_entry_exit_split.push_back(true);
      planning_is_green.push_back(true);
    }
    for (const auto & track : logic3_runtime_cluster_tracks_) {
      if (!track.confirmed || !track.direction_valid) {
        continue;
      }
      clusters.push_back(track.cluster);
      planning_shift_directions.push_back(track.shift_direction);
      planning_direction_valid.push_back(true);
      planning_allow_entry_exit_split.push_back(track.persistent);
      planning_is_green.push_back(false);
    }
  }

  const bool full_route = waypoint_shift_logic_ == 3 &&
    (logic3_force_full_route_ || replay_frozen_clusters || logic3_runtime_dynamic);
  const std::size_t first_index = full_route ? 0 :
    std::min(target_waypoint_index, waypoints_.size() - 1);
  const std::size_t last_index = full_route ? waypoints_.size() : std::min(
    waypoints_.size(), first_index + static_cast<std::size_t>(waypoint_shift_check_count_));
  const std::size_t managed_last_index = full_route ? waypoints_.size() - 1 : std::min(
    waypoints_.size() - 1,
    first_index + static_cast<std::size_t>(waypoint_shift_check_count_) +
    static_cast<std::size_t>(waypoint_shift_points_after_));
  const double check_radius_squared =
    waypoint_cloud_check_radius_ * waypoint_cloud_check_radius_;

  struct ShiftCandidateSegment
  {
    std::size_t obstacle_waypoint{0};
    std::size_t first{0};
    std::size_t last{0};
    double left_distance{0.0};
    double right_distance{0.0};
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

  // channel 只有入口首段和出口尾段在低处空间重合。冻结绿色簇回放整条路线时，
  // 由高度自动截出首尾两段，允许同一障碍分别作用于两个相距很远的路线进度。
  std::size_t entrance_segment_last = 0;
  std::size_t exit_segment_first = original_waypoints_.size() - 1;
  bool entry_exit_ranges_valid = false;
  if (waypoint_shift_logic_ == 3 &&
    (replay_frozen_clusters || logic3_runtime_dynamic))
  {
    while (entrance_segment_last < original_waypoints_.size() &&
      original_waypoints_[entrance_segment_last].y <= logic3_entry_exit_overlap_max_y_)
    {
      ++entrance_segment_last;
    }
    while (exit_segment_first > 0 &&
      original_waypoints_[exit_segment_first].y <= logic3_entry_exit_overlap_max_y_)
    {
      --exit_segment_first;
    }
    entry_exit_ranges_valid =
      original_waypoints_.front().y <= logic3_entry_exit_overlap_max_y_ &&
      original_waypoints_.back().y <= logic3_entry_exit_overlap_max_y_ &&
      entrance_segment_last > 0 &&
      entrance_segment_last < original_waypoints_.size() &&
      exit_segment_first < original_waypoints_.size() - 1 &&
      entrance_segment_last <= exit_segment_first;
  }

  auto find_best_route_segment = [&](
    const ScanPointCluster & cluster, std::size_t segment_first,
    std::size_t segment_last, RouteSegmentMatch & match)
    {
      match = RouteSegmentMatch();
      segment_last = std::min(segment_last, original_waypoints_.size() - 1);
      if (segment_first >= segment_last) {
        return false;
      }
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
        if (distance_squared < match.distance_squared) {
          const double length = std::sqrt(length_squared);
          match.valid = true;
          match.segment = index;
          match.t = projection_t;
          match.distance_squared = distance_squared;
          match.tangent = Point2D{segment_x / length, segment_y / length};
          match.projection = Point2D{projection_x, projection_y};
        }
      }
      return match.valid;
    };

  if (waypoint_shift_logic_ == 3 && !logic2_planning_pose_anchored_ &&
    !replay_frozen_clusters)
  {
    // 第一条发车候选已经保存READY平均快照后，停止接收后续可能包含底盘
    // 震动的静态帧；候选若取消，下一帧会恢复观察并按连续性规则重建。
    if (logic2_motion_candidate_) {
      return;
    }
    std::vector<ScanPointCluster> route_clusters;
    for (const auto & cluster : clusters) {
      if (logic2_cluster_hits_route_window(cluster, first_index, last_index)) {
        route_clusters.push_back(cluster);
      }
    }
    update_logic3_static_cluster_tracks(route_clusters, robot_pose);
    const auto ready_obstacles = logic3_ready_static_obstacles();
    maybe_log_logic3_static_obstacles(route_clusters.size(), ready_obstacles);
    // 未扫码时只记录绿色坐标和世界偏移方向，不改动任何正式路径点。
    return;
  } else if (waypoint_shift_logic_ == 2 && logic2_planning_pose_anchored_ &&
    !replay_frozen_clusters)
  {
    std::vector<ScanPointCluster> route_clusters;
    for (const auto & cluster : clusters) {
      if (logic2_cluster_hits_route_window(cluster, first_index, last_index)) {
        route_clusters.push_back(cluster);
      }
    }
    update_logic2_runtime_red_tracks(route_clusters);
  }

  for (std::size_t cluster_index = 0; cluster_index < clusters.size(); ++cluster_index) {
    const auto & cluster = clusters[cluster_index];
    const bool has_logic3_metadata =
      cluster_index < planning_direction_valid.size();
    const bool cluster_is_green = has_logic3_metadata && planning_is_green[cluster_index];
    const bool allow_entry_exit_split = has_logic3_metadata &&
      planning_allow_entry_exit_split[cluster_index];
    std::size_t cluster_route_first = first_index;
    std::size_t cluster_route_last = last_index;
    if (logic3_runtime_dynamic && !cluster_is_green) {
      cluster_route_first = runtime_first_index;
      cluster_route_last = allow_entry_exit_split ?
        original_waypoints_.size() : runtime_last_index;
    }

    const bool cluster_hits_window = std::any_of(
      cluster.points.begin(), cluster.points.end(), [&](const Point2D & point) {
        for (std::size_t index = cluster_route_first;
        index < cluster_route_last; ++index)
        {
          const double dx = point.x - original_waypoints_[index].x;
          const double dy = point.y - original_waypoints_[index].y;
          if (dx * dx + dy * dy <= check_radius_squared) {
            return true;
          }
          if (logic3_runtime_dynamic && waypoints_.size() == original_waypoints_.size()) {
            const double active_dx = point.x - waypoints_[index].x;
            const double active_dy = point.y - waypoints_[index].y;
            if (active_dx * active_dx + active_dy * active_dy <= check_radius_squared) {
              return true;
            }
          }
        }
        return false;
      });
    if (!cluster_hits_window) {
      continue;
    }

    // 普通动态红簇只匹配当前窗口。冻结绿色簇和已稳定的持久红簇会分别
    // 尝试入口、出口两条分支；是否命中完全由点云到分支的距离决定，不再
    // 用簇中心y值提前排除通道口障碍。
    const std::size_t segment_first = cluster_route_first > 0 ?
      cluster_route_first - 1 : cluster_route_first;
    const std::size_t segment_last = std::min(
      cluster_route_last, original_waypoints_.size() - 1);
    std::vector<RouteSegmentMatch> route_matches;
    const bool split_entry_exit_match = allow_entry_exit_split &&
      entry_exit_ranges_valid;
    if (split_entry_exit_match) {
      RouteSegmentMatch entrance_match;
      const bool entrance_not_passed = cluster_is_green ||
        cluster_route_first <= entrance_segment_last;
      if (entrance_not_passed && find_best_route_segment(
          cluster, cluster_is_green ? 0 : cluster_route_first,
          entrance_segment_last, entrance_match) &&
        entrance_match.distance_squared <= check_radius_squared)
      {
        route_matches.push_back(entrance_match);
      }
      RouteSegmentMatch exit_match;
      if (find_best_route_segment(
          cluster, std::max(exit_segment_first, cluster_route_first),
          original_waypoints_.size() - 1, exit_match) &&
        exit_match.distance_squared <= check_radius_squared)
      {
        route_matches.push_back(exit_match);
      }
    }
    if (route_matches.empty()) {
      RouteSegmentMatch nearest_match;
      if (find_best_route_segment(
          cluster, segment_first, segment_last, nearest_match))
      {
        route_matches.push_back(nearest_match);
      }
    }
    if (debug_control_logs_ && route_matches.size() > 1) {
      RCLCPP_INFO(
        get_logger(),
        "debug: 逻辑3入口/出口重合区%s簇(%.2f, %.2f)命中%zu条路线分支。",
        cluster_is_green ? "绿色" : "持久红色",
        cluster.center.x, cluster.center.y, route_matches.size());
    }

    for (const auto & route_match : route_matches) {
      const Point2D & obstacle_point = cluster.center;
      const Point2D left_normal{-route_match.tangent.y, route_match.tangent.x};
      const Point2D right_normal{-left_normal.x, -left_normal.y};
      double left_distance = 0.0;
      double right_distance = 0.0;
      if (waypoint_shift_logic_ != 3 &&
        (!map_distance_to_blocked_along_ray(
          obstacle_point, left_normal, left_distance) ||
        !map_distance_to_blocked_along_ray(
          obstacle_point, right_normal, right_distance)))
      {
        continue;
      }
      // 每个入口/出口匹配都使用自己的路线投影和法向量计算偏移量。
      auto required_offset_along = [&](const Point2D & shift_normal) {
          double cloud_edge_along_shift = -std::numeric_limits<double>::infinity();
          for (const auto & point : cluster.points) {
            const double lateral_distance =
              (point.x - route_match.projection.x) * shift_normal.x +
              (point.y - route_match.projection.y) * shift_normal.y;
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
      int preferred_direction = 0;
      if (has_logic3_metadata && planning_direction_valid[cluster_index]) {
        const auto & recorded_direction = planning_shift_directions[cluster_index];
        const double left_alignment =
          recorded_direction.x * left_normal.x + recorded_direction.y * left_normal.y;
        preferred_direction = left_alignment >= 0.0 ? 1 : -1;
      } else if (waypoint_shift_logic_ == 3) {
        const double obstacle_lateral =
          (cluster.center.x - route_match.projection.x) * left_normal.x +
          (cluster.center.y - route_match.projection.y) * left_normal.y;
        // +1表示路线左移，-1表示路线右移；始终选择障碍所在侧的反方向。
        preferred_direction = obstacle_lateral >= 0.0 ? -1 : 1;
      }

      std::size_t obstacle_waypoint = route_match.t < 0.5 ?
        route_match.segment : route_match.segment + 1;
      obstacle_waypoint = std::max(
        cluster_route_first,
        std::min(obstacle_waypoint, original_waypoints_.size() - 1));
      const std::size_t points_before =
        static_cast<std::size_t>(waypoint_shift_points_before_);
      const std::size_t points_after =
        static_cast<std::size_t>(waypoint_shift_points_after_);
      const std::size_t influence_first = std::max(
        cluster_route_first,
        obstacle_waypoint > points_before ? obstacle_waypoint - points_before : 0);
      const std::size_t influence_last = std::min(
        std::min(original_waypoints_.size() - 1, cluster_route_last - 1),
        obstacle_waypoint + points_after);
      candidate_segments.push_back(
        ShiftCandidateSegment{
        obstacle_waypoint, influence_first, influence_last, left_distance, right_distance,
        left_required_offset, right_required_offset, preferred_direction, left_normal});
    }
  }

  // 逻辑3按路线进度依次应用每个障碍物。相邻障碍物窗口重叠时，
  // 前1/4保留前一个，后3/4由后一个覆盖。逻辑1/2保留原冲突组规则。
  std::sort(
    candidate_segments.begin(), candidate_segments.end(),
    [&](const ShiftCandidateSegment & lhs, const ShiftCandidateSegment & rhs) {
      if (waypoint_shift_logic_ == 3 &&
      lhs.obstacle_waypoint != rhs.obstacle_waypoint)
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
  if (waypoint_shift_logic_ == 3) {
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
          "debug: 逻辑3四分之一覆盖：障碍点=%zu，原窗口=%zu-%zu，实际写入=%zu-%zu，"
          "向%s偏移%.3fm，重叠=%zu个，覆盖前障碍=%zu个。",
          segment.obstacle_waypoint + 1, segment.first + 1, segment.last + 1,
          applied_first + 1, segment.last + 1,
          segment.preferred_direction > 0 ? "左" : "右", required_offset,
          overlap_count, overwritten_count);
      }
    }
  }

  // 逻辑3已经按障碍顺序生成候选；下列冲突组仅处理逻辑1/2。
  std::size_t group_begin = waypoint_shift_logic_ == 3 ? candidate_segments.size() : 0;
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
    std::size_t preferred_left_count = 0;
    std::size_t preferred_right_count = 0;
    for (std::size_t segment_index = group_begin;
      segment_index < group_end; ++segment_index)
    {
      if (candidate_segments[segment_index].preferred_direction > 0) {
        ++preferred_left_count;
      } else if (candidate_segments[segment_index].preferred_direction < 0) {
        ++preferred_right_count;
      }
    }
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

    const bool has_preferred_direction =
      preferred_left_count + preferred_right_count > 0;
    int shift_direction = 0;
    const char * direction_source = "空旷侧";
    // 逻辑3按障碍物相对路线的左右位置选择反侧；有符号pending负责首次
    // 方向确认和反向重确认，同方向提交后持续刷新偏移量。
    if (has_preferred_direction) {
      if (preferred_left_count == preferred_right_count) {
        shift_direction = candidate_segments[group_begin].preferred_direction;
      } else {
        shift_direction = preferred_left_count > preferred_right_count ? 1 : -1;
      }
      direction_source = replay_frozen_clusters ?
        "静态快照路线反侧" : "障碍相对路线反侧";
    } else if (!logic3_runtime_dynamic &&
      committed_left_count != committed_right_count)
    {
      shift_direction = committed_left_count > committed_right_count ? 1 : -1;
      direction_source = "已提交段锁定";
    } else if (!logic3_runtime_dynamic && committed_left_count > 0 &&
      std::abs(committed_left_magnitude - committed_right_magnitude) > 1e-9)
    {
      shift_direction = committed_left_magnitude > committed_right_magnitude ? 1 : -1;
      direction_source = "已提交段锁定";
    } else if (!logic3_runtime_dynamic &&
      pending_left_score != pending_right_score)
    {
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
      const Point2D world_direction = shift_direction > 0 ?
        segment.left_normal : Point2D{-segment.left_normal.x, -segment.left_normal.y};
      for (std::size_t index = segment.first; index <= segment.last; ++index) {
        if (!observed_offsets_valid[index]) {
          observed_offsets[index] = candidate;
          observed_offsets_valid[index] = true;
          observed_world_directions[index] = world_direction;
          observed_source_indices[index] = segment.obstacle_waypoint;
          observed_geometry_metadata_valid[index] = true;
          ++candidate_count;
        } else if (std::abs(candidate) > std::abs(observed_offsets[index]) + 1e-9) {
          observed_offsets[index] = candidate;
          observed_world_directions[index] = world_direction;
          observed_source_indices[index] = segment.obstacle_waypoint;
          observed_geometry_metadata_valid[index] = true;
        }
      }
    }

    if (debug_control_logs_ && candidate_count > 0 && has_preferred_direction) {
      RCLCPP_INFO(
        get_logger(),
        "debug: waypoint_shift 偏移组：簇段%zu个，覆盖点=%zu-%zu，"
        "向%s（%s），各段偏移=%.3f-%.3fm，候选%zu个点。",
        group_end - group_begin, group_first + 1, group_last + 1,
        shift_direction > 0 ? "左" : "右", direction_source,
        min_required_offset, max_required_offset, candidate_count);
    } else if (debug_control_logs_ && candidate_count > 0) {
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
    if (logic3_runtime_dynamic) {
      const double candidate = observed_offsets_valid[index] ?
        observed_offsets[index] : 0.0;
      const double base_offset = logic3_frozen_base_offsets_.size() ==
        waypoint_shift_offsets_.size() ? logic3_frozen_base_offsets_[index] : 0.0;
      const double dynamic_candidate = candidate - base_offset;
      const double change = candidate - waypoint_shift_offsets_[index];
      waypoint_shift_missing_counts_[index] = 0;
      waypoint_shift_pending_offsets_[index] = candidate;
      waypoint_shift_pending_counts_[index] = waypoint_shift_confirm_scans_;

      if (std::abs(change) >= waypoint_shift_min_change_) {
        double applied_offset = candidate;
        // 首次确认直接取得安全偏移；后续中心细动及释放阶段才限速，避免
        // 单帧定位抖动让整段路线来回跳。
        if (waypoint_shift_dynamic_directions_[index] != 0 &&
          logic3_runtime_offset_max_step_ > 0.0 &&
          std::abs(change) > logic3_runtime_offset_max_step_)
        {
          applied_offset = waypoint_shift_offsets_[index] +
            std::copysign(logic3_runtime_offset_max_step_, change);
        }
        waypoint_shift_offsets_[index] = applied_offset;
        if (observed_geometry_metadata_valid[index] && std::abs(candidate) > 1e-9) {
          waypoint_shift_world_directions_[index] = observed_world_directions[index];
          waypoint_shift_source_indices_[index] = observed_source_indices[index];
        }
        route_changed = true;
        ++confirmed_count;
      } else if (std::abs(waypoint_shift_offsets_[index]) > 1e-9 &&
        observed_geometry_changed(index))
      {
        // 偏移量不变时仍允许稳定障碍来源移动超过4cm后刷新局部曲线。
        waypoint_shift_world_directions_[index] = observed_world_directions[index];
        waypoint_shift_source_indices_[index] = observed_source_indices[index];
        route_changed = true;
      }
      if (std::abs(dynamic_candidate) > 1e-9) {
        waypoint_shift_dynamic_directions_[index] = dynamic_candidate > 0.0 ? 1 : -1;
      } else if (std::abs(candidate - waypoint_shift_offsets_[index]) <
        waypoint_shift_min_change_)
      {
        waypoint_shift_dynamic_directions_[index] = 0;
      }
      if (std::abs(waypoint_shift_offsets_[index]) <= 1e-9) {
        waypoint_shift_world_directions_[index] = Point2D();
        waypoint_shift_source_indices_[index] = waypoints_.size();
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
    const double base_offset = waypoint_shift_logic_ == 3 &&
      logic2_planning_pose_anchored_ &&
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
    if (waypoint_shift_logic_ >= 2 && logic2_planning_pose_anchored_ &&
      !replay_frozen_clusters)
    {
      update_logic2_visualization_route();
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
      // visual_control 从此不再参与巡线/回线；QR到channel已改为雷达避障，立即暂停YOLO。
      suspend_visual_control();
      set_perception_mode(perception_pause_value_, "暂停");
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
  if (waypoint_shift_logic_ == 2 && logic2_planning_pose_anchored_) {
    initialize_logic2_route_visualization();
  } else if (waypoint_shift_logic_ == 3 && logic2_planning_pose_anchored_) {
    apply_logic3_frozen_clusters_to_current_route();
  }
  waiting_for_qr_start_ = false;
  following_qr_path_ = false;
  qr_path_finished_ = false;
  image_analysis_triggered_ = false;
  if (was_following_qr_path || was_waiting_for_qr) {
    current_index_ = 0;
    first_waypoint_reached_ = false;
    qr_channel_approach_active_ = true;
    qr_escape_state_ = QrEscapeState::PENDING;
  } else {
    current_index_ = std::min(current_index_, waypoints_.size() - 1);
    first_waypoint_reached_ = current_index_ > 0;
  }
  const auto route_name = route_direction_name(selected_route_direction_);
  RCLCPP_INFO(
    get_logger(),
    "QR 选择%s路线；先执行 zhy 条件倒车，再用 zhy 雷达避障追 waypoint_channel。",
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
  const bool pose_ready = waypoint_shift_logic_ >= 2 ?
    lookup_logic2_planning_pose(pose) :
    lookup_robot_pose_at(rclcpp::Time(0, 0, get_clock()->get_clock_type()), pose);
  if (!pose_ready) {
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
