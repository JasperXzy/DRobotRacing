#include "origincar_system/visual_lidar_path_follower.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include "yaml-cpp/yaml.h"
#include "follower_utils.hpp"

using origincar_system::to_lower;

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

}  // namespace

void VisualLidarPathFollower::read_parameters()
{
  read_tracking_parameters();
  read_static_planning_parameters();
  read_channel_transition_parameters();
  read_stage_parameters();
  normalize_parameters();
}

void VisualLidarPathFollower::read_tracking_parameters()
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
}

void VisualLidarPathFollower::read_static_planning_parameters()
{
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
}

void VisualLidarPathFollower::read_channel_transition_parameters()
{
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
}

void VisualLidarPathFollower::read_stage_parameters()
{
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
}

void VisualLidarPathFollower::normalize_parameters()
{
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
