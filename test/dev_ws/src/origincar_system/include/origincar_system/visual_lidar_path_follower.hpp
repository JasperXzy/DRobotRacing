#ifndef ORIGINCAR_SYSTEM_VISUAL_LIDAR_PATH_FOLLOWER_HPP_
#define ORIGINCAR_SYSTEM_VISUAL_LIDAR_PATH_FOLLOWER_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "ai_msgs/msg/perception_targets.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

struct Waypoint
{
  std::string name;
  double x{0.0};
  double y{0.0};
  bool reverse{false};
  bool image_capture{false};
  bool task3_start{false};
  double pass_radius{0.0};
  double reverse_pass_radius{0.0};
};

class VisualLidarPathFollower : public rclcpp::Node
{
public:
  VisualLidarPathFollower();

  void publish_stop();
  bool should_publish_stop_on_exit() const;

private:
  enum class ControlState
  {
    WAITING_FOR_POSE,
    WAITING_FOR_QR,
    PATH_FOLLOWING,
    RETURNING_TO_START,
    FINISHED
  };

  enum class QrEscapeState
  {
    IDLE,
    PENDING,
    BACKING_RIGHT
  };

  enum class TaskPhase
  {
    UNKNOWN,
    TASK1,
    TRANSITION,
    TASK2,
    TASK3,
    FINISHED
  };

  enum class ReturnLidarState
  {
    FOLLOWING_ROUTE,
    AVOIDING,
    REJOINING
  };

  struct RobotPose
  {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
  };

  struct Point2D
  {
    double x{0.0};
    double y{0.0};
  };

  struct RouteProjection
  {
    bool valid{false};
    std::size_t segment_index{0};
    double t{0.0};
    double distance{0.0};
    Point2D point;
  };

  struct TargetPoint
  {
    Point2D point;
    std::size_t waypoint_index{0};
    bool reverse{false};
  };

  struct LidarCluster
  {
    std::vector<Point2D> points;
    Point2D center;
    double fitted_radius{0.0};
    double fit_rmse{0.0};
    int point_count{0};
  };

  struct ScanPointCluster
  {
    std::vector<Point2D> points;
    Point2D center;
  };

  struct LidarObstacle
  {
    bool found{false};
    double range{0.0};
    double relative_angle{0.0};
  };

  struct Logic3StaticObstacle
  {
    ScanPointCluster cluster;
  };

  struct Logic3StaticClusterTrack
  {
    std::uint64_t identity_id{0};
    ScanPointCluster cluster;
    Point2D stability_anchor;
    bool confirmed{false};
    // 已通过“最近2秒至少10帧”并获得正式ID。只有全局正式ID集合在本帧
    // 无新增、删除或位置更新时，才由整体状态机进入状态1。
    bool entered_state1{false};
    bool loss_pending{false};
    bool ready{false};
    double last_seen_seconds{0.0};
    double loss_started_seconds{0.0};
    std::deque<double> rolling_hit_times;
    std::deque<Point2D> rolling_hit_centers;
    int state1_sample_count{0};
    double sample_center_sum_x{0.0};
    double sample_center_sum_y{0.0};
    std::vector<Point2D> sampled_points;
    Logic3StaticObstacle ready_obstacle;
  };

  // 发车后逻辑3使用独立的小半径身份跟踪，仅用于红点历史记录和绘图；
  // 红簇不再参与航点偏移，实时避障统一交给雷达控制。
  struct Logic3RuntimeClusterTrack
  {
    ScanPointCluster cluster;
    Point2D latest_center;
    bool confirmed{false};
    int consecutive_hits{0};
    int missing_count{0};
    std::deque<ScanPointCluster> recent_samples;
  };

  struct Logic3PlanningSnapshot
  {
    bool valid{false};
    double stamp_seconds{0.0};
    std::vector<Logic3StaticObstacle> obstacles;
  };

  struct StageControlProfile
  {
    double linear_speed{0.85};
    double lookahead_distance{0.65};
    double turn_angular_gain{1.60};
    bool enable_turn_speed_slowdown{true};
    double max_turn_speed_reduction_percent{20.0};
    double pass_radius{0.60};
    double route_projection_max_distance{0.60};
    double segment_advance_t{0.60};
    double lidar_avoid_distance{0.65};
    double lidar_avoid_fov_deg{160.0};
    double lidar_corridor_half_width{0.45};
    double lidar_avoid_linear_speed{0.45};
    double lidar_avoid_angular_gain{3.0};
    double lidar_direction_upper_angle_deg{8.0};
    double lidar_direction_lower_angle_deg{-8.0};
  };

  // 参数与路线状态
  std::string task1_waypoints_file_;
  std::string route_waypoints_clockwise_file_;
  std::string route_waypoints_counterclockwise_file_;
  std::string frame_id_{"map"};
  std::string base_frame_id_{"base_link"};
  std::string cmd_vel_topic_{"/cmd_vel"};
  std::string scan_topic_{"/scan"};
  std::string map_topic_{"/map"};
  std::string restriction_map_topic_{"/restriction_map"};
  std::string logic3_static_restriction_map_topic_{
    "/logic3_static_restriction_map"};
  std::string logic3_runtime_restriction_map_topic_{
    "/logic3_runtime_restriction_map"};
  std::string qr_topic_{"/qr_info"};
  std::string tts_topic_{"/tts_text"};
  std::string visual_control_suspend_topic_{"/sign4return"};
  std::string route_direction_{"auto"};
  std::string selected_route_direction_;
  bool enable_debug_visualization_{true};
  bool debug_control_logs_{false};
  int clockwise_qr_value_{1};
  int counterclockwise_qr_value_{2};
  int visual_control_suspend_value_{5};
  int visual_control_resume_value_{6};
  int perception_full_resume_value_{9};
  int perception_pause_value_{10};

  double linear_speed_{1.5};
  double lookahead_distance_{0.55};
  double pass_radius_{0.35};
  double route_projection_max_distance_{1.00};
  double segment_advance_t_{0.85};

  double turn_angular_gain_{1.4};
  bool enable_turn_speed_slowdown_{true};
  double turn_speed_full_steering_angle_{0.55};
  double max_turn_speed_reduction_percent_{50.0};
  double reverse_angular_gain_{1.1};
  double reverse_goal_lookahead_distance_{0.25};
  double reverse_pass_radius_{0.25};

  double cluster_max_point_gap_{0.06};
  double cluster_max_range_jump_{0.10};
  int cluster_min_points_{5};
  double cone_circle_min_radius_{0.035};
  double cone_circle_max_radius_{0.065};
  double cone_circle_max_rmse_{0.015};
  int waypoint_shift_occupied_threshold_{50};
  int waypoint_shift_check_count_{20};
  double waypoint_cloud_check_radius_{0.25};
  double waypoint_shift_target_clearance_{0.15};
  double task1_waypoint_shift_target_clearance_{0.45};
  double task1_waypoint_shift_min_y_{-0.10};
  bool waypoint_shift_enable_local_rebuild_{true};
  double waypoint_shift_rebuild_min_progress_ratio_{0.15};
  double waypoint_shift_rebuild_plateau_half_length_{0.08};
  double waypoint_shift_rebuild_bezier_handle_ratio_{0.30};
  // 三段路线使用不同点间距，因此分别以“航点数量”配置偏移前后端。
  int task1_waypoint_shift_points_before_{30};
  int task1_waypoint_shift_points_after_{25};
  int transition_waypoint_shift_points_before_{30};
  int transition_waypoint_shift_points_after_{25};
  int task23_waypoint_shift_points_before_{30};
  int task23_waypoint_shift_points_after_{25};
  int waypoint_shift_confirm_scans_{3};
  int waypoint_shift_release_scans_{8};
  double waypoint_shift_min_change_{0.02};
  bool enable_static_map_filter_{true};
  bool require_map_for_lidar_assist_{true};
  int static_map_occupied_threshold_{90};
  int static_map_inflation_cells_{4};

  // 逻辑3：静态绿色快照冻结路径+绿圈外雷达避障（红点仅记录）。
  std::string logic3_static_route_direction_{"clockwise"};
  std::string logic3_odom_topic_{"/chassis/odom"};
  std::string logic3_odom_frame_id_{"odom"};
  std::string logic3_planning_pose_source_{"filtered_map_odom"};
  std::string logic3_output_root_{
    "/root/D_Robot/test/dev_ws/src/origincar_system/path"};
  std::string logic3_restriction_map_yaml_{
    "/root/D_Robot/test/dev_ws/src/origincar_system/map/race_keepout.yaml"};
  double logic3_motion_speed_threshold_{0.03};
  double logic3_motion_displacement_threshold_{0.01};
  int logic3_motion_confirm_messages_{2};
  bool logic3_start_zone_guard_enabled_{true};
  double logic3_start_zone_min_x_{-0.15};
  double logic3_start_zone_max_x_{0.15};
  double logic3_start_zone_min_y_{-0.05};
  double logic3_start_zone_max_y_{0.20};
  double logic3_start_zone_guard_margin_{0.12};
  double logic3_old_cluster_radius_{0.50};
  double logic3_runtime_roi_width_{3.75};
  double logic3_runtime_roi_height_{3.75};
  double logic3_runtime_channel_min_y_{-0.20};
  double logic3_runtime_cluster_match_radius_{0.35};
  int logic3_runtime_average_samples_{5};
  int logic3_static_cluster_min_points_{1};
  int logic3_static_restriction_inflation_cells_{0};
  double logic3_static_channel_max_x_{1.80};
  double logic3_static_channel_min_y_{1.75};
  double task1_static_min_x_{0.0};
  double task1_static_min_y_{0.0};
  double task1_static_avoid_min_x_{0.0};
  double task1_static_avoid_max_x_{0.60};
  double task1_static_avoid_min_y_{0.0};
  double task1_static_avoid_max_y_{0.50};
  double logic3_static_cluster_match_radius_{0.30};
  double logic3_static_same_frame_merge_radius_{0.20};
  double logic3_static_confirmation_window_seconds_{2.0};
  int logic3_static_confirmation_min_hits_{10};
  double logic3_static_collection_stable_seconds_{0.50};
  double logic3_static_stability_radius_{0.10};
  double logic3_static_far_stability_radius_{0.20};
  double logic3_static_stability_near_distance_{1.0};
  double logic3_static_stability_far_distance_{3.0};
  int logic3_static_average_samples_{10};
  double logic3_static_loss_grace_seconds_{1.0};
  double logic3_static_reacquire_radius_{0.20};
  double logic3_red_history_merge_radius_{0.65};
  double logic3_pose_correction_deadband_xy_{0.005};
  double logic3_pose_correction_deadband_yaw_{0.003};
  double logic3_pose_correction_rate_xy_{0.02};
  double logic3_pose_correction_rate_yaw_{0.02};
  double logic3_pose_outlier_xy_{0.20};
  double logic3_pose_outlier_yaw_{0.15};
  double logic3_static_status_log_rate_hz_{2.0};

  double task1_qr_timeout_seconds_{10.0};
  std::string task1_timeout_qr_text_{"4399"};
  double task1_endpoint_qr_wait_seconds_{0.30};
  double task1_endpoint_arrival_radius_{0.18};
  double task1_transition_min_x_{1.80};
  StageControlProfile task1_profile_;
  StageControlProfile transition_profile_;
  StageControlProfile task2_profile_;
  StageControlProfile task3_profile_;

  // 扫码后持续武装到进入waypoint_channel或达到配置轮数上限的条件倒车；
  // 最大次数为0时完全禁用；
  // 前后检测区均为“扇形与矩形走廊的交集”。PENDING表示机制仍武装，
  // BACKING_RIGHT只表示当前这一轮正在倒车。
  double qr_reverse_y_threshold_{0.10};
  double qr_reverse_x_limit_{4.30};
  double qr_reverse_front_obstacle_distance_{0.40};
  double qr_reverse_front_fov_deg_{120.0};
  double qr_reverse_front_corridor_half_width_{0.35};
  double qr_reverse_rear_obstacle_distance_{0.30};
  double qr_reverse_rear_fov_deg_{120.0};
  double qr_reverse_rear_corridor_half_width_{0.35};
  double qr_reverse_linear_speed_{0.60};
  double qr_reverse_angular_speed_{3.0};
  int qr_reverse_max_count_{0};

  // A→B进场结束判定与入口基准路线圆滑；追点和雷达参数由transition_*提供。
  double channel_handoff_waypoint_distance_{0.30};
  double channel_entry_corner_smoothing_length_{0.20};
  double channel_lidar_avoid_distance_{0.65};
  double channel_lidar_avoid_fov_deg_{120.0};
  double channel_lidar_corridor_half_width_{0.35};
  double channel_lidar_avoid_linear_speed_{0.45};
  double channel_lidar_avoid_angular_gain_{1.60};
  double channel_lidar_direction_upper_angle_deg_{8.0};
  double channel_lidar_direction_lower_angle_deg_{-8.0};

  // 条件倒车与四阶段避障共用的雷达新鲜度和静态墙体过滤参数。
  double lidar_scan_timeout_{0.25};
  bool lidar_enable_static_map_filter_{true};
  int lidar_static_map_occupied_threshold_{50};
  double lidar_static_map_wall_padding_{0.10};

  // 生成路线的最后一段直接连到map起终点；该段仍使用channel追点和
  // 雷达处理。视觉只提供停车标志判定，不启用visual_control状态机。
  bool return_to_start_enabled_{true};
  // 蓝色基地实测矩形；紫色停车触发矩形在X两侧各外扩0.08m、
  // Y两侧各外扩0.05m。
  double return_base_min_x_{-0.15};
  double return_base_max_x_{0.15};
  double return_base_min_y_{-0.05};
  double return_base_max_y_{0.20};
  double return_stop_zone_min_x_{-0.23};
  double return_stop_zone_max_x_{0.23};
  double return_stop_zone_min_y_{-0.10};
  double return_stop_zone_max_y_{0.25};
  double return_stop_confirm_seconds_{0.0};
  // Task3路线走完后持续追踪的虚拟牵引点取紫色停车矩形中心。
  double return_target_x_{0.0};
  double return_target_y_{0.075};
  double return_obstacle_split_y_{-0.20};
  int return_lidar_clear_cycles_{3};
  double return_lidar_rejoin_lateral_tolerance_{0.12};
  double return_lidar_rejoin_linear_speed_{0.45};
  std::string return_visual_detection_topic_{"/racing_obstacle_detection"};
  double return_visual_detection_timeout_{0.50};
  int return_visual_source_width_{1920};
  int return_visual_source_height_{1080};
  int return_visual_control_width_{640};
  int return_visual_control_height_{480};
  double return_visual_avoid_linear_speed_{0.45};
  double return_visual_avoid_angular_ratio_{-0.005};
  double return_visual_obstacle_confidence_threshold_{0.50};
  int return_visual_bottom_threshold_caution_{295};
  int return_visual_bottom_threshold_avoid_{305};
  double return_visual_delta_x_ratio_{0.0};
  double return_visual_parking_sign_confidence_threshold_{0.70};
  int return_visual_parking_y_threshold_{405};

  bool enable_qr_direction_select_{true};
  bool use_qr_parity_direction_{true};
  bool suspend_visual_control_on_qr_{true};
  bool resume_visual_control_on_finish_{true};
  bool enable_image_analysis_trigger_{true};
  double image_capture_clockwise_x_{3.50};
  double image_capture_clockwise_y_{4.05};
  double image_capture_counterclockwise_x_{0.25};
  double image_capture_counterclockwise_y_{4.05};
  double image_capture_radius_{0.10};
  double image_capture_slow_distance_{0.50};
  double image_capture_slow_speed_{0.40};
  double image_capture_angular_gain_{2.0};
  double image_capture_stop_time_{0.30};
  int image_capture_trigger_value_{8};
  std::string image_analysis_trigger_topic_{"/person_trigger"};

  // 运行状态
  std::vector<Waypoint> waypoints_;
  std::vector<Waypoint> original_waypoints_;
  std::vector<Waypoint> channel_route_template_;
  std::vector<double> waypoint_shift_offsets_;
  std::vector<double> waypoint_shift_pending_offsets_;
  std::vector<int> waypoint_shift_pending_counts_;
  std::vector<int> waypoint_shift_missing_counts_;
  std::vector<int> waypoint_shift_dynamic_directions_;
  std::vector<Point2D> waypoint_shift_world_directions_;
  std::vector<std::size_t> waypoint_shift_source_indices_;
  std::size_t current_index_{0};
  bool first_waypoint_reached_{false};
  bool waiting_for_qr_start_{false};
  bool following_qr_path_{true};
  bool qr_path_finished_{false};
  ControlState control_state_{ControlState::PATH_FOLLOWING};
  ControlState next_control_state_{ControlState::PATH_FOLLOWING};
  bool qr_route_selected_{false};
  bool image_analysis_triggered_{false};
  bool has_image_capture_waypoint_{false};
  bool image_capture_stopping_{false};
  std::size_t image_capture_waypoint_index_{0};
  std::chrono::steady_clock::time_point image_capture_stop_started_at_;
  bool publish_stop_on_exit_{true};
  bool task3_route_available_{false};
  bool task3_route_active_{false};
  std::size_t task3_start_index_{0};
  ReturnLidarState return_lidar_state_{ReturnLidarState::FOLLOWING_ROUTE};
  int return_lidar_clear_count_{0};
  // 旧版无task3_start标记的航点文件才会使用独立返程控制。
  bool return_to_start_active_{false};
  std::uint64_t scan_sequence_{0};
  std::uint64_t processed_waypoint_shift_scan_sequence_{0};
  QrEscapeState qr_escape_state_{QrEscapeState::IDLE};
  bool qr_channel_approach_active_{false};
  bool channel_entry_route_rebuild_pending_{false};
  bool channel_entry_route_rebuild_after_reverse_{false};
  std::size_t channel_entry_waypoint_index_{0};
  // [channel_entry_smoothing_first_index_, channel_entry_waypoint_index_]
  // 是偏移前已圆滑的B前末段；waypoint_shift只做原始直接偏移，
  // 不允许偏移后的曲率兜底再重构该段。与B相等表示本次未圆滑。
  std::size_t channel_entry_smoothing_first_index_{0};
  std::size_t channel_completed_reverse_count_{0};
  int channel_lidar_avoid_direction_{0};
  TaskPhase task_phase_{TaskPhase::UNKNOWN};
  bool task1_endpoint_waiting_{false};
  bool task1_departure_timer_started_{false};
  std::string task1_pending_direction_;
  std::string task1_pending_reason_;
  std::string task1_pending_qr_text_;
  bool task1_pending_from_qr_{false};
  bool task1_pending_qr_is_fallback_{false};
  bool task1_pending_gate_logged_{false};
  bool task1_qr_timeout_logged_{false};
  bool task1_endpoint_fallback_logged_{false};
  bool task1_qr_speech_pending_{false};
  bool task1_qr_speech_published_{false};
  std::chrono::steady_clock::time_point task1_endpoint_wait_started_at_;
  std::chrono::steady_clock::time_point task1_departure_started_at_;
  std::chrono::steady_clock::time_point latest_scan_received_at_;
  sensor_msgs::msg::LaserScan::ConstSharedPtr latest_scan_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr latest_map_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr latest_restriction_map_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr latest_logic3_static_restriction_map_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr latest_logic3_runtime_restriction_map_;
  ai_msgs::msg::PerceptionTargets::ConstSharedPtr latest_return_visual_targets_;
  std::chrono::steady_clock::time_point latest_return_visual_received_at_;

  // 逻辑3的可信运动、稳定规划位姿、运行红点和绘图状态。
  std::vector<Point2D> logic3_red_cluster_history_;
  // 全局红簇历史继续跨阶段执行身份去重；04图仅绘制Task1正式结束后
  // 追加的元素，避免与02图中的Task1红圈重影。
  std::size_t logic3_final_red_history_begin_index_{0};
  std::vector<Waypoint> task1_original_route_;
  std::vector<Waypoint> task1_frozen_route_;
  std::vector<Waypoint> task1_dynamic_original_route_;
  std::vector<Waypoint> task1_dynamic_route_;
  std::vector<Waypoint> final_original_route_;
  std::vector<Waypoint> final_frozen_route_;
  std::vector<Waypoint> logic3_baseline_original_route_;
  std::vector<Waypoint> logic3_baseline_route_;
  std::vector<Waypoint> logic3_visualization_route_;
  Point2D task1_transition_marker_;
  Point2D final_frozen_marker_;
  std::string task1_transition_marker_label_;
  std::string final_frozen_marker_label_;
  bool task1_transition_marker_valid_{false};
  bool final_frozen_marker_valid_{false};
  bool return_stop_zone_entry_active_{false};
  std::chrono::steady_clock::time_point return_stop_zone_entered_at_;
  bool qr_reverse_waiting_for_inputs_{false};
  bool channel_lidar_waiting_for_inputs_{false};
  bool channel_lidar_obstacle_active_{false};
  int channel_lidar_logged_direction_{0};
  bool logic3_motion_candidate_{false};
  bool logic3_start_zone_motion_pending_{false};
  bool logic3_planning_pose_anchored_{false};
  bool logic3_odom_reference_valid_{false};
  bool logic3_map_odom_plan_valid_{false};
  bool logic3_final_snapshot_published_{false};
  bool logic3_final_pose_valid_{false};
  int logic3_motion_candidate_count_{0};
  double logic3_odom_reference_x_{0.0};
  double logic3_odom_reference_y_{0.0};
  RobotPose logic3_map_odom_plan_;
  RobotPose logic3_final_pose_;
  std::chrono::steady_clock::time_point logic3_last_pose_correction_at_;
  std::string logic3_session_id_;

  // 仅逻辑3使用：静止阶段持续快照，可信发车后冻结绿色簇和最终路线。
  std::vector<Logic3StaticClusterTrack> logic3_static_cluster_tracks_;
  std::deque<double> logic3_static_scan_times_;
  std::vector<Logic3RuntimeClusterTrack> logic3_runtime_cluster_tracks_;
  Logic3PlanningSnapshot logic3_motion_pending_snapshot_;
  Logic3PlanningSnapshot logic3_last_complete_ready_snapshot_;
  std::vector<ScanPointCluster> logic3_green_clusters_;
  std::vector<double> logic3_frozen_base_offsets_;
  std::vector<Point2D> logic3_frozen_base_shift_directions_;
  std::vector<std::size_t> logic3_frozen_base_source_indices_;
  bool logic3_replay_frozen_clusters_{false};
  bool logic3_force_full_route_{false};
  bool logic3_force_confirm_{false};
  bool logic3_static_state1_active_{false};
  bool logic3_static_ready_{false};
  bool logic3_static_collection_change_time_valid_{false};
  double logic3_static_collection_last_change_seconds_{0.0};
  std::uint64_t logic3_static_next_identity_id_{1};
  bool logic3_static_status_log_started_{false};
  double logic3_last_static_status_log_seconds_{0.0};

  // ROS 通信对象
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr image_analysis_trigger_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr visual_control_suspend_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tts_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr logic3_snapshot_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_route_marker_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr debug_path_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr restriction_map_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr
    logic3_static_restriction_map_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr
    logic3_runtime_restriction_map_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr qr_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr logic3_odom_sub_;
  rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr
    return_visual_detection_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  const char * control_state_name(ControlState state) const;
  void set_control_state(ControlState next_state);
  const char * task_phase_name(TaskPhase phase) const;
  void set_task_phase(TaskPhase phase);
  void apply_stage_profile(const StageControlProfile & profile);

  void control_once();
  bool stop_if_route_finished(const RobotPose & pose);
  bool handle_task3_start(const RobotPose & pose);
  void update_return_lidar_state(
    const RobotPose & pose, bool observation_valid, bool obstacle_found);
  double distance_to_return_route(const RobotPose & pose) const;
  void start_return_to_start();
  void control_return_to_start(const RobotPose & pose);
  void finish_return_to_start(const RobotPose & pose, const char * reason);
  bool return_stop_zone_contains(const RobotPose & pose) const;
  bool return_stop_zone_confirmed(const RobotPose & pose);
  bool return_visual_detection_is_fresh() const;
  int scale_return_visual_x(int x) const;
  int scale_return_visual_y(int y) const;
  int return_visual_target_center_x(const ai_msgs::msg::Target & target) const;
  int return_visual_target_center_y(const ai_msgs::msg::Target & target) const;
  int return_visual_target_bottom_y(const ai_msgs::msg::Target & target) const;
  const ai_msgs::msg::Target * find_return_visual_avoidance_obstacle() const;
  bool return_visual_obstacle_in_caution_zone() const;
  bool return_visual_parking_condition_met() const;

  void update_route_progress(const RobotPose & pose);
  bool find_route_projection(
    const RobotPose & pose, std::size_t extra_segments, RouteProjection & projection) const;
  bool find_nearest_waypoint(
    const RobotPose & pose, std::size_t & nearest_index, double & nearest_distance) const;
  TargetPoint compute_lookahead_target(
    const RobotPose & pose, double lookahead_distance) const;
  void commit_channel_lookahead_progress(const TargetPoint & target);
  bool can_advance_to_index(std::size_t candidate_index, const RobotPose & pose, std::size_t & allowed_index) const;
  void advance_current_index_to(std::size_t new_index, const char * reason);
  double waypoint_pass_radius(std::size_t index) const;
  bool is_segment_reverse(std::size_t segment_index) const;
  bool is_precision_waypoint(std::size_t index) const;
  void insert_image_capture_waypoint(const std::string & direction);
  bool handle_image_capture_stop(const RobotPose & pose);
  void apply_image_capture_speed_limit(
    const RobotPose & pose, geometry_msgs::msg::Twist & cmd) const;
  void apply_turn_speed_limit(geometry_msgs::msg::Twist & cmd) const;

  geometry_msgs::msg::Twist compute_tracking_command(
    const RobotPose & pose, const TargetPoint & target) const;

  bool lookup_pose(
    const std::string & target_frame, const std::string & source_frame,
    const rclcpp::Time & stamp, RobotPose & pose) const;
  Point2D base_to_map(const RobotPose & robot_pose, const Point2D & base_point) const;
  bool original_route_normal_at(std::size_t waypoint_index, Point2D & normal) const;
  std::size_t rebuild_curvature_unsafe_shift_segments(
    std::vector<Waypoint> & shifted_route);
  bool is_static_map_point(const RobotPose & robot_pose, const Point2D & base_point) const;
  bool is_static_map_point(
    const RobotPose & robot_pose, const Point2D & base_point,
    const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & restriction_map,
    int restriction_inflation_cells) const;
  std::vector<ScanPointCluster> collect_non_static_scan_clusters_map(
    const RobotPose & robot_pose, int minimum_cluster_points,
    bool use_logic3_static_restriction_map,
    bool filter_logic3_green_mask = false) const;
  bool fit_cone_circle(LidarCluster & cluster) const;
  std::vector<LidarCluster> collect_cone_clusters(const RobotPose & robot_pose) const;
  bool latest_lidar_scan_is_fresh() const;
  bool prepare_lidar_scan(
    RobotPose & scan_to_base, RobotPose & scan_to_map,
    bool require_map_transform = false,
    bool require_logic3_runtime_map = false) const;
  bool task1_static_avoid_region_contains(const Point2D & map_point) const;
  bool logic3_green_mask_contains(const Point2D & map_point) const;
  bool logic3_runtime_map_point_is_filtered(const Point2D & map_point) const;
  bool lidar_map_point_is_wall(const Point2D & map_point) const;
  LidarObstacle find_lidar_obstacle(
    double center_angle, double maximum_distance, double fov_deg,
    double corridor_half_width, const RobotPose & scan_to_base,
    const RobotPose & scan_to_map,
    bool filter_logic3_green_mask = false,
    bool filter_logic3_runtime_map = false) const;
  bool apply_qr_reverse_maneuver(
    const RobotPose & robot_pose, geometry_msgs::msg::Twist & cmd);
  void update_qr_channel_approach_phase(const RobotPose & robot_pose);
  bool apply_channel_lidar_obstacle_avoidance(
    geometry_msgs::msg::Twist & cmd, bool * observation_valid = nullptr,
    bool * obstacle_found = nullptr);
  void reset_waypoint_shift_state();
  void update_waypoint_shifts(
    const RobotPose & robot_pose, std::size_t target_waypoint_index);
  void update_logic3_runtime_red_tracks(
    const std::vector<ScanPointCluster> & clusters);
  void record_logic3_red_cluster(const ScanPointCluster & cluster);
  void update_logic3_static_cluster_tracks(
    const std::vector<ScanPointCluster> & clusters, const RobotPose & robot_pose);
  double logic3_static_stability_radius_for_range(double obstacle_range) const;
  std::vector<Logic3StaticObstacle> logic3_ready_static_obstacles() const;
  void maybe_log_logic3_static_obstacles(
    std::size_t current_frame_cluster_count,
    const std::vector<Logic3StaticObstacle> & obstacles);
  void logic3_odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void freeze_logic3_route();
  void apply_logic3_frozen_clusters_to_current_route(
    const std::string & snapshot_event);
  void publish_logic3_snapshot(const std::string & event);
  std::string make_logic3_session_id() const;
  bool lookup_logic3_planning_pose(RobotPose & pose);

  void publish_debug_route();
  void publish_debug_visualization(const RobotPose & pose);

  void qr_callback(const std_msgs::msg::String::ConstSharedPtr msg);
  std::string direction_from_qr(const std::string & data) const;
  bool complete_task1_and_select_route(
    const std::string & direction, const char * reason, bool from_qr = false);
  void publish_pending_qr_speech();
  bool switch_route(const std::string & direction);
  double estimate_channel_waypoint_spacing(
    const std::vector<Waypoint> & route) const;
  bool rebuild_channel_entry_route(
    const RobotPose & start_pose, const char * reason);
  void suspend_visual_control();
  void set_perception_mode(int value, const char * mode_name);
  void resume_visual_control(bool restore_full_perception = false);
  void publish_image_analysis_trigger();

  std::vector<Waypoint> load_waypoints(
    const std::string & waypoints_file, const std::string & requested_direction,
    std::string * resolved_direction = nullptr);
  bool lookup_robot_pose(double & x, double & y, double & yaw);
};

#endif  // ORIGINCAR_SYSTEM_VISUAL_LIDAR_PATH_FOLLOWER_HPP_
