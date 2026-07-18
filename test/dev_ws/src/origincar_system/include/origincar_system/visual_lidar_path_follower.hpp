#ifndef ORIGINCAR_SYSTEM_VISUAL_LIDAR_PATH_FOLLOWER_HPP_
#define ORIGINCAR_SYSTEM_VISUAL_LIDAR_PATH_FOLLOWER_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>
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
    FINISHED
  };

  enum class QrEscapeState
  {
    IDLE,
    CHECKING,
    BACKING_RIGHT
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

  struct QrEscapeObservation
  {
    bool ready{false};
    int area_left_count{0};
    int area_right_count{0};
    int rear_left_count{0};
    int rear_right_count{0};
  };

  struct Logic2TrackedCluster
  {
    ScanPointCluster cluster;
    int hit_count{0};
    int missing_count{0};
    double last_seen_seconds{0.0};
  };

  struct Logic3StaticObstacle
  {
    ScanPointCluster cluster;
    Point2D shift_direction;
  };

  struct Logic3StaticClusterTrack
  {
    ScanPointCluster cluster;
    Point2D stability_anchor;
    bool observation_active{false};
    bool state1_active{false};
    bool ready{false};
    double stable_since_seconds{0.0};
    double last_seen_seconds{0.0};
    int consecutive_sample_count{0};
    double sample_center_sum_x{0.0};
    double sample_center_sum_y{0.0};
    std::vector<Point2D> sampled_points;
    Logic3StaticObstacle ready_obstacle;
  };

  struct Logic3PlanningSnapshot
  {
    bool valid{false};
    double stamp_seconds{0.0};
    std::vector<Logic3StaticObstacle> obstacles;
  };

  // 参数与路线状态
  std::string waypoints_file_;
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
  std::string qr_topic_{"/qr_info"};
  std::string qr_channel_visual_detection_topic_{"/racing_obstacle_detection"};
  std::string qr_channel_visual_obstacle_class_name_{"construction_cone"};
  std::string visual_control_suspend_topic_{"/sign4return"};
  std::string route_direction_{"auto"};
  std::string selected_route_direction_;
  bool enable_debug_visualization_{true};
  bool debug_control_logs_{false};
  bool wait_for_qr_before_start_{false};
  int clockwise_qr_value_{1};
  int counterclockwise_qr_value_{2};
  int visual_control_suspend_value_{5};
  int visual_control_resume_value_{6};
  int perception_obstacle_only_value_{7};
  int perception_full_resume_value_{9};
  int perception_pause_value_{10};

  double linear_speed_{1.5};
  double lookahead_distance_{0.55};
  double pass_radius_{0.35};
  double route_projection_max_distance_{1.00};
  double segment_advance_t_{0.85};

  double turn_angular_gain_{1.4};
  bool enable_turn_speed_slowdown_{true};
  double min_turn_speed_percent_{60.0};
  double reverse_angular_gain_{1.1};
  double reverse_goal_lookahead_distance_{0.25};
  double reverse_pass_radius_{0.25};

  double cluster_max_point_gap_{0.06};
  double cluster_max_range_jump_{0.10};
  int cluster_min_points_{5};
  double cone_circle_min_radius_{0.035};
  double cone_circle_max_radius_{0.065};
  double cone_circle_max_rmse_{0.015};
  double waypoint_shift_clearance_radius_{0.50};
  int waypoint_shift_occupied_threshold_{50};
  int waypoint_shift_check_count_{20};
  double waypoint_cloud_check_radius_{0.25};
  double waypoint_shift_target_clearance_{0.15};
  int waypoint_shift_points_before_{5};
  int waypoint_shift_points_after_{5};
  int waypoint_shift_confirm_scans_{3};
  int waypoint_shift_release_scans_{8};
  double waypoint_shift_min_change_{0.02};
  bool enable_static_map_filter_{true};
  bool require_map_for_lidar_assist_{true};
  int static_map_occupied_threshold_{90};
  int static_map_inflation_cells_{4};

  // waypoint_shift 方案选择：1=原有实时，2=动态红簇，3=静态绿色快照+动态红簇。
  int waypoint_shift_logic_{1};
  std::string logic2_odom_topic_{"/chassis/odom"};
  std::string logic2_odom_frame_id_{"odom"};
  std::string logic2_planning_pose_source_{"filtered_map_odom"};
  std::string logic2_output_root_{
    "/root/D_Robot/test/dev_ws/src/origincar_system/path"};
  std::string logic2_restriction_map_yaml_{
    "/root/D_Robot/test/dev_ws/src/origincar_system/map/race_keepout.yaml"};
  double logic2_motion_speed_threshold_{0.03};
  double logic2_motion_displacement_threshold_{0.01};
  int logic2_motion_confirm_messages_{2};
  double logic2_runtime_roi_width_{2.0};
  double logic2_runtime_roi_height_{2.0};
  double logic2_red_history_merge_radius_{0.50};
  double logic2_pose_correction_deadband_xy_{0.005};
  double logic2_pose_correction_deadband_yaw_{0.003};
  double logic2_pose_correction_rate_xy_{0.02};
  double logic2_pose_correction_rate_yaw_{0.02};
  double logic2_pose_outlier_xy_{0.20};
  double logic2_pose_outlier_yaw_{0.15};

  std::string logic3_static_route_direction_{"clockwise"};
  std::string logic3_odom_topic_{"/chassis/odom"};
  std::string logic3_odom_frame_id_{"odom"};
  std::string logic3_planning_pose_source_{"filtered_map_odom"};
  std::string logic3_output_root_{
    "/root/D_Robot/test/dev_ws/src/origincar_system/path"};
  std::string logic3_restriction_map_yaml_{
    "/root/D_Robot/test/dev_ws/src/origincar_system/map/race_keepout.yaml"};
  std::string logic3_tts_topic_{"/tts_text"};
  std::string logic3_static_record_ready_text_{"静态障碍与偏移记录完毕"};
  std::string logic3_route_ready_text_{"路线已规划完毕"};
  double logic3_motion_speed_threshold_{0.03};
  double logic3_motion_displacement_threshold_{0.01};
  int logic3_motion_confirm_messages_{2};
  double logic3_old_cluster_radius_{0.50};
  double logic3_runtime_roi_width_{2.0};
  double logic3_runtime_roi_height_{2.0};
  double logic3_runtime_channel_min_y_{1.60};
  int logic3_static_cluster_min_points_{1};
  int logic3_static_restriction_inflation_cells_{0};
  double logic3_static_channel_min_y_{1.60};
  double logic3_entry_exit_overlap_max_y_{2.50};
  double logic3_static_cluster_match_radius_{0.40};
  double logic3_static_stability_duration_seconds_{1.0};
  double logic3_static_stability_radius_{0.10};
  double logic3_static_far_stability_radius_{0.20};
  double logic3_static_stability_near_distance_{2.0};
  double logic3_static_stability_far_distance_{3.0};
  int logic3_static_average_samples_{10};
  double logic3_static_track_release_seconds_{0.50};
  double logic3_red_history_merge_radius_{0.50};
  double logic3_pose_correction_deadband_xy_{0.005};
  double logic3_pose_correction_deadband_yaw_{0.003};
  double logic3_pose_correction_rate_xy_{0.02};
  double logic3_pose_correction_rate_yaw_{0.02};
  double logic3_pose_outlier_xy_{0.20};
  double logic3_pose_outlier_yaw_{0.15};
  bool logic3_enable_tts_{true};
  double logic3_static_status_log_rate_hz_{2.0};

  double qr_escape_check_distance_{0.60};
  double qr_escape_rear_extension_{0.20};
  double qr_escape_half_width_{0.40};
  int qr_escape_min_points_{3};
  double qr_reverse_linear_speed_{0.10};
  double qr_reverse_angular_speed_{0.25};
  double qr_rear_warning_start_distance_{0.22};
  double qr_rear_check_distance_{0.45};
  double qr_rear_half_width_{0.40};
  int qr_rear_min_points_{2};
  double qr_reverse_min_map_x_{2.0};
  double qr_visual_handoff_waypoint_distance_{0.25};

  double qr_channel_visual_obstacle_confidence_threshold_{0.5};
  int qr_channel_visual_obstacle_bottom_threshold_{305};
  double qr_channel_visual_obstacle_delta_x_ratio_{0.0};
  double qr_channel_visual_avoid_linear_speed_{0.45};
  double qr_channel_visual_avoid_angular_ratio_{-0.005};
  double qr_channel_visual_detection_timeout_{0.30};
  int qr_channel_visual_detection_source_width_{1920};
  int qr_channel_visual_detection_source_height_{1080};
  int qr_channel_visual_control_image_width_{640};
  int qr_channel_visual_control_image_height_{480};

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
  std::vector<double> waypoint_shift_offsets_;
  std::vector<double> waypoint_shift_pending_offsets_;
  std::vector<int> waypoint_shift_pending_counts_;
  std::vector<int> waypoint_shift_missing_counts_;
  std::vector<int> waypoint_shift_dynamic_directions_;
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
  std::uint64_t scan_sequence_{0};
  std::uint64_t processed_waypoint_shift_scan_sequence_{0};
  QrEscapeState qr_escape_state_{QrEscapeState::IDLE};
  QrEscapeObservation latest_qr_escape_observation_;
  bool qr_visual_avoidance_active_{false};
  bool qr_reverse_gate_evaluated_{false};
  bool qr_reverse_eligible_{false};
  bool qr_reverse_triggered_{false};
  bool has_visual_detection_{false};
  std::chrono::steady_clock::time_point latest_visual_detection_received_at_;
  ai_msgs::msg::PerceptionTargets::ConstSharedPtr latest_visual_detection_;
  sensor_msgs::msg::LaserScan::ConstSharedPtr latest_scan_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr latest_map_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr latest_restriction_map_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr latest_logic3_static_restriction_map_;

  // 逻辑2/3共用的可信运动、稳定规划位姿、动态红簇和绘图状态。
  std::vector<Logic2TrackedCluster> logic2_runtime_red_tracks_;
  std::vector<Point2D> logic2_red_cluster_history_;
  std::vector<Waypoint> logic2_baseline_original_route_;
  std::vector<Waypoint> logic2_baseline_route_;
  std::vector<Waypoint> logic2_visualization_route_;
  std::vector<double> logic2_visualization_offsets_;
  bool logic2_motion_candidate_{false};
  bool logic2_planning_pose_anchored_{false};
  bool logic2_odom_reference_valid_{false};
  bool logic2_map_odom_plan_valid_{false};
  bool logic2_final_snapshot_published_{false};
  int logic2_motion_candidate_count_{0};
  double logic2_odom_reference_x_{0.0};
  double logic2_odom_reference_y_{0.0};
  RobotPose logic2_map_odom_plan_;
  std::chrono::steady_clock::time_point logic2_last_pose_correction_at_;
  std::string logic2_session_id_;

  // 仅逻辑3使用：静止阶段持续快照，可信发车后冻结绿色簇并作为动态基线。
  std::vector<Logic3StaticClusterTrack> logic3_static_cluster_tracks_;
  Logic3PlanningSnapshot logic3_motion_pending_snapshot_;
  std::vector<ScanPointCluster> logic3_green_clusters_;
  std::vector<Point2D> logic3_green_shift_directions_;
  std::vector<double> logic3_frozen_base_offsets_;
  bool logic3_replay_frozen_clusters_{false};
  bool logic3_force_full_route_{false};
  bool logic3_force_confirm_{false};
  bool logic3_static_status_log_started_{false};
  double logic3_last_static_status_log_seconds_{0.0};

  // ROS 通信对象
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr image_analysis_trigger_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr visual_control_suspend_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr logic3_tts_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr logic2_snapshot_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_route_marker_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr debug_path_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr restriction_map_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr
    logic3_static_restriction_map_sub_;
  rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr visual_detection_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr qr_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr logic2_odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  const char * control_state_name(ControlState state) const;
  void set_control_state(ControlState next_state);

  void control_once();
  bool stop_if_route_finished();

  void update_route_progress(const RobotPose & pose);
  bool find_route_projection(
    const RobotPose & pose, std::size_t extra_segments, RouteProjection & projection) const;
  bool find_nearest_waypoint(
    const RobotPose & pose, std::size_t & nearest_index, double & nearest_distance) const;
  TargetPoint compute_lookahead_target(
    const RobotPose & pose, double lookahead_distance) const;
  bool can_advance_to_index(std::size_t candidate_index, const RobotPose & pose, std::size_t & allowed_index) const;
  void advance_current_index_to(std::size_t new_index, const char * reason);
  double waypoint_pass_radius(std::size_t index) const;
  bool is_segment_reverse(std::size_t segment_index) const;
  bool is_precision_waypoint(std::size_t index) const;
  void insert_image_capture_waypoint(const std::string & direction);
  bool handle_image_capture_stop(const RobotPose & pose);
  void apply_image_capture_speed_limit(
    const RobotPose & pose, geometry_msgs::msg::Twist & cmd) const;
  void apply_turn_speed_limit(
    const RobotPose & pose, const TargetPoint & target,
    geometry_msgs::msg::Twist & cmd) const;

  geometry_msgs::msg::Twist compute_tracking_command(
    const RobotPose & pose, const TargetPoint & target) const;

  bool lookup_pose(
    const std::string & target_frame, const std::string & source_frame,
    const rclcpp::Time & stamp, RobotPose & pose) const;
  Point2D base_to_map(const RobotPose & robot_pose, const Point2D & base_point) const;
  bool map_distance_to_blocked_along_ray(
    const Point2D & ray_origin, const Point2D & ray_direction,
    double & distance) const;
  bool original_route_normal_at(std::size_t waypoint_index, Point2D & normal) const;
  bool is_static_map_point(const RobotPose & robot_pose, const Point2D & base_point) const;
  bool is_static_map_point(
    const RobotPose & robot_pose, const Point2D & base_point,
    const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & restriction_map,
    int restriction_inflation_cells) const;
  std::vector<ScanPointCluster> collect_non_static_scan_clusters_map(
    const RobotPose & robot_pose, int minimum_cluster_points,
    bool use_logic3_static_restriction_map) const;
  bool fit_cone_circle(LidarCluster & cluster) const;
  std::vector<LidarCluster> collect_cone_clusters(const RobotPose & robot_pose) const;
  QrEscapeObservation observe_qr_escape_point_cloud(const RobotPose & robot_pose) const;
  void evaluate_qr_reverse_gate(const RobotPose & robot_pose);
  bool apply_one_shot_qr_reverse(
    const RobotPose & robot_pose, geometry_msgs::msg::Twist & cmd);
  void update_qr_visual_avoidance_phase(const RobotPose & robot_pose);
  bool apply_qr_channel_visual_obstacle_avoidance(geometry_msgs::msg::Twist & cmd);
  int scale_qr_channel_visual_detection_x(int x) const;
  int scale_qr_channel_visual_detection_y(int y) const;
  void reset_waypoint_shift_state();
  void update_waypoint_shifts(
    const RobotPose & robot_pose, std::size_t target_waypoint_index);
  bool logic2_cluster_hits_route_window(
    const ScanPointCluster & cluster, std::size_t first_index,
    std::size_t last_index) const;
  void update_logic2_runtime_red_tracks(
    const std::vector<ScanPointCluster> & clusters);
  void record_logic2_red_cluster(const ScanPointCluster & cluster);
  void update_logic3_static_cluster_tracks(
    const std::vector<ScanPointCluster> & clusters, const RobotPose & robot_pose);
  double logic3_static_stability_radius_for_range(double obstacle_range) const;
  void reset_logic3_static_track_state(Logic3StaticClusterTrack & track) const;
  bool logic3_static_shift_direction(
    const ScanPointCluster & cluster, Point2D & shift_direction) const;
  std::vector<Logic3StaticObstacle> logic3_ready_static_obstacles() const;
  void maybe_log_logic3_static_obstacles(
    std::size_t current_frame_cluster_count,
    const std::vector<Logic3StaticObstacle> & obstacles);
  void logic2_odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void anchor_logic2_planning_pose();
  void freeze_logic3_route();
  void apply_logic3_frozen_clusters_to_current_route();
  void initialize_logic2_route_visualization();
  void update_logic2_visualization_route();
  void publish_logic2_snapshot(const std::string & event);
  std::string make_logic2_session_id() const;
  bool lookup_logic2_planning_pose(RobotPose & pose);

  void publish_debug_route();
  void publish_debug_visualization(const RobotPose & pose);

  void qr_callback(const std_msgs::msg::String::ConstSharedPtr msg);
  std::string direction_from_qr(const std::string & data) const;
  bool switch_route(const std::string & direction);
  void suspend_visual_control();
  void set_perception_mode(int value, const char * mode_name);
  void resume_visual_control(bool restore_full_perception = false);
  void publish_image_analysis_trigger();

  std::vector<Waypoint> load_waypoints(
    const std::string & waypoints_file, const std::string & requested_direction,
    std::string * resolved_direction = nullptr);
  bool lookup_robot_pose_at(const rclcpp::Time & stamp, RobotPose & pose) const;
  bool lookup_robot_pose(double & x, double & y, double & yaw);
};

#endif  // ORIGINCAR_SYSTEM_VISUAL_LIDAR_PATH_FOLLOWER_HPP_
