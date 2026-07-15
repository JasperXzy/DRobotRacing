#ifndef ORIGINCAR_SYSTEM_VISUAL_LIDAR_PATH_FOLLOWER_HPP_
#define ORIGINCAR_SYSTEM_VISUAL_LIDAR_PATH_FOLLOWER_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
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
    BACKING_LEFT,
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

  struct QrEscapeObservation
  {
    bool ready{false};
    int area_left_count{0};
    int area_right_count{0};
    int rear_left_count{0};
    int rear_right_count{0};
  };

  // 参数与路线状态
  std::string waypoints_file_;
  std::string route_waypoints_file_;
  std::string frame_id_{"map"};
  std::string base_frame_id_{"base_link"};
  std::string cmd_vel_topic_{"/cmd_vel"};
  std::string scan_topic_{"/scan"};
  std::string map_topic_{"/map"};
  std::string restriction_map_topic_{"/restriction_map"};
  std::string qr_topic_{"/qr_info"};
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

  double linear_speed_{1.5};
  double lookahead_distance_{0.55};
  double pass_radius_{0.35};
  double route_projection_max_distance_{1.00};
  double segment_advance_t_{0.85};

  double turn_angular_gain_{1.4};
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
  bool enable_static_map_filter_{true};
  bool require_map_for_lidar_assist_{true};
  int static_map_occupied_threshold_{90};
  int static_map_inflation_cells_{4};

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
  double qr_protect_exit_waypoint_distance_{0.25};

  bool enable_qr_direction_select_{true};
  bool use_qr_parity_direction_{true};
  bool suspend_visual_control_on_qr_{true};
  bool resume_visual_control_on_finish_{true};
  bool enable_image_analysis_trigger_{true};
  double image_capture_clockwise_align_x_{3.15};
  double image_capture_clockwise_align_y_{4.05};
  double image_capture_clockwise_x_{3.50};
  double image_capture_clockwise_y_{4.05};
  double image_capture_counterclockwise_align_x_{0.60};
  double image_capture_counterclockwise_align_y_{4.05};
  double image_capture_counterclockwise_x_{0.25};
  double image_capture_counterclockwise_y_{4.05};
  double image_capture_radius_{0.10};
  double image_capture_slow_distance_{0.50};
  double image_capture_slow_speed_{0.40};
  double image_capture_stop_time_{0.30};
  int image_capture_trigger_value_{8};
  std::string image_analysis_trigger_topic_{"/person_trigger"};

  // 运行状态
  std::vector<Waypoint> waypoints_;
  std::vector<Waypoint> original_waypoints_;
  std::vector<double> waypoint_shift_offsets_;
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
  bool qr_escape_rear_blocked_{false};
  sensor_msgs::msg::LaserScan::ConstSharedPtr latest_scan_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr latest_map_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr latest_restriction_map_;

  // ROS 通信对象
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr image_analysis_trigger_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr visual_control_suspend_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_route_marker_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr debug_path_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr restriction_map_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr qr_sub_;
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
  void insert_image_capture_waypoints(const std::string & direction);
  bool handle_image_capture_stop(const RobotPose & pose);
  void apply_image_capture_speed_limit(
    const RobotPose & pose, geometry_msgs::msg::Twist & cmd) const;

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
  std::vector<Point2D> collect_non_static_scan_points_map(
    const RobotPose & robot_pose) const;
  bool fit_cone_circle(LidarCluster & cluster) const;
  std::vector<LidarCluster> collect_cone_clusters(const RobotPose & robot_pose) const;
  QrEscapeObservation observe_qr_escape_point_cloud(const RobotPose & robot_pose) const;
  bool apply_qr_escape(const RobotPose & robot_pose, geometry_msgs::msg::Twist & cmd);
  void reset_waypoint_shift_state();
  void update_waypoint_shifts(
    const RobotPose & robot_pose, std::size_t target_waypoint_index);

  void publish_debug_route();
  void publish_debug_visualization(const RobotPose & pose);

  void qr_callback(const std_msgs::msg::String::ConstSharedPtr msg);
  std::string direction_from_qr(const std::string & data) const;
  bool switch_route(const std::string & direction);
  void suspend_visual_control();
  void resume_visual_control();
  void publish_image_analysis_trigger();

  std::vector<Waypoint> load_waypoints(
    const std::string & waypoints_file, const std::string & requested_direction,
    std::string * resolved_direction = nullptr);
  bool lookup_robot_pose_at(const rclcpp::Time & stamp, RobotPose & pose) const;
  bool lookup_robot_pose(double & x, double & y, double & yaw);
};

#endif  // ORIGINCAR_SYSTEM_VISUAL_LIDAR_PATH_FOLLOWER_HPP_
