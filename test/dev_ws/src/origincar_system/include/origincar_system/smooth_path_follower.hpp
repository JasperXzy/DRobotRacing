#ifndef ORIGINCAR_SYSTEM_SMOOTH_PATH_FOLLOWER_HPP_
#define ORIGINCAR_SYSTEM_SMOOTH_PATH_FOLLOWER_HPP_

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

struct Waypoint
{
  double x{0.0};
  double y{0.0};
  bool reverse{false};
  double pass_radius{0.0};
  double reverse_pass_radius{0.0};
};

class SmoothPathFollower : public rclcpp::Node
{
public:
  SmoothPathFollower();

  void publish_stop();

private:
  enum class ControlState
  {
    WAITING_FOR_POSE,
    PATH_FOLLOWING,
    OBSTACLE_AVOIDING,
    FINISHED
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

  struct CorridorObstacle
  {
    Point2D center;
    double nearest_range{0.0};
    int point_count{0};
    int left_count{0};
    int right_count{0};
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

  // 参数与路线状态
  std::string waypoints_file_;
  std::string frame_id_{"map"};
  std::string base_frame_id_{"base_link"};
  std::string cmd_vel_topic_{"/cmd_vel"};
  std::string scan_topic_{"/scan"};
  std::string qr_topic_{"/qr_info"};
  std::string route_direction_{"auto"};
  std::string selected_route_direction_;
  int clockwise_qr_value_{1};
  int counterclockwise_qr_value_{2};

  double linear_speed_{1.5};
  double lookahead_distance_{0.55};
  double pass_radius_{0.35};
  int route_projection_window_{10};
  double route_projection_max_distance_{1.00};
  double segment_advance_t_{0.85};

  double goal_tolerance_{0.25};
  double turn_angular_gain_{1.4};
  double reverse_angular_gain_{1.1};
  double reverse_goal_lookahead_distance_{0.25};
  double reverse_pass_radius_{0.25};
  bool enable_reverse_approach_nudge_{true};
  int reverse_approach_nudge_waypoint_{2};
  double reverse_approach_nudge_distance_{0.35};
  double reverse_approach_nudge_angular_z_{-0.35};

  bool enable_obstacle_avoidance_{true};
  int obstacle_enable_from_waypoint_{0};
  double obstacle_corridor_length_{1.00};
  double obstacle_corridor_half_width_{0.28};
  double min_obstacle_range_{0.10};
  int obstacle_min_points_{3};
  double avoid_angular_z_{0.55};
  double avoid_linear_speed_{0.35};
  int obstacle_clear_frames_{5};

  bool enable_qr_skip_first_{true};
  bool enable_qr_direction_select_{true};
  bool use_qr_parity_direction_{true};
  bool enable_image_analysis_trigger_{true};
  int image_analysis_waypoint_{7};
  std::string image_analysis_trigger_topic_{"/person_trigger"};

  // 运行状态
  std::vector<Waypoint> waypoints_;
  std::size_t current_index_{0};
  bool first_waypoint_reached_{false};
  ControlState control_state_{ControlState::PATH_FOLLOWING};
  ControlState next_control_state_{ControlState::PATH_FOLLOWING};
  sensor_msgs::msg::LaserScan::ConstSharedPtr latest_scan_;
  bool obstacle_avoidance_active_{false};
  int obstacle_clear_count_{0};
  int avoid_direction_{0};
  bool qr_skip_done_{false};
  bool image_analysis_triggered_{false};

  // ROS 通信对象
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr image_analysis_trigger_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr qr_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  const char * control_state_name(ControlState state) const;
  void set_control_state(ControlState next_state);

  void control_once();
  bool stop_if_final_goal_reached(const RobotPose & pose);

  void update_route_progress(const RobotPose & pose);
  bool find_route_projection(
    const RobotPose & pose, std::size_t extra_segments, RouteProjection & projection) const;
  TargetPoint compute_lookahead_target(const RobotPose & pose) const;
  bool can_advance_to_index(std::size_t candidate_index, const RobotPose & pose, std::size_t & allowed_index) const;
  void advance_current_index_to(std::size_t new_index, const char * reason);
  double waypoint_pass_radius(std::size_t index) const;
  bool is_segment_reverse(std::size_t segment_index) const;
  bool is_precision_waypoint(std::size_t index) const;

  geometry_msgs::msg::Twist compute_tracking_command(
    const RobotPose & pose, const TargetPoint & target) const;
  geometry_msgs::msg::Twist apply_reverse_approach_nudge(
    geometry_msgs::msg::Twist cmd, std::size_t target_index, double target_distance) const;

  geometry_msgs::msg::Twist apply_corridor_obstacle_avoidance(
    const RobotPose & pose, geometry_msgs::msg::Twist cmd, bool reverse);
  bool detect_corridor_obstacle(bool reverse, CorridorObstacle & obstacle) const;
  int choose_avoid_direction(const CorridorObstacle & obstacle) const;

  void qr_callback(const std_msgs::msg::String::ConstSharedPtr msg);
  std::string direction_from_qr(const std::string & data) const;
  void switch_route(const std::string & direction);
  void skip_first_waypoint_from_qr(const std::string & data);
  void trigger_image_analysis_if_needed(std::size_t waypoint_number);

  std::vector<Waypoint> load_waypoints(const std::string & requested_direction);
  bool lookup_robot_pose(double & x, double & y, double & yaw);
};

#endif  // ORIGINCAR_SYSTEM_SMOOTH_PATH_FOLLOWER_HPP_
