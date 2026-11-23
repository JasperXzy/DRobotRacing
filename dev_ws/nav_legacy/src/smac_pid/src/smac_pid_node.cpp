#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cctype>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/compute_path_through_poses.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "yaml-cpp/yaml.h"

namespace smac_pid
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

double normalize_angle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

geometry_msgs::msg::Quaternion quaternion_from_yaw(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(q);
}

std::string lower_copy(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return value;
}

int64_t steady_now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

}  // namespace

class SmacPidNode : public rclcpp::Node
{
public:
  using ComputePath = nav2_msgs::action::ComputePathThroughPoses;
  using GoalHandleComputePath = rclcpp_action::ClientGoalHandle<ComputePath>;

  SmacPidNode()
  : Node("smac_pid"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    declare_and_read_parameters();
    load_route();

    control_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    planning_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    sensor_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    planner_client_ = rclcpp_action::create_client<ComputePath>(
      get_node_base_interface(), get_node_graph_interface(), get_node_logging_interface(),
      get_node_waitables_interface(), planner_action_, planning_group_);

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    raw_path_pub_ = create_publisher<nav_msgs::msg::Path>("~/raw_path", 10);
    fitted_path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "~/fitted_path", rclcpp::QoS(1).reliable().transient_local());
    active_path_pub_ = create_publisher<nav_msgs::msg::Path>("~/active_path", 10);
    constraint_goals_pub_ = create_publisher<nav_msgs::msg::Path>("~/constraint_goals", 10);
    route_pub_ = create_publisher<nav_msgs::msg::Path>(
      "~/reference_route", rclcpp::QoS(1).reliable().transient_local());

    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = sensor_group_;
    costmap_sub_ = create_subscription<nav2_msgs::msg::Costmap>(
      global_costmap_topic_, rclcpp::QoS(1).reliable(),
      std::bind(&SmacPidNode::costmap_callback, this, std::placeholders::_1),
      subscription_options);

    control_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / control_frequency_),
      std::bind(&SmacPidNode::control_tick, this), control_group_);
    planning_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / planning_frequency_),
      std::bind(&SmacPidNode::planning_tick, this), planning_group_);

    publish_reference_route();
    RCLCPP_INFO(
      get_logger(),
      "smac_pid ready: route=%zu ordered points (%s), constraints=%s, plan=%.1f Hz, "
      "control=%.1f Hz, motion=%s",
      route_.size(), resolved_route_direction_.c_str(), constraint_mode_.c_str(),
      planning_frequency_, control_frequency_,
      enable_motion_ ? "ENABLED" : "DISABLED");
    if (!enable_motion_) {
      RCLCPP_WARN(
        get_logger(),
        "Motion is disabled. Inspect ~/reference_route, ~/raw_path, ~/fitted_path and "
        "~/active_path first; "
        "then relaunch with enable_motion:=true for a low-speed test.");
    }
  }

  ~SmacPidNode() override
  {
    if (enable_motion_ && rclcpp::ok()) {
      publish_stop();
    }
  }

private:
  struct Point2D
  {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
  };

  struct Pose2D
  {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
  };

  struct Projection
  {
    bool valid{false};
    std::size_t segment{0};
    double t{0.0};
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
    double lateral_error{0.0};
    double distance{std::numeric_limits<double>::infinity()};
  };

  struct ActivePath
  {
    std::vector<Point2D> global_points;
    std::vector<Point2D> tracking_points;
    rclcpp::Time accepted_at{0, 0, RCL_ROS_TIME};
    uint64_t generation{0};
  };

  void declare_and_read_parameters()
  {
    global_frame_ = declare_parameter<std::string>("global_frame", "map");
    tracking_frame_ = declare_parameter<std::string>("tracking_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    route_file_ = declare_parameter<std::string>("route_file", "");
    route_direction_ = declare_parameter<std::string>("route_direction", "auto");
    route_closed_ = declare_parameter<bool>("route_closed", true);
    constraint_mode_ = lower_copy(
      declare_parameter<std::string>("constraint_mode", "full_route"));
    route_point_pass_distance_ = declare_parameter<double>(
      "route_point_pass_distance", 0.20);
    planner_action_ = declare_parameter<std::string>(
      "planner_action", "/compute_path_through_poses");
    planner_id_ = declare_parameter<std::string>("planner_id", "GridBased");
    global_costmap_topic_ = declare_parameter<std::string>(
      "global_costmap_topic", "/global_costmap/costmap_raw");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    control_frequency_ = declare_parameter<double>("control_frequency", 50.0);
    planning_frequency_ = declare_parameter<double>("planning_frequency", 5.0);
    planning_request_timeout_ = declare_parameter<double>("planning_request_timeout", 0.45);
    enable_motion_ = declare_parameter<bool>("enable_motion", false);

    window_start_offset_ = declare_parameter<int>("window_start_offset", 2);
    window_end_offset_ = declare_parameter<int>("window_end_offset", 14);
    window_sample_step_ = declare_parameter<int>("window_sample_step", 2);
    window_update_offset_ = declare_parameter<int>("window_update_offset", 9);
    window_advance_ = declare_parameter<int>("window_advance", 10);
    window_update_distance_ = declare_parameter<double>("window_update_distance", 0.30);
    constraint_blocked_cost_threshold_ = declare_parameter<int>(
      "constraint_blocked_cost_threshold", 1);
    obstacle_away_ = declare_parameter<double>("obstacle_away", 0.20);
    obstacle_exit_points_ = declare_parameter<int>("obstacle_exit_points", 3);
    avoidance_side_threshold_ = declare_parameter<double>("avoidance_side_threshold", 0.08);
    avoidance_side_lock_duration_ = declare_parameter<double>(
      "avoidance_side_lock_duration", 2.0);
    avoidance_side_override_distance_ = declare_parameter<double>(
      "avoidance_side_override_distance", 0.55);

    path_resample_spacing_ = declare_parameter<double>("path_resample_spacing", 0.05);
    smoothing_passes_ = declare_parameter<int>("smoothing_passes", 2);
    smoothing_weight_ = declare_parameter<double>("smoothing_weight", 0.20);
    smoothing_data_weight_ = declare_parameter<double>("smoothing_data_weight", 0.08);
    smoothing_anchor_max_shift_ = declare_parameter<double>(
      "smoothing_anchor_max_shift", 0.02);
    projection_heading_weight_ = declare_parameter<double>(
      "projection_heading_weight", 0.05);
    footprint_front_ = declare_parameter<double>("footprint_front", 0.15);
    footprint_rear_ = declare_parameter<double>("footprint_rear", 0.15);
    footprint_half_width_ = declare_parameter<double>("footprint_half_width", 0.095);
    footprint_sample_step_ = declare_parameter<double>("footprint_sample_step", 0.04);
    blocked_cost_threshold_ = declare_parameter<int>("blocked_cost_threshold", 254);
    max_path_age_ = declare_parameter<double>("max_path_age", 0.60);
    hard_path_timeout_ = declare_parameter<double>("hard_path_timeout", 2.0);
    max_pose_cache_age_ = declare_parameter<double>("max_pose_cache_age", 0.20);

    max_speed_ = declare_parameter<double>("max_speed", 0.40);
    min_tracking_speed_ = declare_parameter<double>("min_tracking_speed", 0.20);
    stale_path_speed_ = declare_parameter<double>("stale_path_speed", 0.15);
    max_accel_ = declare_parameter<double>("max_accel", 0.45);
    max_decel_ = declare_parameter<double>("max_decel", 0.90);
    max_lateral_accel_ = declare_parameter<double>("max_lateral_accel", 0.45);
    max_angular_command_ = declare_parameter<double>("max_angular_command", 0.45);
    max_angular_rate_ = declare_parameter<double>("max_angular_rate", 1.5);
    yaw_command_scale_ = declare_parameter<double>("yaw_command_scale", 0.675);
    angular_feedforward_margin_ = declare_parameter<double>("angular_feedforward_margin", 0.90);
    lookahead_distance_ = declare_parameter<double>("lookahead_distance", 0.35);
    lateral_kp_ = declare_parameter<double>("lateral_kp", 1.00);
    lateral_ki_ = declare_parameter<double>("lateral_ki", 0.00);
    lateral_kd_ = declare_parameter<double>("lateral_kd", 0.08);
    heading_kp_ = declare_parameter<double>("heading_kp", 1.50);
    integral_limit_ = declare_parameter<double>("integral_limit", 0.25);
    slow_collision_distance_ = declare_parameter<double>("slow_collision_distance", 0.80);
    emergency_stop_distance_ = declare_parameter<double>("emergency_stop_distance", 0.32);

    if (control_frequency_ <= 0.0 || planning_frequency_ <= 0.0 ||
      path_resample_spacing_ <= 0.0 || max_speed_ <= 0.0 || max_angular_command_ <= 0.0)
    {
      throw std::invalid_argument("smac_pid frequencies, spacing and limits must be positive");
    }
    if (window_start_offset_ < 0 || window_end_offset_ < window_start_offset_ ||
      window_sample_step_ < 1 || window_update_offset_ < 0 || window_advance_ < 1 ||
      window_update_distance_ <= 0.0 || obstacle_away_ < 0.0 || obstacle_exit_points_ < 0 ||
      avoidance_side_threshold_ <= 0.0 || avoidance_side_lock_duration_ < 0.0 ||
      avoidance_side_override_distance_ <= 0.0 || smoothing_data_weight_ < 0.0 ||
      smoothing_anchor_max_shift_ < 0.0 || projection_heading_weight_ < 0.0 ||
      route_point_pass_distance_ < 0.0)
    {
      throw std::invalid_argument("invalid rolling constraint-window parameters");
    }
    if (constraint_mode_ != "full_route" && constraint_mode_ != "rolling_window") {
      throw std::invalid_argument(
              "constraint_mode must be 'full_route' or 'rolling_window'");
    }
    constraint_blocked_cost_threshold_ = std::clamp(
      constraint_blocked_cost_threshold_, 1, 254);
  }

  void load_route()
  {
    if (route_file_.empty()) {
      throw std::runtime_error("route_file is empty");
    }

    const YAML::Node data = YAML::LoadFile(route_file_);
    std::string direction = lower_copy(route_direction_);
    if (direction.empty() || direction == "auto") {
      direction = data["route_direction"] ?
        lower_copy(data["route_direction"].as<std::string>()) : "counterclockwise";
    }

    YAML::Node items;
    bool reverse = false;
    const YAML::Node routes = data["routes"];
    if (routes && routes[direction]) {
      items = routes[direction];
    } else if (routes && direction == "clockwise" && routes["counterclockwise"]) {
      items = routes["counterclockwise"];
      reverse = true;
    } else if (routes && direction == "counterclockwise" && routes["clockwise"]) {
      items = routes["clockwise"];
      reverse = true;
    } else {
      items = data["waypoints"];
    }

    if (!items || !items.IsSequence() || items.size() < 2) {
      throw std::runtime_error("route file has no usable waypoint sequence: " + route_file_);
    }

    route_.clear();
    for (std::size_t index = 0; index < items.size(); ++index) {
      const auto item = items[index];
      Point2D point;
      if (item.IsMap()) {
        point.x = item["x"].as<double>();
        point.y = item["y"].as<double>();
        point.yaw = item["yaw"] ? item["yaw"].as<double>() : 0.0;
      } else if (item.IsSequence() && item.size() >= 2) {
        point.x = item[0].as<double>();
        point.y = item[1].as<double>();
        point.yaw = item.size() >= 3 ? item[2].as<double>() : 0.0;
      } else {
        continue;
      }
      if (std::isfinite(point.x) && std::isfinite(point.y)) {
        route_.push_back(point);
      }
    }
    if (reverse) {
      std::reverse(route_.begin(), route_.end());
    }
    if (route_closed_ && route_.size() >= 2 &&
      std::hypot(route_.front().x - route_.back().x, route_.front().y - route_.back().y) < 1e-4)
    {
      // A closed route is already joined by route_index_after(). Keeping an explicit duplicate
      // endpoint would ask ComputePathThroughPoses to plan a zero-length leg once per lap.
      route_.pop_back();
    }
    if (route_.size() < 2) {
      throw std::runtime_error("route contains fewer than two valid points");
    }
    recompute_yaws(route_, route_closed_);
    resolved_route_direction_ = direction;
  }

  static void recompute_yaws(std::vector<Point2D> & points, bool closed)
  {
    if (points.size() < 2) {
      return;
    }
    for (std::size_t i = 0; i < points.size(); ++i) {
      std::size_t next = i + 1;
      if (next >= points.size()) {
        next = closed ? 0 : i;
      }
      std::size_t prev = i > 0 ? i - 1 : (closed ? points.size() - 1 : i);
      double dx = points[next].x - points[prev].x;
      double dy = points[next].y - points[prev].y;
      if (std::hypot(dx, dy) < 1e-6 && i + 1 < points.size()) {
        dx = points[i + 1].x - points[i].x;
        dy = points[i + 1].y - points[i].y;
      }
      points[i].yaw = std::atan2(dy, dx);
    }
  }

  bool lookup_pose(const std::string & frame, Pose2D & pose)
  {
    try {
      const auto transform = tf_buffer_.lookupTransform(
        frame, base_frame_, rclcpp::Time(0, 0, get_clock()->get_clock_type()));
      pose.x = transform.transform.translation.x;
      pose.y = transform.transform.translation.y;
      pose.yaw = yaw_from_quaternion(transform.transform.rotation);
      if (frame == tracking_frame_) {
        cached_tracking_pose_ = pose;
        cached_tracking_pose_time_ = now();
        have_cached_tracking_pose_ = true;
      }
      return true;
    } catch (const tf2::TransformException & exc) {
      if (frame == tracking_frame_ && have_cached_tracking_pose_ &&
        (now() - cached_tracking_pose_time_).seconds() <= max_pose_cache_age_)
      {
        pose = cached_tracking_pose_;
        return true;
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "Waiting for TF %s -> %s: %s",
        frame.c_str(), base_frame_.c_str(), exc.what());
      return false;
    }
  }

  std::size_t route_index_after(std::size_t index, std::size_t offset) const
  {
    if (route_.empty()) {
      return 0;
    }
    if (route_closed_) {
      return (index + offset) % route_.size();
    }
    return std::min(index + offset, route_.size() - 1);
  }

  double route_segment_length(std::size_t anchor, std::size_t offset) const
  {
    const auto first = route_index_after(anchor, offset);
    const auto second = route_index_after(anchor, offset + 1);
    if (first == second) {
      return 0.0;
    }
    return std::hypot(
      route_[second].x - route_[first].x, route_[second].y - route_[first].y);
  }

  std::size_t first_route_index_ahead(const Pose2D & pose) const
  {
    if (route_.size() < 2) {
      return 0;
    }

    const std::size_t segment_count = route_closed_ ? route_.size() : route_.size() - 1;
    std::size_t best_segment = 0;
    double best_score = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < segment_count; ++i) {
      const std::size_t next = route_closed_ ? (i + 1) % route_.size() : i + 1;
      const double dx = route_[next].x - route_[i].x;
      const double dy = route_[next].y - route_[i].y;
      const double length_sq = dx * dx + dy * dy;
      if (length_sq < 1e-9) {
        continue;
      }
      const double t = std::clamp(
        ((pose.x - route_[i].x) * dx + (pose.y - route_[i].y) * dy) / length_sq,
        0.0, 1.0);
      const double projected_x = route_[i].x + t * dx;
      const double projected_y = route_[i].y + t * dy;
      const double distance = std::hypot(pose.x - projected_x, pose.y - projected_y);
      const double segment_yaw = std::atan2(dy, dx);
      const double heading_error = std::abs(normalize_angle(segment_yaw - pose.yaw));
      const double score = distance + projection_heading_weight_ * heading_error;
      if (score < best_score) {
        best_score = score;
        best_segment = i;
      }
    }
    return route_closed_ ? (best_segment + 1) % route_.size() : best_segment + 1;
  }

  void update_constraint_window(const Pose2D & pose)
  {
    std::vector<std::size_t> check_offsets{
      static_cast<std::size_t>(window_update_offset_)};
    for (const auto offset : last_constraint_offsets_) {
      if (offset >= static_cast<std::size_t>(window_update_offset_) &&
        std::find(check_offsets.begin(), check_offsets.end(), offset) == check_offsets.end())
      {
        check_offsets.push_back(offset);
      }
    }

    int reached_offset = -1;
    for (const auto offset : check_offsets) {
      const auto index = route_index_after(window_anchor_index_, offset);
      if (std::hypot(route_[index].x - pose.x, route_[index].y - pose.y) <=
        window_update_distance_)
      {
        reached_offset = static_cast<int>(offset);
        break;
      }
    }
    if (reached_offset < 0) {
      return;
    }

    const auto old_anchor = window_anchor_index_;
    window_anchor_index_ = route_index_after(
      window_anchor_index_, static_cast<std::size_t>(window_advance_));
    last_constraint_offsets_.clear();
    if (window_anchor_index_ != old_anchor) {
      RCLCPP_INFO(
        get_logger(),
        "Constraint window advanced: anchor point %zu -> %zu (reached offset %d within %.2f m)",
        old_anchor + 1, window_anchor_index_ + 1, reached_offset, window_update_distance_);
    }
  }

  std::vector<Point2D> build_constraint_goals(
    std::size_t start_offset, std::size_t nominal_end, std::size_t sample_step)
  {
    last_constraint_offsets_.clear();
    last_constraint_window_had_obstacle_ = false;
    std::shared_ptr<const nav2_msgs::msg::Costmap> map;
    {
      std::lock_guard<std::mutex> lock(costmap_mutex_);
      map = latest_costmap_;
    }

    auto blocked_at_offset = [this, &map](std::size_t offset) {
        if (!map) {
          return false;
        }
        const auto index = route_index_after(window_anchor_index_, offset);
        return static_cast<int>(cost_at(*map, route_[index].x, route_[index].y)) >=
               constraint_blocked_cost_threshold_;
      };

    std::vector<Point2D> goals;
    std::vector<std::size_t> goal_indices;
    std::vector<std::size_t> goal_offsets;
    auto append_goal = [this, &goals, &goal_indices, &goal_offsets](std::size_t offset) {
        const auto index = route_index_after(window_anchor_index_, offset);
        if (!goal_indices.empty() && goal_indices.back() == index) {
          return;
        }
        goal_indices.push_back(index);
        goal_offsets.push_back(offset);
        goals.push_back(route_[index]);
      };

    std::size_t offset = start_offset;
    const std::size_t available_points = route_closed_ ?
      route_.size() : route_.size() - std::min(window_anchor_index_, route_.size() - 1);
    const std::size_t scan_limit = route_closed_ ?
      start_offset + route_.size() : available_points;
    while (offset <= nominal_end && offset < scan_limit) {
      if (!blocked_at_offset(offset)) {
        append_goal(offset);
        offset += sample_step;
        continue;
      }

      const std::size_t blocked_offset = offset;
      last_constraint_window_had_obstacle_ = true;
      std::size_t recovery_offset = offset;
      bool recovery_found = false;
      for (std::size_t attempts = 0; attempts < route_.size(); ++attempts) {
        while (recovery_offset < scan_limit && blocked_at_offset(recovery_offset)) {
          ++recovery_offset;
        }
        if (recovery_offset >= scan_limit) {
          break;
        }

        double clear_distance = 0.0;
        while (recovery_offset + 1 < scan_limit && clear_distance < obstacle_away_) {
          clear_distance += route_segment_length(window_anchor_index_, recovery_offset);
          ++recovery_offset;
        }
        if (clear_distance + 1e-6 < obstacle_away_) {
          break;
        }

        std::size_t next_blocked = scan_limit;
        bool continuity_available = true;
        for (int extra = 0; extra <= obstacle_exit_points_; ++extra) {
          const auto candidate = recovery_offset + static_cast<std::size_t>(extra);
          if (candidate >= scan_limit) {
            continuity_available = false;
            break;
          }
          if (blocked_at_offset(candidate)) {
            next_blocked = candidate;
            break;
          }
        }
        if (!continuity_available) {
          break;
        }
        if (next_blocked == scan_limit) {
          recovery_found = true;
          break;
        }
        recovery_offset = next_blocked;
      }

      if (!recovery_found) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "No safe route constraint found after blocked point %zu",
          route_index_after(window_anchor_index_, blocked_offset) + 1);
        break;
      }

      for (int extra = 0; extra <= obstacle_exit_points_; ++extra) {
        append_goal(recovery_offset + static_cast<std::size_t>(extra));
      }
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Removed blocked route constraints at point %zu; resumed at point %zu "
        "after %.2f m clearance and appended %d continuity points",
        route_index_after(window_anchor_index_, blocked_offset) + 1,
        route_index_after(window_anchor_index_, recovery_offset) + 1,
        obstacle_away_, obstacle_exit_points_);
      offset = recovery_offset + static_cast<std::size_t>(obstacle_exit_points_) +
        sample_step;
    }

    if (!goal_indices.empty()) {
      last_constraint_offsets_ = goal_offsets;
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Route constraints: %zu goals, route points %zu -> %zu (%s)",
        goals.size(), goal_indices.front() + 1, goal_indices.back() + 1,
        constraint_mode_.c_str());
    }
    return goals;
  }

  void planning_tick()
  {
    if (route_.size() < 2) {
      return;
    }

    if (planner_busy_.load()) {
      const double busy_age = static_cast<double>(steady_now_ns() - request_started_ns_.load()) * 1e-9;
      if (busy_age <= planning_request_timeout_) {
        return;
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "Planner request exceeded %.2f s; cancelling stale request",
        planning_request_timeout_);
      planner_client_->async_cancel_all_goals();
      planner_busy_.store(false);
    }

    if (!planner_client_->action_server_is_ready()) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for %s", planner_action_.c_str());
      return;
    }

    Pose2D start_pose;
    if (!lookup_pose(global_frame_, start_pose)) {
      return;
    }
    std::vector<Point2D> constraint_goals;
    if (constraint_mode_ == "full_route") {
      // Start with the first route point ahead of the current projection. This prevents every
      // real-time replan from turning back toward points that the vehicle has already passed.
      window_anchor_index_ = first_route_index_ahead(start_pose);
      std::size_t passed_points = 0;
      while (passed_points + 1 < route_.size() &&
        std::hypot(
          route_[window_anchor_index_].x - start_pose.x,
          route_[window_anchor_index_].y - start_pose.y) <= route_point_pass_distance_)
      {
        const auto next = route_index_after(window_anchor_index_, 1);
        if (next == window_anchor_index_) {
          break;
        }
        window_anchor_index_ = next;
        ++passed_points;
      }
      if (passed_points > 0) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Treated %zu route points within %.2f m as passed; first A* goal is point %zu",
          passed_points, route_point_pass_distance_, window_anchor_index_ + 1);
      }
      const std::size_t full_route_goal_count = route_closed_ ?
        route_.size() : route_.size() - window_anchor_index_;
      if (full_route_goal_count > 0) {
        constraint_goals = build_constraint_goals(0, full_route_goal_count - 1, 1);
      }
    } else {
      update_constraint_window(start_pose);
      constraint_goals = build_constraint_goals(
        static_cast<std::size_t>(window_start_offset_),
        static_cast<std::size_t>(window_end_offset_),
        static_cast<std::size_t>(window_sample_step_));
    }
    if (constraint_goals.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "No safe constraint goals available for planner");
      return;
    }
    publish_path(constraint_goals_pub_, constraint_goals, global_frame_);

    ComputePath::Goal goal;
    goal.use_start = true;
    goal.planner_id = planner_id_;
    goal.start.header.frame_id = global_frame_;
    goal.start.header.stamp = now();
    goal.start.pose.position.x = start_pose.x;
    goal.start.pose.position.y = start_pose.y;
    goal.start.pose.orientation = quaternion_from_yaw(start_pose.yaw);
    goal.goals.reserve(constraint_goals.size());
    for (const auto & point : constraint_goals) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = global_frame_;
      pose.header.stamp = goal.start.header.stamp;
      pose.pose.position.x = point.x;
      pose.pose.position.y = point.y;
      pose.pose.orientation = quaternion_from_yaw(point.yaw);
      goal.goals.push_back(pose);
    }

    const uint64_t generation = ++latest_generation_;
    planner_busy_.store(true);
    request_started_ns_.store(steady_now_ns());

    rclcpp_action::Client<ComputePath>::SendGoalOptions options;
    options.goal_response_callback = [this, generation](
      GoalHandleComputePath::SharedPtr handle)
      {
        if (!handle && generation == latest_generation_.load()) {
          planner_busy_.store(false);
          RCLCPP_WARN(get_logger(), "Planner rejected request %lu", generation);
        }
      };
    const bool constraint_obstacle_active = last_constraint_window_had_obstacle_;
    options.result_callback =
      [this, generation, constraint_obstacle_active, constraint_goals](
      const GoalHandleComputePath::WrappedResult & result) {
        if (generation != latest_generation_.load()) {
          return;
        }
        planner_busy_.store(false);
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED || !result.result) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Planner request failed");
          return;
        }
        const double elapsed =
          static_cast<double>(steady_now_ns() - request_started_ns_.load()) * 1e-9;
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "A* full-path result: %zu constraints, %zu poses, %.3f s",
          constraint_goals.size(), result.result->path.poses.size(), elapsed);
        accept_candidate_path(
          result.result->path, generation, constraint_obstacle_active, constraint_goals);
      };
    planner_client_->async_send_goal(goal, options);
  }

  static std::vector<Point2D> path_message_to_points(const nav_msgs::msg::Path & path)
  {
    std::vector<Point2D> points;
    points.reserve(path.poses.size());
    for (const auto & pose : path.poses) {
      Point2D point;
      point.x = pose.pose.position.x;
      point.y = pose.pose.position.y;
      point.yaw = yaw_from_quaternion(pose.pose.orientation);
      if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
        return {};
      }
      if (points.empty() || std::hypot(point.x - points.back().x, point.y - points.back().y) > 1e-4) {
        points.push_back(point);
      }
    }
    return points;
  }

  std::vector<Point2D> smooth_points(
    const std::vector<Point2D> & input, const std::vector<Point2D> & anchors,
    double strength = 1.0) const
  {
    if (input.size() < 3 || smoothing_passes_ <= 0 || smoothing_weight_ <= 0.0) {
      return input;
    }
    auto points = input;
    const auto original = input;
    std::vector<bool> anchored(points.size(), false);
    std::vector<Point2D> anchor_targets(points.size());
    for (const auto & anchor : anchors) {
      std::size_t nearest = 0;
      double nearest_distance = std::numeric_limits<double>::infinity();
      for (std::size_t index = 0; index < points.size(); ++index) {
        const double distance = std::hypot(
          points[index].x - anchor.x, points[index].y - anchor.y);
        if (distance < nearest_distance) {
          nearest_distance = distance;
          nearest = index;
        }
      }
      anchored[nearest] = true;
      anchor_targets[nearest] = anchor;
    }
    strength = std::clamp(strength, 0.0, 1.0);
    const double weight = std::clamp(smoothing_weight_ * strength, 0.0, 0.45);
    const double data_weight = std::clamp(smoothing_data_weight_, 0.0, 0.45);
    for (int pass = 0; pass < smoothing_passes_; ++pass) {
      auto next = points;
      for (std::size_t i = 1; i + 1 < points.size(); ++i) {
        next[i].x = points[i].x +
          data_weight * (original[i].x - points[i].x) +
          weight * (points[i - 1].x + points[i + 1].x - 2.0 * points[i].x);
        next[i].y = points[i].y +
          data_weight * (original[i].y - points[i].y) +
          weight * (points[i - 1].y + points[i + 1].y - 2.0 * points[i].y);
        if (anchored[i]) {
          const double dx = next[i].x - anchor_targets[i].x;
          const double dy = next[i].y - anchor_targets[i].y;
          const double shift = std::hypot(dx, dy);
          if (shift > smoothing_anchor_max_shift_ && shift > 1e-9) {
            const double scale = smoothing_anchor_max_shift_ / shift;
            next[i].x = anchor_targets[i].x + scale * dx;
            next[i].y = anchor_targets[i].y + scale * dy;
          }
        }
      }
      points.swap(next);
    }
    recompute_yaws(points, false);
    return points;
  }

  std::vector<Point2D> resample_points(const std::vector<Point2D> & input) const
  {
    if (input.size() < 2) {
      return {};
    }
    std::vector<Point2D> output;
    output.push_back(input.front());
    double remaining = path_resample_spacing_;
    Point2D segment_start = input.front();
    for (std::size_t i = 1; i < input.size(); ++i) {
      const Point2D segment_end = input[i];
      double dx = segment_end.x - segment_start.x;
      double dy = segment_end.y - segment_start.y;
      double length = std::hypot(dx, dy);
      while (length >= remaining && length > 1e-9) {
        const double ratio = remaining / length;
        Point2D point;
        point.x = segment_start.x + ratio * dx;
        point.y = segment_start.y + ratio * dy;
        output.push_back(point);
        segment_start = point;
        dx = segment_end.x - segment_start.x;
        dy = segment_end.y - segment_start.y;
        length = std::hypot(dx, dy);
        remaining = path_resample_spacing_;
      }
      remaining -= length;
      segment_start = segment_end;
    }
    if (std::hypot(output.back().x - input.back().x, output.back().y - input.back().y) >
      0.25 * path_resample_spacing_)
    {
      output.push_back(input.back());
    }
    recompute_yaws(output, false);
    return output;
  }

  uint8_t cost_at(const nav2_msgs::msg::Costmap & map, double world_x, double world_y) const
  {
    const double origin_yaw = yaw_from_quaternion(map.metadata.origin.orientation);
    const double dx = world_x - map.metadata.origin.position.x;
    const double dy = world_y - map.metadata.origin.position.y;
    const double local_x = std::cos(origin_yaw) * dx + std::sin(origin_yaw) * dy;
    const double local_y = -std::sin(origin_yaw) * dx + std::cos(origin_yaw) * dy;
    if (local_x < 0.0 || local_y < 0.0 || map.metadata.resolution <= 0.0) {
      return 255;
    }
    const auto mx = static_cast<uint32_t>(local_x / map.metadata.resolution);
    const auto my = static_cast<uint32_t>(local_y / map.metadata.resolution);
    if (mx >= map.metadata.size_x || my >= map.metadata.size_y) {
      return 255;
    }
    const std::size_t index = static_cast<std::size_t>(my) * map.metadata.size_x + mx;
    return index < map.data.size() ? map.data[index] : 255;
  }

  bool point_is_blocked(
    const nav2_msgs::msg::Costmap & map, const Point2D & point,
    uint8_t * worst_cost = nullptr) const
  {
    uint8_t maximum = cost_at(map, point.x, point.y);
    const double cos_yaw = std::cos(point.yaw);
    const double sin_yaw = std::sin(point.yaw);
    auto check_sample = [&](double local_x, double local_y) {
        const double world_x = point.x + cos_yaw * local_x - sin_yaw * local_y;
        const double world_y = point.y + sin_yaw * local_x + cos_yaw * local_y;
        const uint8_t cost = cost_at(map, world_x, world_y);
        maximum = std::max(maximum, cost);
        return static_cast<int>(cost) >= blocked_cost_threshold_;
      };

    if (static_cast<int>(maximum) >= blocked_cost_threshold_) {
      if (worst_cost) {
        *worst_cost = maximum;
      }
      return true;
    }

    const double longitudinal_length = footprint_front_ + footprint_rear_;
    const int longitudinal_samples = std::max(
      1, static_cast<int>(std::ceil(longitudinal_length / footprint_sample_step_)));
    for (int i = 0; i <= longitudinal_samples; ++i) {
      const double local_x = -footprint_rear_ + longitudinal_length *
        static_cast<double>(i) / static_cast<double>(longitudinal_samples);
      if (check_sample(local_x, footprint_half_width_) ||
        check_sample(local_x, -footprint_half_width_))
      {
        if (worst_cost) {
          *worst_cost = maximum;
        }
        return true;
      }
    }

    const double lateral_length = 2.0 * footprint_half_width_;
    const int lateral_samples = std::max(
      1, static_cast<int>(std::ceil(lateral_length / footprint_sample_step_)));
    for (int i = 0; i <= lateral_samples; ++i) {
      const double local_y = -footprint_half_width_ + lateral_length *
        static_cast<double>(i) / static_cast<double>(lateral_samples);
      if (check_sample(footprint_front_, local_y) || check_sample(-footprint_rear_, local_y)) {
        if (worst_cost) {
          *worst_cost = maximum;
        }
        return true;
      }
    }
    if (worst_cost) {
      *worst_cost = maximum;
    }
    return false;
  }

  bool path_is_collision_free(
    const std::vector<Point2D> & points, Point2D * blocked_point = nullptr,
    uint8_t * blocked_cost = nullptr) const
  {
    std::shared_ptr<const nav2_msgs::msg::Costmap> map;
    {
      std::lock_guard<std::mutex> lock(costmap_mutex_);
      map = latest_costmap_;
    }
    if (!map) {
      return false;
    }
    for (const auto & point : points) {
      uint8_t cost = 0;
      if (point_is_blocked(*map, point, &cost)) {
        if (blocked_point) {
          *blocked_point = point;
        }
        if (blocked_cost) {
          *blocked_cost = cost;
        }
        return false;
      }
    }
    return true;
  }

  bool transform_points_to_tracking(
    const std::vector<Point2D> & global_points, std::vector<Point2D> & tracking_points)
  {
    try {
      const auto transform = tf_buffer_.lookupTransform(
        tracking_frame_, global_frame_, rclcpp::Time(0, 0, get_clock()->get_clock_type()));
      const double yaw = yaw_from_quaternion(transform.transform.rotation);
      const double cos_yaw = std::cos(yaw);
      const double sin_yaw = std::sin(yaw);
      tracking_points.clear();
      tracking_points.reserve(global_points.size());
      for (const auto & global_point : global_points) {
        Point2D point;
        point.x = transform.transform.translation.x +
          cos_yaw * global_point.x - sin_yaw * global_point.y;
        point.y = transform.transform.translation.y +
          sin_yaw * global_point.x + cos_yaw * global_point.y;
        point.yaw = normalize_angle(global_point.yaw + yaw);
        tracking_points.push_back(point);
      }
      return true;
    } catch (const tf2::TransformException & exc) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "Cannot transform candidate path %s -> %s: %s",
        global_frame_.c_str(), tracking_frame_.c_str(), exc.what());
      return false;
    }
  }

  int detour_side(const std::vector<Point2D> & points) const
  {
    double strongest_lateral = 0.0;
    for (const auto & point : points) {
      std::size_t nearest = 0;
      double nearest_distance = std::numeric_limits<double>::infinity();
      for (std::size_t index = 0; index < route_.size(); ++index) {
        const double distance = std::hypot(
          point.x - route_[index].x, point.y - route_[index].y);
        if (distance < nearest_distance) {
          nearest_distance = distance;
          nearest = index;
        }
      }
      const double dx = point.x - route_[nearest].x;
      const double dy = point.y - route_[nearest].y;
      const double lateral =
        -std::sin(route_[nearest].yaw) * dx + std::cos(route_[nearest].yaw) * dy;
      if (std::abs(lateral) > std::abs(strongest_lateral)) {
        strongest_lateral = lateral;
      }
    }
    if (std::abs(strongest_lateral) < avoidance_side_threshold_) {
      return 0;
    }
    return strongest_lateral > 0.0 ? 1 : -1;
  }

  bool active_path_needs_side_override(const std::shared_ptr<const ActivePath> & active)
  {
    if (!active) {
      return true;
    }
    Pose2D pose;
    if (!lookup_pose(global_frame_, pose)) {
      return false;
    }
    const auto projection = project_to_path(active->global_points, pose);
    if (!projection.valid) {
      return true;
    }
    return collision_distance_ahead(*active, projection.segment) <=
           avoidance_side_override_distance_;
  }

  void accept_candidate_path(
    const nav_msgs::msg::Path & path, uint64_t generation, bool constraint_obstacle_active,
    const std::vector<Point2D> & constraint_goals)
  {
    const auto raw_points = path_message_to_points(path);
    if (raw_points.size() < 2) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "Planner returned an empty/invalid path");
      return;
    }

    publish_path(raw_path_pub_, raw_points, global_frame_);
    std::vector<Point2D> candidate;
    Point2D blocked_point;
    uint8_t blocked_cost = 0;
    double accepted_smoothing_strength = 0.0;
    for (const double strength : {1.0, 0.60, 0.30}) {
      auto fitted = resample_points(smooth_points(raw_points, constraint_goals, strength));
      if (path_is_collision_free(fitted, &blocked_point, &blocked_cost)) {
        candidate = std::move(fitted);
        accepted_smoothing_strength = strength;
        break;
      }
    }
    if (candidate.empty()) {
      candidate = resample_points(raw_points);
      if (!path_is_collision_free(candidate, &blocked_point, &blocked_cost)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Rejected candidate path %lu: footprint collision at (%.3f, %.3f), cost=%u",
          generation, blocked_point.x, blocked_point.y, static_cast<unsigned>(blocked_cost));
        return;
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "All fitted paths touched the costmap; using collision-free raw A* path");
    } else if (accepted_smoothing_strength < 1.0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Reduced path-fitting strength to %.2f to preserve footprint clearance",
        accepted_smoothing_strength);
    }
    std::shared_ptr<const ActivePath> previous_active;
    {
      std::lock_guard<std::mutex> lock(path_mutex_);
      previous_active = active_path_;
    }
    const int64_t current_ns = steady_now_ns();
    const int candidate_side = constraint_obstacle_active ? detour_side(candidate) : 0;
    const bool lock_active = avoidance_side_lock_ != 0 &&
      current_ns < avoidance_side_lock_until_ns_;
    if (constraint_obstacle_active && lock_active && candidate_side != avoidance_side_lock_ &&
      !active_path_needs_side_override(previous_active))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 500,
        "Rejected path-side flip while avoiding obstacle: locked=%s candidate=%s",
        avoidance_side_lock_ > 0 ? "left" : "right",
        candidate_side > 0 ? "left" : (candidate_side < 0 ? "right" : "straight"));
      return;
    }
    if (constraint_obstacle_active && candidate_side != 0) {
      avoidance_side_lock_ = candidate_side;
      avoidance_side_lock_until_ns_ = current_ns + static_cast<int64_t>(
        avoidance_side_lock_duration_ * 1e9);
    } else if (!constraint_obstacle_active && current_ns >= avoidance_side_lock_until_ns_) {
      avoidance_side_lock_ = 0;
    }

    std::vector<Point2D> tracking_points;
    if (!transform_points_to_tracking(candidate, tracking_points)) {
      return;
    }
    publish_path(fitted_path_pub_, candidate, global_frame_);

    auto active = std::make_shared<ActivePath>();
    active->global_points = candidate;
    active->tracking_points = std::move(tracking_points);
    active->accepted_at = now();
    active->generation = generation;
    {
      std::lock_guard<std::mutex> lock(path_mutex_);
      active_path_ = active;
    }
    publish_path(active_path_pub_, active->tracking_points, tracking_frame_);
  }

  Projection project_to_path(const std::vector<Point2D> & points, const Pose2D & pose) const
  {
    Projection best;
    double best_score = std::numeric_limits<double>::infinity();
    if (points.size() < 2) {
      return best;
    }
    for (std::size_t i = 0; i + 1 < points.size(); ++i) {
      const double sx = points[i].x;
      const double sy = points[i].y;
      const double dx = points[i + 1].x - sx;
      const double dy = points[i + 1].y - sy;
      const double length_sq = dx * dx + dy * dy;
      if (length_sq < 1e-9) {
        continue;
      }
      const double raw_t = ((pose.x - sx) * dx + (pose.y - sy) * dy) / length_sq;
      const double t = std::clamp(raw_t, 0.0, 1.0);
      const double px = sx + t * dx;
      const double py = sy + t * dy;
      const double distance = std::hypot(pose.x - px, pose.y - py);
      const double segment_yaw = std::atan2(dy, dx);
      const double heading_error = std::abs(normalize_angle(segment_yaw - pose.yaw));
      const double score = distance + projection_heading_weight_ * heading_error;
      if (score < best_score) {
        const double length = std::sqrt(length_sq);
        best_score = score;
        best.valid = true;
        best.segment = i;
        best.t = t;
        best.x = px;
        best.y = py;
        best.yaw = segment_yaw;
        best.distance = distance;
        best.lateral_error = (dx * (pose.y - py) - dy * (pose.x - px)) / length;
      }
    }
    return best;
  }

  std::size_t lookahead_index(
    const std::vector<Point2D> & points, const Projection & projection, double distance) const
  {
    std::size_t index = std::min(projection.segment + 1, points.size() - 1);
    double accumulated = std::hypot(points[index].x - projection.x, points[index].y - projection.y);
    while (index + 1 < points.size() && accumulated < distance) {
      accumulated += std::hypot(
        points[index + 1].x - points[index].x, points[index + 1].y - points[index].y);
      ++index;
    }
    return index;
  }

  static double curvature_at(const std::vector<Point2D> & points, std::size_t index)
  {
    if (points.size() < 3) {
      return 0.0;
    }
    index = std::clamp<std::size_t>(index, 1, points.size() - 2);
    const auto & a = points[index - 1];
    const auto & b = points[index];
    const auto & c = points[index + 1];
    const double ab = std::hypot(b.x - a.x, b.y - a.y);
    const double bc = std::hypot(c.x - b.x, c.y - b.y);
    const double ac = std::hypot(c.x - a.x, c.y - a.y);
    if (ab * bc * ac < 1e-8) {
      return 0.0;
    }
    const double cross = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    return 2.0 * cross / (ab * bc * ac);
  }

  double collision_distance_ahead(const ActivePath & path, std::size_t start_index) const
  {
    std::shared_ptr<const nav2_msgs::msg::Costmap> map;
    {
      std::lock_guard<std::mutex> lock(costmap_mutex_);
      map = latest_costmap_;
    }
    if (!map || path.global_points.empty()) {
      return std::numeric_limits<double>::infinity();
    }
    start_index = std::min(start_index, path.global_points.size() - 1);
    double distance = 0.0;
    for (std::size_t i = start_index; i < path.global_points.size(); ++i) {
      if (i > start_index) {
        distance += std::hypot(
          path.global_points[i].x - path.global_points[i - 1].x,
          path.global_points[i].y - path.global_points[i - 1].y);
      }
      if (point_is_blocked(*map, path.global_points[i])) {
        return distance;
      }
    }
    return std::numeric_limits<double>::infinity();
  }

  void control_tick()
  {
    if (!enable_motion_) {
      return;
    }

    std::shared_ptr<const ActivePath> path;
    {
      std::lock_guard<std::mutex> lock(path_mutex_);
      path = active_path_;
    }
    if (!path || path->tracking_points.size() < 2) {
      publish_stop();
      return;
    }

    Pose2D pose;
    if (!lookup_pose(tracking_frame_, pose)) {
      publish_stop();
      return;
    }

    const Projection projection = project_to_path(path->tracking_points, pose);
    if (!projection.valid || projection.distance > 1.0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "Robot is too far from active path: %.2f m", projection.distance);
      publish_stop();
      return;
    }

    const std::size_t target_index = lookahead_index(
      path->tracking_points, projection, lookahead_distance_);
    const double curvature = std::clamp(
      curvature_at(path->tracking_points, target_index), -3.0, 3.0);
    const double heading_error = normalize_angle(path->tracking_points[target_index].yaw - pose.yaw);
    const double dt = 1.0 / control_frequency_;
    lateral_integral_ = std::clamp(
      lateral_integral_ + projection.lateral_error * dt, -integral_limit_, integral_limit_);
    const double lateral_derivative = (projection.lateral_error - last_lateral_error_) / dt;
    last_lateral_error_ = projection.lateral_error;

    double target_speed = max_speed_;
    if (std::abs(curvature) > 1e-3) {
      const double command_limited_speed =
        angular_feedforward_margin_ * max_angular_command_ /
        (yaw_command_scale_ * std::abs(curvature));
      const double lateral_limited_speed = std::sqrt(max_lateral_accel_ / std::abs(curvature));
      target_speed = std::min({target_speed, command_limited_speed, lateral_limited_speed});
    }
    if (std::abs(heading_error) > 0.45 || std::abs(projection.lateral_error) > 0.25) {
      target_speed = std::min(target_speed, min_tracking_speed_);
    }

    const double path_age = (now() - path->accepted_at).seconds();
    if (path_age > max_path_age_) {
      target_speed = std::min(target_speed, stale_path_speed_);
    }
    if (path_age > hard_path_timeout_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "Active path is stale (%.2f s); holding low speed", path_age);
    }

    const double collision_distance = collision_distance_ahead(*path, projection.segment);
    const bool emergency = collision_distance <= emergency_stop_distance_;
    if (emergency) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 500, "Obstacle %.2f m ahead on active path; emergency stop",
        collision_distance);
      publish_stop();
      return;
    }
    if (collision_distance < slow_collision_distance_) {
      target_speed = std::min(target_speed, min_tracking_speed_);
    }
    target_speed = std::max(std::min(target_speed, max_speed_), min_tracking_speed_);

    const double max_up = max_accel_ * dt;
    const double max_down = max_decel_ * dt;
    const double speed = std::clamp(target_speed, last_linear_command_ - max_down, last_linear_command_ + max_up);

    double angular = yaw_command_scale_ * speed * curvature +
      heading_kp_ * heading_error -
      lateral_kp_ * projection.lateral_error -
      lateral_ki_ * lateral_integral_ -
      lateral_kd_ * lateral_derivative;
    angular = std::clamp(angular, -max_angular_command_, max_angular_command_);
    const double max_angular_step = max_angular_rate_ * dt;
    angular = std::clamp(
      angular, last_angular_command_ - max_angular_step, last_angular_command_ + max_angular_step);

    geometry_msgs::msg::Twist command;
    command.linear.x = speed;
    command.angular.z = angular;
    cmd_pub_->publish(command);
    last_linear_command_ = speed;
    last_angular_command_ = angular;

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "track: v=%.2f z=%.2f ey=%.3f eheading=%.2fdeg curvature=%.2f path_age=%.2fs",
      speed, angular, projection.lateral_error, heading_error * 180.0 / kPi, curvature, path_age);
  }

  void publish_stop()
  {
    geometry_msgs::msg::Twist stop;
    cmd_pub_->publish(stop);
    last_linear_command_ = 0.0;
    last_angular_command_ = 0.0;
    lateral_integral_ = 0.0;
  }

  void costmap_callback(const nav2_msgs::msg::Costmap::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    latest_costmap_ = message;
  }

  void publish_reference_route()
  {
    publish_path(route_pub_, route_, global_frame_);
  }

  void publish_path(
    const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr & publisher,
    const std::vector<Point2D> & points, const std::string & frame)
  {
    nav_msgs::msg::Path message;
    message.header.frame_id = frame;
    message.header.stamp = now();
    message.poses.reserve(points.size());
    for (const auto & point : points) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = message.header;
      pose.pose.position.x = point.x;
      pose.pose.position.y = point.y;
      pose.pose.orientation = quaternion_from_yaw(point.yaw);
      message.poses.push_back(pose);
    }
    publisher->publish(message);
  }

  std::string global_frame_;
  std::string tracking_frame_;
  std::string base_frame_;
  std::string route_file_;
  std::string route_direction_;
  std::string resolved_route_direction_;
  std::string constraint_mode_;
  std::string planner_action_;
  std::string planner_id_;
  std::string global_costmap_topic_;
  std::string cmd_vel_topic_;
  bool route_closed_{true};
  bool enable_motion_{false};
  double control_frequency_{50.0};
  double planning_frequency_{5.0};
  double planning_request_timeout_{0.45};
  double route_point_pass_distance_{0.20};
  int window_start_offset_{2};
  int window_end_offset_{14};
  int window_sample_step_{2};
  int window_update_offset_{9};
  int window_advance_{10};
  double window_update_distance_{0.30};
  int constraint_blocked_cost_threshold_{1};
  double obstacle_away_{0.20};
  int obstacle_exit_points_{3};
  double avoidance_side_threshold_{0.08};
  double avoidance_side_lock_duration_{2.0};
  double avoidance_side_override_distance_{0.55};
  double path_resample_spacing_{0.05};
  int smoothing_passes_{2};
  double smoothing_weight_{0.20};
  double smoothing_data_weight_{0.08};
  double smoothing_anchor_max_shift_{0.02};
  double projection_heading_weight_{0.05};
  double footprint_front_{0.15};
  double footprint_rear_{0.15};
  double footprint_half_width_{0.095};
  double footprint_sample_step_{0.04};
  int blocked_cost_threshold_{254};
  double max_path_age_{0.60};
  double hard_path_timeout_{2.0};
  double max_pose_cache_age_{0.20};
  double max_speed_{0.40};
  double min_tracking_speed_{0.20};
  double stale_path_speed_{0.15};
  double max_accel_{0.45};
  double max_decel_{0.90};
  double max_lateral_accel_{0.45};
  double max_angular_command_{0.45};
  double max_angular_rate_{1.5};
  double yaw_command_scale_{0.675};
  double angular_feedforward_margin_{0.90};
  double lookahead_distance_{0.35};
  double lateral_kp_{1.0};
  double lateral_ki_{0.0};
  double lateral_kd_{0.08};
  double heading_kp_{1.5};
  double integral_limit_{0.25};
  double slow_collision_distance_{0.80};
  double emergency_stop_distance_{0.32};

  std::vector<Point2D> route_;
  std::size_t window_anchor_index_{0};
  std::vector<std::size_t> last_constraint_offsets_;
  bool last_constraint_window_had_obstacle_{false};
  int avoidance_side_lock_{0};
  int64_t avoidance_side_lock_until_ns_{0};
  std::mutex path_mutex_;
  std::shared_ptr<const ActivePath> active_path_;
  mutable std::mutex costmap_mutex_;
  std::shared_ptr<const nav2_msgs::msg::Costmap> latest_costmap_;
  std::atomic<bool> planner_busy_{false};
  std::atomic<uint64_t> latest_generation_{0};
  std::atomic<int64_t> request_started_ns_{0};

  Pose2D cached_tracking_pose_;
  rclcpp::Time cached_tracking_pose_time_{0, 0, RCL_ROS_TIME};
  bool have_cached_tracking_pose_{false};
  double lateral_integral_{0.0};
  double last_lateral_error_{0.0};
  double last_linear_command_{0.0};
  double last_angular_command_{0.0};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::CallbackGroup::SharedPtr control_group_;
  rclcpp::CallbackGroup::SharedPtr planning_group_;
  rclcpp::CallbackGroup::SharedPtr sensor_group_;
  rclcpp_action::Client<ComputePath>::SharedPtr planner_client_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr raw_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr fitted_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr active_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr constraint_goals_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr route_pub_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr planning_timer_;
};

}  // namespace smac_pid

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<smac_pid::SmacPidNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
