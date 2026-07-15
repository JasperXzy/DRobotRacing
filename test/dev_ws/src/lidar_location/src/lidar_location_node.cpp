/*
 * ROS 2 port of the jie_ware lidar_loc grid-map scan matcher.
 *
 * The original implementation is Copyright (C) Zhang Wanjie / 6-robot
 * and distributed under the GNU General Public License version 2.
 * This port preserves that license.
 */

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace
{
constexpr double kPi = 3.14159265358979323846;

double normalize_angle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

std::string normalize_frame(std::string frame)
{
  while (!frame.empty() && frame.front() == '/') {
    frame.erase(frame.begin());
  }
  return frame;
}
}  // namespace

class LidarLocationNode : public rclcpp::Node
{
public:
  LidarLocationNode()
  : Node("lidar_location")
  {
    global_frame_ = normalize_frame(declare_parameter<std::string>("global_frame", "map"));
    odom_frame_ = normalize_frame(declare_parameter<std::string>("odom_frame", "odom"));
    base_frame_ = normalize_frame(declare_parameter<std::string>("base_frame", "base_link"));
    laser_frame_ = normalize_frame(declare_parameter<std::string>("laser_frame", ""));

    map_topic_ = declare_parameter<std::string>("map_topic", "/map");
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    initial_pose_topic_ =
      declare_parameter<std::string>("initial_pose_topic", "/initialpose");
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/amcl_pose");
    scan_cloud_topic_ =
      declare_parameter<std::string>("scan_cloud_topic", "/lidar_location/scan_cloud");

    tf_broadcast_ = declare_parameter<bool>("tf_broadcast", true);
    tf_publish_rate_ = std::max(1.0, declare_parameter<double>("tf_publish_rate", 30.0));
    transform_tolerance_ =
      std::max(0.0, declare_parameter<double>("transform_tolerance", 0.3));
    tf_lookup_timeout_ =
      std::max(0.01, declare_parameter<double>("tf_lookup_timeout", 0.2));

    set_initial_pose_ = declare_parameter<bool>("set_initial_pose", true);
    reset_pose_on_map_ = declare_parameter<bool>("reset_pose_on_map", false);
    initial_pose_.x = declare_parameter<double>("initial_pose.x", 0.0);
    initial_pose_.y = declare_parameter<double>("initial_pose.y", 0.0);
    initial_pose_.yaw = declare_parameter<double>("initial_pose.yaw", 0.0);

    occupied_threshold_ = static_cast<int>(std::clamp<std::int64_t>(
        declare_parameter<std::int64_t>("occupied_threshold", 65), 1, 100));
    likelihood_field_radius_ =
      std::max(0.01, declare_parameter<double>("likelihood_field_radius", 1.25));
    linear_search_step_ =
      std::max(0.0, declare_parameter<double>("linear_search_step", 0.0));
    angular_search_step_ =
      std::max(0.01, declare_parameter<double>("angular_search_step_deg", 1.0)) *
      kPi / 180.0;
    refinement_levels_ = static_cast<int>(std::max<std::int64_t>(
        1, declare_parameter<std::int64_t>("refinement_levels", 3)));
    refinement_step_scale_ = std::clamp(
      declare_parameter<double>("refinement_step_scale", 0.5), 0.1, 0.9);
    max_iterations_ = static_cast<int>(std::max<std::int64_t>(
        1, declare_parameter<std::int64_t>("max_iterations", 80)));
    max_translation_correction_ = std::max(
      0.01, declare_parameter<double>("max_translation_correction", 0.5));
    max_rotation_correction_ =
      std::max(0.1, declare_parameter<double>("max_rotation_correction_deg", 20.0)) *
      kPi / 180.0;
    max_beams_ = static_cast<int>(std::max<std::int64_t>(
        1, declare_parameter<std::int64_t>("max_beams", 180)));
    min_valid_beams_ = static_cast<int>(std::max<std::int64_t>(
        1, declare_parameter<std::int64_t>("min_valid_beams", 20)));
    min_match_score_ =
      std::clamp(declare_parameter<double>("min_match_score", 0.05), 0.0, 1.0);
    lost_score_ = std::clamp(
      declare_parameter<double>("quality.lost_score", 0.20), min_match_score_, 1.0);
    recovery_score_ = std::clamp(
      declare_parameter<double>("quality.recovery_score", 0.45), lost_score_, 1.0);
    locked_score_ = std::clamp(
      declare_parameter<double>("quality.locked_score", 0.55), recovery_score_, 1.0);
    lost_confirm_frames_ = static_cast<int>(std::max<std::int64_t>(
        1, declare_parameter<std::int64_t>("quality.lost_confirm_frames", 3)));
    recovery_confirm_frames_ = static_cast<int>(std::max<std::int64_t>(
        1, declare_parameter<std::int64_t>("quality.recovery_confirm_frames", 5)));
    min_inlier_ratio_ = std::clamp(
      declare_parameter<double>("quality.min_inlier_ratio", 0.45), 0.0, 1.0);
    inlier_likelihood_threshold_ = std::clamp(
      declare_parameter<double>("quality.inlier_likelihood_threshold", 0.35), 0.0, 1.0);
    trim_ratio_ = std::clamp(
      declare_parameter<double>("quality.trim_ratio", 0.25), 0.0, 0.8);
    min_score_improvement_ = std::max(
      0.0, declare_parameter<double>("quality.min_score_improvement", 0.02));
    large_correction_translation_ = std::max(
      0.0, declare_parameter<double>("quality.large_correction_translation", 0.10));
    large_correction_rotation_ = std::max(
      0.0, declare_parameter<double>("quality.large_correction_rotation_deg", 5.0)) *
      kPi / 180.0;
    degraded_correction_gain_ = std::clamp(
      declare_parameter<double>("quality.degraded_correction_gain", 0.25), 0.0, 1.0);
    max_locked_translation_step_ = std::max(
      0.001, declare_parameter<double>("quality.max_locked_translation_step", 0.15));
    max_locked_rotation_step_ = std::max(
      0.01, declare_parameter<double>("quality.max_locked_rotation_step_deg", 5.0)) *
      kPi / 180.0;
    max_degraded_translation_step_ = std::max(
      0.001, declare_parameter<double>("quality.max_degraded_translation_step", 0.05));
    max_degraded_rotation_step_ = std::max(
      0.01, declare_parameter<double>("quality.max_degraded_rotation_step_deg", 2.0)) *
      kPi / 180.0;

    isolated_point_filter_enabled_ =
      declare_parameter<bool>("filter.isolated_point_enabled", true);
    isolated_range_threshold_ = std::max(
      0.0, declare_parameter<double>("filter.isolated_range_threshold", 0.10));
    static_wall_filter_enabled_ =
      declare_parameter<bool>("filter.static_wall_enabled", true);
    static_wall_max_distance_ = std::max(
      0.01, declare_parameter<double>("filter.static_wall_max_distance", 0.25));
    static_wall_min_points_ = static_cast<int>(std::max<std::int64_t>(
        1, declare_parameter<std::int64_t>("filter.static_wall_min_points", 20)));

    deskew_enabled_ = declare_parameter<bool>("deskew.enabled", true);
    deskew_max_scan_time_ = std::max(
      0.01, declare_parameter<double>("deskew.max_scan_time", 0.15));

    coarse_search_enabled_ = declare_parameter<bool>("coarse_search.enabled", true);
    coarse_trigger_score_ = std::clamp(
      declare_parameter<double>("coarse_search.trigger_score", 0.45), 0.0, 1.0);
    coarse_linear_range_ = std::max(
      0.0, declare_parameter<double>("coarse_search.linear_range", 0.75));
    coarse_angular_range_ = std::max(
      0.0, declare_parameter<double>("coarse_search.angular_range_deg", 30.0)) *
      kPi / 180.0;
    coarse_linear_step_ = std::max(
      0.01, declare_parameter<double>("coarse_search.linear_step", 0.10));
    coarse_angular_step_ = std::max(
      0.1, declare_parameter<double>("coarse_search.angular_step_deg", 3.0)) *
      kPi / 180.0;
    coarse_search_interval_frames_ = static_cast<int>(std::max<std::int64_t>(
        1, declare_parameter<std::int64_t>("coarse_search.interval_frames", 3)));
    prior_translation_weight_ = std::max(
      0.0, declare_parameter<double>("coarse_search.prior_translation_weight", 0.05));
    prior_rotation_weight_ = std::max(
      0.0, declare_parameter<double>("coarse_search.prior_rotation_weight", 0.02));
    min_laser_range_ =
      std::max(0.0, declare_parameter<double>("min_laser_range", 0.0));
    max_laser_range_ =
      std::max(0.0, declare_parameter<double>("max_laser_range", 0.0));

    covariance_x_ = std::max(0.0, declare_parameter<double>("covariance.x", 0.05));
    covariance_y_ = std::max(0.0, declare_parameter<double>("covariance.y", 0.05));
    covariance_yaw_ = std::max(0.0, declare_parameter<double>("covariance.yaw", 0.03));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    const auto map_qos = rclcpp::QoS(1).transient_local().reliable();
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, map_qos,
      std::bind(&LidarLocationNode::map_callback, this, std::placeholders::_1));
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&LidarLocationNode::scan_callback, this, std::placeholders::_1));
    initial_pose_sub_ =
      create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      initial_pose_topic_, rclcpp::QoS(10),
      std::bind(&LidarLocationNode::initial_pose_callback, this, std::placeholders::_1));

    pose_pub_ =
      create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      pose_topic_, rclcpp::QoS(10));
    match_score_pub_ =
      create_publisher<std_msgs::msg::Float32>("~/match_score", rclcpp::QoS(10));
    robust_score_pub_ =
      create_publisher<std_msgs::msg::Float32>("~/robust_score", rclcpp::QoS(10));
    inlier_ratio_pub_ =
      create_publisher<std_msgs::msg::Float32>("~/inlier_ratio", rclcpp::QoS(10));
    correction_distance_pub_ =
      create_publisher<std_msgs::msg::Float32>("~/correction_distance", rclcpp::QoS(10));
    correction_yaw_pub_ =
      create_publisher<std_msgs::msg::Float32>("~/correction_yaw", rclcpp::QoS(10));
    valid_beams_pub_ =
      create_publisher<std_msgs::msg::UInt32>("~/valid_beams", rclcpp::QoS(10));
    iterations_pub_ =
      create_publisher<std_msgs::msg::UInt32>("~/iterations", rclcpp::QoS(10));
    state_pub_ =
      create_publisher<std_msgs::msg::String>("~/state", rclcpp::QoS(10));
    coarse_search_used_pub_ =
      create_publisher<std_msgs::msg::Bool>("~/coarse_search_used", rclcpp::QoS(10));
    scan_cloud_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(scan_cloud_topic_, rclcpp::QoS(5));

    const auto tf_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / tf_publish_rate_));
    tf_timer_ = create_wall_timer(
      tf_period, std::bind(&LidarLocationNode::broadcast_map_to_odom, this));

    RCLCPP_INFO(
      get_logger(),
      "ROS 2 lidar localization ready: map=%s scan=%s frames=%s->%s->%s",
      map_topic_.c_str(), scan_topic_.c_str(), global_frame_.c_str(),
      odom_frame_.c_str(), base_frame_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "Quality gate: locked>=%.2f recovery>=%.2f lost<%.2f, inliers>=%.2f",
      locked_score_, recovery_score_, lost_score_, min_inlier_ratio_);
  }

private:
  enum class TrackingState
  {
    kLocked,
    kDegraded,
    kLost,
  };

  struct Pose2D
  {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
  };

  struct ScoreResult
  {
    double score{0.0};
    double robust_score{0.0};
    double inlier_ratio{0.0};
  };

  struct MatchResult
  {
    Pose2D pose;
    double score{0.0};
    double robust_score{0.0};
    double inlier_ratio{0.0};
    double objective{-std::numeric_limits<double>::infinity()};
    double seed_score{0.0};
    int iterations{0};
    bool used_coarse_search{false};
  };

  void set_pose(const Pose2D & pose, const char * source)
  {
    pose_ = pose;
    pose_.yaw = normalize_angle(pose_.yaw);
    has_pose_ = true;
    has_map_to_odom_ = false;
    tracking_state_ = TrackingState::kDegraded;
    low_quality_frames_ = 0;
    recovery_frames_ = 0;
    last_coarse_search_scan_ = -coarse_search_interval_frames_;
    RCLCPP_INFO(
      get_logger(), "Initial pose from %s: x=%.3f y=%.3f yaw=%.1f deg",
      source, pose_.x, pose_.y, pose_.yaw * 180.0 / kPi);
  }

  void map_callback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
  {
    const std::size_t expected_size =
      static_cast<std::size_t>(msg->info.width) * static_cast<std::size_t>(msg->info.height);
    if (msg->info.width == 0 || msg->info.height == 0 ||
      msg->info.resolution <= 0.0 || msg->data.size() != expected_size)
    {
      RCLCPP_ERROR(get_logger(), "Rejected invalid occupancy map");
      return;
    }

    map_ = *msg;
    map_origin_yaw_ = tf2::getYaw(map_.info.origin.orientation);

    cv::Mat distance_input(
      static_cast<int>(map_.info.height), static_cast<int>(map_.info.width),
      CV_8UC1, cv::Scalar(255));

    std::size_t occupied_cells = 0;
    for (std::size_t index = 0; index < map_.data.size(); ++index) {
      if (map_.data[index] >= occupied_threshold_) {
        const int y = static_cast<int>(index / map_.info.width);
        const int x = static_cast<int>(index % map_.info.width);
        distance_input.at<unsigned char>(y, x) = 0;
        ++occupied_cells;
      }
    }

    if (occupied_cells == 0) {
      map_ready_ = false;
      RCLCPP_ERROR(get_logger(), "Map contains no occupied cells");
      return;
    }

    cv::Mat distance_pixels;
    cv::distanceTransform(distance_input, distance_pixels, cv::DIST_L2, cv::DIST_MASK_3);
    likelihood_field_ = cv::Mat(
      distance_pixels.rows, distance_pixels.cols, CV_32FC1, cv::Scalar(0.0F));
    static_distance_field_ = cv::Mat(
      distance_pixels.rows, distance_pixels.cols, CV_32FC1,
      cv::Scalar(std::numeric_limits<float>::infinity()));

    for (int y = 0; y < distance_pixels.rows; ++y) {
      for (int x = 0; x < distance_pixels.cols; ++x) {
        const std::size_t index =
          static_cast<std::size_t>(y) * map_.info.width + static_cast<std::size_t>(x);
        if (map_.data[index] < 0) {
          continue;
        }
        const double distance_m =
          static_cast<double>(distance_pixels.at<float>(y, x)) * map_.info.resolution;
        static_distance_field_.at<float>(y, x) = static_cast<float>(distance_m);
        likelihood_field_.at<float>(y, x) = static_cast<float>(
          std::max(0.0, 1.0 - distance_m / likelihood_field_radius_));
      }
    }

    map_ready_ = true;
    if (reset_pose_on_map_ && set_initial_pose_) {
      set_pose(initial_pose_, "map reset parameters");
    } else if (!has_pose_ && set_initial_pose_) {
      set_pose(initial_pose_, "parameters");
    }

    RCLCPP_INFO(
      get_logger(), "Map likelihood field ready: %ux%u, resolution %.3f m, %zu obstacles",
      map_.info.width, map_.info.height, map_.info.resolution, occupied_cells);
    if (!has_pose_) {
      RCLCPP_WARN(get_logger(), "Waiting for /initialpose before localization can start");
    }
  }

  void initial_pose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
  {
    const std::string frame = normalize_frame(msg->header.frame_id);
    if (!frame.empty() && frame != global_frame_) {
      RCLCPP_WARN(
        get_logger(), "Ignored initial pose in frame '%s'; expected '%s'",
        frame.c_str(), global_frame_.c_str());
      return;
    }

    Pose2D pose;
    pose.x = msg->pose.pose.position.x;
    pose.y = msg->pose.pose.position.y;
    pose.yaw = tf2::getYaw(msg->pose.pose.orientation);
    set_pose(pose, initial_pose_topic_.c_str());
  }

  bool world_to_grid(
    const double world_x, const double world_y, double & grid_x, double & grid_y) const
  {
    const double dx = world_x - map_.info.origin.position.x;
    const double dy = world_y - map_.info.origin.position.y;
    const double cos_origin = std::cos(map_origin_yaw_);
    const double sin_origin = std::sin(map_origin_yaw_);
    const double local_x = cos_origin * dx + sin_origin * dy;
    const double local_y = -sin_origin * dx + cos_origin * dy;

    grid_x = local_x / map_.info.resolution;
    grid_y = local_y / map_.info.resolution;
    return grid_x >= 0.0 && grid_y >= 0.0 &&
           grid_x <= static_cast<double>(likelihood_field_.cols - 1) &&
           grid_y <= static_cast<double>(likelihood_field_.rows - 1);
  }

  double sample_likelihood(const double world_x, const double world_y) const
  {
    double grid_x = 0.0;
    double grid_y = 0.0;
    if (!world_to_grid(world_x, world_y, grid_x, grid_y)) {
      return 0.0;
    }

    const int x0 = static_cast<int>(std::floor(grid_x));
    const int y0 = static_cast<int>(std::floor(grid_y));
    const int x1 = std::min(x0 + 1, likelihood_field_.cols - 1);
    const int y1 = std::min(y0 + 1, likelihood_field_.rows - 1);
    const double x_weight = grid_x - static_cast<double>(x0);
    const double y_weight = grid_y - static_cast<double>(y0);

    const double top =
      (1.0 - x_weight) * static_cast<double>(likelihood_field_.at<float>(y0, x0)) +
      x_weight * static_cast<double>(likelihood_field_.at<float>(y0, x1));
    const double bottom =
      (1.0 - x_weight) * static_cast<double>(likelihood_field_.at<float>(y1, x0)) +
      x_weight * static_cast<double>(likelihood_field_.at<float>(y1, x1));
    return (1.0 - y_weight) * top + y_weight * bottom;
  }

  bool sample_static_distance(
    const double world_x, const double world_y, double & distance) const
  {
    double grid_x = 0.0;
    double grid_y = 0.0;
    if (!world_to_grid(world_x, world_y, grid_x, grid_y) || static_distance_field_.empty()) {
      return false;
    }

    const int x = std::clamp(
      static_cast<int>(std::lround(grid_x)), 0, static_distance_field_.cols - 1);
    const int y = std::clamp(
      static_cast<int>(std::lround(grid_y)), 0, static_distance_field_.rows - 1);
    distance = static_cast<double>(static_distance_field_.at<float>(y, x));
    return std::isfinite(distance);
  }

  ScoreResult evaluate_pose(
    const Pose2D & pose, const std::vector<cv::Point2f> & points) const
  {
    ScoreResult result;
    if (points.empty()) {
      return result;
    }

    const double cos_yaw = std::cos(pose.yaw);
    const double sin_yaw = std::sin(pose.yaw);
    double sum = 0.0;
    std::size_t inliers = 0;
    std::vector<double> likelihoods;
    likelihoods.reserve(points.size());

    for (const auto & point : points) {
      const double world_x =
        pose.x + cos_yaw * static_cast<double>(point.x) -
        sin_yaw * static_cast<double>(point.y);
      const double world_y =
        pose.y + sin_yaw * static_cast<double>(point.x) +
        cos_yaw * static_cast<double>(point.y);

      const double likelihood = sample_likelihood(world_x, world_y);
      sum += likelihood;
      likelihoods.push_back(likelihood);
      if (likelihood >= inlier_likelihood_threshold_) {
        ++inliers;
      }
    }

    result.score = sum / static_cast<double>(points.size());
    result.inlier_ratio = static_cast<double>(inliers) / static_cast<double>(points.size());

    const std::size_t trim_count = std::min<std::size_t>(
      likelihoods.size() - 1,
      static_cast<std::size_t>(
        std::floor(trim_ratio_ * static_cast<double>(likelihoods.size()))));
    if (trim_count > 0) {
      std::nth_element(
        likelihoods.begin(), likelihoods.begin() + static_cast<std::ptrdiff_t>(trim_count),
        likelihoods.end());
    }
    const double robust_sum = std::accumulate(
      likelihoods.begin() + static_cast<std::ptrdiff_t>(trim_count), likelihoods.end(), 0.0);
    result.robust_score = robust_sum / static_cast<double>(likelihoods.size() - trim_count);
    return result;
  }

  double matching_objective(
    const Pose2D & pose, const Pose2D & seed, const ScoreResult & score) const
  {
    const double translation = std::hypot(pose.x - seed.x, pose.y - seed.y);
    const double rotation = std::abs(normalize_angle(pose.yaw - seed.yaw));
    return score.robust_score -
           prior_translation_weight_ * translation * translation -
           prior_rotation_weight_ * rotation * rotation;
  }

  MatchResult make_match_result(
    const Pose2D & pose, const Pose2D & seed, const ScoreResult & score) const
  {
    MatchResult result;
    result.pose = pose;
    result.score = score.score;
    result.robust_score = score.robust_score;
    result.inlier_ratio = score.inlier_ratio;
    result.objective = matching_objective(pose, seed, score);
    return result;
  }

  MatchResult refine_match(
    const Pose2D & seed, const Pose2D & start,
    const std::vector<cv::Point2f> & points,
    const double translation_limit, const double rotation_limit) const
  {
    double linear_step =
      linear_search_step_ > 0.0 ? linear_search_step_ : map_.info.resolution;
    double angular_step = angular_search_step_;

    MatchResult result = make_match_result(start, seed, evaluate_pose(start, points));
    int total_iterations = 0;

    for (int level = 0;
      level < refinement_levels_ && total_iterations < max_iterations_; ++level)
    {
      const std::array<std::pair<double, double>, 5> translations = {{
        {0.0, 0.0},
        {linear_step, 0.0},
        {-linear_step, 0.0},
        {0.0, linear_step},
        {0.0, -linear_step},
      }};
      const std::array<double, 3> rotations = {
        0.0, angular_step, -angular_step};

      while (total_iterations < max_iterations_) {
        MatchResult best = result;
        bool improved = false;

        for (const auto & translation : translations) {
          for (const double rotation : rotations) {
            if (translation.first == 0.0 && translation.second == 0.0 && rotation == 0.0) {
              continue;
            }

            Pose2D candidate;
            candidate.x = result.pose.x + translation.first;
            candidate.y = result.pose.y + translation.second;
            candidate.yaw = normalize_angle(result.pose.yaw + rotation);
            if (std::hypot(candidate.x - seed.x, candidate.y - seed.y) >
              translation_limit ||
              std::abs(normalize_angle(candidate.yaw - seed.yaw)) >
              rotation_limit)
            {
              continue;
            }
            const MatchResult candidate_result =
              make_match_result(candidate, seed, evaluate_pose(candidate, points));
            if (candidate_result.objective > best.objective + 1.0e-7) {
              best = candidate_result;
              improved = true;
            }
          }
        }

        if (!improved) {
          break;
        }
        result = best;
        ++total_iterations;
      }

      linear_step *= refinement_step_scale_;
      angular_step *= refinement_step_scale_;
    }

    result.iterations = total_iterations;
    return result;
  }

  MatchResult coarse_match(
    const Pose2D & seed, const std::vector<cv::Point2f> & points) const
  {
    MatchResult best = make_match_result(seed, seed, evaluate_pose(seed, points));
    const int linear_steps = static_cast<int>(
      std::ceil(coarse_linear_range_ / coarse_linear_step_));
    const int angular_steps = static_cast<int>(
      std::ceil(coarse_angular_range_ / coarse_angular_step_));

    for (int x_step = -linear_steps; x_step <= linear_steps; ++x_step) {
      const double dx = static_cast<double>(x_step) * coarse_linear_step_;
      for (int y_step = -linear_steps; y_step <= linear_steps; ++y_step) {
        const double dy = static_cast<double>(y_step) * coarse_linear_step_;
        if (std::hypot(dx, dy) > coarse_linear_range_ + 1.0e-9) {
          continue;
        }
        for (int yaw_step = -angular_steps; yaw_step <= angular_steps; ++yaw_step) {
          const double dyaw = static_cast<double>(yaw_step) * coarse_angular_step_;
          if (std::abs(dyaw) > coarse_angular_range_ + 1.0e-9) {
            continue;
          }
          Pose2D candidate;
          candidate.x = seed.x + dx;
          candidate.y = seed.y + dy;
          candidate.yaw = normalize_angle(seed.yaw + dyaw);
          const MatchResult candidate_result =
            make_match_result(candidate, seed, evaluate_pose(candidate, points));
          if (candidate_result.objective > best.objective + 1.0e-6) {
            best = candidate_result;
          }
        }
      }
    }
    best.used_coarse_search = true;
    return best;
  }

  MatchResult match_scan(
    const Pose2D & seed, const std::vector<cv::Point2f> & points,
    const bool coarse_search_due, const bool force_coarse_search) const
  {
    const ScoreResult seed_score = evaluate_pose(seed, points);
    MatchResult result = refine_match(
      seed, seed, points, max_translation_correction_, max_rotation_correction_);
    result.seed_score = seed_score.score;

    const bool run_coarse = coarse_search_enabled_ && coarse_search_due &&
      (force_coarse_search || result.score < coarse_trigger_score_);
    if (!run_coarse) {
      return result;
    }

    const MatchResult coarse = coarse_match(seed, points);
    MatchResult refined = refine_match(
      seed, coarse.pose, points, coarse_linear_range_, coarse_angular_range_);
    refined.seed_score = seed_score.score;
    refined.used_coarse_search = true;
    result.used_coarse_search = true;
    if (refined.objective > result.objective) {
      return refined;
    }
    return result;
  }

  bool lookup_transform(
    const std::string & target, const std::string & source, tf2::Transform & result)
  {
    if (target == source) {
      result.setIdentity();
      return true;
    }

    try {
      const auto transform = tf_buffer_->lookupTransform(
        target, source, tf2::TimePointZero, tf2::durationFromSec(tf_lookup_timeout_));
      tf2::fromMsg(transform.transform, result);
      return true;
    } catch (const tf2::TransformException & exception) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Waiting for TF %s -> %s: %s",
        target.c_str(), source.c_str(), exception.what());
      return false;
    }
  }

  bool lookup_transform_at(
    const std::string & target, const std::string & source,
    const rclcpp::Time & stamp, tf2::Transform & result)
  {
    if (target == source) {
      result.setIdentity();
      return true;
    }

    try {
      const auto transform = tf_buffer_->lookupTransform(
        target, source, stamp, rclcpp::Duration::from_seconds(tf_lookup_timeout_));
      tf2::fromMsg(transform.transform, result);
      return true;
    } catch (const tf2::TransformException & exception) {
      RCLCPP_DEBUG(
        get_logger(), "TF at scan time unavailable for %s -> %s (%s); trying latest",
        target.c_str(), source.c_str(), exception.what());
      return lookup_transform(target, source, result);
    }
  }

  bool lookup_transform_at_exact(
    const std::string & target, const std::string & source,
    const rclcpp::Time & stamp, tf2::Transform & result)
  {
    if (target == source) {
      result.setIdentity();
      return true;
    }

    try {
      const auto transform = tf_buffer_->lookupTransform(
        target, source, stamp, rclcpp::Duration::from_seconds(tf_lookup_timeout_));
      tf2::fromMsg(transform.transform, result);
      return true;
    } catch (const tf2::TransformException & exception) {
      RCLCPP_DEBUG(
        get_logger(), "Exact TF unavailable for %s -> %s at %.6f: %s",
        target.c_str(), source.c_str(), stamp.seconds(), exception.what());
      return false;
    }
  }

  bool is_isolated_range(
    const sensor_msgs::msg::LaserScan & scan, const std::size_t index) const
  {
    if (!isolated_point_filter_enabled_ || index == 0 || index + 1 >= scan.ranges.size()) {
      return false;
    }
    const double current = scan.ranges[index];
    const double previous = scan.ranges[index - 1];
    const double next = scan.ranges[index + 1];
    const bool separated_from_previous =
      !std::isfinite(previous) || std::abs(current - previous) > isolated_range_threshold_;
    const bool separated_from_next =
      !std::isfinite(next) || std::abs(current - next) > isolated_range_threshold_;
    return separated_from_previous && separated_from_next;
  }

  std::vector<cv::Point2f> scan_points_in_base(
    const sensor_msgs::msg::LaserScan & scan, const tf2::Transform & scan_to_base,
    const tf2::Transform & odom_to_base_start,
    const tf2::Transform * odom_to_base_end) const
  {
    std::vector<cv::Point2f> points;
    const std::size_t stride = std::max<std::size_t>(
      1, (scan.ranges.size() + static_cast<std::size_t>(max_beams_) - 1) /
      static_cast<std::size_t>(max_beams_));
    points.reserve(std::min<std::size_t>(scan.ranges.size(), max_beams_));

    const double accepted_min =
      min_laser_range_ > 0.0 ?
      std::max(static_cast<double>(scan.range_min), min_laser_range_) :
      static_cast<double>(scan.range_min);
    const double accepted_max =
      max_laser_range_ > 0.0 ?
      std::min(static_cast<double>(scan.range_max), max_laser_range_) :
      static_cast<double>(scan.range_max);

    const bool deskew = odom_to_base_end != nullptr;
    const Pose2D start_pose = transform_to_pose(odom_to_base_start);
    const Pose2D end_pose = deskew ? transform_to_pose(*odom_to_base_end) : start_pose;
    const tf2::Transform base_start_from_odom = odom_to_base_start.inverse();

    for (std::size_t index = 0; index < scan.ranges.size(); index += stride) {
      const double range = scan.ranges[index];
      if (!std::isfinite(range) || range < accepted_min || range > accepted_max) {
        continue;
      }
      if (is_isolated_range(scan, index)) {
        continue;
      }

      const double angle =
        static_cast<double>(scan.angle_min) +
        static_cast<double>(index) * static_cast<double>(scan.angle_increment);
      const tf2::Vector3 scan_point(range * std::cos(angle), range * std::sin(angle), 0.0);
      tf2::Vector3 base_point = scan_to_base * scan_point;
      if (deskew && scan.ranges.size() > 1) {
        const double fraction = static_cast<double>(index) /
          static_cast<double>(scan.ranges.size() - 1);
        Pose2D interpolated_pose;
        interpolated_pose.x = start_pose.x + fraction * (end_pose.x - start_pose.x);
        interpolated_pose.y = start_pose.y + fraction * (end_pose.y - start_pose.y);
        interpolated_pose.yaw = normalize_angle(
          start_pose.yaw + fraction * normalize_angle(end_pose.yaw - start_pose.yaw));
        base_point =
          base_start_from_odom * pose_to_transform(interpolated_pose) * base_point;
      }
      points.emplace_back(
        static_cast<float>(base_point.x()), static_cast<float>(base_point.y()));
    }
    return points;
  }

  std::vector<cv::Point2f> filter_static_wall_points(
    const std::vector<cv::Point2f> & points, const Pose2D & predicted_pose) const
  {
    if (!static_wall_filter_enabled_) {
      return points;
    }

    std::vector<cv::Point2f> filtered;
    filtered.reserve(points.size());
    const double cos_yaw = std::cos(predicted_pose.yaw);
    const double sin_yaw = std::sin(predicted_pose.yaw);
    for (const auto & point : points) {
      const double world_x =
        predicted_pose.x + cos_yaw * static_cast<double>(point.x) -
        sin_yaw * static_cast<double>(point.y);
      const double world_y =
        predicted_pose.y + sin_yaw * static_cast<double>(point.x) +
        cos_yaw * static_cast<double>(point.y);
      double wall_distance = 0.0;
      if (sample_static_distance(world_x, world_y, wall_distance) &&
        wall_distance <= static_wall_max_distance_)
      {
        filtered.push_back(point);
      }
    }
    return filtered;
  }

  static tf2::Transform pose_to_transform(const Pose2D & pose)
  {
    tf2::Quaternion rotation;
    rotation.setRPY(0.0, 0.0, pose.yaw);
    return tf2::Transform(rotation, tf2::Vector3(pose.x, pose.y, 0.0));
  }

  static Pose2D transform_to_pose(const tf2::Transform & transform)
  {
    Pose2D pose;
    pose.x = transform.getOrigin().x();
    pose.y = transform.getOrigin().y();
    double roll = 0.0;
    double pitch = 0.0;
    tf2::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, pose.yaw);
    pose.yaw = normalize_angle(pose.yaw);
    return pose;
  }

  static const char * tracking_state_name(const TrackingState state)
  {
    switch (state) {
      case TrackingState::kLocked:
        return "LOCKED";
      case TrackingState::kDegraded:
        return "DEGRADED";
      case TrackingState::kLost:
        return "LOST";
    }
    return "UNKNOWN";
  }

  Pose2D bounded_correction(
    const Pose2D & predicted, const Pose2D & matched, const double gain,
    const double max_translation_step, const double max_rotation_step) const
  {
    const double dx = matched.x - predicted.x;
    const double dy = matched.y - predicted.y;
    const double dyaw = normalize_angle(matched.yaw - predicted.yaw);
    const double translation = std::hypot(dx, dy);
    double scale = std::clamp(gain, 0.0, 1.0);
    if (translation > 1.0e-9) {
      scale = std::min(scale, max_translation_step / translation);
    }
    if (std::abs(dyaw) > 1.0e-9) {
      scale = std::min(scale, max_rotation_step / std::abs(dyaw));
    }

    Pose2D corrected;
    corrected.x = predicted.x + scale * dx;
    corrected.y = predicted.y + scale * dy;
    corrected.yaw = normalize_angle(predicted.yaw + scale * dyaw);
    return corrected;
  }

  bool correction_is_plausible(
    const Pose2D & predicted, const MatchResult & match) const
  {
    const double translation =
      std::hypot(match.pose.x - predicted.x, match.pose.y - predicted.y);
    const double rotation =
      std::abs(normalize_angle(match.pose.yaw - predicted.yaw));
    if (translation <= large_correction_translation_ &&
      rotation <= large_correction_rotation_)
    {
      return true;
    }
    return match.score >= match.seed_score + min_score_improvement_;
  }

  bool update_tracking_state(
    const MatchResult & match, const bool plausible_correction)
  {
    const TrackingState previous_state = tracking_state_;
    const bool enough_inliers = match.inlier_ratio >= min_inlier_ratio_;
    const bool acceptable = plausible_correction && enough_inliers &&
      match.score >= lost_score_;
    const bool strong = plausible_correction && enough_inliers &&
      match.score >= locked_score_;
    const bool recovery_quality = plausible_correction && enough_inliers &&
      match.score >= recovery_score_;

    if (!acceptable) {
      ++low_quality_frames_;
      recovery_frames_ = 0;
      if (tracking_state_ == TrackingState::kLocked) {
        tracking_state_ = TrackingState::kDegraded;
      }
      if (low_quality_frames_ >= lost_confirm_frames_) {
        tracking_state_ = TrackingState::kLost;
      }
    } else {
      low_quality_frames_ = 0;
      if (tracking_state_ == TrackingState::kLost) {
        recovery_frames_ = recovery_quality ? recovery_frames_ + 1 : 0;
        if (recovery_frames_ >= recovery_confirm_frames_) {
          tracking_state_ = TrackingState::kDegraded;
          recovery_frames_ = 0;
        }
      } else if (strong) {
        if (tracking_state_ == TrackingState::kDegraded) {
          ++recovery_frames_;
          if (recovery_frames_ >= recovery_confirm_frames_) {
            tracking_state_ = TrackingState::kLocked;
            recovery_frames_ = 0;
          }
        } else {
          recovery_frames_ = 0;
        }
      } else {
        tracking_state_ = TrackingState::kDegraded;
        recovery_frames_ = 0;
      }
    }

    if (tracking_state_ != previous_state) {
      RCLCPP_WARN(
        get_logger(), "Localization state %s -> %s (score=%.3f robust=%.3f inliers=%.2f)",
        tracking_state_name(previous_state), tracking_state_name(tracking_state_),
        match.score, match.robust_score, match.inlier_ratio);
    }
    // Strong matches may make a bounded correction while DEGRADED so the
    // estimate does not rely solely on odometry. LOST and weak matches never
    // modify map -> odom.
    return strong && tracking_state_ != TrackingState::kLost;
  }

  void publish_diagnostics(
    const MatchResult & match, const Pose2D & predicted,
    const std::size_t valid_beams) const
  {
    std_msgs::msg::Float32 float_message;
    float_message.data = static_cast<float>(match.score);
    match_score_pub_->publish(float_message);
    float_message.data = static_cast<float>(match.robust_score);
    robust_score_pub_->publish(float_message);
    float_message.data = static_cast<float>(match.inlier_ratio);
    inlier_ratio_pub_->publish(float_message);
    float_message.data = static_cast<float>(
      std::hypot(match.pose.x - predicted.x, match.pose.y - predicted.y));
    correction_distance_pub_->publish(float_message);
    float_message.data = static_cast<float>(
      normalize_angle(match.pose.yaw - predicted.yaw));
    correction_yaw_pub_->publish(float_message);

    std_msgs::msg::UInt32 uint_message;
    uint_message.data = static_cast<std::uint32_t>(std::min<std::size_t>(
        valid_beams, std::numeric_limits<std::uint32_t>::max()));
    valid_beams_pub_->publish(uint_message);
    uint_message.data = static_cast<std::uint32_t>(std::max(0, match.iterations));
    iterations_pub_->publish(uint_message);

    std_msgs::msg::String state_message;
    state_message.data = tracking_state_name(tracking_state_);
    state_pub_->publish(state_message);
    std_msgs::msg::Bool bool_message;
    bool_message.data = match.used_coarse_search;
    coarse_search_used_pub_->publish(bool_message);
  }

  void publish_scan_cloud(const std::vector<cv::Point2f> & points)
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = now();
    cloud.header.frame_id = global_frame_;
    cloud.height = 1;
    cloud.is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> x_iterator(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y_iterator(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> z_iterator(cloud, "z");
    const double cos_yaw = std::cos(pose_.yaw);
    const double sin_yaw = std::sin(pose_.yaw);

    for (const auto & point : points) {
      *x_iterator = static_cast<float>(
        pose_.x + cos_yaw * static_cast<double>(point.x) -
        sin_yaw * static_cast<double>(point.y));
      *y_iterator = static_cast<float>(
        pose_.y + sin_yaw * static_cast<double>(point.x) +
        cos_yaw * static_cast<double>(point.y));
      *z_iterator = 0.05F;
      ++x_iterator;
      ++y_iterator;
      ++z_iterator;
    }

    scan_cloud_pub_->publish(cloud);
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
  {
    if (!map_ready_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000, "Waiting for a valid map on %s",
        map_topic_.c_str());
      return;
    }
    if (!has_pose_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000, "Waiting for an initial pose on %s",
        initial_pose_topic_.c_str());
      return;
    }

    const std::string scan_frame =
      laser_frame_.empty() ? normalize_frame(msg->header.frame_id) : laser_frame_;
    if (scan_frame.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000, "LaserScan has no frame_id");
      return;
    }

    tf2::Transform scan_to_base;
    if (!lookup_transform(base_frame_, scan_frame, scan_to_base)) {
      return;
    }

    const rclcpp::Time scan_stamp(msg->header.stamp, get_clock()->get_clock_type());
    tf2::Transform odom_to_base;
    const bool exact_start_transform =
      lookup_transform_at_exact(odom_frame_, base_frame_, scan_stamp, odom_to_base);
    if (!exact_start_transform &&
      !lookup_transform_at(odom_frame_, base_frame_, scan_stamp, odom_to_base))
    {
      return;
    }

    tf2::Transform odom_to_base_end;
    const tf2::Transform * odom_to_base_end_ptr = nullptr;
    double scan_duration = static_cast<double>(msg->scan_time);
    if (msg->ranges.size() > 1 && msg->time_increment > 0.0F) {
      scan_duration = static_cast<double>(msg->time_increment) *
        static_cast<double>(msg->ranges.size() - 1);
    }
    if (deskew_enabled_ && exact_start_transform && scan_duration > 0.0 &&
      scan_duration <= deskew_max_scan_time_)
    {
      const rclcpp::Time end_stamp =
        scan_stamp + rclcpp::Duration::from_seconds(scan_duration);
      if (lookup_transform_at_exact(
          odom_frame_, base_frame_, end_stamp, odom_to_base_end))
      {
        odom_to_base_end_ptr = &odom_to_base_end;
      }
    }

    const auto raw_points = scan_points_in_base(
      *msg, scan_to_base, odom_to_base, odom_to_base_end_ptr);
    if (raw_points.size() < static_cast<std::size_t>(min_valid_beams_)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Only %zu valid lidar beams; at least %d are required",
        raw_points.size(), min_valid_beams_);
      return;
    }

    Pose2D predicted_pose = pose_;
    if (has_map_to_odom_) {
      predicted_pose = transform_to_pose(map_to_odom_ * odom_to_base);
    }

    const auto points = filter_static_wall_points(raw_points, predicted_pose);
    const int required_points =
      static_wall_filter_enabled_ ? static_wall_min_points_ : min_valid_beams_;
    if (points.size() < static_cast<std::size_t>(required_points)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Static-wall filter kept %zu/%zu lidar points; at least %d are required",
        points.size(), raw_points.size(), required_points);
      return;
    }

    ++scan_sequence_;
    const bool coarse_search_due =
      scan_sequence_ - last_coarse_search_scan_ >= coarse_search_interval_frames_;
    const bool force_coarse_search = tracking_state_ != TrackingState::kLocked;
    const MatchResult match = match_scan(
      predicted_pose, points, coarse_search_due, force_coarse_search);
    if (match.used_coarse_search) {
      last_coarse_search_scan_ = scan_sequence_;
    }

    const bool plausible_correction = correction_is_plausible(predicted_pose, match);
    const bool accept_match = update_tracking_state(match, plausible_correction);
    if (accept_match) {
      if (tracking_state_ == TrackingState::kLocked) {
        pose_ = bounded_correction(
          predicted_pose, match.pose, 1.0,
          max_locked_translation_step_, max_locked_rotation_step_);
      } else {
        pose_ = bounded_correction(
          predicted_pose, match.pose, degraded_correction_gain_,
          max_degraded_translation_step_, max_degraded_rotation_step_);
      }
    } else {
      pose_ = predicted_pose;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Rejected scan match: state=%s score=%.3f robust=%.3f inliers=%.2f plausible=%s; "
        "using odometry prediction",
        tracking_state_name(tracking_state_), match.score, match.robust_score,
        match.inlier_ratio, plausible_correction ? "true" : "false");
    }

    map_to_odom_ = pose_to_transform(pose_) * odom_to_base.inverse();
    has_map_to_odom_ = true;

    publish_pose();
    publish_scan_cloud(points);
    publish_diagnostics(match, predicted_pose, points.size());
    broadcast_map_to_odom();

    RCLCPP_DEBUG(
      get_logger(),
      "pose=(%.3f, %.3f, %.1f deg), state=%s score=%.3f robust=%.3f "
      "inliers=%.2f wall_points=%zu/%zu iterations=%d coarse=%s deskew=%s",
      pose_.x, pose_.y, pose_.yaw * 180.0 / kPi,
      tracking_state_name(tracking_state_), match.score, match.robust_score,
      match.inlier_ratio, points.size(), raw_points.size(), match.iterations,
      match.used_coarse_search ? "true" : "false",
      odom_to_base_end_ptr != nullptr ? "true" : "false");
  }

  void publish_pose()
  {
    geometry_msgs::msg::PoseWithCovarianceStamped message;
    message.header.stamp = now();
    message.header.frame_id = global_frame_;
    message.pose.pose.position.x = pose_.x;
    message.pose.pose.position.y = pose_.y;
    message.pose.pose.position.z = 0.0;
    tf2::Quaternion rotation;
    rotation.setRPY(0.0, 0.0, pose_.yaw);
    message.pose.pose.orientation = tf2::toMsg(rotation);

    double covariance_scale = 1.0;
    if (tracking_state_ == TrackingState::kDegraded) {
      covariance_scale = 4.0;
    } else if (tracking_state_ == TrackingState::kLost) {
      covariance_scale = 25.0;
    }
    message.pose.covariance[0] = covariance_x_ * covariance_scale;
    message.pose.covariance[7] = covariance_y_ * covariance_scale;
    message.pose.covariance[14] = 1.0e6;
    message.pose.covariance[21] = 1.0e6;
    message.pose.covariance[28] = 1.0e6;
    message.pose.covariance[35] = covariance_yaw_ * covariance_scale;
    pose_pub_->publish(message);
  }

  void broadcast_map_to_odom()
  {
    if (!tf_broadcast_ || !has_map_to_odom_) {
      return;
    }

    geometry_msgs::msg::TransformStamped message;
    message.header.stamp =
      now() + rclcpp::Duration::from_seconds(transform_tolerance_);
    message.header.frame_id = global_frame_;
    message.child_frame_id = odom_frame_;
    message.transform = tf2::toMsg(map_to_odom_);
    tf_broadcaster_->sendTransform(message);
  }

  std::string global_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string laser_frame_;
  std::string map_topic_;
  std::string scan_topic_;
  std::string initial_pose_topic_;
  std::string pose_topic_;
  std::string scan_cloud_topic_;

  bool tf_broadcast_{true};
  double tf_publish_rate_{30.0};
  double transform_tolerance_{0.3};
  double tf_lookup_timeout_{0.2};
  bool set_initial_pose_{true};
  bool reset_pose_on_map_{false};
  int occupied_threshold_{65};
  double likelihood_field_radius_{1.25};
  double linear_search_step_{0.0};
  double angular_search_step_{kPi / 180.0};
  int refinement_levels_{3};
  double refinement_step_scale_{0.5};
  int max_iterations_{80};
  double max_translation_correction_{0.5};
  double max_rotation_correction_{20.0 * kPi / 180.0};
  int max_beams_{180};
  int min_valid_beams_{20};
  double min_match_score_{0.05};
  double lost_score_{0.20};
  double recovery_score_{0.45};
  double locked_score_{0.55};
  int lost_confirm_frames_{3};
  int recovery_confirm_frames_{5};
  double min_inlier_ratio_{0.45};
  double inlier_likelihood_threshold_{0.35};
  double trim_ratio_{0.25};
  double min_score_improvement_{0.02};
  double large_correction_translation_{0.10};
  double large_correction_rotation_{5.0 * kPi / 180.0};
  double degraded_correction_gain_{0.25};
  double max_locked_translation_step_{0.15};
  double max_locked_rotation_step_{5.0 * kPi / 180.0};
  double max_degraded_translation_step_{0.05};
  double max_degraded_rotation_step_{2.0 * kPi / 180.0};
  bool isolated_point_filter_enabled_{true};
  double isolated_range_threshold_{0.10};
  bool static_wall_filter_enabled_{true};
  double static_wall_max_distance_{0.25};
  int static_wall_min_points_{20};
  bool deskew_enabled_{true};
  double deskew_max_scan_time_{0.15};
  bool coarse_search_enabled_{true};
  double coarse_trigger_score_{0.45};
  double coarse_linear_range_{0.75};
  double coarse_angular_range_{30.0 * kPi / 180.0};
  double coarse_linear_step_{0.10};
  double coarse_angular_step_{3.0 * kPi / 180.0};
  int coarse_search_interval_frames_{3};
  double prior_translation_weight_{0.05};
  double prior_rotation_weight_{0.02};
  double min_laser_range_{0.0};
  double max_laser_range_{0.0};
  double covariance_x_{0.05};
  double covariance_y_{0.05};
  double covariance_yaw_{0.03};

  nav_msgs::msg::OccupancyGrid map_;
  cv::Mat likelihood_field_;
  cv::Mat static_distance_field_;
  double map_origin_yaw_{0.0};
  bool map_ready_{false};
  Pose2D initial_pose_;
  Pose2D pose_;
  bool has_pose_{false};
  tf2::Transform map_to_odom_;
  bool has_map_to_odom_{false};
  TrackingState tracking_state_{TrackingState::kDegraded};
  int low_quality_frames_{0};
  int recovery_frames_{0};
  std::int64_t scan_sequence_{0};
  std::int64_t last_coarse_search_scan_{-3};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initial_pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr match_score_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr robust_score_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr inlier_ratio_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr correction_distance_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr correction_yaw_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr valid_beams_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr iterations_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr coarse_search_used_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_cloud_pub_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarLocationNode>());
  rclcpp::shutdown();
  return 0;
}
