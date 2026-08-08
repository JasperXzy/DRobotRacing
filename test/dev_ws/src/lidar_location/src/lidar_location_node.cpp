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
#include <cstdio>
#include <functional>
#include <memory>
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

    tf_broadcast_ = declare_parameter<bool>("tf_broadcast", true);
    tf_publish_rate_ = std::max(1.0, declare_parameter<double>("tf_publish_rate", 30.0));
    transform_tolerance_ =
      std::max(0.0, declare_parameter<double>("transform_tolerance", 0.3));
    tf_lookup_timeout_ =
      std::max(0.01, declare_parameter<double>("tf_lookup_timeout", 0.2));

    deskew_enabled_ = declare_parameter<bool>("deskew_enabled", true);
    deskew_min_translation_ =
      std::max(0.0, declare_parameter<double>("deskew_min_translation", 0.002));
    deskew_min_rotation_ =
      std::max(0.0, declare_parameter<double>("deskew_min_rotation_deg", 0.05)) *
      kPi / 180.0;
    deskew_max_scan_duration_ =
      std::max(0.01, declare_parameter<double>("deskew_max_scan_duration", 0.2));
    publish_deskewed_cloud_ =
      declare_parameter<bool>("publish_deskewed_cloud", true);

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
    max_iterations_ = static_cast<int>(std::max<std::int64_t>(
        1, declare_parameter<std::int64_t>("max_iterations", 80)));
    max_beams_ = static_cast<int>(std::max<std::int64_t>(
        1, declare_parameter<std::int64_t>("max_beams", 180)));
    min_valid_beams_ = static_cast<int>(std::max<std::int64_t>(
        1, declare_parameter<std::int64_t>("min_valid_beams", 20)));
    min_match_score_ =
      std::clamp(declare_parameter<double>("min_match_score", 0.05), 0.0, 1.0);
    correction_translation_deadband_ = std::max(
      0.0, declare_parameter<double>("correction_translation_deadband", 0.0));
    correction_rotation_deadband_ = std::max(
      0.0, declare_parameter<double>("correction_rotation_deadband_deg", 0.0)) *
      kPi / 180.0;
    match_score_terminal_rate_ = std::max(
      0.0, declare_parameter<double>("match_score_terminal_rate", 1.0));
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
    deskew_applied_pub_ =
      create_publisher<std_msgs::msg::Bool>("~/deskew_applied", rclcpp::QoS(10));
    if (publish_deskewed_cloud_) {
      deskewed_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "~/deskewed_points", rclcpp::SensorDataQoS().keep_last(1));
    }

    const auto tf_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / tf_publish_rate_));
    tf_timer_ = create_wall_timer(
      tf_period, std::bind(&LidarLocationNode::broadcast_map_to_odom, this));
    if (match_score_terminal_rate_ > 0.0) {
      const auto score_terminal_period =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / match_score_terminal_rate_));
      match_score_terminal_timer_ = create_wall_timer(
        score_terminal_period, [this]() {
          if (!latest_match_score_valid_) {
            return;
          }
          // 总系统在终端1以WARN级别启动本节点；直接写stdout，确保固定频率
          // 的匹配度不会被ROS日志等级过滤，也不额外发布rosout消息。
          std::printf(
            "[lidar_location] match_score=%.3f\n", latest_match_score_);
          std::fflush(stdout);
        });
    }

    RCLCPP_INFO(
      get_logger(),
      "ROS 2 lidar localization ready: map=%s scan=%s frames=%s->%s->%s",
      map_topic_.c_str(), scan_topic_.c_str(), global_frame_.c_str(),
      odom_frame_.c_str(), base_frame_.c_str());
  }

private:
  struct Pose2D
  {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
  };

  struct MatchResult
  {
    Pose2D pose;
    double score{0.0};
    int iterations{0};
  };

  void set_pose(const Pose2D & pose, const char * source)
  {
    pose_ = pose;
    pose_.yaw = normalize_angle(pose_.yaw);
    has_pose_ = true;
    has_map_to_odom_ = false;
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

    for (int y = 0; y < distance_pixels.rows; ++y) {
      for (int x = 0; x < distance_pixels.cols; ++x) {
        const std::size_t index =
          static_cast<std::size_t>(y) * map_.info.width + static_cast<std::size_t>(x);
        if (map_.data[index] < 0) {
          continue;
        }
        const double distance_m =
          static_cast<double>(distance_pixels.at<float>(y, x)) * map_.info.resolution;
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
    const double world_x, const double world_y, int & grid_x, int & grid_y) const
  {
    const double dx = world_x - map_.info.origin.position.x;
    const double dy = world_y - map_.info.origin.position.y;
    const double cos_origin = std::cos(map_origin_yaw_);
    const double sin_origin = std::sin(map_origin_yaw_);
    const double local_x = cos_origin * dx + sin_origin * dy;
    const double local_y = -sin_origin * dx + cos_origin * dy;

    grid_x = static_cast<int>(std::lround(local_x / map_.info.resolution));
    grid_y = static_cast<int>(std::lround(local_y / map_.info.resolution));
    return grid_x >= 0 && grid_y >= 0 &&
           grid_x < likelihood_field_.cols && grid_y < likelihood_field_.rows;
  }

  double score_pose(const Pose2D & pose, const std::vector<cv::Point2f> & points) const
  {
    if (points.empty()) {
      return 0.0;
    }

    const double cos_yaw = std::cos(pose.yaw);
    const double sin_yaw = std::sin(pose.yaw);
    double sum = 0.0;

    for (const auto & point : points) {
      const double world_x =
        pose.x + cos_yaw * static_cast<double>(point.x) -
        sin_yaw * static_cast<double>(point.y);
      const double world_y =
        pose.y + sin_yaw * static_cast<double>(point.x) +
        cos_yaw * static_cast<double>(point.y);

      int grid_x = 0;
      int grid_y = 0;
      if (world_to_grid(world_x, world_y, grid_x, grid_y)) {
        sum += likelihood_field_.at<float>(grid_y, grid_x);
      }
    }

    return sum / static_cast<double>(points.size());
  }

  MatchResult match_scan(
    const Pose2D & seed, const std::vector<cv::Point2f> & points) const
  {
    const double linear_step =
      linear_search_step_ > 0.0 ? linear_search_step_ : map_.info.resolution;
    const std::array<std::pair<double, double>, 5> translations = {{
      {0.0, 0.0},
      {linear_step, 0.0},
      {-linear_step, 0.0},
      {0.0, linear_step},
      {0.0, -linear_step},
    }};
    const std::array<double, 3> rotations = {
      0.0, angular_search_step_, -angular_search_step_};

    MatchResult result;
    result.pose = seed;
    result.score = score_pose(seed, points);

    for (int iteration = 0; iteration < max_iterations_; ++iteration) {
      Pose2D best_pose = result.pose;
      double best_score = result.score;
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
          const double candidate_score = score_pose(candidate, points);
          if (candidate_score > best_score + 1.0e-6) {
            best_pose = candidate;
            best_score = candidate_score;
            improved = true;
          }
        }
      }

      if (!improved) {
        break;
      }
      result.pose = best_pose;
      result.score = best_score;
      result.iterations = iteration + 1;
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

  bool lookup_transform_at_strict(
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
        get_logger(), "Exact TF unavailable for deskew %s -> %s at %.6f: %s",
        target.c_str(), source.c_str(), stamp.seconds(), exception.what());
      return false;
    }
  }

  std::vector<cv::Point2f> scan_points_in_base(
    const sensor_msgs::msg::LaserScan & scan, const tf2::Transform & scan_to_base,
    const tf2::Transform & odom_to_base_start,
    const tf2::Transform & odom_to_base_reference,
    const bool apply_deskew, const std::size_t beam_limit) const
  {
    std::vector<cv::Point2f> points;
    const std::size_t stride = beam_limit > 0 ? std::max<std::size_t>(
      1, (scan.ranges.size() + beam_limit - 1) / beam_limit) : 1;
    points.reserve(
      beam_limit > 0 ? std::min(scan.ranges.size(), beam_limit) : scan.ranges.size());

    const Pose2D start_pose = transform_to_pose(odom_to_base_start);
    const Pose2D reference_pose = transform_to_pose(odom_to_base_reference);
    const double yaw_delta = normalize_angle(reference_pose.yaw - start_pose.yaw);
    const tf2::Transform reference_to_odom = odom_to_base_reference.inverse();
    const double denominator =
      scan.ranges.size() > 1 ? static_cast<double>(scan.ranges.size() - 1) : 1.0;

    const double accepted_min =
      min_laser_range_ > 0.0 ?
      std::max(static_cast<double>(scan.range_min), min_laser_range_) :
      static_cast<double>(scan.range_min);
    const double accepted_max =
      max_laser_range_ > 0.0 ?
      std::min(static_cast<double>(scan.range_max), max_laser_range_) :
      static_cast<double>(scan.range_max);

    for (std::size_t index = 0; index < scan.ranges.size(); index += stride) {
      const double range = scan.ranges[index];
      if (!std::isfinite(range) || range < accepted_min || range > accepted_max) {
        continue;
      }

      const double angle =
        static_cast<double>(scan.angle_min) +
        static_cast<double>(index) * static_cast<double>(scan.angle_increment);
      const tf2::Vector3 scan_point(range * std::cos(angle), range * std::sin(angle), 0.0);
      tf2::Vector3 base_point = scan_to_base * scan_point;
      if (apply_deskew) {
        const double alpha = std::clamp(static_cast<double>(index) / denominator, 0.0, 1.0);
        Pose2D beam_pose;
        beam_pose.x = start_pose.x + alpha * (reference_pose.x - start_pose.x);
        beam_pose.y = start_pose.y + alpha * (reference_pose.y - start_pose.y);
        beam_pose.yaw = normalize_angle(start_pose.yaw + alpha * yaw_delta);
        base_point = reference_to_odom * pose_to_transform(beam_pose) * base_point;
      }
      points.emplace_back(
        static_cast<float>(base_point.x()), static_cast<float>(base_point.y()));
    }
    return points;
  }

  void publish_deskewed_cloud(
    const std::vector<cv::Point2f> & points, const rclcpp::Time & reference_stamp)
  {
    if (!deskewed_cloud_pub_) {
      return;
    }

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = reference_stamp;
    cloud.header.frame_id = base_frame_;
    cloud.height = 1;
    cloud.width = static_cast<std::uint32_t>(points.size());
    cloud.is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());
    sensor_msgs::PointCloud2Iterator<float> x_iterator(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y_iterator(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> z_iterator(cloud, "z");
    for (const auto & point : points) {
      *x_iterator = point.x;
      *y_iterator = point.y;
      *z_iterator = 0.0F;
      ++x_iterator;
      ++y_iterator;
      ++z_iterator;
    }
    deskewed_cloud_pub_->publish(cloud);
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
    rclcpp::Time reference_stamp = scan_stamp;
    tf2::Transform odom_to_base_start;
    tf2::Transform odom_to_base_reference;
    bool have_exact_start = false;
    bool have_reference_pose = false;
    bool deskew_applied = false;
    double scan_translation = 0.0;
    double scan_rotation = 0.0;

    const double beam_time_increment = static_cast<double>(msg->time_increment);
    const double acquisition_duration = msg->ranges.size() > 1 ?
      static_cast<double>(msg->ranges.size() - 1) * beam_time_increment : 0.0;
    const bool valid_deskew_timing =
      msg->ranges.size() > 1 && std::isfinite(beam_time_increment) &&
      beam_time_increment > 0.0 && std::isfinite(acquisition_duration) &&
      acquisition_duration > 0.0 && acquisition_duration <= deskew_max_scan_duration_;

    if (deskew_enabled_ && valid_deskew_timing) {
      reference_stamp = scan_stamp + rclcpp::Duration::from_seconds(acquisition_duration);
      have_exact_start = lookup_transform_at_strict(
        odom_frame_, base_frame_, scan_stamp, odom_to_base_start);
      const bool have_exact_reference = lookup_transform_at_strict(
        odom_frame_, base_frame_, reference_stamp, odom_to_base_reference);
      if (have_exact_start && have_exact_reference) {
        have_reference_pose = true;
        const Pose2D start_pose = transform_to_pose(odom_to_base_start);
        const Pose2D reference_pose = transform_to_pose(odom_to_base_reference);
        scan_translation = std::hypot(
          reference_pose.x - start_pose.x, reference_pose.y - start_pose.y);
        scan_rotation = std::abs(normalize_angle(reference_pose.yaw - start_pose.yaw));
        deskew_applied =
          scan_translation >= deskew_min_translation_ ||
          scan_rotation >= deskew_min_rotation_;
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "Exact EKF TF unavailable across scan; using rigid scan without deskew");
      }
    } else if (deskew_enabled_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Invalid LaserScan timing for deskew: beams=%zu time_increment=%.9f duration=%.6f",
        msg->ranges.size(), beam_time_increment, acquisition_duration);
    }

    if (!have_reference_pose) {
      reference_stamp = scan_stamp;
      if (!have_exact_start && !lookup_transform_at(
          odom_frame_, base_frame_, scan_stamp, odom_to_base_start))
      {
        return;
      }
      odom_to_base_reference = odom_to_base_start;
    }

    const auto points = scan_points_in_base(
      *msg, scan_to_base, odom_to_base_start, odom_to_base_reference,
      deskew_applied, static_cast<std::size_t>(max_beams_));
    if (publish_deskewed_cloud_) {
      const auto display_points = scan_points_in_base(
        *msg, scan_to_base, odom_to_base_start, odom_to_base_reference,
        deskew_applied, 0);
      publish_deskewed_cloud(display_points, reference_stamp);
    }
    std_msgs::msg::Bool deskew_message;
    deskew_message.data = deskew_applied;
    deskew_applied_pub_->publish(deskew_message);

    if (points.size() < static_cast<std::size_t>(min_valid_beams_)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Only %zu valid lidar beams; at least %d are required",
        points.size(), min_valid_beams_);
      return;
    }

    Pose2D predicted_pose = pose_;
    if (has_map_to_odom_) {
      predicted_pose = transform_to_pose(map_to_odom_ * odom_to_base_reference);
    }

    const MatchResult match = match_scan(predicted_pose, points);
    const double correction_translation = std::hypot(
      match.pose.x - predicted_pose.x, match.pose.y - predicted_pose.y);
    const double correction_rotation = normalize_angle(
      match.pose.yaw - predicted_pose.yaw);
    bool translation_deadband_applied = false;
    bool rotation_deadband_applied = false;
    if (match.score >= min_match_score_) {
      pose_ = match.pose;
      // Do not apply correction deadbands to the first successful match: the
      // initial map pose still needs to converge without an existing
      // map->odom estimate. After initialization, suppress the translation and
      // rotation components independently so a valid correction on one axis
      // does not reintroduce jitter on the other.
      if (has_map_to_odom_ && correction_translation_deadband_ > 0.0 &&
        correction_translation <= correction_translation_deadband_)
      {
        pose_.x = predicted_pose.x;
        pose_.y = predicted_pose.y;
        translation_deadband_applied = correction_translation > 0.0;
      }
      if (has_map_to_odom_ && correction_rotation_deadband_ > 0.0 &&
        std::abs(correction_rotation) <= correction_rotation_deadband_)
      {
        pose_.yaw = predicted_pose.yaw;
        rotation_deadband_applied = std::abs(correction_rotation) > 0.0;
      }
    } else {
      pose_ = predicted_pose;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Low scan-match score %.3f < %.3f; using odometry prediction",
        match.score, min_match_score_);
    }

    map_to_odom_ = pose_to_transform(pose_) * odom_to_base_reference.inverse();
    has_map_to_odom_ = true;

    publish_pose();
    std_msgs::msg::Float32 score_message;
    score_message.data = static_cast<float>(match.score);
    latest_match_score_ = match.score;
    latest_match_score_valid_ = std::isfinite(latest_match_score_);
    match_score_pub_->publish(score_message);
    broadcast_map_to_odom();

    RCLCPP_DEBUG(
      get_logger(),
      "pose=(%.3f, %.3f, %.1f deg), score=%.3f, iterations=%d, "
      "correction=(%.4f m, %.3f deg) deadband=(%s,%s), "
      "deskew=%s motion=(%.4f m, %.3f deg)",
      pose_.x, pose_.y, pose_.yaw * 180.0 / kPi, match.score, match.iterations,
      correction_translation, correction_rotation * 180.0 / kPi,
      translation_deadband_applied ? "translation" : "no",
      rotation_deadband_applied ? "rotation" : "no",
      deskew_applied ? "yes" : "no", scan_translation, scan_rotation * 180.0 / kPi);
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

    message.pose.covariance[0] = covariance_x_;
    message.pose.covariance[7] = covariance_y_;
    message.pose.covariance[14] = 1.0e6;
    message.pose.covariance[21] = 1.0e6;
    message.pose.covariance[28] = 1.0e6;
    message.pose.covariance[35] = covariance_yaw_;
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

  bool tf_broadcast_{true};
  double tf_publish_rate_{30.0};
  double transform_tolerance_{0.3};
  double tf_lookup_timeout_{0.2};
  bool deskew_enabled_{true};
  double deskew_min_translation_{0.002};
  double deskew_min_rotation_{0.05 * kPi / 180.0};
  double deskew_max_scan_duration_{0.2};
  bool publish_deskewed_cloud_{true};
  bool set_initial_pose_{true};
  bool reset_pose_on_map_{false};
  int occupied_threshold_{65};
  double likelihood_field_radius_{1.25};
  double linear_search_step_{0.0};
  double angular_search_step_{kPi / 180.0};
  int max_iterations_{80};
  int max_beams_{180};
  int min_valid_beams_{20};
  double min_match_score_{0.05};
  double correction_translation_deadband_{0.0};
  double correction_rotation_deadband_{0.0};
  double match_score_terminal_rate_{1.0};
  double min_laser_range_{0.0};
  double max_laser_range_{0.0};
  double covariance_x_{0.05};
  double covariance_y_{0.05};
  double covariance_yaw_{0.03};

  nav_msgs::msg::OccupancyGrid map_;
  cv::Mat likelihood_field_;
  double map_origin_yaw_{0.0};
  bool map_ready_{false};
  Pose2D initial_pose_;
  Pose2D pose_;
  bool has_pose_{false};
  tf2::Transform map_to_odom_;
  bool has_map_to_odom_{false};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initial_pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr match_score_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr deskew_applied_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr deskewed_cloud_pub_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
  rclcpp::TimerBase::SharedPtr match_score_terminal_timer_;
  double latest_match_score_{0.0};
  bool latest_match_score_valid_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarLocationNode>());
  rclcpp::shutdown();
  return 0;
}
