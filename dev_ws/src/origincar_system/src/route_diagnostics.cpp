#include "origincar_system/visual_lidar_path_follower.hpp"

#include <algorithm>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include "follower_utils.hpp"

using origincar_system::to_lower;

namespace
{

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

std::string VisualLidarPathFollower::make_logic3_session_id() const
{
  const std::time_t timestamp = std::time(nullptr);
  std::tm local_time;
  localtime_r(&timestamp, &local_time);
  std::ostringstream stream;
  stream << std::put_time(&local_time, "%Y.%m.%d_%H:%M:%S");
  return stream.str();
}

void VisualLidarPathFollower::start_logic3_actual_route(
  const VisualLidarPathFollower::RobotPose & pose)
{
  logic3_actual_route_.clear();
  logic3_actual_route_recording_ = logic3_actual_route_enabled_;
  record_logic3_actual_route(pose, true);
}

void VisualLidarPathFollower::record_logic3_actual_route(
  const VisualLidarPathFollower::RobotPose & pose, bool force)
{
  if (!logic3_actual_route_recording_ ||
    !std::isfinite(pose.x) || !std::isfinite(pose.y))
  {
    return;
  }
  if (!force && !logic3_actual_route_.empty()) {
    const Point2D & previous = logic3_actual_route_.back();
    if (std::hypot(pose.x - previous.x, pose.y - previous.y) <
      logic3_actual_route_min_distance_)
    {
      return;
    }
  }
  if (force && !logic3_actual_route_.empty()) {
    Point2D & previous = logic3_actual_route_.back();
    if (std::hypot(pose.x - previous.x, pose.y - previous.y) < 1.0e-6) {
      previous = Point2D{pose.x, pose.y};
      return;
    }
  }
  logic3_actual_route_.push_back(Point2D{pose.x, pose.y});
}

void VisualLidarPathFollower::publish_logic3_snapshot(const std::string & event)
{
  if (!logic3_snapshot_pub_ || logic3_session_id_.empty()) {
    return;
  }
  const bool task1_before_red_event = event == "task1_before_red";
  const bool final_before_red_event = event == "final_before_red";
  const bool image_capture_event = event == "image_capture";
  const bool final_transition_marker_event = event == "final_transition_marker";
  const bool task1_event = event == "task1_frozen" ||
    event == "task1_dynamic" || task1_before_red_event;
  const bool dynamic_event = event == "task1_dynamic" ||
    event == "final_dynamic" || task1_before_red_event ||
    final_before_red_event;
  const bool task1_dynamic_event = event == "task1_dynamic" ||
    task1_before_red_event;
  const std::vector<Waypoint> & original_route =
    image_capture_event ?
    (final_original_route_.empty() ? original_waypoints_ : final_original_route_) :
    (task1_dynamic_event && !task1_dynamic_original_route_.empty() ?
    task1_dynamic_original_route_ :
    (task1_event ? task1_original_route_ : final_original_route_));
  const std::vector<Waypoint> & frozen_route =
    image_capture_event ?
    (final_frozen_route_.empty() ? waypoints_ : final_frozen_route_) :
    (task1_dynamic_event && !task1_dynamic_route_.empty() ?
    task1_dynamic_route_ :
    (task1_event ? task1_frozen_route_ : final_frozen_route_));
  // 04图严格以最新03冻结路线为基线，只额外叠加红色历史。
  const std::vector<Waypoint> & plotted_route = frozen_route;
  const bool include_qr_marker =
    task1_transition_marker_valid_ &&
    (event == "task1_dynamic" || final_transition_marker_event ||
    event == "final_frozen" ||
    event == "final_dynamic" || task1_before_red_event ||
    final_before_red_event);
  const bool include_route_start_marker =
    final_frozen_marker_valid_ &&
    (event == "task1_dynamic" || final_transition_marker_event ||
    event == "final_frozen" ||
    event == "final_dynamic" || task1_before_red_event ||
    final_before_red_event);
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
  if (event == "task1_dynamic" || task1_before_red_event) {
    // 02图逐次保留Task1期间真正触发过的雷达避障动作，不去重。
    for (const auto & record : logic3_red_avoidance_records_) {
      if (record.seen_in_task1) {
        plotted_red_clusters.push_back(record.center);
      }
    }
  } else if (event == "final_dynamic" || final_before_red_event) {
    // 04图逐次保留Task1结束后真正触发过的雷达避障动作，不去重。
    for (const auto & record : logic3_red_avoidance_records_) {
      if (record.seen_after_task1) {
        plotted_red_clusters.push_back(record.center);
      }
    }
  }
  const std::size_t orange_green_begin_index =
    logic3_orange_green_boundary_locked_ ?
    std::min(logic3_orange_green_begin_index_, logic3_green_clusters_.size()) :
    logic3_green_clusters_.size();

  std::ostringstream json;
  json << std::setprecision(10);
  json << "{\"event\":\"" << json_escape(event)
       << "\",\"session\":\"" << json_escape(logic3_session_id_)
       << "\",\"frozen_revision\":" << channel_completed_reverse_count_
       << ",\"output_root\":\"" << json_escape(logic3_output_root_)
       << "\",\"map_yaml\":\"" << json_escape(logic3_restriction_map_yaml_)
       << "\",\"red_radius\":" << logic3_red_takeover_radius_
       << ",\"green_radius\":" << logic3_old_cluster_radius_
       << ",\"green_red_filter_radius\":" << logic3_green_red_filter_radius_
       << ",\"green_identity_radius\":" << logic3_runtime_green_match_radius_
       << ",\"green_anchor_update_radius\":" <<
    logic3_runtime_green_anchor_update_radius_
       << ",\"green_update_min_distance\":" <<
    logic3_runtime_green_update_min_distance_
       << ",\"red_trigger_index\":" <<
    (task1_before_red_event ? logic3_task1_before_red_snapshot_count_ :
    (final_before_red_event ? logic3_final_before_red_snapshot_count_ : 0))
       << ",\"orange_green_begin_index\":" << orange_green_begin_index
       << ",\"base_zone\":{\"min_x\":" << return_base_min_x_
       << ",\"max_x\":" << return_base_max_x_
       << ",\"min_y\":" << return_base_min_y_
       << ",\"max_y\":" << return_base_max_y_ << '}'
       << ",\"parking_zone\":{\"min_x\":" << return_stop_zone_min_x_
       << ",\"top_min_x\":" << return_stop_zone_top_min_x_
       << ",\"top_max_x\":" << return_stop_zone_top_max_x_
       << ",\"max_x\":" << return_stop_zone_max_x_
       << ",\"min_y\":" << return_stop_zone_min_y_
       << ",\"right_top_y\":" << return_stop_zone_right_top_y_
       << ",\"max_y\":" << return_stop_zone_max_y_
       << ",\"left_curve_control_x\":" <<
    return_stop_zone_left_curve_control_x_
       << ",\"left_curve_control_y\":" <<
    return_stop_zone_left_curve_control_y_
       << ",\"confirm_seconds\":" << return_stop_confirm_seconds_ << '}'
       << ",\"task3_slowdown_zone\":{\"enabled\":"
       << (task3_return_slowdown_enabled_ ? "true" : "false")
       << ",\"center_x\":" << task3_terminal_target_x_
       << ",\"center_y\":" << task3_terminal_target_y_
       << ",\"radius\":" << task3_return_slowdown_radius_
       << ",\"start_speed\":" << task3_profile_.linear_speed
       << ",\"min_speed\":" << task3_return_slowdown_min_speed_ << '}'
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
         << final_frozen_marker_.y << "],\"yaw\":"
         << final_frozen_marker_yaw_ << ",\"heading_distance\":"
         << channel_entry_heading_reference_distance_ << ",\"label\":\""
         << json_escape(final_frozen_marker_label_) << "\"}";
  } else {
    json << "null";
  }
  json << ",\"task3_terminal_target_marker\":";
  if (task3_terminal_homing_used_) {
    json << "{\"point\":[" << task3_terminal_target_x_ << ','
         << task3_terminal_target_y_
         << "],\"label\":\"TASK3 FALLBACK\"}";
  } else {
    json << "null";
  }
  json << ",\"image_capture_corridor\":";
  if (image_capture_event) {
    const std::string direction = to_lower(selected_route_direction_);
    const bool clockwise = direction == "clockwise";
    const bool counterclockwise = direction == "counterclockwise";
    if (clockwise || counterclockwise) {
      const double enter_x = clockwise ?
        image_capture_clockwise_enter_x_ : image_capture_counterclockwise_enter_x_;
      const double exit_x = clockwise ?
        image_capture_clockwise_exit_x_ : image_capture_counterclockwise_exit_x_;
      json << "{\"enabled\":true,\"direction\":\""
           << json_escape(direction)
           << "\",\"min_x\":" << std::min(enter_x, exit_x)
           << ",\"max_x\":" << std::max(enter_x, exit_x)
           << ",\"min_y\":" << image_capture_corridor_min_y_
           << ",\"max_y\":" << image_capture_corridor_max_y_
           << ",\"speed\":" << image_capture_corridor_speed_ << '}';
    } else {
      json << "null";
    }
  } else {
    json << "null";
  }
  json << ",\"image_capture_stop_marker\":";
  if (image_capture_event && image_capture_stop_point_valid_) {
    json << "{\"point\":[" << image_capture_stop_point_.x << ','
         << image_capture_stop_point_.y << "],\"yaw\":"
         << image_capture_stop_point_yaw_ << ",\"label\":\"STOP\"}";
  } else {
    json << "null";
  }
  json << ",\"image_capture_marker\":";
  if (image_capture_event && image_capture_point_valid_) {
    json << "{\"point\":[" << image_capture_point_.x << ','
         << image_capture_point_.y << "],\"yaw\":"
         << image_capture_point_yaw_ << ",\"label\":\"CAPTURE\"}";
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
  json << ",\"actual_route_enabled\":"
       << (logic3_actual_route_enabled_ ? "true" : "false")
       << ",\"actual_route_line_width\":" << logic3_actual_route_line_width_
       << ",\"actual_route_color\":\""
       << json_escape(logic3_actual_route_color_) << "\""
       << ",\"actual_route\":";
  append_points(
    json,
    (event == "final_dynamic") ? logic3_actual_route_ : std::vector<Point2D>());
  json << ",\"green\":";
  append_clusters(json, logic3_green_clusters_);
  json << ",\"red\":";
  append_points(json, dynamic_event ? plotted_red_clusters : std::vector<Point2D>());
  json << '}';

  std_msgs::msg::String message;
  message.data = json.str();
  logic3_snapshot_pub_->publish(message);
}
