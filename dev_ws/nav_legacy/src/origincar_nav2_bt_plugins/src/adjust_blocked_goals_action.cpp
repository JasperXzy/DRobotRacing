#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

namespace origincar_nav2_bt_plugins
{

class AdjustBlockedGoals : public BT::SyncActionNode
{
public:
  using Goals = std::vector<geometry_msgs::msg::PoseStamped>;

  AdjustBlockedGoals(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf)
  : BT::SyncActionNode(xml_tag_name, conf)
  {
    node_ = config().blackboard->template get<rclcpp::Node::SharedPtr>("node");
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<Goals>("input_goals", "Original goals to adjust"),
      BT::OutputPort<Goals>("output_goals", "Goals moved out of blocked costmap cells"),
      BT::InputPort<std::string>(
        "costmap_topic", std::string("/global_costmap/costmap"),
        "OccupancyGrid costmap topic used to check goals"),
      BT::InputPort<int>(
        "blocked_cost", 50, "Cost at or above this value is treated as blocked"),
      BT::InputPort<double>(
        "search_radius", 0.45, "Maximum distance in meters to search around a blocked goal"),
      BT::InputPort<double>(
        "clearance_radius", 0.18, "Required free radius around an adjusted goal"),
      BT::InputPort<bool>(
        "unknown_is_blocked", true, "Treat unknown costmap cells as blocked")
    };
  }

private:
  BT::NodeStatus tick() override
  {
    Goals goals;
    const auto goals_result = getInput("input_goals", goals);
    if (!goals_result) {
      RCLCPP_ERROR(
        node_->get_logger(), "AdjustBlockedGoals: failed to read input_goals: %s",
        goals_result.error().c_str());
      return BT::NodeStatus::FAILURE;
    }

    if (goals.empty()) {
      setOutput("output_goals", goals);
      return BT::NodeStatus::SUCCESS;
    }

    std::string costmap_topic;
    int blocked_cost = 50;
    double search_radius = 0.45;
    double clearance_radius = 0.18;
    bool unknown_is_blocked = true;

    getInput("costmap_topic", costmap_topic);
    getInput("blocked_cost", blocked_cost);
    getInput("search_radius", search_radius);
    getInput("clearance_radius", clearance_radius);
    getInput("unknown_is_blocked", unknown_is_blocked);

    if (costmap_topic.empty()) {
      costmap_topic = "/global_costmap/costmap";
    }

    blocked_cost = std::clamp(blocked_cost, 0, 100);
    search_radius = std::max(0.0, search_radius);
    clearance_radius = std::max(0.0, clearance_radius);

    ensureCostmapSubscription(costmap_topic);

    nav_msgs::msg::OccupancyGrid::SharedPtr costmap;
    {
      std::lock_guard<std::mutex> lock(costmap_mutex_);
      costmap = latest_costmap_;
    }

    if (!costmap) {
      if (!no_costmap_warned_) {
        no_costmap_warned_ = true;
        RCLCPP_WARN(
          node_->get_logger(),
          "AdjustBlockedGoals: no costmap received from %s yet, leaving goals unchanged",
          costmap_topic.c_str());
      }
      setOutput("output_goals", goals);
      return BT::NodeStatus::SUCCESS;
    }

    Goals adjusted_goals = goals;
    bool adjusted_any = false;
    bool unresolved_blocked_goal = false;

    for (size_t i = 0; i < goals.size(); ++i) {
      const auto & pose = goals[i].pose.position;
      if (!isPoseBlocked(
          pose.x, pose.y, *costmap, blocked_cost, clearance_radius, unknown_is_blocked))
      {
        continue;
      }

      geometry_msgs::msg::Point replacement;
      if (!findReplacementPose(
          goals, i, *costmap, blocked_cost, search_radius, clearance_radius,
          unknown_is_blocked, replacement))
      {
        unresolved_blocked_goal = true;
        RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 2000,
          "AdjustBlockedGoals: goal #%zu (%.2f, %.2f) is blocked, "
          "but no free cell was found within %.2fm",
          i + 1, pose.x, pose.y, search_radius);
        continue;
      }

      adjusted_goals[i].pose.position.x = replacement.x;
      adjusted_goals[i].pose.position.y = replacement.y;
      adjusted_any = true;

      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "AdjustBlockedGoals: goal #%zu moved from (%.2f, %.2f) to (%.2f, %.2f)",
        i + 1, pose.x, pose.y, replacement.x, replacement.y);
    }

    setOutput("output_goals", adjusted_goals);

    if (unresolved_blocked_goal) {
      return BT::NodeStatus::FAILURE;
    }

    if (adjusted_any) {
      RCLCPP_DEBUG(node_->get_logger(), "AdjustBlockedGoals: adjusted blocked route goals");
    }
    return BT::NodeStatus::SUCCESS;
  }

  void ensureCostmapSubscription(const std::string & topic)
  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    if (costmap_sub_ && topic == costmap_topic_) {
      return;
    }

    costmap_topic_ = topic;
    latest_costmap_.reset();
    no_costmap_warned_ = false;

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    costmap_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
      costmap_topic_, qos,
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        std::lock_guard<std::mutex> callback_lock(costmap_mutex_);
        latest_costmap_ = msg;
      });

    RCLCPP_INFO(
      node_->get_logger(), "AdjustBlockedGoals: subscribed to %s", costmap_topic_.c_str());
  }

  bool findReplacementPose(
    const Goals & goals,
    size_t goal_index,
    const nav_msgs::msg::OccupancyGrid & costmap,
    int blocked_cost,
    double search_radius,
    double clearance_radius,
    bool unknown_is_blocked,
    geometry_msgs::msg::Point & replacement) const
  {
    const auto & original = goals[goal_index].pose.position;
    int center_mx = 0;
    int center_my = 0;
    if (!worldToMap(costmap, original.x, original.y, center_mx, center_my)) {
      return false;
    }

    const auto & info = costmap.info;
    const int max_cells = static_cast<int>(std::ceil(search_radius / info.resolution));
    double best_score = std::numeric_limits<double>::infinity();
    bool found = false;

    for (int dx = -max_cells; dx <= max_cells; ++dx) {
      for (int dy = -max_cells; dy <= max_cells; ++dy) {
        const double offset_dist = std::hypot(dx, dy) * info.resolution;
        if (offset_dist > search_radius) {
          continue;
        }

        const int mx = center_mx + dx;
        const int my = center_my + dy;
        if (!cellHasClearance(
            costmap, mx, my, blocked_cost, clearance_radius, unknown_is_blocked))
        {
          continue;
        }

        geometry_msgs::msg::Point candidate = mapToWorld(costmap, mx, my);
        const double route_penalty = distanceToRouteContext(goals, goal_index, candidate);
        const double score = offset_dist + 0.35 * route_penalty;
        if (score < best_score) {
          best_score = score;
          replacement = candidate;
          found = true;
        }
      }
    }

    return found;
  }

  bool isPoseBlocked(
    double x,
    double y,
    const nav_msgs::msg::OccupancyGrid & costmap,
    int blocked_cost,
    double clearance_radius,
    bool unknown_is_blocked) const
  {
    int mx = 0;
    int my = 0;
    if (!worldToMap(costmap, x, y, mx, my)) {
      return true;
    }

    return !cellHasClearance(
      costmap, mx, my, blocked_cost, clearance_radius, unknown_is_blocked);
  }

  bool cellHasClearance(
    const nav_msgs::msg::OccupancyGrid & costmap,
    int center_mx,
    int center_my,
    int blocked_cost,
    double clearance_radius,
    bool unknown_is_blocked) const
  {
    const auto & info = costmap.info;
    if (!isInside(costmap, center_mx, center_my) || info.resolution <= 0.0) {
      return false;
    }

    const int radius_cells = static_cast<int>(std::ceil(clearance_radius / info.resolution));
    for (int mx = center_mx - radius_cells; mx <= center_mx + radius_cells; ++mx) {
      for (int my = center_my - radius_cells; my <= center_my + radius_cells; ++my) {
        if (!isInside(costmap, mx, my)) {
          return false;
        }

        const double dist = std::hypot(mx - center_mx, my - center_my) * info.resolution;
        if (dist > clearance_radius) {
          continue;
        }

        const int cost = static_cast<int>(costmap.data[my * info.width + mx]);
        if (cost < 0) {
          if (unknown_is_blocked) {
            return false;
          }
          continue;
        }

        if (cost >= blocked_cost) {
          return false;
        }
      }
    }

    return true;
  }

  bool worldToMap(
    const nav_msgs::msg::OccupancyGrid & costmap,
    double wx,
    double wy,
    int & mx,
    int & my) const
  {
    const auto & info = costmap.info;
    if (info.resolution <= 0.0 || info.width == 0 || info.height == 0) {
      return false;
    }

    const double yaw = quaternionYaw(info.origin.orientation);
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);
    const double dx = wx - info.origin.position.x;
    const double dy = wy - info.origin.position.y;
    const double local_x = cos_yaw * dx + sin_yaw * dy;
    const double local_y = -sin_yaw * dx + cos_yaw * dy;

    mx = static_cast<int>(std::floor(local_x / info.resolution));
    my = static_cast<int>(std::floor(local_y / info.resolution));
    return isInside(costmap, mx, my);
  }

  geometry_msgs::msg::Point mapToWorld(
    const nav_msgs::msg::OccupancyGrid & costmap,
    int mx,
    int my) const
  {
    const auto & info = costmap.info;
    const double yaw = quaternionYaw(info.origin.orientation);
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);
    const double local_x = (static_cast<double>(mx) + 0.5) * info.resolution;
    const double local_y = (static_cast<double>(my) + 0.5) * info.resolution;

    geometry_msgs::msg::Point point;
    point.x = info.origin.position.x + cos_yaw * local_x - sin_yaw * local_y;
    point.y = info.origin.position.y + sin_yaw * local_x + cos_yaw * local_y;
    point.z = 0.0;
    return point;
  }

  bool isInside(
    const nav_msgs::msg::OccupancyGrid & costmap,
    int mx,
    int my) const
  {
    const auto & info = costmap.info;
    return mx >= 0 && my >= 0 &&
           mx < static_cast<int>(info.width) &&
           my < static_cast<int>(info.height);
  }

  double distanceToRouteContext(
    const Goals & goals,
    size_t goal_index,
    const geometry_msgs::msg::Point & candidate) const
  {
    if (goals.size() < 2) {
      return 0.0;
    }

    if (goal_index > 0 && goal_index + 1 < goals.size()) {
      return distanceToSegment(
        candidate,
        goals[goal_index - 1].pose.position,
        goals[goal_index + 1].pose.position);
    }

    if (goal_index > 0) {
      return distance2D(candidate, goals[goal_index - 1].pose.position);
    }

    return distance2D(candidate, goals[goal_index + 1].pose.position);
  }

  double distanceToSegment(
    const geometry_msgs::msg::Point & point,
    const geometry_msgs::msg::Point & segment_start,
    const geometry_msgs::msg::Point & segment_end) const
  {
    const double vx = segment_end.x - segment_start.x;
    const double vy = segment_end.y - segment_start.y;
    const double wx = point.x - segment_start.x;
    const double wy = point.y - segment_start.y;
    const double len_sq = vx * vx + vy * vy;
    if (len_sq <= std::numeric_limits<double>::epsilon()) {
      return distance2D(point, segment_start);
    }

    const double t = std::clamp((wx * vx + wy * vy) / len_sq, 0.0, 1.0);
    geometry_msgs::msg::Point projection;
    projection.x = segment_start.x + t * vx;
    projection.y = segment_start.y + t * vy;
    return distance2D(point, projection);
  }

  double distance2D(
    const geometry_msgs::msg::Point & a,
    const geometry_msgs::msg::Point & b) const
  {
    return std::hypot(a.x - b.x, a.y - b.y);
  }

  double quaternionYaw(const geometry_msgs::msg::Quaternion & q) const
  {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap_;
  std::string costmap_topic_;
  mutable std::mutex costmap_mutex_;
  bool no_costmap_warned_{false};
};

}  // namespace origincar_nav2_bt_plugins

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<origincar_nav2_bt_plugins::AdjustBlockedGoals>("AdjustBlockedGoals");
}
