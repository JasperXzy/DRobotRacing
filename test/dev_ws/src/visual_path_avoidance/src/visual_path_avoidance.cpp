#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "ai_msgs/msg/perception_targets.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

namespace visual_path_avoidance
{

class VisualPathAvoidance : public rclcpp::Node
{
public:
  VisualPathAvoidance()
  : Node("visual_path_avoidance")
  {
    declare_parameter<std::string>("path_cmd_vel_topic", path_cmd_vel_topic_);
    declare_parameter<std::string>("output_cmd_vel_topic", output_cmd_vel_topic_);
    declare_parameter<std::string>("detection_topic", detection_topic_);
    declare_parameter<std::string>("handoff_topic", handoff_topic_);
    declare_parameter<int>("activate_value", activate_value_);
    declare_parameter<int>("deactivate_value", deactivate_value_);
    declare_parameter<bool>("active_on_start", active_on_start_);
    declare_parameter<double>("control_frequency", control_frequency_);
    declare_parameter<double>("path_cmd_timeout", path_cmd_timeout_);
    declare_parameter<double>("detection_timeout", detection_timeout_);
    declare_parameter<double>("obstacle_clear_hold_time", obstacle_clear_hold_time_);
    declare_parameter<double>("path_direction_deadband", path_direction_deadband_);
    declare_parameter<double>("avoid_linear_speed", avoid_linear_speed_);
    declare_parameter<double>("avoid_angular_ratio", avoid_angular_ratio_);
    declare_parameter<double>("obstacle_confidence_threshold", obstacle_confidence_threshold_);
    declare_parameter<int>("bottom_threshold_caution", bottom_threshold_caution_);
    declare_parameter<int>("bottom_threshold_avoid", bottom_threshold_avoid_);
    declare_parameter<double>("delta_x_ratio", delta_x_ratio_);
    declare_parameter<int>("detection_source_width", detection_source_width_);
    declare_parameter<int>("detection_source_height", detection_source_height_);
    declare_parameter<int>("control_image_width", control_image_width_);
    declare_parameter<int>("control_image_height", control_image_height_);
    declare_parameter<bool>("debug_control_logs", debug_control_logs_);

    get_parameter("path_cmd_vel_topic", path_cmd_vel_topic_);
    get_parameter("output_cmd_vel_topic", output_cmd_vel_topic_);
    get_parameter("detection_topic", detection_topic_);
    get_parameter("handoff_topic", handoff_topic_);
    get_parameter("activate_value", activate_value_);
    get_parameter("deactivate_value", deactivate_value_);
    get_parameter("active_on_start", active_on_start_);
    get_parameter("control_frequency", control_frequency_);
    get_parameter("path_cmd_timeout", path_cmd_timeout_);
    get_parameter("detection_timeout", detection_timeout_);
    get_parameter("obstacle_clear_hold_time", obstacle_clear_hold_time_);
    get_parameter("path_direction_deadband", path_direction_deadband_);
    get_parameter("avoid_linear_speed", avoid_linear_speed_);
    get_parameter("avoid_angular_ratio", avoid_angular_ratio_);
    get_parameter("obstacle_confidence_threshold", obstacle_confidence_threshold_);
    get_parameter("bottom_threshold_caution", bottom_threshold_caution_);
    get_parameter("bottom_threshold_avoid", bottom_threshold_avoid_);
    get_parameter("delta_x_ratio", delta_x_ratio_);
    get_parameter("detection_source_width", detection_source_width_);
    get_parameter("detection_source_height", detection_source_height_);
    get_parameter("control_image_width", control_image_width_);
    get_parameter("control_image_height", control_image_height_);
    get_parameter("debug_control_logs", debug_control_logs_);

    control_frequency_ = std::max(1.0, control_frequency_);
    path_cmd_timeout_ = std::max(0.05, path_cmd_timeout_);
    detection_timeout_ = std::max(0.05, detection_timeout_);
    obstacle_clear_hold_time_ = std::max(0.0, obstacle_clear_hold_time_);
    path_direction_deadband_ = std::max(0.0, path_direction_deadband_);
    avoid_linear_speed_ = std::max(0.0, avoid_linear_speed_);
    obstacle_confidence_threshold_ = std::max(0.0, obstacle_confidence_threshold_);
    bottom_threshold_caution_ = std::max(0, bottom_threshold_caution_);
    bottom_threshold_avoid_ = std::max(bottom_threshold_caution_, bottom_threshold_avoid_);
    detection_source_width_ = std::max(1, detection_source_width_);
    detection_source_height_ = std::max(1, detection_source_height_);
    control_image_width_ = std::max(1, control_image_width_);
    control_image_height_ = std::max(1, control_image_height_);

    output_pub_ = create_publisher<geometry_msgs::msg::Twist>(output_cmd_vel_topic_, 10);
    path_cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      path_cmd_vel_topic_, 10,
      std::bind(&VisualPathAvoidance::path_cmd_callback, this, std::placeholders::_1));
    detection_sub_ = create_subscription<ai_msgs::msg::PerceptionTargets>(
      detection_topic_, rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&VisualPathAvoidance::detection_callback, this, std::placeholders::_1));
    handoff_sub_ = create_subscription<std_msgs::msg::Int32>(
      handoff_topic_, rclcpp::QoS(3).reliable(),
      std::bind(&VisualPathAvoidance::handoff_callback, this, std::placeholders::_1));

    active_ = active_on_start_;
    const auto period = std::chrono::duration<double>(1.0 / control_frequency_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&VisualPathAvoidance::control_once, this));

    RCLCPP_INFO(
      get_logger(),
      "视觉追点避障已启动：输入=%s，输出=%s，检测=%s，初始状态=%s。",
      path_cmd_vel_topic_.c_str(), output_cmd_vel_topic_.c_str(), detection_topic_.c_str(),
      active_ ? "启用" : "等待追点接管");
  }

private:
  using SteadyClock = std::chrono::steady_clock;

  struct ObstacleObservation
  {
    bool caution{false};
    bool avoid{false};
    int center_x{0};
    int bottom_y{0};
    float confidence{0.0F};
  };

  std::string path_cmd_vel_topic_{"/visual_path_cmd_vel"};
  std::string output_cmd_vel_topic_{"/cmd_vel"};
  std::string detection_topic_{"/racing_obstacle_detection"};
  std::string handoff_topic_{"/sign4return"};
  int activate_value_{5};
  int deactivate_value_{6};
  bool active_on_start_{false};
  double control_frequency_{30.0};
  double path_cmd_timeout_{0.25};
  double detection_timeout_{0.30};
  double obstacle_clear_hold_time_{0.15};
  double path_direction_deadband_{0.05};
  double avoid_linear_speed_{0.5};
  double avoid_angular_ratio_{-0.005};
  double obstacle_confidence_threshold_{0.5};
  int bottom_threshold_caution_{295};
  int bottom_threshold_avoid_{305};
  double delta_x_ratio_{0.0};
  int detection_source_width_{1920};
  int detection_source_height_{1080};
  int control_image_width_{640};
  int control_image_height_{480};
  bool debug_control_logs_{true};

  bool active_{false};
  bool have_path_cmd_{false};
  bool have_detection_{false};
  bool avoiding_{false};
  bool clearing_obstacle_{false};
  int avoid_direction_{0};
  double last_avoid_angular_speed_{0.0};
  geometry_msgs::msg::Twist latest_path_cmd_;
  ai_msgs::msg::PerceptionTargets::ConstSharedPtr latest_detection_;
  SteadyClock::time_point last_path_cmd_time_{};
  SteadyClock::time_point last_detection_time_{};
  SteadyClock::time_point obstacle_clear_since_{};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr output_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr path_cmd_sub_;
  rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr detection_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr handoff_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void path_cmd_callback(const geometry_msgs::msg::Twist::ConstSharedPtr msg)
  {
    latest_path_cmd_ = *msg;
    last_path_cmd_time_ = SteadyClock::now();
    have_path_cmd_ = true;
  }

  void detection_callback(const ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg)
  {
    latest_detection_ = msg;
    last_detection_time_ = SteadyClock::now();
    have_detection_ = true;
  }

  void handoff_callback(const std_msgs::msg::Int32::ConstSharedPtr msg)
  {
    if (msg->data == activate_value_) {
      if (!active_) {
        RCLCPP_INFO(get_logger(), "追点阶段开始，启用视觉避障过滤器。");
      }
      active_ = true;
      have_path_cmd_ = false;
      reset_avoidance();
      output_pub_->publish(geometry_msgs::msg::Twist());
      return;
    }

    if (msg->data == deactivate_value_) {
      if (active_) {
        RCLCPP_INFO(get_logger(), "追点阶段结束，视觉避障过滤器停止发布速度。");
      }
      active_ = false;
      have_path_cmd_ = false;
      reset_avoidance();
    }
  }

  bool is_fresh(
    const SteadyClock::time_point & stamp, double timeout,
    const SteadyClock::time_point & now) const
  {
    return std::chrono::duration<double>(now - stamp).count() <= timeout;
  }

  int scale_x(int x) const
  {
    return static_cast<int>(std::round(
             static_cast<double>(x) * static_cast<double>(control_image_width_) /
             static_cast<double>(detection_source_width_)));
  }

  int scale_y(int y) const
  {
    return static_cast<int>(std::round(
             static_cast<double>(y) * static_cast<double>(control_image_height_) /
             static_cast<double>(detection_source_height_)));
  }

  ObstacleObservation observe_obstacle() const
  {
    ObstacleObservation result;
    if (!latest_detection_) {
      return result;
    }

    for (const auto & target : latest_detection_->targets) {
      if (target.type != "construction_cone" || target.rois.empty()) {
        continue;
      }
      const auto & roi = target.rois.front();
      if (roi.confidence < obstacle_confidence_threshold_ || roi.rect.width == 0U) {
        continue;
      }

      const int center_x = scale_x(static_cast<int>(roi.rect.x_offset + roi.rect.width / 2U));
      const int bottom_y = scale_y(static_cast<int>(roi.rect.y_offset + roi.rect.height));
      const int delta_x = std::abs(center_x - control_image_width_ / 2);
      const double offset = static_cast<double>(delta_x) * delta_x_ratio_;
      const bool caution = static_cast<double>(bottom_y) >=
        static_cast<double>(bottom_threshold_caution_) + offset;
      const bool avoid = static_cast<double>(bottom_y) >=
        static_cast<double>(bottom_threshold_avoid_) + offset;

      result.caution = result.caution || caution;
      if (avoid && (!result.avoid || bottom_y > result.bottom_y)) {
        result.avoid = true;
        result.center_x = center_x;
        result.bottom_y = bottom_y;
        result.confidence = roi.confidence;
      }
    }
    return result;
  }

  int choose_avoid_direction(
    const geometry_msgs::msg::Twist & path_cmd,
    int obstacle_center_x) const
  {
    if (path_cmd.angular.z > path_direction_deadband_) {
      return 1;
    }
    if (path_cmd.angular.z < -path_direction_deadband_) {
      return -1;
    }

    // 目标基本在正前方时，才退回到远离障碍物中心的方向。
    return obstacle_center_x < control_image_width_ / 2 ? -1 : 1;
  }

  double compute_avoid_angular_speed(int obstacle_center_x) const
  {
    const int clamped_x = std::max(0, std::min(control_image_width_, obstacle_center_x));
    const int edge_distance = std::min(clamped_x, control_image_width_ - clamped_x);
    return std::abs(avoid_angular_ratio_) * static_cast<double>(edge_distance);
  }

  void reset_avoidance()
  {
    avoiding_ = false;
    clearing_obstacle_ = false;
    avoid_direction_ = 0;
    last_avoid_angular_speed_ = 0.0;
  }

  void publish_avoid_command()
  {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = avoid_linear_speed_;
    cmd.angular.z = static_cast<double>(avoid_direction_) * last_avoid_angular_speed_;
    output_pub_->publish(cmd);
  }

  void control_once()
  {
    if (!active_) {
      return;
    }

    const auto now = SteadyClock::now();
    if (!have_path_cmd_ || !is_fresh(last_path_cmd_time_, path_cmd_timeout_, now)) {
      output_pub_->publish(geometry_msgs::msg::Twist());
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "等待新鲜的追点速度，保持停车。");
      return;
    }
    if (!have_detection_ || !is_fresh(last_detection_time_, detection_timeout_, now)) {
      output_pub_->publish(geometry_msgs::msg::Twist());
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "视觉检测超时，保持停车。");
      return;
    }

    // 前视相机无法保护倒车方向，倒车点维持原追点命令。
    if (latest_path_cmd_.linear.x < 0.0) {
      reset_avoidance();
      output_pub_->publish(latest_path_cmd_);
      return;
    }

    const auto obstacle = observe_obstacle();
    if (obstacle.avoid) {
      if (!avoiding_) {
        avoid_direction_ = choose_avoid_direction(latest_path_cmd_, obstacle.center_x);
        avoiding_ = true;
        RCLCPP_WARN(
          get_logger(), "视觉障碍物过近，沿追点方向向%s避障。",
          avoid_direction_ > 0 ? "左" : "右");
      }
      clearing_obstacle_ = false;
      last_avoid_angular_speed_ = compute_avoid_angular_speed(obstacle.center_x);
      publish_avoid_command();
      if (debug_control_logs_) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 500,
          "debug: 避障 x=%d, bottom=%d, confidence=%.2f, linear=%.2f, angular=%.2f。",
          obstacle.center_x, obstacle.bottom_y, obstacle.confidence, avoid_linear_speed_,
          static_cast<double>(avoid_direction_) * last_avoid_angular_speed_);
      }
      return;
    }

    if (avoiding_) {
      if (!clearing_obstacle_) {
        clearing_obstacle_ = true;
        obstacle_clear_since_ = now;
      }
      if (std::chrono::duration<double>(now - obstacle_clear_since_).count() <
        obstacle_clear_hold_time_)
      {
        publish_avoid_command();
        return;
      }
      RCLCPP_WARN(get_logger(), "视觉障碍物已清除，直接恢复追点。");
      reset_avoidance();
    }

    auto cmd = latest_path_cmd_;
    if (obstacle.caution && cmd.linear.x > avoid_linear_speed_) {
      cmd.linear.x = avoid_linear_speed_;
    }
    output_pub_->publish(cmd);
  }
};

}  // namespace visual_path_avoidance

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<visual_path_avoidance::VisualPathAvoidance>());
  rclcpp::shutdown();
  return 0;
}
