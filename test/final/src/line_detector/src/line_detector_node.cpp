#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include <ai_msgs/msg/perception_targets.hpp>
#include <ai_msgs/msg/point.hpp>
#include <ai_msgs/msg/target.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <hbm_img_msgs/msg/hbm_msg1080_p.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

#include "line_detector/line_detector.hpp"

namespace line_detector
{

namespace
{

builtin_interfaces::msg::Time timeMsgFromRclTime(const rclcpp::Time & time)
{
  const int64_t nanoseconds = time.nanoseconds();
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(nanoseconds / 1000000000LL);
  stamp.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000LL);
  return stamp;
}

}  // namespace

class LineDetectorNode : public rclcpp::Node
{
public:
  LineDetectorNode()
  : Node("GetLineCoordinate")
  {
    declareParameters();
    loadParameters();

    std::string error;
    if (!detector_.init(detector_config_, &error)) {
      throw std::runtime_error("failed to initialize line detector: " + error);
    }

    auto sensor_qos = rclcpp::SensorDataQoS();
    sensor_qos.keep_last(static_cast<std::size_t>(qos_keep_last_));
    sensor_qos.best_effort();

    publisher_ = create_publisher<ai_msgs::msg::PerceptionTargets>(output_topic_, sensor_qos);
    image_sub_ = create_subscription<hbm_img_msgs::msg::HbmMsg1080P>(
      input_topic_,
      sensor_qos,
      std::bind(&LineDetectorNode::imageCallback, this, std::placeholders::_1));
    sign_sub_ = create_subscription<std_msgs::msg::Int32>(
      "/sign4return",
      sensor_qos,
      std::bind(&LineDetectorNode::signCallback, this, std::placeholders::_1));

    fps_window_start_ = now();
    RCLCPP_INFO(
      get_logger(),
      "line_detector started: input=%s output=%s model=%s crop=%dx%d+%d+%d logical=%dx%d",
      input_topic_.c_str(),
      output_topic_.c_str(),
      detector_config_.model_path.c_str(),
      detector_config_.crop_width,
      detector_config_.crop_height,
      detector_config_.crop_left,
      detector_config_.crop_top,
      detector_config_.logical_width,
      detector_config_.logical_height);
  }

private:
  void declareParameters()
  {
    declare_parameter<std::string>("input_topic", "/camera/image_hbmem");
    declare_parameter<std::string>("output_topic", "racing_track_center_detection");
    declare_parameter<std::string>("camera_frame_id", "usb_cam");
    declare_parameter<std::string>(
      "model_path",
      "/root/D_Robot/test/dev_ws/models/line_detector_resnet18_nashe_224x224_nv12.hbm");

    declare_parameter<int>("source_width", 1920);
    declare_parameter<int>("source_height", 1080);
    declare_parameter<int>("model_input_width", 224);
    declare_parameter<int>("model_input_height", 224);
    declare_parameter<int>("logical_width", 640);
    declare_parameter<int>("logical_height", 480);
    declare_parameter<int>("crop_left", 0);
    declare_parameter<int>("crop_top", 256);
    declare_parameter<int>("crop_width", 640);
    declare_parameter<int>("crop_height", 224);
    declare_parameter<double>("output_x_offset", 0.0);
    declare_parameter<double>("output_y_offset", 256.0);
    declare_parameter<double>("confidence_threshold", 0.0);
    declare_parameter<int>("qos_keep_last", 1);
    declare_parameter<bool>("log_timing", true);
    declare_parameter<bool>("log_fps", true);
  }

  void loadParameters()
  {
    input_topic_ = get_parameter("input_topic").as_string();
    output_topic_ = get_parameter("output_topic").as_string();
    camera_frame_id_ = get_parameter("camera_frame_id").as_string();

    source_width_ = static_cast<int>(get_parameter("source_width").as_int());
    source_height_ = static_cast<int>(get_parameter("source_height").as_int());

    detector_config_.model_path = get_parameter("model_path").as_string();
    detector_config_.model_input_width = static_cast<int>(get_parameter("model_input_width").as_int());
    detector_config_.model_input_height = static_cast<int>(get_parameter("model_input_height").as_int());
    detector_config_.logical_width = static_cast<int>(get_parameter("logical_width").as_int());
    detector_config_.logical_height = static_cast<int>(get_parameter("logical_height").as_int());
    detector_config_.crop_left = static_cast<int>(get_parameter("crop_left").as_int());
    detector_config_.crop_top = static_cast<int>(get_parameter("crop_top").as_int());
    detector_config_.crop_width = static_cast<int>(get_parameter("crop_width").as_int());
    detector_config_.crop_height = static_cast<int>(get_parameter("crop_height").as_int());

    output_x_offset_ = static_cast<float>(get_parameter("output_x_offset").as_double());
    output_y_offset_ = static_cast<float>(get_parameter("output_y_offset").as_double());
    confidence_threshold_ = static_cast<float>(get_parameter("confidence_threshold").as_double());
    qos_keep_last_ = std::max(1, static_cast<int>(get_parameter("qos_keep_last").as_int()));
    log_timing_ = get_parameter("log_timing").as_bool();
    log_fps_ = get_parameter("log_fps").as_bool();
  }

  void signCallback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    const int sign_value = msg->data;
    if (sign_value == 5) {
      enable_lane_following_ = false;
      RCLCPP_INFO(get_logger(), "Lane following disabled by /sign4return=5");
    } else if (sign_value == 6) {
      enable_lane_following_ = true;
      RCLCPP_INFO(get_logger(), "Lane following enabled by /sign4return=6");
    }
  }

  void imageCallback(const hbm_img_msgs::msg::HbmMsg1080P::SharedPtr msg)
  {
    if (!msg || !rclcpp::ok() || !enable_lane_following_) {
      return;
    }

    const int src_width = source_width_ > 0 ? source_width_ : static_cast<int>(msg->width);
    const int src_height = source_height_ > 0 ? source_height_ : static_cast<int>(msg->height);
    if (msg->step > 0U && msg->step != static_cast<uint32_t>(src_width)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        3000,
        "drop frame: unsupported NV12 row step=%u for source width=%d",
        msg->step,
        src_width);
      return;
    }

    const auto expected_size = expectedNv12Size(src_width, src_height);
    const auto available_size = msg->data_size > 0U ?
      static_cast<std::size_t>(msg->data_size) : msg->data.size();
    if (expected_size == 0U || available_size < expected_size) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        3000,
        "drop frame: invalid NV12 size, source=%dx%d expected=%zu available=%zu",
        src_width,
        src_height,
        expected_size,
        available_size);
      return;
    }

    builtin_interfaces::msg::Time stamp = msg->time_stamp;
    if (stamp.sec == 0 && stamp.nanosec == 0U) {
      stamp = timeMsgFromRclTime(now());
    }

    LineDetectorResult result;
    LineDetectorTiming timing;
    std::string error;
    if (!detector_.infer(
        msg->data.data(),
        src_width,
        src_height,
        result,
        &timing,
        &error))
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(),
        *get_clock(),
        3000,
        "line detector inference failed: %s",
        error.c_str());
      return;
    }

    ++processed_frames_;
    if (result.confidence >= confidence_threshold_) {
      publishLine(result, stamp);
      ++published_frames_;
    }

    if (log_timing_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "line result: x=%.1f y=%.1f conf=%.3f; timing ms: preprocess=%.2f infer=%.2f postprocess=%.2f total=%.2f",
        result.crop_x + output_x_offset_,
        result.crop_y + output_y_offset_,
        result.confidence,
        timing.preprocess_ms,
        timing.infer_ms,
        timing.postprocess_ms,
        timing.total_ms);
    }
    maybeLogFps();
  }

  void publishLine(
    const LineDetectorResult & result,
    const builtin_interfaces::msg::Time & stamp)
  {
    ai_msgs::msg::PerceptionTargets msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = camera_frame_id_;
    msg.fps = last_output_fps_;

    ai_msgs::msg::Target target;
    target.type = "track_center";
    target.track_id = 0;

    ai_msgs::msg::Point track_center_point;
    track_center_point.type = "midline_point";

    geometry_msgs::msg::Point32 point;
    point.x = result.crop_x + output_x_offset_;
    point.y = result.crop_y + output_y_offset_;
    point.z = 0.0F;

    track_center_point.point.push_back(point);
    track_center_point.confidence.push_back(result.confidence);
    target.points.push_back(std::move(track_center_point));
    msg.targets.push_back(std::move(target));

    publisher_->publish(std::move(msg));
  }

  void maybeLogFps()
  {
    if (!log_fps_) {
      return;
    }

    const auto current_time = now();
    const double elapsed = (current_time - fps_window_start_).seconds();
    if (elapsed < 1.0) {
      return;
    }

    const double processed_fps = processed_frames_ / elapsed;
    const double published_fps = published_frames_ / elapsed;
    last_output_fps_ = static_cast<int16_t>(std::clamp(
      static_cast<int>(std::round(published_fps)),
      0,
      static_cast<int>(std::numeric_limits<int16_t>::max())));

    RCLCPP_INFO(
      get_logger(),
      "line detector fps: processed=%.1f published=%.1f",
      processed_fps,
      published_fps);

    processed_frames_ = 0;
    published_frames_ = 0;
    fps_window_start_ = current_time;
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string camera_frame_id_;
  int source_width_ = 1920;
  int source_height_ = 1080;
  int qos_keep_last_ = 1;
  float output_x_offset_ = 0.0F;
  float output_y_offset_ = 256.0F;
  float confidence_threshold_ = 0.0F;
  bool log_timing_ = true;
  bool log_fps_ = true;
  bool enable_lane_following_ = true;
  int16_t last_output_fps_ = -1;
  int processed_frames_ = 0;
  int published_frames_ = 0;
  rclcpp::Time fps_window_start_;

  LineDetectorConfig detector_config_;
  LineDetector detector_;

  rclcpp::Subscription<hbm_img_msgs::msg::HbmMsg1080P>::SharedPtr image_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sign_sub_;
  rclcpp::Publisher<ai_msgs::msg::PerceptionTargets>::SharedPtr publisher_;
};

}  // namespace line_detector

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<line_detector::LineDetectorNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("GetLineCoordinate"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
