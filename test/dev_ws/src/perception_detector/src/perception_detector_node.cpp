#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <ai_msgs/msg/perception_targets.hpp>
#include <ai_msgs/msg/roi.hpp>
#include <ai_msgs/msg/target.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <hbm_img_msgs/msg/hbm_msg1080_p.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int32.hpp>

#include "perception_detector/nv12_utils.hpp"
#include "perception_detector/qr_roi_publisher.hpp"
#include "perception_detector/yolo26_detector.hpp"

namespace perception_detector
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

std::vector<int> toIntVector(const std::vector<int64_t> & values)
{
  std::vector<int> converted;
  converted.reserve(values.size());
  for (auto value : values) {
    converted.push_back(static_cast<int>(value));
  }
  return converted;
}

}  // namespace

class PerceptionDetectorNode : public rclcpp::Node
{
public:
  PerceptionDetectorNode()
  : Node("perception_detector_node")
  {
    declareParameters();
    loadParameters();

    std::string error;
    if (!detector_.init(detector_config_, &error)) {
      throw std::runtime_error("failed to initialize YOLO26 detector: " + error);
    }

    auto sensor_qos = rclcpp::SensorDataQoS();
    sensor_qos.keep_last(static_cast<std::size_t>(qos_keep_last_));
    sensor_qos.best_effort();

    detection_pub_ = create_publisher<ai_msgs::msg::PerceptionTargets>(detection_topic_, sensor_qos);
    qr_roi_pub_ = create_publisher<sensor_msgs::msg::Image>(qr_roi_topic_, sensor_qos);
    qr_roi_publisher_.configure(qr_roi_pub_, qr_config_);

    image_sub_ = create_subscription<hbm_img_msgs::msg::HbmMsg1080P>(
      input_topic_,
      sensor_qos,
      std::bind(&PerceptionDetectorNode::imageCallback, this, std::placeholders::_1));
    sign_sub_ = create_subscription<std_msgs::msg::Int32>(
      "/sign4return",
      10,
      std::bind(&PerceptionDetectorNode::signCallback, this, std::placeholders::_1));

    last_fps_report_time_ = std::chrono::steady_clock::now();
    if (log_fps_) {
      fps_timer_ = create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&PerceptionDetectorNode::reportFps, this));
    }
    RCLCPP_INFO(
      get_logger(),
      "perception_detector started: input=%s, detections=%s, qr_roi=%s, model=%s",
      input_topic_.c_str(),
      detection_topic_.c_str(),
      qr_roi_topic_.c_str(),
      detector_config_.model_path.c_str());
  }

private:
  void signCallback(const std_msgs::msg::Int32 & msg)
  {
    if (msg.data != shutdown_value_ || shutdown_requested_.exchange(true)) {
      return;
    }

    // YOLO只服务于Task1二维码ROI。收到终止值后必须释放模型和张量，
    // 不能只暂停逐帧推理，否则packed model仍会占用BPU资源。
    inference_enabled_.store(false);
    image_sub_.reset();
    RCLCPP_WARN(
      get_logger(),
      "Task1 vision complete: releasing YOLO/BPU resources and exiting "
      "(/sign4return=%d)",
      shutdown_value_);
    detector_.release();
    rclcpp::shutdown();
  }

  void declareParameters()
  {
    declare_parameter<std::string>("input_topic", "/camera/image_hbmem");
    declare_parameter<std::string>("detection_topic", "/racing_obstacle_detection");
    declare_parameter<std::string>("qr_roi_topic", "/camera/qr_roi/image");
    declare_parameter<std::string>("camera_frame_id", "usb_cam");

    declare_parameter<int>("source_width", 1920);
    declare_parameter<int>("source_height", 1080);

    declare_parameter<std::string>(
      "model_path",
      "/root/D_Robot/test/dev_ws/models/yolo26m_detect_nashe_640x640_nv12_resize_default.hbm");
    declare_parameter<int>("model_input_width", 640);
    declare_parameter<int>("model_input_height", 640);
    declare_parameter<std::string>("preprocess_type", "letterbox");
    declare_parameter<int>("class_num", 2);
    declare_parameter<std::vector<std::string>>(
      "class_names", {"construction_cone", "parking_sign"});

    declare_parameter<double>("score_threshold", 0.25);
    declare_parameter<double>("nms_threshold", 0.45);
    declare_parameter<int>("nms_top_k", 300);
    declare_parameter<std::vector<int64_t>>("strides", {8, 16, 32});

    declare_parameter<std::string>("qr_class_name", "parking_sign");
    declare_parameter<int>("qr_class_id", 1);
    declare_parameter<double>("qr_roi_margin_ratio", 0.25);
    declare_parameter<int>("qr_roi_min_width", 16);
    declare_parameter<int>("qr_roi_min_height", 16);
    declare_parameter<bool>("publish_qr_roi_without_detection", false);
    declare_parameter<std::string>("choose_qr_roi_policy", "highest_score");

    declare_parameter<int>("qos_keep_last", 1);
    declare_parameter<std::string>("bpu_core", "any");
    declare_parameter<bool>("log_timing", true);
    declare_parameter<bool>("log_fps", true);
    declare_parameter<bool>("debug_image", false);
    declare_parameter<int>("shutdown_value", 10);
  }

  void loadParameters()
  {
    input_topic_ = get_parameter("input_topic").as_string();
    detection_topic_ = get_parameter("detection_topic").as_string();
    qr_roi_topic_ = get_parameter("qr_roi_topic").as_string();
    camera_frame_id_ = get_parameter("camera_frame_id").as_string();

    source_width_ = static_cast<int>(get_parameter("source_width").as_int());
    source_height_ = static_cast<int>(get_parameter("source_height").as_int());

    detector_config_.model_path = get_parameter("model_path").as_string();
    detector_config_.model_input_width = static_cast<int>(get_parameter("model_input_width").as_int());
    detector_config_.model_input_height = static_cast<int>(get_parameter("model_input_height").as_int());
    detector_config_.preprocess_type =
      parsePreprocessType(get_parameter("preprocess_type").as_string());
    detector_config_.class_num = static_cast<int>(get_parameter("class_num").as_int());
    detector_config_.class_names = get_parameter("class_names").as_string_array();
    detector_config_.score_threshold =
      static_cast<float>(get_parameter("score_threshold").as_double());
    detector_config_.nms_threshold =
      static_cast<float>(get_parameter("nms_threshold").as_double());
    detector_config_.nms_top_k = static_cast<int>(get_parameter("nms_top_k").as_int());
    detector_config_.strides = toIntVector(get_parameter("strides").as_integer_array());
    detector_config_.bpu_core = get_parameter("bpu_core").as_string();

    qr_config_.qr_class_name = get_parameter("qr_class_name").as_string();
    qr_config_.qr_class_id = static_cast<int>(get_parameter("qr_class_id").as_int());
    qr_config_.margin_ratio = get_parameter("qr_roi_margin_ratio").as_double();
    qr_config_.min_width = static_cast<int>(get_parameter("qr_roi_min_width").as_int());
    qr_config_.min_height = static_cast<int>(get_parameter("qr_roi_min_height").as_int());
    qr_config_.publish_without_detection =
      get_parameter("publish_qr_roi_without_detection").as_bool();
    qr_config_.choose_policy = get_parameter("choose_qr_roi_policy").as_string();

    qos_keep_last_ = std::max(1, static_cast<int>(get_parameter("qos_keep_last").as_int()));
    log_timing_ = get_parameter("log_timing").as_bool();
    log_fps_ = get_parameter("log_fps").as_bool();
    debug_image_ = get_parameter("debug_image").as_bool();
    shutdown_value_ = static_cast<int>(get_parameter("shutdown_value").as_int());

    if (debug_image_) {
      RCLCPP_WARN(
        get_logger(),
        "debug_image is requested but not implemented in this runtime path; no debug image will be published");
    }
    if (detector_config_.bpu_core != "any") {
      RCLCPP_WARN(
        get_logger(),
        "bpu_core=%s requested; this runtime currently submits with HB_UCP_BPU_CORE_ANY",
        detector_config_.bpu_core.c_str());
    }
  }

  void imageCallback(const hbm_img_msgs::msg::HbmMsg1080P & msg)
  {
    if (!inference_enabled_.load()) {
      return;
    }

    const int src_width = source_width_ > 0 ? source_width_ : static_cast<int>(msg.width);
    const int src_height = source_height_ > 0 ? source_height_ : static_cast<int>(msg.height);
    if (msg.step > 0U && msg.step != static_cast<uint32_t>(src_width)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        3000,
        "drop frame: unsupported NV12 row step=%u for source width=%d",
        msg.step,
        src_width);
      return;
    }

    const auto expected_size = expectedNv12Size(src_width, src_height);
    const auto available_size = msg.data_size > 0U ?
      static_cast<std::size_t>(msg.data_size) : msg.data.size();

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

    builtin_interfaces::msg::Time stamp = msg.time_stamp;
    if (stamp.sec == 0 && stamp.nanosec == 0U) {
      stamp = timeMsgFromRclTime(now());
    }

    std::vector<Detection> detections;
    PreprocessInfo preprocess_info;
    DetectorTiming timing;
    std::string error;
    const uint8_t * nv12_data = msg.data.data();
    if (!detector_.infer(
        nv12_data,
        src_width,
        src_height,
        detections,
        preprocess_info,
        &timing,
        &error))
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(),
        *get_clock(),
        3000,
        "detector inference failed: %s",
        error.c_str());
      return;
    }

    ++processed_frames_;
    publishDetections(detections, stamp);
    qr_roi_publisher_.publishBestRoi(
      nv12_data,
      src_width,
      src_height,
      detections,
      stamp,
      camera_frame_id_);

    if (log_timing_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "timing ms: preprocess=%.2f infer=%.2f postprocess=%.2f total=%.2f detections=%zu",
        timing.preprocess_ms,
        timing.infer_ms,
        timing.postprocess_ms,
        timing.total_ms,
        detections.size());
    }
  }

  void publishDetections(
    const std::vector<Detection> & detections,
    const builtin_interfaces::msg::Time & stamp)
  {
    ai_msgs::msg::PerceptionTargets msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = camera_frame_id_;
    msg.fps = last_output_fps_;

    msg.targets.reserve(detections.size());
    for (const auto & detection : detections) {
      ai_msgs::msg::Target target;
      target.type = detection.class_name;
      target.track_id = 0;

      ai_msgs::msg::Roi roi;
      roi.type = "rect";
      roi.rect.x_offset = static_cast<uint32_t>(detection.x);
      roi.rect.y_offset = static_cast<uint32_t>(detection.y);
      roi.rect.width = static_cast<uint32_t>(detection.width);
      roi.rect.height = static_cast<uint32_t>(detection.height);
      roi.rect.do_rectify = false;
      roi.confidence = detection.score;

      target.rois.push_back(std::move(roi));
      msg.targets.push_back(std::move(target));
    }

    detection_pub_->publish(std::move(msg));
  }

  void reportFps()
  {
    const auto current_time = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(
      current_time - last_fps_report_time_).count();
    if (elapsed <= 0.0) {
      return;
    }

    const double processed_fps = processed_frames_ / elapsed;
    last_output_fps_ = static_cast<int16_t>(std::clamp(
      static_cast<int>(std::round(processed_fps)),
      0,
      static_cast<int>(std::numeric_limits<int16_t>::max())));

    // Fast启动把本节点日志等级设为WARN；直接写stdout，保证实际完成的
    // YOLO推理帧率每秒可见，又不会将正常诊断伪装成WARN写入rosout。
    std::printf("[perception_detector] yolo_scan_fps=%.1f\n", processed_fps);
    std::fflush(stdout);

    processed_frames_ = 0;
    last_fps_report_time_ = current_time;
  }

  std::string input_topic_;
  std::string detection_topic_;
  std::string qr_roi_topic_;
  std::string camera_frame_id_;
  int source_width_ = 1920;
  int source_height_ = 1080;
  int qos_keep_last_ = 1;
  bool log_timing_ = true;
  bool log_fps_ = true;
  bool debug_image_ = false;
  int shutdown_value_ = 10;
  int16_t last_output_fps_ = -1;
  int processed_frames_ = 0;
  std::chrono::steady_clock::time_point last_fps_report_time_;
  std::atomic<bool> inference_enabled_{true};
  std::atomic<bool> shutdown_requested_{false};

  DetectorConfig detector_config_;
  QrRoiConfig qr_config_;
  Yolo26Detector detector_;
  QrRoiPublisher qr_roi_publisher_;

  rclcpp::Subscription<hbm_img_msgs::msg::HbmMsg1080P>::SharedPtr image_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sign_sub_;
  rclcpp::TimerBase::SharedPtr fps_timer_;
  rclcpp::Publisher<ai_msgs::msg::PerceptionTargets>::SharedPtr detection_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr qr_roi_pub_;
};

}  // namespace perception_detector

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<perception_detector::PerceptionDetectorNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("perception_detector_node"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
