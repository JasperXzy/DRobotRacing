#include "perception_detector/qr_roi_publisher.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <utility>

#include <sensor_msgs/image_encodings.hpp>

#include "perception_detector/nv12_utils.hpp"

namespace perception_detector
{

void QrRoiPublisher::configure(
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher,
  const QrRoiConfig & config)
{
  publisher_ = std::move(publisher);
  config_ = config;
}

const Detection * QrRoiPublisher::selectDetection(const std::vector<Detection> & detections) const
{
  const Detection * best = nullptr;

  for (const auto & detection : detections) {
    const bool class_matches =
      detection.class_id == config_.qr_class_id ||
      (!config_.qr_class_name.empty() && detection.class_name == config_.qr_class_name);
    if (!class_matches) {
      continue;
    }

    if (best == nullptr) {
      best = &detection;
      continue;
    }

    if (config_.choose_policy == "largest_area") {
      const auto best_area = best->width * best->height;
      const auto area = detection.width * detection.height;
      if (area > best_area) {
        best = &detection;
      }
    } else if (detection.score > best->score) {
      best = &detection;
    }
  }

  return best;
}

bool QrRoiPublisher::publishBestRoi(
  const uint8_t * src_nv12,
  int src_width,
  int src_height,
  const std::vector<Detection> & detections,
  const builtin_interfaces::msg::Time & stamp,
  const std::string & frame_id)
{
  if (!publisher_ || src_nv12 == nullptr || src_width <= 0 || src_height <= 0) {
    return false;
  }

  int roi_x = 0;
  int roi_y = 0;
  int roi_w = src_width;
  int roi_h = src_height;

  const auto * selected = selectDetection(detections);
  if (selected == nullptr) {
    if (!config_.publish_without_detection) {
      return false;
    }
  } else {
    const int dx = static_cast<int>(std::round(selected->width * config_.margin_ratio));
    const int dy = static_cast<int>(std::round(selected->height * config_.margin_ratio));
    const int x1 = clampInt(selected->x - dx, 0, src_width);
    const int y1 = clampInt(selected->y - dy, 0, src_height);
    const int x2 = clampInt(selected->x + selected->width + dx, 0, src_width);
    const int y2 = clampInt(selected->y + selected->height + dy, 0, src_height);

    roi_x = x1;
    roi_y = y1;
    roi_w = x2 - x1;
    roi_h = y2 - y1;
  }

  if (roi_w < config_.min_width || roi_h < config_.min_height) {
    return false;
  }

  sensor_msgs::msg::Image roi_msg;
  roi_msg.header.stamp = stamp;
  roi_msg.header.frame_id = frame_id;
  roi_msg.height = static_cast<uint32_t>(roi_h);
  roi_msg.width = static_cast<uint32_t>(roi_w);
  roi_msg.encoding = sensor_msgs::image_encodings::MONO8;
  roi_msg.is_bigendian = false;
  roi_msg.step = static_cast<uint32_t>(roi_w);
  roi_msg.data.resize(static_cast<std::size_t>(roi_w) * static_cast<std::size_t>(roi_h));

  const uint8_t * y_plane = src_nv12;
  for (int row = 0; row < roi_h; ++row) {
    const uint8_t * src = y_plane + (roi_y + row) * src_width + roi_x;
    uint8_t * dst = roi_msg.data.data() + row * roi_w;
    std::memcpy(dst, src, static_cast<std::size_t>(roi_w));
  }

  publisher_->publish(std::move(roi_msg));
  return true;
}

}  // namespace perception_detector
