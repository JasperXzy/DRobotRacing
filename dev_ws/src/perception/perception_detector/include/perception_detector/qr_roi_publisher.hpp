#ifndef PERCEPTION_DETECTOR_QR_ROI_PUBLISHER_HPP_
#define PERCEPTION_DETECTOR_QR_ROI_PUBLISHER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "perception_detector/detector_types.hpp"

namespace perception_detector
{

class QrRoiPublisher
{
public:
  void configure(
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher,
    const QrRoiConfig & config);

  bool publishBestRoi(
    const uint8_t * src_nv12,
    int src_width,
    int src_height,
    const std::vector<Detection> & detections,
    const builtin_interfaces::msg::Time & stamp,
    const std::string & frame_id);

private:
  const Detection * selectDetection(const std::vector<Detection> & detections) const;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  QrRoiConfig config_;
};

}  // namespace perception_detector

#endif  // PERCEPTION_DETECTOR_QR_ROI_PUBLISHER_HPP_
