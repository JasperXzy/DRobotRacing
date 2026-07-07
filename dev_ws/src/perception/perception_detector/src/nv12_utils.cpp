#include "perception_detector/nv12_utils.hpp"

#include <algorithm>
#include <cmath>

#include <opencv2/imgproc.hpp>

namespace perception_detector
{

namespace
{

int evenAtLeastTwo(int value, int max_value)
{
  int clipped = std::max(2, std::min(value, max_value));
  clipped &= ~1;
  if (clipped <= 0) {
    clipped = 2;
  }
  return std::min(clipped, max_value & ~1);
}

}  // namespace

PreprocessType parsePreprocessType(const std::string & value)
{
  if (value == "resize") {
    return PreprocessType::kResize;
  }
  return PreprocessType::kLetterbox;
}

std::size_t expectedNv12Size(int width, int height)
{
  if (width <= 0 || height <= 0) {
    return 0;
  }
  return static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 3U / 2U;
}

bool preprocessNv12(
  const uint8_t * src_nv12,
  int src_width,
  int src_height,
  std::vector<uint8_t> & dst_nv12,
  int dst_width,
  int dst_height,
  PreprocessType preprocess_type,
  PreprocessInfo & info)
{
  if (src_nv12 == nullptr || src_width <= 0 || src_height <= 0 ||
    dst_width <= 0 || dst_height <= 0 ||
    (src_width % 2) != 0 || (src_height % 2) != 0 ||
    (dst_width % 2) != 0 || (dst_height % 2) != 0)
  {
    return false;
  }

  dst_nv12.resize(expectedNv12Size(dst_width, dst_height));
  auto * dst_data = dst_nv12.data();

  cv::Mat dst_y(dst_height, dst_width, CV_8UC1, dst_data);
  cv::Mat dst_uv(dst_height / 2, dst_width / 2, CV_8UC2, dst_data + dst_width * dst_height);
  dst_y.setTo(127);
  dst_uv.setTo(cv::Scalar(127, 127));

  const cv::Mat src_y(src_height, src_width, CV_8UC1, const_cast<uint8_t *>(src_nv12));
  const cv::Mat src_uv(
    src_height / 2,
    src_width / 2,
    CV_8UC2,
    const_cast<uint8_t *>(src_nv12 + src_width * src_height));

  int new_width = dst_width;
  int new_height = dst_height;
  int x_shift = 0;
  int y_shift = 0;

  if (preprocess_type == PreprocessType::kLetterbox) {
    const float scale = std::min(
      static_cast<float>(dst_width) / static_cast<float>(src_width),
      static_cast<float>(dst_height) / static_cast<float>(src_height));
    new_width = evenAtLeastTwo(static_cast<int>(std::round(src_width * scale)), dst_width);
    new_height = evenAtLeastTwo(static_cast<int>(std::round(src_height * scale)), dst_height);
    x_shift = ((dst_width - new_width) / 2) & ~1;
    y_shift = ((dst_height - new_height) / 2) & ~1;
  }

  cv::Mat dst_y_roi = dst_y(cv::Rect(x_shift, y_shift, new_width, new_height));
  cv::resize(src_y, dst_y_roi, dst_y_roi.size(), 0.0, 0.0, cv::INTER_LINEAR);

  cv::Mat dst_uv_roi = dst_uv(cv::Rect(x_shift / 2, y_shift / 2, new_width / 2, new_height / 2));
  cv::resize(src_uv, dst_uv_roi, dst_uv_roi.size(), 0.0, 0.0, cv::INTER_LINEAR);

  info.x_scale = static_cast<float>(new_width) / static_cast<float>(src_width);
  info.y_scale = static_cast<float>(new_height) / static_cast<float>(src_height);
  info.x_shift = x_shift;
  info.y_shift = y_shift;
  info.input_width = dst_width;
  info.input_height = dst_height;
  return true;
}

int clampInt(int value, int min_value, int max_value)
{
  return std::max(min_value, std::min(value, max_value));
}

}  // namespace perception_detector
