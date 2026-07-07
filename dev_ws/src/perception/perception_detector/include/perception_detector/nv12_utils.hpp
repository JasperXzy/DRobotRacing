#ifndef PERCEPTION_DETECTOR_NV12_UTILS_HPP_
#define PERCEPTION_DETECTOR_NV12_UTILS_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

#include "perception_detector/detector_types.hpp"

namespace perception_detector
{

std::size_t expectedNv12Size(int width, int height);

bool preprocessNv12(
  const uint8_t * src_nv12,
  int src_width,
  int src_height,
  std::vector<uint8_t> & dst_nv12,
  int dst_width,
  int dst_height,
  PreprocessType preprocess_type,
  PreprocessInfo & info);

int clampInt(int value, int min_value, int max_value);

}  // namespace perception_detector

#endif  // PERCEPTION_DETECTOR_NV12_UTILS_HPP_
