#ifndef PERCEPTION_DETECTOR_DETECTOR_TYPES_HPP_
#define PERCEPTION_DETECTOR_DETECTOR_TYPES_HPP_

#include <string>
#include <vector>

namespace perception_detector
{

enum class PreprocessType
{
  kLetterbox,
  kResize,
};

struct Detection
{
  int class_id = -1;
  std::string class_name;
  float score = 0.0F;
  int x = 0;
  int y = 0;
  int width = 0;
  int height = 0;
};

struct PreprocessInfo
{
  float x_scale = 1.0F;
  float y_scale = 1.0F;
  int x_shift = 0;
  int y_shift = 0;
  int input_width = 0;
  int input_height = 0;
};

struct DetectorTiming
{
  double preprocess_ms = 0.0;
  double infer_ms = 0.0;
  double postprocess_ms = 0.0;
  double total_ms = 0.0;
};

struct DetectorConfig
{
  std::string model_path;
  int model_input_width = 640;
  int model_input_height = 640;
  PreprocessType preprocess_type = PreprocessType::kLetterbox;
  int class_num = 2;
  std::vector<std::string> class_names;
  float score_threshold = 0.25F;
  float nms_threshold = 0.45F;
  int nms_top_k = 300;
  std::vector<int> strides{8, 16, 32};
  std::string bpu_core = "any";
};

struct QrRoiConfig
{
  std::string qr_class_name = "parking_sign";
  int qr_class_id = 1;
  double margin_ratio = 0.25;
  int min_width = 16;
  int min_height = 16;
  bool publish_without_detection = false;
  std::string choose_policy = "highest_score";
};

PreprocessType parsePreprocessType(const std::string & value);

}  // namespace perception_detector

#endif  // PERCEPTION_DETECTOR_DETECTOR_TYPES_HPP_
