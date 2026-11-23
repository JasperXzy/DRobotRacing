#ifndef LINE_DETECTOR_LINE_DETECTOR_HPP_
#define LINE_DETECTOR_LINE_DETECTOR_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "hobot/dnn/hb_dnn.h"
#include "hobot/hb_ucp.h"
#include "hobot/hb_ucp_sys.h"

namespace line_detector
{

struct LineDetectorConfig
{
  std::string model_path;
  int model_input_width = 224;
  int model_input_height = 224;
  int logical_width = 640;
  int logical_height = 480;
  int crop_left = 0;
  int crop_top = 256;
  int crop_width = 640;
  int crop_height = 224;
};

struct LineDetectorResult
{
  float crop_x = 0.0F;
  float crop_y = 0.0F;
  float confidence = 0.0F;
  float x_norm = 0.0F;
  float y_norm = 0.0F;
  float confidence_logit = 0.0F;
};

struct LineDetectorTiming
{
  double preprocess_ms = 0.0;
  double infer_ms = 0.0;
  double postprocess_ms = 0.0;
  double total_ms = 0.0;
};

class LineDetector
{
public:
  LineDetector() = default;
  ~LineDetector();

  LineDetector(const LineDetector &) = delete;
  LineDetector & operator=(const LineDetector &) = delete;

  bool init(const LineDetectorConfig & config, std::string * error = nullptr);

  bool infer(
    const uint8_t * src_nv12,
    int src_width,
    int src_height,
    LineDetectorResult & result,
    LineDetectorTiming * timing,
    std::string * error = nullptr);

  void release();

  int inputWidth() const { return model_input_width_; }
  int inputHeight() const { return model_input_height_; }

private:
  bool loadModel(std::string * error);
  bool inspectModel(std::string * error);
  bool allocateTensors(std::string * error);
  bool preprocessNv12Crop(
    const uint8_t * src_nv12,
    int src_width,
    int src_height,
    std::string * error);
  bool fillInputTensors(std::string * error);
  bool runBpu(LineDetectorTiming * timing, std::string * error);
  bool postprocess(LineDetectorResult & result, std::string * error);

  static float sigmoid(float value);

  LineDetectorConfig config_;
  bool initialized_ = false;

  hbDNNPackedHandle_t packed_handle_ = nullptr;
  hbDNNHandle_t dnn_handle_ = nullptr;
  int input_count_ = 0;
  int output_count_ = 0;
  int model_input_width_ = 0;
  int model_input_height_ = 0;

  std::vector<hbDNNTensor> input_tensors_;
  std::vector<hbDNNTensor> output_tensors_;
  std::vector<bool> input_allocated_;
  std::vector<bool> output_allocated_;

  std::vector<uint8_t> model_nv12_;
  int output_element_count_ = 0;
};

std::size_t expectedNv12Size(int width, int height);

}  // namespace line_detector

#endif  // LINE_DETECTOR_LINE_DETECTOR_HPP_
