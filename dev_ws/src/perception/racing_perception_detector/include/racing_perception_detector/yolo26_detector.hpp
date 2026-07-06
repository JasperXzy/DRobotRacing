#ifndef RACING_PERCEPTION_DETECTOR_YOLO26_DETECTOR_HPP_
#define RACING_PERCEPTION_DETECTOR_YOLO26_DETECTOR_HPP_

#include <array>
#include <string>
#include <vector>

#include "hobot/dnn/hb_dnn.h"
#include "hobot/hb_ucp.h"
#include "hobot/hb_ucp_sys.h"

#include "racing_perception_detector/detector_types.hpp"

namespace racing_perception_detector
{

class Yolo26Detector
{
public:
  Yolo26Detector() = default;
  ~Yolo26Detector();

  Yolo26Detector(const Yolo26Detector &) = delete;
  Yolo26Detector & operator=(const Yolo26Detector &) = delete;

  bool init(const DetectorConfig & config, std::string * error = nullptr);

  bool infer(
    const uint8_t * src_nv12,
    int src_width,
    int src_height,
    std::vector<Detection> & detections,
    PreprocessInfo & preprocess_info,
    DetectorTiming * timing,
    std::string * error = nullptr);

  void release();

  int inputWidth() const { return model_input_width_; }
  int inputHeight() const { return model_input_height_; }

private:
  bool loadModel(std::string * error);
  bool inspectModel(std::string * error);
  bool allocateTensors(std::string * error);
  bool resolveOutputOrder(std::string * error);
  bool fillInputTensors(const std::vector<uint8_t> & nv12, std::string * error);
  bool runBpu(DetectorTiming * timing, std::string * error);
  bool postprocess(
    int src_width,
    int src_height,
    const PreprocessInfo & preprocess_info,
    std::vector<Detection> & detections,
    std::string * error);

  static float sigmoid(float value);
  static float calcLogitThreshold(float score_threshold);

  DetectorConfig config_;
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
  std::array<int, 6> output_order_{{-1, -1, -1, -1, -1, -1}};

  std::vector<uint8_t> model_nv12_;
};

}  // namespace racing_perception_detector

#endif  // RACING_PERCEPTION_DETECTOR_YOLO26_DETECTOR_HPP_
