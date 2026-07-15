#include "perception_detector/yolo26_detector.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <sstream>

#include <opencv2/dnn/dnn.hpp>

#include "perception_detector/nv12_utils.hpp"

namespace perception_detector
{

namespace
{

using Clock = std::chrono::steady_clock;

double elapsedMs(const Clock::time_point & start, const Clock::time_point & end)
{
  return std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
}

bool checkReturn(int ret, const char * what, std::string * error)
{
  if (ret == 0) {
    return true;
  }
  if (error != nullptr) {
    std::ostringstream oss;
    oss << what << " failed, ret=" << ret;
    *error = oss.str();
  }
  return false;
}

std::string shapeToString(const hbDNNTensorProperties & props)
{
  std::ostringstream oss;
  oss << "(";
  for (int i = 0; i < props.validShape.numDimensions; ++i) {
    if (i > 0) {
      oss << ", ";
    }
    oss << props.validShape.dimensionSize[i];
  }
  oss << ")";
  return oss.str();
}

}  // namespace

Yolo26Detector::~Yolo26Detector()
{
  release();
}

bool Yolo26Detector::init(const DetectorConfig & config, std::string * error)
{
  release();
  config_ = config;

  if (config_.model_path.empty()) {
    if (error != nullptr) {
      *error = "model_path is empty";
    }
    return false;
  }
  if (config_.class_num <= 0) {
    if (error != nullptr) {
      *error = "class_num must be positive";
    }
    return false;
  }
  if (config_.class_names.empty()) {
    config_.class_names.reserve(static_cast<std::size_t>(config_.class_num));
    for (int i = 0; i < config_.class_num; ++i) {
      config_.class_names.push_back("class_" + std::to_string(i));
    }
  }
  if (static_cast<int>(config_.class_names.size()) != config_.class_num) {
    if (error != nullptr) {
      *error = "class_names size must equal class_num";
    }
    return false;
  }
  if (config_.strides.size() != 3U) {
    if (error != nullptr) {
      *error = "strides must contain exactly three entries";
    }
    return false;
  }

  if (!loadModel(error) || !inspectModel(error) || !resolveOutputOrder(error) ||
    !allocateTensors(error))
  {
    release();
    return false;
  }

  model_nv12_.resize(expectedNv12Size(model_input_width_, model_input_height_));
  initialized_ = true;
  return true;
}

bool Yolo26Detector::loadModel(std::string * error)
{
  const char * model_file = config_.model_path.c_str();
  if (!checkReturn(
      hbDNNInitializeFromFiles(&packed_handle_, &model_file, 1),
      "hbDNNInitializeFromFiles", error))
  {
    return false;
  }

  const char ** model_names = nullptr;
  int model_count = 0;
  if (!checkReturn(
      hbDNNGetModelNameList(&model_names, &model_count, packed_handle_),
      "hbDNNGetModelNameList", error))
  {
    return false;
  }
  if (model_count <= 0 || model_names == nullptr) {
    if (error != nullptr) {
      *error = "HBM contains no model";
    }
    return false;
  }

  return checkReturn(
    hbDNNGetModelHandle(&dnn_handle_, packed_handle_, model_names[0]),
    "hbDNNGetModelHandle", error);
}

bool Yolo26Detector::inspectModel(std::string * error)
{
  if (!checkReturn(hbDNNGetInputCount(&input_count_, dnn_handle_), "hbDNNGetInputCount", error)) {
    return false;
  }
  if (input_count_ != 2) {
    if (error != nullptr) {
      *error = "YOLO26 NV12 HBM must have 2 inputs (Y and UV), got " +
        std::to_string(input_count_);
    }
    return false;
  }

  hbDNNTensorProperties input_props{};
  if (!checkReturn(
      hbDNNGetInputTensorProperties(&input_props, dnn_handle_, 0),
      "hbDNNGetInputTensorProperties[0]", error))
  {
    return false;
  }
  if (input_props.validShape.numDimensions != 4) {
    if (error != nullptr) {
      *error = "input[0] must be 4D, got shape " + shapeToString(input_props);
    }
    return false;
  }

  int input_h = input_props.validShape.dimensionSize[1];
  int input_w = input_props.validShape.dimensionSize[2];
  if (input_h <= 3 && input_props.validShape.dimensionSize[2] > 3) {
    input_h = input_props.validShape.dimensionSize[2];
    input_w = input_props.validShape.dimensionSize[3];
  }
  if (input_h <= 0 || input_w <= 0) {
    if (error != nullptr) {
      *error = "invalid model input shape " + shapeToString(input_props);
    }
    return false;
  }

  if (config_.model_input_width > 0 && config_.model_input_width != input_w) {
    if (error != nullptr) {
      *error = "model_input_width mismatch: config=" + std::to_string(config_.model_input_width) +
        ", model=" + std::to_string(input_w);
    }
    return false;
  }
  if (config_.model_input_height > 0 && config_.model_input_height != input_h) {
    if (error != nullptr) {
      *error = "model_input_height mismatch: config=" + std::to_string(config_.model_input_height) +
        ", model=" + std::to_string(input_h);
    }
    return false;
  }

  model_input_width_ = input_w;
  model_input_height_ = input_h;
  config_.model_input_width = input_w;
  config_.model_input_height = input_h;

  if (!checkReturn(hbDNNGetOutputCount(&output_count_, dnn_handle_), "hbDNNGetOutputCount", error)) {
    return false;
  }
  if (output_count_ != 6) {
    if (error != nullptr) {
      *error = "YOLO26 detect HBM must have 6 outputs, got " + std::to_string(output_count_);
    }
    return false;
  }

  for (int i = 0; i < output_count_; ++i) {
    hbDNNTensorProperties output_props{};
    if (!checkReturn(
        hbDNNGetOutputTensorProperties(&output_props, dnn_handle_, i),
        "hbDNNGetOutputTensorProperties", error))
    {
      return false;
    }
    if (output_props.validShape.numDimensions != 4) {
      if (error != nullptr) {
        *error = "output[" + std::to_string(i) + "] must be 4D, got " +
          shapeToString(output_props);
      }
      return false;
    }
    if (static_cast<int>(output_props.quantiType) != 0) {
      if (error != nullptr) {
        *error = "output[" + std::to_string(i) +
          "] is quantized; this runtime currently expects float outputs after Dequantize";
      }
      return false;
    }
  }

  return true;
}

bool Yolo26Detector::resolveOutputOrder(std::string * error)
{
  output_order_.fill(-1);

  for (int i = 0; i < output_count_; ++i) {
    hbDNNTensorProperties props{};
    if (!checkReturn(
        hbDNNGetOutputTensorProperties(&props, dnn_handle_, i),
        "hbDNNGetOutputTensorProperties", error))
    {
      return false;
    }

    const int h = props.validShape.dimensionSize[1];
    const int w = props.validShape.dimensionSize[2];
    const int c = props.validShape.dimensionSize[3];

    for (std::size_t s = 0; s < config_.strides.size(); ++s) {
      const int stride = config_.strides[s];
      if (stride <= 0) {
        continue;
      }
      const int expected_h = model_input_height_ / stride;
      const int expected_w = model_input_width_ / stride;
      if (h != expected_h || w != expected_w) {
        continue;
      }
      if (c == 4) {
        output_order_[s * 2] = i;
      } else if (c == config_.class_num) {
        output_order_[s * 2 + 1] = i;
      }
    }
  }

  for (std::size_t i = 0; i < output_order_.size(); ++i) {
    if (output_order_[i] < 0) {
      if (error != nullptr) {
        *error = "failed to resolve YOLO26 output order at slot " + std::to_string(i);
      }
      return false;
    }
  }

  return true;
}

bool Yolo26Detector::allocateTensors(std::string * error)
{
  input_tensors_.assign(static_cast<std::size_t>(input_count_), hbDNNTensor{});
  output_tensors_.assign(static_cast<std::size_t>(output_count_), hbDNNTensor{});
  input_allocated_.assign(static_cast<std::size_t>(input_count_), false);
  output_allocated_.assign(static_cast<std::size_t>(output_count_), false);

  for (int i = 0; i < input_count_; ++i) {
    if (!checkReturn(
        hbDNNGetInputTensorProperties(&input_tensors_[i].properties, dnn_handle_, i),
        "hbDNNGetInputTensorProperties", error))
    {
      return false;
    }

    int byte_size = 0;
    if (i == 0) {
      byte_size = model_input_height_ * model_input_width_;
      input_tensors_[i].properties.validShape.dimensionSize[0] = 1;
      input_tensors_[i].properties.validShape.dimensionSize[1] = model_input_height_;
      input_tensors_[i].properties.validShape.dimensionSize[2] = model_input_width_;
      input_tensors_[i].properties.validShape.dimensionSize[3] = 1;
      input_tensors_[i].properties.stride[3] = 1;
      input_tensors_[i].properties.stride[2] = 1;
      input_tensors_[i].properties.stride[1] = model_input_width_;
      input_tensors_[i].properties.stride[0] = model_input_height_ * model_input_width_;
    } else {
      const int uv_h = model_input_height_ / 2;
      const int uv_w = model_input_width_ / 2;
      byte_size = uv_h * uv_w * 2;
      input_tensors_[i].properties.validShape.dimensionSize[0] = 1;
      input_tensors_[i].properties.validShape.dimensionSize[1] = uv_h;
      input_tensors_[i].properties.validShape.dimensionSize[2] = uv_w;
      input_tensors_[i].properties.validShape.dimensionSize[3] = 2;
      input_tensors_[i].properties.stride[3] = 1;
      input_tensors_[i].properties.stride[2] = 2;
      input_tensors_[i].properties.stride[1] = uv_w * 2;
      input_tensors_[i].properties.stride[0] = uv_h * uv_w * 2;
    }

    if (!checkReturn(
        hbUCPMallocCached(&input_tensors_[i].sysMem, byte_size, 0),
        "hbUCPMallocCached[input]", error))
    {
      return false;
    }
    input_allocated_[i] = true;
  }

  for (int i = 0; i < output_count_; ++i) {
    if (!checkReturn(
        hbDNNGetOutputTensorProperties(&output_tensors_[i].properties, dnn_handle_, i),
        "hbDNNGetOutputTensorProperties", error))
    {
      return false;
    }
    if (!checkReturn(
        hbUCPMallocCached(&output_tensors_[i].sysMem, output_tensors_[i].properties.alignedByteSize, 0),
        "hbUCPMallocCached[output]", error))
    {
      return false;
    }
    output_allocated_[i] = true;
  }

  return true;
}

bool Yolo26Detector::infer(
  const uint8_t * src_nv12,
  int src_width,
  int src_height,
  std::vector<Detection> & detections,
  PreprocessInfo & preprocess_info,
  DetectorTiming * timing,
  std::string * error)
{
  detections.clear();
  if (timing != nullptr) {
    *timing = DetectorTiming{};
  }

  if (!initialized_) {
    if (error != nullptr) {
      *error = "detector is not initialized";
    }
    return false;
  }

  const auto total_start = Clock::now();
  const auto pre_start = Clock::now();
  if (!preprocessNv12(
      src_nv12,
      src_width,
      src_height,
      model_nv12_,
      model_input_width_,
      model_input_height_,
      config_.preprocess_type,
      preprocess_info))
  {
    if (error != nullptr) {
      *error = "NV12 preprocessing failed";
    }
    return false;
  }
  const auto pre_end = Clock::now();
  if (timing != nullptr) {
    timing->preprocess_ms = elapsedMs(pre_start, pre_end);
  }

  if (!fillInputTensors(model_nv12_, error)) {
    return false;
  }

  if (!runBpu(timing, error)) {
    return false;
  }

  const auto post_start = Clock::now();
  if (!postprocess(src_width, src_height, preprocess_info, detections, error)) {
    return false;
  }
  const auto post_end = Clock::now();
  if (timing != nullptr) {
    timing->postprocess_ms = elapsedMs(post_start, post_end);
    timing->total_ms = elapsedMs(total_start, post_end);
  }

  return true;
}

bool Yolo26Detector::fillInputTensors(const std::vector<uint8_t> & nv12, std::string * error)
{
  const std::size_t y_size = static_cast<std::size_t>(model_input_width_) *
    static_cast<std::size_t>(model_input_height_);
  const std::size_t uv_size = y_size / 2U;
  if (nv12.size() < y_size + uv_size) {
    if (error != nullptr) {
      *error = "preprocessed NV12 buffer is too small";
    }
    return false;
  }

  std::memcpy(input_tensors_[0].sysMem.virAddr, nv12.data(), y_size);
  std::memcpy(input_tensors_[1].sysMem.virAddr, nv12.data() + y_size, uv_size);

  if (!checkReturn(
      hbUCPMemFlush(&input_tensors_[0].sysMem, HB_SYS_MEM_CACHE_CLEAN),
      "hbUCPMemFlush[input0]", error))
  {
    return false;
  }
  return checkReturn(
    hbUCPMemFlush(&input_tensors_[1].sysMem, HB_SYS_MEM_CACHE_CLEAN),
    "hbUCPMemFlush[input1]", error);
}

bool Yolo26Detector::runBpu(DetectorTiming * timing, std::string * error)
{
  const auto infer_start = Clock::now();
  hbUCPTaskHandle_t task = nullptr;
  if (!checkReturn(
      hbDNNInferV2(&task, output_tensors_.data(), input_tensors_.data(), dnn_handle_),
      "hbDNNInferV2", error))
  {
    return false;
  }

  hbUCPSchedParam param;
  HB_UCP_INITIALIZE_SCHED_PARAM(&param);
  param.backend = HB_UCP_BPU_CORE_ANY;

  if (!checkReturn(hbUCPSubmitTask(task, &param), "hbUCPSubmitTask", error)) {
    hbUCPReleaseTask(task);
    return false;
  }
  if (!checkReturn(hbUCPWaitTaskDone(task, 0), "hbUCPWaitTaskDone", error)) {
    hbUCPReleaseTask(task);
    return false;
  }

  for (auto & output : output_tensors_) {
    if (!checkReturn(
        hbUCPMemFlush(&output.sysMem, HB_SYS_MEM_CACHE_INVALIDATE),
        "hbUCPMemFlush[output]", error))
    {
      hbUCPReleaseTask(task);
      return false;
    }
  }

  hbUCPReleaseTask(task);
  const auto infer_end = Clock::now();
  if (timing != nullptr) {
    timing->infer_ms = elapsedMs(infer_start, infer_end);
  }
  return true;
}

bool Yolo26Detector::postprocess(
  int src_width,
  int src_height,
  const PreprocessInfo & preprocess_info,
  std::vector<Detection> & detections,
  std::string * error)
{
  if (preprocess_info.x_scale <= 0.0F || preprocess_info.y_scale <= 0.0F) {
    if (error != nullptr) {
      *error = "invalid preprocess scale";
    }
    return false;
  }

  std::vector<std::vector<cv::Rect2d>> bboxes(static_cast<std::size_t>(config_.class_num));
  std::vector<std::vector<float>> scores(static_cast<std::size_t>(config_.class_num));
  const float conf_threshold_raw = calcLogitThreshold(config_.score_threshold);

  for (std::size_t s = 0; s < config_.strides.size(); ++s) {
    const int stride = config_.strides[s];
    const int box_idx = output_order_[s * 2];
    const int cls_idx = output_order_[s * 2 + 1];
    const int h = model_input_height_ / stride;
    const int w = model_input_width_ / stride;

    const auto * box_ptr = static_cast<const float *>(output_tensors_[box_idx].sysMem.virAddr);
    const auto * cls_ptr = static_cast<const float *>(output_tensors_[cls_idx].sysMem.virAddr);

    for (int row = 0; row < h; ++row) {
      for (int col = 0; col < w; ++col) {
        const int offset = row * w + col;
        const float * cur_cls = cls_ptr + offset * config_.class_num;
        const float * cur_box = box_ptr + offset * 4;

        int max_id = 0;
        for (int c = 1; c < config_.class_num; ++c) {
          if (cur_cls[c] > cur_cls[max_id]) {
            max_id = c;
          }
        }
        if (cur_cls[max_id] < conf_threshold_raw) {
          continue;
        }

        const float grid_center_x = static_cast<float>(col) + 0.5F;
        const float grid_center_y = static_cast<float>(row) + 0.5F;

        const float x1 = (grid_center_x - cur_box[0]) * stride;
        const float y1 = (grid_center_y - cur_box[1]) * stride;
        const float x2 = (grid_center_x + cur_box[2]) * stride;
        const float y2 = (grid_center_y + cur_box[3]) * stride;
        if (x2 <= x1 || y2 <= y1) {
          continue;
        }

        bboxes[max_id].emplace_back(x1, y1, x2 - x1, y2 - y1);
        scores[max_id].push_back(sigmoid(cur_cls[max_id]));
      }
    }
  }

  for (int class_id = 0; class_id < config_.class_num; ++class_id) {
    auto & class_boxes = bboxes[class_id];
    auto & class_scores = scores[class_id];
    if (class_boxes.empty()) {
      continue;
    }

    std::vector<int> indices;
    cv::dnn::NMSBoxes(
      class_boxes,
      class_scores,
      config_.score_threshold,
      config_.nms_threshold,
      indices,
      1.0F,
      config_.nms_top_k);

    for (int idx : indices) {
      const auto & box = class_boxes[idx];
      float x1 = static_cast<float>((box.x - preprocess_info.x_shift) / preprocess_info.x_scale);
      float y1 = static_cast<float>((box.y - preprocess_info.y_shift) / preprocess_info.y_scale);
      float x2 = static_cast<float>(
        (box.x + box.width - preprocess_info.x_shift) / preprocess_info.x_scale);
      float y2 = static_cast<float>(
        (box.y + box.height - preprocess_info.y_shift) / preprocess_info.y_scale);

      x1 = std::clamp(x1, 0.0F, static_cast<float>(src_width));
      y1 = std::clamp(y1, 0.0F, static_cast<float>(src_height));
      x2 = std::clamp(x2, 0.0F, static_cast<float>(src_width));
      y2 = std::clamp(y2, 0.0F, static_cast<float>(src_height));

      const int x = clampInt(static_cast<int>(std::round(x1)), 0, src_width);
      const int y = clampInt(static_cast<int>(std::round(y1)), 0, src_height);
      const int right = clampInt(static_cast<int>(std::round(x2)), 0, src_width);
      const int bottom = clampInt(static_cast<int>(std::round(y2)), 0, src_height);
      const int width = right - x;
      const int height = bottom - y;
      if (width <= 0 || height <= 0) {
        continue;
      }

      Detection detection;
      detection.class_id = class_id;
      detection.class_name = config_.class_names[static_cast<std::size_t>(class_id)];
      detection.score = class_scores[idx];
      detection.x = x;
      detection.y = y;
      detection.width = width;
      detection.height = height;
      detections.push_back(std::move(detection));
    }
  }

  std::sort(detections.begin(), detections.end(), [](const Detection & lhs, const Detection & rhs) {
    return lhs.score > rhs.score;
  });

  return true;
}

float Yolo26Detector::sigmoid(float value)
{
  return 1.0F / (1.0F + std::exp(-value));
}

float Yolo26Detector::calcLogitThreshold(float score_threshold)
{
  const float t = std::max(1e-6F, std::min(score_threshold, 1.0F - 1e-6F));
  return -std::log(1.0F / t - 1.0F);
}

void Yolo26Detector::release()
{
  for (std::size_t i = 0; i < input_tensors_.size(); ++i) {
    if (i < input_allocated_.size() && input_allocated_[i]) {
      hbUCPFree(&input_tensors_[i].sysMem);
      input_allocated_[i] = false;
    }
  }
  for (std::size_t i = 0; i < output_tensors_.size(); ++i) {
    if (i < output_allocated_.size() && output_allocated_[i]) {
      hbUCPFree(&output_tensors_[i].sysMem);
      output_allocated_[i] = false;
    }
  }

  input_tensors_.clear();
  output_tensors_.clear();
  input_allocated_.clear();
  output_allocated_.clear();
  model_nv12_.clear();
  output_order_.fill(-1);

  if (packed_handle_ != nullptr) {
    hbDNNRelease(packed_handle_);
    packed_handle_ = nullptr;
  }
  dnn_handle_ = nullptr;
  input_count_ = 0;
  output_count_ = 0;
  model_input_width_ = 0;
  model_input_height_ = 0;
  initialized_ = false;
}

}  // namespace perception_detector
