#include "line_detector/line_detector.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <sstream>

#include <opencv2/imgproc.hpp>

namespace line_detector
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

int evenFloor(int value)
{
  return value & ~1;
}

int evenCeil(int value)
{
  return (value + 1) & ~1;
}

int elementCount(const hbDNNTensorProperties & props)
{
  int count = 1;
  for (int i = 0; i < props.validShape.numDimensions; ++i) {
    const int dim = props.validShape.dimensionSize[i];
    if (dim <= 0) {
      return 0;
    }
    count *= dim;
  }
  return count;
}

}  // namespace

std::size_t expectedNv12Size(int width, int height)
{
  if (width <= 0 || height <= 0) {
    return 0U;
  }
  return static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 3U / 2U;
}

LineDetector::~LineDetector()
{
  release();
}

bool LineDetector::init(const LineDetectorConfig & config, std::string * error)
{
  release();
  config_ = config;

  if (config_.model_path.empty()) {
    if (error != nullptr) {
      *error = "model_path is empty";
    }
    return false;
  }
  if (config_.model_input_width <= 0 || config_.model_input_height <= 0 ||
    (config_.model_input_width % 2) != 0 || (config_.model_input_height % 2) != 0)
  {
    if (error != nullptr) {
      *error = "model input width/height must be positive even numbers";
    }
    return false;
  }
  if (config_.logical_width <= 0 || config_.logical_height <= 0 ||
    config_.crop_width <= 0 || config_.crop_height <= 0 ||
    config_.crop_left < 0 || config_.crop_top < 0 ||
    config_.crop_left + config_.crop_width > config_.logical_width ||
    config_.crop_top + config_.crop_height > config_.logical_height)
  {
    if (error != nullptr) {
      *error = "invalid logical crop parameters";
    }
    return false;
  }

  if (!loadModel(error) || !inspectModel(error) || !allocateTensors(error)) {
    release();
    return false;
  }

  model_nv12_.resize(expectedNv12Size(model_input_width_, model_input_height_));
  initialized_ = true;
  return true;
}

bool LineDetector::loadModel(std::string * error)
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

bool LineDetector::inspectModel(std::string * error)
{
  if (!checkReturn(hbDNNGetInputCount(&input_count_, dnn_handle_), "hbDNNGetInputCount", error)) {
    return false;
  }
  if (input_count_ != 2) {
    if (error != nullptr) {
      *error = "line detector NV12 HBM must have 2 inputs (Y and UV), got " +
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
  if (output_count_ != 1) {
    if (error != nullptr) {
      *error = "line detector HBM must have 1 output, got " + std::to_string(output_count_);
    }
    return false;
  }

  hbDNNTensorProperties output_props{};
  if (!checkReturn(
      hbDNNGetOutputTensorProperties(&output_props, dnn_handle_, 0),
      "hbDNNGetOutputTensorProperties[0]", error))
  {
    return false;
  }
  output_element_count_ = elementCount(output_props);
  if (output_element_count_ < 3) {
    if (error != nullptr) {
      *error = "line detector output must contain at least 3 values, got shape " +
        shapeToString(output_props);
    }
    return false;
  }
  if (static_cast<int>(output_props.quantiType) != 0) {
    if (error != nullptr) {
      *error = "line detector output is quantized; this runtime expects float output";
    }
    return false;
  }

  return true;
}

bool LineDetector::allocateTensors(std::string * error)
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
    input_allocated_[static_cast<std::size_t>(i)] = true;
  }

  for (int i = 0; i < output_count_; ++i) {
    if (!checkReturn(
        hbDNNGetOutputTensorProperties(&output_tensors_[i].properties, dnn_handle_, i),
        "hbDNNGetOutputTensorProperties", error))
    {
      return false;
    }
    if (!checkReturn(
        hbUCPMallocCached(
          &output_tensors_[i].sysMem,
          output_tensors_[i].properties.alignedByteSize,
          0),
        "hbUCPMallocCached[output]", error))
    {
      return false;
    }
    output_allocated_[static_cast<std::size_t>(i)] = true;
  }

  return true;
}

bool LineDetector::infer(
  const uint8_t * src_nv12,
  int src_width,
  int src_height,
  LineDetectorResult & result,
  LineDetectorTiming * timing,
  std::string * error)
{
  result = LineDetectorResult{};
  if (timing != nullptr) {
    *timing = LineDetectorTiming{};
  }

  if (!initialized_) {
    if (error != nullptr) {
      *error = "detector is not initialized";
    }
    return false;
  }

  const auto total_start = Clock::now();
  const auto pre_start = Clock::now();
  if (!preprocessNv12Crop(src_nv12, src_width, src_height, error)) {
    return false;
  }
  const auto pre_end = Clock::now();
  if (timing != nullptr) {
    timing->preprocess_ms = elapsedMs(pre_start, pre_end);
  }

  if (!fillInputTensors(error) || !runBpu(timing, error)) {
    return false;
  }

  const auto post_start = Clock::now();
  if (!postprocess(result, error)) {
    return false;
  }
  const auto post_end = Clock::now();
  if (timing != nullptr) {
    timing->postprocess_ms = elapsedMs(post_start, post_end);
    timing->total_ms = elapsedMs(total_start, post_end);
  }

  return true;
}

bool LineDetector::preprocessNv12Crop(
  const uint8_t * src_nv12,
  int src_width,
  int src_height,
  std::string * error)
{
  if (src_nv12 == nullptr || src_width <= 0 || src_height <= 0 ||
    (src_width % 2) != 0 || (src_height % 2) != 0)
  {
    if (error != nullptr) {
      *error = "invalid source NV12 image";
    }
    return false;
  }

  int left = evenFloor(static_cast<int>(
    std::round(static_cast<double>(config_.crop_left) * src_width / config_.logical_width)));
  int top = evenFloor(static_cast<int>(
    std::round(static_cast<double>(config_.crop_top) * src_height / config_.logical_height)));
  int right = evenCeil(static_cast<int>(
    std::round(static_cast<double>(config_.crop_left + config_.crop_width) *
    src_width / config_.logical_width)));
  int bottom = evenCeil(static_cast<int>(
    std::round(static_cast<double>(config_.crop_top + config_.crop_height) *
    src_height / config_.logical_height)));

  left = std::clamp(left, 0, src_width - 2);
  top = std::clamp(top, 0, src_height - 2);
  right = std::clamp(right, left + 2, src_width);
  bottom = std::clamp(bottom, top + 2, src_height);
  right = evenFloor(right);
  bottom = evenFloor(bottom);

  const int roi_width = right - left;
  const int roi_height = bottom - top;
  if (roi_width <= 0 || roi_height <= 0) {
    if (error != nullptr) {
      *error = "computed source crop ROI is empty";
    }
    return false;
  }

  model_nv12_.resize(expectedNv12Size(model_input_width_, model_input_height_));
  auto * dst_data = model_nv12_.data();

  cv::Mat dst_y(model_input_height_, model_input_width_, CV_8UC1, dst_data);
  cv::Mat dst_uv(
    model_input_height_ / 2,
    model_input_width_ / 2,
    CV_8UC2,
    dst_data + model_input_width_ * model_input_height_);

  const cv::Mat src_y(src_height, src_width, CV_8UC1, const_cast<uint8_t *>(src_nv12));
  const cv::Mat src_uv(
    src_height / 2,
    src_width / 2,
    CV_8UC2,
    const_cast<uint8_t *>(src_nv12 + src_width * src_height));

  const cv::Mat roi_y = src_y(cv::Rect(left, top, roi_width, roi_height));
  const cv::Mat roi_uv = src_uv(cv::Rect(left / 2, top / 2, roi_width / 2, roi_height / 2));

  cv::resize(roi_y, dst_y, dst_y.size(), 0.0, 0.0, cv::INTER_LINEAR);
  cv::resize(roi_uv, dst_uv, dst_uv.size(), 0.0, 0.0, cv::INTER_LINEAR);
  return true;
}

bool LineDetector::fillInputTensors(std::string * error)
{
  const std::size_t y_size = static_cast<std::size_t>(model_input_width_) *
    static_cast<std::size_t>(model_input_height_);
  const std::size_t uv_size = y_size / 2U;
  if (model_nv12_.size() < y_size + uv_size) {
    if (error != nullptr) {
      *error = "preprocessed NV12 buffer is too small";
    }
    return false;
  }

  std::memcpy(input_tensors_[0].sysMem.virAddr, model_nv12_.data(), y_size);
  std::memcpy(input_tensors_[1].sysMem.virAddr, model_nv12_.data() + y_size, uv_size);

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

bool LineDetector::runBpu(LineDetectorTiming * timing, std::string * error)
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

bool LineDetector::postprocess(LineDetectorResult & result, std::string * error)
{
  if (output_element_count_ < 3) {
    if (error != nullptr) {
      *error = "invalid output element count";
    }
    return false;
  }

  const auto * output_data = static_cast<const float *>(output_tensors_[0].sysMem.virAddr);
  float x_norm = output_data[0];
  float y_norm = output_data[1];
  const float confidence_logit = output_data[2];

  x_norm = std::clamp(x_norm, -1.0F, 1.0F);
  y_norm = std::clamp(y_norm, -1.0F, 1.0F);

  result.x_norm = x_norm;
  result.y_norm = y_norm;
  result.confidence_logit = confidence_logit;
  result.confidence = sigmoid(confidence_logit);
  result.crop_x = (x_norm * 0.5F + 0.5F) * static_cast<float>(config_.crop_width);
  result.crop_y = (0.5F - y_norm * 0.5F) * static_cast<float>(config_.crop_height);
  result.crop_x = std::clamp(result.crop_x, 0.0F, static_cast<float>(config_.crop_width));
  result.crop_y = std::clamp(result.crop_y, 0.0F, static_cast<float>(config_.crop_height));
  return true;
}

float LineDetector::sigmoid(float value)
{
  return 1.0F / (1.0F + std::exp(-value));
}

void LineDetector::release()
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

  if (packed_handle_ != nullptr) {
    hbDNNRelease(packed_handle_);
    packed_handle_ = nullptr;
  }
  dnn_handle_ = nullptr;
  input_count_ = 0;
  output_count_ = 0;
  output_element_count_ = 0;
  model_input_width_ = 0;
  model_input_height_ = 0;
  initialized_ = false;
}

}  // namespace line_detector
