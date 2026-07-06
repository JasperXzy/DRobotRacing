/**
 * @file main.cpp
 * @brief YOLO26 Object Detection inference on RDK S100 (Nash-E)
 */

// ============================================================================
// Configuration Macros
// ============================================================================

#define LETTERBOX_TYPE 1
#define RESIZE_TYPE 0
#define PREPROCESS_TYPE LETTERBOX_TYPE

#define CLASSES_NUM 80
#define NMS_THRESHOLD 0.45f
#define SCORE_THRESHOLD 0.25f

#define FONT_SCALE 0.6
#define FONT_THICKNESS 2
#define BOX_THICKNESS 2

// ============================================================================
// Standard Includes
// ============================================================================

#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn/dnn.hpp>

#ifdef _WIN32
#include <direct.h>
#define getcwd _getcwd
#else
#include <unistd.h>
#include <libgen.h>
#endif

#include "hobot/dnn/hb_dnn.h"
#include "hobot/hb_ucp.h"
#include "hobot/hb_ucp_sys.h"

// ============================================================================
// Utility Macros
// ============================================================================

#define CHECK_SUCCESS(value, errmsg)                                         \
    do {                                                                     \
        auto ret_code = value;                                               \
        if (ret_code != 0) {                                                 \
            std::cerr << "\033[1;31m[ERROR]\033[0m " << __FILE__ << ":"      \
                      << __LINE__ << " " << errmsg                           \
                      << ", error code: " << ret_code << std::endl;          \
            return ret_code;                                                 \
        }                                                                    \
    } while (0)

#define LOG_INFO(msg) \
    std::cout << "\033[1;32m[INFO]\033[0m " << msg << std::endl

#define LOG_WARN(msg) \
    std::cout << "\033[1;33m[WARN]\033[0m " << msg << std::endl

#define LOG_ERROR(msg) \
    std::cerr << "\033[1;31m[ERROR]\033[0m " << msg << std::endl

#define LOG_TIME(msg, duration) \
    std::cout << "\033[1;36m[" << msg << "]\033[0m = " << std::fixed         \
              << std::setprecision(2) << (duration) << " ms" << std::endl

// ============================================================================
// Detection Structure
// ============================================================================

struct Detection {
    int class_id;
    float confidence;
    cv::Rect2d bbox;
    
    Detection(int id, float conf, const cv::Rect2d& box) 
        : class_id(id), confidence(conf), bbox(box) {}
};

// ============================================================================
// Path Utilities
// ============================================================================

/**
 * @brief Get project root directory relative to current working directory
 * build -> detect -> cpp -> runtime -> yolo26 -> Vision -> samples -> root
 */
std::string getProjectRoot() {
    char cwd[4096];
    if (getcwd(cwd, sizeof(cwd)) == nullptr) {
        return ".";
    }
    std::string relative_path = std::string(cwd) + "/../../../../../../../";
    
    char* resolved = realpath(relative_path.c_str(), nullptr);
    if (resolved) {
        std::string result(resolved);
        free(resolved);
        return result;
    }
    return relative_path;
}

/**
 * @brief Get default label file path based on task type
 */
std::string getDefaultLabelPath(const std::string& project_root, 
                                        const std::string& task) {
    if (task == "detect" || task == "seg" || task == "pose") {
        return project_root + "/datasets/coco/coco_classes.names";
    } else if (task == "cls") {
        return project_root + "/datasets/imagenet/imagenet_classes.names";
    } else if (task == "obb") {
        return project_root + "/datasets/dotav1/dota_classes.names";
    }
    return project_root + "/datasets/coco/coco_classes.names";
}

// ============================================================================
// Default Color Palette for Visualization
// ============================================================================

const std::vector<cv::Scalar>& getDefaultColors() {
    static const std::vector<cv::Scalar> COLORS = {
        cv::Scalar(56, 56, 255),    cv::Scalar(151, 157, 255),
        cv::Scalar(31, 112, 255),   cv::Scalar(29, 178, 255),
        cv::Scalar(49, 210, 207),   cv::Scalar(10, 249, 72),
        cv::Scalar(23, 204, 146),   cv::Scalar(134, 219, 61),
        cv::Scalar(52, 147, 26),    cv::Scalar(187, 212, 0),
        cv::Scalar(168, 153, 44),   cv::Scalar(255, 194, 0),
        cv::Scalar(147, 69, 52),    cv::Scalar(255, 115, 100),
        cv::Scalar(236, 24, 0),     cv::Scalar(255, 56, 132),
        cv::Scalar(133, 0, 82),     cv::Scalar(255, 56, 203),
        cv::Scalar(200, 149, 255),  cv::Scalar(199, 55, 255)
    };
    return COLORS;
}

// ============================================================================
// Class Names Loading
// ============================================================================

std::vector<std::string> loadClassNames(const std::string& filepath) {
    std::vector<std::string> names;
    std::ifstream file(filepath);
    if (!file.is_open()) {
        return names;
    }
    
    std::string line;
    while (std::getline(file, line)) {
        while (!line.empty() && (line.back() == '\r' || line.back() == '\n' || line.back() == ' ')) {
            line.pop_back();
        }
        if (!line.empty()) {
            names.push_back(line);
        }
    }
    std::cout << "Loaded default labels from " << filepath << std::endl;
    return names;
}

// ============================================================================
// Preprocessing Utilities
// ============================================================================

/**
 * @brief Preprocess image with resize or letterbox
 */
cv::Mat preprocessImage(const cv::Mat& img, int input_h, int input_w,
                                float& x_scale, float& y_scale,
                                int& x_shift, int& y_shift,
                                bool letterbox = true) {
    cv::Mat result;
    
    if (letterbox) {
        x_scale = std::min(1.0f * input_h / img.rows, 1.0f * input_w / img.cols);
        y_scale = x_scale;
        int new_w = static_cast<int>(img.cols * x_scale);
        int new_h = static_cast<int>(img.rows * y_scale);
        
        x_shift = (input_w - new_w) / 2;
        y_shift = (input_h - new_h) / 2;
        cv::resize(img, result, cv::Size(new_w, new_h));
        cv::copyMakeBorder(result, result, 
                          y_shift, input_h - new_h - y_shift, 
                          x_shift, input_w - new_w - x_shift, 
                          cv::BORDER_CONSTANT, cv::Scalar(127, 127, 127));
    } else {
        cv::resize(img, result, cv::Size(input_w, input_h));
        x_scale = 1.0f * input_w / img.cols;
        y_scale = 1.0f * input_h / img.rows;
        x_shift = 0;
        y_shift = 0;
    }
    
    return result;
}

/**
 * @brief Convert BGR image to NV12 format
 */
cv::Mat bgr2nv12(const cv::Mat& bgr_img) {
    int h = bgr_img.rows, w = bgr_img.cols;
    cv::Mat yuv;
    cv::cvtColor(bgr_img, yuv, cv::COLOR_BGR2YUV_I420);
    cv::Mat nv12(h * 3 / 2, w, CV_8UC1);
    uint8_t* y_ptr = nv12.ptr<uint8_t>();
    uint8_t* uv_ptr = y_ptr + h * w;
    uint8_t* u_src = yuv.ptr<uint8_t>() + h * w;
    uint8_t* v_src = u_src + (h/2) * (w/2);
    
    memcpy(y_ptr, yuv.ptr<uint8_t>(), h * w);
    
    for (int i = 0; i < (h/2) * (w/2); ++i) {
        *uv_ptr++ = *u_src++;
        *uv_ptr++ = *v_src++;
    }

    return nv12;
}

// ============================================================================
// Print Detection Report
// ============================================================================

/**
 * @brief Print detection results
 */
void printDetections(const std::vector<Detection>& detections,
                            const std::vector<std::string>& class_names,
                            float x_scale, float y_scale,
                            int x_shift, int y_shift) {
    std::cout << "\n" << std::string(20, '=') << " Detection Report " 
              << std::string(20, '=') << std::endl;
    std::cout << "Total Objects Found: " << detections.size() << std::endl;
    
    if (detections.empty()) {
        std::cout << "No objects detected." << std::endl;
        return;
    }
    
    std::cout << std::left << std::setw(5) << "ID" 
              << std::setw(16) << "Label"
              << std::setw(9) << "Score"
              << "Box (x1, y1, x2, y2)" << std::endl;
    std::cout << std::string(60, '-') << std::endl;
    
    for (size_t i = 0; i < detections.size(); ++i) {
        const auto& det = detections[i];
        
        int x1 = static_cast<int>((det.bbox.x - x_shift) / x_scale);
        int y1 = static_cast<int>((det.bbox.y - y_shift) / y_scale);
        int x2 = static_cast<int>((det.bbox.x + det.bbox.width - x_shift) / x_scale);
        int y2 = static_cast<int>((det.bbox.y + det.bbox.height - y_shift) / y_scale);
        
        std::string label;
        if (det.class_id < static_cast<int>(class_names.size())) {
            label = class_names[det.class_id];
        } else {
            label = std::to_string(det.class_id);
        }
        
        std::cout << std::left << std::setw(5) << i
                  << std::setw(16) << label
                  << std::fixed << std::setprecision(2) << std::setw(9) << det.confidence
                  << "[" << x1 << ", " << y1 << ", " << x2 << ", " << y2 << "]"
                  << std::endl;
    }
    
    std::cout << std::string(58, '=') << std::endl;
}

// ============================================================================
// Visualization Utilities
// ============================================================================

/**
 * @brief Draw detection results on image
 */
void drawDetections(cv::Mat& img, 
                           const std::vector<Detection>& detections,
                           const std::vector<std::string>& class_names,
                           float x_scale, float y_scale, 
                           int x_shift, int y_shift,
                           double font_scale = 0.6,
                           int font_thickness = 2,
                           int box_thickness = 2) {
    const auto& colors = getDefaultColors();

    for (const auto& det : detections) {
        float x1 = (det.bbox.x - x_shift) / x_scale;
        float y1 = (det.bbox.y - y_shift) / y_scale;
        float x2 = x1 + det.bbox.width / x_scale;
        float y2 = y1 + det.bbox.height / y_scale;

        x1 = std::max(0.0f, std::min(x1, static_cast<float>(img.cols)));
        y1 = std::max(0.0f, std::min(y1, static_cast<float>(img.rows)));
        x2 = std::max(0.0f, std::min(x2, static_cast<float>(img.cols)));
        y2 = std::max(0.0f, std::min(y2, static_cast<float>(img.rows)));

        cv::Scalar color = colors[det.class_id % colors.size()];

        cv::rectangle(img, cv::Point(x1, y1), cv::Point(x2, y2), color, box_thickness);

        std::string label;
        if (det.class_id < static_cast<int>(class_names.size())) {
            label = class_names[det.class_id];
        } else {
            label = "class_" + std::to_string(det.class_id);
        }
        label += ": " + std::to_string(static_cast<int>(det.confidence * 100)) + "%";

        int baseline = 0;
        cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX,
                                             font_scale, font_thickness, &baseline);

        int label_y = std::max(static_cast<int>(y1), text_size.height + 10);
        cv::rectangle(img,
                     cv::Point(x1, label_y - text_size.height - 10),
                     cv::Point(x1 + text_size.width, label_y),
                     color, cv::FILLED);

        cv::putText(img, label, cv::Point(x1, label_y - 5),
                   cv::FONT_HERSHEY_SIMPLEX, font_scale,
                   cv::Scalar(255, 255, 255), font_thickness, cv::LINE_AA);
    }
}

// ============================================================================
// YOLO26 Anchor-Free Box Decoding
// ============================================================================

/**
 * @brief Sigmoid activation function
 */
float sigmoid(float x) {
    return 1.0f / (1.0f + std::exp(-x));
}

/**
 * @brief Calculate logit threshold from sigmoid threshold
 * 
 * sigmoid(x) >= thres  <=>  x >= -ln(1/thres - 1)
 */
float calcLogitThreshold(float thres) {
    float safe_thres = std::max(1e-6f, std::min(thres, 1.0f - 1e-6f));
    return -std::log(1.0f / safe_thres - 1.0f);
}

// ============================================================================
// Main Inference Pipeline
// ============================================================================

int main(int argc, char** argv) {
    std::string project_root = getProjectRoot();
    
    std::string model_path = "yolo26n_detect.hbm";
    std::string img_path = project_root + "/resource/assets/bus.jpg";
    std::string save_path = "yolo26_detect_result.jpg";
    std::string class_names_path = getDefaultLabelPath(project_root, "detect");

    if (argc >= 2) model_path = argv[1];
    if (argc >= 3) img_path = argv[2];
    if (argc >= 4) save_path = argv[3];
    if (argc >= 5) class_names_path = argv[4];

    std::vector<std::string> class_names = loadClassNames(class_names_path);
    if (class_names.empty()) {
        LOG_ERROR("Failed to load class names from: " << class_names_path);
        return -1;
    }

    auto start_load = std::chrono::high_resolution_clock::now();

    hbDNNPackedHandle_t packed_handle;
    const char* model_fn = model_path.c_str();
    CHECK_SUCCESS(hbDNNInitializeFromFiles(&packed_handle, &model_fn, 1), "Model init failed");
    
    const char** model_names;
    int model_count = 0;
    CHECK_SUCCESS(hbDNNGetModelNameList(&model_names, &model_count, packed_handle), "Get model name failed");
    
    hbDNNHandle_t dnn_handle;
    CHECK_SUCCESS(hbDNNGetModelHandle(&dnn_handle, packed_handle, model_names[0]), "Get handle failed");

    auto dur_load = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::high_resolution_clock::now() - start_load).count() / 1000.0;

    hbDNNTensorProperties in_props;
    CHECK_SUCCESS(hbDNNGetInputTensorProperties(&in_props, dnn_handle, 0), "Get input props failed");
    
    int input_h = in_props.validShape.dimensionSize[1];
    int input_w = in_props.validShape.dimensionSize[2];
    
    if (input_h <= 3 && in_props.validShape.dimensionSize[2] > 3) {
        input_h = in_props.validShape.dimensionSize[2];
        input_w = in_props.validShape.dimensionSize[3];
    }

    cv::Mat img = cv::imread(img_path);
    if (img.empty()) { 
        LOG_ERROR("Failed to load image: " << img_path);
        return -1; 
    }
    
    auto start_pre = std::chrono::high_resolution_clock::now();
    
    float x_scale, y_scale;
    int x_shift, y_shift;
    bool use_letterbox = (PREPROCESS_TYPE == LETTERBOX_TYPE);
    cv::Mat preprocessed_img = preprocessImage(img, input_h, input_w, 
                                                     x_scale, y_scale, x_shift, y_shift, 
                                                     use_letterbox);
    cv::Mat nv12_img_full = bgr2nv12(preprocessed_img);

    auto dur_pre = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::high_resolution_clock::now() - start_pre).count() / 1000.0;

    int32_t input_count = 0;
    CHECK_SUCCESS(hbDNNGetInputCount(&input_count, dnn_handle), "Get input count failed");

    std::vector<hbDNNTensor> input_tensors(input_count);

    for (int i = 0; i < input_count; ++i) {
        CHECK_SUCCESS(hbDNNGetInputTensorProperties(&input_tensors[i].properties, dnn_handle, i), 
                      "Get props failed");
        int data_size = 0;
        
        if (i == 0) {
            data_size = input_h * input_w;
            input_tensors[i].properties.validShape.dimensionSize[0] = 1;
            input_tensors[i].properties.validShape.dimensionSize[1] = input_h;
            input_tensors[i].properties.validShape.dimensionSize[2] = input_w;
            input_tensors[i].properties.validShape.dimensionSize[3] = 1;
            input_tensors[i].properties.stride[3] = 1;
            input_tensors[i].properties.stride[2] = 1;
            input_tensors[i].properties.stride[1] = input_w;
            input_tensors[i].properties.stride[0] = input_h * input_w;
        } else {
            int uv_h = input_h / 2;
            int uv_w = input_w / 2;
            data_size = uv_h * uv_w * 2;
            input_tensors[i].properties.validShape.dimensionSize[0] = 1;
            input_tensors[i].properties.validShape.dimensionSize[1] = uv_h;
            input_tensors[i].properties.validShape.dimensionSize[2] = uv_w;
            input_tensors[i].properties.validShape.dimensionSize[3] = 2;
            input_tensors[i].properties.stride[3] = 1;
            input_tensors[i].properties.stride[2] = 2;
            input_tensors[i].properties.stride[1] = uv_w * 2;
            input_tensors[i].properties.stride[0] = uv_h * uv_w * 2;
        }
        
        CHECK_SUCCESS(hbUCPMallocCached(&input_tensors[i].sysMem, data_size, 0), "Malloc failed");
        
        if (i == 0) {
            memcpy(input_tensors[i].sysMem.virAddr, nv12_img_full.data, data_size);
        } else {
            memcpy(input_tensors[i].sysMem.virAddr, 
                   nv12_img_full.data + input_h * input_w, data_size);
        }
        
        hbUCPMemFlush(&input_tensors[i].sysMem, HB_SYS_MEM_CACHE_CLEAN);
    }

    int output_cnt = 0;
    hbDNNGetOutputCount(&output_cnt, dnn_handle);
    
    std::vector<hbDNNTensor> outputs(output_cnt);
    for (int i = 0; i < output_cnt; ++i) {
        hbDNNGetOutputTensorProperties(&outputs[i].properties, dnn_handle, i);
        hbUCPMallocCached(&outputs[i].sysMem, outputs[i].properties.alignedByteSize, 0);
    }

    auto start_infer = std::chrono::high_resolution_clock::now();
    
    hbUCPTaskHandle_t task = nullptr;
    CHECK_SUCCESS(hbDNNInferV2(&task, outputs.data(), input_tensors.data(), dnn_handle), "Infer failed");
    
    hbUCPSchedParam param;
    HB_UCP_INITIALIZE_SCHED_PARAM(&param);
    param.backend = HB_UCP_BPU_CORE_ANY;
    
    CHECK_SUCCESS(hbUCPSubmitTask(task, &param), "Submit failed");
    CHECK_SUCCESS(hbUCPWaitTaskDone(task, 0), "Wait failed");
    
    auto dur_infer = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::high_resolution_clock::now() - start_infer).count() / 1000.0;

    auto start_post = std::chrono::high_resolution_clock::now();
    
    int H_8 = input_h / 8;
    int H_16 = input_h / 16;
    int H_32 = input_h / 32;
    
    int W_8 = input_w / 8;
    int W_16 = input_w / 16;
    int W_32 = input_w / 32;
    
    // Output pairs: [Box_8, Cls_8, Box_16, Cls_16, Box_32, Cls_32]
    int order[6] = {0};
    
    for (int i = 0; i < output_cnt; ++i) {
        hbUCPMemFlush(&outputs[i].sysMem, HB_SYS_MEM_CACHE_INVALIDATE);
        
        int h = outputs[i].properties.validShape.dimensionSize[1];
        int w = outputs[i].properties.validShape.dimensionSize[2];
        int c = outputs[i].properties.validShape.dimensionSize[3];
        
        if (h == H_8 && w == W_8) {
            if (c == 4) order[0] = i;
            else order[1] = i;
        } else if (h == H_16 && w == W_16) {
            if (c == 4) order[2] = i;
            else order[3] = i;
        } else if (h == H_32 && w == W_32) {
            if (c == 4) order[4] = i;
            else order[5] = i;
        }
    }

    std::vector<std::vector<cv::Rect2d>> bboxes(CLASSES_NUM);
    std::vector<std::vector<float>> scores(CLASSES_NUM);
    float conf_thres_raw = calcLogitThreshold(SCORE_THRESHOLD);

    int strides[] = {8, 16, 32};
    for (int s = 0; s < 3; ++s) {
        int box_idx = order[s * 2];
        int cls_idx = order[s * 2 + 1];
        int stride = strides[s];
        int h = input_h / stride;
        int w = input_w / stride;
        
        float* box_ptr = static_cast<float*>(outputs[box_idx].sysMem.virAddr);
        float* cls_ptr = static_cast<float*>(outputs[cls_idx].sysMem.virAddr);

        for (int i = 0; i < h; ++i) {
            for (int j = 0; j < w; ++j) {
                int offset = i * w + j;
                float* cur_cls = cls_ptr + offset * CLASSES_NUM;
                float* cur_box = box_ptr + offset * 4;

                int max_id = 0;
                for (int k = 1; k < CLASSES_NUM; ++k) {
                    if (cur_cls[k] > cur_cls[max_id]) max_id = k;
                }
                
                if (cur_cls[max_id] < conf_thres_raw) continue;

                // Box format: [l, t, r, b]
                // Grid center: (j + 0.5, i + 0.5)
                float l = cur_box[0];
                float t = cur_box[1];
                float r = cur_box[2];
                float b = cur_box[3];
                
                float grid_center_x = j + 0.5f;
                float grid_center_y = i + 0.5f;
                
                float x1 = (grid_center_x - l) * stride;
                float y1 = (grid_center_y - t) * stride;
                float x2 = (grid_center_x + r) * stride;
                float y2 = (grid_center_y + b) * stride;
                
                float score = sigmoid(cur_cls[max_id]);
                
                bboxes[max_id].push_back(cv::Rect2d(x1, y1, x2 - x1, y2 - y1));
                scores[max_id].push_back(score);
            }
        }
    }

    std::vector<Detection> dets;
    for (int c = 0; c < CLASSES_NUM; ++c) {
        if (bboxes[c].empty()) continue;
        std::vector<int> indices;
        cv::dnn::NMSBoxes(bboxes[c], scores[c], SCORE_THRESHOLD, NMS_THRESHOLD, indices);
        for (int idx : indices) {
            dets.emplace_back(c, scores[c][idx], bboxes[c][idx]);
        }
    }
    
    auto dur_post = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::high_resolution_clock::now() - start_post).count() / 1000.0;

    std::cout << "\033[1;31m[Detect] Load Model time = " << std::fixed << std::setprecision(2) << dur_load << " ms\033[0m" << std::endl;
    std::cout << "\033[1;31m[Detect] Pre Process time = " << std::fixed << std::setprecision(2) << dur_pre << " ms\033[0m" << std::endl;
    std::cout << "\033[1;31m[Detect] Forward time = " << std::fixed << std::setprecision(2) << dur_infer << " ms\033[0m" << std::endl;
    std::cout << "\033[1;31m[Detect] Post Process time = " << std::fixed << std::setprecision(2) << dur_post << " ms\033[0m" << std::endl;

    printDetections(dets, class_names, x_scale, y_scale, x_shift, y_shift);

    drawDetections(img, dets, class_names, x_scale, y_scale, x_shift, y_shift,
                        FONT_SCALE, FONT_THICKNESS, BOX_THICKNESS);
    
    cv::imwrite(save_path, img);
    std::cout << "\n[Saved] Result saved to: " << save_path << std::endl;

    hbUCPReleaseTask(task);
    for (auto& t : input_tensors) hbUCPFree(&t.sysMem);
    for (auto& t : outputs) hbUCPFree(&t.sysMem);
    hbDNNRelease(packed_handle);
    
    return 0;
}
