#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <zbar.h>
#include <origincar_msg/msg/sign.hpp>
#include <algorithm>
#include <cctype>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <string>

class QRDetectorNode : public rclcpp::Node
{
public:
    QRDetectorNode() : Node("qr_detector_node")
    {
        this->declare_parameter("image_topic", "/camera/qr_roi/image");
        this->declare_parameter("process_interval", 0.0);
        this->declare_parameter("stop_on_detect", true);
        this->declare_parameter("zbar_density", 3);
        this->declare_parameter("small_roi_upscale", true);
        this->declare_parameter("small_roi_min_size", 120);
        this->declare_parameter("small_roi_upscale_factor", 2.0);
        this->declare_parameter("speak_on_detect", false);
        this->declare_parameter("suspend_visual_control_on_detect", true);
        this->declare_parameter("visual_control_suspend_topic", "/sign4return");
        this->declare_parameter("visual_control_suspend_value", 5);
        this->declare_parameter("save_detected_camera_frame", true);
        this->declare_parameter("camera_image_topic", "/camera/image");
        this->declare_parameter(
            "qr_frame_output_root",
            "/root/D_Robot/test/dev_ws/src/origincar_system/path");
        this->declare_parameter("qr_frame_cache_size", 60);
        this->declare_parameter("qr_frame_match_tolerance", 0.20);

        image_topic_ = this->get_parameter("image_topic").as_string();
        process_interval_ = std::max(0.0, this->get_parameter("process_interval").as_double());
        stop_on_detect_ = this->get_parameter("stop_on_detect").as_bool();
        speak_on_detect_ = this->get_parameter("speak_on_detect").as_bool();
        suspend_visual_control_on_detect_ =
            this->get_parameter("suspend_visual_control_on_detect").as_bool();
        visual_control_suspend_topic_ =
            this->get_parameter("visual_control_suspend_topic").as_string();
        visual_control_suspend_value_ =
            static_cast<int>(this->get_parameter("visual_control_suspend_value").as_int());
        save_detected_camera_frame_ =
            this->get_parameter("save_detected_camera_frame").as_bool();
        camera_image_topic_ = this->get_parameter("camera_image_topic").as_string();
        qr_frame_output_root_ = this->get_parameter("qr_frame_output_root").as_string();
        const int64_t requested_cache_size =
            this->get_parameter("qr_frame_cache_size").as_int();
        qr_frame_cache_size_ = static_cast<std::size_t>(
            std::max<int64_t>(1, requested_cache_size));
        qr_frame_match_tolerance_ = std::max(
            0.0, this->get_parameter("qr_frame_match_tolerance").as_double());
        auto requested_density = static_cast<int>(this->get_parameter("zbar_density").as_int());
        zbar_density_ = std::clamp(requested_density, 1, 5);
        small_roi_upscale_ = this->get_parameter("small_roi_upscale").as_bool();
        small_roi_min_size_ = std::max(1, static_cast<int>(this->get_parameter("small_roi_min_size").as_int()));
        small_roi_upscale_factor_ = std::clamp(
            this->get_parameter("small_roi_upscale_factor").as_double(), 1.0, 4.0);

        scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
        scanner_.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);
        scanner_.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_X_DENSITY, zbar_density_);
        scanner_.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_Y_DENSITY, zbar_density_);

        last_fps_time_ = this->now();
        last_process_time_ = this->now();
        cooldown_until_ = this->now();

        sign_switch_pub_ = this->create_publisher<origincar_msg::msg::Sign>("/sign_switch", 10);
        visual_control_suspend_pub_ =
            this->create_publisher<std_msgs::msg::Int32>(visual_control_suspend_topic_, 10);
        publisher_ = this->create_publisher<std_msgs::msg::String>("qr_code", 10);
        qr_info_pub_ = this->create_publisher<std_msgs::msg::String>("qr_info", 10);
        tts_pub_ = this->create_publisher<std_msgs::msg::String>("/tts_text", 10);

        auto image_qos = rclcpp::SensorDataQoS();
        image_qos.keep_last(1);
        image_qos.best_effort();
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            image_topic_, image_qos,
            std::bind(&QRDetectorNode::image_callback, this, std::placeholders::_1));
        if (save_detected_camera_frame_)
        {
            camera_image_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
                camera_image_topic_, image_qos,
                std::bind(
                    &QRDetectorNode::camera_image_callback,
                    this,
                    std::placeholders::_1));
        }

        RCLCPP_INFO(this->get_logger(),
            "QR detector started (ROI mono8 mode, topic=%s, density=%d, interval=%.3fs)",
            image_topic_.c_str(), zbar_density_, process_interval_);
    }

private:
    static int64_t stamp_to_nanoseconds(const builtin_interfaces::msg::Time& stamp)
    {
        return static_cast<int64_t>(stamp.sec) * 1000000000LL +
            static_cast<int64_t>(stamp.nanosec);
    }

    static bool is_session_directory_name(const std::string& name)
    {
        if (name.size() != 19 || name[4] != '.' || name[7] != '.' ||
            name[10] != '_' || name[13] != ':' || name[16] != ':') {
            return false;
        }
        for (std::size_t index = 0; index < name.size(); ++index) {
            if (index == 4 || index == 7 || index == 10 || index == 13 || index == 16) {
                continue;
            }
            if (!std::isdigit(static_cast<unsigned char>(name[index]))) {
                return false;
            }
        }
        return true;
    }

    void camera_image_callback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg)
    {
        camera_frames_.push_back(msg);
        while (camera_frames_.size() > qr_frame_cache_size_) {
            camera_frames_.pop_front();
        }
    }

    std::filesystem::path current_output_directory() const
    {
        const std::filesystem::path output_root(qr_frame_output_root_);
        std::error_code error;
        std::filesystem::create_directories(output_root, error);
        if (error) {
            return {};
        }

        std::string latest_session;
        for (const auto& entry : std::filesystem::directory_iterator(output_root, error)) {
            if (error) {
                return {};
            }
            if (!entry.is_directory(error) || error) {
                error.clear();
                continue;
            }
            const std::string name = entry.path().filename().string();
            if (is_session_directory_name(name) && name > latest_session) {
                latest_session = name;
            }
        }
        return latest_session.empty() ? output_root : output_root / latest_session;
    }

    void save_detected_camera_frame(
        const builtin_interfaces::msg::Time& detection_stamp)
    {
        if (!save_detected_camera_frame_) {
            return;
        }
        if (camera_frames_.empty()) {
            RCLCPP_ERROR(this->get_logger(),
                "Cannot save QR camera frame: no frame received from %s",
                camera_image_topic_.c_str());
            return;
        }

        const int64_t target_nanoseconds = stamp_to_nanoseconds(detection_stamp);
        sensor_msgs::msg::CompressedImage::ConstSharedPtr matched_frame = camera_frames_.back();
        int64_t best_difference = std::numeric_limits<int64_t>::max();
        if (target_nanoseconds > 0) {
            for (const auto& frame : camera_frames_) {
                const int64_t frame_nanoseconds = stamp_to_nanoseconds(frame->header.stamp);
                if (frame_nanoseconds <= 0) {
                    continue;
                }
                const int64_t difference = frame_nanoseconds >= target_nanoseconds ?
                    frame_nanoseconds - target_nanoseconds : target_nanoseconds - frame_nanoseconds;
                if (difference < best_difference) {
                    best_difference = difference;
                    matched_frame = frame;
                }
            }
            const int64_t tolerance_nanoseconds = static_cast<int64_t>(
                qr_frame_match_tolerance_ * 1000000000.0);
            if (best_difference == std::numeric_limits<int64_t>::max() ||
                best_difference > tolerance_nanoseconds) {
                RCLCPP_ERROR(this->get_logger(),
                    "Cannot save QR camera frame: nearest full frame differs by %.3fs (limit %.3fs)",
                    best_difference == std::numeric_limits<int64_t>::max() ?
                    -1.0 : static_cast<double>(best_difference) / 1000000000.0,
                    qr_frame_match_tolerance_);
                return;
            }
        }

        const auto output_directory = current_output_directory();
        if (output_directory.empty()) {
            RCLCPP_ERROR(this->get_logger(),
                "Cannot create QR camera frame output directory: %s",
                qr_frame_output_root_.c_str());
            return;
        }

        std::string format = matched_frame->format;
        std::transform(format.begin(), format.end(), format.begin(), [](unsigned char character) {
            return static_cast<char>(std::tolower(character));
        });
        const std::string extension = format.find("png") != std::string::npos ? ".png" : ".jpg";
        const std::filesystem::path output_path =
            output_directory / ("05_QR_camera" + extension);

        std::ofstream output(output_path, std::ios::binary | std::ios::trunc);
        output.write(
            reinterpret_cast<const char*>(matched_frame->data.data()),
            static_cast<std::streamsize>(matched_frame->data.size()));
        output.close();
        if (!output) {
            RCLCPP_ERROR(this->get_logger(),
                "Failed to save QR camera frame: %s", output_path.c_str());
        }
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (!activated_) return;

        auto current_time = this->now();
        if (process_interval_ > 0.0 &&
            (current_time - last_process_time_).seconds() < process_interval_) {
            return;
        }
        if (current_time < cooldown_until_) {
            return;
        }

        frame_count_++;
        double elapsed_sec = (current_time - last_fps_time_).seconds();
        if (elapsed_sec >= 1.0)
        {
            double fps = frame_count_ / elapsed_sec;
            RCLCPP_INFO(this->get_logger(), "ROI scan rate: %.1f FPS", fps);
            frame_count_ = 0;
            last_fps_time_ = current_time;
        }

        cv_bridge::CvImageConstPtr mono_ptr;
        cv_bridge::CvImagePtr converted_ptr;
        cv::Mat gray;
        try
        {
            if (msg->encoding == sensor_msgs::image_encodings::MONO8 ||
                msg->encoding == sensor_msgs::image_encodings::TYPE_8UC1) {
                mono_ptr = cv_bridge::toCvShare(msg, msg->encoding);
                gray = mono_ptr->image;
            } else {
                converted_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
                gray = converted_ptr->image;
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                    "Expected mono8 ROI input, got encoding=%s; converted for this frame",
                    msg->encoding.c_str());
            }
        }
        catch (const cv_bridge::Exception& e)
        {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "cv_bridge conversion failed: %s", e.what());
            last_process_time_ = current_time;
            return;
        }

        if (gray.empty() || gray.cols <= 0 || gray.rows <= 0) {
            last_process_time_ = current_time;
            return;
        }

        cv::Mat scan_image;
        const int min_side = std::min(gray.cols, gray.rows);
        if (small_roi_upscale_ &&
            min_side > 0 &&
            min_side < small_roi_min_size_ &&
            small_roi_upscale_factor_ > 1.0) {
            cv::resize(gray, scan_image, cv::Size(), small_roi_upscale_factor_,
                small_roi_upscale_factor_, cv::INTER_LINEAR);
        } else {
            scan_image = gray;
        }

        if (!scan_image.isContinuous()) {
            scan_image = scan_image.clone();
        }

        const int width = scan_image.cols;
        const int height = scan_image.rows;
        zbar::Image zbar_image(width, height, "Y800", scan_image.data, width * height);

        int result = scanner_.scan(zbar_image);
        if (result > 0)
        {
            for (zbar::Image::SymbolIterator symbol = zbar_image.symbol_begin();
                 symbol != zbar_image.symbol_end(); ++symbol)
            {
                if (symbol->get_type() != zbar::ZBAR_QRCODE) continue;

                std::string barcode_data = symbol->get_data();
                RCLCPP_INFO(this->get_logger(), "QR code detected: %s", barcode_data.c_str());
                save_detected_camera_frame(msg->header.stamp);
                publish_qr_result(barcode_data, current_time);
                last_process_time_ = current_time;
                return;
            }
        }

        last_process_time_ = current_time;
    }

    void publish_qr_result(const std::string& barcode_data, const rclcpp::Time& current_time)
    {
        auto qr_msg = std::make_unique<std_msgs::msg::String>();
        qr_msg->data = barcode_data;
        publisher_->publish(std::move(qr_msg));

        auto qr_info = std::make_unique<std_msgs::msg::String>();
        qr_info->data = "QR content:" + barcode_data;
        qr_info_pub_->publish(std::move(qr_info));

        bool numeric = !barcode_data.empty() &&
            std::all_of(barcode_data.begin(), barcode_data.end(), [](unsigned char c) {
                return std::isdigit(c);
            });
        if (!numeric) {
            return;
        }

        const bool is_odd = ((barcode_data.back() - '0') % 2) == 1;
        const char* parity_text = is_odd ? "奇数" : "偶数";
        const char* direction_text = is_odd ? "顺时针" : "逆时针";
        origincar_msg::msg::Sign sign_msg;
        sign_msg.sign_data = is_odd ? 3 : 4;

        publish_visual_control_suspend();
        sign_switch_pub_->publish(sign_msg);

        RCLCPP_INFO(this->get_logger(), "Published direction signal: %d (QR: %s -> %s)",
            sign_msg.sign_data, barcode_data.c_str(),
            is_odd ? "clockwise" : "counterclockwise");

        if (speak_on_detect_ && !tts_spoken_)
        {
            auto tts_msg = std::make_unique<std_msgs::msg::String>();
            tts_msg->data = "数字：" + barcode_data + "，为" + parity_text +
                "，进行" + direction_text + "绕行";
            tts_pub_->publish(std::move(tts_msg));
            tts_spoken_ = true;
        }

        if (stop_on_detect_)
        {
            activated_ = false;
            RCLCPP_INFO(this->get_logger(), "QR scan complete; stopping QR scan");
            return;
        }

        cooldown_until_ = current_time + rclcpp::Duration::from_seconds(3.0);
    }

    void publish_visual_control_suspend()
    {
        if (!suspend_visual_control_on_detect_ || !visual_control_suspend_pub_)
        {
            return;
        }

        auto suspend_msg = std::make_unique<std_msgs::msg::Int32>();
        suspend_msg->data = visual_control_suspend_value_;
        visual_control_suspend_pub_->publish(std::move(suspend_msg));
        RCLCPP_INFO(this->get_logger(), "Published visual_control suspend signal: %s=%d",
            visual_control_suspend_topic_.c_str(), visual_control_suspend_value_);
    }

    std::string image_topic_;
    std::string camera_image_topic_;
    std::string qr_frame_output_root_;
    std::string visual_control_suspend_topic_;
    size_t frame_count_ = 0;
    rclcpp::Time last_fps_time_;
    rclcpp::Time last_process_time_;
    rclcpp::Time cooldown_until_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr camera_image_sub_;
    rclcpp::Publisher<origincar_msg::msg::Sign>::SharedPtr sign_switch_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr visual_control_suspend_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr qr_info_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tts_pub_;

    bool activated_ = true;
    bool save_detected_camera_frame_ = true;
    bool tts_spoken_ = false;
    bool speak_on_detect_ = false;
    bool stop_on_detect_ = true;
    bool suspend_visual_control_on_detect_ = true;
    bool small_roi_upscale_ = true;
    int zbar_density_ = 3;
    int small_roi_min_size_ = 120;
    int visual_control_suspend_value_ = 5;
    std::size_t qr_frame_cache_size_ = 60;
    double process_interval_ = 0.0;
    double qr_frame_match_tolerance_ = 0.20;
    double small_roi_upscale_factor_ = 2.0;
    std::deque<sensor_msgs::msg::CompressedImage::ConstSharedPtr> camera_frames_;
    zbar::ImageScanner scanner_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<QRDetectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
