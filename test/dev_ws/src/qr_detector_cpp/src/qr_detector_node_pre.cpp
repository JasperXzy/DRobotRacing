#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <opencv2/opencv.hpp>
#include <zbar.h>
#include <origincar_msg/msg/sign.hpp>
#include <memory>
#include <vector>
#include <algorithm>
#include <filesystem>

class QRDetectorNode : public rclcpp::Node
{
public:
    QRDetectorNode() : Node("qr_detector_node")
    {
        // 声明参数
        this->declare_parameter("image_topic", "/camera/image");
        this->declare_parameter("process_interval", 0.05);
        this->declare_parameter("scale_factor", 1.0);
        this->declare_parameter("stop_on_detect", true);

        std::string image_topic = this->get_parameter("image_topic").as_string();
        process_interval_ = this->get_parameter("process_interval").as_double();
        scale_factor_ = this->get_parameter("scale_factor").as_double();
        stop_on_detect_ = this->get_parameter("stop_on_detect").as_bool();

        // 初始化 zbar 扫描器
        scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
        scanner_.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);
        scanner_.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_X_DENSITY, 1);
        scanner_.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_Y_DENSITY, 1);

        last_fps_time_ = this->now();
        last_process_time_ = this->now();
        cooldown_until_ = this->now();

        // ★ 启动即持续扫描，不需要等激活信号 ★
        activated_ = true;

        // ---- 发布者 ----
        sign_switch_pub_ = this->create_publisher<origincar_msg::msg::Sign>("/sign_switch", 10);
        publisher_ = this->create_publisher<std_msgs::msg::String>("qr_code", 10);
        qr_info_pub_ = this->create_publisher<std_msgs::msg::String>("qr_info", 10);
        tts_pub_ = this->create_publisher<std_msgs::msg::String>("/tts_text", 10);

        // ---- 图像订阅（持续处理） ----
        image_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            image_topic, rclcpp::SensorDataQoS(),
            std::bind(&QRDetectorNode::image_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(),
            "QR检测器已启动 (持续扫描模式, 话题=%s, 缩放=%.1fx, 间隔=%.2fs)",
            image_topic.c_str(), scale_factor_, process_interval_);
    }

private:
    void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
    {
        if (!activated_) return;

        auto current_time = this->now();
        if ((current_time - last_process_time_).seconds() < process_interval_) return;

        frame_count_++;

        double elapsed_sec = (current_time - last_fps_time_).seconds();
        if (elapsed_sec >= 1.0)
        {
            double fps = frame_count_ / elapsed_sec;
            RCLCPP_INFO(this->get_logger(), "扫描帧率: %.1f FPS", fps);
            frame_count_ = 0;
            last_fps_time_ = current_time;
        }

        try
        {
            // JPEG 解码
            std::vector<unsigned char> jpeg_data(msg->data.begin(), msg->data.end());
            cv::Mat frame = cv::imdecode(jpeg_data, cv::IMREAD_COLOR);
            if (frame.empty())
            {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "JPEG解码失败，数据大小=%zu", msg->data.size());
                last_process_time_ = current_time;
                return;
            }

            // ★ 缩小到 0.5x 加速 zbar（960×540）
            cv::Mat scan_frame;
            cv::resize(frame, scan_frame, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR);

            // ---- 转灰度 ----
            cv::Mat gray;
            cv::cvtColor(scan_frame, gray, cv::COLOR_BGR2GRAY);

            // ---- zbar 全图扫描 ----
            int width = gray.cols;
            int height = gray.rows;

            zbar::Image zbar_image(width, height, "Y800", gray.data, width * height);

            int result = scanner_.scan(zbar_image);

            if (result > 0)
            {
                for (zbar::Image::SymbolIterator symbol = zbar_image.symbol_begin();
                     symbol != zbar_image.symbol_end(); ++symbol)
                {
                    if (symbol->get_type() != zbar::ZBAR_QRCODE) continue;

                    std::string barcodeData = symbol->get_data();
                    RCLCPP_INFO(this->get_logger(), "检测到二维码: %s", barcodeData.c_str());


                        
                        
                    

                    // 发布原始数据
                    auto qr_msg = std::make_unique<std_msgs::msg::String>();
                    qr_msg->data = barcodeData;
                    publisher_->publish(std::move(qr_msg));

                    // 发布识别信息
                    auto qr_info = std::make_unique<std_msgs::msg::String>();
                    qr_info->data = "二维码内容:" + barcodeData;
                    qr_info_pub_->publish(std::move(qr_info));

                    // 纯数字 → 转向信号 + TTS播报
                    if (std::all_of(barcodeData.begin(), barcodeData.end(), ::isdigit))
                    {
                        int qr_value = std::stoi(barcodeData);
                        origincar_msg::msg::Sign sign_msg;
                        sign_msg.sign_data = (qr_value % 2 == 1) ? 3 : 4;
                        sign_switch_pub_->publish(sign_msg);
                        RCLCPP_INFO(this->get_logger(),
                            "发布转向信号: %d (QR: %s → %s)",
                            sign_msg.sign_data, barcodeData.c_str(),
                            (sign_msg.sign_data == 3) ? "顺时针" : "逆时针");

                        // ★ TTS语音播报方向（只报一次） ★
                        if (!tts_spoken_)
                        {
                            auto tts_msg = std::make_unique<std_msgs::msg::String>();
                            tts_msg->data = (sign_msg.sign_data == 3) ? "扫描到顺时针" : "扫描到逆时针";
                            tts_pub_->publish(std::move(tts_msg));
                            tts_spoken_ = true;
                            RCLCPP_INFO(this->get_logger(),
                                "TTS播报: %s", (sign_msg.sign_data == 3) ? "扫描到顺时针" : "扫描到逆时针");
                        }

                        // ★ 扫到后停止扫描并退出节点 ★
                        if (stop_on_detect_)
                        {
                            activated_ = false;
                            RCLCPP_INFO(this->get_logger(), "QR扫描完成，停止扫描（保持其他节点运行）");
                            return;
                        }

                        // 加冷却时间，防止同一二维码重复触发
                        cooldown_until_ = current_time + rclcpp::Duration::from_seconds(3.0);
                        last_process_time_ = current_time;
                        return;
                    }
                }
            }

            // 冷却期内跳过扫描（防止同一二维码重复识别）
            if (current_time < cooldown_until_)
            {
                last_process_time_ = current_time;
                return;
            }
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "处理图像异常: %s", e.what());
        }

        last_process_time_ = current_time;
    }

    // ---- 成员变量 ----
    size_t frame_count_ = 0;
    rclcpp::Time last_fps_time_;
    rclcpp::Time last_process_time_;
    rclcpp::Time cooldown_until_;

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_;

    rclcpp::Publisher<origincar_msg::msg::Sign>::SharedPtr sign_switch_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr qr_info_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tts_pub_;

    bool activated_;
    bool tts_spoken_ = false;
    bool stop_on_detect_;
    double process_interval_;
    double scale_factor_;
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
