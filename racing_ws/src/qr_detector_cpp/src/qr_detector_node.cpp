#include <rclcpp/rclcpp.hpp> 
#include <sensor_msgs/msg/image.hpp> 
#include <std_msgs/msg/string.hpp> 
#include <std_msgs/msg/int32.hpp> 
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp> 
#include <zbar.h>
#include <origincar_msg/msg/sign.hpp> 
#include <ai_msgs/msg/perception_targets.hpp> 
#include <memory>
#include <vector>
#include <algorithm>
 
class QRDetectorNode : public rclcpp::Node 
{
public:
    QRDetectorNode() : Node("qr_detector_node")
    {
        // 初始化扫描器
        scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
        scanner_.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);
        // 设置扫描密度（值越大越密集，但可能更慢）
        scanner_.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_X_DENSITY, 3);  // 水平方向密度 
        scanner_.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_Y_DENSITY, 3);
        last_fps_time_ = this->now();
        // 创建订阅者和发布者 
        sign_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/sign4return", 10, std::bind(&QRDetectorNode::sign_callback, this, std::placeholders::_1));
        sign_switch_pub_ = this->create_publisher<origincar_msg::msg::Sign>("/sign_switch", 10);
        publisher_ = this->create_publisher<std_msgs::msg::String>("qr_code", 10);
        qr_info_pub_ = this->create_publisher<std_msgs::msg::String>("qr_info", 1);
        sign4return_pub_ = this->create_publisher<std_msgs::msg::Int32>("/sign4return", 10);
        qr_sub_ = this->create_subscription<ai_msgs::msg::PerceptionTargets>(
            "racing_obstacle_detection", 10, std::bind(&QRDetectorNode::target_callback, this, std::placeholders::_1));
 
        activated_ = false;
        last_sign_time_ = this->now();
        last_process_time_ = this->now();
        process_interval_ = 0.03;  // 约33Hz 
        scale_factor_ = 1;
 
        // 预分配内存
        qr_codes_.reserve(10);
 
        RCLCPP_INFO(this->get_logger(), "QR检测器节点已启动 (等待/sign4return信号)");
    }
 
private:

    void sign_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        auto current_time = this->now();
        last_sign_time_ = current_time;
 
        if (msg->data == 0 && !activated_)
        {
            RCLCPP_INFO(this->get_logger(), "收到激活信号(0)，启动二维码检测");
            activated_ = true;
 
            if (!image_sub_)
            {
                image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                    "image_raw", rclcpp::SensorDataQoS(), 
                    std::bind(&QRDetectorNode::image_callback, this, std::placeholders::_1));
                RCLCPP_INFO(this->get_logger(), "已创建图像订阅");
            }
        }
    }
 
    void target_callback(const ai_msgs::msg::PerceptionTargets::SharedPtr msg)
    {
        qr_codes_.clear();
        
        for (const auto& target : msg->targets)
        {
            const auto& roi = target.rois[0]; 
            if (roi.confidence  >= 0.6)
            {
                qr_codes_.emplace_back(std::array<unsigned int, 4>{
                    static_cast<unsigned int>(roi.rect.x_offset), 
                    static_cast<unsigned int>(roi.rect.y_offset), 
                    static_cast<unsigned int>(roi.rect.width), 
                    static_cast<unsigned int>(roi.rect.height) 
                });
            }
        }
    }
 
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (!activated_) return;
 
        auto current_time = this->now();
        if ((current_time - last_process_time_).seconds() < process_interval_) return;
        frame_count_++;
        // 每秒计算一次 FPS 
        double elapsed_sec = (current_time - last_fps_time_).seconds();
        if (elapsed_sec >= 1.0)  // 每 1 秒输出一次 FPS
        {
            double fps = frame_count_ / elapsed_sec;
            RCLCPP_INFO(this->get_logger(), "当前扫描帧率: %.2f FPS", fps);
            frame_count_ = 0;  // 重置计数 
            last_fps_time_ = current_time;
        }
        try
        {
            cv_bridge::CvImageConstPtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
            } catch (cv_bridge::Exception& e) {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what()); 
                return;
            }
 
            const cv::Mat& cv_image = cv_ptr->image;
            
            // 使用普通循环替代并行处理 
            for (const auto& qr_code : qr_codes_)
            {
                process_qr_region(cv_image, qr_code);
            }
 
            if ((current_time - last_sign_time_).seconds() > 5)
            {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "扫描中...");
            }
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "处理图像时出错: %s", e.what()); 
        }
 
        last_process_time_ = current_time;
    }
 
    void process_qr_region(const cv::Mat& cv_image, const std::array<unsigned int, 4>& qr_code)
    {
        // 计算感兴趣区域（ROI）
        int x = std::max(static_cast<int>(qr_code[0]) - 20, 0);
        int y = std::max(static_cast<int>(qr_code[1]) - 20, 0);
        int width = std::min(static_cast<int>(qr_code[2]) + 40, cv_image.cols  - x);
        int height = std::min(static_cast<int>(qr_code[3]) + 40, cv_image.rows  - y);
        
        if (width <= 0 || height <= 0) return;
 
        cv::Mat roi = cv_image(cv::Rect(x, y, width, height));
        
        // 转换为灰度图 
        cv::Mat gray;
        cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
        
        // 创建zbar图像
        zbar::Image zbar_image(roi.cols,  roi.rows,  "Y800", gray.data,  roi.cols  * roi.rows); 
 
        // 扫描二维码 
        int result = scanner_.scan(zbar_image);
        if (result > 0)
        {
            for (zbar::Image::SymbolIterator symbol = zbar_image.symbol_begin();  
                 symbol != zbar_image.symbol_end();  ++symbol)
            {
                if (symbol->get_type() == zbar::ZBAR_QRCODE)
                {
                    std::string barcodeData = symbol->get_data();
                    RCLCPP_INFO(this->get_logger(), "检测到二维码: %s", barcodeData.c_str());
 
                    // 发布消息 
                    auto qr_msg = std::make_unique<std_msgs::msg::String>();
                    qr_msg->data = barcodeData;
                    publisher_->publish(std::move(qr_msg));
 
                    auto qr_res = std::make_unique<std_msgs::msg::String>();
                    qr_res->data = "二维码内容为:" + barcodeData;
                    qr_info_pub_->publish(std::move(qr_res));
 
                    if (std::all_of(barcodeData.begin(),  barcodeData.end(),  ::isdigit))
                    {
                        int qr_value = std::stoi(barcodeData);
                        origincar_msg::msg::Sign sign_msg;
                        sign_msg.sign_data  = (qr_value % 2 == 1) ? 3 : 4;
                        sign_switch_pub_->publish(sign_msg);
                        RCLCPP_INFO(this->get_logger(), "发布转向信号: %d (二维码: %s)", 
                                   sign_msg.sign_data,  barcodeData.c_str());
 
                        send_stop_command();
 
                        activated_ = false;
                        RCLCPP_INFO(this->get_logger(), "二维码处理完成，等待下次激活");
                        return;
                    }
                }
            }
        }
    }
 
    void send_stop_command()
    {
        auto stop_msg = std::make_unique<std_msgs::msg::Int32>();
        stop_msg->data = 5;
        sign4return_pub_->publish(std::move(stop_msg));
        RCLCPP_INFO(this->get_logger(), "已发布停止命令(5)");
    }
 
    // 成员变量
    size_t frame_count_ = 0;
    rclcpp::Time last_fps_time_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sign_sub_;
    rclcpp::Publisher<origincar_msg::msg::Sign>::SharedPtr sign_switch_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr qr_info_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr sign4return_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr qr_sub_;
 
    bool activated_;
    rclcpp::Time last_sign_time_;
    rclcpp::Time last_process_time_;
    double process_interval_;
    double scale_factor_;
    std::vector<std::array<unsigned int, 4>> qr_codes_;
    zbar::ImageScanner scanner_;  // 重用扫描器实例 
};
 
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<QRDetectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}