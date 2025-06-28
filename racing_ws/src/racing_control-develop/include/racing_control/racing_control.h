// Copyright (c) 2022，Horizon Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RACING_CONTROL_H_
#define RACING_CONTROL_H_

#include <vector>
#include <queue>
#include <mutex>  // 确保包含mutex头文件

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "ai_msgs/msg/perception_targets.hpp"
#include "std_msgs/msg/string.hpp"  // 添加头文件

class RacingControlNode : public rclcpp::Node{
public:
  RacingControlNode(const std::string& node_name,
                        const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~RacingControlNode() override;
private:
    rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr point_subscriber_;
    rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr target_subscriber_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr qr_subscriber_;  // 添加二维码订阅
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

    void subscription_callback_point(const ai_msgs::msg::PerceptionTargets::SharedPtr point_msg);
    void subscription_callback_target(const ai_msgs::msg::PerceptionTargets::SharedPtr targets_msg);
    void subscription_callback_qr(const std_msgs::msg::String::SharedPtr qr_msg);  // 添加二维码回调
    void LineFollowing(const ai_msgs::msg::Target &point_msg);
    void ObstaclesAvoiding(const ai_msgs::msg::Target &target);
    void StopCar();  // 添加停车函数
    void MessageProcess(void);
    std::string pub_control_topic_ = "cmd_vel";

    std::priority_queue<ai_msgs::msg::PerceptionTargets::SharedPtr,
                      std::vector<ai_msgs::msg::PerceptionTargets::SharedPtr>>
    point_queue_;
    std::priority_queue<ai_msgs::msg::PerceptionTargets::SharedPtr,
                      std::vector<ai_msgs::msg::PerceptionTargets::SharedPtr>>
    targets_queue_;

    std::mutex point_target_mutex_;
    std::mutex qr_mutex_;  // 添加二维码互斥锁
    bool process_stop_ = false;
    std::shared_ptr<std::thread> msg_process_;
    float avoid_angular_ratio_ = 1.1;
    float avoid_linear_speed_ = 0.25;
    float follow_angular_ratio_ = -1.0;
    float follow_linear_speed_ = 1.5;
    bool sub_target_ = false; 
    int bottom_threshold_ = 340;
    float confidence_threshold_ = 0.5;
    bool qr_detected_ = false;  // 添加二维码检测标志
};


#endif  // RACING_CONTROL_H_