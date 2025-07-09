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

#include "racing_control/racing_control.h"

#include <unistd.h>


// racing_control.cpp

RacingControlNode::RacingControlNode(const std::string& node_name, const rclcpp::NodeOptions& options)
  : rclcpp::Node(node_name, options) {
  if (!msg_process_) {
    msg_process_ = std::make_shared<std::thread>(
        std::bind(&RacingControlNode::MessageProcess, this));
  }
  
  // --- 声明和获取参数 ---
  this->declare_parameter<std::string>("pub_control_topic", pub_control_topic_);
  this->get_parameter<std::string>("pub_control_topic", pub_control_topic_);

  this->declare_parameter<float>("avoid_angular_ratio", avoid_angular_ratio_);
  this->get_parameter<float>("avoid_angular_ratio", avoid_angular_ratio_);

  this->declare_parameter<float>("avoid_linear_speed", avoid_linear_speed_);
  this->get_parameter<float>("avoid_linear_speed", avoid_linear_speed_);

  this->declare_parameter<int>("bottom_threshold", bottom_threshold_);
  this->get_parameter<int>("bottom_threshold", bottom_threshold_);

  this->declare_parameter<float>("follow_linear_speed", follow_linear_speed_);
  this->get_parameter<float>("follow_linear_speed", follow_linear_speed_);

  this->declare_parameter<float>("follow_angular_ratio", follow_angular_ratio_);
  this->get_parameter<float>("follow_angular_ratio", follow_angular_ratio_);

  this->declare_parameter<float>("confidence_threshold", confidence_threshold_);
  this->get_parameter<float>("confidence_threshold", confidence_threshold_);

  // =========== 新增参数的声明与获取 ===========
  this->declare_parameter<double>("search_duration_sec", search_duration_sec_);
  this->get_parameter<double>("search_duration_sec", search_duration_sec_);
  RCLCPP_INFO(this->get_logger(), "Parameter search_duration_sec set to: %.2f s", search_duration_sec_);
  // ===========================================

  // --- 创建订阅者和发布者 (这部分不变) ---
  point_subscriber_ =
      this->create_subscription<ai_msgs::msg::PerceptionTargets>(
            "racing_track_center_detection",
            rclcpp::SensorDataQoS(),
            std::bind(&RacingControlNode::subscription_callback_point,
                      this,
                      std::placeholders::_1));

  target_subscriber_ =
      this->create_subscription<ai_msgs::msg::PerceptionTargets>(
            "racing_obstacle_detection",
            rclcpp::SensorDataQoS(),
            std::bind(&RacingControlNode::subscription_callback_target,
                      this,
                      std::placeholders::_1));
  publisher_ =
    this->create_publisher<geometry_msgs::msg::Twist>(pub_control_topic_, 5);
  RCLCPP_INFO(rclcpp::get_logger("RacingControlNode"), "RacingControlNode initialized!");
}

RacingControlNode::~RacingControlNode(){
  if (msg_process_ && msg_process_->joinable()) {
    process_stop_ = true;
    msg_process_->join();
    msg_process_ = nullptr;
  }
  {
    std::unique_lock<std::mutex> lock(point_target_mutex_);
    while (!point_queue_.empty()) {
      point_queue_.pop();
    }
  }
  {
    std::unique_lock<std::mutex> lock(point_target_mutex_);
    while (!targets_queue_.empty()) {
      targets_queue_.pop();
    }
  }
}

void RacingControlNode::subscription_callback_point(const ai_msgs::msg::PerceptionTargets::SharedPtr point_msg){
  {
    std::unique_lock<std::mutex> lock(point_target_mutex_);
    point_queue_.push(point_msg);
    if (point_queue_.size() > 1) {
      point_queue_.pop();
    }
  }
  return;
}

void RacingControlNode::subscription_callback_target(const ai_msgs::msg::PerceptionTargets::SharedPtr targets_msg){
  {
    sub_target_ = true;
    std::unique_lock<std::mutex> lock(point_target_mutex_);
    targets_queue_.push(targets_msg);
    if (targets_queue_.size() > 1) {
      targets_queue_.pop();
    }
  }
  return;
}

void RacingControlNode::MessageProcess(){
  while(process_stop_ == false){
    this->get_parameter<float>("avoid_angular_ratio", avoid_angular_ratio_);
    this->get_parameter<float>("avoid_linear_speed", avoid_linear_speed_);
    this->get_parameter<int>("bottom_threshold", bottom_threshold_);
    this->get_parameter<float>("follow_linear_speed", follow_linear_speed_);
    this->get_parameter<float>("follow_angular_ratio", follow_angular_ratio_);
    this->get_parameter<float>("confidence_threshold", confidence_threshold_);
    this->get_parameter<double>("search_duration_sec", search_duration_sec_); // 动态更新回正时长

    std::unique_lock<std::mutex> lock(point_target_mutex_);
    if (!point_queue_.empty() && sub_target_ == false){
      auto point_msg = point_queue_.top();
      lock.unlock();
      LineFollowing(point_msg->targets[0]);
      lock.lock();
      point_queue_.pop();
    }
    if (!point_queue_.empty() && !targets_queue_.empty() && sub_target_== true) {
      auto point_msg = point_queue_.top();
      auto targets_msg = targets_queue_.top();
      point_queue_.pop();
      targets_queue_.pop();
      lock.unlock();

    // --- 决策预处理 ---
    bool has_close_obstacle = false;
    ai_msgs::msg::Target close_obstacle;
    if (!targets_msg->targets.empty()) {
        for (const auto &target : targets_msg->targets) {
            int bottom = target.rois[0].rect.y_offset + target.rois[0].rect.height;
            if (bottom >= bottom_threshold_ && target.rois[0].confidence > confidence_threshold_) {
                has_close_obstacle = true;
                close_obstacle = target;
                break;
            }
        }
    }
    
    // =========== 状态机逻辑开始 ============
    switch (current_state_) {
        case RobotState::LINE_FOLLOWING:
            RCLCPP_INFO_ONCE(this->get_logger(), "Current State: LINE_FOLLOWING");
            if (has_close_obstacle) {
                // 条件：巡线时发现近距离障碍物
                current_state_ = RobotState::OBSTACLE_AVOIDING;
                RCLCPP_WARN(this->get_logger(), "Transitioning to OBSTACLE_AVOIDING state.");

                // 记录避障方向
                int center_x = close_obstacle.rois[0].rect.x_offset + close_obstacle.rois[0].rect.width / 2;
                avoidance_direction_ = (center_x < 320) ? 1 : -1; // 障碍物在左，向右(1)避；障碍物在右，向左(-1)避。

                ObstaclesAvoiding(close_obstacle); // 立即执行一次避障
            } else {
                // 保持巡线
                if (!point_msg->targets.empty()) {
                    LineFollowing(point_msg->targets[0]);
                } else {
                    // 理论上你的模型总有输出，但作为保护逻辑，如果没点就停下
                    publisher_->publish(geometry_msgs::msg::Twist());
                }
            }
            break;

        case RobotState::OBSTACLE_AVOIDING:
            RCLCPP_INFO_ONCE(this->get_logger(), "Current State: OBSTACLE_AVOIDING");
            if (!has_close_obstacle) {
                // 条件：障碍物消失
                current_state_ = RobotState::SEARCHING_FOR_LINE;
                RCLCPP_WARN(this->get_logger(), "Obstacle cleared. Transitioning to SEARCHING_FOR_LINE state.");

                // 记录开始“回正”动作的时刻
                search_start_time_ = this->get_clock()->now();
            } else {
                // 保持避障
                ObstaclesAvoiding(close_obstacle);
            }
            break;

        case RobotState::SEARCHING_FOR_LINE:
            RCLCPP_INFO_ONCE(this->get_logger(), "Current State: SEARCHING_FOR_LINE");
            
            // 检查“回正”动作是否超时
            if ((this->get_clock()->now() - search_start_time_).seconds() >= search_duration_sec_) {
                // 条件：回正时间到
                current_state_ = RobotState::LINE_FOLLOWING;
                RCLCPP_WARN(this->get_logger(), "Search cooldown finished. Transitioning back to LINE_FOLLOWING state.");
            } else {
                // 保持“回正”动作
                auto search_twist = geometry_msgs::msg::Twist();
                search_twist.linear.x = avoid_linear_speed_ * 1.2f; // 使用一个安全的快速前进
                
                // 向避障的相反方向转弯，以找回赛道
                // 例如：向右避障(avoidance_direction_=1, 角速度为正)，则回正时向左转(角速度为负)
                search_twist.angular.z = 10 * static_cast<double>(avoidance_direction_); // 1.0是可调的转向强度

                RCLCPP_INFO(this->get_logger(), "Re-centering... Turning with angular.z: %f for %.2fs", search_twist.angular.z, search_duration_sec_);
                publisher_->publish(search_twist);
            }
            break;
      }
    // =========== 状态机逻辑结束 ============
    }
  }
}

void RacingControlNode::LineFollowing(const ai_msgs::msg::Target &point_msg){
  int x = int(point_msg.points[0].point[0].x);
  int y = int(point_msg.points[0].point[0].y);
  float temp = x - 320.0;
  if ((-20 < x ) && ( x < 0)) {
    temp = -20;
  } else if ((x > 0) && (x < 20)) {
    temp = 20;
  }
  auto twist_msg = geometry_msgs::msg::Twist();
  float angular_z = follow_angular_ratio_ * temp / 200.0 * y / 224.0;
  twist_msg.linear.x = follow_linear_speed_;
  twist_msg.linear.y = 0.0;
  twist_msg.linear.z = 0.0;
  twist_msg.angular.x = 0.0;
  twist_msg.angular.y = 0.0;
  twist_msg.angular.z = angular_z;
  publisher_->publish(twist_msg);
}

void RacingControlNode::ObstaclesAvoiding(const ai_msgs::msg::Target &target){
  auto twist_msg = geometry_msgs::msg::Twist();
  int center_x = target.rois[0].rect.x_offset + target.rois[0].rect.width / 2;
  float temp = center_x - 320.0;
  if ((-20 < temp ) && ( temp <= 0)) {
    temp = -20;
  } else if ((temp > 0) && (temp < 20)) {
    temp = 20;
  }
  float angular_z = avoid_angular_ratio_ * 100 / temp;
  twist_msg.linear.x = avoid_linear_speed_;
  twist_msg.linear.y = 0.0;
  twist_msg.linear.z = 0.0;
  twist_msg.angular.x = 0.0;
  twist_msg.angular.y = 0.0;
  twist_msg.angular.z = angular_z;
  publisher_->publish(twist_msg);
  return;
}

int main(int argc, char* argv[]) {

  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<RacingControlNode>("GetLineCoordinate"));

  rclcpp::shutdown();

  RCLCPP_WARN(rclcpp::get_logger("RacingControlNode"), "Pkg exit.");
  return 0;
}
