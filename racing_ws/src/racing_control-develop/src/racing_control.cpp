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


RacingControlNode::RacingControlNode(const std::string& node_name,const rclcpp::NodeOptions& options)
  : rclcpp::Node(node_name, options) {
  if (!msg_process_) {
    msg_process_ = std::make_shared<std::thread>(
        std::bind(&RacingControlNode::MessageProcess, this));
  }
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

  this->declare_parameter<float>("line_confidence_threshold", line_confidence_threshold_);
  this->get_parameter<float>("line_confidence_threshold", line_confidence_threshold_);

  this->declare_parameter<float>("obstacle_confidence_threshold", obstacle_confidence_threshold_);
  this->get_parameter<float>("obstacle_confidence_threshold", obstacle_confidence_threshold_);

  this->declare_parameter<float>("recovering_angular_ratio", recovering_angular_ratio_);
  this->get_parameter<float>("recovering_angular_ratio", recovering_angular_ratio_);
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
    this->get_parameter<float>("obstacle_confidence_threshold", obstacle_confidence_threshold_);
    this->get_parameter<float>("line_confidence_threshold", line_confidence_threshold_);

    std::unique_lock<std::mutex> lock(point_target_mutex_);
    if (!point_queue_.empty() && sub_target_ == false){
      auto point_msg = point_queue_.top();
      float line_confidence = point_msg->targets[0].points[0].confidence[0];
      lock.unlock();
      LineFollowing(point_msg->targets[0], line_confidence);
      lock.lock();
      point_queue_.pop();
    }
    if (!point_queue_.empty() && !targets_queue_.empty() && sub_target_== true) {
      auto point_msg = point_queue_.top();
      auto targets_msg = targets_queue_.top();
      point_queue_.pop();
      targets_queue_.pop();
      lock.unlock();
      if(targets_msg->targets.size() == 0){
        float line_confidence = point_msg->targets[0].points[0].confidence[0];
        LineFollowing(point_msg->targets[0], line_confidence);
      } else {
          for(const auto &target : targets_msg->targets){
            if(target.type == "construction_cone"){
              int bottom = target.rois[0].rect.y_offset + target.rois[0].rect.height;
              if (bottom < bottom_threshold_ ){
                float line_confidence = point_msg->targets[0].points[0].confidence[0];
                LineFollowing(point_msg->targets[0], line_confidence);
              } else {
                if(target.rois[0].confidence > obstacle_confidence_threshold_){
                  ObstaclesAvoiding(target);
                }
              }
            }
          }
      }
      lock.lock();
    }
  }
}

void RacingControlNode::LineFollowing(const ai_msgs::msg::Target &point_msg,float line_confidence){
  int x = int(point_msg.points[0].point[0].x);
  int y = int(point_msg.points[0].point[0].y);
  float temp = x - 320.0;
  if ((-20 < x ) && ( x < 0)) {
    temp = -20;
  } else if ((x > 0) && (x < 20)) {
    temp = 20;
  }
  auto twist_msg = geometry_msgs::msg::Twist();
  float angular_z;
  if (line_confidence < line_confidence_threshold_) {
    angular_z = -1.0 * std::copysign(recovering_angular_ratio_, last_avoidance_angular_z_); // 如果置信度低于阈值，使用最后一次避障的反方向角速度
    twist_msg.linear.x = 0.6 * follow_linear_speed_;
  }
  else {
    float line_y_relative = (static_cast<float>(y) - 256.0f) / (480.0f - 256.0f);
    line_y_relative = std::max(0.0f, std::min(1.0f, line_y_relative));
    angular_z = follow_angular_ratio_ * (temp / 320.0f) * line_y_relative; 
    twist_msg.linear.x = follow_linear_speed_;
  }

  
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
  if ((-5.0f < temp ) && ( temp <= 0)) {
    temp = -5.0f;
  } else if ((temp > 0) && (temp < 5.0f)) {
    temp = 5.0f;
  }
  float angular_z = -1.0f * avoid_angular_ratio_ * std::min(4.0f, (320.0f / temp));

  last_avoidance_angular_z_ = angular_z; // 记录最后一次避障的角速度
  
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
