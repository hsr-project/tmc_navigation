/*
Copyright (c) 2025 TOYOTA MOTOR CORPORATION
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the disclaimer
below) provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of the copyright holder nor the names of its contributors may be used
  to endorse or promote products derived from this software without specific
  prior written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/
#include <limits>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace tmc_base_path_planner {

/// Periodically publish dynamic map, self-position, and TF
class CyclicSender : public rclcpp::Node {
 public:
  explicit CyclicSender(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) :
      Node("cyclic_sender", options), send_dynamic_map_(false), send_global_pose_(false), send_transform_(false) {}

  void Init(const double hz) {
    if (hz <= std::numeric_limits<double>::epsilon()) {
      RCLCPP_FATAL(this->get_logger(), "hz must be plus.");
      exit(EXIT_FAILURE);
    }
    rate_ = std::make_shared<rclcpp::Rate>(hz);
    pub_global_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/global_pose", 1);
    pub_dynamic_map_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/dynamic_obstacle_map", 1);
    broadcaster_.reset(new tf2_ros::TransformBroadcaster(shared_from_this()));
    // Wait until linked with subscriber
    const rclcpp::Time start = this->get_clock()->now();
    while (pub_dynamic_map_->get_subscription_count() == 0 ||
           pub_global_pose_->get_subscription_count() == 0) {
      if (this->get_clock()->now() - start > rclcpp::Duration::from_seconds(10.0)) {
        RCLCPP_FATAL(this->get_logger(), "Can not link to subscribers.");
        exit(EXIT_FAILURE);
      }
      rate_->sleep();
    }
  }

  void Run() {
    killed_ = false;
    while (rclcpp::ok() && !killed_) {
      if (send_dynamic_map_) {
        // Send dynamic map
        dynamic_map_.header.stamp = this->get_clock()->now();
        pub_dynamic_map_->publish(dynamic_map_);
      }
      if (send_global_pose_) {
        // Send self-position
        global_pose_.header.stamp = this->get_clock()->now();
        pub_global_pose_->publish(global_pose_);
      }
      if (send_transform_) {
        // Send TF
        transform_.header.stamp = this->get_clock()->now();
        broadcaster_->sendTransform(transform_);
      }
      rate_->sleep();
      rclcpp::spin_some(shared_from_this());
    }
  }

  void Kill() {
    killed_ = true;
  }

  // Start sending dynamic map
  void StartSendDynamicMap(const nav_msgs::msg::OccupancyGrid& dynamic_map) {
    dynamic_map_ = dynamic_map;
    dynamic_map_.header.stamp = this->get_clock()->now();
    pub_dynamic_map_->publish(dynamic_map_);
    send_dynamic_map_ = true;
    rate_->sleep();
    rclcpp::spin_some(shared_from_this());
    return;
  }

  // Start sending self-position
  void StartSendGlobalPose(const geometry_msgs::msg::PoseStamped& global_pose) {
    global_pose_ = global_pose;
    global_pose_.header.stamp = this->get_clock()->now();
    pub_global_pose_->publish(global_pose_);
    send_global_pose_ = true;
    rate_->sleep();
    rclcpp::spin_some(shared_from_this());
    return;
  }

  // Start publishing TF
  void StartSendTransform(const std::string& frame_id, const std::string& child_frame_id,
                          const geometry_msgs::msg::Pose& pose) {
    transform_.header.stamp = this->get_clock()->now();
    transform_.header.frame_id = frame_id;
    transform_.child_frame_id = child_frame_id;
    transform_.transform.translation.x = pose.position.x;
    transform_.transform.translation.y = pose.position.y;
    transform_.transform.translation.z = pose.position.z;
    transform_.transform.rotation.x = pose.orientation.x;
    transform_.transform.rotation.y = pose.orientation.y;
    transform_.transform.rotation.z = pose.orientation.z;
    transform_.transform.rotation.w = pose.orientation.w;
    broadcaster_->sendTransform(transform_);
    send_transform_ = true;
    rate_->sleep();
    rclcpp::spin_some(shared_from_this());
    return;
  }

  void StopSendDynamicMap() {
    send_dynamic_map_ = false;
  }

  void StopSendGlobalPose() {
    send_global_pose_ = false;
  }

  void StopSendTransform() {
    send_transform_ = false;
  }

 private:
  std::shared_ptr<rclcpp::Rate> rate_;

  bool send_dynamic_map_;
  bool send_global_pose_;
  bool send_transform_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_global_pose_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_dynamic_map_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

  nav_msgs::msg::OccupancyGrid dynamic_map_;
  geometry_msgs::msg::PoseStamped global_pose_;
  geometry_msgs::msg::TransformStamped transform_;
  // Stop flag
  bool killed_;
};
}  // namespace tmc_base_path_planner
