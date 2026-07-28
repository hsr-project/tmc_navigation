/*
Copyright (c) 2026 TOYOTA MOTOR CORPORATION
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
/// @file obstacle.hpp
/// @brief Obstacle class
#include "obstacle.hpp"
#include <memory>
#include <string>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_listener.h>

#include "param.hpp"

namespace {
const char* kTopicObstacleCloud = "obstacle_cloud";    // PointCloud topic name
const char* const kBaseFrameId = "base_link";          // Base coordinate frame name
const int32_t kConsoleMessageIndicatePeriod = 10000;   // Log output period [msec]
const double kTfWaitDuration = 1.0;                    // tf reception wait time [sec]
}  // anonymous namespace

namespace tmc_safety_velocity_limiter {
using std::placeholders::_1;
/// Obstacle class Singleton
Obstacle* Obstacle::GetInstance() {
  static Obstacle obstacle_;
  return &obstacle_;
}

void Obstacle::Init(const rclcpp::Node::SharedPtr node) {
  // Parameter retrieval
  GetOptionalParam(node, "base_frame", base_frame_, std::string(kBaseFrameId));
  // Subscriber registration
  sub_pointcloud_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
      kTopicObstacleCloud, rclcpp::SensorDataQoS(), std::bind(&Obstacle::ObstacleCloudCallback, this, _1));
}

/// Retrieve obstacle cloud
PointCloudPtr Obstacle::ObstacleCloud() {
  PointCloudPtr cloud = PointCloudPtr(new PointCloud());
  // Coordinate transformation
  TransformPointCloud(obstacle_cloud_, base_frame_, cloud);
  return cloud;
}

/// Constructor
Obstacle::Obstacle() :
    tf_buffer_(std::make_shared<rclcpp::Clock>(rclcpp::Clock(RCL_ROS_TIME))),
    tf_listener_(tf_buffer_) {
  obstacle_cloud_ = PointCloudPtr(new PointCloud());
}

/// Destructor
Obstacle::~Obstacle() {}

/// Pointcloud callback
void Obstacle::ObstacleCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  if (msg->data.empty() || msg->width == 0 || msg->height == 0) {
    obstacle_cloud_->clear();
    return;
  }
  // PointCloud2 -> PointCloud
  pcl::fromROSMsg(*msg, *obstacle_cloud_);
}

/// Point cloud coordinate transformation
void Obstacle::TransformPointCloud(const PointCloudPtr& input, const std::string& frame, PointCloudPtr& output) {
  output->clear();
  output->header.frame_id = frame;
  output->header.stamp = input->header.stamp;
  auto clock = rclcpp::Clock(RCL_ROS_TIME);
  if (!input->empty() && (pcl_conversions::fromPCL(input->header).stamp.sec != 0)) {
    try {
      bool transform_is_found = false;
      transform_is_found = tf_buffer_.canTransform(
          frame, input->header.frame_id, rclcpp::Time(0), rclcpp::Duration::from_seconds(kTfWaitDuration));
      if (transform_is_found) {
        bool ret = pcl_ros::transformPointCloud(frame, *input, *output, tf_buffer_);
        if (!ret) {
          RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("safety_velocity_limiter"),
              clock, kConsoleMessageIndicatePeriod,
              "transform error : [%s]->[%s]",
              input->header.frame_id.c_str(), frame.c_str());
        }
      } else {
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("safety_velocity_limiter"),
              clock, kConsoleMessageIndicatePeriod,
              "wait for transform timeout...[%s]->[%s]",
              input->header.frame_id.c_str(), frame.c_str());
      }
    } catch (const std::exception& ex) {
      RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("safety_velocity_limiter"),
          clock, kConsoleMessageIndicatePeriod,
          "transform error : %s", ex.what());
    }
  }
}
}  // namespace tmc_safety_velocity_limiter
