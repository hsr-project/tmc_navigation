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
/// @file obstacle.hpp
/// @brief Obstacle class
#ifndef TMC_SAFETY_VELOCITY_LIMITER_OBSTACLE_HPP_
#define TMC_SAFETY_VELOCITY_LIMITER_OBSTACLE_HPP_
#include <string>
#include <boost/utility.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "common.hpp"

namespace tmc_safety_velocity_limiter {
/// Obstacle class Singleton
class Obstacle : private boost::noncopyable {
 public:
  static Obstacle* GetInstance();

  void Init(const rclcpp::Node::SharedPtr node);

  /// Get obstacle cloud
  PointCloudPtr ObstacleCloud();

 private:
  /// Constructor
  Obstacle();
  /// Destructor
  virtual ~Obstacle();
  /// Pointcloud callback
  void ObstacleCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  /// Point cloud coordinate transformation
  void TransformPointCloud(const PointCloudPtr& input, const std::string& frame, PointCloudPtr& output);

  // Point cloud Subscriber
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pointcloud_;
  // Frame name of base coordinates
  std::string base_frame_;
  // Obstacle cloud
  PointCloudPtr obstacle_cloud_;
  // Coordinate transformation
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};
}  // namespace tmc_safety_velocity_limiter
#endif  // TMC_SAFETY_VELOCITY_LIMITER_OBSTACLE_HPP_
