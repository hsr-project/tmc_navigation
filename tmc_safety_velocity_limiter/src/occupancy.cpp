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
/// @file occupancy.cpp
/// @brief Occupancy class
#include "occupancy.hpp"

#include <memory>
#include <string>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>

#include "param.hpp"
namespace {
const char* kTopicOccupancyGrid = "obstacle_map";  // OccupancyGrid Topic name
const int32_t kMessageIndicatePeriod = 10000;      // Log output period [msec]
const char* const kBaseFrameId = "base_link";      // Base coordinate frame name
const int32_t kUnknown = -1;                       // Occupancy Unknown value
}  // anonymous namespace

namespace tmc_safety_velocity_limiter {
using std::placeholders::_1;
/// Occupancy class Singleton
Occupancy* Occupancy::GetInstance() {
  static Occupancy occupancy_;
  return &occupancy_;
}

void Occupancy::Init(const rclcpp::Node::SharedPtr node) {
  // Parameter acquisition
  GetOptionalParam(node, "base_frame", base_frame_, std::string(kBaseFrameId));
  // Subscriber registration
  sub_occupancy_grid_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
      kTopicOccupancyGrid, 1, std::bind(&Occupancy::OccupancyGridCallback, this, _1));
}

/// Retrieve raw data of OccupancyGrid
nav_msgs::msg::OccupancyGrid::ConstSharedPtr Occupancy::OccupancyGrid() {
  return occupancy_grid_;
}

/// Get the occupancy value of the corresponding location from the base coordinates
int32_t Occupancy::GetOccupancyFromBase(const double x, const double y) {
  // If OccupancyGrid has not been received, return unknown (-1)
  if (!occupancy_grid_) {
    return kUnknown;
  }
  // Convert the specified base coordinates to the coordinate system of OccupancyGrid
  // Since the update frequency of OccupancyGrid is low and the temporal accuracy of coordinate conversion is not important,
  // Convert using Time(0) (latest value available without waiting) without specifying the time axis of tf
  geometry_msgs::msg::PointStamped position;
  position.header.frame_id = base_frame_;
  position.point.x = x;
  position.point.y = y;
  geometry_msgs::msg::PointStamped transform_position;
  try {
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped = tf_buffer_.lookupTransform(occupancy_grid_->header.frame_id, base_frame_, rclcpp::Time(0));
    tf2::doTransform(position, transform_position, transform_stamped);
  } catch (...) {
    auto clock = rclcpp::Clock(RCL_ROS_TIME);
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("safety_velocity_limiter"),
        clock, kMessageIndicatePeriod,
        "transform error. [%s]->[%s]",
        base_frame_.c_str(), occupancy_grid_->header.frame_id.c_str());
    return kUnknown;
  }
  // Calculate X, Y indices of Grid data
  double dx = transform_position.point.x - occupancy_grid_->info.origin.position.x;
  double dy = transform_position.point.y - occupancy_grid_->info.origin.position.y;
  int32_t index_x = static_cast<int32_t>(dx / occupancy_grid_->info.resolution);
  int32_t index_y = static_cast<int32_t>(dy / occupancy_grid_->info.resolution);
  // If the specified coordinates are outside the Grid range, return unknown (-1)
  if ((index_x < 0) || (index_x >= static_cast<int32_t>(occupancy_grid_->info.width)) ||
      (index_y < 0) || (index_y >= static_cast<int32_t>(occupancy_grid_->info.height))) {
    return kUnknown;
  } else {
    size_t index = static_cast<size_t>(index_y) * occupancy_grid_->info.width + static_cast<size_t>(index_x);
    return occupancy_grid_->data[index];
  }
}

/// Constructor
Occupancy::Occupancy() :
    tf_buffer_(std::make_shared<rclcpp::Clock>(rclcpp::Clock(RCL_ROS_TIME))),
    tf_listener_(tf_buffer_) {
  occupancy_grid_ = nav_msgs::msg::OccupancyGrid::ConstSharedPtr();
}
/// Destructor
Occupancy::~Occupancy() {}
/// Pointcloud callback
void Occupancy::OccupancyGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  occupancy_grid_ = msg;
}
}  // namespace tmc_safety_velocity_limiter
