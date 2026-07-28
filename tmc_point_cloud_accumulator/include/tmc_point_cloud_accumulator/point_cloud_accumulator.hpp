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
/// @file point_cloud_accumulator.hpp
/// @brief Past point cloud accumulation class
#ifndef TMC_POINT_CLOUD_ACCUMULATOR_POINT_CLOUD_ACCUMULATOR_HPP_
#define TMC_POINT_CLOUD_ACCUMULATOR_POINT_CLOUD_ACCUMULATOR_HPP_

#include <memory>
#include <string>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2_ros/transform_listener.h>

#include "common.hpp"

namespace tmc_point_cloud_accumulator {
class PointCloudAccumulator {
 public:
  // Constructor
  explicit PointCloudAccumulator(const rclcpp::Clock::SharedPtr& clock);
  // Destructor
  ~PointCloudAccumulator() {}

  // Mutator
  void set_point_cloud_keep_period(const double period) { point_cloud_keep_period_ = period; }

  // Accumulate point cloud
  void Accumulate(const PointCloudPtr& current_cloud);

  // Reset accumulated point cloud
  void ClearPointCloud();

  // Merge and extract accumulated point cloud
  PointCloudPtr GetPointCloud();

 private:
  // Buffer for storing accumulated point cloud
  std::vector<PointCloudPtr> point_cloud_buffer_;

  // Duration to retain past point cloud [s]
  double point_cloud_keep_period_;

  rclcpp::Clock::SharedPtr clock_;
};
}  // namespace tmc_point_cloud_accumulator

#endif  // TMC_POINT_CLOUD_ACCUMULATOR_POINT_CLOUD_ACCUMULATOR_HPP_
