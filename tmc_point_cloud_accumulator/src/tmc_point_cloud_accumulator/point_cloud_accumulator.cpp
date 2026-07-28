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
/// @file point_cloud_accumulator.cpp
/// @brief Past Point Cloud Accumulation Class
#include <vector>
#include "tmc_point_cloud_accumulator/point_cloud_accumulator.hpp"

namespace tmc_point_cloud_accumulator {
/// PointCloudAccumulator Class
/// Constructor
PointCloudAccumulator::PointCloudAccumulator(const rclcpp::Clock::SharedPtr& clock)
    : point_cloud_keep_period_(0.0), clock_(clock) {}

/// Accumulate point clouds
void PointCloudAccumulator::Accumulate(const PointCloudPtr& input_cloud) {
  // Store point clouds in the buffer
  point_cloud_buffer_.push_back(input_cloud);

  // Remove point clouds that exceed the retention time
  for (std::vector<PointCloudPtr>::iterator it = point_cloud_buffer_.begin(); it != point_cloud_buffer_.end();) {
    rclcpp::Duration keep_duration = clock_->now() - rclcpp::Time(pcl_conversions::fromPCL((*it)->header).stamp);
    if (keep_duration.seconds() > point_cloud_keep_period_) {
      it = point_cloud_buffer_.erase(it);
    } else {
      ++it;
    }
  }
}

/// Reset accumulated point clouds
void PointCloudAccumulator::ClearPointCloud() { point_cloud_buffer_.clear(); }

/// Merge and extract accumulated point clouds
PointCloudPtr PointCloudAccumulator::GetPointCloud() {
  typedef std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> > Points;

  // Merge accumulated point clouds
  PointCloudPtr accumulated_cloud(new PointCloud());
  PointCloudPtr merged_cloud(new PointCloud());
  for (std::vector<PointCloudPtr>::iterator it = point_cloud_buffer_.begin(); it != point_cloud_buffer_.end(); ++it) {
    accumulated_cloud = *it;
    for (Points::iterator it2 = accumulated_cloud->points.begin(); it2 != accumulated_cloud->points.end(); ++it2) {
      merged_cloud->points.push_back(*it2);
    }
  }
  return merged_cloud;
}
}  // namespace tmc_point_cloud_accumulator
