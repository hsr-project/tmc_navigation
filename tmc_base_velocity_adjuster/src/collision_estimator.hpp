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
/// @file collision_estimator.hpp
/// @brief Predicts interference between the robot and obstacles
#ifndef TMC_BASE_VELOCITY_ADJUSTER_COLLISION_ESTIMATOR_HPP_
#define TMC_BASE_VELOCITY_ADJUSTER_COLLISION_ESTIMATOR_HPP_
#include <vector>

#include <pcl/common/transforms.h>
#include <pcl/filters/crop_hull.h>

namespace { // NOLINT
// Index of cart velocity elements
enum BaseVelocityCoordinates { kX, kY, kYaw, kNumCoordinates };
// Approximation angle count for circular polygon approximation
constexpr uint32_t kCirclePartitionNumber = 12;
}  // anonymous namespace

namespace tmc_base_velocity_adjuster {
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

/// @brief Obstacle interference prediction class
/// @tparam Obstacle Obstacle data type
template<typename Obstacle>
class CollisionEstimator {};

/// @brief Interference prediction class with point cloud
template<>
class CollisionEstimator<PointCloud> {
 public:
  /// @brief Constructor
  /// @param [in] collision_area_radius Radius of the area for interference judgment
  /// @param [in] collision_area_increase_rate Increase rate of the interference judgment area radius relative to the input velocity magnitude
  /// @param [in] estimation_time Time interval for prediction
  CollisionEstimator(const double collision_area_radius,
                     const double collision_area_increase_rate,
                     const double estimation_time)
      : collision_area_(PointCloud::Ptr(new PointCloud())),
        scaling_factor_(collision_area_increase_rate / collision_area_radius),
        estimation_time_(estimation_time) {
    // Prepare interference judgment area
    // For computational efficiency, determine inside/outside points using a CropHull filter on a regular polygon approximation of a circle
    pcl::Vertices hull_vertices;
    std::vector<pcl::Vertices> vertices;
    for (uint32_t i = 0; i < kCirclePartitionNumber; ++i) {
      const double angle = (2.0 * M_PI * static_cast<double>(i)) / static_cast<double>(kCirclePartitionNumber);
      collision_area_->push_back(pcl::PointXYZ(collision_area_radius * cos(angle),
                                               collision_area_radius * sin(angle),
                                               0.0));
      hull_vertices.vertices.push_back(i);
    }
    vertices.push_back(hull_vertices);
    crop_.setHullIndices(vertices);
    crop_.setHullCloud(collision_area_);
    crop_.setDim(2);
    crop_.setCropOutside(true);
  }
  ~CollisionEstimator() {}

  /// @brief Interference score calculation
  /// @param [in] velocity Input cart velocity
  /// @param [in] obstacle Point cloud
  /// @return Score representing the predicted degree of interference
  double EstimateCollisionScore(const Eigen::Vector3d& velocity, const PointCloud::Ptr& obstacle) {
    // Expand and move the interference judgment area corresponding to the movement velocity
    const Eigen::Affine3d scale(Eigen::Scaling(1.0 + scaling_factor_ * velocity.head(2).norm()));
    const Eigen::Affine3d rot(Eigen::AngleAxisd(estimation_time_ * velocity(kYaw), Eigen::Vector3d::UnitZ()));
    const Eigen::Affine3d trans(Eigen::Translation3d(estimation_time_ *
                                                     Eigen::Vector3d(velocity(kX), velocity(kY), 0.0)));
    auto transform = trans * rot * scale;
    PointCloud::Ptr transformed_collision_area(new PointCloud());
    PointCloud::Ptr collision_cloud(new PointCloud());
    pcl::transformPointCloud(*collision_area_, *transformed_collision_area, transform);

    // Extract point cloud within the interference area
    crop_.setHullCloud(transformed_collision_area);
    crop_.setInputCloud(obstacle);
    crop_.filter(*collision_cloud);
    // Return the number of interference points as a score
    return static_cast<double>(collision_cloud->points.size());
  }

 private:
  // Interference judgment area
  PointCloud::Ptr collision_area_;
  // Filter for extracting point cloud within the range
  pcl::CropHull<pcl::PointXYZ> crop_;
  // Scale change rate of interference judgment area relative to input velocity magnitude [1/(m/s)]
  const double scaling_factor_;
  // Time interval for prediction [s]
  const double estimation_time_;
};

}  // namespace tmc_base_velocity_adjuster

#endif  // TMC_BASE_VELOCITY_ADJUSTER_COLLISION_ESTIMATOR_HPP_
