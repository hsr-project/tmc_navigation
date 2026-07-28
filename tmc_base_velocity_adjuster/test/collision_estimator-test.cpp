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
#include "../src/collision_estimator.hpp"

#include <math.h>
#include <Eigen/Core>
#include <gtest/gtest.h>
#include <pcl/common/transforms.h>
#include "test_utils.hpp"

namespace {
// Parameters
constexpr double kCollisionAreaRadius = 0.25;
constexpr double kCollisionAreaIncreaseRate = 0.5;
constexpr double kEstimationTime = 1.5;
// Radius of obstacle [m]
constexpr double kObstacleRadius = 0.3;
// Number of points per obstacle
constexpr int32_t kObstaclePointNum = 5;
// Minimum value for boundary analysis [m]
constexpr double kSmallValue = 0.1;
// Subscriber timeout [s]
constexpr double kTimeOut = 1.0;
}  // anonymous namespace

namespace tmc_base_velocity_adjuster {
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
/// CollisionEstimator test fixture
class CollisionEstimatorTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    collision_estimator_.reset(
        new CollisionEstimator<PointCloud>(kCollisionAreaRadius, kCollisionAreaIncreaseRate, kEstimationTime));
  }

  // Test target
  std::unique_ptr<CollisionEstimator<PointCloud>> collision_estimator_;
};

/// CollisionEstimator test
/// Judgment area moves based on velocity
TEST_F(CollisionEstimatorTest, MoveCollisionAreaAccordingToVelocity) {
  // setup
  // Input velocity at rest
  const Eigen::Vector3d input_velocity0(0.0, 0.0, 0.0);
  // Input velocity of 1 in the positive x direction
  const Eigen::Vector3d input_velocity1(1.0, 0.0, 0.0);
  // Input velocity of -1 in the negative x direction
  const Eigen::Vector3d input_velocity2(-1.0, 0.0, 0.0);
  // Generate an obstacle at the position after EstimationTime when moving at velocity 1 in the positive x direction
  const Eigen::Vector3d estimation_point = input_velocity1 * kEstimationTime;
  PointCloud::Ptr obstacle(new PointCloud());
  AddCircularObstacle(estimation_point.x(), estimation_point.y(),
                      kObstacleRadius, kObstaclePointNum, *obstacle);

  // exercise&verify
  // Confirm that all points are outside the judgment area if input velocity is 0
  EXPECT_DOUBLE_EQ(0.0, collision_estimator_->EstimateCollisionScore(input_velocity0, obstacle));
  // Confirm that all points are inside the judgment area if input velocity is 1 in the positive x direction
  EXPECT_DOUBLE_EQ(static_cast<double>(kObstaclePointNum),
                   collision_estimator_->EstimateCollisionScore(input_velocity1, obstacle));
  // Confirm that all points are outside the judgment area if input velocity is -1 in the negative x direction
  EXPECT_DOUBLE_EQ(0.0, collision_estimator_->EstimateCollisionScore(input_velocity2, obstacle));
}

/// CollisionEstimator test
/// The number of points within the judgment area is obtained
/// The judgment area expands or contracts according to velocity
TEST_F(CollisionEstimatorTest, GetObstacleScoreInCollisionArea) {
  // setup
  // Calculate the boundary line of the judgment area corresponding to input velocity 1
  const Eigen::Vector3d input_velocity1(1.0, 0.0, 0.0);
  const Eigen::Vector3d estimation_point1 = input_velocity1 * kEstimationTime;
  // The boundary of the judgment area is determined by robot movement prediction + judgment area + area increase based on velocity
  const double boundary1_x =
      estimation_point1.x() +
      kCollisionAreaRadius +
      kCollisionAreaIncreaseRate * input_velocity1.head(2).norm();

  // Calculate the boundary line of the judgment area corresponding to input velocity 2, which is greater than input velocity 1
  const Eigen::Vector3d input_velocity2(2.0, 0.0, 0.0);
  const Eigen::Vector3d estimation_point2 = input_velocity2 * kEstimationTime;
  const double boundary2_x =
      estimation_point2.x() +
      kCollisionAreaRadius +
      kCollisionAreaIncreaseRate * input_velocity2.head(2).norm();

  // Place 8 points in total, 2 points inside and outside each judgment area boundary line
  // Place 2 points each to prevent filtering from removing single points
  PointCloud::Ptr obstacle(new PointCloud());
  // Outside the boundary range of input velocity 1
  obstacle->points.push_back(pcl::PointXYZ(boundary1_x + kSmallValue, kSmallValue, 0.0));
  obstacle->points.push_back(pcl::PointXYZ(boundary1_x + kSmallValue, -kSmallValue, 0.0));
  // Inside the boundary range of input velocity 1
  obstacle->points.push_back(pcl::PointXYZ(boundary1_x - kSmallValue, kSmallValue, 0.0));
  obstacle->points.push_back(pcl::PointXYZ(boundary1_x - kSmallValue, -kSmallValue, 0.0));
  // Outside the boundary range of input velocity 2
  obstacle->points.push_back(pcl::PointXYZ(boundary2_x + kSmallValue, kSmallValue, 0.0));
  obstacle->points.push_back(pcl::PointXYZ(boundary2_x + kSmallValue, -kSmallValue, 0.0));
  // Inside the boundary range of input velocity 2
  obstacle->points.push_back(pcl::PointXYZ(boundary2_x - kSmallValue, kSmallValue, 0.0));
  obstacle->points.push_back(pcl::PointXYZ(boundary2_x - kSmallValue, -kSmallValue, 0.0));
  obstacle->height = 1;
  obstacle->width = obstacle->points.size();
  obstacle->is_dense = false;

  // exercise&verify
  // Confirm that only the 2 points within the boundary range of input velocity 1 are counted for input velocity 1
  EXPECT_DOUBLE_EQ(2.0, collision_estimator_->EstimateCollisionScore(input_velocity1, obstacle));
  // Confirm that 6 points within the boundary range of input velocity 2 are counted for input velocity 2
  // Input velocity 2 is greater than input velocity 1, so it also includes the 4 points corresponding to input velocity 1
  EXPECT_DOUBLE_EQ(6.0, collision_estimator_->EstimateCollisionScore(input_velocity2, obstacle));
}

}  // namespace tmc_base_velocity_adjuster

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
