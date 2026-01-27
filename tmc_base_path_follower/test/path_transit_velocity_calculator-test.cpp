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
#include <vector>

#include <angles/angles.h>
#include <gtest/gtest.h>

#include <tmc_base_path_follower/common.hpp>
#include <tmc_base_path_follower/path_transit_velocity_calculator.hpp>

#include "test_utils.hpp"

namespace {
// Input path point interval [m]
constexpr double kPathInterval = 0.01;
// Number of points in a straight path
constexpr int32_t kStraightPathPointNum = 100;
}

namespace tmc_base_path_follower {

/// Parameter test
/// Ability to generate parameters
TEST(PathTransitVelocityCalculatorParameterTest, ConstructParameter) {
  // exercise
  // Input parameters with the condition max_linear_velocity > min_linear_velocity
  PathTransitVelocityCalculator::Parameter param(0.1, 0.02, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8);

  // verify
  // Check if the values are set as input
  EXPECT_DOUBLE_EQ(0.1, param.max_linear_velocity);
  EXPECT_DOUBLE_EQ(0.02, param.min_linear_velocity);
  EXPECT_DOUBLE_EQ(0.3, param.max_angular_velocity);
  EXPECT_DOUBLE_EQ(0.4, param.max_linear_acceleration);
  EXPECT_DOUBLE_EQ(0.5, param.max_linear_deceleration);
  EXPECT_DOUBLE_EQ(0.6, param.max_angular_acceleration);
  EXPECT_DOUBLE_EQ(0.7, param.max_angular_deceleration);
  EXPECT_DOUBLE_EQ(0.8, param.transit_velocity_angular_velocity_ratio);
}


/// Parameter test
/// If invalid values are specified, default values are generated
TEST(PathTransitVelocityCalculatorParameterTest, ConstructWithInvalidParameterMakeDefault) {
  // exercise
  // Set all parameters with invalid values
  PathTransitVelocityCalculator::Parameter param(-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0);

  // verify
  // Check if the values are set as input
  EXPECT_DOUBLE_EQ(kMaxLinearVelocityDefault, param.max_linear_velocity);
  EXPECT_DOUBLE_EQ(kMinLinearVelocityDefault, param.min_linear_velocity);
  EXPECT_DOUBLE_EQ(kMaxAngularVelocityDefault, param.max_angular_velocity);
  EXPECT_DOUBLE_EQ(kMaxLinearAccelerationDefault, param.max_linear_acceleration);
  EXPECT_DOUBLE_EQ(kMaxLinearDecelerationDefault, param.max_linear_deceleration);
  EXPECT_DOUBLE_EQ(kMaxAngularAccelerationDefault, param.max_angular_acceleration);
  EXPECT_DOUBLE_EQ(kMaxAngularDecelerationDefault, param.max_angular_deceleration);
  EXPECT_DOUBLE_EQ(kTransitVelocityAngularVelocityRatioDefault, param.transit_velocity_angular_velocity_ratio);
}

/// Parameter test
/// If the maximum translational velocity is less than the minimum translational velocity, default values are used
TEST(DiffDriveVelocityCalculatorParameterTest, MaxVelocityLowerThanMinVelocity) {
  // exercise
  /// Input parameters with the condition max_linear_velocity < min_linear_velocity
  PathTransitVelocityCalculator::Parameter param(0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8);

  // verify
  // max_linear_velocity and min_linear_velocity become default values
  EXPECT_DOUBLE_EQ(kMaxLinearVelocityDefault, param.max_linear_velocity);
  EXPECT_DOUBLE_EQ(kMinLinearVelocityDefault, param.min_linear_velocity);
}

/// NearestPathPointSearcher test fixture
class PathTransitVelocityCalculatorTest : public ::testing::Test {
 public:
  PathTransitVelocityCalculatorTest() {
  }
  void SetUp() {
    path_transit_velocity_calculator_ = std::make_shared<PathTransitVelocityCalculator>(
        PathTransitVelocityCalculator::Parameter(
            kMaxLinearVelocityDefault, kMinLinearVelocityDefault, kMaxAngularVelocityDefault,
            kMaxLinearAccelerationDefault, kMaxLinearDecelerationDefault,
            kMaxAngularAccelerationDefault, kMaxAngularDecelerationDefault,
            kTransitVelocityAngularVelocityRatioDefault));
  }

 protected:
  PathTransitVelocityCalculator::Ptr path_transit_velocity_calculator_;
};


/// CalculatePathTransitVelocity test
/// In the case of a straight path, there is no restriction on transit velocity
TEST_F(PathTransitVelocityCalculatorTest, StraightPath) {
  // setup
  PathInfo path_info;
  // Generate a straight path
  for (int32_t i = 0; i < kStraightPathPointNum; ++i) {
    path_info.splined_path.push_back(Pose2d(i * kPathInterval, 0.0, 0.0));
    path_info.splined_path_curvatures.push_back(0.0);
  }
  // exercise
  path_transit_velocity_calculator_->CalculatePathTransitVelocity(path_info);
  // verify
  // All transit velocities are at the maximum translational velocity
  for (int32_t i = 0; i < path_info.splined_path.size(); ++i) {
    const double transit_velocity = path_transit_velocity_calculator_->GetPathTransitVelocity(i);
    EXPECT_DOUBLE_EQ(kMaxLinearVelocityDefault, transit_velocity);
  }
}

/// CalculatePathTransitVelocity test
/// If the curve is gentle, there is no restriction on transit velocity
TEST_F(PathTransitVelocityCalculatorTest, GentleCurvePath) {
  // Make the curvature of the curve slightly smaller than the threshold where transit velocity restriction applies
  const double curvature = kMaxAngularVelocityDefault * kTransitVelocityAngularVelocityRatioDefault /
      kMaxLinearVelocityDefault - kEpsilon;
  const double radius = 1.0 / curvature;
  PathInfo path_info;
  CreateArcPath(radius, Pose2d(radius, 0.0, 0.0), kPathInterval, M_PI, path_info.splined_path);
  std::vector<double> curvatures(path_info.splined_path.size(), curvature);
  path_info.splined_path_curvatures = curvatures;

  // exercise
  path_transit_velocity_calculator_->CalculatePathTransitVelocity(path_info);
  // All transit velocities are at the maximum translational velocity
  for (int32_t i = 0; i < path_info.splined_path.size(); ++i) {
    const double transit_velocity = path_transit_velocity_calculator_->GetPathTransitVelocity(i);
    EXPECT_DOUBLE_EQ(kMaxLinearVelocityDefault, transit_velocity);
  }
}


/// CalculatePathTransitVelocity test
/// In the case of a curved path, transit velocity is restricted
/// The greater the curvature of the curve, the more the transit velocity decelerates
TEST_F(PathTransitVelocityCalculatorTest, CurvePath) {
  // Make the curvature of the curve slightly larger than the threshold where transit velocity restriction applies
  double curvature = (kMaxAngularVelocityDefault * kTransitVelocityAngularVelocityRatioDefault) /
      kMaxLinearVelocityDefault + kEpsilon;
  double radius = 1.0 / curvature;
  PathInfo path_info;
  CreateArcPath(radius, Pose2d(radius, 0.0, 0.0), kPathInterval, M_PI, path_info.splined_path);
  std::vector<double> small_curvatures(path_info.splined_path.size(), curvature);
  path_info.splined_path_curvatures = small_curvatures;

  // exercise
  path_transit_velocity_calculator_->CalculatePathTransitVelocity(path_info);
  // verify
  // Transit velocity is smaller than the maximum translational velocity
  double small_curvature_transit_velocity;
  for (int32_t i = 0; i < path_info.splined_path.size(); ++i) {
    small_curvature_transit_velocity = path_transit_velocity_calculator_->GetPathTransitVelocity(i);
    EXPECT_GT(kMaxLinearVelocityDefault, small_curvature_transit_velocity);
  }

  // setup
  // Make the curvature of the curve slightly smaller than the threshold where transit velocity becomes the minimum velocity
  curvature = kMaxAngularVelocityDefault * kTransitVelocityAngularVelocityRatioDefault /
      kMinLinearVelocityDefault - kEpsilon;
  radius = 1.0 / curvature;
  path_info.splined_path.clear();
  path_info.splined_path_curvatures.clear();
  CreateArcPath(radius, Pose2d(radius, 0.0, 0.0), kPathInterval, M_PI, path_info.splined_path);
  std::vector<double> large_curvatures(path_info.splined_path.size(), curvature);
  path_info.splined_path_curvatures = large_curvatures;

  // exercise
  path_transit_velocity_calculator_->CalculatePathTransitVelocity(path_info);
  // verify
  // Transit velocity is smaller than when the curvature was smaller
  // Greater than the minimum velocity
  for (int32_t i = 0; i < path_info.splined_path.size(); ++i) {
    const double large_curvature_transit_velocity = path_transit_velocity_calculator_->GetPathTransitVelocity(i);
    EXPECT_GT(small_curvature_transit_velocity, large_curvature_transit_velocity);
    EXPECT_LT(kMinLinearVelocityDefault, large_curvature_transit_velocity);
  }
}

/// CalculatePathTransitVelocity test
/// If the transit velocity for curvature falls below the minimum velocity, it is rounded to the minimum velocity
TEST_F(PathTransitVelocityCalculatorTest, SharpCurvePath) {
  // Make the curvature of the curve slightly larger than the threshold where transit velocity becomes the minimum velocity
  double curvature = (kMaxAngularVelocityDefault * kTransitVelocityAngularVelocityRatioDefault) /
      kMinLinearVelocityDefault + kEpsilon;
  double radius = 1.0 / curvature;
  PathInfo path_info;
  CreateArcPath(radius, Pose2d(radius, 0.0, 0.0), kPathInterval, M_PI, path_info.splined_path);
  std::vector<double> curvatures(path_info.splined_path.size(), curvature);
  path_info.splined_path_curvatures = curvatures;

  // exercise
  path_transit_velocity_calculator_->CalculatePathTransitVelocity(path_info);
  // verify
  // Transit velocity is at the minimum velocity
  double transit_velocity;
  for (int32_t i = 0; i < path_info.splined_path.size(); ++i) {
    transit_velocity = path_transit_velocity_calculator_->GetPathTransitVelocity(i);
    EXPECT_DOUBLE_EQ(kMinLinearVelocityDefault, transit_velocity);
  }
}

/// CalculatePathTransitVelocity test
/// In the case of a path entering a curve that decelerates from a straight line to the minimum velocity, the maximum deceleration is applied to prevent sudden deceleration
/// Deceleration starts from the straight section, and when entering the curve, it is decelerated to the minimum velocity
TEST_F(PathTransitVelocityCalculatorTest, SuddenDecelerationPath) {
  // setup
  // Set the maximum deceleration value small for testing
  const double max_linear_deceleration = 0.2;
  path_transit_velocity_calculator_.reset(new PathTransitVelocityCalculator(
        PathTransitVelocityCalculator::Parameter(
        kMaxLinearVelocityDefault, kMinLinearVelocityDefault, kMaxAngularVelocityDefault,
        kMaxLinearAccelerationDefault, max_linear_deceleration,
        kMaxAngularAccelerationDefault, kMaxAngularDecelerationDefault,
        kTransitVelocityAngularVelocityRatioDefault)));

  // Calculate the curvature and radius of the curve, make the curvature slightly larger than the threshold where transit velocity becomes the minimum velocity
  const double curvature = (kMaxAngularVelocityDefault * kTransitVelocityAngularVelocityRatioDefault) /
      kMinLinearVelocityDefault + kEpsilon;
  const double radius = 1.0 / curvature;
  // Generate a straight path
  PoseSeq straight_path;
  std::vector<double> straight_path_curvatures;
  for (int32_t i = 1; i <= kStraightPathPointNum; ++i) {
    straight_path.push_back(Pose2d(i * kPathInterval, 0.0, 0.0));
    straight_path_curvatures.push_back(0.0);
  }

  // Generate a curve that decelerates to the minimum velocity from the final point of the straight path
  PoseSeq curve_path;
  CreateArcPath(radius, straight_path.back(), kPathInterval, M_PI, curve_path);
  std::vector<double> curve_path_curvatures(curve_path.size(), curvature);

  // Connect the two paths in the order of straight -> curve
  PathInfo path_info;
  path_info.splined_path = straight_path;
  path_info.splined_path_curvatures = straight_path_curvatures;
  path_info.splined_path.insert(path_info.splined_path.end(), curve_path.begin(), curve_path.end());
  path_info.splined_path_curvatures.insert(path_info.splined_path_curvatures.end(),
                                           curve_path_curvatures.begin(), curve_path_curvatures.end());

  // exercise
  path_transit_velocity_calculator_->CalculatePathTransitVelocity(path_info);
  // verify
  double prev_velocity = path_transit_velocity_calculator_->GetPathTransitVelocity(0);
  // Deceleration starts in the straight section and gradually decelerates until the start point of the curve
  for (int32_t i = 1; i < straight_path.size() + 1; ++i) {
    const double transit_velocity = path_transit_velocity_calculator_->GetPathTransitVelocity(i);
    const double velocity_diff = transit_velocity - prev_velocity;
    // Theoretical time taken to move between path points
    const double time_diff = kPathInterval / prev_velocity;
    // Deceleration
    const double linear_deceleration = velocity_diff / time_diff;
    // Deceleration is less than or equal to 0
    EXPECT_LE(linear_deceleration, 0.0);
    // Not decelerating more than the set value (may exceed within the error range, so judged by kEpsilon)
    EXPECT_LT(fabs(linear_deceleration) - max_linear_deceleration, kEpsilon);
    prev_velocity = transit_velocity;
  }
  for (int32_t i = straight_path.size(); i < path_info.splined_path.size(); ++i) {
    // After entering the curve, it is at the minimum velocity
    const double curve_transit_velocity = path_transit_velocity_calculator_->GetPathTransitVelocity(i);
    EXPECT_DOUBLE_EQ(kMinLinearVelocityDefault, curve_transit_velocity);
  }
}

/// CalculatePathTransitVelocity test
/// In the case of a path that suddenly becomes straight from a curve, the maximum acceleration is applied to prevent sudden acceleration
TEST_F(PathTransitVelocityCalculatorTest, SuddenAccelerationPath) {
  // setup
  // Set the maximum acceleration value small for testing
  const double max_linear_acceleration = 0.2;
  path_transit_velocity_calculator_.reset(new PathTransitVelocityCalculator(
        PathTransitVelocityCalculator::Parameter(
        kMaxLinearVelocityDefault, kMinLinearVelocityDefault, kMaxAngularVelocityDefault,
        max_linear_acceleration, kMaxLinearDecelerationDefault,
        kMaxAngularAccelerationDefault, kMaxAngularDecelerationDefault,
        kTransitVelocityAngularVelocityRatioDefault)));

  // Calculate the curvature and radius of the curve, make the curvature slightly larger than the threshold where transit velocity becomes the minimum velocity
  const double curvature = (kMaxAngularVelocityDefault * kTransitVelocityAngularVelocityRatioDefault) /
      kMinLinearVelocityDefault + kEpsilon;
  const double radius = 1.0 / curvature;
  // Generate a curve that decelerates to the minimum velocity
  PoseSeq curve_path;
  CreateArcPath(radius, Pose2d(0.0, 0.0, 0.0), kPathInterval, M_PI, curve_path);
  std::vector<double> curve_path_curvatures(curve_path.size(), curvature);

  // Generate a straight path so that the straight path connects to the end position of the curve
  PoseSeq straight_path;
  std::vector<double> straight_path_curvatures;
  for (int32_t i = 1; i <= kStraightPathPointNum; ++i) {
    straight_path.push_back(Pose2d(curve_path.back().x() - i * kPathInterval,
                                   curve_path.back().y(),
                                   curve_path.back().theta()));
    straight_path_curvatures.push_back(0.0);
  }

  // Connect the two paths in the order of curve -> straight
  PathInfo path_info;
  path_info.splined_path = curve_path;
  path_info.splined_path_curvatures = curve_path_curvatures;
  path_info.splined_path.insert(path_info.splined_path.end(), straight_path.begin(), straight_path.end());
  path_info.splined_path_curvatures.insert(path_info.splined_path_curvatures.end(),
                                           straight_path_curvatures.begin(), straight_path_curvatures.end());
  // exercise
  path_transit_velocity_calculator_->CalculatePathTransitVelocity(path_info);
  // verify
  for (int32_t i = 0; i < curve_path.size(); ++i) {
    // During the curve, it is at the minimum velocity
    const double curve_transit_velocity = path_transit_velocity_calculator_->GetPathTransitVelocity(i);
    EXPECT_DOUBLE_EQ(kMinLinearVelocityDefault, curve_transit_velocity);
  }
  double prev_velocity = kMinLinearVelocityDefault;
  // Acceleration starts in the straight section and gradually accelerates to the maximum velocity
  for (int32_t i = curve_path.size(); i < path_info.splined_path.size(); ++i) {
    const double transit_velocity = path_transit_velocity_calculator_->GetPathTransitVelocity(i);
    const double velocity_diff = transit_velocity - prev_velocity;
    // Theoretical time taken to move between path points
    const double time_diff = kPathInterval / prev_velocity;
    // Acceleration
    const double linear_acceleration = velocity_diff / time_diff;
    // Acceleration is greater than or equal to 0
    EXPECT_GE(linear_acceleration, 0.0);
    // Not accelerating more than the set value (may exceed within the error range, so judged by kEpsilon)
    EXPECT_LT(fabs(linear_acceleration) - max_linear_acceleration, kEpsilon);
    // Does not exceed the maximum velocity
    EXPECT_LE(transit_velocity, kMaxLinearVelocityDefault);
    prev_velocity = transit_velocity;
  }
}
}  // namespace tmc_base_path_follower

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
