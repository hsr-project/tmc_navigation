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
#include <cmath>
#include <math.h>
#include <limits>
#include <optional>

#include <angles/angles.h>
#include <gtest/gtest.h>

#include <tmc_base_path_follower/common.hpp>
#include <tmc_base_path_follower/velocity_calculator_omni.hpp>

#include "test_utils.hpp"

namespace {
// Test path length [m]
constexpr double kTestPathLength = 5.0;
// Test path waypoint interval [m]
constexpr double kTestPathInterval = 0.1;
// Direction of the goal in the test path [rad]
// Reverse the direction of the path. If it is exactly 180° opposite, it is unclear which way to rotate, so make it closer to counterclockwise
constexpr double kGoalAngle = (135.0 / 180. * M_PI);
// Direction of the goal from the self-position for goal area approach test [rad]
constexpr double kGoalDirection = (135.0 / 180.0 * M_PI);
// Distance from the self-position to the goal for goal area approach test [m]
constexpr double kGoalDistance = 0.5;
// Radius of circular path for turning speed test
constexpr double kTestR = 5.0;
// Speed calculation cycle [Hz]
constexpr double kFrequency = 100.0;
// Speed calculation time interval [s]
constexpr double kInterval = 1.0 / kFrequency;
}  // anonymous namespace


namespace tmc_base_path_follower {

/// Parameter test
/// Ability to generate parameters
TEST(OmniVelocityCalculatorParameterTest, SetInputParameter) {
  // exercise
  // Input parameters (set so that max_linear_velocity > velocity_margin)
  OmniVelocityCalculator::Parameter param(0.1, 0.2, 0.3, 0.4, 0.5, 0.06, 0.7, 0.8, 0.9, 1.0);

  // verify
  // Check if the values are set as input
  EXPECT_DOUBLE_EQ(0.1, param.max_linear_velocity);
  EXPECT_DOUBLE_EQ(0.2, param.max_angular_velocity);
  EXPECT_DOUBLE_EQ(0.3, param.max_linear_acceleration);
  EXPECT_DOUBLE_EQ(0.4, param.max_angular_acceleration);
  EXPECT_DOUBLE_EQ(0.5, param.goal_deceleration);
  EXPECT_DOUBLE_EQ(0.06, param.velocity_margin);
  EXPECT_DOUBLE_EQ(0.7, param.path_length_threshold);
  EXPECT_DOUBLE_EQ(0.8, param.linear_p_gain);
  EXPECT_DOUBLE_EQ(0.9, param.angular_p_gain);
  EXPECT_DOUBLE_EQ(1.0, param.goal_angle_gain);
}

/// Parameter test
/// If invalid values are specified, default values are generated
TEST(OmniVelocityCalculatorParameterTest, SetDefaultParameter) {
  // exercise
  // Set all parameters to invalid values
  OmniVelocityCalculator::Parameter param(-1.0, -1.0, -1.0, -1.0, -1.0, -0.1, -1.0, -1.0, -1.0, -1.0);

  // verify
  // Check if default values are set
  EXPECT_DOUBLE_EQ(kMaxLinearVelocityDefault, param.max_linear_velocity);
  EXPECT_DOUBLE_EQ(kMaxAngularVelocityDefault, param.max_angular_velocity);
  EXPECT_DOUBLE_EQ(kMaxLinearAccelerationDefault, param.max_linear_acceleration);
  EXPECT_DOUBLE_EQ(kMaxAngularAccelerationDefault, param.max_angular_acceleration);
  EXPECT_DOUBLE_EQ(kGoalDecelerationDefault, param.goal_deceleration);
  EXPECT_DOUBLE_EQ(kVelocityMarginDefault, param.velocity_margin);
  EXPECT_DOUBLE_EQ(kPathLengthThresholdDefault, param.path_length_threshold);
  EXPECT_DOUBLE_EQ(kLinearPGainDefault, param.linear_p_gain);
  EXPECT_DOUBLE_EQ(kAngularPGainDefault, param.angular_p_gain);
  EXPECT_DOUBLE_EQ(kGoalAngleGainDefault, param.goal_angle_gain);
}

/// Parameter test
/// If the maximum translational velocity is less than the velocity margin, the default margin value is used
TEST(OmniVelocityCalculatorParameterTest, LowerMaxVelocityThanMergin) {
  // exercise
  // Input parameters (set so that max_linear_velocity < velocity_margin)
  OmniVelocityCalculator::Parameter param(kVelocityMarginDefault + kEpsilon, 0.2, 0.3, 0.4, 0.5,
                                          kVelocityMarginDefault + kEpsilon * 2.0, 0.7, 0.8, 0.9, 1.0);

  // verify
  // Velocity margin is set to the default value
  EXPECT_DOUBLE_EQ(kVelocityMarginDefault, param.velocity_margin);
}

/// Parameter test
/// If the maximum translational velocity is less than both the velocity margin and the default margin value
/// The maximum translational velocity and margin are set to default values
TEST(OmniVelocityCalculatorParameterTest, LowerMaxVelocityThanMerginDefault) {
  // exercise
  // Input parameters (set so that max_linear_velocity < velocity_margin, kVelocityMarginDefault)
  OmniVelocityCalculator::Parameter param(kVelocityMarginDefault - kEpsilon * 2.0, 0.2, 0.3, 0.4, 0.5,
                                          kVelocityMarginDefault - kEpsilon, 0.7, 0.8, 0.9, 1.0);

  // verify
  // Velocity margin is set to the default value
  EXPECT_DOUBLE_EQ(kMaxLinearVelocityDefault, param.max_linear_velocity);
  EXPECT_DOUBLE_EQ(kVelocityMarginDefault, param.velocity_margin);
}

/// OmniVelocityCalculator test fixture
class OmniVelocityCalculatorTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    // Generate target instance
    velocity_calculator_ = std::make_shared<OmniVelocityCalculator>(
        OmniVelocityCalculator::Parameter(kMaxLinearVelocityDefault, kMaxAngularVelocityDefault,
                                          kMaxLinearAccelerationDefault, kMaxAngularAccelerationDefault,
                                          kGoalDecelerationDefault, kVelocityMarginDefault,
                                          kPathLengthThresholdDefault, kLinearPGainDefault,
                                          kAngularPGainDefault, kGoalAngleGainDefault));
    // Generate test path
    // Generate a straight path with the goal facing the opposite direction
    const int32_t path_points = static_cast<int32_t>(kTestPathLength / kTestPathInterval);
    for (int32_t i = 0; i < path_points; ++i) {
      const double x = kTestPathLength * static_cast<double>(i) / static_cast<double>(path_points - 1);
      path_info_.splined_path.push_back(Pose2d(x, 0.0, 0.0));
      path_info_.splined_path_curvatures.push_back(0.0);
      path_info_.splined_path_left_lengths.push_back(kTestPathLength - x);
    }
    path_info_.splined_path.back().set_theta(kGoalAngle);
  }

  OmniVelocityCalculator::Ptr velocity_calculator_;
  PathInfo path_info_;
};

/// CalculateVelocity test
/// Outputs turning speed according to translational speed and curvature
TEST_F(OmniVelocityCalculatorTest, AngularVelocity) {
  // setup
  // Since the path shape is not considered, only the curvature value is set
  for (double& curvature : path_info_.splined_path_curvatures) {
    curvature = 1.0 / kTestR;
  }
  // Theoretical angular velocity [rad/s] when traveling at maximum speed
  const double expected_angular_velocity = (1.0 / kTestR) * kMaxLinearVelocityDefault;

  // Current position
  const Pose2d current_pose(path_info_.splined_path.front().x(),
                            path_info_.splined_path.front().y(),
                            0.0);
  // Previous velocity Set theoretical value
  Vector3d output_velocity = Vector3d(kMaxLinearVelocityDefault, 0.0, expected_angular_velocity);

  // exercise
  const bool result = velocity_calculator_->CalculateVelocity(
      path_info_, current_pose, 0, output_velocity, kInterval, false, std::nullopt, output_velocity);
  ASSERT_TRUE(result);

  // verify
  // Verify that the output velocity matches the input velocity (= the internally calculated target velocity matches the theoretical value)
  EXPECT_DOUBLE_EQ(expected_angular_velocity, output_velocity(kPoseTheta));
}

/// CalculateVelocity test
/// Within the goal area, velocity is set in the direction of approaching the goal point regardless of the path
TEST_F(OmniVelocityCalculatorTest, MoveToGoalPoint) {
  // setup
  // Current position Set 45° ahead of the goal
  const Pose2d current_pose(path_info_.splined_path.back().x() + cos(kGoalDirection + M_PI) * kGoalDistance,
                            path_info_.splined_path.back().y() + sin(kGoalDirection + M_PI) * kGoalDistance,
                            0.0);
  // Previous velocity Input as 0 since only direction is considered
  Vector3d last_velocity = Vector3d::Zero();

  // exercise
  Vector3d output_velocity;
  const bool result = velocity_calculator_->CalculateVelocity(path_info_, current_pose,
                                                              path_info_.splined_path.size() - 1,
                                                              last_velocity, kInterval, true,
                                                              std::nullopt, output_velocity);

  // verify
  ASSERT_TRUE(result);
  // Verify that the output velocity is directed toward the goal
  const double velocity_direction = atan2(output_velocity(kPoseY), output_velocity(kPoseX));
  EXPECT_DOUBLE_EQ(kGoalDirection, velocity_direction);
}

/// CalculateVelocity test
/// If the goal is significantly exceeded (outside the goal judgment distance and the nearest point is the goal), it returns failure and stops
TEST_F(OmniVelocityCalculatorTest, StopFarFromGoal) {
  // setup
  // Current position Set 45° ahead of the goal
  const Pose2d current_pose(path_info_.splined_path.back().x() + cos(kGoalDirection + M_PI) * kGoalDistance,
                            path_info_.splined_path.back().y() + sin(kGoalDirection + M_PI) * kGoalDistance,
                            0.0);
  // Previous velocity
  Vector3d last_velocity = Vector3d::Zero();

  // exercise
  Vector3d output_velocity;
  const bool result = velocity_calculator_->CalculateVelocity(path_info_, current_pose,
                                                              path_info_.splined_path.size() - 1,
                                                              last_velocity, kInterval, false,
                                                              std::nullopt, output_velocity);

  // verify
  // Verify that failure is returned
  ASSERT_FALSE(result);
  // Verify that the output velocity is 0
  EXPECT_DOUBLE_EQ(0.0, output_velocity(kPoseX));
  EXPECT_DOUBLE_EQ(0.0, output_velocity(kPoseY));
  EXPECT_DOUBLE_EQ(0.0, output_velocity(kPoseTheta));
}

/// CalculateVelocity test
/// Within the goal area, if the maximum translational velocity > deceleration speed to the goal, it is limited to the smaller value
TEST_F(OmniVelocityCalculatorTest, ChooseCalculatedVeloctyInGoalArea) {
  // setup
  // Threshold where target velocity matches maximum velocity
  const double threshold_distance = kMaxLinearVelocityDefault / kLinearPGainDefault;
  const double threshold_angle = kMaxAngularVelocityDefault / kAngularPGainDefault;
  // Current position Set slightly inside the threshold
  // Since setting both translation and angle simultaneously makes it difficult to verify due to rotation in velocity, verify them separately
  const Pose2d current_pose_xy(path_info_.splined_path.back().x() - threshold_distance + kEpsilon,
                               path_info_.splined_path.back().y(),
                               0.0);
  const Pose2d current_pose_t(path_info_.splined_path.back().x(),
                              path_info_.splined_path.back().y(),
                              kGoalAngle - threshold_angle + kEpsilon);
  // Previous velocity Assume moving straight toward the goal at maximum speed
  Vector3d last_velocity = Vector3d(kMaxLinearVelocityDefault, 0.0, kMaxAngularVelocityDefault);

  // exercise
  Vector3d output_velocity_xy;
  const bool result_xy = velocity_calculator_->CalculateVelocity(path_info_, current_pose_xy,
                                                                 path_info_.splined_path.size() - 1,
                                                                 last_velocity, kInterval, true,
                                                                 std::nullopt, output_velocity_xy);
  Vector3d output_velocity_t;
  const bool result_t = velocity_calculator_->CalculateVelocity(path_info_, current_pose_t,
                                                                path_info_.splined_path.size() - 1,
                                                                last_velocity, kInterval, true,
                                                                std::nullopt, output_velocity_t);

  // verify
  ASSERT_TRUE(result_xy);
  ASSERT_TRUE(result_t);
  // Verify that translational and turning speeds are limited
  EXPECT_LT(output_velocity_xy.head(2).norm(), kMaxLinearVelocityDefault);
  EXPECT_LT(output_velocity_t(kPoseTheta), kMaxAngularVelocityDefault);
}

/// CalculateVelocity test
/// Within the goal area, if the maximum translational velocity < deceleration speed to the goal, it is limited to the smaller value
TEST_F(OmniVelocityCalculatorTest, ChooseMaxVeloctyInGoalArea) {
  // setup
  // Threshold where target velocity matches maximum velocity
  const double threshold_distance = kMaxLinearVelocityDefault / kLinearPGainDefault;
  const double threshold_angle = kMaxAngularVelocityDefault / kAngularPGainDefault;
  // Current position Set slightly outside the threshold
  // Since setting both translation and angle simultaneously makes it difficult to verify due to rotation in velocity, verify them separately
  const Pose2d current_pose_xy(path_info_.splined_path.back().x() - threshold_distance - kEpsilon,
                               path_info_.splined_path.back().y(),
                               0.0);
  const Pose2d current_pose_t(path_info_.splined_path.back().x(),
                              path_info_.splined_path.back().y(),
                              kGoalAngle - threshold_angle - kEpsilon);
  // Previous velocity Assume moving straight toward the goal at maximum speed
  Vector3d last_velocity = Vector3d(kMaxLinearVelocityDefault, 0.0, kMaxAngularVelocityDefault);

  // exercise
  Vector3d output_velocity_xy;
  const bool result_xy = velocity_calculator_->CalculateVelocity(path_info_, current_pose_xy,
                                                                 path_info_.splined_path.size() - 1,
                                                                 last_velocity, kInterval, true,
                                                                 std::nullopt, output_velocity_xy);
  Vector3d output_velocity_t;
  const bool result_t = velocity_calculator_->CalculateVelocity(path_info_, current_pose_t,
                                                                path_info_.splined_path.size() - 1,
                                                                last_velocity, kInterval, true,
                                                                std::nullopt, output_velocity_t);

  // verify
  ASSERT_TRUE(result_xy);
  ASSERT_TRUE(result_t);
  // Verify that both translational and turning speeds are at maximum velocity
  EXPECT_DOUBLE_EQ(kMaxLinearVelocityDefault, output_velocity_xy.head(2).norm());
  EXPECT_DOUBLE_EQ(kMaxAngularVelocityDefault, output_velocity_t(kPoseTheta));
}

/// CalculateVelocity test
/// Within the goal area, the closer to the goal, the smaller the velocity
TEST_F(OmniVelocityCalculatorTest, NearerGoalSlowerVelocityInGoalArea) {
  // setup
  // Threshold where target velocity matches maximum velocity
  const double threshold_distance = kMaxLinearVelocityDefault / kLinearPGainDefault;
  const double threshold_angle = kMaxAngularVelocityDefault / kAngularPGainDefault;
  // Prepare two cases: one close to the goal and one far from the goal
  const Pose2d current_pose_near(path_info_.splined_path.back().x() - threshold_distance / 4.0,
                                 path_info_.splined_path.back().y(),
                                 kGoalAngle - threshold_angle / 4.0);
  const Pose2d current_pose_far(path_info_.splined_path.back().x() - threshold_distance / 2.0,
                                path_info_.splined_path.back().y(),
                                kGoalAngle - threshold_angle / 2.0);
  // Previous velocity Assume moving straight toward the goal at maximum speed
  Vector3d output_velocity_near = Vector3d(kMaxLinearVelocityDefault, 0.0, 0.0);
  Vector3d output_velocity_far = Vector3d(kMaxLinearVelocityDefault, 0.0, 0.0);

  // exercise
  // Since the target velocity does not appear immediately due to acceleration limits, call repeatedly
  const uint32_t iterations =
      static_cast<uint32_t>(kMaxLinearVelocityDefault / kMaxLinearAccelerationDefault * kFrequency);
  for (uint32_t i = 0; i < iterations; ++i) {
    const bool result_near = velocity_calculator_->CalculateVelocity(path_info_, current_pose_near,
                                                                     path_info_.splined_path.size() - 1,
                                                                     output_velocity_near, kInterval, true,
                                                                     std::nullopt, output_velocity_near);
    const bool result_far = velocity_calculator_->CalculateVelocity(path_info_, current_pose_far,
                                                                    path_info_.splined_path.size() - 1,
                                                                    output_velocity_far, kInterval, true,
                                                                    std::nullopt, output_velocity_far);
    ASSERT_TRUE(result_near);
    ASSERT_TRUE(result_far);
  }

  // verify
  // Verify that the closer to the goal, the smaller the velocity
  EXPECT_LT(output_velocity_near.head(2).norm(), output_velocity_far.head(2).norm());
  EXPECT_LT(std::abs(output_velocity_near(kPoseTheta)), std::abs(output_velocity_far(kPoseTheta)));
}

/// CalculateVelocity test
/// Outside the goal area, if the maximum translational velocity > deceleration speed to the goal, it is limited to the smaller value
TEST_F(OmniVelocityCalculatorTest, ChooseCalculatedVeloctyOutGoalArea) {
  // setup
  // Threshold where target velocity matches maximum velocity
  const double threshold_distance = (kMaxLinearVelocityDefault - kVelocityMarginDefault) / kGoalDecelerationDefault;
  // Current position Set slightly inside the threshold
  const Pose2d current_pose(path_info_.splined_path.back().x() - threshold_distance + kEpsilon,
                            path_info_.splined_path.back().y(), 0.0);
  // Previous velocity Assume moving straight toward the goal at maximum speed
  Vector3d last_velocity = Vector3d(kMaxLinearVelocityDefault, 0.0, 0.0);

  // exercise
  Vector3d output_velocity;
  const bool result = velocity_calculator_->CalculateVelocity(path_info_, current_pose,
                                                              path_info_.splined_path.size() - 2,
                                                              last_velocity, kInterval, false,
                                                              std::nullopt, output_velocity);

  // verify
  ASSERT_TRUE(result);
  // Verify that translational speed is limited
  EXPECT_LT(output_velocity(kPoseX), kMaxLinearVelocityDefault);
}

/// CalculateVelocity test
/// Outside the goal area, if the maximum translational velocity < deceleration speed to the goal, it is limited to the smaller value
TEST_F(OmniVelocityCalculatorTest, ChooseMaxVeloctyOutGoalArea) {
  // setup
  // Threshold where target velocity matches maximum velocity
  const double threshold_distance = (kMaxLinearVelocityDefault - kVelocityMarginDefault) / kGoalDecelerationDefault;
  // Current position Set slightly outside the threshold
  const Pose2d current_pose(path_info_.splined_path.back().x() - threshold_distance - kEpsilon,
                            path_info_.splined_path.back().y(), 0.0);
  // Previous velocity Assume moving straight toward the goal at maximum speed
  Vector3d last_velocity = Vector3d(kMaxLinearVelocityDefault, 0.0, 0.0);

  // exercise
  Vector3d output_velocity;
  const bool result = velocity_calculator_->CalculateVelocity(path_info_, current_pose,
                                                              path_info_.splined_path.size() - 2,
                                                              last_velocity, kInterval, false,
                                                              std::nullopt, output_velocity);

  // verify
  ASSERT_TRUE(result);
  // Verify that translational speed is at maximum velocity
  EXPECT_DOUBLE_EQ(kMaxLinearVelocityDefault, output_velocity(kPoseX));
}

/// CalculateVelocity test
/// Outside the goal area, if a passing speed is set, it is limited to the passing speed
TEST_F(OmniVelocityCalculatorTest, LimitTransitVelocity) {
  // setup
  // Threshold where target velocity matches maximum velocity
  const double threshold_distance = (kMaxLinearVelocityDefault - kVelocityMarginDefault) / kGoalDecelerationDefault;
  // Current position Set slightly outside the threshold
  const Pose2d current_pose(path_info_.splined_path.back().x() - threshold_distance - kEpsilon,
                            path_info_.splined_path.back().y(), 0.0);
  // Previous velocity Assume moving straight toward the goal at maximum speed
  Vector3d last_velocity = Vector3d(kMaxLinearVelocityDefault, 0.0, 0.0);
  // Set passing speed slightly lower than the maximum translational speed
  const double transit_velocity = kMaxLinearVelocityDefault - kEpsilon;
  // exercise
  Vector3d output_velocity;
  const bool result = velocity_calculator_->CalculateVelocity(path_info_, current_pose,
                                                              path_info_.splined_path.size() - 2,
                                                              last_velocity, kInterval, false,
                                                              transit_velocity, output_velocity);

  // verify
  ASSERT_TRUE(result);
  // Verify that translational speed matches the passing speed
  EXPECT_DOUBLE_EQ(transit_velocity, output_velocity(kPoseX));
}

/// CalculateVelocity test
/// Outside the goal area, the closer to the goal, the smaller the velocity
TEST_F(OmniVelocityCalculatorTest, NearerGoalSlowerVelocityOutGoalArea) {
  // setup
  // Threshold where target velocity matches maximum velocity
  const double threshold_distance = (kMaxLinearVelocityDefault - kVelocityMarginDefault) / kGoalDecelerationDefault;
  // Prepare two cases: one close to the goal and one far from the goal
  const Pose2d current_pose_near(path_info_.splined_path.back().x() - threshold_distance / 4.0,
                                 path_info_.splined_path.back().y(),
                                 0.0);
  const Pose2d current_pose_far(path_info_.splined_path.back().x() - threshold_distance / 2.0,
                                path_info_.splined_path.back().y(),
                                0.0);
  // Previous velocity Assume moving straight toward the goal at maximum speed
  Vector3d output_velocity_near = Vector3d(kMaxLinearVelocityDefault, 0.0, 0.0);
  Vector3d output_velocity_far = Vector3d(kMaxLinearVelocityDefault, 0.0, 0.0);

  // exercise
  // Since the target velocity does not appear immediately due to acceleration limits, call repeatedly
  const uint32_t iterations =
      static_cast<uint32_t>(kMaxLinearVelocityDefault / kMaxLinearAccelerationDefault * kFrequency);
  for (uint32_t i = 0; i < iterations; ++i) {
    const bool result_near = velocity_calculator_->CalculateVelocity(path_info_, current_pose_near,
                                                                  path_info_.splined_path.size() - 2,
                                                                  output_velocity_near, kInterval, false,
                                                                  std::nullopt, output_velocity_near);
    const bool result_far = velocity_calculator_->CalculateVelocity(path_info_, current_pose_far,
                                                                  path_info_.splined_path.size() - 2,
                                                                  output_velocity_far, kInterval, false,
                                                                  std::nullopt, output_velocity_far);
    ASSERT_TRUE(result_near);
    ASSERT_TRUE(result_far);
  }

  // verify
  // Verify that the closer to the goal, the smaller the velocity
  EXPECT_LT(output_velocity_near.head(2).norm(), output_velocity_far.head(2).norm());
}

/// CalculateVelocity test
/// If the distance to the goal is within a certain range, the turning speed of the vehicle is set to face the direction of the goal
TEST_F(OmniVelocityCalculatorTest, TurnToGoalPoint) {
  // setup
  // Current position Place slightly inside the threshold and face the direction between the goal and the path
  const Pose2d current_pose(path_info_.splined_path.back().x() - kPathLengthThresholdDefault + kTestPathInterval,
                            path_info_.splined_path.back().y(), kGoalAngle / 2.0);
  const uint32_t path_index =
      static_cast<uint32_t>((kTestPathLength - kPathLengthThresholdDefault) / kTestPathInterval) + 1;
  // Previous velocity
  Vector3d last_velocity = Vector3d::Zero();

  // exercise
  Vector3d output_velocity;
  const bool result = velocity_calculator_->CalculateVelocity(path_info_, current_pose,
                                                              path_index, last_velocity,
                                                              kInterval, false, std::nullopt, output_velocity);

  // verify
  ASSERT_TRUE(result);
  // Turning speed is output to face the goal
  EXPECT_GT(output_velocity(kPoseTheta), 0.0);
}

/// CalculateVelocity test
/// If the distance to the goal is beyond a certain range, the turning speed of the vehicle is set to face the direction of the nearest point
TEST_F(OmniVelocityCalculatorTest, TurnToNearestPoint) {
  // setup
  // Current position Place slightly outside the threshold and face the direction between the goal and the path
  const Pose2d current_pose(path_info_.splined_path.back().x() - kPathLengthThresholdDefault - kTestPathInterval,
                            path_info_.splined_path.back().y(), kGoalAngle / 2.0);
  const uint32_t path_index =
      static_cast<uint32_t>((kTestPathLength - kPathLengthThresholdDefault) / kTestPathInterval) - 1;
  // Previous velocity
  Vector3d last_velocity = Vector3d::Zero();

  // exercise
  Vector3d output_velocity;
  const bool result = velocity_calculator_->CalculateVelocity(path_info_, current_pose,
                                                              path_index, last_velocity,
                                                              kInterval, false, std::nullopt, output_velocity);

  // verify
  ASSERT_TRUE(result);
  // Turning speed is output to face the path
  EXPECT_LT(output_velocity(kPoseTheta), 0.0);
}

/// CalculateVelocity test
/// X, Y, and turning angle speeds are limited
TEST_F(OmniVelocityCalculatorTest, LimitVelocities) {
  // setup
  // Current position Place at a position and orientation deviated from the path, so that x, y, and t are each controlled
  const Pose2d current_pose(path_info_.splined_path.front().x(),
                            path_info_.splined_path.front().y() - kTestPathInterval,
                            M_PI / 4.0);
  // Previous velocity Set a speed that attempts to exceed the maximum speed toward the target position
  Vector3d last_velocity = Vector3d(kMaxLinearVelocityDefault * 2.0,
                                    kMaxLinearVelocityDefault * 2.0,
                                    -kMaxAngularVelocityDefault * 2.0);

  // exercise
  Vector3d output_velocity;
  const bool result = velocity_calculator_->CalculateVelocity(path_info_, current_pose,
                                                              0, last_velocity,
                                                              kInterval, false, std::nullopt, output_velocity);

  // verify
  ASSERT_TRUE(result);
  // Speed is at the upper limit
  EXPECT_DOUBLE_EQ(kMaxLinearVelocityDefault, std::abs(output_velocity(kPoseX)));
  EXPECT_DOUBLE_EQ(kMaxLinearVelocityDefault, std::abs(output_velocity(kPoseY)));
  EXPECT_DOUBLE_EQ(kMaxAngularVelocityDefault, std::abs(output_velocity(kPoseTheta)));
}

/// CalculateVelocity test
/// X, Y, and turning angle accelerations are limited
TEST_F(OmniVelocityCalculatorTest, LimitAccelerations) {
  // setup
  // Current position Place at a position and orientation deviated from the path, so that x, y, and t are each controlled
  const Pose2d current_pose(path_info_.splined_path.front().x(),
                            path_info_.splined_path.front().y() - kTestPathInterval,
                            M_PI / 4.0);
  // Previous velocity Check how much acceleration occurs from zero
  Vector3d last_velocity = Vector3d::Zero();

  // exercise
  Vector3d output_velocity;
  const bool result = velocity_calculator_->CalculateVelocity(path_info_, current_pose,
                                                              0, last_velocity,
                                                              kInterval, false, std::nullopt, output_velocity);

  // verify
  ASSERT_TRUE(result);
  // Acceleration is at the upper limit
  EXPECT_DOUBLE_EQ(kMaxLinearAccelerationDefault / kFrequency, std::abs(output_velocity(kPoseX)));
  EXPECT_DOUBLE_EQ(kMaxLinearAccelerationDefault / kFrequency, std::abs(output_velocity(kPoseY)));
  EXPECT_DOUBLE_EQ(kMaxAngularAccelerationDefault / kFrequency, std::abs(output_velocity(kPoseTheta)));
}
}  // namespace tmc_base_path_follower

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
