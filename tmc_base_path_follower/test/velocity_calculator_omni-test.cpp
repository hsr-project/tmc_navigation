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
// Test path interval between path points [m]
constexpr double kTestPathInterval = 0.1;
// Direction of the goal in the test path [rad]
// Make it opposite to the path. Since it's unclear which direction to rotate if it's exactly 180° opposite, make it closer to counterclockwise
constexpr double kGoalAngle = (135.0 / 180. * M_PI);
// Direction of the goal from the self-position for goal area proximity test [rad]
constexpr double kGoalDirection = (135.0 / 180.0 * M_PI);
// Distance from the self-position to the goal for goal area proximity test [m]
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
  // Check if it is set according to the input values
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
/// If invalid values are specified, it is generated with default values
TEST(OmniVelocityCalculatorParameterTest, SetDefaultParameter) {
  // exercise
  // Set all parameters to invalid values
  OmniVelocityCalculator::Parameter param(-1.0, -1.0, -1.0, -1.0, -1.0, -0.1, -1.0, -1.0, -1.0, -1.0);

  // verify
  // Check if it is set with default values
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
/// If the maximum translational speed is less than the speed margin, the default value of the margin is used
TEST(OmniVelocityCalculatorParameterTest, LowerMaxVelocityThanMergin) {
  // exercise
  // Input parameters (set so that max_linear_velocity < velocity_margin)
  OmniVelocityCalculator::Parameter param(kVelocityMarginDefault + kEpsilon, 0.2, 0.3, 0.4, 0.5,
                                          kVelocityMarginDefault + kEpsilon * 2.0, 0.7, 0.8, 0.9, 1.0);

  // verify
  // The speed margin is set to the default value
  EXPECT_DOUBLE_EQ(kVelocityMarginDefault, param.velocity_margin);
}

/// Parameter test
/// If the maximum translational speed is less than both the speed margin and the default value of the margin
/// The maximum translational speed and margin are set to default values
TEST(OmniVelocityCalculatorParameterTest, LowerMaxVelocityThanMerginDefault) {
  // exercise
  // Input parameters (set so that max_linear_velocity < velocity_margin, kVelocityMarginDefault)
  OmniVelocityCalculator::Parameter param(kVelocityMarginDefault - kEpsilon * 2.0, 0.2, 0.3, 0.4, 0.5,
                                          kVelocityMarginDefault - kEpsilon, 0.7, 0.8, 0.9, 1.0);

  // verify
  // The speed margin is set to the default value
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
    // Generate a straight path with only the goal facing the opposite direction
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
/// The turning speed is output according to the translational speed and curvature
TEST_F(OmniVelocityCalculatorTest, AngularVelocity) {
  // setup
  // Since the path shape is not considered, only set the curvature value
  for (double& curvature : path_info_.splined_path_curvatures) {
    curvature = 1.0 / kTestR;
  }
  // Theoretical angular velocity value when traveling at maximum speed [rad/s]
  const double expected_angular_velocity = (1.0 / kTestR) * kMaxLinearVelocityDefault;

  // Current position
  const Pose2d current_pose(path_info_.splined_path.front().x(),
                            path_info_.splined_path.front().y(),
                            0.0);
  // Set the theoretical value of the previous speed
  Vector3d output_velocity = Vector3d(kMaxLinearVelocityDefault, 0.0, expected_angular_velocity);

  // exercise
  const bool result = velocity_calculator_->CalculateVelocity(
      path_info_, current_pose, 0, output_velocity, kInterval, false, std::nullopt, output_velocity);
  ASSERT_TRUE(result);

  // verify
  // The output speed and input speed are the same (= the internally calculated target speed and theoretical value are equal)
  EXPECT_DOUBLE_EQ(expected_angular_velocity, output_velocity(kPoseTheta));
}

/// CalculateVelocity test
/// In the goal area, the speed is set in the direction approaching the goal point regardless of the path
TEST_F(OmniVelocityCalculatorTest, MoveToGoalPoint) {
  // setup
  // Current position set 45° ahead of the goal
  const Pose2d current_pose(path_info_.splined_path.back().x() + cos(kGoalDirection + M_PI) * kGoalDistance,
                            path_info_.splined_path.back().y() + sin(kGoalDirection + M_PI) * kGoalDistance,
                            0.0);
  // Previous speed, only looking at direction, so input as 0
  Vector3d last_velocity = Vector3d::Zero();

  // exercise
  Vector3d output_velocity;
  const bool result = velocity_calculator_->CalculateVelocity(path_info_, current_pose,
                                                              path_info_.splined_path.size() - 1,
                                                              last_velocity, kInterval, true,
                                                              std::nullopt, output_velocity);

  // verify
  ASSERT_TRUE(result);
  // Confirm that the output velocity is facing the direction of the goal
  const double velocity_direction = atan2(output_velocity(kPoseY), output_velocity(kPoseX));
  EXPECT_DOUBLE_EQ(kGoalDirection, velocity_direction);
}

/// CalculateVelocity test
/// If the goal is significantly exceeded (outside the goal judgment distance and the nearest point is the goal), it returns failure and stops
TEST_F(OmniVelocityCalculatorTest, StopFarFromGoal) {
  // setup
  // Current position set 45° ahead of the goal
  const Pose2d current_pose(path_info_.splined_path.back().x() + cos(kGoalDirection + M_PI) * kGoalDistance,
                            path_info_.splined_path.back().y() + sin(kGoalDirection + M_PI) * kGoalDistance,
                            0.0);
  // Previous speed
  Vector3d last_velocity = Vector3d::Zero();

  // exercise
  Vector3d output_velocity;
  const bool result = velocity_calculator_->CalculateVelocity(path_info_, current_pose,
                                                              path_info_.splined_path.size() - 1,
                                                              last_velocity, kInterval, false,
                                                              std::nullopt, output_velocity);

  // verify
  // Confirm that failure is returned
  ASSERT_FALSE(result);
  // Confirm that the output velocity is 0
  EXPECT_DOUBLE_EQ(0.0, output_velocity(kPoseX));
  EXPECT_DOUBLE_EQ(0.0, output_velocity(kPoseY));
  EXPECT_DOUBLE_EQ(0.0, output_velocity(kPoseTheta));
}

/// CalculateVelocity test
/// In the goal area, if the maximum translational speed > speed including deceleration to the goal, it is limited to the smaller one
TEST_F(OmniVelocityCalculatorTest, ChooseCalculatedVeloctyInGoalArea) {
  // setup
  // Threshold where the target speed matches the maximum speed
  const double threshold_distance = kMaxLinearVelocityDefault / kLinearPGainDefault;
  const double threshold_angle = kMaxAngularVelocityDefault / kAngularPGainDefault;
  // Current position set slightly inside the threshold
  // Since setting both translation and angle simultaneously makes it difficult to verify due to rotation in velocity, check each
  const Pose2d current_pose_xy(path_info_.splined_path.back().x() - threshold_distance + kEpsilon,
                               path_info_.splined_path.back().y(),
                               0.0);
  const Pose2d current_pose_t(path_info_.splined_path.back().x(),
                              path_info_.splined_path.back().y(),
                              kGoalAngle - threshold_angle + kEpsilon);
  // Previous speed, assuming moving straight at maximum speed towards the goal
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
  // Confirm that translational speed and turning speed are suppressed
  EXPECT_LT(output_velocity_xy.head(2).norm(), kMaxLinearVelocityDefault);
  EXPECT_LT(output_velocity_t(kPoseTheta), kMaxAngularVelocityDefault);
}

/// CalculateVelocity test
/// In the goal area, if the maximum translational speed < speed including deceleration to the goal, it is limited to the smaller one
TEST_F(OmniVelocityCalculatorTest, ChooseMaxVeloctyInGoalArea) {
  // setup
  // Threshold where the target speed matches the maximum speed
  const double threshold_distance = kMaxLinearVelocityDefault / kLinearPGainDefault;
  const double threshold_angle = kMaxAngularVelocityDefault / kAngularPGainDefault;
  // Current position set slightly outside the threshold
  // Since setting both translation and angle simultaneously makes it difficult to verify due to rotation in velocity, check each
  const Pose2d current_pose_xy(path_info_.splined_path.back().x() - threshold_distance - kEpsilon,
                               path_info_.splined_path.back().y(),
                               0.0);
  const Pose2d current_pose_t(path_info_.splined_path.back().x(),
                              path_info_.splined_path.back().y(),
                              kGoalAngle - threshold_angle - kEpsilon);
  // Previous speed, assuming moving straight at maximum speed towards the goal
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
  // Confirm that both translational speed and turning speed are at maximum speed
  EXPECT_DOUBLE_EQ(kMaxLinearVelocityDefault, output_velocity_xy.head(2).norm());
  EXPECT_DOUBLE_EQ(kMaxAngularVelocityDefault, output_velocity_t(kPoseTheta));
}

/// CalculateVelocity test
/// In the goal area, the closer to the goal, the smaller the speed
TEST_F(OmniVelocityCalculatorTest, NearerGoalSlowerVelocityInGoalArea) {
  // setup
  // Threshold where the target speed matches the maximum speed
  const double threshold_distance = kMaxLinearVelocityDefault / kLinearPGainDefault;
  const double threshold_angle = kMaxAngularVelocityDefault / kAngularPGainDefault;
  // Prepare two cases: close to the goal and far from the goal
  const Pose2d current_pose_near(path_info_.splined_path.back().x() - threshold_distance / 4.0,
                                 path_info_.splined_path.back().y(),
                                 kGoalAngle - threshold_angle / 4.0);
  const Pose2d current_pose_far(path_info_.splined_path.back().x() - threshold_distance / 2.0,
                                path_info_.splined_path.back().y(),
                                kGoalAngle - threshold_angle / 2.0);
  // Previous speed, assuming moving straight at maximum speed towards the goal
  Vector3d output_velocity_near = Vector3d(kMaxLinearVelocityDefault, 0.0, 0.0);
  Vector3d output_velocity_far = Vector3d(kMaxLinearVelocityDefault, 0.0, 0.0);

  // exercise
  // Since the target speed does not appear immediately due to acceleration limits, call repeatedly
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
  // The closer to the goal, the smaller the speed
  EXPECT_LT(output_velocity_near.head(2).norm(), output_velocity_far.head(2).norm());
  EXPECT_LT(std::abs(output_velocity_near(kPoseTheta)), std::abs(output_velocity_far(kPoseTheta)));
}

/// CalculateVelocity test
/// Outside the goal area, if the maximum translational speed > speed including deceleration to the goal, it is limited to the smaller one
TEST_F(OmniVelocityCalculatorTest, ChooseCalculatedVeloctyOutGoalArea) {
  // setup
  // Threshold where the target speed matches the maximum speed
  const double threshold_distance = (kMaxLinearVelocityDefault - kVelocityMarginDefault) / kGoalDecelerationDefault;
  // Current position set slightly inside the threshold
  const Pose2d current_pose(path_info_.splined_path.back().x() - threshold_distance + kEpsilon,
                            path_info_.splined_path.back().y(), 0.0);
  // Previous speed, assuming moving straight at maximum speed towards the goal
  Vector3d last_velocity = Vector3d(kMaxLinearVelocityDefault, 0.0, 0.0);

  // exercise
  Vector3d output_velocity;
  const bool result = velocity_calculator_->CalculateVelocity(path_info_, current_pose,
                                                              path_info_.splined_path.size() - 2,
                                                              last_velocity, kInterval, false,
                                                              std::nullopt, output_velocity);

  // verify
  ASSERT_TRUE(result);
  // Confirm that translational speed is suppressed
  EXPECT_LT(output_velocity(kPoseX), kMaxLinearVelocityDefault);
}

/// CalculateVelocity test
/// Outside the goal area, if the maximum translational speed < speed including deceleration to the goal, it is limited to the smaller one
TEST_F(OmniVelocityCalculatorTest, ChooseMaxVeloctyOutGoalArea) {
  // setup
  // Threshold where the target speed matches the maximum speed
  const double threshold_distance = (kMaxLinearVelocityDefault - kVelocityMarginDefault) / kGoalDecelerationDefault;
  // Current position set slightly outside the threshold
  const Pose2d current_pose(path_info_.splined_path.back().x() - threshold_distance - kEpsilon,
                            path_info_.splined_path.back().y(), 0.0);
  // Previous speed, assuming moving straight at maximum speed towards the goal
  Vector3d last_velocity = Vector3d(kMaxLinearVelocityDefault, 0.0, 0.0);

  // exercise
  Vector3d output_velocity;
  const bool result = velocity_calculator_->CalculateVelocity(path_info_, current_pose,
                                                              path_info_.splined_path.size() - 2,
                                                              last_velocity, kInterval, false,
                                                              std::nullopt, output_velocity);

  // verify
  ASSERT_TRUE(result);
  // Confirm that the translational speed is at maximum speed
  EXPECT_DOUBLE_EQ(kMaxLinearVelocityDefault, output_velocity(kPoseX));
}

/// CalculateVelocity test
/// Outside the goal area, if a passing speed is set, it is limited to the passing speed
TEST_F(OmniVelocityCalculatorTest, LimitTransitVelocity) {
  // setup
  // Threshold where the target speed matches the maximum speed
  const double threshold_distance = (kMaxLinearVelocityDefault - kVelocityMarginDefault) / kGoalDecelerationDefault;
  // Current position set slightly outside the threshold
  const Pose2d current_pose(path_info_.splined_path.back().x() - threshold_distance - kEpsilon,
                            path_info_.splined_path.back().y(), 0.0);
  // Previous speed, assuming moving straight at maximum speed towards the goal
  Vector3d last_velocity = Vector3d(kMaxLinearVelocityDefault, 0.0, 0.0);
  // Set the passing speed slightly less than the maximum translational speed
  const double transit_velocity = kMaxLinearVelocityDefault - kEpsilon;
  // exercise
  Vector3d output_velocity;
  const bool result = velocity_calculator_->CalculateVelocity(path_info_, current_pose,
                                                              path_info_.splined_path.size() - 2,
                                                              last_velocity, kInterval, false,
                                                              transit_velocity, output_velocity);

  // verify
  ASSERT_TRUE(result);
  // Confirm that the translational speed is at the passing speed
  EXPECT_DOUBLE_EQ(transit_velocity, output_velocity(kPoseX));
}

/// CalculateVelocity test
/// Outside the goal area, the closer to the goal, the smaller the speed
TEST_F(OmniVelocityCalculatorTest, NearerGoalSlowerVelocityOutGoalArea) {
  // setup
  // Threshold where the target speed matches the maximum speed
  const double threshold_distance = (kMaxLinearVelocityDefault - kVelocityMarginDefault) / kGoalDecelerationDefault;
  // Prepare two cases: close to the goal and far from the goal
  const Pose2d current_pose_near(path_info_.splined_path.back().x() - threshold_distance / 4.0,
                                 path_info_.splined_path.back().y(),
                                 0.0);
  const Pose2d current_pose_far(path_info_.splined_path.back().x() - threshold_distance / 2.0,
                                path_info_.splined_path.back().y(),
                                0.0);
  // Previous speed, assuming moving straight at maximum speed towards the goal
  Vector3d output_velocity_near = Vector3d(kMaxLinearVelocityDefault, 0.0, 0.0);
  Vector3d output_velocity_far = Vector3d(kMaxLinearVelocityDefault, 0.0, 0.0);

  // exercise
  // Since the target speed does not appear immediately due to acceleration limits, call repeatedly
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
  // The closer to the goal, the smaller the speed
  EXPECT_LT(output_velocity_near.head(2).norm(), output_velocity_far.head(2).norm());
}

/// CalculateVelocity test
/// If the distance to the goal is within a certain range, the cart's turning speed is set to face the direction of the goal
TEST_F(OmniVelocityCalculatorTest, TurnToGoalPoint) {
  // setup
  // Current position placed slightly inside the threshold, facing the direction between the goal and the path
  const Pose2d current_pose(path_info_.splined_path.back().x() - kPathLengthThresholdDefault + kTestPathInterval,
                            path_info_.splined_path.back().y(), kGoalAngle / 2.0);
  const uint32_t path_index =
      static_cast<uint32_t>((kTestPathLength - kPathLengthThresholdDefault) / kTestPathInterval) + 1;
  // Previous speed
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
/// If the distance to the goal is beyond a certain range, the cart's turning speed is set to face the direction of the nearest point
TEST_F(OmniVelocityCalculatorTest, TurnToNearestPoint) {
  // setup
  // Current position placed slightly outside the threshold, facing the direction between the goal and the path
  const Pose2d current_pose(path_info_.splined_path.back().x() - kPathLengthThresholdDefault - kTestPathInterval,
                            path_info_.splined_path.back().y(), kGoalAngle / 2.0);
  const uint32_t path_index =
      static_cast<uint32_t>((kTestPathLength - kPathLengthThresholdDefault) / kTestPathInterval) - 1;
  // Previous speed
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
/// The speed of X, Y, and turning angle is limited
TEST_F(OmniVelocityCalculatorTest, LimitVelocities) {
  // setup
  // Current position placed in a position and orientation off the path, so that control is applied to x, y, and t
  const Pose2d current_pose(path_info_.splined_path.front().x(),
                            path_info_.splined_path.front().y() - kTestPathInterval,
                            M_PI / 4.0);
  // Previous speed set to a speed attempting to reach the target position exceeding the maximum speed
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
/// The acceleration of X, Y, and turning angle is limited
TEST_F(OmniVelocityCalculatorTest, LimitAccelerations) {
  // setup
  // Current position placed in a position and orientation off the path, so that control is applied to x, y, and t
  const Pose2d current_pose(path_info_.splined_path.front().x(),
                            path_info_.splined_path.front().y() - kTestPathInterval,
                            M_PI / 4.0);
  // Previous speed, check how much acceleration occurs from zero
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
