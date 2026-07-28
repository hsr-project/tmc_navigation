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
#include <tmc_base_path_follower/velocity_calculator_diff_drive.hpp>

#include "test_utils.hpp"

namespace {
// Test path length [m]
constexpr double kTestPathLength = 5.0;
// Test path waypoint interval [m]
constexpr double kTestPathInterval = 0.1;
// Goal direction of the test path [rad]
// Reverse the direction of the path. If it is exactly 180° opposite, it is unclear which way to rotate, so it is made closer to counterclockwise
constexpr double kGoalAngle = (135.0 / 180. * M_PI);
// Direction of the goal from the self-position for goal area approach test [rad]
constexpr double kGoalDirection = (135.0 / 180.0 * M_PI);
// Distance from the self-position to the goal for goal area approach test [m]
constexpr double kGoalDistance = 0.5;
// Radius of circular path for turning speed test [m]
constexpr double kTestR = 5.0;
// Speed calculation time interval [s]
constexpr double kInterval = 0.01;
}  // anonymous namespace


namespace tmc_base_path_follower {

/// Parameter test
/// Ability to generate parameters
TEST(DiffDriveVelocityCalculatorParameterTest, CreateParameter) {
  // exercise
  /// Input parameters under the following conditions
  /// max_linear_velocity > velocity_mergin
  /// spin_start_error_angle > spin_end_error_angle
  /// spin_max_angular_velocity > spin_min_angular_velocity
  DiffDriveVelocityCalculator::Parameter param(0.1, 0.2, 0.3, 0.4, 0.5, 0.06, 0.7, 0.8, 0.9, 1.0, 0.11, 1.2, 0.13);

  // verify
  // Check if the values are set as input
  EXPECT_DOUBLE_EQ(0.1, param.max_linear_velocity);
  EXPECT_DOUBLE_EQ(0.2, param.max_angular_velocity);
  EXPECT_DOUBLE_EQ(0.3, param.max_linear_acceleration);
  EXPECT_DOUBLE_EQ(0.4, param.max_angular_acceleration);
  EXPECT_DOUBLE_EQ(0.5, param.goal_deceleration);
  EXPECT_DOUBLE_EQ(0.06, param.velocity_margin);
  EXPECT_DOUBLE_EQ(0.7, param.linear_alpha_gain);
  EXPECT_DOUBLE_EQ(0.8, param.linear_beta_gain);
  EXPECT_DOUBLE_EQ(0.9, param.angle_error_angular_velocity_rate);
  EXPECT_DOUBLE_EQ(1.0, param.spin_start_error_angle);
  EXPECT_DOUBLE_EQ(0.11, param.spin_end_error_angle);
  EXPECT_DOUBLE_EQ(1.2, param.spin_max_angular_velocity);
  EXPECT_DOUBLE_EQ(0.13, param.spin_min_angular_velocity);
}

/// Parameter test
/// If invalid values are specified, default values are used
TEST(DiffDriveVelocityCalculatorParameterTest, InvalidParameter) {
  // exercise
  // Set all parameters to invalid values
  DiffDriveVelocityCalculator::Parameter param(-1.0, -1.0, -1.0, -1.0, -1.0, -0.1, -1.0,
                                               -1.0, -1.0, -1.0, -1.0, -1.0, -1.0);

  // verify
  // Check if default values are set
  EXPECT_DOUBLE_EQ(kMaxLinearVelocityDefault, param.max_linear_velocity);
  EXPECT_DOUBLE_EQ(kMaxAngularVelocityDefault, param.max_angular_velocity);
  EXPECT_DOUBLE_EQ(kMaxLinearAccelerationDefault, param.max_linear_acceleration);
  EXPECT_DOUBLE_EQ(kMaxAngularAccelerationDefault, param.max_angular_acceleration);
  EXPECT_DOUBLE_EQ(kGoalDecelerationDefault, param.goal_deceleration);
  EXPECT_DOUBLE_EQ(kVelocityMarginDefault, param.velocity_margin);
  EXPECT_DOUBLE_EQ(kLinearAlphaGainDefault, param.linear_alpha_gain);
  EXPECT_DOUBLE_EQ(kLinearBetaGainDefault, param.linear_beta_gain);
  EXPECT_DOUBLE_EQ(kAngleErrorAngularVelocityRateDefault, param.angle_error_angular_velocity_rate);
  EXPECT_DOUBLE_EQ(kSpinStartErrorAngleDefault, param.spin_start_error_angle);
  EXPECT_DOUBLE_EQ(kSpinEndErrorAngleDefault, param.spin_end_error_angle);
  EXPECT_DOUBLE_EQ(kSpinMaxAngularVelocityDefault, param.spin_max_angular_velocity);
  EXPECT_DOUBLE_EQ(kSpinMinAngularVelocityDefault, param.spin_min_angular_velocity);
}

/// Parameter test
/// If the maximum translational velocity is smaller than the velocity margin, the default value of the margin is used
TEST(DiffDriveVelocityCalculatorParameterTest, MaxVelocityLowerThanMergin) {
  // exercise
  // Input parameters (set so that max_linear_velocity < velocity_margin)
  DiffDriveVelocityCalculator::Parameter param(kVelocityMarginDefault + kEpsilon, 0.2, 0.3, 0.4, 0.5,
                                               kVelocityMarginDefault + kEpsilon * 2.0, 0.7, 0.8, 0.9,
                                               1.0, 0.11, 1.2, 0.13);

  // verify
  // The velocity margin is set to the default value
  EXPECT_DOUBLE_EQ(kVelocityMarginDefault, param.velocity_margin);
}

/// Parameter test
/// If the maximum translational velocity is smaller than the velocity margin and the default value of the margin,
/// The maximum translational velocity and velocity margin are set to default values
TEST(DiffDriveVelocityCalculatorParameterTest, MaxVelocityLowerThanMerginDefault) {
  // exercise
  // Input parameters (set so that max_linear_velocity < velocity_margin, kVelocityMarginDefault)
  DiffDriveVelocityCalculator::Parameter param(kVelocityMarginDefault - kEpsilon * 2.0, 0.2, 0.3, 0.4, 0.5,
                                               kVelocityMarginDefault - kEpsilon, 0.7, 0.8, 0.9,
                                               1.0, 0.11, 1.2, 0.13);
  // verify
  // The maximum translational velocity and velocity margin are set to default values
  EXPECT_DOUBLE_EQ(kMaxLinearVelocityDefault, param.max_linear_velocity);
  EXPECT_DOUBLE_EQ(kVelocityMarginDefault, param.velocity_margin);
}

/// Parameter test
/// If the in-place rotation start angle error is smaller than the in-place rotation end angle error, the default value is used
TEST(DiffDriveVelocityCalculatorParameterTest, SpinStartErrorAngleLowerSpinEndErrorAngle) {
  // exercise
  // Input parameters (set so that spin_start_error_angle < spin_end_error_angle)
  DiffDriveVelocityCalculator::Parameter param(0.1, 0.2, 0.3, 0.4, 0.5, 0.06, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 0.13);

  // verify
  // The in-place rotation start angle error and the in-place rotation end angle error are set to default values
  EXPECT_DOUBLE_EQ(kSpinStartErrorAngleDefault, param.spin_start_error_angle);
  EXPECT_DOUBLE_EQ(kSpinEndErrorAngleDefault, param.spin_end_error_angle);
}

/// Parameter test
/// If the maximum rotation speed for in-place rotation is smaller than the minimum rotation speed for in-place rotation, the default value is used
TEST(DiffDriveVelocityCalculatorParameterTest, SpinMaxAngularVelocityLowerThanSpinMinAngularVelocity) {
  // exercise
  // Input parameters (set so that spin_start_error_angle < spin_end_error_angle)
  DiffDriveVelocityCalculator::Parameter param(0.1, 0.2, 0.3, 0.4, 0.5, 0.06, 0.7, 0.8, 0.9, 1.0, 0.11, 1.2, 1.3);

  // verify
  // The maximum rotation speed and the minimum rotation speed for in-place rotation are set to default values
  EXPECT_DOUBLE_EQ(kSpinMaxAngularVelocityDefault, param.spin_max_angular_velocity);
  EXPECT_DOUBLE_EQ(kSpinMinAngularVelocityDefault, param.spin_min_angular_velocity);
}

/// DiffDriveVelocityCalculator test fixture
class DiffDriveVelocityCalculatorTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    // Create target instance
    velocity_calculator_ = std::make_shared<DiffDriveVelocityCalculator>(
        DiffDriveVelocityCalculator::Parameter(kMaxLinearVelocityDefault, kMaxAngularVelocityDefault,
                                               kMaxLinearAccelerationDefault, kMaxAngularAccelerationDefault,
                                               kGoalDecelerationDefault, kVelocityMarginDefault,
                                               kLinearAlphaGainDefault, kLinearBetaGainDefault,
                                               kAngleErrorAngularVelocityRateDefault,
                                               kSpinStartErrorAngleDefault, kSpinEndErrorAngleDefault,
                                               kSpinMaxAngularVelocityDefault, kSpinMinAngularVelocityDefault));
    // Generate test path
    // Generate a straight path
    const int32_t path_points = static_cast<int32_t>(kTestPathLength / kTestPathInterval);
    for (int32_t i = 0; i < path_points; ++i) {
      const double x = kTestPathLength * static_cast<double>(i) / static_cast<double>(path_points - 1);
      path_info_.splined_path.push_back(Pose2d(x, 0.0, 0.0));
      path_info_.splined_path_curvatures.push_back(0.0);
      path_info_.splined_path_left_lengths.push_back(kTestPathLength - x);
    }
    path_info_.splined_path.back().set_theta(kGoalAngle);
  }

  DiffDriveVelocityCalculator::Ptr velocity_calculator_;
  PathInfo path_info_;
};

/// CalculateVelocity test
/// Near the goal, translational velocity proportional to the distance from the goal is output
TEST_F(DiffDriveVelocityCalculatorTest, DecelerationLinearVelocityDistanceToGoal) {
  // Initial velocity is set to move straight toward the goal at maximum speed
  Vector3d velocity = Vector3d(kMaxLinearVelocityDefault, 0.0, 0.0);
  // Distance at which translational velocity proportional to the distance from the goal starts
  const double deceleration_distance =
      (kMaxLinearVelocityDefault - kVelocityMarginDefault) / kGoalDecelerationDefault;
  // Gradually bring the self-position closer
  for (int32_t i = 9; i >= 0; --i) {
    const double distance = deceleration_distance * (i / 10);
    const Pose2d global_pose(path_info_.splined_path.back().x() - distance, 0.0, 0.0);
    const bool result = velocity_calculator_->CalculateVelocity(path_info_, global_pose, 0, velocity, kInterval,
                                                                false, std::nullopt, velocity);
    ASSERT_TRUE(result);
    // X decelerates in proportion to the distance, and the minimum value is velocity_margin
    const double expect_x = distance * kGoalDecelerationDefault + kVelocityMarginDefault;
    EXPECT_DOUBLE_EQ(expect_x, velocity(kPoseX));
    // Y is always 0
    EXPECT_DOUBLE_EQ(0.0, velocity(kPoseY));
  }
}

/// CalculateVelocity test
/// If the translational velocity proportional to the distance from the goal exceeds the maximum speed, it is rounded to the maximum speed
TEST_F(DiffDriveVelocityCalculatorTest, MaxLinearVelocity) {
  // Initial velocity is set to move straight toward the goal at maximum speed
  Vector3d velocity = Vector3d(kMaxLinearVelocityDefault, 0.0, 0.0);
  // Distance at which translational velocity proportional to the distance from the goal starts
  const double deceleration_distance =
      (kMaxLinearVelocityDefault - kVelocityMarginDefault) / kGoalDecelerationDefault;
  // The self-position is farther than the deceleration distance
  const Pose2d global_pose(path_info_.splined_path.back().x() - deceleration_distance - 1.0, 0.0, 0.0);
  const bool result = velocity_calculator_->CalculateVelocity(path_info_, global_pose, 0, velocity, kInterval,
                                                              false, std::nullopt, velocity);
  ASSERT_TRUE(result);
  // X is rounded to the maximum speed
  EXPECT_DOUBLE_EQ(kMaxLinearVelocityDefault, velocity(kPoseX));
  // Y is always 0
  EXPECT_DOUBLE_EQ(0.0, velocity(kPoseY));
}

/// CalculateVelocity test
/// If the translational velocity proportional to the distance from the goal exceeds the passing speed, it is rounded to the passing speed
TEST_F(DiffDriveVelocityCalculatorTest, TransitVelocity) {
  // Initial velocity is set to move straight toward the goal at maximum speed
  Vector3d velocity = Vector3d(kMaxLinearVelocityDefault, 0.0, 0.0);
  // Distance at which translational velocity proportional to the distance from the goal starts
  const double deceleration_distance =
      (kMaxLinearVelocityDefault - kVelocityMarginDefault) / kGoalDecelerationDefault;
  // The self-position is farther than the deceleration distance
  const Pose2d global_pose(path_info_.splined_path.back().x() - deceleration_distance - 1.0, 0.0, 0.0);
  // Set the passing speed slightly lower than the maximum translational speed
  const double transit_velocity = kMaxLinearVelocityDefault - kEpsilon;
  const bool result = velocity_calculator_->CalculateVelocity(path_info_, global_pose, 0, velocity, kInterval,
                                                              false, transit_velocity, velocity);
  ASSERT_TRUE(result);
  // X is rounded to the maximum speed
  EXPECT_DOUBLE_EQ(transit_velocity, velocity(kPoseX));
  // Y is always 0
  EXPECT_DOUBLE_EQ(0.0, velocity(kPoseY));
}

/// CalculateVelocity test
/// Feedforward turning speed is output according to the curvature, translational speed, and angle difference with the path point
TEST_F(DiffDriveVelocityCalculatorTest, AngularVelocityFeedforward) {
  /// Turning speed is a combination of Feedforward and Feedback, but this test aims to verify the correctness of Feedforward
  /// Therefore, set the gain to 0.0 to disable the Feedback component and create the object
  DiffDriveVelocityCalculator::Ptr velocity_calculator = std::make_shared<DiffDriveVelocityCalculator>(
      DiffDriveVelocityCalculator::Parameter(kMaxLinearVelocityDefault, kMaxAngularVelocityDefault,
                                             kMaxLinearAccelerationDefault, kMaxAngularAccelerationDefault,
                                             kGoalDecelerationDefault, kVelocityMarginDefault,
                                             0.0, 0.0,
                                             kAngleErrorAngularVelocityRateDefault,
                                             kSpinStartErrorAngleDefault, kSpinEndErrorAngleDefault,
                                             kSpinMaxAngularVelocityDefault, kSpinMinAngularVelocityDefault));
  // Only the curvature value is set as the path shape is not considered
  for (double& curvature : path_info_.splined_path_curvatures) {
    curvature = 1.0 / kTestR;
  }
  // Angle difference with the path point [rad]
  const double diff_t = 0.1;
  // Theoretical angular velocity when traveling at maximum speed [rad/s]
  const double expected_angular_velocity = (1.0 / kTestR) * kMaxLinearVelocityDefault * cos(diff_t);
  // Current position
  const Pose2d current_pose(path_info_.splined_path.front().x(),
                            path_info_.splined_path.front().y(),
                            diff_t);
  // Set the theoretical value of the previous velocity
  Vector3d output_velocity = Vector3d(kMaxLinearVelocityDefault, 0.0, expected_angular_velocity);

  // exercise
  const bool result = velocity_calculator->CalculateVelocity(
      path_info_, current_pose, 0, output_velocity, kInterval, false, std::nullopt, output_velocity);
  ASSERT_TRUE(result);

  // verify
  EXPECT_DOUBLE_EQ(expected_angular_velocity, output_velocity(kPoseTheta));
}

/// CalculateVelocity test
/// Feedback turning speed is output according to the lateral deviation from the path direction
TEST_F(DiffDriveVelocityCalculatorTest, AngularVelocityFeedbackLinearError) {
  /// It is not possible to confirm if it is limited by the maximum turning acceleration
  /// Set the maximum turning acceleration to a large value to avoid limitation
  velocity_calculator_.reset(new DiffDriveVelocityCalculator(
      DiffDriveVelocityCalculator::Parameter(kMaxLinearVelocityDefault, kMaxAngularVelocityDefault,
                                             kMaxLinearAccelerationDefault, 10000,
                                             kGoalDecelerationDefault, kVelocityMarginDefault,
                                             kLinearAlphaGainDefault, kLinearBetaGainDefault,
                                             kAngleErrorAngularVelocityRateDefault,
                                             kSpinStartErrorAngleDefault, kSpinEndErrorAngleDefault,
                                             kSpinMaxAngularVelocityDefault, kSpinMinAngularVelocityDefault)));
  /// Turning speed is a combination of Feedforward and Feedback, but this test aims to verify the correctness of Feedback
  /// Since the path used in the test is straight, the Feedforward component becomes 0, and only the Feedback component's turning speed can be confirmed
  // exercise
  // When the current position is not deviated to either side
  Vector3d output_velocity = Vector3d(kMaxLinearVelocityDefault, 0.0, 0.0);
  Pose2d current_pose(path_info_.splined_path.front().x(),
                      path_info_.splined_path.front().y(),
                      0.0);
  bool result = velocity_calculator_->CalculateVelocity(
      path_info_, current_pose, 0, output_velocity, kInterval, false, std::nullopt, output_velocity);
  ASSERT_TRUE(result);
  const double no_diff_angular_velocity = output_velocity(kPoseTheta);
  // When the current position is slightly deviated to the right
  current_pose.set_y(path_info_.splined_path.front().y() - 0.1);
  result = velocity_calculator_->CalculateVelocity(
      path_info_, current_pose, 0, output_velocity, kInterval, false, std::nullopt, output_velocity);
  ASSERT_TRUE(result);
  const double diff_right_little_angular_velocity = output_velocity(kPoseTheta);
  // When the current position is significantly deviated to the right
  current_pose.set_y(path_info_.splined_path.front().y() - 0.5);
  result = velocity_calculator_->CalculateVelocity(
      path_info_, current_pose, 0, output_velocity, kInterval, false, std::nullopt, output_velocity);
  ASSERT_TRUE(result);
  const double diff_right_large_angular_velocity = output_velocity(kPoseTheta);
  // When the current position is slightly deviated to the left
  current_pose.set_y(path_info_.splined_path.front().y() + 0.1);
  result = velocity_calculator_->CalculateVelocity(
      path_info_, current_pose, 0, output_velocity, kInterval, false, std::nullopt, output_velocity);
  ASSERT_TRUE(result);
  const double diff_left_little_angular_velocity = output_velocity(kPoseTheta);
  // When the current position is significantly deviated to the left
  current_pose.set_y(path_info_.splined_path.front().y() + 0.5);
  result = velocity_calculator_->CalculateVelocity(
      path_info_, current_pose, 0, output_velocity, kInterval, false, std::nullopt, output_velocity);
  ASSERT_TRUE(result);
  const double diff_left_large_angular_velocity = output_velocity(kPoseTheta);


  // verify
  // When the current position is not deviated to either side, the turning speed is 0
  EXPECT_DOUBLE_EQ(0.0, no_diff_angular_velocity);
  // When the current position is slightly deviated to the right, the turning speed is positive
  EXPECT_GT(diff_right_little_angular_velocity, 0.0);
  // When the current position is significantly deviated to the right, the turning speed is even greater than when it was slightly deviated
  EXPECT_GT(diff_right_large_angular_velocity, diff_right_little_angular_velocity);
  // When the current position is slightly deviated to the left, the turning speed is negative
  EXPECT_LT(diff_left_little_angular_velocity, 0.0);
  // When the current position is significantly deviated to the left, the turning speed is even smaller than when it was slightly deviated
  EXPECT_LT(diff_left_large_angular_velocity, diff_left_little_angular_velocity);
}

/// CalculateVelocity test
/// Feedback turning speed is output according to the angular deviation from the path point
TEST_F(DiffDriveVelocityCalculatorTest, AngularVelocityFeedbackAngularError) {
  /// It is not possible to confirm if it is limited by the maximum turning acceleration
  /// Set the maximum turning acceleration to a large value to avoid limitation
  velocity_calculator_.reset(new DiffDriveVelocityCalculator(
      DiffDriveVelocityCalculator::Parameter(kMaxLinearVelocityDefault, kMaxAngularVelocityDefault,
                                             kMaxLinearAccelerationDefault, 10000,
                                             kGoalDecelerationDefault, kVelocityMarginDefault,
                                             kLinearAlphaGainDefault, kLinearBetaGainDefault,
                                             kAngleErrorAngularVelocityRateDefault,
                                             kSpinStartErrorAngleDefault, kSpinEndErrorAngleDefault,
                                             kSpinMaxAngularVelocityDefault, kSpinMinAngularVelocityDefault)));
  /// Turning speed is a combination of Feedforward and Feedback, but this test aims to verify the correctness of Feedback
  /// Since the path used in the test is straight, the Feedforward component becomes 0, and only the Feedback component's turning speed can be confirmed
  // exercise
  // When the current position is not deviated in any direction
  Vector3d output_velocity = Vector3d(kMaxLinearVelocityDefault, 0.0, 0.0);
  Pose2d current_pose(path_info_.splined_path.front().x(),
                      path_info_.splined_path.front().y(),
                      0.0);
  bool result = velocity_calculator_->CalculateVelocity(
      path_info_, current_pose, 0, output_velocity, kInterval, false, std::nullopt, output_velocity);
  ASSERT_TRUE(result);
  const double no_diff_angular_velocity = output_velocity(kPoseTheta);
  // When the angle is slightly deviated in the positive direction
  current_pose.set_theta(0.1);
  result = velocity_calculator_->CalculateVelocity(
      path_info_, current_pose, 0, output_velocity, kInterval, false, std::nullopt, output_velocity);
  ASSERT_TRUE(result);
  const double diff_plus_little_angular_velocity = output_velocity(kPoseTheta);
  // When the angle is significantly deviated in the positive direction
  current_pose.set_theta(0.5);
  result = velocity_calculator_->CalculateVelocity(
      path_info_, current_pose, 0, output_velocity, kInterval, false, std::nullopt, output_velocity);
  ASSERT_TRUE(result);
  const double diff_plus_large_angular_velocity = output_velocity(kPoseTheta);
  // When the angle is slightly deviated in the negative direction
  current_pose.set_theta(-0.1);
  result = velocity_calculator_->CalculateVelocity(
      path_info_, current_pose, 0, output_velocity, kInterval, false, std::nullopt, output_velocity);
  ASSERT_TRUE(result);
  const double diff_minus_little_angular_velocity = output_velocity(kPoseTheta);
  // When the angle is significantly deviated in the negative direction
  current_pose.set_theta(-0.5);
  result = velocity_calculator_->CalculateVelocity(
      path_info_, current_pose, 0, output_velocity, kInterval, false, std::nullopt, output_velocity);
  ASSERT_TRUE(result);
  const double diff_minus_large_angular_velocity = output_velocity(kPoseTheta);

  // verify
  // When the angle is not deviated in any direction, the turning speed is 0
  EXPECT_DOUBLE_EQ(0.0, no_diff_angular_velocity);
  // When the angle is slightly deviated in the positive direction, the turning speed is negative
  EXPECT_LT(diff_plus_little_angular_velocity, 0.0);
  // When the angle is significantly deviated in the positive direction, the turning speed is even smaller than when it was slightly deviated
  EXPECT_LT(diff_plus_large_angular_velocity, diff_plus_little_angular_velocity);
  // When the angle is slightly deviated in the negative direction, the turning speed is positive
  EXPECT_GT(diff_minus_little_angular_velocity, 0.0);
  // When the current position is significantly deviated to the left, the turning speed is even smaller than when it was slightly deviated
  EXPECT_GT(diff_minus_large_angular_velocity, diff_minus_little_angular_velocity);
}

/// CalculateVelocity test
/// If the goal is significantly exceeded (outside the goal judgment distance and the nearest point is the goal), failure is returned and it stops
TEST_F(DiffDriveVelocityCalculatorTest, StopFarFromGoal) {
  // setup
  // Set the current position 45° ahead of the goal
  const Pose2d current_pose(path_info_.splined_path.back().x() + cos(kGoalDirection + M_PI) * kGoalDistance,
                            path_info_.splined_path.back().y() + sin(kGoalDirection + M_PI) * kGoalDistance,
                            0.0);
  // Previous velocity
  Vector3d last_velocity = Vector3d::Zero();

  // exercise
  Vector3d output_velocity;
  const bool result = velocity_calculator_->CalculateVelocity(path_info_, current_pose,
                                                              path_info_.splined_path.size() - 1,
                                                              last_velocity, kInterval, false, std::nullopt,
                                                              output_velocity);
  // verify
  // It results in failure
  ASSERT_FALSE(result);
  // The output velocity is 0
  EXPECT_DOUBLE_EQ(0.0, output_velocity(kPoseX));
  EXPECT_DOUBLE_EQ(0.0, output_velocity(kPoseY));
  EXPECT_DOUBLE_EQ(0.0, output_velocity(kPoseTheta));
}

/// CalculateVelocity test
/// It is limited to the maximum speed
TEST_F(DiffDriveVelocityCalculatorTest, LimitVelocities) {
  // Current position
  const Pose2d current_pose(path_info_.splined_path.front().x(),
                            path_info_.splined_path.front().y() - kTestPathInterval,
                            kSpinStartErrorAngleDefault - kEpsilon);
  // Previous velocity Set a speed that exceeds the maximum speed and attempts to move toward the target position
  Vector3d last_velocity = Vector3d(kMaxLinearVelocityDefault * 2.0,
                                    0.0,
                                    -kMaxAngularVelocityDefault * 2.0);

  // exercise
  Vector3d output_velocity;
  const bool result = velocity_calculator_->CalculateVelocity(path_info_, current_pose,
                                                              0, last_velocity,
                                                              kInterval, false, std::nullopt, output_velocity);

  // verify
  ASSERT_TRUE(result);
  // The speed is at the upper limit
  EXPECT_DOUBLE_EQ(kMaxLinearVelocityDefault, output_velocity(kPoseX));
  EXPECT_DOUBLE_EQ(0.0, std::abs(output_velocity(kPoseY)));
  EXPECT_DOUBLE_EQ(kMaxAngularVelocityDefault, std::abs(output_velocity(kPoseTheta)));
}

/// CalculateVelocity test
/// It is limited to the maximum acceleration
TEST_F(DiffDriveVelocityCalculatorTest, LimitAccelerations) {
  // Current position
  const Pose2d current_pose(path_info_.splined_path.front().x(),
                            path_info_.splined_path.front().y() - kTestPathInterval,
                            kSpinStartErrorAngleDefault - kEpsilon);
  // Previous velocity Check how much acceleration occurs from zero
  Vector3d last_velocity = Vector3d::Zero();

  // exercise
  Vector3d output_velocity;
  const bool result = velocity_calculator_->CalculateVelocity(
      path_info_, current_pose, 0, last_velocity, kInterval, false, std::nullopt, output_velocity);

  // verify
  ASSERT_TRUE(result);
  // The acceleration is at the upper limit
  EXPECT_DOUBLE_EQ(kMaxLinearAccelerationDefault * kInterval, std::abs(output_velocity(kPoseX)));
  EXPECT_DOUBLE_EQ(0.0, std::abs(output_velocity(kPoseY)));
  EXPECT_DOUBLE_EQ(kMaxAngularAccelerationDefault * kInterval, std::abs(output_velocity(kPoseTheta)));
}

/// CalculateVelocity test
/// If the posture deviates from the path direction, it rotates in place to face the path direction
TEST_F(DiffDriveVelocityCalculatorTest, SpinToPath) {
  /// It is not possible to confirm if it is limited by the maximum acceleration
  /// Set the maximum acceleration to a large value to avoid limitation
  velocity_calculator_.reset(new DiffDriveVelocityCalculator(
      DiffDriveVelocityCalculator::Parameter(kMaxLinearVelocityDefault, kMaxAngularVelocityDefault,
                                             10000, 10000,
                                             kGoalDecelerationDefault, kVelocityMarginDefault,
                                             kLinearAlphaGainDefault, kLinearBetaGainDefault,
                                             kAngleErrorAngularVelocityRateDefault,
                                             kSpinStartErrorAngleDefault, kSpinEndErrorAngleDefault,
                                             kSpinMaxAngularVelocityDefault, kSpinMinAngularVelocityDefault)));
  // Initial velocity is set to move straight toward the goal at maximum speed
  Vector3d velocity = Vector3d(kMaxLinearVelocityDefault, 0.0, 0.0);

  // The posture of the current position is slightly inside the threshold for starting in-place rotation
  Pose2d current_pose(path_info_.splined_path.front().x(),
                      path_info_.splined_path.front().y(),
                      - kSpinStartErrorAngleDefault + kEpsilon);
  bool result = velocity_calculator_->CalculateVelocity(
      path_info_, current_pose, 0, velocity, kInterval, false, std::nullopt, velocity);
  ASSERT_TRUE(result);
  // X is at maximum speed, and in-place rotation has not started
  EXPECT_DOUBLE_EQ(kMaxLinearVelocityDefault, velocity(kPoseX));
  // Y is always 0
  EXPECT_DOUBLE_EQ(0.0, velocity(kPoseY));

  // The posture of the current position is slightly outside the threshold for starting in-place rotation
  current_pose.set_theta(- kSpinStartErrorAngleDefault - kEpsilon);
  result = velocity_calculator_->CalculateVelocity(
      path_info_, current_pose, 0, velocity, kInterval,
      false, std::nullopt, velocity);
  ASSERT_TRUE(result);
  // XY becomes 0
  EXPECT_DOUBLE_EQ(0.0, velocity(kPoseX));
  EXPECT_DOUBLE_EQ(0.0, velocity(kPoseY));
  const double spin_start_angular_velocity = velocity(kPoseTheta);
  // Rotates in the positive direction
  EXPECT_GT(spin_start_angular_velocity, 0.0);
  // Does not exceed the maximum turning speed
  EXPECT_LE(spin_start_angular_velocity, kSpinMaxAngularVelocityDefault);

  // The posture of the current position is slightly outside the threshold for ending in-place rotation
  current_pose.set_theta(- kSpinEndErrorAngleDefault - kEpsilon);
  result = velocity_calculator_->CalculateVelocity(
      path_info_, current_pose, 0, velocity, kInterval,
      false, std::nullopt, velocity);
  ASSERT_TRUE(result);
  // XY becomes 0
  EXPECT_DOUBLE_EQ(0.0, velocity(kPoseX));
  EXPECT_DOUBLE_EQ(0.0, velocity(kPoseY));
  // Rotates in the positive direction
  EXPECT_GT(velocity(kPoseTheta), 0.0);
  // As the target angle is approached, the turning speed decreases compared to when the rotation started
  EXPECT_LT(velocity(kPoseTheta), spin_start_angular_velocity);

  // The posture of the current position is slightly inside the threshold for ending in-place rotation
  current_pose.set_theta(- kSpinEndErrorAngleDefault + kEpsilon);
  result = velocity_calculator_->CalculateVelocity(
      path_info_, current_pose, 0, velocity, kInterval,
      false, std::nullopt, velocity);
  ASSERT_TRUE(result);
  // X is at maximum speed, and in-place rotation ends, resuming progress
  EXPECT_DOUBLE_EQ(kMaxLinearVelocityDefault, velocity(kPoseX));
  EXPECT_DOUBLE_EQ(0.0, velocity(kPoseY));
}

/// CalculateVelocity test
/// When entering the goal area, a turning speed to face the goal direction is output
TEST_F(DiffDriveVelocityCalculatorTest, MoveToGoal) {
  Vector3d velocity = Vector3d::Zero();

  // The position has reached the goal, and the posture is facing forward
  Pose2d current_pose(path_info_.splined_path.back().x(),
                      path_info_.splined_path.back().y(),
                      0.0);
  bool result = velocity_calculator_->CalculateVelocity(
      path_info_, current_pose, 0, velocity, kInterval,
      true, std::nullopt, velocity);
  ASSERT_TRUE(result);
  // XY becomes 0
  EXPECT_DOUBLE_EQ(0.0, velocity(kPoseX));
  EXPECT_DOUBLE_EQ(0.0, velocity(kPoseY));
  const double spin_start_angular_velocity = velocity(kPoseTheta);
  // Rotates in the positive direction
  EXPECT_GT(spin_start_angular_velocity, 0.0);
  // Does not exceed the maximum turning speed
  EXPECT_LE(spin_start_angular_velocity, kSpinMaxAngularVelocityDefault);

  // The position has reached the goal, and the posture is close to the goal
  current_pose.set_theta(kGoalAngle - kEpsilon);
  result = velocity_calculator_->CalculateVelocity(
      path_info_, current_pose, 0, velocity, kInterval,
      true, std::nullopt, velocity);
  ASSERT_TRUE(result);
  // XY becomes 0
  EXPECT_DOUBLE_EQ(0.0, velocity(kPoseX));
  EXPECT_DOUBLE_EQ(0.0, velocity(kPoseY));
  // As the target angle is approached, the turning speed decreases compared to when the rotation started
  EXPECT_LT(velocity(kPoseTheta), spin_start_angular_velocity);
  // Rotates in the positive direction and does not fall below the minimum turning speed
  EXPECT_GT(velocity(kPoseTheta), kSpinMinAngularVelocityDefault);
}
}  // namespace tmc_base_path_follower

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
