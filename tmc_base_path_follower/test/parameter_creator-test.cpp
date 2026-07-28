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
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <tmc_base_path_follower/parameter_creator.hpp>

namespace tmc_base_path_follower {
/// ParameterCreator test fixture
class ParameterCreatorTest : public ::testing::Test {
 public:
  ParameterCreatorTest() {
    rclcpp::NodeOptions option;
    option.allow_undeclared_parameters();
    option.automatically_declare_parameters_from_overrides(true);
    test_node_ = std::make_shared<rclcpp::Node>("parameter", option);
  }

  template<typename T>
  void SetParameter(const std::string name, T value) {
    test_node_->declare_parameter<T>(name, value);
    test_node_->set_parameter(rclcpp::Parameter(name, value));
  }

  virtual void TearDown() {}
 protected:
  std::shared_ptr<rclcpp::Node> test_node_;
};


/// OmniGoalChecker::Parameter generation test
/// If rosparam is set, generate according to rosparam
TEST_F(ParameterCreatorTest, CreateOmniGoalCheckerParameter) {
  // setup
  const double goal_area_length = 0.1;
  SetParameter("omni_goal_checker.goal_area_length", goal_area_length);
  const double goal_line_length = 0.2;
  SetParameter("omni_goal_checker.goal_line_length", goal_line_length);
  const double goal_stop_error_length = 0.3;
  SetParameter("omni_goal_checker.goal_stop_error_length", goal_stop_error_length);
  const double goal_stop_error_angle = 0.4;
  SetParameter("omni_goal_checker.goal_stop_error_angle", goal_stop_error_angle);

  // exercise
  OmniGoalChecker::Parameter param = CreateOmniGoalCheckerParameter(test_node_);

  // verify
  EXPECT_EQ(goal_area_length, param.goal_area_length);
  EXPECT_EQ(goal_line_length, param.goal_line_length);
  EXPECT_EQ(goal_stop_error_length, param.goal_stop_error_length);
  EXPECT_EQ(goal_stop_error_angle, param.goal_stop_error_angle);
}

/// OmniGoalChecker::Parameter generation test
/// If rosparam is not set, generate with default values
TEST_F(ParameterCreatorTest, CreateOmniGoalCheckerParameterDefault) {
  // exercise
  OmniGoalChecker::Parameter param = CreateOmniGoalCheckerParameter(test_node_);

  // verify
  EXPECT_EQ(kGoalAreaLengthDefault, param.goal_area_length);
  EXPECT_EQ(kGoalLineLengthDefault, param.goal_line_length);
  EXPECT_EQ(kGoalStopErrorLengthDefault, param.goal_stop_error_length);
  EXPECT_EQ(kGoalStopErrorAngleDefault, param.goal_stop_error_angle);
}

/// DiffDriveGoalChecker::Parameter generation test
/// If rosparam is set, generate according to rosparam
TEST_F(ParameterCreatorTest, CreateDiffDriveGoalCheckerParameter) {
  // setup
  const double goal_area_length = 0.1;
  SetParameter("diff_drive_goal_checker.goal_area_length", goal_area_length);
  const double goal_line_length = 0.2;
  SetParameter("diff_drive_goal_checker.goal_line_length", goal_line_length);
  const double goal_stop_error_angle = 0.3;
  SetParameter("diff_drive_goal_checker.goal_stop_error_angle", goal_stop_error_angle);

  // exercise
  DiffDriveGoalChecker::Parameter param = CreateDiffDriveGoalCheckerParameter(test_node_);

  // verify
  EXPECT_EQ(goal_area_length, param.goal_area_length);
  EXPECT_EQ(goal_line_length, param.goal_line_length);
  EXPECT_EQ(goal_stop_error_angle, param.goal_stop_error_angle);
}

/// DiffDriveGoalChecker::Parameter generation test
/// If rosparam is not set, generate with default values
TEST_F(ParameterCreatorTest, CreateDiffDriveGoalCheckerParameterDefault) {
  // exercise
  DiffDriveGoalChecker::Parameter param = CreateDiffDriveGoalCheckerParameter(test_node_);

  // verify
  EXPECT_EQ(kGoalAreaLengthDefault, param.goal_area_length);
  EXPECT_EQ(kGoalLineLengthDefault, param.goal_line_length);
  EXPECT_EQ(kGoalStopErrorAngleDefault, param.goal_stop_error_angle);
}

/// NearestPathPointSearcher::Parameter generation test
/// If rosparam is set, generate according to rosparam
TEST_F(ParameterCreatorTest, CreateNearestPathPointSearcherParameter) {
  // setup
  const double partial_search_range = 0.1;
  SetParameter("nearest_path_point_searcher.partial_search_range", partial_search_range);
  const double partial_search_permit_error = 0.2;
  SetParameter("nearest_path_point_searcher.partial_search_permit_error", partial_search_permit_error);
  // exercise
  NearestPathPointSearcher::Parameter param = CreateNearestPathPointSearcherParameter(test_node_);

  // verify
  EXPECT_EQ(partial_search_range, param.partial_search_range);
  EXPECT_EQ(partial_search_permit_error, param.partial_search_permit_error);
}

/// NearestPathPointSearcher::Parameter generation test
/// If rosparam is not set, generate with default values
TEST_F(ParameterCreatorTest, CreateNearestPathPointSearcherParameterDefault) {
  // exercise
  NearestPathPointSearcher::Parameter param = CreateNearestPathPointSearcherParameter(test_node_);
  EXPECT_EQ(kPartialSearchRangeDefault, param.partial_search_range);
  EXPECT_EQ(kPartialSearchPermitErrorDefault, param.partial_search_permit_error);
}

/// PathInfoCreator::Parameter generation test
/// If rosparam is set, generate according to rosparam
TEST_F(ParameterCreatorTest, CreatePathInfoCreatorParameter) {
  // setup
  const int32_t interpolation_number = 10;
  SetParameter("path_info_creator.interpolation_number", interpolation_number);
  const double max_linear_velocity = 0.2;
  SetParameter("path_info_creator.max_linear_velocity", max_linear_velocity);
  // exercise
  PathInfoCreator::Parameter param = CreatePathInfoCreatorParameter(test_node_);

  // verify
  EXPECT_EQ(interpolation_number, param.interpolation_number);
  EXPECT_EQ(max_linear_velocity, param.passing_velocity);
}

/// PathInfoCreator::Parameter generation test
/// If rosparam is not set, generate with default values
TEST_F(ParameterCreatorTest, CreatePathInfoCreatorParameterDefault) {
  // exercise
  PathInfoCreator::Parameter param = CreatePathInfoCreatorParameter(test_node_);

  // verify
  EXPECT_EQ(kInterpolationNumberDefault, param.interpolation_number);
  EXPECT_EQ(kMaxLinearVelocityDefault, param.passing_velocity);
}

/// OmniVelocityCalculator::Parameter generation test
/// If rosparam is set, generate according to rosparam
TEST_F(ParameterCreatorTest, CreateOmniVelocityCalculatorParameter) {
  // setup
  const double max_linear_velocity = 0.1;
  SetParameter("omni_velocity_calculator.max_linear_velocity", max_linear_velocity);
  const double max_angular_velocity = 0.2;
  SetParameter("omni_velocity_calculator.max_angular_velocity", max_angular_velocity);
  const double max_linear_acceleration = 0.3;
  SetParameter("omni_velocity_calculator.max_linear_acceleration", max_linear_acceleration);
  const double max_angular_acceleration = 0.4;
  SetParameter("omni_velocity_calculator.max_angular_acceleration", max_angular_acceleration);
  const double goal_deceleration = 0.5;
  SetParameter("omni_velocity_calculator.linear_deceleration_near_goal", goal_deceleration);
  const double velocity_margin = 0.01;
  SetParameter("omni_velocity_calculator.linear_velocity_margin", velocity_margin);
  const double path_length_threshold = 0.6;
  SetParameter("omni_velocity_calculator.path_length_threshold", path_length_threshold);
  const double linear_p_gain = 0.7;
  SetParameter("omni_velocity_calculator.linear_p_gain", linear_p_gain);
  const double angular_p_gain = 0.8;
  SetParameter("omni_velocity_calculator.angular_p_gain", angular_p_gain);
  const double goal_angle_gain = 0.9;
  SetParameter("omni_velocity_calculator.goal_angle_gain", goal_angle_gain);
  // exercise
  OmniVelocityCalculator::Parameter param = CreateOmniVelocityCalculatorParameter(test_node_);

  // verify
  EXPECT_EQ(max_linear_velocity, param.max_linear_velocity);
  EXPECT_EQ(max_angular_velocity, param.max_angular_velocity);
  EXPECT_EQ(max_linear_acceleration, param.max_linear_acceleration);
  EXPECT_EQ(max_angular_acceleration, param.max_angular_acceleration);
  EXPECT_EQ(goal_deceleration, param.goal_deceleration);
  EXPECT_EQ(velocity_margin, param.velocity_margin);
  EXPECT_EQ(path_length_threshold, param.path_length_threshold);
  EXPECT_EQ(linear_p_gain, param.linear_p_gain);
  EXPECT_EQ(angular_p_gain, param.angular_p_gain);
  EXPECT_EQ(goal_angle_gain, param.goal_angle_gain);
}

/// OmniVelocityCalculator::Parameter generation test
/// If rosparam is not set, generate with default values
TEST_F(ParameterCreatorTest, CreateOmniVelocityCalculatorParameterDefault) {
  // exercise
  OmniVelocityCalculator::Parameter param = CreateOmniVelocityCalculatorParameter(test_node_);

  // verify
  EXPECT_EQ(kMaxLinearVelocityDefault, param.max_linear_velocity);
  EXPECT_EQ(kMaxAngularVelocityDefault, param.max_angular_velocity);
  EXPECT_EQ(kMaxLinearAccelerationDefault, param.max_linear_acceleration);
  EXPECT_EQ(kMaxAngularAccelerationDefault, param.max_angular_acceleration);
  EXPECT_EQ(kGoalDecelerationDefault, param.goal_deceleration);
  EXPECT_EQ(kVelocityMarginDefault, param.velocity_margin);
  EXPECT_EQ(kPathLengthThresholdDefault, param.path_length_threshold);
  EXPECT_EQ(kLinearPGainDefault, param.linear_p_gain);
  EXPECT_EQ(kAngularPGainDefault, param.angular_p_gain);
  EXPECT_EQ(kGoalAngleGainDefault, param.goal_angle_gain);
}

/// DiffDriveVelocityCalculator::Parameter generation test
/// If rosparam is set, generate according to rosparam
TEST_F(ParameterCreatorTest, CreateDiffDriveVelocityCalculatorParameter) {
  // setup
  const double max_linear_velocity = 0.1;
  SetParameter("diff_drive_velocity_calculator.max_linear_velocity", max_linear_velocity);
  const double max_angular_velocity = 0.2;
  SetParameter("diff_drive_velocity_calculator.max_angular_velocity", max_angular_velocity);
  const double max_linear_acceleration = 0.3;
  SetParameter("diff_drive_velocity_calculator.max_linear_acceleration", max_linear_acceleration);
  const double max_angular_acceleration = 0.4;
  SetParameter("diff_drive_velocity_calculator.max_angular_acceleration", max_angular_acceleration);
  const double goal_deceleration = 0.5;
  SetParameter("diff_drive_velocity_calculator.linear_deceleration_near_goal", goal_deceleration);
  const double velocity_margin = 0.01;
  SetParameter("diff_drive_velocity_calculator.linear_velocity_margin", velocity_margin);
  const double linear_alpha_gain = 2.0;
  SetParameter("diff_drive_velocity_calculator.linear_alpha_gain", linear_alpha_gain);
  const double linear_beta_gain = 1.0;
  SetParameter("diff_drive_velocity_calculator.linear_beta_gain", linear_beta_gain);
  const double angle_error_angular_velocity_rate = 0.6;
  SetParameter("diff_drive_velocity_calculator.angle_error_angular_velocity_rate", angle_error_angular_velocity_rate);
  const double spin_start_error_angle = 0.7;
  SetParameter("diff_drive_velocity_calculator.spin_start_error_angle", spin_start_error_angle);
  const double spin_end_error_angle = 0.1;
  SetParameter("diff_drive_velocity_calculator.spin_end_error_angle", spin_end_error_angle);
  const double spin_max_angular_velocity = 3.0;
  SetParameter("diff_drive_velocity_calculator.spin_max_angular_velocity", spin_max_angular_velocity);
  const double spin_min_angular_velocity = 0.8;
  SetParameter("diff_drive_velocity_calculator.spin_min_angular_velocity", spin_min_angular_velocity);

  // exercise
  DiffDriveVelocityCalculator::Parameter param = CreateDiffDriveVelocityCalculatorParameter(test_node_);

  // verify
  EXPECT_EQ(max_linear_velocity, param.max_linear_velocity);
  EXPECT_EQ(max_angular_velocity, param.max_angular_velocity);
  EXPECT_EQ(max_linear_acceleration, param.max_linear_acceleration);
  EXPECT_EQ(max_angular_acceleration, param.max_angular_acceleration);
  EXPECT_EQ(goal_deceleration, param.goal_deceleration);
  EXPECT_EQ(velocity_margin, param.velocity_margin);
  EXPECT_EQ(linear_alpha_gain, param.linear_alpha_gain);
  EXPECT_EQ(linear_beta_gain, param.linear_beta_gain);
  EXPECT_EQ(angle_error_angular_velocity_rate, param.angle_error_angular_velocity_rate);
  EXPECT_EQ(spin_start_error_angle, param.spin_start_error_angle);
  EXPECT_EQ(spin_end_error_angle, param.spin_end_error_angle);
  EXPECT_EQ(spin_max_angular_velocity, param.spin_max_angular_velocity);
  EXPECT_EQ(spin_min_angular_velocity, param.spin_min_angular_velocity);
}

/// DiffDriveVelocityCalculator::Parameter generation test
/// If rosparam is not set, generate with default values
TEST_F(ParameterCreatorTest, CreateDiffDriveVelocityCalculatorParameterDefault) {
  // exercise
  DiffDriveVelocityCalculator::Parameter param = CreateDiffDriveVelocityCalculatorParameter(test_node_);

  // verify
  EXPECT_EQ(kMaxLinearVelocityDefault, param.max_linear_velocity);
  EXPECT_EQ(kMaxAngularVelocityDefault, param.max_angular_velocity);
  EXPECT_EQ(kMaxLinearAccelerationDefault, param.max_linear_acceleration);
  EXPECT_EQ(kMaxAngularAccelerationDefault, param.max_angular_acceleration);
  EXPECT_EQ(kGoalDecelerationDefault, param.goal_deceleration);
  EXPECT_EQ(kVelocityMarginDefault, param.velocity_margin);
  EXPECT_EQ(kLinearAlphaGainDefault, param.linear_alpha_gain);
  EXPECT_EQ(kLinearBetaGainDefault, param.linear_beta_gain);
  EXPECT_EQ(kAngleErrorAngularVelocityRateDefault, param.angle_error_angular_velocity_rate);
  EXPECT_EQ(kSpinStartErrorAngleDefault, param.spin_start_error_angle);
  EXPECT_EQ(kSpinEndErrorAngleDefault, param.spin_end_error_angle);
  EXPECT_EQ(kSpinMaxAngularVelocityDefault, param.spin_max_angular_velocity);
  EXPECT_EQ(kSpinMinAngularVelocityDefault, param.spin_min_angular_velocity);
}


/// PathTransitVelocityCalculator::Parameter generation test
/// If rosparam is set, generate according to rosparam
TEST_F(ParameterCreatorTest, CreatePathTransitVelocityCalculatorParameter) {
  // setup
  const double max_linear_velocity = 0.1;
  SetParameter("path_transit_velocity_calculator.max_linear_velocity", max_linear_velocity);
  const double max_angular_velocity = 0.2;
  SetParameter("path_transit_velocity_calculator.max_angular_velocity", max_angular_velocity);
  const double max_linear_acceleration = 0.3;
  SetParameter("path_transit_velocity_calculator.max_linear_acceleration", max_linear_acceleration);
  const double max_linear_deceleration = 0.4;
  SetParameter("path_transit_velocity_calculator.max_linear_deceleration", max_linear_deceleration);
  const double max_angular_acceleration = 0.5;
  SetParameter("path_transit_velocity_calculator.max_angular_acceleration", max_angular_acceleration);
  const double max_angular_deceleration = 0.6;
  SetParameter("path_transit_velocity_calculator.max_angular_deceleration", max_angular_deceleration);
  const double min_linear_velocity = 0.01;
  SetParameter("path_transit_velocity_calculator.min_linear_velocity", min_linear_velocity);
  const double transit_velocity_angular_velocity_ratio = 0.7;
  SetParameter("path_transit_velocity_calculator.transit_velocity_angular_velocity_ratio",
               transit_velocity_angular_velocity_ratio);

  // exercise
  PathTransitVelocityCalculator::Parameter param = CreatePathTransitVelocityCalculatorParameter(test_node_);

  // verify
  EXPECT_EQ(max_linear_velocity, param.max_linear_velocity);
  EXPECT_EQ(max_angular_velocity, param.max_angular_velocity);
  EXPECT_EQ(max_linear_acceleration, param.max_linear_acceleration);
  EXPECT_EQ(max_linear_deceleration, param.max_linear_deceleration);
  EXPECT_EQ(max_angular_acceleration, param.max_angular_acceleration);
  EXPECT_EQ(max_angular_deceleration, param.max_angular_deceleration);
  EXPECT_EQ(min_linear_velocity, param.min_linear_velocity);
  EXPECT_EQ(transit_velocity_angular_velocity_ratio, param.transit_velocity_angular_velocity_ratio);
}

/// PathTransitVelocityCalculator::Parameter generation test
/// If rosparam is not set, generate with default values
TEST_F(ParameterCreatorTest, CreatePathTransitVelocityCalculatorParameterDefault) {
  // exercise
  PathTransitVelocityCalculator::Parameter param = CreatePathTransitVelocityCalculatorParameter(test_node_);

  // verify
  EXPECT_EQ(kMaxLinearVelocityDefault, param.max_linear_velocity);
  EXPECT_EQ(kMaxAngularVelocityDefault, param.max_angular_velocity);
  EXPECT_EQ(kMaxLinearAccelerationDefault, param.max_linear_acceleration);
  EXPECT_EQ(kMaxLinearDecelerationDefault, param.max_linear_deceleration);
  EXPECT_EQ(kMaxAngularAccelerationDefault, param.max_angular_acceleration);
  EXPECT_EQ(kMaxAngularDecelerationDefault, param.max_angular_deceleration);
  EXPECT_EQ(kMinLinearVelocityDefault, param.min_linear_velocity);
  EXPECT_EQ(kTransitVelocityAngularVelocityRatioDefault, param.transit_velocity_angular_velocity_ratio);
}
}  // namespace tmc_base_path_follower

int main(int argc, char** argv) {
  // Initialization
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
