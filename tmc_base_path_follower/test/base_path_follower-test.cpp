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
#include <memory>
#include <vector>

#include <gtest/gtest.h>
#include <tmc_base_path_follower/base_path_follower.hpp>
#include "test_mock.hpp"

namespace tmc_base_path_follower {

/// Result of each mock function
struct MockModulesResults {
  MockModulesResults(
      const uint32_t in_nearest_index, const bool in_arrived_goal_area, const bool in_arrived_goal,
      const double in_transit_velocity, const Vector3d& in_calculate_velocity,
      const bool in_calculate_velocity_result)
      : nearest_index(in_nearest_index), arrived_goal_area(in_arrived_goal_area),
        arrived_goal(in_arrived_goal), transit_velocity(in_transit_velocity),
        calculate_velocity(in_calculate_velocity),
        calculate_velocity_result(in_calculate_velocity_result) {}

  uint32_t nearest_index;
  bool arrived_goal_area;
  bool arrived_goal;
  double transit_velocity;
  Vector3d calculate_velocity;
  bool calculate_velocity_result;
};

// Expected output value
struct ExpectedOutput {
  ExpectedOutput(const bool in_result, const Vector3d& in_velocity,
                 const uint32_t in_current_path_index, const bool in_is_arrived_goal)
      : result(in_result), velocity(in_velocity),
        current_path_index(in_current_path_index), is_arrived_goal(in_is_arrived_goal) {}
  bool result;
  Vector3d velocity;
  uint32_t current_path_index;
  bool is_arrived_goal;
};

/// Test parameters
struct BasePathFollowerTestParameter {
  BasePathFollowerTestParameter(const MockModulesResults& in_mock_results,
                                const bool in_use_path_transit_velocity,
                                const std::optional<double>& in_expect_arg_transit_velocity,
                                const ExpectedOutput& in_expected_output)
    : mock_results(in_mock_results),
      use_path_transit_velocity(in_use_path_transit_velocity),
      expect_arg_transit_velocity(in_expect_arg_transit_velocity),
      expected_output(in_expected_output) {}
  // Result of each mock function
  MockModulesResults mock_results;
  // Setting whether to perform path transit velocity control
  bool use_path_transit_velocity;
  // Expected value of path transit velocity passed to CalculateVelocity
  std::optional<double> expect_arg_transit_velocity;
  // Expected output value
  ExpectedOutput expected_output;
};

/// Test of the BasePathFollower class
/// Test the result of PlanPath against the results of each mocked lower module
class BasePathFollowerTest : public ::testing::TestWithParam<BasePathFollowerTestParameter> {
 public:
  BasePathFollowerTest()
      : mock_nearest_path_point_searcher_(std::make_shared<MockNearestPathPointSearcher>()),
        mock_goal_checker_(std::make_shared<MockGoalChecker>()),
        mock_velocity_calculator_(std::make_shared<MockVelocityCalculator>()),
        mock_path_transit_velocity_calculator_(std::make_shared<MockPathTransitVelocityCalculator>()) {}

 protected:
  virtual void SetUp() {
    using ::testing::_;
    using ::testing::DoAll;
    using ::testing::SetArgReferee;
    using ::testing::Return;
    // Set the behavior of each mock according to the parameters
    BasePathFollowerTestParameter param = (BasePathFollowerTestParameter)GetParam();
    const MockModulesResults mock_results = param.mock_results;

    if (param.use_path_transit_velocity) {
      base_path_follower_.reset(new BasePathFollower(mock_nearest_path_point_searcher_,
                                                     mock_goal_checker_,
                                                     mock_velocity_calculator_,
                                                     mock_path_transit_velocity_calculator_));
    } else {
      // If use_path_transit_velocity is false, PathTransitVelocityCalculator is passed as nullptr
      base_path_follower_.reset(new BasePathFollower(mock_nearest_path_point_searcher_,
                                                     mock_goal_checker_,
                                                     mock_velocity_calculator_,
                                                     nullptr));
    }
    // Set the result of SearchNearestPathPointIndex
    ON_CALL(*mock_nearest_path_point_searcher_,
            SearchNearestPathPointIndex(_, _, _, _))
            .WillByDefault(Return(mock_results.nearest_index));
    // Set the result of CheckGoal
    ON_CALL(*mock_goal_checker_,
            CheckGoal(_, _, _, _))
            .WillByDefault(DoAll(SetArgReferee<2>(mock_results.arrived_goal_area),
                                 SetArgReferee<3>(mock_results.arrived_goal)));
    // Set the result of GetPathTransitVelocity
    ON_CALL(*mock_path_transit_velocity_calculator_,
            GetPathTransitVelocity(_))
            .WillByDefault(Return(mock_results.transit_velocity));
    // Set the result of CalculateVelocity
    // Also verify if the passed transit_velocity is correct
    ON_CALL(*mock_velocity_calculator_,
            CalculateVelocity(_, _, _, _, _, _, param.expect_arg_transit_velocity, _))
            .WillByDefault(DoAll(SetArgReferee<7>(mock_results.calculate_velocity),
                                 Return(mock_results.calculate_velocity_result)));
  }

  virtual void TearDown() {}

  // Mock module
  MockNearestPathPointSearcher::Ptr mock_nearest_path_point_searcher_;
  MockGoalChecker::Ptr mock_goal_checker_;
  MockVelocityCalculator::Ptr mock_velocity_calculator_;
  MockPathTransitVelocityCalculator::Ptr mock_path_transit_velocity_calculator_;
  // Test subject
  BasePathFollower::Ptr base_path_follower_;
};

// Test case
INSTANTIATE_TEST_CASE_P(
    BasePathFollowerFollowPathVelocityTest,
    BasePathFollowerTest,
    testing::Values(
        // use_path_transit_velocity is true
        // The transit velocity passed to CalculateVelocity is the value output by GetPathTransitVelocity
        // The velocity output by CalculateVelocity is returned
        BasePathFollowerTestParameter(
            MockModulesResults(1, false, false, 1.0, Vector3d(0.1, 0.2, 0.3), true),
            true, 1.0,
            ExpectedOutput(true, Vector3d(0.1, 0.2, 0.3), 1, false)),
        // use_path_transit_velocity is false
        // The transit velocity passed to CalculateVelocity is std::nullopt
        // The velocity output by CalculateVelocity is returned
        BasePathFollowerTestParameter(
            MockModulesResults(1, false, false, 1.0, Vector3d(0.1, 0.2, 0.3), true),
            false, std::nullopt,
            ExpectedOutput(true, Vector3d(0.1, 0.2, 0.3), 1, false)),
        // arrived_goal_area is true, arrived_goal is false
        // Goal arrival judgment outputs false
        BasePathFollowerTestParameter(
            MockModulesResults(1, true, false, 1.0, Vector3d(0.1, 0.2, 0.3), true),
            true, 1.0,
            ExpectedOutput(true, Vector3d(0.1, 0.2, 0.3), 1, false)),
        // arrived_goal_area is false, arrived_goal is true
        // Goal arrival judgment outputs true, and stop velocity is output
        BasePathFollowerTestParameter(
            MockModulesResults(1, false, true, 1.0, Vector3d(0.1, 0.2, 0.3), true),
            true, 1.0,
            ExpectedOutput(true, Vector3d::Zero(), 1, true)),
        // arrived_goal_area is true, arrived_goal is true
        // Goal arrival judgment outputs true, and stop velocity is output
        BasePathFollowerTestParameter(
            MockModulesResults(1, true, true, 1.0, Vector3d(0.1, 0.2, 0.3), true),
            true, 1.0,
            ExpectedOutput(true, Vector3d::Zero(), 1, true)),
        // The current path index is output as the value output by NearestPathPointSearcher
        BasePathFollowerTestParameter(
            MockModulesResults(5, false, false, 1.0, Vector3d(0.1, 0.2, 0.3), true),
            true, 1.0,
            ExpectedOutput(true, Vector3d(0.1, 0.2, 0.3), 5, false)),
        // If velocity calculation returns failure, velocity 0 is output and failure is returned
        BasePathFollowerTestParameter(
            MockModulesResults(1, false, false, 1.0, Vector3d(0.1, 0.2, 0.3), false),
            true, 1.0,
            ExpectedOutput(false, Vector3d::Zero(), 1, false))
    )
);


TEST_P(BasePathFollowerTest, FollowPathVelocityTest) {
  // setup
  const PathInfo path_info;
  const Pose2d global_pose(0.0, 0.0, 0.0);
  const Vector3d last_velocity = Vector3d::Zero();
  const double time_interval = 0.01;
  bool is_arrived_goal = false;
  uint32_t current_path_index = 0;
  const ExpectedOutput expected_output = ((BasePathFollowerTestParameter)GetParam()).expected_output;

  // exercise
  base_path_follower_->Initialize(path_info);
  Vector3d output_velocity;
  const bool result = base_path_follower_->FollowPathVelocity(
      global_pose, last_velocity, time_interval, is_arrived_goal, current_path_index, output_velocity);

  // verify
  EXPECT_EQ(expected_output.result, result);
  EXPECT_FLOAT_EQ(expected_output.velocity(0), output_velocity(0));
  EXPECT_FLOAT_EQ(expected_output.velocity(1), output_velocity(1));
  EXPECT_FLOAT_EQ(expected_output.velocity(2), output_velocity(2));
  EXPECT_EQ(expected_output.current_path_index, current_path_index);
  EXPECT_EQ(expected_output.is_arrived_goal, is_arrived_goal);
}
}  // namespace tmc_base_path_follower

int main(int argc, char** argv) {
  ::testing::FLAGS_gtest_death_test_style = "fast";
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

