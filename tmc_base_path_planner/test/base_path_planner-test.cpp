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
#include <tmc_base_path_planner/base_path_planner.hpp>

#include <memory>
#include <vector>
#include <gtest/gtest.h>
#include <tmc_base_path_planner/common.hpp>
#include "test_mock.hpp"
#include "test_utils.hpp"

namespace tmc_base_path_planner {

/// Results of each mock function
struct MockModulesReturn {
  MockModulesReturn(const BasePathPlannerErrorCode& in_ret_check_condition,
                    const bool in_ret_plan_grid_path, const bool in_ret_update_path, const bool in_ret_smoothing_path)
      : ret_check_condition(in_ret_check_condition), ret_plan_grid_path(in_ret_plan_grid_path),
        ret_update_path(in_ret_update_path), ret_smoothing_path(in_ret_smoothing_path) {}

  BasePathPlannerErrorCode ret_check_condition;
  bool ret_plan_grid_path;
  bool ret_update_path;
  bool ret_smoothing_path;
};

/// Test parameters
struct BasePathPlanerTestParameter {
  BasePathPlanerTestParameter(const bool in_skip_none_update,
                              const MockModulesReturn& in_mock_return,
                              const BasePathPlannerErrorCode in_expected_error_code)
    : skip_none_update(in_skip_none_update),
      mock_return(in_mock_return), expected_error_code(in_expected_error_code) {}
  // Setting whether to skip if there are no updates
  bool skip_none_update;
  // Results of each mock function
  MockModulesReturn mock_return;
  // Expected value of PlanPath result
  BasePathPlannerErrorCode expected_error_code;
};

/// Test of the BasePathPlanner class
/// Test the results of PlanPath against the results of each mocked lower module
class BasePathPlanerTest : public ::testing::TestWithParam<BasePathPlanerTestParameter> {
 public:
  BasePathPlanerTest()
      : mock_condition_checker_(std::make_shared<MockConditionChecker>()),
        mock_map_filter_(std::make_shared<MockMapFilter>()),
        mock_path_updater_(std::make_shared<MockPathUpdater>()),
        mock_path_smoother_(std::make_shared<MockPathSmoother>()),
        mock_planner_core_(std::make_shared<MockPathPlannerCore>()) {
    base_path_planner_.reset(new BasePathPlanner(
        std::make_shared<CostMap>(Pose2d(), 0.05, 10, 10, std::vector<unsigned char>(100, 0)),
        mock_condition_checker_,
        mock_map_filter_,
        mock_path_updater_,
        mock_path_smoother_,
        mock_planner_core_));
    smoothed_path_.push_back(Pose2d(0.0, 1.0, 0.0));
    smoothed_path_.push_back(Pose2d(0.0, 2.0, 0.0));
    smoothed_path_.push_back(Pose2d(0.0, 3.0, 0.0));
  }

 protected:
  virtual void SetUp() {
    using ::testing::_;
    using ::testing::DoAll;
    using ::testing::SetArgReferee;
    using ::testing::Return;
    const MockModulesReturn mock_return = ((BasePathPlanerTestParameter)GetParam()).mock_return;
    // ClearPrevPath is always called once during initialization
    // If CheckCondition, PlanPath, or SmoothingPath fails, it is called an additional time
    if (mock_return.ret_check_condition == BasePathPlannerErrorCode::kSuccess &&
        mock_return.ret_plan_grid_path &&
        mock_return.ret_smoothing_path) {
      EXPECT_CALL(*mock_path_updater_,
                  ClearPrevPath())
                  .Times(1);
    } else {
      EXPECT_CALL(*mock_path_updater_,
                  ClearPrevPath())
                  .Times(2);
    }
    // Configure the behavior of each mock according to the parameters
    // Set the return value of CheckCondition
    ON_CALL(*mock_condition_checker_,
            CheckCondition(_, _, _, _, _, _, _))
            .WillByDefault(Return(mock_return.ret_check_condition));
    // Set the return value of PlanPath
    ON_CALL(*mock_planner_core_,
            PlanPath(_, _, _, _, _, _, _))
            .WillByDefault(Return(mock_return.ret_plan_grid_path));
    // Set the return value of UpdatePath
    ON_CALL(*mock_path_updater_,
            SearchStartPoseOnPrevPath(_, _))
            .WillByDefault(Return(0));
    ON_CALL(*mock_path_updater_,
            UpdatePath(_, _, _))
            .WillByDefault(Return(mock_return.ret_update_path));
    // Set the return value of SmoothingPath
    // Specify the output path as well
    ON_CALL(*mock_path_smoother_,
            SmoothingPath(_, _))
            .WillByDefault(DoAll(SetArgReferee<1>(smoothed_path_),
                           Return(mock_return.ret_smoothing_path)));
  }

  virtual void TearDown() {}

  MockConditionChecker::Ptr mock_condition_checker_;
  MockMapFilter::Ptr mock_map_filter_;
  MockPathUpdater::Ptr mock_path_updater_;
  MockPathSmoother::Ptr mock_path_smoother_;
  MockPathPlannerCore::Ptr mock_planner_core_;
  BasePathPlanner::Ptr base_path_planner_;
  PoseSeq smoothed_path_;
};

// Test cases
INSTANTIATE_TEST_CASE_P(
    BasePathPlanerPlanPathTest,
    BasePathPlanerTest,
    testing::Values(
        // If all lower modules return success, kSuccess is returned
        BasePathPlanerTestParameter(true,
            MockModulesReturn(BasePathPlannerErrorCode::kSuccess, true, true, true),
            BasePathPlannerErrorCode::kSuccess),
        // If CheckCondition returns something other than kSuccess, the same error as CheckCondition is returned
        BasePathPlanerTestParameter(true,
            MockModulesReturn(BasePathPlannerErrorCode::kRobotIsOutOfMap, true, true, true),
            BasePathPlannerErrorCode::kRobotIsOutOfMap),
        // If PlanPath returns a failure, kPlanningFail is returned
        BasePathPlanerTestParameter(true,
            MockModulesReturn(BasePathPlannerErrorCode::kSuccess, false, true, true),
            BasePathPlannerErrorCode::kPlanningFail),
        // If skip_none_update is true and UpdatePath returns no update needed, kSkip is returned
        BasePathPlanerTestParameter(true,
            MockModulesReturn(BasePathPlannerErrorCode::kSuccess, true, false, true),
            BasePathPlannerErrorCode::kSkip),
        // If skip_none_update is false and UpdatePath returns no update needed, kSuccess is returned
        BasePathPlanerTestParameter(false,
            MockModulesReturn(BasePathPlannerErrorCode::kSuccess, true, false, true),
            BasePathPlannerErrorCode::kSuccess),
        // If SmoothingPath returns a failure, kSmoothingFail is returned
        BasePathPlanerTestParameter(true,
            MockModulesReturn(BasePathPlannerErrorCode::kSuccess, true, true, false),
            BasePathPlannerErrorCode::kSmoothingFail)
    )
);


TEST_P(BasePathPlanerTest, PlanPathTest) {
  const BasePathPlanerTestParameter test_param = ((BasePathPlanerTestParameter)GetParam());

  // setup
  Pose2d start_pose(1.0, 1.0, 0.0);
  Pose2d global_pose(2.0, 2.0, 0.0);
  Pose2d goal_pose(3.0, 3.0, 0.0);
  Pose2d dynamic_map_origin(1.0, 1.0, 0.0);
  CostMapPtr dynamic_map = std::make_shared<CostMap>(
      dynamic_map_origin, 0.05, 5, 5, std::vector<unsigned char>(25, 0));
  // exercise
  base_path_planner_->Initialize();
  PoseSeq out_path;
  BasePathPlannerErrorCode error_code;
  error_code = base_path_planner_->PlanPath(
      start_pose, goal_pose, global_pose, dynamic_map, dynamic_map_origin, test_param.skip_none_update, out_path);

  // verify
  const BasePathPlannerErrorCode expected_error_code = test_param.expected_error_code;
  // The return value of PlanPath matches the expected value
  EXPECT_EQ(expected_error_code, error_code);

  // The output path in case of successful route planning matches the output path of SmoothingPath
  if (error_code == BasePathPlannerErrorCode::kSuccess) {
    EXPECT_TRUE(IsMatchPaths(smoothed_path_, out_path));
  }
}
}  // namespace tmc_base_path_planner

int main(int argc, char** argv) {
  ::testing::FLAGS_gtest_death_test_style = "fast";
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

