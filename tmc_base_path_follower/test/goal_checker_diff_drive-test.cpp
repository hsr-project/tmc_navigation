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

#include <gtest/gtest.h>

#include <tmc_base_path_follower/common.hpp>
#include <tmc_base_path_follower/goal_checker_diff_drive.hpp>

#include "test_utils.hpp"

namespace tmc_base_path_follower {

/// Parameter test
/// Able to generate parameters
TEST(DiffDriveGoalCheckerParameterTest, ConstructParameter) {
  // setup
  const double goal_area_length = 0.1;
  const double goal_line_length = 0.2;
  const double goal_stop_error_angle = 0.3;
  // exercise
  const DiffDriveGoalChecker::Parameter param = DiffDriveGoalChecker::Parameter(
      goal_area_length, goal_line_length, goal_stop_error_angle);

  // verify
  EXPECT_EQ(goal_area_length, param.goal_area_length);
  EXPECT_EQ(goal_line_length, param.goal_line_length);
  EXPECT_EQ(goal_stop_error_angle, param.goal_stop_error_angle);
}


/// Parameter test
/// If an invalid value is specified, it is generated with the default value
TEST(DiffDriveGoalCheckerParameterTest, ConstructWithInvalidParameterMakeDefault) {
  // setup
  const double goal_area_length = -0.1;
  const double goal_line_length = -0.2;
  const double goal_stop_error_angle = -0.3;
  // exercise
  const DiffDriveGoalChecker::Parameter param = DiffDriveGoalChecker::Parameter(
      goal_area_length, goal_line_length, goal_stop_error_angle);

  // verify
  EXPECT_EQ(kGoalAreaLengthDefault, param.goal_area_length);
  EXPECT_EQ(kGoalLineLengthDefault, param.goal_line_length);
  EXPECT_EQ(kGoalStopErrorAngleDefault, param.goal_stop_error_angle);
}

/// DiffDriveGoalChecker test fixture
class DiffDriveGoalCheckerTest : public ::testing::Test {
 public:
  DiffDriveGoalCheckerTest() {
    // Generate a straight path
    path_ = CreateLinearPath(Pose2d(0.0, 0.0, 0.0), Pose2d(3.0, 0.0, 0.0), 0.5);
  }

  void SetUp() {
    goal_checker_ = std::make_shared<DiffDriveGoalChecker>(
        DiffDriveGoalChecker::Parameter(kGoalAreaLengthDefault, kGoalLineLengthDefault,
                                        kGoalStopErrorAngleDefault));
    goal_checker_->Initialize();
  }

 protected:
  DiffDriveGoalChecker::Ptr goal_checker_;
  PoseSeq path_;
};

/// CheckGoal test
/// If the distance difference between the self-position and the goal is within the threshold, and
/// If the self-position is further along the path than the goal line, it is determined to have entered the goal area
TEST_F(DiffDriveGoalCheckerTest, ArrivedGoalArea) {
  bool is_arrived_goal_area = false;
  bool is_arrived_goal = false;

  goal_checker_->CheckGoal(path_, ArriveGoalAreaPose(path_.back()), is_arrived_goal_area, is_arrived_goal);
  EXPECT_TRUE(is_arrived_goal_area);
}

/// CheckGoal test
/// If the self-position is before the goal line on the path, it is determined not to have entered the goal area
TEST_F(DiffDriveGoalCheckerTest, NotArrivedGoalAreaGoalLine) {
  bool is_arrived_goal_area = false;
  bool is_arrived_goal = false;

  goal_checker_->CheckGoal(path_, AheadGoalLinePose(path_.back()), is_arrived_goal_area, is_arrived_goal);
  EXPECT_FALSE(is_arrived_goal_area);
}

/// CheckGoal test
/// If the distance difference between the self-position and the goal is farther than the threshold, it is determined not to have entered the goal area
TEST_F(DiffDriveGoalCheckerTest, NotArrivedGoalAreaGoalAreaLength) {
  bool is_arrived_goal_area = false;
  bool is_arrived_goal = false;

  goal_checker_->CheckGoal(path_, OutsideGoalAreaPose(path_.back()), is_arrived_goal_area, is_arrived_goal);
  EXPECT_FALSE(is_arrived_goal_area);
}

/// CheckGoal test
/// Once it is determined to have entered the goal area, a margin is allowed in the judgment range, and it is not determined to have exited the goal area even if slightly off
TEST_F(DiffDriveGoalCheckerTest, GoalAreaMergin) {
  bool is_arrived_goal_area = false;
  bool is_arrived_goal = false;

  // Determined to have entered the goal area once
  goal_checker_->CheckGoal(path_, ArriveGoalAreaPose(path_.back()), is_arrived_goal_area, is_arrived_goal);
  EXPECT_TRUE(is_arrived_goal_area);

  // Even if slightly out of the goal area, it is determined to be within the goal area
  goal_checker_->CheckGoal(path_, AheadGoalLinePose(path_.back()), is_arrived_goal_area, is_arrived_goal);
  EXPECT_TRUE(is_arrived_goal_area);
  goal_checker_->CheckGoal(path_, OutsideGoalAreaPose(path_.back()), is_arrived_goal_area, is_arrived_goal);
  EXPECT_TRUE(is_arrived_goal_area);

  // Move the X coordinate twice the distance from the goal line
  Pose2d global_pose = ArriveGoalAreaPose(path_.back());
  const double diff_x = -kGoalLineLengthDefault * 2.0;
  global_pose.set_x(path_.back().x() + diff_x);

  goal_checker_->CheckGoal(path_, global_pose, is_arrived_goal_area, is_arrived_goal);
  // If significantly out of the goal area, it is determined to be outside the goal area
  EXPECT_FALSE(is_arrived_goal_area);
}

/// CheckGoal test
/// In the case of a path with only one point, if the distance difference between the self-position and the goal is within the threshold, it is determined to have entered the goal area
TEST_F(DiffDriveGoalCheckerTest, ArrivedGoalAreaOnePointPath) {
  bool is_arrived_goal_area = false;
  bool is_arrived_goal = false;

  PoseSeq one_point_path;
  one_point_path.push_back(Pose2d(0.0, 0.0, 0.0));
  const Pose2d global_pose(kGoalAreaLengthDefault - kEpsilon, 0.0, 0.0);
  goal_checker_->CheckGoal(one_point_path, global_pose, is_arrived_goal_area, is_arrived_goal);
  EXPECT_TRUE(is_arrived_goal_area);
}

/// CheckGoal test
/// In the case of a path with only one point, if the distance difference between the self-position and the goal is farther than the threshold, it is determined not to have entered the goal area
TEST_F(DiffDriveGoalCheckerTest, NotArrivedGoalAreaOnePointPath) {
  bool is_arrived_goal_area = false;
  bool is_arrived_goal = false;

  PoseSeq one_point_path;
  one_point_path.push_back(Pose2d(0.0, 0.0, 0.0));
  const Pose2d global_pose(kGoalAreaLengthDefault + kEpsilon, 0.0, 0.0);
  goal_checker_->CheckGoal(one_point_path, global_pose, is_arrived_goal_area, is_arrived_goal);
  EXPECT_FALSE(is_arrived_goal_area);
}


/// CheckGoal test
/// If it was previously determined to have entered the goal area,
/// It is determined to have reached the goal if the angle difference between the self-position and the goal is within the threshold
TEST_F(DiffDriveGoalCheckerTest, ArrivedGoal) {
  bool is_arrived_goal_area = false;
  bool is_arrived_goal = false;

  // Determined to have entered the goal area
  goal_checker_->CheckGoal(path_, ArriveGoalAreaPose(path_.back()), is_arrived_goal_area, is_arrived_goal);
  EXPECT_TRUE(is_arrived_goal_area);

  // If the angle difference is within the threshold, it is determined to have reached the goal
  goal_checker_->CheckGoal(path_, NotArrivedGoalLinerPose(path_.back()), is_arrived_goal_area, is_arrived_goal);
  EXPECT_TRUE(is_arrived_goal);
}

/// CheckGoal test
/// If it was previously determined not to have entered the goal area,
/// It is determined not to have reached the goal even if the angle difference between the self-position and the goal is within the threshold
TEST_F(DiffDriveGoalCheckerTest, NotArrivedGoalAreaNotArrivedGoal) {
  bool is_arrived_goal_area = false;
  bool is_arrived_goal = false;

  // Determined not to have entered the goal area
  goal_checker_->CheckGoal(path_, AheadGoalLinePose(path_.back()), is_arrived_goal_area, is_arrived_goal);
  EXPECT_FALSE(is_arrived_goal_area);

  // Even if the position meets the conditions for reaching the goal, it is determined not to have reached the goal
  goal_checker_->CheckGoal(path_, ArriveGoalPose(path_.back()), is_arrived_goal_area, is_arrived_goal);
  EXPECT_FALSE(is_arrived_goal);
}

/// CheckGoal test
/// Even if it was previously determined to have entered the goal area,
/// It is determined not to have reached the goal if the angle difference between the self-position and the goal is not within the threshold
TEST_F(DiffDriveGoalCheckerTest, NotArrivedGoalAngular) {
  bool is_arrived_goal_area = false;
  bool is_arrived_goal = false;

  // Determined to have entered the goal area
  goal_checker_->CheckGoal(path_, ArriveGoalAreaPose(path_.back()), is_arrived_goal_area, is_arrived_goal);
  EXPECT_TRUE(is_arrived_goal_area);

  // Determined not to have reached the goal due to being out of range
  goal_checker_->CheckGoal(path_, NotArrivedGoalAngularPose(path_.back()), is_arrived_goal_area, is_arrived_goal);
  EXPECT_FALSE(is_arrived_goal);
}

/// CheckGoal test
/// When initialized, it should not be determined to have entered the goal area again
/// It is determined not to have reached the goal
TEST_F(DiffDriveGoalCheckerTest, Initialize) {
  bool is_arrived_goal_area = false;
  bool is_arrived_goal = false;

  // Determined to have entered the goal area
  goal_checker_->CheckGoal(path_, ArriveGoalAreaPose(path_.back()), is_arrived_goal_area, is_arrived_goal);
  EXPECT_TRUE(is_arrived_goal_area);
  EXPECT_FALSE(is_arrived_goal);

  // Determined to have reached the goal
  goal_checker_->CheckGoal(path_, ArriveGoalPose(path_.back()), is_arrived_goal_area, is_arrived_goal);
  EXPECT_TRUE(is_arrived_goal_area);
  EXPECT_TRUE(is_arrived_goal);

  // If not initialized, it is determined to have reached the goal
  goal_checker_->CheckGoal(path_, ArriveGoalPose(path_.back()), is_arrived_goal_area, is_arrived_goal);
  EXPECT_TRUE(is_arrived_goal_area);
  EXPECT_TRUE(is_arrived_goal);

  // When initialized, it is determined not to have reached the goal
  goal_checker_->Initialize();
  goal_checker_->CheckGoal(path_, ArriveGoalPose(path_.back()), is_arrived_goal_area, is_arrived_goal);
  EXPECT_TRUE(is_arrived_goal_area);
  EXPECT_FALSE(is_arrived_goal);

  // After initializing, if it is determined to have entered the goal area once, it is determined to have reached the goal
  goal_checker_->CheckGoal(path_, ArriveGoalPose(path_.back()), is_arrived_goal_area, is_arrived_goal);
  EXPECT_TRUE(is_arrived_goal_area);
  EXPECT_TRUE(is_arrived_goal);
}
}  // namespace tmc_base_path_follower

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
