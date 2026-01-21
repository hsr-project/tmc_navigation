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
#include <tmc_base_path_planner/condition_checker.hpp>

#include <memory>
#include <vector>
#include <gtest/gtest.h>

namespace tmc_base_path_planner {

/// Test fixture for CheckCondition
class CheckConditionTest : public ::testing::Test {
 public:
  CheckConditionTest() = default;

 protected:
  virtual void SetUp() {
    condition_checker_ = std::make_shared<ConditionChecker>();
    /// Set to clear conditions and return kSuccess by default
    /// Use this as a base to test by changing conditions in each test pattern
    // Create a static map with the upper half as Free and the lower half as Wall in a 4mx4m area
    CreateStaticMap(0.05, 80, 80);
    static_map_occupancy_threshold_ = kWallValue - 1;

    // Create a dynamic map with the left half as Free and the right half as Wall in a 2mx2m area
    // Place the origin so that the center of the map matches the static map
    CreateDynamicMap(Pose2d(1.0, 1.0, 0.0), 0.05, 40, 40);

    // Place the start, self-position, and goal in the Free area of both static and dynamic maps
    start_pose_ = Pose2d(1.1, 1.1, 0.0);
    global_pose_ = Pose2d(1.2, 1.2, 0.0);
    goal_pose_ = Pose2d(1.5, 1.5, 0.0);
  }

  ConditionChecker::Ptr condition_checker_;
  // Static map
  CostMapPtr static_map_;
  // Wall threshold for static map
  uint32_t static_map_occupancy_threshold_;
  // Dynamic map
  CostMapPtr dynamic_map_;
  // Dynamic map origin
  Pose2d dynamic_map_origin_;
  // Start
  Pose2d start_pose_;
  // Goal
  Pose2d goal_pose_;
  // Self-position
  Pose2d global_pose_;

  // Generate a static map. Make the upper half Free and the lower half Wall
  void CreateStaticMap(const double map_resolution, const uint32_t map_width, const uint32_t map_height) {
    std::vector<unsigned char> map_data;
    // Initialize as Free
    map_data.resize(map_width * map_height, kFreeGrid);
    // Make the lower half Wall
    for (uint32_t v = map_height / 2; v < map_height; ++v) {
      for (uint32_t u = 0; u < map_width; ++u) {
        map_data[u + v * map_width] = kWallValue;
      }
    }
    static_map_.reset(new CostMap(Pose2d(), map_resolution, map_width, map_height, map_data));
  }

  // Generate a dynamic map. Make the left half Free and the right half Wall
  void CreateDynamicMap(const Pose2d& map_origin, const double map_resolution,
                        const uint32_t map_width, const uint32_t map_height) {
    std::vector<unsigned char> map_data;
    // Initialize as Free
    map_data.resize(map_width * map_height, kFreeGrid);
    // Make the right half Wall
    for (uint32_t v = 0; v < map_height; ++v) {
      for (uint32_t u = map_width / 2; u < map_width; ++u) {
        map_data[u + v * map_width] = kWallValue;
      }
    }
    dynamic_map_origin_ = map_origin;
    dynamic_map_.reset(new CostMap(dynamic_map_origin_, map_resolution, map_width, map_height, map_data));
  }
};


/// CheckCondition test
/// Return kSuccess if all conditions are met
TEST_F(CheckConditionTest, Success) {
  // exercise
  BasePathPlannerErrorCode error_code = condition_checker_->CheckCondition(
      static_map_, static_map_occupancy_threshold_, dynamic_map_, dynamic_map_origin_,
      start_pose_, goal_pose_, global_pose_);

  // verify
  EXPECT_EQ(BasePathPlannerErrorCode::kSuccess, error_code);
}


/// CheckCondition test
/// Return kRobotIsOutOfMap if the robot position is outside the range of the static map
TEST_F(CheckConditionTest, RobotIsOutOfMap) {
  // setup
  global_pose_.set_y((static_map_->max_v() + 1) * static_map_->resolution() + 0.01);

  // exercise
  BasePathPlannerErrorCode error_code = condition_checker_->CheckCondition(
      static_map_, static_map_occupancy_threshold_, dynamic_map_, dynamic_map_origin_,
      start_pose_, goal_pose_, global_pose_);

  // verify
  EXPECT_EQ(BasePathPlannerErrorCode::kRobotIsOutOfMap, error_code);
}


/// CheckCondition test
/// Return kGoalIsOnStaticObstacle if the goal position is on a prohibited area of the static map
TEST_F(CheckConditionTest, GoalIsOnStaticObstacle) {
  // setup
  goal_pose_.set_y(static_map_->max_v() * static_map_->resolution());

  // exercise
  BasePathPlannerErrorCode error_code = condition_checker_->CheckCondition(
      static_map_, static_map_occupancy_threshold_, dynamic_map_, dynamic_map_origin_,
      start_pose_, goal_pose_, global_pose_);

  // verify
  EXPECT_EQ(BasePathPlannerErrorCode::kGoalIsOnStaticObstacle, error_code);
}


/// CheckCondition test
/// Return kGoalIsOnStaticObstacle if the goal position is outside the range of the static map
TEST_F(CheckConditionTest, GoalIsOutOfMap) {
  // setup
  goal_pose_.set_y((static_map_->max_v() + 1) * static_map_->resolution() + 0.01);

  // exercise
  BasePathPlannerErrorCode error_code = condition_checker_->CheckCondition(
      static_map_, static_map_occupancy_threshold_, dynamic_map_, dynamic_map_origin_,
      start_pose_, goal_pose_, global_pose_);

  // verify
  EXPECT_EQ(BasePathPlannerErrorCode::kGoalIsOnStaticObstacle, error_code);
}

/// CheckCondition test
/// Return kGoalIsOnDynamicObstacle if the goal position is on a prohibited area of the dynamic map
TEST_F(CheckConditionTest, GoalIsOnDynamicObstacle) {
  // setup
  goal_pose_.set_x(dynamic_map_origin_.x() + (dynamic_map_->max_u()) * dynamic_map_->resolution());

  // exercise
  BasePathPlannerErrorCode error_code = condition_checker_->CheckCondition(
      static_map_, static_map_occupancy_threshold_, dynamic_map_, dynamic_map_origin_,
      start_pose_, goal_pose_, global_pose_);

  // verify
  EXPECT_EQ(BasePathPlannerErrorCode::kGoalIsOnDynamicObstacle, error_code);
}

}  // namespace tmc_base_path_planner

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
