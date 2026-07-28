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
#include <memory>
#include <optional>

#include <gtest/gtest.h>

#include <tmc_base_path_planner/path_updater.hpp>
#include "test_utils.hpp"

namespace tmc_base_path_planner {

/// Parameter test
/// Parameters can be generated
TEST(PathUpdaterParameterTest, ConstructParameter) {
  // exercise
  const PathUpdater::Parameter param = PathUpdater::Parameter(0.1, 0.2, 1);

  // verify
  EXPECT_EQ(0.1, param.distance_on_prev_path);
  EXPECT_EQ(0.2, param.grid_error);
  EXPECT_EQ(1, param.same_point_num_merge_path);
}


/// Parameter test
/// If invalid values are specified, default values are used for generation
TEST(PathUpdaterParameterTest, ConstructWithInvalidParameterMakeDefault) {
  // exercise
  const PathUpdater::Parameter invalid_param = PathUpdater::Parameter(0.0, -0.1, -1);

  // verify
  EXPECT_EQ(kDistanceOnPrevPathDefault, invalid_param.distance_on_prev_path);
  EXPECT_EQ(kGridErrorDefault, invalid_param.grid_error);
  EXPECT_EQ(kSamePointNumMergePathDefault, invalid_param.same_point_num_merge_path);
}


/// SearchStartPoseOnPrevPath test
/// If there is no previous path, no nearest point is found, and the input position becomes the start position
TEST(SearchStartPoseOnPrevPathTest, NoPrevPath) {
  // setup
  PathUpdater path_updater = PathUpdater(
    PathUpdater::Parameter(kDistanceOnPrevPathDefault, kGridErrorDefault, kSamePointNumMergePathDefault));
  const Pose2d pose = Pose2d(1.0, 1.0, 0.0);

  // exercise
  Pose2d start_pose;
  std::optional<uint32_t> start_index_on_prev_path = path_updater.SearchStartPoseOnPrevPath(pose, start_pose);

  // verify
  // No nearest point is found
  EXPECT_FALSE((bool)start_index_on_prev_path);
  // The start position matches the input position
  EXPECT_TRUE(IsMatchPoses(start_pose, pose));
}


/// SearchStartPoseOnPrevPath test
/// Test fixture for distance to the previous path
class SearchStartPoseDistanceTest : public ::testing::Test {
 public:
  SearchStartPoseDistanceTest() {
    path_updater_ = std::make_shared<PathUpdater>(
        PathUpdater::Parameter(kDistanceOnPrevPathDefault, kGridErrorDefault, kSamePointNumMergePathDefault));
  }

 protected:
  virtual void SetUp() {
    near_index_ = 5;
    for (uint32_t i = 0; i < 30; ++i) {
      const Pose2d path_point(i * 1.0, i * 1.0, 0.0);
      prev_path_.push_back(path_point);
    }
    PoseSeq update_path;
    path_updater_->UpdatePath(prev_path_, std::nullopt, update_path);
  }
  PathUpdater::Ptr path_updater_;
  // Previous path
  PoseSeq prev_path_;
  // Nearest index
  int32_t near_index_;
};

/// SearchStartPoseDistance test
/// If the previous path is closer than the threshold, the start position is found on the previous path
TEST_F(SearchStartPoseDistanceTest, FoundStartPoseNearByPrevPath) {
  // setup
  // Shift the point on the previous path by less than the threshold in the x-direction to set it as the self-position
  const Pose2d global_pose = Pose2d(prev_path_[near_index_].x() + kDistanceOnPrevPathDefault - 0.01,
                                    prev_path_[near_index_].y(), 0.0);
  // exercise
  Pose2d start_pose;
  std::optional<uint32_t> start_index_on_prev_path = path_updater_->SearchStartPoseOnPrevPath(
      global_pose, start_pose);

  // verify
  // The nearest point is found, and it is output as the start position
  ASSERT_TRUE((bool)start_index_on_prev_path);
  EXPECT_EQ(near_index_, start_index_on_prev_path.value());
  EXPECT_TRUE(IsMatchPoses(start_pose, prev_path_[near_index_]));
}


/// SearchStartPoseDistance test
/// If the previous path is farther than the threshold, no start position is found on the previous path, and the self-position becomes the start position
TEST_F(SearchStartPoseDistanceTest, NotFoundStartPoseFarFromPrevPath) {
  // setup
  // Shift the point on the previous path by more than the threshold in the x-direction to set it as the self-position
  const Pose2d global_pose = Pose2d(prev_path_[near_index_].x() + kDistanceOnPrevPathDefault + 0.01,
                                    prev_path_[near_index_].y(), 0.0);
  // exercise
  Pose2d start_pose;
  std::optional<uint32_t> start_index_on_prev_path = path_updater_->SearchStartPoseOnPrevPath(
      global_pose, start_pose);

  // verify
  // No nearest point is found, and the self-position is output as the start position
  EXPECT_FALSE((bool)start_index_on_prev_path);
  EXPECT_TRUE(IsMatchPoses(start_pose, global_pose));
}


/// SearchStartPoseDistance test
/// By clearing the previous path with ClearPrevPath, the previous path is cleared, no start position is found on the previous path, and the self-position becomes the start position
TEST_F(SearchStartPoseDistanceTest, ClearPrevPath) {
  // setup
  // Shift the point on the previous path by less than the threshold in the x-direction to set it as the self-position
  const Pose2d global_pose = Pose2d(prev_path_[near_index_].x() + kDistanceOnPrevPathDefault - 0.01,
                                    prev_path_[near_index_].y(), 0.0);
  // exercise
  path_updater_->ClearPrevPath();
  Pose2d start_pose;
  std::optional<uint32_t> start_index_on_prev_path = path_updater_->SearchStartPoseOnPrevPath(
      global_pose, start_pose);

  // verify
  // No nearest point is found, and the self-position is output as the start position
  EXPECT_FALSE((bool)start_index_on_prev_path);
  EXPECT_TRUE(IsMatchPoses(start_pose, global_pose));
}

/// UpdatePath test fixture
class UpdatePathTest : public ::testing::Test {
 public:
  UpdatePathTest() {
    path_updater_ = std::make_shared<PathUpdater>(
        PathUpdater::Parameter(kDistanceOnPrevPathDefault, kGridErrorDefault, kSamePointNumMergePathDefault));
  }

 protected:
  virtual void SetUp() {
    start_index_on_prev_path_ = 10;
    for (uint32_t i = 0; i < 30; ++i) {
      const Pose2d path_point(i * 1.0, i * 1.0, 0.0);
      prev_path_.push_back(path_point);
    }
    PoseSeq update_path;
    const bool is_needed_update = path_updater_->UpdatePath(prev_path_, std::nullopt, update_path);
    // The first input returns that an update is needed
    EXPECT_TRUE(is_needed_update);
    // The output path matches the input path
    EXPECT_TRUE(IsMatchPaths(update_path, prev_path_));
  }
  PathUpdater::Ptr path_updater_;
  // Previous path
  PoseSeq prev_path_;
  // Index on the previous path
  uint32_t start_index_on_prev_path_;
};


/// UpdatePath test
/// If a path that matches the previous path from the back is input, no update is needed
TEST_F(UpdatePathTest, BackwardMatch) {
  // setup
  PoseSeq path;
  // Create a path that matches the previous path from the back
  for (uint32_t i = start_index_on_prev_path_; i < prev_path_.size(); ++i) {
    path.push_back(prev_path_[i]);
  }

  // exercise
  PoseSeq update_path;
  const bool is_needed_update = path_updater_->UpdatePath(path, start_index_on_prev_path_, update_path);

  // verify
  // Returns that no update is needed
  EXPECT_FALSE(is_needed_update);
  // The output path matches the previous path
  EXPECT_TRUE(IsMatchPaths(update_path, prev_path_));
}


/// UpdatePath test
/// If a path is input that does not match the goal of the previous path, a full update is performed
TEST_F(UpdatePathTest, GoalChange) {
  // setup
  PoseSeq path;
  // Create a path that matches the previous path from the back except for the goal
  for (uint32_t i = start_index_on_prev_path_; i < prev_path_.size() - 1; ++i) {
    path.push_back(prev_path_[i]);
  }
  // Slightly move the goal
  path.push_back(Pose2d(prev_path_.back().x() + 0.001, prev_path_.back().y(), prev_path_.back().theta()));

  // exercise
  PoseSeq update_path;
  const bool is_needed_update = path_updater_->UpdatePath(path, start_index_on_prev_path_, update_path);

  // verify
  // Returns that an update is needed
  EXPECT_TRUE(is_needed_update);
  // The output path matches the input path
  EXPECT_TRUE(IsMatchPaths(update_path, path));
}


/// UpdatePath test
/// If the changes to each point (except the goal) on the previous path are smaller than the threshold, no update is needed
TEST_F(UpdatePathTest, ChangeLessThanGridError) {
  // setup
  PoseSeq path;
  // Move all points except the goal on the previous path by less than the threshold
  for (uint32_t i = start_index_on_prev_path_; i < prev_path_.size() - 1; ++i) {
    const Pose2d path_point = Pose2d(prev_path_[i].x() + kGridErrorDefault - 0.01,
                                     prev_path_[i].y() + kGridErrorDefault - 0.01,
                                     prev_path_[i].theta());
    path.push_back(path_point);
  }
  // The goal matches
  path.push_back(prev_path_.back());

  // exercise
  PoseSeq update_path;
  const bool is_needed_update = path_updater_->UpdatePath(path, start_index_on_prev_path_, update_path);

  // verify
  // Returns that no update is needed
  EXPECT_FALSE(is_needed_update);
  // The output path matches the previous path
  EXPECT_TRUE(IsMatchPaths(update_path, prev_path_));
}


/// UpdatePath test
/// If changes occur from the beginning of the previous path, a full update is performed
TEST_F(UpdatePathTest, ChangeGreaterThanGridErrorAtBeginning) {
  // setup
  PoseSeq path;
  // Create a path that matches the previous path from the back
  for (uint32_t i = start_index_on_prev_path_; i < prev_path_.size(); ++i) {
    path.push_back(prev_path_[i]);
  }
  // Move the points at the beginning by more than the threshold
  path[kSamePointNumMergePathDefault] = Pose2d(
      path[kSamePointNumMergePathDefault].x() + kGridErrorDefault + 0.01,
      path[kSamePointNumMergePathDefault].y() + kGridErrorDefault + 0.01,
      path[kSamePointNumMergePathDefault].theta());

  // exercise
  PoseSeq update_path;
  const bool is_needed_update = path_updater_->UpdatePath(path, start_index_on_prev_path_, update_path);

  // verify
  // Returns that an update is needed
  EXPECT_TRUE(is_needed_update);
  // The output path matches the input path
  EXPECT_TRUE(IsMatchPaths(update_path, path));
}


/// UpdatePath test
/// If changes occur from the middle of the previous path, a merge update is performed
TEST_F(UpdatePathTest, ChangeGreaterThanGridErrorAtMiddle) {
  // setup
  PoseSeq path;
  // Create a path that matches the previous path from the back
  for (uint32_t i = start_index_on_prev_path_; i < prev_path_.size(); ++i) {
    path.push_back(prev_path_[i]);
  }
  // Move the points in the middle by more than the threshold
  path[kSamePointNumMergePathDefault + 1] = Pose2d(
      path[kSamePointNumMergePathDefault + 1].x() + kGridErrorDefault + 0.01,
      path[kSamePointNumMergePathDefault + 1].y() + kGridErrorDefault + 0.01,
      path[kSamePointNumMergePathDefault + 1].theta());

  // exercise
  PoseSeq update_path;
  const bool is_needed_update = path_updater_->UpdatePath(path, start_index_on_prev_path_, update_path);

  // verify
  // Returns that an update is needed
  EXPECT_TRUE(is_needed_update);
  PoseSeq expect_path;
  for (uint32_t i = 0; i < start_index_on_prev_path_; ++i) {
    expect_path.push_back(prev_path_[i]);
  }
  for (uint32_t i = 0; i < path.size(); ++i) {
    expect_path.push_back(path[i]);
  }
  // The previous path and the input path are concatenated and output
  EXPECT_TRUE(IsMatchPaths(update_path, expect_path));
}


/// UpdatePath test
/// Even if the previous path matches from the back, clearing the previous path with ClearPrevPath results in a full update
TEST_F(UpdatePathTest, ClearPrevPath) {
  // setup
  PoseSeq path;
  // Create a path that matches the previous path from the back
  for (uint32_t i = start_index_on_prev_path_; i < prev_path_.size(); ++i) {
    path.push_back(prev_path_[i]);
  }

  // exercise
  path_updater_->ClearPrevPath();
  PoseSeq update_path;
  const bool is_needed_update = path_updater_->UpdatePath(path, start_index_on_prev_path_, update_path);

  // verify
  // Returns that an update is needed
  EXPECT_TRUE(is_needed_update);
  // The output path matches the input path
  EXPECT_TRUE(IsMatchPaths(update_path, path));
}
}  // namespace tmc_base_path_planner

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
