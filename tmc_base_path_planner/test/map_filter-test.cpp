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

#include <tmc_base_path_planner/map_filter.hpp>

namespace tmc_base_path_planner {

/// Parameter test
/// Able to generate parameters
TEST(MapFilterParameterTest, ConstructParameter) {
  // exercise
  const MapFilter::Parameter param = MapFilter::Parameter(0.1, 0.2, 0.3);

  // verify
  EXPECT_EQ(0.1, param.map_filter_range_around_start);
  EXPECT_EQ(0.2, param.map_filter_range_around_goal);
  EXPECT_EQ(0.3, param.map_filter_distance_goal_limit);
}


/// Parameter test
/// If an invalid value is specified, it is generated with the default value
TEST(MapFilterParameterTest, ConstructWithInvalidParameterMakeDefault) {
  // exercise
  const MapFilter::Parameter invalid_param = MapFilter::Parameter(-0.1, -0.1, -0.1);

  // verify
  EXPECT_EQ(kMapFilterRangeAroundStartDefault, invalid_param.map_filter_range_around_start);
  EXPECT_EQ(kMapFilterRangeAroundGoalDefault, invalid_param.map_filter_range_around_goal);
  EXPECT_EQ(kMapFilterDistanceGoalLimitDefault, invalid_param.map_filter_distance_goal_limit);
}


/// Test fixture for FilterMapOnStartAndGoal
class FilterMapOnStartAndGoalTest : public ::testing::Test {
 public:
  FilterMapOnStartAndGoalTest() {}

 protected:
  virtual void SetUp() {
    /// By default, set conditions to filter both around the start and goal
    /// Use this as a base to change conditions in each test pattern
    // Generate a map with origin at (x, y)=(1.0, 1.0), size 2m x 2m, all prohibited area
    map_origin_ = Pose2d(1.0, 1.0, 0.0);
    const double map_resolution = 0.05;
    const uint32_t map_width = 40;
    const uint32_t map_height = 40;
    std::vector<unsigned char> data;
    data.resize(map_width * map_height, kWallValue);
    map_ = std::make_shared<CostMap>(map_origin_, map_resolution, map_width, map_height, data);

    // Set to remove areas within a radius of 0.2m around the start and 0.5m around the goal
    param_ = std::make_shared<MapFilter::Parameter>(0.2, 0.5, 0.6);
    map_filter_ = std::make_shared<MapFilter>(*param_);
    /// If the start and goal locations are at the boundary of map_resolution,
    /// Slightly shift them to avoid floating-point errors in filter range judgment
    start_pose_ = Pose2d(1.51, 1.51);
    goal_pose_ = Pose2d(2.51, 2.51);
    /// The self-position is close to the start and sufficiently far from the goal
    global_pose_ = Pose2d(1.50, 1.50);
  }

  MapFilter::Ptr map_filter_;
  std::shared_ptr<MapFilter::Parameter> param_;
  // Map
  CostMapPtr map_;
  // Map origin coordinates
  Pose2d map_origin_;
  // Self-position
  Pose2d global_pose_;
  // Start location
  Pose2d start_pose_;
  // Goal location
  Pose2d goal_pose_;
};


/// FilterMapOnStartAndGoal test
/// Specified range obstacles are removed
TEST_F(FilterMapOnStartAndGoalTest, RemoveObstacleFilterRange) {
  // exercise
  map_filter_->FilterMapOnStartAndGoal(map_, map_origin_, start_pose_, goal_pose_, global_pose_);
  // verify
  for (uint32_t u = 0; u < map_->width(); ++u) {
    for (uint32_t v = 0; v < map_->height(); ++v) {
      unsigned char value;
      map_->GetValueAt(u, v, value);
      const Pose2d pose = Pose2d(u * map_->resolution(), v * map_->resolution(), 0.0) * map_origin_;
      const double distance_to_start = (start_pose_.point() - pose.point()).norm();
      const double distance_to_goal = (goal_pose_.point() - pose.point()).norm();
      if (distance_to_start < param_->map_filter_range_around_start ||
          distance_to_goal < param_->map_filter_range_around_goal) {
        // Areas around the start and goal are removed
        EXPECT_EQ(kFreeGrid, value) << " u, v　is " << u << ", " << v << std::endl;
      } else {
        // Areas other than around the start and goal are not removed
        EXPECT_EQ(kWallValue, value) << " u, v　is " << u << ", " << v << std::endl;
      }
    }
  }
}


/// FilterMapOnStartAndGoal test
/// Perform filtering around the start until the self-position moves away from the start
TEST_F(FilterMapOnStartAndGoalTest, RemoveObstacleAroundStartWhenNearByStart) {
  // setup
  // Set the self-position within the removal range around the start
  global_pose_ = Pose2d(start_pose_.x() + param_->map_filter_range_around_start,
                        start_pose_.y(), 0.0);
  // exercise
  map_filter_->FilterMapOnStartAndGoal(map_, map_origin_, start_pose_, goal_pose_, global_pose_);
  // verify
  for (uint32_t u = 0; u < map_->width(); ++u) {
    for (uint32_t v = 0; v < map_->height(); ++v) {
      unsigned char value;
      map_->GetValueAt(u, v, value);
      const Pose2d pose = Pose2d(u * map_->resolution(), v * map_->resolution(), 0.0) * map_origin_;
      const double distance_to_start = (start_pose_.point() - pose.point()).norm();
      const double distance_to_goal = (goal_pose_.point() - pose.point()).norm();
      if (distance_to_start < param_->map_filter_range_around_start ||
          distance_to_goal < param_->map_filter_range_around_goal) {
        // Areas around the start and goal are removed
        EXPECT_EQ(kFreeGrid, value) << " u, v　is " << u << ", " << v << std::endl;
      } else {
        // Areas other than around the start and goal are not removed
        EXPECT_EQ(kWallValue, value) << " u, v　is " << u << ", " << v << std::endl;
      }
    }
  }
}


/// FilterMapOnStartAndGoal test
/// Do not perform filtering around the start once the self-position moves away from it
TEST_F(FilterMapOnStartAndGoalTest, NotRemoveObstacleAroundStartWhenFarFromStart) {
  // setup
  // Move the self-position slightly outside the removal range around the start
  global_pose_ = Pose2d(start_pose_.x() + param_->map_filter_range_around_start + 0.01,
                        start_pose_.y(), 0.0);
  // exercise
  map_filter_->FilterMapOnStartAndGoal(map_, map_origin_, start_pose_, goal_pose_, global_pose_);
  // verify
  for (uint32_t u = 0; u < map_->width(); ++u) {
    for (uint32_t v = 0; v < map_->height(); ++v) {
      unsigned char value;
      map_->GetValueAt(u, v, value);
      const Pose2d pose = Pose2d(u * map_->resolution(), v * map_->resolution(), 0.0) * map_origin_;
      const double distance_to_goal = (goal_pose_.point() - pose.point()).norm();
      if (distance_to_goal < param_->map_filter_range_around_goal) {
        // Areas around the goal are removed
        EXPECT_EQ(kFreeGrid, value) << " u, v　is " << u << ", " << v << std::endl;
      } else {
        // Areas other than around the goal are not removed
        EXPECT_EQ(kWallValue, value) << " u, v　is " << u << ", " << v << std::endl;
      }
    }
  }
}


/// FilterMapOnStartAndGoal test
/// Perform filtering around the goal until the self-position approaches the goal
TEST_F(FilterMapOnStartAndGoalTest, RemoveObstacleAroundGoalWhenFarFromGoal) {
  // setup
  /// Move the self-position slightly farther from the goal than the distance threshold for removal
  /// This self-position is far from the start, so the area around the start is not removed
  global_pose_ = Pose2d(goal_pose_.x() + param_->map_filter_distance_goal_limit + 0.01,
                        goal_pose_.y(), 0.0);
  // exercise
  map_filter_->FilterMapOnStartAndGoal(map_, map_origin_, start_pose_, goal_pose_, global_pose_);
  // verify
  for (uint32_t u = 0; u < map_->width(); ++u) {
    for (uint32_t v = 0; v < map_->height(); ++v) {
      unsigned char value;
      map_->GetValueAt(u, v, value);
      const Pose2d pose = Pose2d(u * map_->resolution(), v * map_->resolution(), 0.0) * map_origin_;
      const double distance_to_goal = (goal_pose_.point() - pose.point()).norm();
      if (distance_to_goal < param_->map_filter_range_around_goal) {
        // Areas around the goal are removed
        EXPECT_EQ(kFreeGrid, value) << " u, v　is " << u << ", " << v << std::endl;
      } else {
        // Areas other than around the goal are not removed
        EXPECT_EQ(kWallValue, value) << " u, v　is " << u << ", " << v << std::endl;
      }
    }
  }
}


/// FilterMapOnStartAndGoal test
/// Do not perform filtering around the goal once the self-position approaches it
TEST_F(FilterMapOnStartAndGoalTest, NotRemoveObstacleAroundGoalWhenNearByGoal) {
  // setup
  /// Move the self-position slightly closer to the goal than the distance threshold for removal
  /// This self-position is far from the start, so the area around the start is not removed
  global_pose_ = Pose2d(goal_pose_.x() + param_->map_filter_distance_goal_limit - 0.01,
                        goal_pose_.y(), 0.0);
  // exercise
  map_filter_->FilterMapOnStartAndGoal(map_, map_origin_, start_pose_, goal_pose_, global_pose_);
  // verify
  for (uint32_t u = 0; u < map_->width(); ++u) {
    for (uint32_t v = 0; v < map_->height(); ++v) {
      unsigned char value;
      map_->GetValueAt(u, v, value);
      // Not removed
      EXPECT_EQ(kWallValue, value) << " u, v　is " << u << ", " << v << std::endl;
    }
  }
}


/// FilterMapOnStartAndGoal test
/// Even if the self-position is around the start, do not remove if the goal is within the removal area
TEST_F(FilterMapOnStartAndGoalTest, NotRemoveObstacleAroundStartWhenStartNearByGoal) {
  // setup
  // Include the goal position within the removal range around the start
  // Since this self-position is close to the goal, it is expected that the area around the goal is not removed
  goal_pose_ = Pose2d(start_pose_.x() + param_->map_filter_range_around_start - 0.01,
                      start_pose_.y(), 0.0);
  // exercise
  map_filter_->FilterMapOnStartAndGoal(map_, map_origin_, start_pose_, goal_pose_, global_pose_);
  // verify
  for (uint32_t u = 0; u < map_->width(); ++u) {
    for (uint32_t v = 0; v < map_->height(); ++v) {
      unsigned char value;
      map_->GetValueAt(u, v, value);
      // Not removed
      EXPECT_EQ(kWallValue, value) << " u, v　is " << u << ", " << v << std::endl;
    }
  }
}


/// FilterMapOnStartAndGoal test
/// If the parameter setting is not to filter, then it is not filtered
TEST_F(FilterMapOnStartAndGoalTest, NotRemoveObstacleIfNoFilterSetting) {
  // setup
  param_.reset(new MapFilter::Parameter(0.0, 0.0, 0.6));
  map_filter_.reset(new MapFilter(*param_));
  // exercise
  map_filter_->FilterMapOnStartAndGoal(map_, map_origin_, start_pose_, goal_pose_, global_pose_);
  // verify
  for (uint32_t u = 0; u < map_->width(); ++u) {
    for (uint32_t v = 0; v < map_->height(); ++v) {
      unsigned char value;
      map_->GetValueAt(u, v, value);
      // Nothing is removed
      EXPECT_EQ(kWallValue, value) << " u, v　is " << u << ", " << v << std::endl;
    }
  }
}
}  // namespace tmc_base_path_planner

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
