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
#include <tmc_base_path_planner/astar_path_planner.hpp>

#include <memory>
#include <vector>
#include <gtest/gtest.h>

namespace {
// Edge size of the static map
const int32_t kStaticMapSize = 1000;
// Edge size of the dynamic map
const int32_t kDynamicMapSize = 500;
// Map resolution
const double kResolution = 0.05;
// Map parameters
const tmc_astar_lib::LayeredCostMap::Parameter kMapParam(0.2, 0.5, 1.0, 100.0, 0, 50, 71);
// Origin of the dynamic map
const Pose2d kDynamicMapOrigin = Pose2d(12.5, 12.5, 0.0);
// Position of a single-point obstacle
// To avoid rounding errors when converting map to grid coordinates, resolution/2 is added to point to the center of the grid
const Pose2d kPointObstalce(30.0 + (kResolution / 2.0), 25.0 + (kResolution / 2.0), 0.0);
// Position of rectangular obstacles
const Pose2d kRectObstacleLeftBottom(32.5 + (kResolution / 2.0), 27.5 + (kResolution / 2.0), 0.0);
const Pose2d kRectObstacleRightTop(37.5 + (kResolution / 2.0), 32.5 + (kResolution / 2.0), 0.0);
// Start and goal for normal test cases
const Pose2d kStart(30.0 + (kResolution / 2.0), 30.0 + (kResolution / 2.0), 0.0);
const Pose2d kGoal(30.0 + (kResolution / 2.0), 32.0 + (kResolution / 2.0), 0.0);
// Goal located within the same grid as the start
const Pose2d kGoalSameGrid(30.0 + (kResolution / 3.0), 30.0 + (kResolution / 4.0), 0.0);
}  // anonymous namespace

namespace tmc_base_path_planner {

/// Test fixture for AstarPathPlanner
class AstarPathPlannerTest : public ::testing::Test {
 public:
  AstarPathPlannerTest() {}

 protected:
  virtual void SetUp() {
    // Generation of the static map
    Pose2d static_map_origin(0.0, 0.0, 0.0);
    std::vector<uint8_t> static_map_data(kStaticMapSize * kStaticMapSize, 1);
    // Place a single-point and square-shaped obstacle for success/failure determination
    const int32_t point_obstacle_x = static_cast<int32_t>(kPointObstalce.x() / kResolution);
    const int32_t point_obstacle_y = static_cast<int32_t>(kPointObstalce.y() / kResolution);
    static_map_data[kStaticMapSize * point_obstacle_y + point_obstacle_x] = kWallValue;
    const int32_t rect_left = static_cast<int32_t>(kRectObstacleLeftBottom.x() / kResolution);
    const int32_t rect_bottom = static_cast<int32_t>(kRectObstacleLeftBottom.y() / kResolution);
    const int32_t rect_right = static_cast<int32_t>(kRectObstacleRightTop.x() / kResolution);
    const int32_t rect_top = static_cast<int32_t>(kRectObstacleRightTop.y() / kResolution);
    for (int32_t y = rect_bottom; y < rect_top; ++y) {
      for (int32_t x = rect_left; x < rect_right; ++x) {
        static_map_data[kStaticMapSize * y + x] = kWallValue;
      }
    }
    CostMapPtr static_map = std::make_shared<CostMap>(
        CostMap(static_map_origin, kResolution, kStaticMapSize, kStaticMapSize, static_map_data));
    LayeredCostMap::Ptr map = std::make_shared<LayeredCostMap>(LayeredCostMap(kMapParam, static_map));

    // Generation of the dynamic map. In this test, the state of the dynamic map is not considered, so the entire area is set to Free
    std::vector<uint8_t> dynamic_map_data(kDynamicMapSize * kDynamicMapSize, 1);
    dynamic_map_ = std::make_shared<CostMap>(
        CostMap(kDynamicMapOrigin, kResolution, kDynamicMapSize, kDynamicMapSize, dynamic_map_data));

    // Generation of AstarPathPlanner
    planner_ = std::make_shared<AstarPathPlanner>(AstarPathPlanner(map, cost_correctors_));
  }

  AstarPathPlanner::Ptr planner_;
  CostMapPtr dynamic_map_;
  std::vector<ICostCorrector::Ptr> cost_correctors_;
  PoseSeq preferred_path_;
};

/// Test for AsarPathPlanner
/// Normal case: Returns true if lower modules do not encounter errors
TEST_F(AstarPathPlannerTest, NormalCase) {
  // exercise
  // Set start and goal positions in areas without obstacles
  PoseSeq output_path;
  const bool result = planner_->PlanPath(
      kStart, kGoal, dynamic_map_, kDynamicMapOrigin, false, preferred_path_, output_path);
  // verify
  EXPECT_TRUE(result);
}

/// Test for AsarPathPlanner
/// When enable_adaptive_start_positioning = false, path planning fails if the start position is on an obstacle
TEST_F(AstarPathPlannerTest, NotAdjustStartPosition) {
  // exercise
  // Set the start position on a single-point obstacle and the goal at an arbitrary position
  PoseSeq output_path;
  const bool result = planner_->PlanPath(
      kPointObstalce, kGoal, dynamic_map_, kDynamicMapOrigin, false, preferred_path_, output_path);
  // verify
  EXPECT_FALSE(result);
}

/// Test for AsarPathPlanner
/// When enable_adaptive_start_positioning = true, path planning succeeds even if the start position is on an obstacle
TEST_F(AstarPathPlannerTest, AdjustStartPosition) {
  // exercise
  // Set the start position on a single-point obstacle and the goal at an arbitrary position
  PoseSeq output_path;
  const bool result = planner_->PlanPath(
      kPointObstalce, kGoal, dynamic_map_, kDynamicMapOrigin, true, preferred_path_, output_path);
  // verify
  EXPECT_TRUE(result);
}

/// Test for AsarPathPlanner
/// Returns false if start position adjustment fails
TEST_F(AstarPathPlannerTest, StartPositionAdjustingFail) {
  // exercise
  // Set the start position at the center of a rectangular obstacle and the goal at an arbitrary position
  const Pose2d start((kRectObstacleLeftBottom.x() + kRectObstacleRightTop.x()) / 2.0,
                     (kRectObstacleLeftBottom.y() + kRectObstacleRightTop.y()) / 2.0, 0.0);
  PoseSeq output_path;
  const bool result = planner_->PlanPath(
      start, kGoal, dynamic_map_, kDynamicMapOrigin, true, preferred_path_, output_path);
  // verify
  EXPECT_FALSE(result);
}

/// Test for AsarPathPlanner
/// Returns false if path planning with AstarCore fails
TEST_F(AstarPathPlannerTest, AstarCoreFail) {
  // exercise
  // Set the goal at the center of an obstacle and the start at an arbitrary position
  const Pose2d goal((kRectObstacleLeftBottom.x() + kRectObstacleRightTop.x()) / 2.0,
                    (kRectObstacleLeftBottom.y() + kRectObstacleRightTop.y()) / 2.0, 0.0);
  PoseSeq output_path;
  const bool result = planner_->PlanPath(
      kStart, goal, dynamic_map_, kDynamicMapOrigin, false, preferred_path_, output_path);
  // verify
  EXPECT_FALSE(result);
}

/// Test for AsarPathPlanner
/// Returns a two-point path if the start and goal are in the same grid
TEST_F(AstarPathPlannerTest, StartGoalOnSameGrid) {
  // exercise
  // Set start and goal positions at slightly different coordinates within the same grid
  PoseSeq output_path;
  const bool result = planner_->PlanPath(
      kStart, kGoalSameGrid, dynamic_map_, kDynamicMapOrigin, false, preferred_path_, output_path);
  // verify
  // Path planning succeeds with a two-point path, and the start and goal coordinates are stored as is
  ASSERT_TRUE(result);
  ASSERT_EQ(2, output_path.size());
  EXPECT_DOUBLE_EQ(kStart.x(), output_path[0].x());
  EXPECT_DOUBLE_EQ(kStart.y(), output_path[0].y());
  EXPECT_DOUBLE_EQ(kGoalSameGrid.x(), output_path[1].x());
  EXPECT_DOUBLE_EQ(kGoalSameGrid.y(), output_path[1].y());
}

}  // namespace tmc_base_path_planner

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
