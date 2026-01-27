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
// Dynamic map origin
const Pose2d kDynamicMapOrigin = Pose2d(12.5, 12.5, 0.0);
// Position of a single-point obstacle
// Adding resolution/2 to point to the center of the grid to avoid rounding errors when converting map to grid coordinates
const Pose2d kPointObstalce(30.0 + (kResolution / 2.0), 25.0 + (kResolution / 2.0), 0.0);
// Position of rectangular obstacles
const Pose2d kRectObstacleLeftBottom(32.5 + (kResolution / 2.0), 27.5 + (kResolution / 2.0), 0.0);
const Pose2d kRectObstacleRightTop(37.5 + (kResolution / 2.0), 32.5 + (kResolution / 2.0), 0.0);
// Start and goal for normal test cases
const Pose2d kStart(30.0 + (kResolution / 2.0), 30.0 + (kResolution / 2.0), 0.0);
const Pose2d kGoal(30.0 + (kResolution / 2.0), 32.0 + (kResolution / 2.0), 0.0);
// Goal within the same grid as the start
const Pose2d kGoalSameGrid(30.0 + (kResolution / 3.0), 30.0 + (kResolution / 4.0), 0.0);
}  // anonymous namespace

namespace tmc_base_path_planner {

/// Test fixture for AstarPathPlanner
class AstarPathPlannerTest : public ::testing::Test {
 public:
  AstarPathPlannerTest() {}

 protected:
  virtual void SetUp() {
    // Generation of static map
    Pose2d static_map_origin(0.0, 0.0, 0.0);
    std::vector<uint8_t> static_map_data(kStaticMapSize * kStaticMapSize, 1);
    // Place obstacles at one point and in a square area for success/failure determination
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

    // Generation of dynamic map. In this test, the state of the dynamic map is not questioned, so set the entire area to Free
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

/// AsarPathPlanner test
/// Normal case: Return true if the lower module does not result in an error
TEST_F(AstarPathPlannerTest, NormalCase) {
  // exercise
  // Set start and goal at positions without obstacles
  PoseSeq output_path;
  const bool result = planner_->PlanPath(
      kStart, kGoal, dynamic_map_, kDynamicMapOrigin, false, preferred_path_, output_path);
  // verify
  EXPECT_TRUE(result);
}

/// AsarPathPlanner test
/// With enable_adaptive_start_positioning = false, path planning fails if the start position is an obstacle
TEST_F(AstarPathPlannerTest, NotAdjustStartPosition) {
  // exercise
  // Set start at the position of a single-point obstacle and goal at an appropriate position
  PoseSeq output_path;
  const bool result = planner_->PlanPath(
      kPointObstalce, kGoal, dynamic_map_, kDynamicMapOrigin, false, preferred_path_, output_path);
  // verify
  EXPECT_FALSE(result);
}

/// AsarPathPlanner test
/// With enable_adaptive_start_positioning = true, path planning succeeds if the start position is an obstacle
TEST_F(AstarPathPlannerTest, AdjustStartPosition) {
  // exercise
  // Set start at the position of a single-point obstacle and goal at an appropriate position
  PoseSeq output_path;
  const bool result = planner_->PlanPath(
      kPointObstalce, kGoal, dynamic_map_, kDynamicMapOrigin, true, preferred_path_, output_path);
  // verify
  EXPECT_TRUE(result);
}

/// AsarPathPlanner test
/// Return false if start position adjustment fails
TEST_F(AstarPathPlannerTest, StartPositionAdjustingFail) {
  // exercise
  // Set start at the center of a rectangular obstacle and goal at an appropriate position
  const Pose2d start((kRectObstacleLeftBottom.x() + kRectObstacleRightTop.x()) / 2.0,
                     (kRectObstacleLeftBottom.y() + kRectObstacleRightTop.y()) / 2.0, 0.0);
  PoseSeq output_path;
  const bool result = planner_->PlanPath(
      start, kGoal, dynamic_map_, kDynamicMapOrigin, true, preferred_path_, output_path);
  // verify
  EXPECT_FALSE(result);
}

/// AsarPathPlanner test
/// Return false if path planning by AstarCore fails
TEST_F(AstarPathPlannerTest, AstarCoreFail) {
  // exercise
  // Set goal at the center of an obstacle and start at an appropriate position
  const Pose2d goal((kRectObstacleLeftBottom.x() + kRectObstacleRightTop.x()) / 2.0,
                    (kRectObstacleLeftBottom.y() + kRectObstacleRightTop.y()) / 2.0, 0.0);
  PoseSeq output_path;
  const bool result = planner_->PlanPath(
      kStart, goal, dynamic_map_, kDynamicMapOrigin, false, preferred_path_, output_path);
  // verify
  EXPECT_FALSE(result);
}

/// AsarPathPlanner test
/// Return a two-point path if start and goal are in the same grid
TEST_F(AstarPathPlannerTest, StartGoalOnSameGrid) {
  // exercise
  // Set start and goal at slightly different coordinates within the same grid
  PoseSeq output_path;
  const bool result = planner_->PlanPath(
      kStart, kGoalSameGrid, dynamic_map_, kDynamicMapOrigin, false, preferred_path_, output_path);
  // verify
  // Successful planning with a two-point path, storing start and goal coordinates as they are
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
