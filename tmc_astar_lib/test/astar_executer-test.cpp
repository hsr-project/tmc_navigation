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
#include <algorithm>
#include <chrono>   // NOLINT
#include <memory>
#include <vector>

#include <gtest/gtest.h>
#include <tmc_astar_lib/astar_executer.hpp>
#include <tmc_astar_lib/layered_cost_map.hpp>

namespace {
// Edge size of the static map
constexpr int32_t kStaticMapSize = 100;
// Edge size of the dynamic map
constexpr int32_t kDynamicMapSize = 50;
// Edge size of the static map used for performance evaluation
constexpr int32_t kStaticMapSizeForPerformanceTest = 1000;
// Map resolution
constexpr double kResolution = 0.05;
// Map parameters
const tmc_astar_lib::LayeredCostMap::Parameter kMapParam(0.2, 0.5, 1.0, 150.0, 0, 50, 71);
// Origin of the dynamic map
const Pose2d kDynamicMapOrigin = Pose2d(1.25, 1.25, 0.0);
// Grid coordinates of the start point. Use this as a reference unless there is a specific reason.
const tmc_astar_lib::MapIndex kStartIndex = tmc_astar_lib::MapIndex(50, 50);
// Maximum cost for normal test cases. Set to a value that does not exceed this.
constexpr int32_t kMaxCost = 10000;
// Y-coordinate of the wall for the wall bypass test
constexpr int32_t kWallY = 50;
// Width of the wall for the wall bypass test
constexpr int32_t kWallWidth = 40;
// Median for the cost gradient test
constexpr int32_t kSlopeCenterCost = 130;
// Distance to the goal set in the test
constexpr int32_t kGoalDistance = 20;
// Shift amount for the bypass direction test. Tilt to either left or right by this value and check if the tilted side is prioritized.
constexpr int32_t kTestBias = 10;
// Distance between the wall surrounding the goal and the goal point for the unreachable test
constexpr int32_t kFenceDistance = 10;
// Margin from the map edge for the start and goal points in the cost gradient test
constexpr int32_t kMapMargin = 10;
}  // anonymous namespace

namespace tmc_astar_lib {

/// Test fixture for AstarExecuter
class AstarExecuterTest : public ::testing::Test {
 public:
  AstarExecuterTest() {}

 protected:
  virtual void SetUp() {
    astar_executer_ = std::make_shared<AstarExecuter>(AstarExecuter(kStaticMapSize, kStaticMapSize));
  }

  // Generate a map without obstacles
  LayeredCostMap::Ptr CreateBlankMap() {
    // Generate a static map without obstacles
    Pose2d static_map_origin(0.0, 0.0, 0.0);
    std::vector<uint8_t> static_map_data(kStaticMapSize * kStaticMapSize, 1);
    const CostMapPtr static_map = std::make_shared<CostMap>(
      CostMap(static_map_origin, kResolution, kStaticMapSize, kStaticMapSize, static_map_data));
    LayeredCostMap::Ptr map = std::make_shared<LayeredCostMap>(LayeredCostMap(kMapParam, static_map));
    // Generate a dynamic map without obstacles
    std::vector<uint8_t> dynamic_map_data(kDynamicMapSize * kDynamicMapSize, 0);
    CostMapPtr dynamic_map = std::make_shared<CostMap>(
      CostMap(kDynamicMapOrigin, kResolution, kDynamicMapSize, kDynamicMapSize, dynamic_map_data));
    map->SetDynamicMap(dynamic_map, kDynamicMapOrigin);
    return map;
  }

  // Generate a map with a static wall in the center
  LayeredCostMap::Ptr CreateStaticLineObstacleMap() {
    // Set a horizontal wall in the center of the static map
    Pose2d static_map_origin(0.0, 0.0, 0.0);
    std::vector<uint8_t> static_map_data(kStaticMapSize * kStaticMapSize, 1);
    for (int32_t x = (kStaticMapSize / 2) - (kWallWidth / 2); x < (kStaticMapSize / 2) + (kWallWidth / 2); ++x) {
      static_map_data[kStaticMapSize * kWallY + x] = kWallValue;
    }
    const CostMapPtr static_map = std::make_shared<CostMap>(
      CostMap(static_map_origin, kResolution, kStaticMapSize, kStaticMapSize, static_map_data));
    LayeredCostMap::Ptr map = std::make_shared<LayeredCostMap>(LayeredCostMap(kMapParam, static_map));
    // Generate a dynamic map without obstacles
    std::vector<uint8_t> dynamic_map_data(kDynamicMapSize * kDynamicMapSize, 0);
    CostMapPtr dynamic_map = std::make_shared<CostMap>(
      CostMap(kDynamicMapOrigin, kResolution, kDynamicMapSize, kDynamicMapSize, dynamic_map_data));
    map->SetDynamicMap(dynamic_map, kDynamicMapOrigin);
    return map;
  }

  // Generate a map with a dynamic wall in the center
  LayeredCostMap::Ptr CreateDynamicLineObstacleMap() {
    // Generate a static map without obstacles
    Pose2d static_map_origin(0.0, 0.0, 0.0);
    std::vector<uint8_t> static_map_data(kStaticMapSize * kStaticMapSize, 1);
    const CostMapPtr static_map = std::make_shared<CostMap>(
      CostMap(static_map_origin, kResolution, kStaticMapSize, kStaticMapSize, static_map_data));
    LayeredCostMap::Ptr map = std::make_shared<LayeredCostMap>(LayeredCostMap(kMapParam, static_map));
    // Set a dynamic map wall horizontally in the center of the static map
    std::vector<uint8_t> dynamic_map_data(kDynamicMapSize * kDynamicMapSize, 0);
    for (int32_t x = (kDynamicMapSize / 2) - (kWallWidth / 2); x < (kDynamicMapSize / 2) + (kWallWidth / 2); ++x) {
      dynamic_map_data[kDynamicMapSize * (kWallY - ((kStaticMapSize - kDynamicMapSize) / 2)) + x] = kWallValue;
    }
    CostMapPtr dynamic_map = std::make_shared<CostMap>(
      CostMap(kDynamicMapOrigin, kResolution, kDynamicMapSize, kDynamicMapSize, dynamic_map_data));
    map->SetDynamicMap(dynamic_map, kDynamicMapOrigin);
    return map;
  }

  // Generate a map with a gradient from Free to Wall along the X-axis in the static map
  LayeredCostMap::Ptr CreateStaticGradationObstacleMap() {
    // Place a gradient from Free to Wall along the X-axis in the center of the static map
    Pose2d static_map_origin(0.0, 0.0, 0.0);
    std::vector<uint8_t> static_map_data(kStaticMapSize * kStaticMapSize, 1);
    for (int32_t y = 0; y < kStaticMapSize; ++y) {
      int32_t cost = kSlopeCenterCost - (kStaticMapSize / 2);
      for (int32_t x = 0; x < kStaticMapSize; ++x) {
        static_map_data[kStaticMapSize * y + x] = cost;
        cost++;
      }
    }
    const CostMapPtr static_map = std::make_shared<CostMap>(
        CostMap(static_map_origin, kResolution, kStaticMapSize, kStaticMapSize, static_map_data));
    LayeredCostMap::Ptr map = std::make_shared<LayeredCostMap>(LayeredCostMap(kMapParam, static_map));
    // Generate a dynamic map without obstacles
    std::vector<uint8_t> dynamic_map_data(kDynamicMapSize * kDynamicMapSize, 0);
    CostMapPtr dynamic_map = std::make_shared<CostMap>(
        CostMap(kDynamicMapOrigin, kResolution, kDynamicMapSize, kDynamicMapSize, dynamic_map_data));
    map->SetDynamicMap(dynamic_map, kDynamicMapOrigin);
    return map;
  }

  // Generate a map with a gradient from Free to Wall along the X-axis in the dynamic map
  LayeredCostMap::Ptr CreateDynamicGradationObstacleMap() {
    // Generate a static map without obstacles
    Pose2d static_map_origin(0.0, 0.0, 0.0);
    std::vector<uint8_t> static_map_data(kStaticMapSize * kStaticMapSize, 1);
    const CostMapPtr static_map = std::make_shared<CostMap>(
        CostMap(static_map_origin, kResolution, kStaticMapSize, kStaticMapSize, static_map_data));
    LayeredCostMap::Ptr map = std::make_shared<LayeredCostMap>(LayeredCostMap(kMapParam, static_map));
    // Place a gradient from Free to Wall along the X-axis in the center of the dynamic map
    std::vector<uint8_t> dynamic_map_data(kDynamicMapSize * kDynamicMapSize, 0);
    for (int32_t y = 0; y < kDynamicMapSize; ++y) {
      int32_t cost = kSlopeCenterCost - (kDynamicMapSize / 2);
      for (int32_t x = 0; x < kDynamicMapSize; ++x) {
        dynamic_map_data[kDynamicMapSize * y + x] = cost;
        cost += 2;
      }
    }
    CostMapPtr dynamic_map = std::make_shared<CostMap>(
        CostMap(kDynamicMapOrigin, kResolution, kDynamicMapSize, kDynamicMapSize, dynamic_map_data));
    map->SetDynamicMap(dynamic_map, kDynamicMapOrigin);
    return map;
  }

  // Generate a map with walls surrounding the specified point
  LayeredCostMap::Ptr CreateFenceObstacleMap(const MapIndex& center) {
    // Place walls in a rectangle to surround the center point
    Pose2d static_map_origin(0.0, 0.0, 0.0);
    std::vector<uint8_t> static_map_data(kStaticMapSize * kStaticMapSize, 1);
    for (int32_t x = center.x - kFenceDistance; x <= center.x + kFenceDistance; ++x) {
      static_map_data[kStaticMapSize * (center.y - kFenceDistance) + x] = kWallValue;
      static_map_data[kStaticMapSize * (center.y + kFenceDistance) + x] = kWallValue;
    }
    for (int32_t y = center.y - kFenceDistance; y <= center.y + kFenceDistance; ++y) {
      static_map_data[kStaticMapSize * y + center.x - kFenceDistance] = kWallValue;
      static_map_data[kStaticMapSize * y + center.x + kFenceDistance] = kWallValue;
    }
    const CostMapPtr static_map = std::make_shared<CostMap>(
        CostMap(static_map_origin, kResolution, kStaticMapSize, kStaticMapSize, static_map_data));
    LayeredCostMap::Ptr map = std::make_shared<LayeredCostMap>(LayeredCostMap(kMapParam, static_map));
    // Generate a dynamic map without obstacles
    std::vector<uint8_t> dynamic_map_data(kDynamicMapSize * kDynamicMapSize, 0);
    CostMapPtr dynamic_map = std::make_shared<CostMap>(
        CostMap(kDynamicMapOrigin, kResolution, kDynamicMapSize, kDynamicMapSize, dynamic_map_data));
    map->SetDynamicMap(dynamic_map, kDynamicMapOrigin);
    return map;
  }

  // Generate a large map for performance evaluation
  LayeredCostMap::Ptr CreatePerformanceTestMap() {
    // Generate a static map without obstacles
    Pose2d static_map_origin(0.0, 0.0, 0.0);
    std::vector<uint8_t> static_map_data(kStaticMapSizeForPerformanceTest * kStaticMapSizeForPerformanceTest, 1);
    const CostMapPtr static_map = std::make_shared<CostMap>(
        CostMap(static_map_origin, kResolution, kStaticMapSizeForPerformanceTest, kStaticMapSizeForPerformanceTest,
                static_map_data));
    LayeredCostMap::Ptr map = std::make_shared<LayeredCostMap>(LayeredCostMap(kMapParam, static_map));
    // Generate a dynamic map without obstacles
    std::vector<uint8_t> dynamic_map_data(kDynamicMapSize * kDynamicMapSize, 0);
    CostMapPtr dynamic_map = std::make_shared<CostMap>(
        CostMap(kDynamicMapOrigin, kResolution, kDynamicMapSize, kDynamicMapSize, dynamic_map_data));
    map->SetDynamicMap(dynamic_map, kDynamicMapOrigin);
    return map;
  }

  AstarExecuter::Ptr astar_executer_;
  std::vector<ICostCorrector::Ptr> cost_correcotrs_;
};

/// CostCorrector for evaluation
/// Output correction values with - on the left and + on the right, centered on the map
class TestCostCorrector : public ICostCorrector {
 public:
  ~TestCostCorrector() {}
  void Setup(const ICostCorrector::SetupParams& params) {}
  int32_t GetAdditionalCost(const ICostCorrector::GetAdditionalCostParams& params) {
    int32_t additional_cost = params.index.x - (kStaticMapSize / 2);
    return additional_cost;
  }
};

/// AstarExecuter test
/// If there are no obstacles, a straight path to the goal is generated (0° direction)
TEST_F(AstarExecuterTest, Straight0) {
  // setup
  // Generate a static map without obstacles
  LayeredCostMap::Ptr map = CreateBlankMap();
  // Set the goal in the 0° direction
  MapIndex goal_index = kStartIndex;
  goal_index.x += kGoalDistance;

  // exercise
  PoseSeq path;
  const bool result = astar_executer_->ExecuteAstar(map, kStartIndex, goal_index, kMaxCost, path);

  // verify
  ASSERT_TRUE(result);
  // The number of points in the path is as expected (distance +1 because it includes the start point)
  ASSERT_EQ(kGoalDistance + 1, path.size());
  // A straight path is drawn from start to goal
  for (int32_t i = 0; i < kGoalDistance + 1; ++i) {
    MapIndex index;
    map->PoseToIndex(path[i], index);
    EXPECT_EQ(index.x, kStartIndex.x + i);
    EXPECT_EQ(index.y, kStartIndex.y);
  }
}

/// AstarExecuter test
/// If there are no obstacles, a straight path to the goal is generated (45° direction)
TEST_F(AstarExecuterTest, Straight45) {
  // setup
  // Generate a static map without obstacles
  LayeredCostMap::Ptr map = CreateBlankMap();
  // Set the goal in the 45° direction
  MapIndex goal_index = kStartIndex;
  goal_index.x += kGoalDistance;
  goal_index.y += kGoalDistance;

  // exercise
  PoseSeq path;
  const bool result = astar_executer_->ExecuteAstar(map, kStartIndex, goal_index, kMaxCost, path);

  // verify
  ASSERT_TRUE(result);
  // The number of points in the path is as expected (distance +1 because it includes the start point)
  ASSERT_EQ(kGoalDistance + 1, path.size());
  // A straight path is drawn from start to goal
  for (int32_t i = 0; i < kGoalDistance + 1; ++i) {
    MapIndex index;
    map->PoseToIndex(path[i], index);
    EXPECT_EQ(index.x, kStartIndex.x + i);
    EXPECT_EQ(index.y, kStartIndex.y + i);
  }
}

/// AstarExecuter test
/// If there are no obstacles, a straight path to the goal is generated (90° direction)
TEST_F(AstarExecuterTest, Straight90) {
  // setup
  // Generate a static map without obstacles
  LayeredCostMap::Ptr map = CreateBlankMap();
  // Set the goal in the 90° direction
  MapIndex goal_index = kStartIndex;
  goal_index.y += kGoalDistance;

  // exercise
  PoseSeq path;
  const bool result = astar_executer_->ExecuteAstar(map, kStartIndex, goal_index, kMaxCost, path);

  // verify
  ASSERT_TRUE(result);
  // The number of points in the path is as expected (distance +1 because it includes the start point)
  ASSERT_EQ(kGoalDistance + 1, path.size());
  // A straight path is drawn from start to goal
  for (int32_t i = 0; i < kGoalDistance + 1; ++i) {
    MapIndex index;
    map->PoseToIndex(path[i], index);
    EXPECT_EQ(index.x, kStartIndex.x);
    EXPECT_EQ(index.y, kStartIndex.y + i);
  }
}

/// AstarExecuter test
/// If there are no obstacles, a straight path to the goal is generated (135° direction)
TEST_F(AstarExecuterTest, Straight135) {
  // setup
  // Generate a static map without obstacles
  LayeredCostMap::Ptr map = CreateBlankMap();
  // Set the goal in the 135° direction
  MapIndex goal_index = kStartIndex;
  goal_index.x -= kGoalDistance;
  goal_index.y += kGoalDistance;

  // exercise
  PoseSeq path;
  const bool result = astar_executer_->ExecuteAstar(map, kStartIndex, goal_index, kMaxCost, path);

  // verify
  ASSERT_TRUE(result);
  // The number of points in the path is as expected (distance +1 because it includes the start point)
  ASSERT_EQ(kGoalDistance + 1, path.size());
  // A straight path is drawn from start to goal
  for (int32_t i = 0; i < kGoalDistance + 1; ++i) {
    MapIndex index;
    map->PoseToIndex(path[i], index);
    EXPECT_EQ(index.x, kStartIndex.x - i);
    EXPECT_EQ(index.y, kStartIndex.y + i);
  }
}

/// AstarExecuter test
/// If there are no obstacles, a straight path to the goal is generated (180° direction)
TEST_F(AstarExecuterTest, Straight180) {
  // setup
  // Generate a static map without obstacles
  LayeredCostMap::Ptr map = CreateBlankMap();
  // Set the goal in the 180° direction
  MapIndex goal_index = kStartIndex;
  goal_index.x -= kGoalDistance;

  // exercise
  PoseSeq path;
  const bool result = astar_executer_->ExecuteAstar(map, kStartIndex, goal_index, kMaxCost, path);

  // verify
  ASSERT_TRUE(result);
  // The number of points in the path is as expected (distance +1 because it includes the start point)
  ASSERT_EQ(kGoalDistance + 1, path.size());
  // A straight path is drawn from start to goal
  for (int32_t i = 0; i < kGoalDistance + 1; ++i) {
    MapIndex index;
    map->PoseToIndex(path[i], index);
    EXPECT_EQ(index.x, kStartIndex.x - i);
    EXPECT_EQ(index.y, kStartIndex.y);
  }
}

/// AstarExecuter test
/// If there are no obstacles, a straight path to the goal is generated (225° direction)
TEST_F(AstarExecuterTest, Straight225) {
  // setup
  // Generate a static map without obstacles
  LayeredCostMap::Ptr map = CreateBlankMap();
  // Set the goal in the 180° direction
  MapIndex goal_index = kStartIndex;
  goal_index.x -= kGoalDistance;
  goal_index.y -= kGoalDistance;

  // exercise
  PoseSeq path;
  const bool result = astar_executer_->ExecuteAstar(map, kStartIndex, goal_index, kMaxCost, path);

  // verify
  ASSERT_TRUE(result);
  // The number of points in the path is as expected (distance +1 because it includes the start point)
  ASSERT_EQ(kGoalDistance + 1, path.size());
  // A straight path is drawn from start to goal
  for (int32_t i = 0; i < kGoalDistance + 1; ++i) {
    MapIndex index;
    map->PoseToIndex(path[i], index);
    EXPECT_EQ(index.x, kStartIndex.x - i);
    EXPECT_EQ(index.y, kStartIndex.y - i);
  }
}

/// AstarExecuter test
/// If there are no obstacles, a straight path to the goal is generated (270° direction)
TEST_F(AstarExecuterTest, Straight270) {
  // setup
  // Generate a static map without obstacles
  LayeredCostMap::Ptr map = CreateBlankMap();
  // Set the goal in the 270° direction
  MapIndex goal_index = kStartIndex;
  goal_index.y -= kGoalDistance;

  // exercise
  PoseSeq path;
  const bool result = astar_executer_->ExecuteAstar(map, kStartIndex, goal_index, kMaxCost, path);

  // verify
  ASSERT_TRUE(result);
  // The number of points in the path is as expected (distance +1 because it includes the start point)
  ASSERT_EQ(kGoalDistance + 1, path.size());
  // A straight path is drawn from start to goal
  for (int32_t i = 0; i < kGoalDistance + 1; ++i) {
    MapIndex index;
    map->PoseToIndex(path[i], index);
    EXPECT_EQ(index.x, kStartIndex.x);
    EXPECT_EQ(index.y, kStartIndex.y - i);
  }
}

/// AstarExecuter test
/// If there are no obstacles, a straight path to the goal is generated (315° direction)
TEST_F(AstarExecuterTest, Straight315) {
  // setup
  // Generate a static map without obstacles
  LayeredCostMap::Ptr map = CreateBlankMap();
  // Set the goal in the 315° direction
  MapIndex goal_index = kStartIndex;
  goal_index.x += kGoalDistance;
  goal_index.y -= kGoalDistance;

  // exercise
  PoseSeq path;
  const bool result = astar_executer_->ExecuteAstar(map, kStartIndex, goal_index, kMaxCost, path);

  // verify
  ASSERT_TRUE(result);
  // The number of points in the path is as expected (distance +1 because it includes the start point)
  ASSERT_EQ(kGoalDistance + 1, path.size());
  // A straight path is drawn from start to goal
  for (int32_t i = 0; i < kGoalDistance + 1; ++i) {
    MapIndex index;
    map->PoseToIndex(path[i], index);
    EXPECT_EQ(index.x, kStartIndex.x + i);
    EXPECT_EQ(index.y, kStartIndex.y - i);
  }
}

/// AstarExecuter test
/// When there is a static obstacle, the detour should go through the shorter side (right side)
TEST_F(AstarExecuterTest, StaticObstacleBypassRight) {
  // setup
  // Set a horizontal wall in the center of the static map
  const LayeredCostMap::Ptr map = CreateStaticLineObstacleMap();

  // Set the start and goal slightly to the right of the wall
  const MapIndex start_index((kStaticMapSize / 2) + kTestBias, kWallY - kGoalDistance);
  const MapIndex goal_index(start_index.x, kWallY + kGoalDistance);

  // exercise
  PoseSeq path;
  const bool result = astar_executer_->ExecuteAstar(map, start_index, goal_index, kMaxCost, path);

  // verify
  ASSERT_TRUE(result);
  // Check the maximum and minimum X-coordinates in the path
  int32_t min_x = start_index.x;
  int32_t max_x = start_index.x;
  for (const Pose2d path_point : path) {
    MapIndex index;
    map->PoseToIndex(path_point, index);
    min_x = std::min(min_x, index.x);
    max_x = std::max(max_x, index.x);
  }
  // Confirm that the right-side course was followed
  EXPECT_EQ(start_index.x, min_x);
  EXPECT_GT(max_x, start_index.x);
}

/// AstarExecuter test
/// When there is a static obstacle, the detour should go through the shorter side (left side)
TEST_F(AstarExecuterTest, StaticObstacleBypassLeft) {
  // setup
  // Set a horizontal wall in the center of the static map
  const LayeredCostMap::Ptr map = CreateStaticLineObstacleMap();

  // Set the start and goal slightly to the left of the wall
  const MapIndex start_index((kStaticMapSize / 2) - kTestBias, kWallY - kGoalDistance);
  const MapIndex goal_index(start_index.x, kWallY + kGoalDistance);

  // exercise
  PoseSeq path;
  const bool result = astar_executer_->ExecuteAstar(map, start_index, goal_index, kMaxCost, path);

  // verify
  ASSERT_TRUE(result);
  // Check the maximum and minimum X-coordinates in the path
  int32_t min_x = start_index.x;
  int32_t max_x = start_index.x;
  for (const Pose2d path_point : path) {
    MapIndex index;
    map->PoseToIndex(path_point, index);
    min_x = std::min(min_x, index.x);
    max_x = std::max(max_x, index.x);
  }
  // Confirm that the left-side course was followed
  EXPECT_EQ(start_index.x, max_x);
  EXPECT_LT(min_x, start_index.x);
}

/// AstarExecuter test
/// When there is a dynamic obstacle, the detour should go through the shorter side (right side)
TEST_F(AstarExecuterTest, DynamicObstacleBypassRight) {
  // setup
  // Set a dynamic map wall horizontally in the center of the static map
  const LayeredCostMap::Ptr map = CreateDynamicLineObstacleMap();
  // Set the start and goal slightly to the right of the wall
  const MapIndex start_index((kStaticMapSize / 2) + kTestBias, kWallY - kGoalDistance);
  const MapIndex goal_index(start_index.x, kWallY + kGoalDistance);

  // exercise
  PoseSeq path;
  const bool result = astar_executer_->ExecuteAstar(map, start_index, goal_index, kMaxCost, path);

  // verify
  ASSERT_TRUE(result);
  // Check the maximum and minimum X-coordinates in the path
  int32_t min_x = start_index.x;
  int32_t max_x = start_index.x;
  for (const Pose2d path_point : path) {
    MapIndex index;
    map->PoseToIndex(path_point, index);
    min_x = std::min(min_x, index.x);
    max_x = std::max(max_x, index.x);
  }
  // Confirm that the right-side course was followed
  EXPECT_EQ(start_index.x, min_x);
  EXPECT_GT(max_x, start_index.x);
}

/// AstarExecuter test
/// When there is a dynamic obstacle, the detour should go through the shorter side (left side)
TEST_F(AstarExecuterTest, DynamicObstacleBypassLeft) {
  // setup
  // Set a dynamic map wall horizontally in the center of the static map
  const LayeredCostMap::Ptr map = CreateDynamicLineObstacleMap();
  // Set the start and goal slightly to the left of the wall
  const MapIndex start_index((kStaticMapSize / 2) - kTestBias, kWallY - kGoalDistance);
  const MapIndex goal_index(start_index.x, kWallY + kGoalDistance);

  // exercise
  PoseSeq path;
  const bool result = astar_executer_->ExecuteAstar(map, start_index, goal_index, kMaxCost, path);

  // verify
  ASSERT_TRUE(result);
  // Check the maximum and minimum X-coordinates in the path
  int32_t min_x = start_index.x;
  int32_t max_x = start_index.x;
  for (const Pose2d path_point : path) {
    MapIndex index;
    map->PoseToIndex(path_point, index);
    min_x = std::min(min_x, index.x);
    max_x = std::max(max_x, index.x);
  }
  // Confirm that the left-side course was followed
  EXPECT_EQ(start_index.x, max_x);
  EXPECT_LT(min_x, start_index.x);
}

/// AstarExecuter test
/// When passing through an area with a cost gradient of static obstacles, the path should go through the lower cost side
TEST_F(AstarExecuterTest, StaticObstacleSlope) {
  // Generate a map with a gradient from Free to Wall along the X-axis in the center of the static map
  const LayeredCostMap::Ptr map = CreateStaticGradationObstacleMap();

  // Set the start and goal to cross the map from edge to edge
  // If the travel distance is short, cutting through is cheaper, so take a longer path
  const MapIndex start_index(kStaticMapSize / 2, 0);
  const MapIndex goal_index(start_index.x, kStaticMapSize - 1);

  // exercise
  PoseSeq path;
  const bool result = astar_executer_->ExecuteAstar(map, start_index, goal_index, kMaxCost, path);

  // verify
  ASSERT_TRUE(result);

  // Check the maximum and minimum X-coordinates in the path
  int32_t min_x = start_index.x;
  int32_t max_x = start_index.x;
  for (const Pose2d path_point : path) {
    MapIndex index;
    map->PoseToIndex(path_point, index);
    min_x = std::min(min_x, index.x);
    max_x = std::max(max_x, index.x);
  }
  // Confirm that the left-side course was followed
  EXPECT_EQ(start_index.x, max_x);
  EXPECT_LT(min_x, start_index.x);
}

/// AstarExecuter test
/// When passing through an area with a cost gradient of dynamic obstacles, the path should go through the lower cost side
TEST_F(AstarExecuterTest, DynamicObstacleSlope) {
  const LayeredCostMap::Ptr map = CreateDynamicGradationObstacleMap();

  // Set the start and goal to cross the map from edge to edge
  // Set slightly inside from the edge of the dynamic map to avoid bypassing the dynamic map itself
  const MapIndex start_index(kStaticMapSize / 2, (kStaticMapSize - kDynamicMapSize) / 2 + kMapMargin);
  const MapIndex goal_index(start_index.x, (kStaticMapSize - kDynamicMapSize) / 2 + kDynamicMapSize - kMapMargin);

  // exercise
  PoseSeq path;
  const bool result = astar_executer_->ExecuteAstar(map, start_index, goal_index, kMaxCost, path);

  // verify
  ASSERT_TRUE(result);

  // Check the maximum and minimum X-coordinates in the path
  int32_t min_x = start_index.x;
  int32_t max_x = start_index.x;
  for (const Pose2d path_point : path) {
    MapIndex index;
    map->PoseToIndex(path_point, index);
    min_x = std::min(min_x, index.x);
    max_x = std::max(max_x, index.x);
  }
  // Confirm that the left-side course was followed
  EXPECT_EQ(start_index.x, max_x);
  EXPECT_LT(min_x, start_index.x);
}

/// AstarExecuter test
/// CostCorrector is called and the correction value is reflected in the result
/// In this test, use a dummy CostCorrector to confirm that the correction process is effective
/// The correctness of the actual CostCorrector used is ensured by the automatic tests of each class
TEST_F(AstarExecuterTest, CostCorrector) {
  // setup
  // Generate a static map without obstacles
  LayeredCostMap::Ptr map = CreateBlankMap();
  // Register a test-specific CostCorrector that applies a negative correction to the left side of the area
  cost_correcotrs_.push_back(std::make_shared<TestCostCorrector>(TestCostCorrector()));
  map->SetCostCollectors(cost_correcotrs_);

  // Set the start and goal to cross the map from edge to edge
  // If the travel distance is short, cutting through is cheaper, so take a longer path
  const MapIndex start_index(kStaticMapSize / 2, 0);
  const MapIndex goal_index(start_index.x, kStaticMapSize - 1);

  // exercise
  PoseSeq path;
  const bool result = astar_executer_->ExecuteAstar(map, start_index, goal_index, kMaxCost, path);

  // verify
  ASSERT_TRUE(result);

  // Check the maximum and minimum X-coordinates in the path
  int32_t min_x = start_index.x;
  int32_t max_x = start_index.x;
  for (const Pose2d path_point : path) {
    MapIndex index;
    map->PoseToIndex(path_point, index);
    min_x = std::min(min_x, index.x);
    max_x = std::max(max_x, index.x);
  }
  // Confirm that the left-side course was followed
  EXPECT_EQ(start_index.x, max_x);
  EXPECT_LT(min_x, start_index.x);
}

/// AstarExecuter test
/// If the goal is not reached within the specified maximum cost, the path planning fails
TEST_F(AstarExecuterTest, FailCaseTooLargeCost) {
  // setup
  // Generate a static map without obstacles
  LayeredCostMap::Ptr map = CreateBlankMap();
  MapIndex goal_index = kStartIndex;
  goal_index.x += kGoalDistance;

  // exercise
  // Set a value smaller than the theoretical cost to reach
  PoseSeq path;
  const bool result = astar_executer_->ExecuteAstar(map, kStartIndex, goal_index,
      kMapParam.single_cost * kGoalDistance - 1, path);

  // verify
  EXPECT_FALSE(result);
}

/// AstarExecuter test
/// If the goal is set in a position unreachable due to obstacles, the path planning fails
TEST_F(AstarExecuterTest, FailCaseUnreachable) {
  // setup
  const MapIndex goal_index(kStartIndex.x, kStartIndex.y + kGoalDistance);
  // Place walls in a rectangle to surround the goal point
  LayeredCostMap::Ptr map = CreateFenceObstacleMap(goal_index);

  // exercise
  PoseSeq path;
  const bool result = astar_executer_->ExecuteAstar(map, kStartIndex, goal_index, kMaxCost, path);

  // verify
  EXPECT_FALSE(result);
}


/// AstarExecuter test
/// Performance evaluation
/// The results vary each time depending on the performance of the execution environment, especially with large variations in Jenkins
/// The test is usually disabled because it cannot correctly determine pass/fail
/// When making changes to this package, compare the print log results before and after the change
/// Verify that there is no performance degradation
#if 0
TEST_F(AstarExecuterTest, PerformanceTest) {
  // setup
  // Create a large map for performance evaluation and plan from edge to edge
  LayeredCostMap::Ptr map = CreatePerformanceTestMap();
  astar_executer_ = std::make_shared<AstarExecuter>(AstarExecuter(map->width(), map->height()));
  const MapIndex start_index(0, 0);
  const MapIndex goal_index(map->width() - 1, map->height() - 1);
  Pose2d start;
  map->IndexToPose(start_index, start);
  Pose2d goal;
  map->IndexToPose(goal_index, goal);
  const int32_t max_cost = map->EstimateMaxCost(start, goal);
  const int32_t test_num = 10;

  // exercise
  std::chrono::system_clock::time_point start_time;
  std::chrono::system_clock::time_point end_time;
  double sum_elapse_time = 0.0;
  for (int32_t i = 0; i < test_num; ++i) {
    start_time = std::chrono::system_clock::now();
    PoseSeq path;
    const bool result = astar_executer_->ExecuteAstar(map, start_index, goal_index, max_cost, path);
    ASSERT_TRUE(result);
    end_time = std::chrono::system_clock::now();
    sum_elapse_time += std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
  }
  const double elapsed_time = sum_elapse_time / static_cast<double>(test_num);
  printf("AstarExecuterTest::PerformanceTest elapsed_time : %f [msec] \n", elapsed_time);
  // verify
  // At the time of test creation, it was around 65ms on the development PC used
  EXPECT_LT(elapsed_time, 75.0);
}
#endif
}  // namespace tmc_astar_lib

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
