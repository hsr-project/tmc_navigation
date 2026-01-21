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
#include <memory>
#include <vector>

#include <gtest/gtest.h>
#include <tmc_astar_lib/layered_cost_map.hpp>

namespace {
// To make it easier to set and reference test data, keep the map size a multiple of 256
// Edge size of static map; multiplier against 256
constexpr int32_t kStaticMapSize = 512;
constexpr int32_t kStaticMapScale = 2;
// Edge size of dynamic map; multiplier against 256
constexpr int32_t kDynamicMapSize = 256;
constexpr int32_t kDynamicMapScale = 1;
// Map resolution
constexpr double kResolution = 0.05;
// Map parameters
const tmc_astar_lib::LayeredCostMap::Parameter kMapParam(0.2, 0.5, 1.0, 150.0, 0, 50, 71);
// Wall threshold in the above parameters
constexpr int32_t kWallThreshold = 204;
// Origin of dynamic map
const Pose2d kDynamicMapOrigin = Pose2d(5.0, 5.0, 0.0);
constexpr double kEpsilon = 1.0e-4;
// Maximum cost
constexpr int32_t kMaxCost = 1000;
}  // anonymous namespace

namespace tmc_astar_lib {

/// Parameter test
/// Ability to generate parameters
TEST(LayeredCostMapParameterTest, ConstructParameter) {
  // exercise
  const LayeredCostMap::Parameter param = LayeredCostMap::Parameter(0.1, 0.2, 0.5, 150.0, 0, 10, 20);

  // verify
  EXPECT_DOUBLE_EQ(0.1, param.exclusive_size);
  EXPECT_DOUBLE_EQ(0.2, param.potential_size);
  EXPECT_DOUBLE_EQ(0.5, param.wall_threshold);
  EXPECT_DOUBLE_EQ(150.0, param.cost_factor);
  EXPECT_EQ(0, param.cost_unknown);
  EXPECT_EQ(10, param.single_cost);
  EXPECT_EQ(20, param.diagonal_cost);
}

/// Parameter test
/// If an invalid value is specified, it is generated with the default value
TEST(LayeredCostMapParameterTest, ConstructWithInvalidValueMakeDefault) {
  // exercise
  const LayeredCostMap::Parameter invalid_param = LayeredCostMap::Parameter(-0.1, -0.1, 0.0, -0.1, 0, 0, 0);

  // verify
  EXPECT_EQ(kExclusiveSizeDefault, invalid_param.exclusive_size);
  EXPECT_EQ(kPotentialSizeDefault, invalid_param.potential_size);
  EXPECT_EQ(kWallThresholdDefault, invalid_param.wall_threshold);
  EXPECT_EQ(kCostFactorDefault, invalid_param.cost_factor);
  // There is no invalid value definition for cost_unknown
  EXPECT_EQ(0, invalid_param.cost_unknown);
  EXPECT_EQ(kSingleCostDefault, invalid_param.single_cost);
  EXPECT_EQ(kDiagonalCostDefault, invalid_param.diagonal_cost);
}

/// Parameter test
/// If there is an inconsistency in values between parameters, it is generated with the default value
TEST(LayeredCostMapParameterTest, ConstructWithInvalidValueBetweenParametersMakeDefault) {
  // Inconsistency if exclusive_size is greater than wall_threshold
  // exercise
  LayeredCostMap::Parameter invalid_param = LayeredCostMap::Parameter(0.31, 0.0, 0.3, 150.0, 0, 10, 14);

  // verify
  EXPECT_EQ(kExclusiveSizeDefault, invalid_param.exclusive_size);
  EXPECT_EQ(kPotentialSizeDefault, invalid_param.potential_size);
  EXPECT_EQ(kWallThresholdDefault, invalid_param.wall_threshold);
  EXPECT_EQ(150.0, invalid_param.cost_factor);
  EXPECT_EQ(0, invalid_param.cost_unknown);
  EXPECT_EQ(10, invalid_param.single_cost);
  EXPECT_EQ(14, invalid_param.diagonal_cost);

  // Inconsistency if (exclusive_size + potential_size) is greater than wall_threshold
  // exercise
  invalid_param = LayeredCostMap::Parameter(0.11, 0.2, 0.3, 150.0, 0, 10, 14);

  // verify
  EXPECT_EQ(kExclusiveSizeDefault, invalid_param.exclusive_size);
  EXPECT_EQ(kPotentialSizeDefault, invalid_param.potential_size);
  EXPECT_EQ(kWallThresholdDefault, invalid_param.wall_threshold);
  EXPECT_EQ(150.0, invalid_param.cost_factor);
  EXPECT_EQ(0, invalid_param.cost_unknown);
  EXPECT_EQ(10, invalid_param.single_cost);
  EXPECT_EQ(14, invalid_param.diagonal_cost);
}

/// LayeredCostMap test fixture
class LayeredCostMapTest : public ::testing::Test {
 public:
  LayeredCostMapTest() {}

 protected:
  virtual void SetUp() {
    // Generate a static map with a gradient of 0..255 in the X direction
    Pose2d static_map_origin(0.0, 0.0, 0.0);
    std::vector<uint8_t> static_map_data(kStaticMapSize * kStaticMapSize);
    for (int32_t y = 0; y < kStaticMapSize; ++y) {
      for (int32_t x = 0; x < kStaticMapSize; ++x) {
        static_map_data[kStaticMapSize * y + x] = x / kStaticMapScale;
      }
    }
    static_map_ = std::make_shared<CostMap>(
        CostMap(static_map_origin, kResolution, kStaticMapSize, kStaticMapSize, static_map_data));
    map_ = std::make_shared<LayeredCostMap>(LayeredCostMap(kMapParam, static_map_));
    // Generate a dynamic map with a gradient of 0..255 in the Y direction
    std::vector<uint8_t> dynamic_map_data(kDynamicMapSize * kDynamicMapSize);
    for (int32_t y = 0; y < kDynamicMapSize; ++y) {
      for (int32_t x = 0; x < kDynamicMapSize; ++x) {
        dynamic_map_data[kDynamicMapSize * y + x] = y / kDynamicMapScale;
      }
    }
    CostMapPtr dynamic_map = std::make_shared<CostMap>(
        CostMap(kDynamicMapOrigin, kResolution, kDynamicMapSize, kDynamicMapSize, dynamic_map_data));
    map_->SetDynamicMap(dynamic_map, kDynamicMapOrigin);
    dynamic_map_offset_x_ = static_cast<int32_t>(kDynamicMapOrigin.x() / kResolution);
    dynamic_map_offset_y_ = static_cast<int32_t>(kDynamicMapOrigin.y() / kResolution);
  }

  LayeredCostMap::Ptr map_;
  // Map
  CostMapPtr static_map_;
  int32_t dynamic_map_offset_x_;
  int32_t dynamic_map_offset_y_;
};

/// Getter test
/// Ability to obtain basic map information
TEST_F(LayeredCostMapTest, MapAttributes) {
  // verify
  EXPECT_EQ(kStaticMapSize, map_->width());
  EXPECT_EQ(kStaticMapSize, map_->height());
  EXPECT_EQ(kWallThreshold, map_->static_map_occupancy_threshold());
}

/// IsOnMap test
/// Ability to determine inside/outside the range of the static map
TEST_F(LayeredCostMapTest, MapBoundaryCheck) {
  // exercise
  // Check boundary values for upper and lower limits of X and Y
  const bool x_upper_in = map_->IsOnMap(MapIndex(kStaticMapSize - 1, 0));
  const bool x_upper_out = map_->IsOnMap(MapIndex(kStaticMapSize, 0));
  const bool x_lower_in = map_->IsOnMap(MapIndex(0, 0));
  const bool x_lower_out = map_->IsOnMap(MapIndex(-1, 0));
  const bool y_upper_in = map_->IsOnMap(MapIndex(0, kStaticMapSize - 1));
  const bool y_upper_out = map_->IsOnMap(MapIndex(0, kStaticMapSize));
  const bool y_lower_in = map_->IsOnMap(MapIndex(0, 0));
  const bool y_lower_out = map_->IsOnMap(MapIndex(0, -1));

  // verify
  EXPECT_TRUE(x_upper_in);
  EXPECT_FALSE(x_upper_out);
  EXPECT_TRUE(x_lower_in);
  EXPECT_FALSE(x_lower_out);
  EXPECT_TRUE(y_upper_in);
  EXPECT_FALSE(y_upper_out);
  EXPECT_TRUE(y_lower_in);
  EXPECT_FALSE(y_lower_out);
}

/// IsPassable test
/// Areas of Wall in the dynamic map are determined to be impassable
/// Areas below Wall and Unknown are determined to be passable
TEST_F(LayeredCostMapTest, DynamicMapPassability) {
  // exercise
  // Check areas of kWallValue, kWallValue-1, and Unknown
  const bool wall_passable = map_->IsPassable(MapIndex(dynamic_map_offset_x_, dynamic_map_offset_y_ + kWallValue));
  const bool not_wall_passable =
      map_->IsPassable(MapIndex(dynamic_map_offset_x_, dynamic_map_offset_y_ + kWallValue - 1));
  const bool unknown_passable = map_->IsPassable(MapIndex(dynamic_map_offset_x_, dynamic_map_offset_y_));

  // verify
  EXPECT_FALSE(wall_passable);
  EXPECT_TRUE(not_wall_passable);
  // Unknown areas in the dynamic map are passable
  EXPECT_TRUE(unknown_passable);
}

/// IsPassable test
/// Areas outside the static map are determined to be impassable
TEST_F(LayeredCostMapTest, StaticMapBoundaryCheck) {
  // exercise
  // Input coordinates that deviate by one grid each in X and Y from the static map and confirm they are impassable
  const bool out_x_m = map_->IsPassable(MapIndex(-1, 0));
  const bool out_x_p = map_->IsPassable(MapIndex(kStaticMapSize, 0));
  const bool out_y_m = map_->IsPassable(MapIndex(0, -1));
  const bool out_y_p = map_->IsPassable(MapIndex(0, kStaticMapSize));

  // verify
  EXPECT_FALSE(out_x_m);
  EXPECT_FALSE(out_x_p);
  EXPECT_FALSE(out_y_m);
  EXPECT_FALSE(out_y_p);
}

/// IsPassable test
/// Areas outside the dynamic map are determined to be passable
TEST_F(LayeredCostMapTest, DynamicMapBoundaryCheck) {
  // exercize
  // Input coordinates that deviate by one grid each in X and Y from the dynamic map and confirm they are passable
  const bool out_x_m = map_->IsPassable(MapIndex(dynamic_map_offset_x_ - 1, dynamic_map_offset_y_));
  const bool out_x_p = map_->IsPassable(MapIndex(dynamic_map_offset_x_ + kDynamicMapSize, dynamic_map_offset_y_));
  const bool out_y_m = map_->IsPassable(MapIndex(dynamic_map_offset_x_, dynamic_map_offset_y_  - 1));
  const bool out_y_p = map_->IsPassable(MapIndex(dynamic_map_offset_x_, dynamic_map_offset_y_ + kDynamicMapSize));
  // verify
  EXPECT_TRUE(out_x_m);
  EXPECT_TRUE(out_x_p);
  EXPECT_TRUE(out_y_m);
  EXPECT_TRUE(out_y_p);
}

/// IsPassable test
/// Unknown areas of the static map are determined to be impassable
TEST_F(LayeredCostMapTest, StaticMapCostUnknown) {
  // exercise
  // Unknown areas of the static map are treated as Wall
  const bool static_unknown = map_->IsPassable(MapIndex(kMapParam.cost_unknown, 0));

  // vefiry
  EXPECT_FALSE(static_unknown);
}

/// IsPassable test
/// All areas above the Wall threshold in the static map are determined to be impassable
TEST_F(LayeredCostMapTest, StaticMapCostWallRange) {
  // exercise
  bool result = false;
  for (int32_t i = map_->static_map_occupancy_threshold() * kStaticMapScale; i < kStaticMapSize; ++i) {
    result |= map_->IsPassable(MapIndex(i, 0));
  }

  // vefiry
  EXPECT_FALSE(result);
}

/// IsPassable test
/// All areas below the Wall threshold in the static map, except unknown, are determined to be passable
TEST_F(LayeredCostMapTest, StaticMapCostFreeRange) {
  // exercise
  bool result = true;
  for (int32_t i = 0; i < map_->static_map_occupancy_threshold() * kStaticMapScale; ++i) {
    // Excluding unknown
    if (i / kStaticMapScale == kMapParam.cost_unknown) {
      continue;
    }
    result &= map_->IsPassable(MapIndex(i, 0));
  }

  // vefiry
  EXPECT_TRUE(result);
}

/// IsPassable test
/// When the resolution of the dynamic map is smaller than that of the static map, passability is correctly determined
TEST_F(LayeredCostMapTest, DynamicMapSmallResolution) {
  // setup
  // For the dynamic map of other tests, halve the resolution and double the size
  // Generate a dynamic map with a gradient of 0..255 in the Y direction
  const double dynamic_map_resolution = kResolution / 2.0;
  const int32_t dynamic_map_size = kDynamicMapSize * 2;
  std::vector<uint8_t> dynamic_map_data(dynamic_map_size * dynamic_map_size);
  for (int32_t y = 0; y < dynamic_map_size; ++y) {
    for (int32_t x = 0; x < dynamic_map_size; ++x) {
      dynamic_map_data[dynamic_map_size * y + x] = static_cast<uint8_t>(y / 2.0);
    }
  }
  CostMapPtr dynamic_map = std::make_shared<CostMap>(
      CostMap(kDynamicMapOrigin, dynamic_map_resolution, dynamic_map_size, dynamic_map_size, dynamic_map_data));
  map_->SetDynamicMap(dynamic_map, kDynamicMapOrigin);

  // exercize
  // Areas of Wall are determined to be impassable, and areas below Wall and Unknown are determined to be passable
  const bool wall_passable =
      map_->IsPassable(MapIndex(dynamic_map_offset_x_, dynamic_map_offset_y_ + kDynamicMapSize - 1));
  const bool not_wall_passable =
      map_->IsPassable(MapIndex(dynamic_map_offset_x_, dynamic_map_offset_y_ + kDynamicMapSize - 3));
  const bool unknown_passable = map_->IsPassable(MapIndex(dynamic_map_offset_x_, dynamic_map_offset_y_));

  // verify
  EXPECT_FALSE(wall_passable);
  EXPECT_TRUE(not_wall_passable);
  EXPECT_TRUE(unknown_passable);
}

/// LayeredCostMap test
/// When the dynamic map is rotated 180 degrees, passability is correctly determined
TEST_F(LayeredCostMapTest, DynamicMapRotate) {
  // setup
  // Generate a dynamic map with a gradient of 0..255 in the Y direction
  std::vector<uint8_t> dynamic_map_data(kDynamicMapSize * kDynamicMapSize);
  for (int32_t y = 0; y < kDynamicMapSize; ++y) {
    for (int32_t x = 0; x < kDynamicMapSize; ++x) {
      dynamic_map_data[kDynamicMapSize * y + x] = y;
    }
  }
  Pose2d origin;
  // Rotate the origin of the dynamic map 180 degrees and move the origin by the map size
  // Slightly offset to avoid errors when perfectly overlapping
  origin.set_x(kDynamicMapOrigin.x() + kDynamicMapSize * kResolution - kEpsilon);
  origin.set_y(kDynamicMapOrigin.y() + kDynamicMapSize * kResolution - kEpsilon);
  origin.set_theta(M_PI);
  CostMapPtr dynamic_map = std::make_shared<CostMap>(
      CostMap(origin, kResolution, kDynamicMapSize, kDynamicMapSize, dynamic_map_data));
  map_->SetDynamicMap(dynamic_map, origin);

  // exercize
  // Areas of Wall are determined to be impassable, and areas below Wall and Unknown are determined to be passable
  const bool wall_passable = map_->IsPassable(MapIndex(dynamic_map_offset_x_, dynamic_map_offset_y_));
  const bool not_wall_passable = map_->IsPassable(MapIndex(dynamic_map_offset_x_, dynamic_map_offset_y_ + 1));
  const bool unknown_passable =
      map_->IsPassable(MapIndex(dynamic_map_offset_x_, dynamic_map_offset_y_ + kDynamicMapSize - 1));

  // verify
  EXPECT_FALSE(wall_passable);
  EXPECT_TRUE(not_wall_passable);
  EXPECT_TRUE(unknown_passable);
}

/// PoseToIndex test
/// When converting map coordinates to grid coordinates, they are converted to the coordinates of the grid containing the map coordinates
/// Example: When resolution is 0.05, it is converted as follows (common for X and Y)
/// [-0.05,  0.00) → -1
/// [ 0.00,  0.05) →  0
/// [ 0.05,  0.10) →  1
TEST_F(LayeredCostMapTest, PoseToIndex) {
  // setup
  // The range of map coordinates [0.00, 0.05) becomes grid coordinate 0
  // Lower limit
  const Pose2d plus_boundary_0_lower_pose(0.0 + kEpsilon, 0.0 + kEpsilon, 0.0);
  // Upper limit
  const Pose2d plus_boundary_0_upper_pose(kResolution - kEpsilon, kResolution - kEpsilon, 0.0);

  // The range of map coordinates [0.05, 0.10) becomes grid coordinate 1
  // Lower limit
  const Pose2d plus_boundary_1_lower_pose(kResolution + kEpsilon, kResolution + kEpsilon, 0.0);
  // Upper limit
  const Pose2d plus_boundary_1_upper_pose(kResolution * 2 - kEpsilon, kResolution * 2 - kEpsilon, 0.0);

  // The range of map coordinates [-0.05, 0.00) becomes grid coordinate -1
  // Lower limit
  const Pose2d minus_boundary_1_upper_pose(0.0 - kEpsilon, 0.0 - kEpsilon, 0.0);  // Upper limit
  // Upper limit
  const Pose2d minus_boundary_1_lower_pose(-kResolution + kEpsilon, -kResolution + kEpsilon, 0.0);  // Lower limit

  MapIndex plus_boundary_0_lower_index;
  MapIndex plus_boundary_0_upper_index;
  MapIndex plus_boundary_1_lower_index;
  MapIndex plus_boundary_1_upper_index;
  MapIndex minus_boundary_1_lower_index;
  MapIndex minus_boundary_1_upper_index;

  // exercise
  map_->PoseToIndex(plus_boundary_0_lower_pose, plus_boundary_0_lower_index);
  map_->PoseToIndex(plus_boundary_0_upper_pose, plus_boundary_0_upper_index);
  map_->PoseToIndex(plus_boundary_1_lower_pose, plus_boundary_1_lower_index);
  map_->PoseToIndex(plus_boundary_1_upper_pose, plus_boundary_1_upper_index);
  map_->PoseToIndex(minus_boundary_1_lower_pose, minus_boundary_1_lower_index);
  map_->PoseToIndex(minus_boundary_1_upper_pose, minus_boundary_1_upper_index);

  // verify
  EXPECT_EQ(MapIndex(0, 0), plus_boundary_0_lower_index);
  EXPECT_EQ(MapIndex(0, 0), plus_boundary_0_upper_index);
  EXPECT_EQ(MapIndex(1, 1), plus_boundary_1_lower_index);
  EXPECT_EQ(MapIndex(1, 1), plus_boundary_1_upper_index);
  EXPECT_EQ(MapIndex(-1, -1), minus_boundary_1_lower_index);
  EXPECT_EQ(MapIndex(-1, -1), minus_boundary_1_upper_index);
}

/// PoseToIndexes test
/// When converting map coordinates to surrounding grid coordinates, the grid overlapping the square centered on the specified coordinates is output
TEST_F(LayeredCostMapTest, PoseToIndexes) {
  // setup
  const Pose2d center(kStaticMapSize * kResolution / 2, kStaticMapSize * kResolution / 2);
  const double range = 0.5;
  std::vector<MapIndex> indexes;

  // exercise
  map_->PoseToIndexes(center, range, indexes);

  // verify
  const int32_t min_x = static_cast<int32_t>((center.x() - range) / kResolution);
  const int32_t max_x = static_cast<int32_t>((center.x() + range) / kResolution);
  const int32_t min_y = static_cast<int32_t>((center.y() - range) / kResolution);
  const int32_t max_y = static_cast<int32_t>((center.y() + range) / kResolution);
  // The number of elements is as expected
  ASSERT_EQ((max_x - min_x + 1) * (max_y - min_y + 1), indexes.size());
  // All indices within the range are obtained
  for (int32_t x = min_x; x <= max_x; ++x) {
    for (int32_t y = min_y; y <= max_y; ++y) {
      const MapIndex expect_index(x, y);
      ASSERT_NE(indexes.end(), std::find(indexes.begin(), indexes.end(), expect_index));
    }
  }
}

/// PoseToIndexes test
/// When obtaining surrounding grid coordinates at the edge of the static map, only those within the range of the static map are output
TEST_F(LayeredCostMapTest, PoseToIndexesEdgeOfStatiMap) {
  // setup
  const Pose2d center(0.0, 0.0);
  const double range = 0.5;
  std::vector<MapIndex> indexes;

  // exercise
  map_->PoseToIndexes(center, range, indexes);

  // verify
  const int32_t min_x = 0;
  const int32_t max_x = static_cast<int32_t>((center.x() + range) / kResolution);
  const int32_t min_y = 0;
  const int32_t max_y = static_cast<int32_t>((center.y() + range) / kResolution);
  // The number of elements is as expected
  ASSERT_EQ((max_x - min_x + 1) * (max_y - min_y + 1), indexes.size());
  // All indices within the range are obtained
  for (int32_t x = min_x; x <= max_x; ++x) {
    for (int32_t y = min_y; y <= max_y; ++y) {
      const MapIndex expect_index(x, y);
      ASSERT_NE(indexes.end(), std::find(indexes.begin(), indexes.end(), expect_index));
    }
  }
}

/// PoseToIndexes test
/// If a positive value is not set for the range when obtaining surrounding grid coordinates, only the grid coordinates to which the specified coordinates belong are output
TEST_F(LayeredCostMapTest, PoseToIndexesRange0) {
  // setup
  const Pose2d center(0.0, 0.0);
  const double range = 0.0;
  std::vector<MapIndex> indexes;

  // exercise
  map_->PoseToIndexes(center, range, indexes);

  // verify
  // The number of elements is as expected
  ASSERT_EQ(1, indexes.size());
  // All indices within the range are obtained
  EXPECT_EQ(MapIndex(0, 0), indexes[0]);
}

/// IndexToPose test
/// When converting grid coordinates to map coordinates, they are converted to map coordinates pointing to the center of the grid
/// Example: When resolution is 0.05, it is converted as follows (common for X and Y)
/// -1 → -0.025
///  0 →  0.025
///  1 →  0.075
TEST_F(LayeredCostMapTest, IndexToPose) {
  // exercise
  // Conversion from grid to map is to coordinates pointing to the center of the grid
  // Grid (0, 0) becomes map (0.025, 0.025) This one case is checked independently for X and Y
  Pose2d plus_0_map_x;
  Pose2d plus_0_map_y;
  map_->IndexToPose(MapIndex(0, 10), plus_0_map_x);
  map_->IndexToPose(MapIndex(10, 0), plus_0_map_y);
  // Grid (1, 1) becomes map (0.075, 0.075)
  Pose2d plus_1_map;
  map_->IndexToPose(MapIndex(1, 1), plus_1_map);
  // Grid (-1, -1) becomes map (-0.025, -0.025)
  Pose2d minus_1_map;
  map_->IndexToPose(MapIndex(-1, -1), minus_1_map);

  // verify
  EXPECT_DOUBLE_EQ(0.5 * kResolution, plus_0_map_x.x());
  EXPECT_DOUBLE_EQ(0.5 * kResolution, plus_0_map_y.y());
  EXPECT_DOUBLE_EQ(1.5 * kResolution, plus_1_map.x());
  EXPECT_DOUBLE_EQ(1.5 * kResolution, plus_1_map.y());
  EXPECT_DOUBLE_EQ(-0.5 * kResolution, minus_1_map.x());
  EXPECT_DOUBLE_EQ(-0.5 * kResolution, minus_1_map.y());
}

/// EstimateMaxCost test
/// Maximum cost estimation outputs the value obtained by multiplying the difference in grid coordinates between start and goal by cost_factor
TEST_F(LayeredCostMapTest, EstimateMaxCost) {
  // setup
  const int32_t x_diff = 10;
  const int32_t y_diff = 20;
  const Pose2d start(0, 0, 0);
  const Pose2d goal(x_diff * kResolution, y_diff * kResolution, 0);

  // exercise
  const int32_t estimate_max_cost = map_->EstimateMaxCost(start, goal);

  // vefiry
  EXPECT_EQ((x_diff + y_diff) * kMapParam.cost_factor, estimate_max_cost);
}

// Parameters for GetNextNodes test
struct GetNextNodesParameter {
  GetNextNodesParameter(
      const MapIndex& in_current_index, const NodeDirection& in_current_direction,
      const int32_t in_max_cost, const std::vector<MapIndex>& in_expect_indexes) :
      current_index(in_current_index), current_direction(in_current_direction),
      max_cost(in_max_cost), expect_indexes(in_expect_indexes) {}
  MapIndex current_index;
  NodeDirection current_direction;
  int32_t max_cost;
  std::vector<MapIndex> expect_indexes;
};

// GetNextNodes test
class GetNextNodesParameterTest : public ::testing::TestWithParam<GetNextNodesParameter> {
 public:
  GetNextNodesParameterTest() {}

 protected:
  virtual void SetUp() {
    // The static and dynamic maps used in this test are made the same size as the dynamic map
    // Generate a static map with a gradient of 0..255 in the X direction
    Pose2d static_map_origin(0.0, 0.0, 0.0);
    std::vector<uint8_t> static_map_data(kDynamicMapSize * kDynamicMapSize);
    for (int32_t y = 0; y < kDynamicMapSize; ++y) {
      for (int32_t x = 0; x < kDynamicMapSize; ++x) {
        static_map_data[kDynamicMapSize * y + x] = x;
      }
    }
    static_map_ = std::make_shared<CostMap>(
      CostMap(static_map_origin, kResolution, kDynamicMapSize, kDynamicMapSize, static_map_data));
    map_ = std::make_shared<LayeredCostMap>(LayeredCostMap(kMapParam, static_map_));
    // Generate a dynamic map with a gradient of 0..255 in the Y direction, the same size as the static map
    std::vector<uint8_t> dynamic_map_data(kDynamicMapSize * kDynamicMapSize);
    for (int32_t y = 0; y < kDynamicMapSize; ++y) {
      for (int32_t x = 0; x < kDynamicMapSize; ++x) {
        dynamic_map_data[kDynamicMapSize * y + x] = y;
      }
    }
    CostMapPtr dynamic_map = std::make_shared<CostMap>(
        CostMap(Pose2d(0.0, 0.0, 0.0), kResolution, kDynamicMapSize, kDynamicMapSize, dynamic_map_data));
    map_->SetDynamicMap(dynamic_map, Pose2d(0.0, 0.0, 0.0));
    queue_ = std::make_shared<AstarQueue>();
    node_manager_ = std::make_shared<AstarNodeManager>(AstarNodeManager(kDynamicMapSize, kDynamicMapSize));
  }
  LayeredCostMap::Ptr map_;
  // Map
  CostMapPtr static_map_;
  AstarQueue::Ptr queue_;
  IAstarNodeManager::Ptr node_manager_;
};

// Test case
INSTANTIATE_TEST_CASE_P(
    GetNextNodesTest,
    GetNextNodesParameterTest,
    testing::Values(
        // In the case of DIR_0, the front and diagonal adjacent nodes of DIR0 are output
        GetNextNodesParameter(MapIndex(10, 10), NodeDirection::DIR_0, kMaxCost,
                              {MapIndex(11, 9), MapIndex(11, 10), MapIndex(11, 11)}),
        // In the case of DIR_45, the front and diagonal adjacent nodes of DIR45 are output
        GetNextNodesParameter(MapIndex(10, 10), NodeDirection::DIR_45, kMaxCost,
                              {MapIndex(10, 11), MapIndex(11, 11), MapIndex(11, 10)}),
        // In the case of DIR_90, the front and diagonal adjacent nodes of DIR90 are output
        GetNextNodesParameter(MapIndex(10, 10), NodeDirection::DIR_90, kMaxCost,
                              {MapIndex(9, 11), MapIndex(10, 11), MapIndex(11, 11)}),
        // In the case of DIR_135, the front and diagonal adjacent nodes of DIR135 are output
        GetNextNodesParameter(MapIndex(10, 10), NodeDirection::DIR_135, kMaxCost,
                              {MapIndex(9, 10), MapIndex(9, 11), MapIndex(10, 11)}),
        // In the case of DIR_180, the front and diagonal adjacent nodes of DIR180 are output
        GetNextNodesParameter(MapIndex(10, 10), NodeDirection::DIR_180, kMaxCost,
                              {MapIndex(9, 9), MapIndex(9, 10), MapIndex(9, 11)}),
        // In the case of DIR_225, the front and diagonal adjacent nodes of DIR225 are output
        GetNextNodesParameter(MapIndex(10, 10), NodeDirection::DIR_225, kMaxCost,
                              {MapIndex(9, 9), MapIndex(9, 10), MapIndex(10, 9)}),
        // In the case of DIR_270, the front and diagonal adjacent nodes of DIR270 are output
        GetNextNodesParameter(MapIndex(10, 10), NodeDirection::DIR_270, kMaxCost,
                              {MapIndex(9, 9), MapIndex(10, 9), MapIndex(11, 9)}),
        // In the case of DIR_315, the front and diagonal adjacent nodes of DIR315 are output
        GetNextNodesParameter(MapIndex(10, 10), NodeDirection::DIR_315, kMaxCost,
                              {MapIndex(10, 9), MapIndex(11, 9), MapIndex(11, 10)}),
        // In the case of DIR_NONE, all surrounding adjacent nodes are output
        GetNextNodesParameter(MapIndex(10, 10), NodeDirection::DIR_None, kMaxCost,
                              {MapIndex(9, 9), MapIndex(9, 10), MapIndex(9, 11),
                               MapIndex(10, 9), MapIndex(10, 11),
                               MapIndex(11, 9), MapIndex(11, 10), MapIndex(11, 11)}),
        // When exploring from the edge of the static map, only adjacent nodes within the static map are output
        GetNextNodesParameter(MapIndex(10, 0), NodeDirection::DIR_None, kMaxCost,
                              {MapIndex(9, 0), MapIndex(9, 1), MapIndex(10, 1), MapIndex(11, 0), MapIndex(11, 1)}),
        // When exploring from the edge of the static map near a wall, only adjacent nodes other than the wall are output
        GetNextNodesParameter(MapIndex(kWallThreshold - 1, 10), NodeDirection::DIR_None, kMaxCost,
                              {MapIndex(kWallThreshold - 2, 9),
                               MapIndex(kWallThreshold - 2, 10),
                               MapIndex(kWallThreshold - 2, 11),
                               MapIndex(kWallThreshold - 1, 9),
                               MapIndex(kWallThreshold - 1, 11)}),
        // When exploring from the edge of the dynamic map near a wall, only adjacent nodes other than the wall are output
        GetNextNodesParameter(MapIndex(10, kDynamicMapSize - 2), NodeDirection::DIR_None, kMaxCost,
                              {MapIndex(9, kDynamicMapSize - 3),
                               MapIndex(10, kDynamicMapSize - 3),
                               MapIndex(11, kDynamicMapSize - 3),
                               MapIndex(9, kDynamicMapSize - 2),
                               MapIndex(11, kDynamicMapSize - 2)}),
        // When exploring from next to unknown in the static map, only adjacent nodes other than unknown are output
        GetNextNodesParameter(MapIndex(kMapParam.cost_unknown + 1, 10), NodeDirection::DIR_None, kMaxCost,
                              {MapIndex(kMapParam.cost_unknown + 1, 9),
                               MapIndex(kMapParam.cost_unknown + 1, 11),
                               MapIndex(kMapParam.cost_unknown + 2, 9),
                               MapIndex(kMapParam.cost_unknown + 2, 10),
                               MapIndex(kMapParam.cost_unknown + 2, 11)}),
        // Nodes that exceed the max cost are not output; cost setting allows only straight direction
        GetNextNodesParameter(MapIndex(10, 10), NodeDirection::DIR_None, kMapParam.diagonal_cost - 1,
                              {MapIndex(9, 10), MapIndex(10, 9), MapIndex(10, 11), MapIndex(11, 10)})
    )
);

/// GetNextNodes parameter test
/// Confirm that the exploration results from the current node are as expected
TEST_P(GetNextNodesParameterTest, GetNextNodes) {
  // setup
  GetNextNodesParameter param = ((GetNextNodesParameter)GetParam());
  queue_->Initialize(param.max_cost);
  AstarNode* const current_node = node_manager_->GetNode(param.current_index);
  current_node->Update(nullptr, 0, 0, false, static_cast<int32_t>(param.current_direction));

  // exercise
  map_->GetNextNodes(queue_, node_manager_, current_node, param.max_cost);

  // verify
  std::vector<MapIndex> indexes;
  // Dequeue from the queue
  while (indexes.size() <= param.expect_indexes.size()) {
    AstarNode* const next_node = queue_->Pop();
    if (NULL == next_node) {
      break;
    }
    // Store the indices of the obtained nodes
    indexes.push_back(next_node->index());
  }

  // The obtained size is as expected
  ASSERT_EQ(param.expect_indexes.size(), indexes.size());
  // The obtained adjacent nodes are as expected
  for (const MapIndex expect_index : param.expect_indexes) {
    ASSERT_NE(indexes.end(), std::find(indexes.begin(), indexes.end(), expect_index));
  }
}
}  // namespace tmc_astar_lib

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
