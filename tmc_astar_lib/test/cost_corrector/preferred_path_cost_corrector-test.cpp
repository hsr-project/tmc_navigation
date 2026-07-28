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
#include <vector>

#include <gtest/gtest.h>
#include <tmc_astar_lib/common.hpp>
#include <tmc_astar_lib/cost_corrector/preferred_path_cost_corrector.hpp>
#include <tmc_astar_lib/layered_cost_map.hpp>

namespace tmc_astar_lib {
// Edge size of the static map
constexpr int32_t kStaticMapSize = 100;
// Parameters
const PreferredPathCostCorrector::Parameter kCorrectorParam(-20, -10);
// Path length used for testing
constexpr int32_t kTestPathLength = 10;
// Fixed index of the straight path (Y-coordinate for horizontal paths, X-coordinate for vertical paths)
constexpr int32_t kPathFixedIndex = 50;
/// Threshold to consider as a wall when taking potential into account
const int32_t kOccupancyThreshold = static_cast<int32_t>(
    (1.0 - kExclusiveSizeDefault / kWallThresholdDefault) * kWallValue);

/// Parameter test
/// Ability to generate parameters
TEST(PreferredPathCostCorrectorParameterTest, ConstructParameter) {
  // exercise
  const PreferredPathCostCorrector::Parameter param = PreferredPathCostCorrector::Parameter(-20, -10);

  // verify
  EXPECT_DOUBLE_EQ(-20, param.cost_on_preferred_path);
  EXPECT_DOUBLE_EQ(-10, param.cost_around_preferred_path);
}

/// Parameter test
/// When invalid values are specified, default values are generated (absolute values are invalid)
TEST(PreferredPathCostCorrectorParameterTest, ConstructWithInvalidParameter) {
  // exercise
  const PreferredPathCostCorrector::Parameter param = PreferredPathCostCorrector::Parameter(10, 20);

  // verify
  EXPECT_DOUBLE_EQ(kCostOnPreferredPathDefault, param.cost_on_preferred_path);
  EXPECT_DOUBLE_EQ(kCostAroundPreferredPathDefault, param.cost_around_preferred_path);
}

/// Parameter test
/// When invalid values are specified, default values are generated (mutual relationship between two parameters is invalid)
TEST(PreferredPathCostCorrectorParameterTest, ConstructWithRelativeInvalidParameter) {
  // exercise
  const PreferredPathCostCorrector::Parameter param = PreferredPathCostCorrector::Parameter(-10, -20);

  // verify
  EXPECT_DOUBLE_EQ(kCostOnPreferredPathDefault, param.cost_on_preferred_path);
  EXPECT_DOUBLE_EQ(kCostAroundPreferredPathDefault, param.cost_around_preferred_path);
}


/// Test fixture for PreferredPathCostCorrector
class PreferredPathCostCorrectorTest : public ::testing::Test {
 public:
  PreferredPathCostCorrectorTest() {}

 protected:
  virtual void SetUp() {
    // This CostCorrector depends on the size of the static map and the preferred_path
    // Only generation is performed here as preferred_path is set for each test
    corrector_ = std::make_shared<PreferredPathCostCorrector>(
        PreferredPathCostCorrector(kCorrectorParam));
  }

  ICostCorrector::Ptr corrector_;
};

/// PreferredPathCostCorrector test
/// Input a horizontal path and verify that correction values are set on the path and on the grids above and below it
TEST_F(PreferredPathCostCorrectorTest, HorizontalPath) {
  // setup
  std::vector<MapIndex> preferred_path_indexes;
  for (int32_t i = 0; i < kTestPathLength; ++i) {
    preferred_path_indexes.push_back(MapIndex(i, kPathFixedIndex));
  }
  ICostCorrector::SetupParams setup_param(kStaticMapSize, kStaticMapSize, preferred_path_indexes);
  corrector_->Setup(setup_param);

  // exercise & verify
  for (int32_t i = 0; i < kTestPathLength; ++i) {
    // On the path, correction is applied by cost_on_preferred_path
    const ICostCorrector::GetAdditionalCostParams param_on(
        nullptr, MapIndex(i, kPathFixedIndex), 0, 0, 0, kOccupancyThreshold);
    EXPECT_EQ(kCorrectorParam.cost_on_preferred_path, corrector_->GetAdditionalCost(param_on));
    // One grid above the path is corrected by cost_around_preferred_path
    const ICostCorrector::GetAdditionalCostParams param_up(
        nullptr, MapIndex(i, kPathFixedIndex + 1), 0, 0, 0, kOccupancyThreshold);
    EXPECT_EQ(kCorrectorParam.cost_around_preferred_path, corrector_->GetAdditionalCost(param_up));
    // One grid below the path is corrected by cost_around_preferred_path
    const ICostCorrector::GetAdditionalCostParams param_down(
        nullptr, MapIndex(i, kPathFixedIndex - 1), 0, 0, 0, kOccupancyThreshold);
    EXPECT_EQ(kCorrectorParam.cost_around_preferred_path, corrector_->GetAdditionalCost(param_down));
    // Two grids above the path are not corrected
    const ICostCorrector::GetAdditionalCostParams param_up_out(
        nullptr, MapIndex(i, kPathFixedIndex + 2), 0, 0, 0, kOccupancyThreshold);
    EXPECT_EQ(0, corrector_->GetAdditionalCost(param_up_out));
    // Two grids below the path are not corrected
    const ICostCorrector::GetAdditionalCostParams param_down_out(
        nullptr, MapIndex(i, kPathFixedIndex - 2), 0, 0, 0, kOccupancyThreshold);
    EXPECT_EQ(0, corrector_->GetAdditionalCost(param_down_out));
  }
}

/// PreferredPathCostCorrector test
/// Input a vertical path and verify that correction values are set on the path and on the grids to the left and right of it
TEST_F(PreferredPathCostCorrectorTest, VerticalPath) {
  // setup
  std::vector<MapIndex> preferred_path_indexes;
  for (int32_t i = 0; i < kTestPathLength; ++i) {
    preferred_path_indexes.push_back(MapIndex(kPathFixedIndex, i));
  }
  ICostCorrector::SetupParams setup_param(kStaticMapSize, kStaticMapSize, preferred_path_indexes);
  corrector_->Setup(setup_param);

  // exercise & verify
  for (int32_t i = 0; i < kTestPathLength; ++i) {
    // On the path, correction is applied by cost_on_preferred_path
    const ICostCorrector::GetAdditionalCostParams param_on(
        nullptr, MapIndex(kPathFixedIndex, i), 0, 0, 0, kOccupancyThreshold);
    EXPECT_EQ(kCorrectorParam.cost_on_preferred_path, corrector_->GetAdditionalCost(param_on));
    // One grid to the right of the path is corrected by cost_around_preferred_path
    const ICostCorrector::GetAdditionalCostParams param_right(
        nullptr, MapIndex(kPathFixedIndex + 1, i), 0, 0, 0, kOccupancyThreshold);
    EXPECT_EQ(kCorrectorParam.cost_around_preferred_path, corrector_->GetAdditionalCost(param_right));
    // One grid to the left of the path is corrected by cost_around_preferred_path
    const ICostCorrector::GetAdditionalCostParams param_left(
        nullptr, MapIndex(kPathFixedIndex - 1, i), 0, 0, 0, kOccupancyThreshold);
    EXPECT_EQ(kCorrectorParam.cost_around_preferred_path, corrector_->GetAdditionalCost(param_left));
    // Two grids to the right of the path are not corrected
    const ICostCorrector::GetAdditionalCostParams param_right_out(
        nullptr, MapIndex(kPathFixedIndex + 2, i), 0, 0, 0, kOccupancyThreshold);
    EXPECT_EQ(0, corrector_->GetAdditionalCost(param_right_out));
    // Two grids to the left of the path are not corrected
    const ICostCorrector::GetAdditionalCostParams param_left_out(
        nullptr, MapIndex(kPathFixedIndex - 2, i), 0, 0, 0, kOccupancyThreshold);
    EXPECT_EQ(0, corrector_->GetAdditionalCost(param_left_out));
  }
}

/// PreferredPathCostCorrector test
/// Input a 45° path and verify that correction values are set on the path and on the grids above, below, left, and right of it
TEST_F(PreferredPathCostCorrectorTest, DiagonalPath) {
  // setup
  std::vector<MapIndex> preferred_path_indexes;
  for (int32_t i = 0; i < kTestPathLength; ++i) {
    preferred_path_indexes.push_back(MapIndex(i, i));
  }
  ICostCorrector::SetupParams setup_param(kStaticMapSize, kStaticMapSize, preferred_path_indexes);
  corrector_->Setup(setup_param);

  // exercise & verify
  for (int32_t i = 1; i < kTestPathLength; ++i) {
    // On the path, correction is applied by cost_on_preferred_path
    const ICostCorrector::GetAdditionalCostParams param_on(
        nullptr, MapIndex(i, i), 0, 0, 0, kOccupancyThreshold);
    EXPECT_EQ(kCorrectorParam.cost_on_preferred_path, corrector_->GetAdditionalCost(param_on));
    // One grid above the path is corrected by cost_around_preferred_path
    const ICostCorrector::GetAdditionalCostParams param_up(
        nullptr, MapIndex(i, i + 1), 0, 0, 0, kOccupancyThreshold);
    EXPECT_EQ(kCorrectorParam.cost_around_preferred_path, corrector_->GetAdditionalCost(param_up));
    // One grid below the path is corrected by cost_around_preferred_path
    const ICostCorrector::GetAdditionalCostParams param_down(
        nullptr, MapIndex(i, i - 1), 0, 0, 0, kOccupancyThreshold);
    EXPECT_EQ(kCorrectorParam.cost_around_preferred_path, corrector_->GetAdditionalCost(param_down));
    // One grid to the right of the path is corrected by cost_around_preferred_path
    const ICostCorrector::GetAdditionalCostParams param_right(
        nullptr, MapIndex(i + 1, i), 0, 0, 0, kOccupancyThreshold);
    EXPECT_EQ(kCorrectorParam.cost_around_preferred_path, corrector_->GetAdditionalCost(param_right));
    // One grid to the left of the path is corrected by cost_around_preferred_path
    const ICostCorrector::GetAdditionalCostParams param_left(
        nullptr, MapIndex(i - 1, i), 0, 0, 0, kOccupancyThreshold);
    EXPECT_EQ(kCorrectorParam.cost_around_preferred_path, corrector_->GetAdditionalCost(param_left));
    // Two grids above the path are not corrected
    const ICostCorrector::GetAdditionalCostParams param_up_out(
        nullptr, MapIndex(i, i + 2), 0, 0, 0, kOccupancyThreshold);
    EXPECT_EQ(0, corrector_->GetAdditionalCost(param_up_out));
    // Two grids below the path are not corrected
    const ICostCorrector::GetAdditionalCostParams param_down_out(
        nullptr, MapIndex(i, i - 2), 0, 0, 0, kOccupancyThreshold);
    EXPECT_EQ(0, corrector_->GetAdditionalCost(param_down_out));
    // Two grids to the right of the path are not corrected
    const ICostCorrector::GetAdditionalCostParams param_right_out(
        nullptr, MapIndex(i + 2, i), 0, 0, 0, kOccupancyThreshold);
    EXPECT_EQ(0, corrector_->GetAdditionalCost(param_right_out));
    // Two grids to the left of the path are not corrected
    const ICostCorrector::GetAdditionalCostParams param_left_out(
        nullptr, MapIndex(i - 2, i), 0, 0, 0, kOccupancyThreshold);
    EXPECT_EQ(0, corrector_->GetAdditionalCost(param_left_out));
  }
}
}  // namespace tmc_astar_lib

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
