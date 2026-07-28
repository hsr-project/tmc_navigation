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
#include <tmc_astar_lib/cost_corrector/direction_cost_corrector.hpp>
#include <tmc_astar_lib/layered_cost_map.hpp>

namespace tmc_astar_lib {
/// Threshold considered as a wall when taking potential into account
const int32_t kOccupancyThreshold = static_cast<int32_t>(
    (1.0 - kExclusiveSizeDefault / kWallThresholdDefault) * kWallValue);

/// Test fixture for DirectionCostCorrector
class DirectionCostCorrectorTest : public ::testing::Test {
 public:
  DirectionCostCorrectorTest() {}

 protected:
  virtual void SetUp() {
    // Since this CostCorrector does not depend on setup parameters, call the initialization method with arbitrary values
    nodes_ = std::make_shared<AstarNodeManager>(AstarNodeManager(100, 100));
    std::vector<MapIndex> preferred_path_indexes;
    ICostCorrector::SetupParams setup_param(50, 50, preferred_path_indexes);
    corrector_ = std::make_shared<DirectionCostCorrector>(DirectionCostCorrector());
    corrector_->Setup(setup_param);
  }

  ICostCorrector::Ptr corrector_;
  AstarNodeManager::Ptr nodes_;
};

// Test all combinations for one direction and try representative patterns for each direction

/// DirectionCostCorrector test
/// No correction if the number of steps from the start is less than 3
TEST_F(DirectionCostCorrectorTest, RightAfterStart) {
  // exercise
  // Start
  // Arbitrarily set values for parameters other than parent, total_steps, and in_direction as they are not referenced
  nodes_->GetNode(MapIndex(0, 0))->Update(nullptr, 0, 0, false, static_cast<int32_t>(NodeDirection::DIR_None));
  // First step
  nodes_->GetNode(MapIndex(1, 0))->Update(nodes_->GetNode(MapIndex(0, 0)), 0, 1, false,
      static_cast<int32_t>(NodeDirection::DIR_0));
  // Get correction value for the second step
  ICostCorrector::GetAdditionalCostParams param(
      nodes_->GetNode(MapIndex(1, 0)), MapIndex(2, 1), static_cast<int32_t>(NodeDirection::DIR_45),
      0, 0, kOccupancyThreshold);

  const int32_t additional_cost = corrector_->GetAdditionalCost(param);

  // verify
  EXPECT_EQ(0, additional_cost);
}

/// DirectionCostCorrector test
/// Pattern 0°→0°→0°: No correction
TEST_F(DirectionCostCorrectorTest, DIR_0_0_0) {
  // exercise
  int32_t total_steps = 0;
  nodes_->GetNode(MapIndex(0, 0))->Update(nullptr, 0, 0, false, static_cast<int32_t>(NodeDirection::DIR_None));
  nodes_->GetNode(MapIndex(1, 0))->Update(nodes_->GetNode(MapIndex(0, 0)), 0, 1, false,
      static_cast<int32_t>(NodeDirection::DIR_0));
  nodes_->GetNode(MapIndex(2, 0))->Update(nodes_->GetNode(MapIndex(1, 0)), 0, 2, false,
      static_cast<int32_t>(NodeDirection::DIR_0));
  ICostCorrector::GetAdditionalCostParams param(
      nodes_->GetNode(MapIndex(2, 0)), MapIndex(3, 0), static_cast<int32_t>(NodeDirection::DIR_0),
      0, 0, kOccupancyThreshold);
  const int32_t additional_cost = corrector_->GetAdditionalCost(param);

  // verify
  EXPECT_EQ(0, additional_cost);
}

/// DirectionCostCorrector test
/// Pattern 0°→0°→45°: Correction value for [adjacent→adjacent→diagonal] is applied
TEST_F(DirectionCostCorrectorTest, DIR_0_0_45) {
  // exercise
  nodes_->GetNode(MapIndex(0, 0))->Update(nullptr, 0, 0, false, static_cast<int32_t>(NodeDirection::DIR_None));
  nodes_->GetNode(MapIndex(1, 0))->Update(nodes_->GetNode(MapIndex(0, 0)), 0, 1, false,
      static_cast<int32_t>(NodeDirection::DIR_0));
  nodes_->GetNode(MapIndex(2, 0))->Update(nodes_->GetNode(MapIndex(1, 0)), 0, 2, false,
      static_cast<int32_t>(NodeDirection::DIR_0));
  ICostCorrector::GetAdditionalCostParams param(
      nodes_->GetNode(MapIndex(2, 0)), MapIndex(3, 1), static_cast<int32_t>(NodeDirection::DIR_45),
      0, 0, kOccupancyThreshold);
  const int32_t additional_cost = corrector_->GetAdditionalCost(param);

  // verify
  EXPECT_EQ(kCostAdjAdjDiag, additional_cost);
}

/// DirectionCostCorrector test
/// Pattern 0°→45°→0°: Correction value for [adjacent→diagonal→adjacent] is applied
TEST_F(DirectionCostCorrectorTest, DIR_0_45_0) {
  // exercise
  nodes_->GetNode(MapIndex(0, 0))->Update(nullptr, 0, 0, false, static_cast<int32_t>(NodeDirection::DIR_None));
  nodes_->GetNode(MapIndex(1, 0))->Update(nodes_->GetNode(MapIndex(0, 0)), 0, 1, false,
      static_cast<int32_t>(NodeDirection::DIR_0));
  nodes_->GetNode(MapIndex(2, 1))->Update(nodes_->GetNode(MapIndex(1, 0)), 0, 2, false,
      static_cast<int32_t>(NodeDirection::DIR_45));
  ICostCorrector::GetAdditionalCostParams param(
      nodes_->GetNode(MapIndex(2, 1)), MapIndex(3, 1), static_cast<int32_t>(NodeDirection::DIR_0),
      0, 0, kOccupancyThreshold);
  const int32_t additional_cost = corrector_->GetAdditionalCost(param);

  // verify
  EXPECT_EQ(kCostAdjDiagAdj, additional_cost);
}

/// DirectionCostCorrector test
/// Pattern 0°→45°→45°: Correction value for [adjacent→diagonal→diagonal] is applied
TEST_F(DirectionCostCorrectorTest, DIR_0_45_45) {
  // exercise
  nodes_->GetNode(MapIndex(0, 0))->Update(nullptr, 0, 0, false, static_cast<int32_t>(NodeDirection::DIR_None));
  nodes_->GetNode(MapIndex(1, 0))->Update(nodes_->GetNode(MapIndex(0, 0)), 0, 1, false,
      static_cast<int32_t>(NodeDirection::DIR_0));
  nodes_->GetNode(MapIndex(2, 1))->Update(nodes_->GetNode(MapIndex(1, 0)), 0, 2, false,
      static_cast<int32_t>(NodeDirection::DIR_45));
  ICostCorrector::GetAdditionalCostParams param(
      nodes_->GetNode(MapIndex(2, 1)), MapIndex(3, 2), static_cast<int32_t>(NodeDirection::DIR_45),
      0, 0, kOccupancyThreshold);
  const int32_t additional_cost = corrector_->GetAdditionalCost(param);

  // verify
  EXPECT_EQ(kCostAdjDiagDiag, additional_cost);
}

/// DirectionCostCorrector test
/// Pattern 45°→0°→0°: Correction value for [diagonal→adjacent→adjacent] is applied
TEST_F(DirectionCostCorrectorTest, DIR_45_0_0) {
  // exercise
  nodes_->GetNode(MapIndex(0, 0))->Update(nullptr, 0, 0, false, static_cast<int32_t>(NodeDirection::DIR_None));
  nodes_->GetNode(MapIndex(1, 1))->Update(nodes_->GetNode(MapIndex(0, 0)), 0, 1, false,
      static_cast<int32_t>(NodeDirection::DIR_45));
  nodes_->GetNode(MapIndex(2, 1))->Update(nodes_->GetNode(MapIndex(1, 1)), 0, 2, false,
      static_cast<int32_t>(NodeDirection::DIR_0));
  ICostCorrector::GetAdditionalCostParams param(
      nodes_->GetNode(MapIndex(2, 1)), MapIndex(3, 1), static_cast<int32_t>(NodeDirection::DIR_0),
      0, 0, kOccupancyThreshold);
  const int32_t additional_cost = corrector_->GetAdditionalCost(param);

  // verify
  EXPECT_EQ(kCostDiagAdjAdj, additional_cost);
}

/// DirectionCostCorrector test
/// Pattern 45°→0°→45°: Correction value for [diagonal→adjacent→diagonal] is applied
TEST_F(DirectionCostCorrectorTest, DIR_45_0_45) {
  // exercise
  nodes_->GetNode(MapIndex(0, 0))->Update(nullptr, 0, 0, false, static_cast<int32_t>(NodeDirection::DIR_None));
  nodes_->GetNode(MapIndex(1, 1))->Update(nodes_->GetNode(MapIndex(0, 0)), 0, 1, false,
      static_cast<int32_t>(NodeDirection::DIR_45));
  nodes_->GetNode(MapIndex(2, 1))->Update(nodes_->GetNode(MapIndex(1, 1)), 0, 2, false,
      static_cast<int32_t>(NodeDirection::DIR_0));
  ICostCorrector::GetAdditionalCostParams param(
      nodes_->GetNode(MapIndex(2, 1)), MapIndex(3, 2), static_cast<int32_t>(NodeDirection::DIR_45),
      0, 0, kOccupancyThreshold);
  const int32_t additional_cost = corrector_->GetAdditionalCost(param);

  // verify
  EXPECT_EQ(kCostDiagAdjDiag, additional_cost);
}

/// DirectionCostCorrector test
/// Pattern 45°→45°→45°: No correction
TEST_F(DirectionCostCorrectorTest, DIR_45_45_45) {
  // exercise
  nodes_->GetNode(MapIndex(0, 0))->Update(nullptr, 0, 0, false, static_cast<int32_t>(NodeDirection::DIR_None));
  nodes_->GetNode(MapIndex(1, 1))->Update(nodes_->GetNode(MapIndex(0, 0)), 0, 1, false,
      static_cast<int32_t>(NodeDirection::DIR_45));
  nodes_->GetNode(MapIndex(2, 2))->Update(nodes_->GetNode(MapIndex(1, 1)), 0, 2, false,
      static_cast<int32_t>(NodeDirection::DIR_45));
  ICostCorrector::GetAdditionalCostParams param(
      nodes_->GetNode(MapIndex(2, 2)), MapIndex(3, 3), static_cast<int32_t>(NodeDirection::DIR_45),
      0, 0, kOccupancyThreshold);
  const int32_t additional_cost = corrector_->GetAdditionalCost(param);

  // verify
  EXPECT_EQ(0, additional_cost);
}

/// DirectionCostCorrector test
/// Pattern 90°→90°→135°: Correction value for [adjacent→adjacent→diagonal] is applied
TEST_F(DirectionCostCorrectorTest, DIR_90_90_135) {
  // exercise
  nodes_->GetNode(MapIndex(10, 10))->Update(nullptr, 0, 0, false, static_cast<int32_t>(NodeDirection::DIR_None));
  nodes_->GetNode(MapIndex(10, 11))->Update(nodes_->GetNode(MapIndex(10, 10)), 0, 1, false,
      static_cast<int32_t>(NodeDirection::DIR_90));
  nodes_->GetNode(MapIndex(10, 12))->Update(nodes_->GetNode(MapIndex(10, 11)), 0, 2, false,
      static_cast<int32_t>(NodeDirection::DIR_90));
  ICostCorrector::GetAdditionalCostParams param(
      nodes_->GetNode(MapIndex(10, 12)), MapIndex(9, 13), static_cast<int32_t>(NodeDirection::DIR_135),
      0, 0, kOccupancyThreshold);
  const int32_t additional_cost = corrector_->GetAdditionalCost(param);

  // verify
  EXPECT_EQ(kCostAdjAdjDiag, additional_cost);
}

/// DirectionCostCorrector test
/// Pattern 135°→135°→180°: Correction value for [diagonal→diagonal→adjacent] is applied
TEST_F(DirectionCostCorrectorTest, DIR_135_135_180) {
  // exercise
  nodes_->GetNode(MapIndex(10, 10))->Update(nullptr, 0, 0, false, static_cast<int32_t>(NodeDirection::DIR_None));
  nodes_->GetNode(MapIndex(9, 11))->Update(nodes_->GetNode(MapIndex(10, 10)), 0, 1, false,
      static_cast<int32_t>(NodeDirection::DIR_135));
  nodes_->GetNode(MapIndex(8, 12))->Update(nodes_->GetNode(MapIndex(9, 11)), 0, 2, false,
      static_cast<int32_t>(NodeDirection::DIR_135));
  ICostCorrector::GetAdditionalCostParams param(
      nodes_->GetNode(MapIndex(8, 12)), MapIndex(7, 12), static_cast<int32_t>(NodeDirection::DIR_180),
      0, 0, kOccupancyThreshold);
  const int32_t additional_cost = corrector_->GetAdditionalCost(param);

  // verify
  EXPECT_EQ(kCostDiagDiagAdj, additional_cost);
}

/// DirectionCostCorrector test
/// Pattern 180°→180°→225°: Correction value for [adjacent→adjacent→diagonal] is applied
TEST_F(DirectionCostCorrectorTest, DIR_180_180_225) {
  // exercise
  nodes_->GetNode(MapIndex(10, 10))->Update(nullptr, 0, 0, false, static_cast<int32_t>(NodeDirection::DIR_None));
  nodes_->GetNode(MapIndex(9, 10))->Update(nodes_->GetNode(MapIndex(10, 10)), 0, 1, false,
      static_cast<int32_t>(NodeDirection::DIR_180));
  nodes_->GetNode(MapIndex(8, 10))->Update(nodes_->GetNode(MapIndex(9, 10)), 0, 2, false,
      static_cast<int32_t>(NodeDirection::DIR_180));
  ICostCorrector::GetAdditionalCostParams param(
      nodes_->GetNode(MapIndex(8, 10)), MapIndex(7, 9), static_cast<int32_t>(NodeDirection::DIR_225),
      0, 0, kOccupancyThreshold);
  const int32_t additional_cost = corrector_->GetAdditionalCost(param);

  // verify
  EXPECT_EQ(kCostAdjAdjDiag, additional_cost);
}

/// DirectionCostCorrector test
/// Pattern 225°→225°→270°: Correction value for [diagonal→diagonal→adjacent] is applied
TEST_F(DirectionCostCorrectorTest, DIR_225_225_270) {
  // exercise
  nodes_->GetNode(MapIndex(10, 10))->Update(nullptr, 0, 0, false, static_cast<int32_t>(NodeDirection::DIR_None));
  nodes_->GetNode(MapIndex(9, 9))->Update(nodes_->GetNode(MapIndex(10, 10)), 0, 1, false,
      static_cast<int32_t>(NodeDirection::DIR_225));
  nodes_->GetNode(MapIndex(8, 8))->Update(nodes_->GetNode(MapIndex(9, 9)), 0, 2, false,
      static_cast<int32_t>(NodeDirection::DIR_225));
  ICostCorrector::GetAdditionalCostParams param(
      nodes_->GetNode(MapIndex(8, 8)), MapIndex(8, 7), static_cast<int32_t>(NodeDirection::DIR_270),
      0, 0, kOccupancyThreshold);
  const int32_t additional_cost = corrector_->GetAdditionalCost(param);

  // verify
  EXPECT_EQ(kCostDiagDiagAdj, additional_cost);
}

/// DirectionCostCorrector test
/// Pattern 270°→270°→315°: Correction value for [adjacent→adjacent→diagonal] is applied
TEST_F(DirectionCostCorrectorTest, DIR_270_270_315) {
  // exercise
  nodes_->GetNode(MapIndex(10, 10))->Update(nullptr, 0, 0, false, static_cast<int32_t>(NodeDirection::DIR_None));
  nodes_->GetNode(MapIndex(10, 9))->Update(nodes_->GetNode(MapIndex(10, 10)), 0, 1, false,
      static_cast<int32_t>(NodeDirection::DIR_270));
  nodes_->GetNode(MapIndex(10, 8))->Update(nodes_->GetNode(MapIndex(10, 9)), 0, 2, false,
      static_cast<int32_t>(NodeDirection::DIR_270));
  ICostCorrector::GetAdditionalCostParams param(
      nodes_->GetNode(MapIndex(10, 8)), MapIndex(11, 7), static_cast<int32_t>(NodeDirection::DIR_315),
      0, 0, kOccupancyThreshold);
  const int32_t additional_cost = corrector_->GetAdditionalCost(param);

  // verify
  EXPECT_EQ(kCostAdjAdjDiag, additional_cost);
}

/// DirectionCostCorrector test
/// Pattern 315°→315°→0°: Correction value for [diagonal→diagonal→adjacent] is applied
TEST_F(DirectionCostCorrectorTest, DIR_315_315_0) {
  // exercise
  nodes_->GetNode(MapIndex(10, 10))->Update(nullptr, 0, 0, false, static_cast<int32_t>(NodeDirection::DIR_None));
  nodes_->GetNode(MapIndex(11, 9))->Update(nodes_->GetNode(MapIndex(10, 10)), 0, 1, false,
      static_cast<int32_t>(NodeDirection::DIR_315));
  nodes_->GetNode(MapIndex(12, 8))->Update(nodes_->GetNode(MapIndex(11, 9)), 0, 2, false,
      static_cast<int32_t>(NodeDirection::DIR_315));
  ICostCorrector::GetAdditionalCostParams param(
      nodes_->GetNode(MapIndex(12, 8)), MapIndex(13, 8), static_cast<int32_t>(NodeDirection::DIR_0),
      0, 0, kOccupancyThreshold);
  const int32_t additional_cost = corrector_->GetAdditionalCost(param);

  // verify
  EXPECT_EQ(kCostDiagDiagAdj, additional_cost);
}
}  // namespace tmc_astar_lib

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
