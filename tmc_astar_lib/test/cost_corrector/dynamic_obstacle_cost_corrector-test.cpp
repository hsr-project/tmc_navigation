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
#include <cmath>
#include <memory>
#include <vector>

#include <gtest/gtest.h>
#include <tmc_astar_lib/common.hpp>
#include <tmc_astar_lib/cost_corrector/dynamic_obstacle_cost_corrector.hpp>
#include <tmc_astar_lib/layered_cost_map.hpp>

namespace tmc_astar_lib {
/// Threshold considered as a wall when taking potential into account
const int32_t kOccupancyThreshold = static_cast<int32_t>(
    (1.0 - kExclusiveSizeDefault / kWallThresholdDefault) * kWallValue);

/// Test fixture for DynamicObstacleCostCorrector
class DynamicObstacleCostCorrectorTest : public ::testing::Test {
 public:
  DynamicObstacleCostCorrectorTest() {}

 protected:
  virtual void SetUp() {
    // Since this CostCorrector does not depend on setup parameters, call the initialization method with arbitrary values
    std::vector<MapIndex> preferred_path_indexes;
    ICostCorrector::SetupParams setup_param(50, 50, preferred_path_indexes);
    corrector_ = std::make_shared<DynamicObstacleCostCorrector>(DynamicObstacleCostCorrector());
    corrector_->Setup(setup_param);
  }
  ICostCorrector::Ptr corrector_;
};

/// DynamicObstacleCostCorrector test
/// No correction is applied in the case of Free
TEST_F(DynamicObstacleCostCorrectorTest, Free) {
  // exercise
  ICostCorrector::GetAdditionalCostParams param(
      nullptr, MapIndex(0, 0), 0, 0, 0, kOccupancyThreshold);
  const int32_t additional_cost = corrector_->GetAdditionalCost(param);

  // verify
  EXPECT_EQ(0, additional_cost);
}

/// DynamicObstacleCostCorrector test
/// No correction is applied in the gradient section from Free to Wall
TEST_F(DynamicObstacleCostCorrectorTest, FreeNorWall) {
  // exercise
  ICostCorrector::GetAdditionalCostParams param(
      nullptr, MapIndex(0, 0), 0, 0, kOccupancyThreshold - 1, kOccupancyThreshold);
  const int32_t additional_cost = corrector_->GetAdditionalCost(param);

  // verify
  EXPECT_EQ(0, additional_cost);
}

/// DynamicObstacleCostCorrector test
/// As a result of considering potential, in the case of Wall, the value is corrected to twice the original cost value
TEST_F(DynamicObstacleCostCorrectorTest, Wall) {
  // exercise
  ICostCorrector::GetAdditionalCostParams param(
      nullptr, MapIndex(0, 0), 0, 0, kOccupancyThreshold, kOccupancyThreshold);
  const int32_t additional_cost = corrector_->GetAdditionalCost(param);

  // verify
  // The difference between the value multiplied by the cost ratio and the original cost is returned as the correction value
  EXPECT_EQ(static_cast<int32_t>(static_cast<double>(param.dynamic_cost) * (kObstacleCostFactor - 1.0)),
            additional_cost);
}

}  // namespace tmc_astar_lib

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
