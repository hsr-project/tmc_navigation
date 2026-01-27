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

#include <gtest/gtest.h>
#include <tmc_astar_lib/astar_node_manager.hpp>
#include <tmc_astar_lib/node_direction.hpp>

namespace {
// Edge size of the static map
constexpr int32_t kStaticMapSize = 1000;
}  // anonymous namespace

namespace tmc_astar_lib {

/// Test fixture for AstarNodeManager
class AstarNodeManagerTest : public ::testing::Test {
 public:
  AstarNodeManagerTest() {}

 protected:
  virtual void SetUp() {
    nodes_ = std::make_shared<AstarNodeManager>(AstarNodeManager(kStaticMapSize, kStaticMapSize));
  }

  AstarNodeManager::Ptr nodes_;
};

/// AstarNodeManager test
/// Nodes corresponding to all grids can be obtained, and individual values can be set
TEST_F(AstarNodeManagerTest, MaximumCapacity) {
  // exercise
  for (int32_t y = 0; y < kStaticMapSize; ++y) {
    for (int32_t x = 0; x < kStaticMapSize; ++x) {
      AstarNode* node = nodes_->GetNode(MapIndex(x, y));
      node->Update(nullptr, kStaticMapSize * y + x, 0, true, static_cast<int32_t>(NodeDirection::DIR_0));
    }
  }

  // verify
  for (int32_t y = 0; y < kStaticMapSize; ++y) {
    for (int32_t x = 0; x < kStaticMapSize; ++x) {
      AstarNode* node = nodes_->GetNode(MapIndex(x, y));
      // Use ASSERT to check because EXPECT would result in an enormous log if it fails
      ASSERT_EQ(kStaticMapSize * y + x, node->total_cost());
    }
  }
}

/// AstarNodeManager test
/// An exception is raised when trying to reference a node out of range
TEST_F(AstarNodeManagerTest, FailCase) {
  // exercise & verify
  EXPECT_THROW(nodes_->GetNode(MapIndex(kStaticMapSize, 0)), std::exception);
}

}  // namespace tmc_astar_lib

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
