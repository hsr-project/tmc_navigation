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
#include <limits>
#include <gtest/gtest.h>
#include <tmc_astar_lib/astar_node.hpp>
#include <tmc_astar_lib/node_direction.hpp>

namespace {
// Test values: AstarNode only holds values without processing, so the actual values can be arbitrary
constexpr int32_t kGridX = 1;
constexpr int32_t kGridY = 2;
constexpr int32_t kTotalCost = 100;
constexpr int32_t kTotalStep = 200;
}  // anonymous namespace

namespace tmc_astar_lib {

/// Test fixture for AstarNode
class AstarNodeTest : public ::testing::Test {
 public:
  AstarNodeTest() : node_(MapIndex(kGridX, kGridY)) {}

 protected:
  virtual void SetUp() {}
  AstarNode node_;
};

/// AstarNode test
/// Verification of values initialized by the constructor
TEST_F(AstarNodeTest, Constructor) {
  // verify
  EXPECT_EQ(kGridX, node_.index().x);
  EXPECT_EQ(kGridY, node_.index().y);
  EXPECT_FALSE(node_.is_closed());
  EXPECT_EQ(std::numeric_limits<int32_t>::max(), node_.total_cost());
  EXPECT_EQ(0, node_.total_step());
  EXPECT_EQ(0, node_.additional_info());
  EXPECT_EQ(nullptr, node_.parent());
  EXPECT_EQ(-1, node_.queue_index());
  EXPECT_EQ(nullptr, node_.next());
  EXPECT_EQ(nullptr, node_.prev());
}

/// AstarNode test
/// Ensuring the state becomes closed with Close
TEST_F(AstarNodeTest, Close) {
  // exercise
  node_.Close();

  // verify
  EXPECT_TRUE(node_.is_closed());
}

/// AstarNode test
/// Verification of value updates with Update
TEST_F(AstarNodeTest, Update) {
  // exercise
  AstarNode dummy_parent(MapIndex(0, 0));
  node_.Update(&dummy_parent, kTotalCost, kTotalStep, true, static_cast<int32_t>(NodeDirection::DIR_180));

  // verify
  EXPECT_EQ(&dummy_parent, node_.parent());
  EXPECT_EQ(kTotalCost, node_.total_cost());
  EXPECT_EQ(kTotalStep, node_.total_step());
  EXPECT_EQ(static_cast<int32_t>(NodeDirection::DIR_180), node_.additional_info());
}

/// AstarNode test
/// After closing, Update(open=true) transitions to Open state
TEST_F(AstarNodeTest, ReOpen) {
  // exercise
  node_.Close();
  node_.Update(nullptr, kTotalCost, kTotalStep, true, static_cast<int32_t>(NodeDirection::DIR_180));

  // verify
  EXPECT_FALSE(node_.is_closed());
}

/// AstarNode test
/// After closing, Update(open=false) maintains the Closed state
TEST_F(AstarNodeTest, StayInClose) {
  // exersise
  node_.Close();
  node_.Update(nullptr, kTotalCost, kTotalStep, false, static_cast<int32_t>(NodeDirection::DIR_180));

  // verify
  EXPECT_TRUE(node_.is_closed());
}

/// AstarNode test
/// For Open state, Update(open=false) does not transition to Closed state
TEST_F(AstarNodeTest, NotClosedByUpdate) {
  // exersise
  node_.Update(nullptr, kTotalCost, kTotalStep, false, static_cast<int32_t>(NodeDirection::DIR_180));

  // verify
  EXPECT_FALSE(node_.is_closed());
}

// List operation-related tests are ensured by AstarQueue's automated tests

}  // namespace tmc_astar_lib

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
