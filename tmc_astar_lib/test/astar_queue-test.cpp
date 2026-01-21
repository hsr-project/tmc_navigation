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
#include <tmc_astar_lib/astar_queue.hpp>
#include <tmc_astar_lib/node_direction.hpp>

namespace {
// Maximum cost for normal cases, set to a value that does not exceed
constexpr int32_t kMaxCost = 200;
// Base cost used for testing, express size by adding/subtracting from this
constexpr int32_t kTestCost = 100;
}  // anonymous namespace

namespace tmc_astar_lib {

/// AstarQueue test fixture
class AstarQueueTest : public ::testing::Test {
 public:
  AstarQueueTest() {}

 protected:
  virtual void SetUp() {
    queue_ = std::make_shared<AstarQueue>(AstarQueue());
  }

  AstarQueue::Ptr queue_;
};

/// AstarQueue test
/// Ensure nodes already inserted are cleared in Initialize
TEST_F(AstarQueueTest, Initialize) {
  // exercise
  // Initialize after PUSH, then POP
  queue_->Initialize(kMaxCost);
  AstarNode node(MapIndex(0, 0));
  node.Update(nullptr, kTestCost, 0, true, static_cast<int32_t>(NodeDirection::DIR_0));
  queue_->Push(&node);
  queue_->Initialize(kMaxCost);
  // Confirm null is POPed
  const AstarNode* popped = queue_->Pop();

  // verify
  EXPECT_EQ(nullptr, popped);
}

/// AstarQueue test
/// PUSH in order of small cost → medium cost → large cost, confirm they are POPed in order of smallest cost
TEST_F(AstarQueueTest, SameOrder) {
  // exercise
  queue_->Initialize(kMaxCost);
  // Set arbitrarily as only cost values are referenced from the queue
  AstarNode node_low(MapIndex(0, 0));
  node_low.Update(nullptr, kTestCost, 0, true, static_cast<int32_t>(NodeDirection::DIR_0));
  AstarNode node_middle(MapIndex(1, 0));
  node_middle.Update(nullptr, kTestCost + 1, 0, true, static_cast<int32_t>(NodeDirection::DIR_0));
  AstarNode node_high(MapIndex(2, 0));
  node_high.Update(nullptr, kTestCost + 2, 0, true, static_cast<int32_t>(NodeDirection::DIR_0));
  queue_->Push(&node_low);
  queue_->Push(&node_middle);
  queue_->Push(&node_high);
  const AstarNode* popped1 = queue_->Pop();
  const AstarNode* popped2 = queue_->Pop();
  const AstarNode* popped3 = queue_->Pop();
  // Confirm null is returned after it becomes empty
  const AstarNode* popped4 = queue_->Pop();

  // verify
  EXPECT_EQ(&node_low, popped1);
  EXPECT_EQ(&node_middle, popped2);
  EXPECT_EQ(&node_high, popped3);
  EXPECT_EQ(nullptr, popped4);
}

/// AstarQueue test
/// PUSH in order of large cost → medium cost → small cost, confirm they are POPed in order of smallest cost
TEST_F(AstarQueueTest, ReverseOrder) {
  // exercise
  queue_->Initialize(kMaxCost);
  AstarNode node_low(MapIndex(0, 0));
  node_low.Update(nullptr, kTestCost, 0, true, static_cast<int32_t>(NodeDirection::DIR_0));
  AstarNode node_middle(MapIndex(1, 0));
  node_middle.Update(nullptr, kTestCost + 1, 0, true, static_cast<int32_t>(NodeDirection::DIR_0));
  AstarNode node_high(MapIndex(2, 0));
  node_high.Update(nullptr, kTestCost + 2, 0, true, static_cast<int32_t>(NodeDirection::DIR_0));
  queue_->Push(&node_high);
  queue_->Push(&node_middle);
  queue_->Push(&node_low);
  const AstarNode* popped1 = queue_->Pop();
  const AstarNode* popped2 = queue_->Pop();
  const AstarNode* popped3 = queue_->Pop();
  // Confirm null is returned after it becomes empty
  const AstarNode* popped4 = queue_->Pop();

  // verify
  EXPECT_EQ(&node_low, popped1);
  EXPECT_EQ(&node_middle, popped2);
  EXPECT_EQ(&node_high, popped3);
  EXPECT_EQ(nullptr, popped4);
}

/// AstarQueue test
/// PUSH multiple nodes with the same cost, confirm all can be POPed
TEST_F(AstarQueueTest, SamePriority) {
  // exercise
  queue_->Initialize(kMaxCost);
  AstarNode node1(MapIndex(0, 0));
  node1.Update(nullptr, kTestCost, 0, true, static_cast<int32_t>(NodeDirection::DIR_0));
  AstarNode node2(MapIndex(1, 0));
  node2.Update(nullptr, kTestCost, 0, true, static_cast<int32_t>(NodeDirection::DIR_0));
  AstarNode node3(MapIndex(2, 0));
  node3.Update(nullptr, kTestCost, 0, true, static_cast<int32_t>(NodeDirection::DIR_0));
  queue_->Push(&node1);
  queue_->Push(&node2);
  queue_->Push(&node3);
  const AstarNode* popped1 = queue_->Pop();
  const AstarNode* popped2 = queue_->Pop();
  const AstarNode* popped3 = queue_->Pop();
  const AstarNode* popped4 = queue_->Pop();

  // verify
  // In the current implementation, nodes with the same priority are POPed in FILO order
  EXPECT_EQ(&node3, popped1);
  EXPECT_EQ(&node2, popped2);
  EXPECT_EQ(&node1, popped3);
  EXPECT_EQ(nullptr, popped4);
  // Confirm the list link of the POPed node is cleared
  EXPECT_EQ(nullptr, popped1->prev());
  EXPECT_EQ(nullptr, popped1->next());
  EXPECT_EQ(-1, popped1->queue_index());
  EXPECT_EQ(nullptr, popped2->prev());
  EXPECT_EQ(nullptr, popped2->next());
  EXPECT_EQ(-1, popped2->queue_index());
  EXPECT_EQ(nullptr, popped3->prev());
  EXPECT_EQ(nullptr, popped3->next());
  EXPECT_EQ(-1, popped3->queue_index());
}

/// AstarQueue test
/// When a node at the head of the same priority list is re-PUSHed with a different cost value, confirm it is removed from the original position and inserted at the new position
TEST_F(AstarQueueTest, RemoveHead) {
  // exercise
  queue_->Initialize(kMaxCost);
  AstarNode node1(MapIndex(0, 0));
  node1.Update(nullptr, kTestCost, 0, true, static_cast<int32_t>(NodeDirection::DIR_0));
  AstarNode node2(MapIndex(1, 0));
  node2.Update(nullptr, kTestCost, 0, true, static_cast<int32_t>(NodeDirection::DIR_0));
  AstarNode node3(MapIndex(2, 0));
  node3.Update(nullptr, kTestCost, 0, true, static_cast<int32_t>(NodeDirection::DIR_0));
  queue_->Push(&node1);
  queue_->Push(&node2);
  queue_->Push(&node3);
  // Re-PUSH node3 at the head of the list with a lower priority
  node3.Update(nullptr, kTestCost + 1, 0, true, static_cast<int32_t>(NodeDirection::DIR_0));
  queue_->Push(&node3);
  const AstarNode* popped1 = queue_->Pop();
  const AstarNode* popped2 = queue_->Pop();
  const AstarNode* popped3 = queue_->Pop();
  const AstarNode* popped4 = queue_->Pop();

  // verify
  EXPECT_EQ(&node2, popped1);
  EXPECT_EQ(&node1, popped2);
  EXPECT_EQ(&node3, popped3);
  EXPECT_EQ(nullptr, popped4);
}

/// AstarQueue test
/// When a node in the middle of the same priority list is re-PUSHed with a different cost value, confirm it is removed from the original position and inserted at the new position
TEST_F(AstarQueueTest, RemoveMiddle) {
  // exercise
  queue_->Initialize(kMaxCost);
  AstarNode node1(MapIndex(0, 0));
  node1.Update(nullptr, kTestCost, 0, true, static_cast<int32_t>(NodeDirection::DIR_0));
  AstarNode node2(MapIndex(1, 0));
  node2.Update(nullptr, kTestCost, 0, true, static_cast<int32_t>(NodeDirection::DIR_0));
  AstarNode node3(MapIndex(2, 0));
  node3.Update(nullptr, kTestCost, 0, true, static_cast<int32_t>(NodeDirection::DIR_0));
  queue_->Push(&node1);
  queue_->Push(&node2);
  queue_->Push(&node3);
  // Re-PUSH node2 in the middle of the list with a higher priority
  node2.Update(nullptr, kTestCost - 1, 0, true, static_cast<int32_t>(NodeDirection::DIR_0));
  queue_->Push(&node2);
  const AstarNode* popped1 = queue_->Pop();
  const AstarNode* popped2 = queue_->Pop();
  const AstarNode* popped3 = queue_->Pop();
  const AstarNode* popped4 = queue_->Pop();

  // verify
  EXPECT_EQ(&node2, popped1);
  EXPECT_EQ(&node3, popped2);
  EXPECT_EQ(&node1, popped3);
  EXPECT_EQ(nullptr, popped4);
}

/// AstarQueue test
/// When a node at the end of the same priority list is re-PUSHed with a different cost value, confirm it is removed from the original position and inserted at the new position
TEST_F(AstarQueueTest, RemoveTail) {
  // exercise
  queue_->Initialize(kMaxCost);
  AstarNode node1(MapIndex(0, 0));
  node1.Update(nullptr, kTestCost, 0, true, static_cast<int32_t>(NodeDirection::DIR_0));
  AstarNode node2(MapIndex(1, 0));
  node2.Update(nullptr, kTestCost, 0, true, static_cast<int32_t>(NodeDirection::DIR_0));
  AstarNode node3(MapIndex(2, 0));
  node3.Update(nullptr, kTestCost, 0, true, static_cast<int32_t>(NodeDirection::DIR_0));
  queue_->Push(&node1);
  queue_->Push(&node2);
  queue_->Push(&node3);
  // Re-PUSH node1 at the end of the list with a higher priority
  node1.Update(nullptr, kTestCost - 1, 0, true, static_cast<int32_t>(NodeDirection::DIR_0));
  queue_->Push(&node1);
  const AstarNode* popped1 = queue_->Pop();
  const AstarNode* popped2 = queue_->Pop();
  const AstarNode* popped3 = queue_->Pop();
  const AstarNode* popped4 = queue_->Pop();

  // verify
  EXPECT_EQ(&node1, popped1);
  EXPECT_EQ(&node3, popped2);
  EXPECT_EQ(&node2, popped3);
  EXPECT_EQ(nullptr, popped4);
}

/// AstarQueue test
/// When all nodes in the same priority list are re-PUSHed with different cost values, confirm they are removed from the original positions and inserted at the new positions
TEST_F(AstarQueueTest, RemoveLast) {
  // exercise
  queue_->Initialize(kMaxCost);
  AstarNode node1(MapIndex(0, 0));
  node1.Update(nullptr, kTestCost, 0, true, static_cast<int32_t>(NodeDirection::DIR_0));
  AstarNode node2(MapIndex(1, 0));
  node2.Update(nullptr, kTestCost, 0, true, static_cast<int32_t>(NodeDirection::DIR_0));
  AstarNode node3(MapIndex(2, 0));
  node3.Update(nullptr, kTestCost, 0, true, static_cast<int32_t>(NodeDirection::DIR_0));
  queue_->Push(&node1);
  queue_->Push(&node2);
  queue_->Push(&node3);
  // Re-PUSH all nodes with different priorities
  node1.Update(nullptr, kTestCost - 2, 0, true, static_cast<int32_t>(NodeDirection::DIR_0));
  queue_->Push(&node1);
  node2.Update(nullptr, kTestCost - 1, 0, true, static_cast<int32_t>(NodeDirection::DIR_0));
  queue_->Push(&node2);
  node3.Update(nullptr, kTestCost + 1, 0, true, static_cast<int32_t>(NodeDirection::DIR_0));
  queue_->Push(&node3);
  const AstarNode* popped1 = queue_->Pop();
  const AstarNode* popped2 = queue_->Pop();
  const AstarNode* popped3 = queue_->Pop();
  const AstarNode* popped4 = queue_->Pop();

  // verify
  EXPECT_EQ(&node1, popped1);
  EXPECT_EQ(&node2, popped2);
  EXPECT_EQ(&node3, popped3);
  EXPECT_EQ(nullptr, popped4);
}
}  // namespace tmc_astar_lib

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
