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
#include <tmc_astar_lib/astar_queue.hpp>

#include <limits>
#include <queue>
#include <vector>

namespace tmc_astar_lib {

/// Initialization
void AstarQueue::Initialize(const int32_t max_cost) {
  // Allow up to max_cost, so +1
  priority_list_.resize(max_cost + 1);
  std::fill(priority_list_.begin(), priority_list_.end(), static_cast<AstarNode*>(nullptr));
  min_cost_in_queue_ = std::numeric_limits<int32_t>::max();
}

/// Insert node into queue
void AstarQueue::Push(AstarNode* const node) {
  const int32_t queue_index = node->total_cost();
  // Remove if the node is already queued
  const int32_t index_when_queued = node->queue_index();
  if (index_when_queued >= 0) {
    if (priority_list_[index_when_queued] == node) {
      // If the target for deletion is at the beginning of the list
      priority_list_[index_when_queued] = node->next();
      if (node->next() != nullptr) {
        node->next()->set_prev(nullptr);
      }
    } else {
      // If the target for deletion is in the middle or at the end of the list
      node->prev()->set_next(node->next());
      if (node->next() != nullptr) {
        node->next()->set_prev(node->prev());
      }
    }
  }
  // Insert at the beginning of the node list for each cost
  AstarNode* const head = priority_list_[queue_index];
  node->Connect(queue_index, nullptr, head);
  if (head != nullptr) {
    head->set_prev(node);
  }
  priority_list_[queue_index] = node;
  // Check for update of the minimum cost in the queue
  if (min_cost_in_queue_ > queue_index) {
    min_cost_in_queue_ = queue_index;
  }
}

// Extract the node with the smallest cost; return nullptr if the queue is empty
AstarNode* AstarQueue::Pop() {
  while (min_cost_in_queue_ < static_cast<int32_t>(priority_list_.size())) {
    if (priority_list_[min_cost_in_queue_] != nullptr) {
      AstarNode* const popped_node = priority_list_[min_cost_in_queue_];
      priority_list_[min_cost_in_queue_] = popped_node->next();
      if (popped_node->next() != nullptr) {
        popped_node->next()->set_prev(nullptr);
      }
      popped_node->Connect(-1, nullptr, nullptr);
      return popped_node;
    }
    min_cost_in_queue_++;
  }
  return nullptr;
}
}  // namespace tmc_astar_lib
