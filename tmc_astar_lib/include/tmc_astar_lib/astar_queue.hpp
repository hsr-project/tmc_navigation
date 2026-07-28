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
#ifndef TMC_ASTAR_LIB_ASTAR_QUEUE_HPP_
#define TMC_ASTAR_LIB_ASTAR_QUEUE_HPP_
#include <memory>
#include <queue>
#include <vector>

#include "astar_node.hpp"
#include "common.hpp"

namespace tmc_astar_lib {

class IAstarQueue {
 public:
  using Ptr = std::shared_ptr<IAstarQueue>;
  virtual ~IAstarQueue() = default;
  virtual void Initialize(const int32_t max_cost) = 0;
  virtual void Push(AstarNode* const node) = 0;
  virtual AstarNode* Pop() = 0;
};

/*
Priority queue for storing nodes.

Nodes stored are popped in ascending order of cost.
The priority within the queue is determined by the cost value at the time of storage, and the pop order does not change even if the cost changes later.
If a node already registered in the queue is pushed again, the old information is automatically removed from the queue.

Queue data structure:
- Prepare a list array for the maximum cost that can be taken during a single path planning (cost = array index).
- Since std::list has significant overhead when frequently allocated and deallocated, a custom implementation of the list is adopted.
- When pushing, add the node to the head of the list corresponding to the index of its cost.
- When popping, retrieve the head node from the list with the smallest non-empty index.
- In other words, when multiple nodes with the same cost are registered, they are retrieved in FILO order.
  This is not an intentionally designed specification but rather due to implementation constraints.
  This specification slightly affects the priority of paths with the same cost but is treated as undefined in the algorithm.
*/
class AstarQueue : public IAstarQueue {
 public:
  /// Initialization
  /// @param [I] max_cost Maximum cost. Nodes with a cost value greater than this cannot be inserted.
  /// @return None
  void Initialize(const int32_t max_cost);
  /// Insert a node into the queue
  void Push(AstarNode* const node);
  /// Retrieve the node with the smallest cost. Returns nullptr if the queue is empty.
  AstarNode* Pop();

 private:
  // Data structure of the queue (array of lists)
  std::vector<AstarNode*> priority_list_;
  // The smallest index of priority_list that may not be empty
  // This is for optimizing the search process during popping, and the index it points to is not necessarily non-empty.
  int32_t min_cost_in_queue_;
};

}  // namespace tmc_astar_lib

#endif  // TMC_ASTAR_LIB_ASTAR_QUEUE_HPP_
