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
#ifndef TMC_ASTAR_LIB_ASTAR_NODE_HPP_
#define TMC_ASTAR_LIB_ASTAR_NODE_HPP_

#include <stdint.h>
#include <limits>
#include "common.hpp"

namespace tmc_astar_lib {
/*
Node data structure class.

In A*, a node stores the state of one grid cell.
It stores the cumulative cost from the start point and the direction of the path on that grid.
*/
class AstarNode {
 public:
  /// Constructor
  explicit AstarNode(const MapIndex& index) :
      parent_(nullptr),
      index_(index),
      is_closed_(false),
      total_cost_(std::numeric_limits<int32_t>::max()),  // Initialize cost to the maximum value to ensure it gets updated on the first calculation
      total_step_(0),
      prev_(nullptr), next_(nullptr), queue_index_(-1),
      additional_info_(0) {}
  /// getter
  AstarNode* parent() const { return parent_; }
  MapIndex index() const { return index_; }
  int32_t total_cost() const { return total_cost_; }
  int32_t total_step() const { return total_step_; }
  int32_t additional_info() const { return additional_info_; }
  AstarNode* prev() const { return prev_; }
  AstarNode* next() const { return next_; }
  int32_t queue_index() const { return queue_index_; }
  bool is_closed() const { return is_closed_; }
  /// setter
  void set_prev(AstarNode* const prev) { prev_ = prev; }
  void set_next(AstarNode* const next) { next_ = next; }

  /// Mark the node as CLOSED
  void Close();
  /// Update the state of the node and mark it as OPEN
  /// @param [I] parent Parent node (the previous node). For the start point, this is nullptr.
  /// @param [I] total_cost The total cost from the start to this node.
  /// @param [I] total_step The total steps from the start to this node (counting horizontal, vertical, and diagonal as 1).
  /// @param [I] open true: Mark this node as OPEN. false: Do nothing (even if the node is OPEN, it will not be marked as CLOSED).
  /// @param [I] additional_info Additional information (optional).
  /// @return None
  void Update(AstarNode* const parent, const int32_t total_cost, const int32_t total_step,
              const bool open, const int32_t additional_info = 0);
  /// List linking
  /// @param [I] queue_index The cost value when the node was added to the queue. Specify -1 if removed.
  /// @param [I] prev_node Pointer to the previous node in the queue. Specify nullptr if it's the head or removed.
  /// @param [I] next_node Pointer to the next node in the queue. Specify nullptr if it's the tail or removed.
  /// @return None
  void Connect(const int32_t queue_index, AstarNode* const prev_node, AstarNode* const next_node);

 private:
  AstarNode* parent_;            // Parent node of this node
  MapIndex index_;               // Index of this node
  bool is_closed_;               // true: Explored, false: Not explored
  int32_t total_cost_;           // Total cost from the start point to this node
  int32_t total_step_;           // Number of steps (grids passed) from the start point to this node
  AstarNode* prev_;              // Previous node in the queue list
  AstarNode* next_;              // Next node in the queue list
  int32_t queue_index_;          // Queue index when the node was added to the queue (cost value)
  int32_t additional_info_;      // Additional information. Set any information specific to the map characteristics here.
};

}  // namespace tmc_astar_lib

#endif  // TMC_ASTAR_LIB_ASTAR_NODE_HPP_
