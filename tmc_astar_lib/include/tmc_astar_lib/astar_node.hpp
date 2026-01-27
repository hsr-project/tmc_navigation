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
#ifndef TMC_ASTAR_LIB_ASTAR_NODE_HPP_
#define TMC_ASTAR_LIB_ASTAR_NODE_HPP_

#include <stdint.h>
#include <limits>
#include "common.hpp"

namespace tmc_astar_lib {
/*
Node data structure class.

In A*, a node stores the state of one grid.
Stores cumulative cost from the start point and the direction of the path on that grid.
*/
class AstarNode {
 public:
  /// Constructor
  explicit AstarNode(const MapIndex& index) :
      parent_(nullptr),
      index_(index),
      is_closed_(false),
      total_cost_(std::numeric_limits<int32_t>::max()),  // Initialize cost to maximum value to ensure it is updated on the first run
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

  /// Set the node to CLOSE
  void Close();
  /// Update the node's state and set it to OPEN
  /// @param [I] parent Parent node (one step before) In the case of the start point, it is nullptr
  /// @param [I] total_cost Total cost from the start to this node
  /// @param [I] total_step Total steps from the start to this node (count horizontal, vertical, and diagonal as 1)
  /// @param [I] open true: Set this node to OPEN false: Do nothing (even if it is OPEN, do not set to CLOSE)
  /// @param [I] additional_info Additional information (optional)
  /// @return None
  void Update(AstarNode* const parent, const int32_t total_cost, const int32_t total_step,
              const bool open, const int32_t additional_info = 0);
  /// List linking
  /// @param [I] queue_index Cost value when added to the queue Specify -1 when removed
  /// @param [I] prev_node Pointer to the previous node in the queue Specify nullptr if it is the head or removed
  /// @param [I] next_node Pointer to the next node in the queue Specify nullptr if it is the tail or removed
  /// @return None
  void Connect(const int32_t queue_index, AstarNode* const prev_node, AstarNode* const next_node);

 private:
  AstarNode* parent_;            // Parent node of this node
  MapIndex index_;               // Index of this node
  bool is_closed_;               // This node is true: explored false: unexplored
  int32_t total_cost_;           // Total cost from the start point to this node
  int32_t total_step_;           // Number of steps from the start point to this node (number of grids passed)
  AstarNode* prev_;              // List link in the queue Previous
  AstarNode* next_;              // List link in the queue Next
  int32_t queue_index_;          // Queue index when added to the queue (cost value)
  int32_t additional_info_;      // Additional information. Set any information you want to add to the node according to the characteristics of the map
};

}  // namespace tmc_astar_lib

#endif  // TMC_ASTAR_LIB_ASTAR_NODE_HPP_
