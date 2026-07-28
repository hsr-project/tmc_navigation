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
#include <tmc_astar_lib/cost_corrector/direction_cost_corrector.hpp>

namespace tmc_astar_lib {

/// Constructor
DirectionCostCorrector::DirectionCostCorrector() {
  // Generate a lookup table for correction values corresponding to all input patterns
  for (int32_t anc_dir = 0; anc_dir < kDirectionNum; ++anc_dir) {
    for (int32_t prev_dir = 0; prev_dir < kDirectionNum; ++prev_dir) {
      for (int32_t cur_dir = 0; cur_dir < kDirectionNum; ++cur_dir) {
        // No correction if it includes no direction
        if (anc_dir == static_cast<int32_t>(NodeDirection::DIR_None) ||
            prev_dir == static_cast<int32_t>(NodeDirection::DIR_None) ||
            cur_dir == static_cast<int32_t>(NodeDirection::DIR_None)) {
          correction_table_[anc_dir][prev_dir][cur_dir] = 0;
          continue;
        }
        int32_t cost = 0;
        int32_t anc_offset_x;
        int32_t anc_offset_y;
        GetOffsetFromDirection(static_cast<NodeDirection>(anc_dir), anc_offset_x, anc_offset_y);
        if (anc_offset_x == 0 || anc_offset_y == 0) {
          // Two steps back are horizontal or vertical
          if (anc_dir == prev_dir) {
            // The directions of two steps back and one step back are the same
            if (prev_dir != cur_dir) {
              // This time they are different
              cost = kCostAdjAdjDiag;
            }
          } else {
            // The directions of two steps back and one step back are different
            if (anc_dir == cur_dir) {
              // The directions of two steps back and this time are the same
              cost = kCostAdjDiagAdj;
            } else if (prev_dir == cur_dir) {
              // The directions of one step back and this time are the same
              cost = kCostAdjDiagDiag;
            }
          }
        } else {
          // Two steps back are diagonal
          if ((anc_dir) == (prev_dir)) {
            // The directions of two steps back and one step back are the same
            if (prev_dir != cur_dir) {
              // This time they are different
              cost = kCostDiagDiagAdj;
            }
          } else {
            // The directions of two steps back and one step back are different
            if (anc_dir == cur_dir) {
              // The directions of two steps back and this time are the same
              cost = kCostDiagAdjDiag;
            } else if (prev_dir == cur_dir) {
              // The directions of one step back and this time are the same
              cost = kCostDiagAdjAdj;
            }
          }
        }
        correction_table_[anc_dir][prev_dir][cur_dir] = cost;
      }
    }
  }
}

/// Destructor
DirectionCostCorrector::~DirectionCostCorrector() {}

/// Initialization before path planning
void DirectionCostCorrector::Setup(const SetupParams& params) {}

/// Get cost correction value
int32_t DirectionCostCorrector::GetAdditionalCost(const GetAdditionalCostParams& params) {
  // Since the starting point has no direction, correction begins from the third step onward
  if (params.prev_node->total_step() < 2) {
    return 0;
  }
  const int32_t cur_dir = static_cast<int32_t>(params.direction);
  const int32_t prev_dir = params.prev_node->additional_info();
  const int32_t anc_dir = params.prev_node->parent()->additional_info();
  return correction_table_[anc_dir][prev_dir][cur_dir];
}

}  // namespace tmc_astar_lib
