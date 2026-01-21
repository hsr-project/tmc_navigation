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
#ifndef TMC_ASTAR_LIB_NODE_DIRECTION_HPP_
#define TMC_ASTAR_LIB_NODE_DIRECTION_HPP_

#include <stdint.h>

namespace tmc_astar_lib {

/// Definition of direction in grid map
enum class NodeDirection {
  DIR_None = 0,  // No direction defined (starting point)
  DIR_0,         // 0 degrees direction based on X-axis
  DIR_45,        // 45 degrees direction based on X-axis
  DIR_90,        // 90 degrees direction based on X-axis
  DIR_135,       // 135 degrees direction based on X-axis
  DIR_180,       // 180 degrees direction based on X-axis
  DIR_225,       // 225 degrees direction based on X-axis
  DIR_270,       // 270 degrees direction based on X-axis
  DIR_315,       // 315 degrees direction based on X-axis
  DIR_Max
};
}  // namespace tmc_astar_lib

namespace { // NOLINT
/// Get X, Y offset from NodeDirection
/// @param [I] direction Direction constant
/// @param [O] x Offset in X direction
/// @param [O] y Offset in Y direction
/// @return None
void GetOffsetFromDirection(const tmc_astar_lib::NodeDirection direction, int32_t& x, int32_t& y) {
  const int32_t direction_table[static_cast<int32_t>(tmc_astar_lib::NodeDirection::DIR_Max)][2] = {
    {0, 0}, {1, 0}, {1, 1}, {0, 1}, {-1, 1}, {-1, 0}, {-1, -1}, {0, -1}, {1, -1}
  };
  x = direction_table[static_cast<int32_t>(direction)][0];
  y = direction_table[static_cast<int32_t>(direction)][1];
}
}  // anonymous namespace

#endif  // TMC_ASTAR_LIB_NODE_DIRECTION_HPP
