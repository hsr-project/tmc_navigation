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
#ifndef TMC_ASTAR_LIB_COMMON_HPP_
#define TMC_ASTAR_LIB_COMMON_HPP_
#include <memory>
#include <vector>

#include <console_bridge/console.h>
#include <tmc_pose_2d_lib/distance_map.hpp>
#include <tmc_pose_2d_lib/pose_2d.hpp>

using tmc_pose_2d_lib::Point2d;
using tmc_pose_2d_lib::Pose2d;
using CostMap = tmc_pose_2d_lib::DistanceMap;
using CostMapPtr = std::shared_ptr<CostMap>;

namespace tmc_astar_lib {
/// Free value of the map
constexpr int32_t kFreeGrid = 1;
/// Wall value of the map
constexpr int32_t kWallValue = 255;

typedef std::vector<Pose2d> PoseSeq;
struct MapIndex {
  MapIndex() {}
  MapIndex(const int32_t in_x, const int32_t in_y) {
    x = in_x;
    y = in_y;
  }
  bool operator==(const MapIndex& rhs) const {
     return (x == rhs.x && y == rhs.y);
  }
  int32_t x;
  int32_t y;
};

struct MapIndexWithCost {
  MapIndexWithCost() {}
  MapIndexWithCost(const MapIndex& in_index, const int32_t in_cost) {
    index = in_index;
    cost = in_cost;
  }
  MapIndex index;
  int32_t cost;
};
}  // namespace tmc_astar_lib

#endif  // TMC_ASTAR_LIB_COMMON_HPP_
