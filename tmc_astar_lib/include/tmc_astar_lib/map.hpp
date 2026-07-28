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
#ifndef TMC_ASTAR_LIB_MAP_HPP_
#define TMC_ASTAR_LIB_MAP_HPP_
#include <memory>
#include <vector>

#include "astar_node_manager.hpp"
#include "astar_queue.hpp"
#include "common.hpp"

namespace tmc_astar_lib {
class IMap {
 public:
  using Ptr = std::shared_ptr<IMap>;
  using ConstPtr = std::shared_ptr<IMap const>;
  virtual ~IMap() = default;
  /// Get indices within the range from coordinates and range
  virtual void PoseToIndexes(const Pose2d& pose, const double range, std::vector<MapIndex>& indexes) const = 0;
  /// Get coordinates from index
  virtual void IndexToPose(const MapIndex& index, Pose2d& pose) const = 0;
  /// Is the specified index passable?
  virtual bool IsPassable(const MapIndex& index) const = 0;
  /// Maximum cost estimation
  virtual int32_t EstimateMaxCost(const Pose2d& start, const Pose2d& goal) const = 0;
  /// Get adjacent nodes that can be moved to from the specified node
  virtual void GetNextNodes(AstarQueue::Ptr& queue, IAstarNodeManager::Ptr& node_manager, AstarNode* const node,
                            const int32_t max_cost) const = 0;
  // Width of the map
  virtual int32_t width() const = 0;
  // Height of the map
  virtual int32_t height() const = 0;
};
}  // namespace tmc_astar_lib
#endif  // TMC_ASTAR_LIB_MAP_HPP_
