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
#ifndef TMC_ASTAR_LIB_ASTAR_EXECUTER_HPP_
#define TMC_ASTAR_LIB_ASTAR_EXECUTER_HPP_
#include <memory>
#include <vector>

#include <tmc_pose_2d_lib/pose_2d.hpp>
#include "common.hpp"
#include "map.hpp"

namespace tmc_astar_lib {
class IAstarExecuter {
 public:
  using Ptr = std::shared_ptr<IAstarExecuter>;
  virtual ~IAstarExecuter() = default;
  virtual bool ExecuteAstar(
      const IMap::ConstPtr& map,
      const MapIndex& start,
      const MapIndex& goal,
      const int32_t max_cost,
      PoseSeq& path) = 0;

  virtual bool ExecuteAstar(
      const IMap::ConstPtr& map,
      const std::vector<MapIndexWithCost>& start_index_with_costs,
      const std::vector<MapIndexWithCost>& goal_index_with_costs,
      const int32_t max_cost,
      PoseSeq& path) = 0;
};

/// Astar execution class
class AstarExecuter : public IAstarExecuter {
 public:
  /// Constructor
  AstarExecuter(const int32_t width, const int32_t height);
  bool ExecuteAstar(
      const IMap::ConstPtr& map,
      const MapIndex& start,
      const MapIndex& goal,
      const int32_t max_cost,
      PoseSeq& path);

  bool ExecuteAstar(
      const IMap::ConstPtr& map,
      const std::vector<MapIndexWithCost>& start_index_with_costs,
      const std::vector<MapIndexWithCost>& goal_index_with_costs,
      const int32_t max_cost,
      PoseSeq& path);

 private:
  void ConvertToPath(
      const IMap::ConstPtr& map,
      const MapIndex& goal_index, PoseSeq& output_path);

  IAstarNodeManager::Ptr node_manager_;
};
}  // namespace tmc_astar_lib
#endif  // TMC_ASTAR_LIB_ASTAR_EXECUTER_HPP_
