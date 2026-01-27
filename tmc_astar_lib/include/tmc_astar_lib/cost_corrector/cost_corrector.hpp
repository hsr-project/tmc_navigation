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
#ifndef TMC_ASTAR_LIB_COST_CORRECTOR_HPP_
#define TMC_ASTAR_LIB_COST_CORRECTOR_HPP_
#include <memory>
#include <vector>

#include "../astar_node_manager.hpp"
#include "../common.hpp"

namespace tmc_astar_lib {

/// Interface for cost correction class
class ICostCorrector {
 public:
  using Ptr = std::shared_ptr<ICostCorrector>;
  /// Parameter structure to pass to the Setup method
  struct SetupParams {
    SetupParams(const int32_t in_map_width,
                const int32_t in_map_height,
                const std::vector<MapIndex>& in_preferred_path_indexes) :
        map_width(in_map_width), map_height(in_map_height),
        preferred_path_indexes(in_preferred_path_indexes) {}
    const int32_t map_width;
    const int32_t map_height;
    const std::vector<MapIndex>& preferred_path_indexes;
  };
  /// Parameter structure to pass to the GetAdditionalCost method
  struct GetAdditionalCostParams {
    GetAdditionalCostParams(
        const AstarNode* const in_prev_node,
        const MapIndex& in_index,
        const int32_t in_direction,
        const int32_t in_static_cost,
        const int32_t in_dynamic_cost,
        const int32_t in_occupancy_threshold) :
        prev_node(in_prev_node), index(in_index), direction(in_direction),
        static_cost(in_static_cost), dynamic_cost(in_dynamic_cost), occupancy_threshold(in_occupancy_threshold) {}
    const AstarNode* const prev_node;
    const MapIndex index;
    const int32_t direction;
    const int32_t static_cost;
    const int32_t dynamic_cost;
    const int32_t occupancy_threshold;
  };
  // Destructor
  virtual ~ICostCorrector() = default;
  // Initialization before route planning
  virtual void Setup(const SetupParams& params) = 0;
  // Get cost correction value
  virtual int32_t GetAdditionalCost(const GetAdditionalCostParams& params) = 0;
};
}  // namespace tmc_astar_lib
#endif  // TMC_ASTAR_LIB_COST_CORRECTOR_HPP_
