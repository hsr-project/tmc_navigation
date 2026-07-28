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
#include <tmc_astar_lib/cost_corrector/preferred_path_cost_corrector.hpp>

namespace tmc_astar_lib {

/// Constructor
PreferredPathCostCorrector::PreferredPathCostCorrector(const Parameter& param) :
  cost_on_preferred_path_(param.cost_on_preferred_path),
  cost_around_preferred_path_(param.cost_around_preferred_path), width_(0), height_(0) {}

/// Destructor
PreferredPathCostCorrector::~PreferredPathCostCorrector() {}

/// Initialization before route planning
void PreferredPathCostCorrector::Setup(const SetupParams& params) {
  // XY offset of surrounding grids (up, down, left, right)
  const int32_t around_offset[4][2] = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};
  // Set negative costs for grids traversed by the recommended route and their adjacent grids
  width_ = params.map_width;
  height_ = params.map_height;
  additional_cost_map_.resize(width_ * height_);
  std::fill(additional_cost_map_.begin(), additional_cost_map_.end(), 0);
  for (const MapIndex& index : params.preferred_path_indexes) {
    if (index.x >= 0 && index.x < width_ &&
        index.y >= 0 && index.y < height_) {
      // Set cost_on_preferred_path for the same grid as the route point, and cost_arround_preferred_path for up, down, left, and right
      const int32_t center_index = index.y * width_ + index.x;
      additional_cost_map_[center_index] = cost_on_preferred_path_;
      for (int32_t i = 0; i < 4; ++i) {
        MapIndex around_map_index = MapIndex(index.x + around_offset[i][0], index.y + around_offset[i][1]);
        if (around_map_index.x >= 0 && around_map_index.x < width_ &&
            around_map_index.y >= 0 && around_map_index.y < height_) {
          const int32_t around_index = around_map_index.y * width_ + around_map_index.x;
          if (additional_cost_map_[around_index] > cost_around_preferred_path_) {
            additional_cost_map_[around_index] = cost_around_preferred_path_;
          }
        }
      }
    }
  }
}

/// Get cost correction value
int32_t PreferredPathCostCorrector::GetAdditionalCost(const GetAdditionalCostParams& params) {
  const MapIndex index = params.index;
  if (index.x >= 0 && index.x < width_ &&
      index.y >= 0 && index.y < height_) {
    // Return the correction cost for the corresponding grid
    const int32_t grid_index = index.y * width_ + index.x;
    return additional_cost_map_[grid_index];
  } else {
    // Return 0 for areas outside the map
    return 0;
  }
}
}  // namespace tmc_astar_lib
