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
#ifndef TMC_ASTAR_LIB_PREFERRED_PATH_COST_CORRECTOR_HPP_
#define TMC_ASTAR_LIB_PREFERRED_PATH_COST_CORRECTOR_HPP_

#include <vector>
#include <console_bridge/console.h>
#include "cost_corrector.hpp"

/*
Priority Path Cost Correction

Overview:
Lower the cost when overlapping with a pre-specified path.

Background:
Among the paths from start to goal, there can be multiple paths with the minimum cost,
and which one is chosen is undefined by the algorithm and changes with the processing progress.
Therefore, even a slight change in the situation can significantly alter the generated path.

Objective:
Provide the path you want to prioritize (e.g., the previous path),
and apply negative cost correction to the matching sections to minimize path fluctuation.
*/

namespace tmc_astar_lib {

/// Parameter Default Values
static constexpr int32_t kCostOnPreferredPathDefault = 0;      // Cost correction value on the recommended path grid
static constexpr int32_t kCostAroundPreferredPathDefault = 0;  // Cost correction value around the recommended path grid

class PreferredPathCostCorrector : public ICostCorrector {
 public:
  /// Parameters
  struct Parameter {
    Parameter(const int32_t in_cost_on_preferred_path, const int32_t in_cost_around_preferred_path) :
        cost_on_preferred_path(in_cost_on_preferred_path),
        cost_around_preferred_path(in_cost_around_preferred_path) {
      if (cost_on_preferred_path > 0) {
        CONSOLE_BRIDGE_logWarn("Value of 'cost_on_preferred_path' is invalid. Use default value.");
        cost_on_preferred_path = kCostOnPreferredPathDefault;
      }
      if (cost_around_preferred_path > 0) {
        CONSOLE_BRIDGE_logWarn("Value of 'cost_around_preferred_path' is invalid. Use default value.");
        cost_around_preferred_path = kCostAroundPreferredPathDefault;
      }
      if (cost_around_preferred_path < cost_on_preferred_path) {
        CONSOLE_BRIDGE_logWarn("Value of 'cost_around_preferred_path' must be greater than or equal to" \
          "'cost_on_preferred_path'. Use default values.");
        cost_on_preferred_path = kCostOnPreferredPathDefault;
        cost_around_preferred_path = kCostAroundPreferredPathDefault;
      }
    }
    // Correction cost on the priority path (negative)
    int32_t cost_on_preferred_path;
    // Correction cost around the priority path (negative)
    int32_t cost_around_preferred_path;
  };
  explicit PreferredPathCostCorrector(const Parameter& param);
  virtual ~PreferredPathCostCorrector();
  virtual void Setup(const SetupParams& params);
  virtual int32_t GetAdditionalCost(const GetAdditionalCostParams& params);

 private:
  // Parameter Values
  const int32_t cost_on_preferred_path_;
  const int32_t cost_around_preferred_path_;
  // Map Size
  int32_t width_;
  int32_t height_;
  // Expanded correction cost on the map
  std::vector<int32_t> additional_cost_map_;
};
}  // namespace tmc_astar_lib
#endif  // TMC_ASTAR_LIB_PREFERRED_PATH_COST_CORRECTOR_HPP_
