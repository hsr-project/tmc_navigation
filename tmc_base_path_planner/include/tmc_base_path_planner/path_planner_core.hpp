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
#ifndef TMC_BASE_PATH_PLANNER_PATH_PLANNER_CORE_HPP_
#define TMC_BASE_PATH_PLANNER_PATH_PLANNER_CORE_HPP_
#include <memory>

#include "common.hpp"

namespace tmc_base_path_planner {

/// Interface for a class that generates a path connecting the start and goal
class IPathPlannerCore {
 public:
  using Ptr = std::shared_ptr<IPathPlannerCore>;
  /// Destructor
  virtual ~IPathPlannerCore() = default;
  /// Execute path planning
  /// @param [I] start Start coordinates (relative to static map)
  /// @param [I] goal Goal coordinates (relative to static map)
  /// @param [I] dynamic_map Dynamic map
  /// @param [I] dynamic_map_origin Dynamic map origin (relative to static map)
  /// @param [I] enable_adaptive_start_positioning Enable/disable adaptive start positioning
  /// @param [I] preferred_path Preferred path
  /// @param [O] output_path Generated path
  /// @return true: success false: failure
  virtual bool PlanPath(const Pose2d& start, const Pose2d& goal,
                        const CostMapPtr& dynamic_map, const Pose2d& dynamic_map_origin,
                        const bool enable_adaptive_start_positioning,
                        const PoseSeq& preferred_path,
                        PoseSeq& output_path) = 0;
  virtual uint8_t static_map_occupancy_threshold() = 0;
};

}  // namespace tmc_base_path_planner

#endif  // TMC_BASE_PATH_PLANNER_PATH_PLANNER_CORE_HPP_
