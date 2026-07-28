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
#ifndef TMC_BASE_PATH_PLANNER_TEST_MOCK_HPP_
#define TMC_BASE_PATH_PLANNER_TEST_MOCK_HPP_
#include <memory>
#include <optional>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <tmc_base_path_planner/common.hpp>
#include <tmc_base_path_planner/condition_checker.hpp>
#include <tmc_base_path_planner/map_filter.hpp>
#include <tmc_base_path_planner/path_planner_core.hpp>
#include <tmc_base_path_planner/path_smoother.hpp>
#include <tmc_base_path_planner/path_updater.hpp>

namespace tmc_base_path_planner {

/// Mock of IConditionChecker
class MockConditionChecker : public IConditionChecker {
 public:
  using Ptr = std::shared_ptr<MockConditionChecker>;
  MOCK_METHOD7(CheckCondition,
               BasePathPlannerErrorCode(const CostMapPtr& static_map,
                                        const unsigned char static_map_occupancy_threshold,
                                        const CostMapPtr& dynamic_map,
                                        const Pose2d& dynamic_map_origin,
                                        const Pose2d& start_pose,
                                        const Pose2d& goal_pose,
                                        const Pose2d& global_pose));
};

/// Mock of IMapFilter
class MockMapFilter : public IMapFilter {
 public:
  using Ptr = std::shared_ptr<MockMapFilter>;
  MOCK_METHOD5(FilterMapOnStartAndGoal,
               void(CostMapPtr& map,
                    const Pose2d& map_origin,
                    const Pose2d& start_pose,
                    const Pose2d& goal_pose,
                    const Pose2d& global_pose));
};

/// Mock of IPathPlannerCore
class MockPathPlannerCore : public IPathPlannerCore {
 public:
  using Ptr = std::shared_ptr<MockPathPlannerCore>;
  MOCK_METHOD7(PlanPath,
               bool(const Pose2d& start,
                    const Pose2d& goal,
                    const CostMapPtr& dynamic_map,
                    const Pose2d& dynamic_map_origin,
                    const bool enable_adaptive_start_positioning,
                    const PoseSeq& preferred_path,
                    PoseSeq& output_path));

  MOCK_METHOD0(static_map_occupancy_threshold, unsigned char());
};

/// Mock of IPathUpdater
class MockPathUpdater : public IPathUpdater {
 public:
  using Ptr = std::shared_ptr<MockPathUpdater>;
  MOCK_METHOD0(ClearPrevPath,
               void());

  MOCK_METHOD2(SearchStartPoseOnPrevPath,
               std::optional<uint32_t>(const Pose2d& global_pose,
                                         Pose2d& start_pose));
  MOCK_METHOD3(UpdatePath,
               bool(const PoseSeq& path,
                    const std::optional<uint32_t>& start_index_on_prev_path,
                    PoseSeq& update_path));
};

/// Mock of IPathSmoother
class MockPathSmoother : public IPathSmoother {
 public:
  using Ptr = std::shared_ptr<MockPathSmoother>;
  MOCK_METHOD2(SmoothingPath,
               bool(const PoseSeq& input_path,
                    PoseSeq& output_path));
};

}  // namespace tmc_base_path_planner

#endif  // TMC_BASE_PATH_PLANNER_TEST_MOCK_HPP_
