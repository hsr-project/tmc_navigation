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
#ifndef TMC_BASE_PATH_FOLLOWER_TEST_MOCK_HPP_
#define TMC_BASE_PATH_FOLLOWER_TEST_MOCK_HPP_
#include <memory>
#include <optional>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <tmc_base_path_follower/common.hpp>
#include <tmc_base_path_follower/goal_checker.hpp>
#include <tmc_base_path_follower/nearest_path_point_searcher.hpp>
#include <tmc_base_path_follower/path_transit_velocity_calculator.hpp>
#include <tmc_base_path_follower/velocity_calculator.hpp>

namespace tmc_base_path_follower {

/// Mock of GoalChecker
class MockGoalChecker : public IGoalChecker {
 public:
  using Ptr = std::shared_ptr<MockGoalChecker>;
  MOCK_METHOD0(Initialize,
               void());
  MOCK_METHOD4(CheckGoal,
               void(const PoseSeq& path,
                    const Pose2d& global_pose,
                    bool& is_arrived_goal_area,
                    bool& is_arrived_goal));
};

/// Mock of NearestPathPointSearcher
class MockNearestPathPointSearcher : public INearestPathPointSearcher {
 public:
  using Ptr = std::shared_ptr<MockNearestPathPointSearcher>;
  MOCK_METHOD4(SearchNearestPathPointIndex,
               uint32_t(const PoseSeq& path,
                        const std::vector<double>& left_path_lengths,
                        const Pose2d& global_pose,
                        const std::optional<uint32_t>& prev_index));
};

/// Mock of VelocityCalculator
class MockVelocityCalculator : public IVelocityCalculator {
 public:
  using Ptr = std::shared_ptr<MockVelocityCalculator>;
  MOCK_METHOD8(CalculateVelocity,
               bool(const PathInfo& path_info,
                    const Pose2d& global_pose,
                    const uint32_t current_path_index,
                    const Vector3d& last_velocity,
                    const double time_interval,
                    const bool is_arrived_goal_area,
                    const std::optional<double>& transit_velocity,
                    Vector3d& output_velocity));
};

/// Mock of PathTransitVelocityCalculator
class MockPathTransitVelocityCalculator : public IPathTransitVelocityCalculator {
 public:
  using Ptr = std::shared_ptr<MockPathTransitVelocityCalculator>;
  MOCK_METHOD1(CalculatePathTransitVelocity,
               void(const PathInfo& path_info));
  MOCK_METHOD1(GetPathTransitVelocity,
               double(const uint32_t path_index));
};

}  // namespace tmc_base_path_follower

#endif  // TMC_BASE_PATH_FOLLOWER_TEST_MOCK_HPP_
