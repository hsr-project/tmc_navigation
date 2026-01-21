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

#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include <tmc_base_path_follower/common.hpp>
#include <tmc_base_path_follower/nearest_path_point_searcher.hpp>

#include "test_utils.hpp"

namespace tmc_base_path_follower {

/// Parameter test
/// Parameters can be generated
TEST(NearestPathPointSearcherParameterTest, ConstructParameter) {
  // setup
  double partial_search_range = 0.1;
  double partial_search_permit_error = 0.2;
  // exercise
  const NearestPathPointSearcher::Parameter param = NearestPathPointSearcher::Parameter(
      partial_search_range, partial_search_permit_error);

  // verify
  EXPECT_EQ(partial_search_range, param.partial_search_range);
  EXPECT_EQ(partial_search_permit_error, param.partial_search_permit_error);
}


/// Parameter test
/// If an invalid value is specified, it is generated with the default value
TEST(NearestPathPointSearcherParameterTest, ConstructWithInvalidParameterMakeDefault) {
  // setup
  double partial_search_range = -0.1;
  double partial_search_permit_error = -0.2;
  // exercise
  const NearestPathPointSearcher::Parameter param = NearestPathPointSearcher::Parameter(
      partial_search_range, partial_search_permit_error);

  // verify
  EXPECT_EQ(kPartialSearchRangeDefault, param.partial_search_range);
  EXPECT_EQ(kPartialSearchPermitErrorDefault, param.partial_search_permit_error);
}

/// NearestPathPointSearcher test fixture
class NearestPathPointSearcherTest : public ::testing::Test {
 public:
  NearestPathPointSearcherTest() : param_(1.0, 0.5) {
    nearest_path_point_searcher_ = std::make_shared<NearestPathPointSearcher>(param_);
    // Connect paths of the same length vertically and horizontally to create a path in the shape of a 4, intersecting at their midpoints
    const PoseSeq vertical_path = CreateLinearPath(Pose2d(-5.0, 0.0, 0.0), Pose2d(5.0, 0.0, 0.0), 0.5);
    const PoseSeq horizontal_path = CreateLinearPath(Pose2d(0.0, -5.0, 0.0), Pose2d(0.0, 5.0, 0.0), 0.5);
    path_ = vertical_path;
    path_.insert(path_.end(), horizontal_path.begin(), horizontal_path.end());
    // Generate remaining distance information for each path point
    left_path_lengths_.resize(path_.size());
    double sum_length = 0.0;
    left_path_lengths_.back() = 0.0;
    for (int i = path_.size() - 2; i >= 0; --i) {
      sum_length += (path_[i + 1].point() - path_[i].point()).norm();
      left_path_lengths_[i] = sum_length;
    }
  }
 protected:
  NearestPathPointSearcher::Ptr nearest_path_point_searcher_;
  PoseSeq path_;
  std::vector<double> left_path_lengths_;
  NearestPathPointSearcher::Parameter param_;
};


/// NearestPathPointSearcherTest test
/// If there is no previous path point information, the closest point is found
TEST_F(NearestPathPointSearcherTest, PrevIndexNoneSearchAll) {
  // setup
  const uint32_t expect_search_index = 1;
  const Pose2d global_pose(
      path_[expect_search_index].x(),
      path_[expect_search_index].y(),
      path_[expect_search_index].theta());
  // exercise
  const uint32_t search_index = nearest_path_point_searcher_->SearchNearestPathPointIndex(
      path_, left_path_lengths_, global_pose, std::nullopt);
  // verify
  EXPECT_EQ(expect_search_index, search_index);
}

/// NearestPathPointSearcherTest test
/// If the closest path point within range from the previous path point is near the self-position, that point is found
TEST_F(NearestPathPointSearcherTest, PartialSearch) {
  // setup
  // Index on the vertical line where the vertical and horizontal lines intersect
  const uint32_t vertical_path_cross_index = static_cast<uint32_t>(path_.size() / 4);

  // The self-position is offset from the intersection towards the horizontal line, but the offset is within the allowable range
  const Pose2d global_pose(
      path_[vertical_path_cross_index].x(),
      path_[vertical_path_cross_index].y() + param_.partial_search_permit_error - kEpsilon,
      path_[vertical_path_cross_index].theta());

  // The previous path point is one before the intersection on the vertical line
  const uint32_t prev_index = vertical_path_cross_index - 1;

  // exercise
  const uint32_t search_index = nearest_path_point_searcher_->SearchNearestPathPointIndex(
      path_, left_path_lengths_, global_pose, prev_index);
  // verify
  EXPECT_EQ(vertical_path_cross_index, search_index);
}

/// NearestPathPointSearcherTest test
/// If the closest point within range from the previous path point is outside the self-position, a full search is conducted and the closest point is found
TEST_F(NearestPathPointSearcherTest, PartialSearchOutOfRangeSearchAll) {
  // setup
  // Index on the vertical line where the vertical and horizontal lines intersect
  const uint32_t vertical_path_cross_index = static_cast<uint32_t>(path_.size() / 4);

  // The self-position is offset from the intersection towards the horizontal line, but the offset is outside the allowable range
  const Pose2d global_pose(
      path_[vertical_path_cross_index].x(),
      path_[vertical_path_cross_index].y() + param_.partial_search_permit_error + kEpsilon,
      path_[vertical_path_cross_index].theta());
  // The previous path point is one before the intersection on the vertical line
  const uint32_t prev_index = vertical_path_cross_index - 1;

  // A full search is conducted, and the expectation is to find the point closest to the self-position
  double min_distance = std::numeric_limits<double>::max();
  uint32_t expect_search_index = 0;
  for (uint32_t i = 0; i < path_.size(); ++i) {
    const double error = (path_[i].point() - global_pose.point()).norm();
    if (error < min_distance) {
      min_distance = error;
      expect_search_index = i;
    }
  }

  // exercise
  const uint32_t search_index = nearest_path_point_searcher_->SearchNearestPathPointIndex(
      path_, left_path_lengths_, global_pose, prev_index);
  // verify
  EXPECT_EQ(expect_search_index, search_index);
}

}  // namespace tmc_base_path_follower

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
