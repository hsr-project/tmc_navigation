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
#include <tmc_base_path_planner/path_smoother.hpp>

#include <memory>
#include <gtest/gtest.h>

namespace {
// Map resolution
const double kResolution = 0.05;
// Test path length
const int32_t kTestPathLength = 40;
}  // anonymous namespace

namespace tmc_base_path_planner {

/// PathSmoother test fixture
class PathSmootherTest : public ::testing::Test {
 public:
  PathSmootherTest() {}

 protected:
  virtual void SetUp() {
    smoother_ = std::make_shared<PathSmoother>(PathSmoother());
  }

  PathSmoother::Ptr smoother_;
};

/// PathSmoother test
/// The start, goal, and number of waypoints are the same as the input, and a smoothed path is returned
TEST_F(PathSmootherTest, NormalCase) {
  // exercise
  PoseSeq input;
  // Generate a path with zigzags of one grid width
  for (int32_t i = 0; i < kTestPathLength / 2; ++i) {
    // Set dummy values for orientation
    input.push_back(Pose2d(static_cast<double>(i) * kResolution, 0.0, 0.123));
    input.push_back(Pose2d(static_cast<double>(i) * kResolution, kResolution, 0.123));
  }
  PoseSeq output;
  smoother_->SmoothingPath(input, output);

  // verify
  // The number of waypoints is the same
  ASSERT_EQ(input.size(), output.size());
  // The start coordinates are the same
  EXPECT_DOUBLE_EQ(input.front().x(), output.front().x());
  EXPECT_DOUBLE_EQ(input.front().y(), output.front().y());
  // The start point faces the direction of the next point
  const double expect_start_theta = atan2(output[1].y() - output[0].y(), output[1].x() - output[0].x());
  EXPECT_DOUBLE_EQ(expect_start_theta, output[0].theta());
  // Points other than the start and goal face the direction of progress from the previous point
  for (size_t i = 1; i < output.size() - 1; ++i) {
    const double expect_theta = atan2(output[i].y() - output[i - 1].y(), output[i].x() - output[i - 1].x());
    EXPECT_DOUBLE_EQ(expect_theta, output[i].theta());
  }
  // The goal coordinates and direction are the same
  EXPECT_DOUBLE_EQ(input.back().x(), output.back().x());
  EXPECT_DOUBLE_EQ(input.back().y(), output.back().y());
  EXPECT_DOUBLE_EQ(input.back().theta(), output.back().theta());
  // The zigzag width is reduced
  // Since the start and goal are not subject to correction, check the others
  double min_y = output[1].y();
  double max_y = output[1].y();
  for (size_t i = 1; i < output.size() - 1; ++i) {
    if (min_y > output[i].y()) {
      min_y = output[i].y();
    }
    if (max_y < output[i].y()) {
      max_y = output[i].y();
    }
  }
  EXPECT_LT(max_y - min_y, kResolution);
}

/// PathSmoother test
/// In the case of a two-point path, the original start and goal are returned as is
TEST_F(PathSmootherTest, TwoPointsPath) {
  // exercise
  PoseSeq input;
  // Set two points with arbitrary values
  input.push_back(Pose2d(1.234, 2.345, 0.345));
  input.push_back(Pose2d(4.567, 5.678, 0.789));
  PoseSeq output;
  smoother_->SmoothingPath(input, output);

  // verify
  ASSERT_EQ(2, output.size());
  EXPECT_DOUBLE_EQ(input[0].x(), output[0].x());
  EXPECT_DOUBLE_EQ(input[0].y(), output[0].y());
  const double expect_start_theta = atan2(output[1].y() - output[0].y(), output[1].x() - output[0].x());
  EXPECT_DOUBLE_EQ(expect_start_theta, output[0].theta());
  EXPECT_DOUBLE_EQ(input[1].x(), output[1].x());
  EXPECT_DOUBLE_EQ(input[1].y(), output[1].y());
  EXPECT_DOUBLE_EQ(input[1].theta(), output[1].theta());
}

}  // namespace tmc_base_path_planner

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
