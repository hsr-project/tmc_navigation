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

#include <algorithm>
#include <limits>
#include <angles/angles.h>
#include <gtest/gtest.h>
#include <tmc_base_path_follower/common.hpp>
#include <tmc_base_path_follower/path_info_creator.hpp>

#include "test_utils.hpp"

namespace {
// Input waypoint interval [m]
constexpr double kInputPathInterval = 0.5;
// Radius of the arc path [m]
constexpr double kArcR = 5.0;
// Allowable error ratio
constexpr double kAcceptableErrorRatio = 0.1;
}  // anonymous namespace

namespace tmc_base_path_follower {

/// Parameter test
/// Ability to generate parameters
TEST(PathInfoCreatorParameterTest, ConstructParameter) {
  // setup
  const double interpolation_number = 4.0;
  const double passing_velocity = 0.4;

  // exercise
  const PathInfoCreator::Parameter param = PathInfoCreator::Parameter(
      interpolation_number, passing_velocity);

  // verify
  EXPECT_EQ(interpolation_number, param.interpolation_number);
  EXPECT_EQ(passing_velocity, param.passing_velocity);
}


/// Parameter test
/// Default values are generated when invalid values are specified
TEST(PathInfoCreatorParameterTest, ConstructWithInvalidParameterMakeDefault) {
  // setup
  const double interpolation_number = kMinimumInterpolationNumber - 1;
  const double passing_velocity = -1.0;

  // exercise
  const PathInfoCreator::Parameter param = PathInfoCreator::Parameter(
      interpolation_number, passing_velocity);

  // verify
  EXPECT_EQ(kInterpolationNumberDefault, param.interpolation_number);
  EXPECT_EQ(kMaxLinearVelocityDefault, param.passing_velocity);
}

/// PathInfoCreator test fixture
class PathInfoCreatorTest : public ::testing::Test {
 public:
  PathInfoCreatorTest() {
    creator_ = std::make_shared<PathInfoCreator>(
        PathInfoCreator::Parameter(kInterpolationNumberDefault, kMaxLinearVelocityDefault * 2));
  }

 protected:
  PathInfoCreator::Ptr creator_;
};

/// CreatePathInfo test
/// The number of output waypoints is multiplied by the number of interpolation points
TEST_F(PathInfoCreatorTest, InterpolationNumber) {
  // setup
  PoseSeq input_path;
  for (int32_t i = 0; i < 10; ++i) {
    input_path.push_back(Pose2d(kInputPathInterval * i, 0.0, 0.0));
  }

  // exercise
  PathInfo path_info = creator_->CreatePathInfo(input_path);

  // verify
  // Since interpolation is not performed beyond the goal, the expected result is ([number of points excluding the goal] * multiplier + 1 for the goal point)
  EXPECT_EQ((input_path.size() - 1) * kInterpolationNumberDefault + 1, path_info.splined_path.size());
}

/// CreatePathInfo test
/// Interpolation points are inserted to make the angle changes of waypoints smoother, and the coordinates and angles of the output waypoints become obtuse
TEST_F(PathInfoCreatorTest, InterpolationAngle) {
  // setup
  PoseSeq input_path;
  // Left curve
  CreateArcPath(kArcR, Pose2d(0.0, 0.0, 0.0), kInputPathInterval, M_PI, input_path);
  // Right curve
  CreateArcPath(kArcR, input_path.back(), kInputPathInterval, -M_PI, input_path);

  // The angle between waypoints of the original path is calculated to have a uniform curvature, so it is determined only from the first two points
  const double input_path_angle =
      std::abs(angles::shortest_angular_distance(input_path[0].theta(), input_path[1].theta()));

  // exercise
  PathInfo path_info = creator_->CreatePathInfo(input_path);

  // verify
  for (int32_t i = 1; i < path_info.splined_path.size() - 1; i++) {
    // Obtain the angle of the waypoint from the directions of the preceding and following points
    const double from_direction = atan2(path_info.splined_path[i].y() - path_info.splined_path[i - 1].y(),
                                        path_info.splined_path[i].x() - path_info.splined_path[i - 1].x());
    const double to_direction = atan2(path_info.splined_path[i + 1].y() - path_info.splined_path[i].y(),
                                      path_info.splined_path[i + 1].x() - path_info.splined_path[i].x());
    const double angle = angles::shortest_angular_distance(from_direction, to_direction);
    // Confirm that it is relaxed by 1 divided by the number of interpolation waypoints
    EXPECT_LT(std::abs(angle), input_path_angle / kInterpolationNumberDefault * (1.0 + kAcceptableErrorRatio));
    // Confirm that the direction of the waypoint obtained from the coordinates matches the direction set in the output theta
    const double theta = angles::normalize_angle(from_direction + angle / 2.0);
    EXPECT_LT(std::abs(angles::shortest_angular_distance(theta, path_info.splined_path[i].theta())),
              std::abs(theta * kAcceptableErrorRatio));
  }
  // The angle of the final point is the same as the input path
  EXPECT_FLOAT_EQ(input_path.back().theta(), path_info.splined_path.back().theta());
}

/// CreatePathInfo test
/// The angle of the goal point does not change
TEST_F(PathInfoCreatorTest, KeepGoalTheta) {
  // setup
  PoseSeq input_path;
  for (int32_t i = 0; i < 10; ++i) {
    input_path.push_back(Pose2d(kInputPathInterval * i, 0.0, 0.0));
  }
  // Reverse only the goal
  input_path.back().set_theta(M_PI);

  // exercise
  PathInfo path_info = creator_->CreatePathInfo(input_path);

  // verify
  EXPECT_DOUBLE_EQ(input_path.back().theta(), path_info.splined_path.back().theta());
}

/// CreatePathInfo test
/// Points duplicated in the input path are ignored
TEST_F(PathInfoCreatorTest, IgnoreDoublePoint) {
  // setup
  // Generate a 5-point path with one duplicate point
  PoseSeq input_path;
  input_path.push_back(Pose2d(0.0, 0.0, 0.0));
  input_path.push_back(Pose2d(input_path.back().x() + kInputPathInterval, input_path.back().y(), 0.0));
  input_path.push_back(Pose2d(input_path.back().x(), input_path.back().y(), 0.0));
  input_path.push_back(Pose2d(input_path.back().x() + kInputPathInterval, input_path.back().y(), 0.0));
  input_path.push_back(Pose2d(input_path.back().x() + kInputPathInterval, input_path.back().y(), 0.0));

  // exercise
  PathInfo path_info = creator_->CreatePathInfo(input_path);

  // verify
  // Confirm that the number of waypoints matches the case where a 4-point path without duplicates is processed
  EXPECT_EQ((input_path.size() - 2) * kInterpolationNumberDefault + 1, path_info.splined_path.size());
}

/// CreatePathInfo test
/// Points included in the input path are included in the output with the same coordinates
TEST_F(PathInfoCreatorTest, KeepOriginalPoints) {
  // setup
  PoseSeq input_path;
  // Left curve
  CreateArcPath(kArcR, Pose2d(0.0, 0.0, 0.0), kInputPathInterval, M_PI, input_path);

  // exercise
  PathInfo path_info = creator_->CreatePathInfo(input_path);

  // verify
  // Confirm that all waypoints of the input path are included in the output path
  for (const Pose2d& in_point : input_path) {
    bool match = false;
    for (const Pose2d& out_point : path_info.splined_path) {
      if (in_point == out_point) {
        match = true;
        break;
      }
    }
    EXPECT_TRUE(match);
  }
}

/// CreatePathInfo test
/// When input that causes spline extrema is given, linear interpolation is used
TEST_F(PathInfoCreatorTest, AvoidExtremum) {
  // setup
  PoseSeq input_path;
  // Input a straight path containing a point facing backward (135° to avoid being exactly opposite at 180°)
  input_path.push_back(Pose2d(0.0, 0.0, 0.0));
  input_path.push_back(Pose2d(kInputPathInterval, 0.0, M_PI * 3.0 / 4.0));
  input_path.push_back(Pose2d(kInputPathInterval * 2.0, 0.0, 0.0));

  // exercise
  PathInfo path_info = creator_->CreatePathInfo(input_path);

  // verify
  // Confirm that the output path is a straight line and aligned in the direction from start to goal
  for (int32_t i = 1; i < path_info.splined_path.size(); ++i) {
    EXPECT_DOUBLE_EQ(0.0, path_info.splined_path[i].y());
    EXPECT_GT(path_info.splined_path[i].x(), path_info.splined_path[i - 1].x());
  }
}

/// CreatePathInfo test
/// The curvature of the straight section becomes 0
TEST_F(PathInfoCreatorTest, CurvatureStraight) {
  PoseSeq input_path;
  for (int32_t i = 0; i < 10; ++i) {
    input_path.push_back(Pose2d(kInputPathInterval * i, 0.0, 0.0));
  }

  // exercise
  PathInfo path_info = creator_->CreatePathInfo(input_path);

  // verify
  for (int32_t i = 1; i < path_info.splined_path.size() - 1; ++i) {
    EXPECT_DOUBLE_EQ(0.0, path_info.splined_path_curvatures[i]);
  }
}

/// CreatePathInfo test
/// The curvature of the left curve matches the theoretical value [1/R]
TEST_F(PathInfoCreatorTest, CurvatureLeftCurve) {
  // setup
  PoseSeq input_path;
  // Left curve
  CreateArcPath(kArcR, Pose2d(0.0, 0.0, 0.0), kInputPathInterval, M_PI, input_path);
  // Theoretical curvature value [1/R]
  const double expected_r = 1.0 / kArcR;

  // exercise
  PathInfo path_info = creator_->CreatePathInfo(input_path);

  // verify
  // Confirm that the curvature of waypoints excluding the start and goal matches the theoretical value
  for (int32_t i = 1; i < path_info.splined_path.size() - 1; ++i) {
    EXPECT_NEAR(expected_r, path_info.splined_path_curvatures[i], std::abs(expected_r * kAcceptableErrorRatio));
  }
}

/// CreatePathInfo test
/// The curvature of the right curve matches the theoretical value [-1/R]
TEST_F(PathInfoCreatorTest, CurvatureRightCurve) {
  // setup
  PoseSeq input_path;
  // Right curve
  CreateArcPath(kArcR, Pose2d(0.0, 0.0, 0.0), kInputPathInterval, -M_PI, input_path);
  // Theoretical curvature value [-1/R]
  const double expected_r = -1.0 / kArcR;

  // exercise
  PathInfo path_info = creator_->CreatePathInfo(input_path);

  // verify
  // Confirm that the curvature of waypoints excluding the start and goal matches the theoretical value
  for (int32_t i = 1; i < path_info.splined_path.size() - 1; ++i) {
    EXPECT_NEAR(expected_r, path_info.splined_path_curvatures[i], std::abs(expected_r * kAcceptableErrorRatio));
  }
}

/// CreatePathInfo test
/// Distances from each point after spline interpolation to the goal are generated and monotonically decrease towards the goal
TEST_F(PathInfoCreatorTest, LeftLength) {
  // setup
  PoseSeq input_path;
  // Left curve
  CreateArcPath(kArcR, Pose2d(0.0, 0.0, 0.0), kInputPathInterval, M_PI, input_path);
  // Theoretical path length (semicircle with radius kArcR)
  const double whole_length = kArcR * M_PI;

  // exercise
  PathInfo path_info = creator_->CreatePathInfo(input_path);

  // verify
  // Confirm that the remaining distance at the start point approximately matches the theoretical path length
  EXPECT_NEAR(whole_length, path_info.splined_path_left_lengths.front(), whole_length * kAcceptableErrorRatio);
  // Confirm that it is monotonically decreasing
  for (int32_t i = 1; i < path_info.splined_path.size(); ++i) {
    EXPECT_LT(path_info.splined_path_left_lengths[i], path_info.splined_path_left_lengths[i - 1]);
  }
  // Confirm that the remaining distance at the goal point is 0
  EXPECT_DOUBLE_EQ(0.0, path_info.splined_path_left_lengths.back());
}
}  // namespace tmc_base_path_follower

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
