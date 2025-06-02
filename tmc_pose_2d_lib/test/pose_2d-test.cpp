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
#include <algorithm>
#include <string>
#include <vector>
#include <gtest/gtest.h>
#include "tmc_pose_2d_lib/pose_2d.hpp"

namespace tmc_pose_2d_lib {

// Point2d test
TEST(Pose2dTest, NormalizeAngleTest) {
  const double angle = 0.1;
  const double very_small_val = 0.000000001;
  ASSERT_NEAR(NormalizeAngle(2.0 * M_PI + angle), angle, very_small_val);
  ASSERT_NEAR(NormalizeAngle(-2.0 * M_PI + angle), angle, very_small_val);
}

// Point2d test
TEST(Pose2dTest, PointTest) {
  const Point2d p1(1.0, 2.0);
  const Point2d p2(3.0, 4.0);
  Point2d p3;

  // setter, getter
  p3.set_x(5.0);
  p3.set_y(6.0);
  ASSERT_DOUBLE_EQ(p3.x(), 5.0);
  ASSERT_DOUBLE_EQ(p3.y(), 6.0);

  // Addition
  p3 = p1 + p2;
  ASSERT_DOUBLE_EQ(p3.x(), 4.0);
  ASSERT_DOUBLE_EQ(p3.y(), 6.0);

  // Subtraction
  p3 = p1 - p2;
  ASSERT_DOUBLE_EQ(p3.x(), -2.0);
  ASSERT_DOUBLE_EQ(p3.y(), -2.0);

  // Multiplication
  p3 = p1 * 3.0;
  ASSERT_DOUBLE_EQ(p3.x(), 3.0);
  ASSERT_DOUBLE_EQ(p3.y(), 6.0);

  p3 = 3.0 * p1;
  ASSERT_DOUBLE_EQ(p3.x(), 3.0);
  ASSERT_DOUBLE_EQ(p3.y(), 6.0);

  p3 = p1;
  p3 *= 3.0;
  ASSERT_DOUBLE_EQ(p3.x(), 3.0);
  ASSERT_DOUBLE_EQ(p3.y(), 6.0);

  // Distance
  ASSERT_DOUBLE_EQ(p1.norm(), sqrt(1.0*1.0 + 2.0*2.0));
}

// Rotation2d test
TEST(Pose2dTest, RotationTest) {
  const double theta01 = M_PI / 10.0;
  const double theta02 = M_PI / 20.0;

  const Rotation2d r01(theta01);  // From coordinate system 0 to coordinate system 1 (Rotation transformation from coordinate system 1 to coordinate system 0)
  const Rotation2d r02(theta02);  // From coordinate system 0 to coordinate system 2 (Rotation transformation from coordinate system 2 to coordinate system 0)
  Rotation2d r3;

  // setter, getter
  r3.set_theta(theta01);
  ASSERT_DOUBLE_EQ(r3.theta(), theta01);

  // Multiplication
  r3 = r01 * r02;
  ASSERT_DOUBLE_EQ(r3.theta(), theta01 + theta02);

  // Point coordinate transformation
  Point2d p0;  // Coordinates of point p in coordinate system 0
  Point2d p1;  // Coordinates of point p in coordinate system 1.
  Point2d p2;  // Coordinates of point p in coordinate system 2. p0, p1, p2 refer to the same point viewed from different coordinate systems

  p0 = Point2d(1.0, 0.0);
  p1 = r01.Inverse() * p0;
  ASSERT_DOUBLE_EQ(p1.x(),  cos(theta01));
  ASSERT_DOUBLE_EQ(p1.y(), -sin(theta01));

  p0 = Point2d(0.0, 1.0);
  p1 = r01.Inverse() * p0;
  ASSERT_DOUBLE_EQ(p1.x(), sin(theta01));
  ASSERT_DOUBLE_EQ(p1.y(), cos(theta01));
}


// Pose2d test
TEST(Pose2dTest, PoseTest) {
  const double x01 = 1.0;
  const double y01 = 1.0;
  const double theta01 = M_PI / 10.0;
  // From coordinate system 0 to coordinate system 1 (Rigid body transformation from coordinate system 1 to coordinate system 0)
  const Pose2d pose01(x01, y01, theta01);

  // getter, setter
  ASSERT_DOUBLE_EQ(pose01.x(),  x01);
  ASSERT_DOUBLE_EQ(pose01.y(),  y01);
  ASSERT_DOUBLE_EQ(pose01.theta(),  theta01);


  const double x12 = 1.0;
  const double y12 = 1.0;
  const double theta12 = M_PI / 10.0;
  const Pose2d pose12(x12, y12, theta12);

  // Composite transformation
  Pose2d pose02 = pose01 * pose12;
  ASSERT_DOUBLE_EQ(pose02.x(),  x12 * cos(theta01) - y12 * sin(theta01) + x01);
  ASSERT_DOUBLE_EQ(pose02.y(),  x12 * sin(theta01) + y12 * cos(theta01) + y01);
  ASSERT_DOUBLE_EQ(pose02.theta(), theta01 + theta12);

  // Inverse transformation
  Pose2d pose10 = pose01.Inverse();
  ASSERT_DOUBLE_EQ(pose10.x(),  -y01 * (cos(theta01) + sin(theta01)));
  ASSERT_DOUBLE_EQ(pose10.y(),  -y01 * (cos(theta01) - sin(theta01)));
  ASSERT_DOUBLE_EQ(pose10.theta(), -theta01);

  // Point coordinate transformation
  Point2d p0;  // Coordinates of point p in coordinate system 0
  Point2d p1;  // Coordinates of point p in coordinate system 1.
  Point2d p2;  // Coordinates of point p in coordinate system 2. p0, p1, p2 refer to the same point viewed from different coordinate systems
  p0 = Point2d(1.0, 0.0);
  p1 = pose01.Inverse() * p0;
  ASSERT_DOUBLE_EQ(p1.x(),  -y01 * sin(theta01));
  ASSERT_DOUBLE_EQ(p1.y(),  -y01 * cos(theta01));

  p0 = Point2d(0.0, 1.0);
  p1 = pose01.Inverse() * p0;
  ASSERT_DOUBLE_EQ(p1.x(),  -y01 * cos(theta01));
  ASSERT_DOUBLE_EQ(p1.y(),   y01 * sin(theta01));

  // Coordinate transformation of point cloud
  std::vector<Point2d> pts0;
  std::vector<Point2d> pts1;

  // (1,0)x5
  p0 = Point2d(1.0, 0.0);
  pts0.resize(5, p0);
  pts1 = pose01.Inverse() * pts0;
  for (const auto& p1 : pts1) {
    ASSERT_DOUBLE_EQ(p1.x(),  -y01 * sin(theta01));
    ASSERT_DOUBLE_EQ(p1.y(),  -y01 * cos(theta01));
  }

  // (0,1)x5
  p0 = Point2d(0.0, 1.0);
  pts0.resize(5, p0);
  pts1 = pose01.Inverse() * pts0;
  for (const auto& p1 : pts1) {
    ASSERT_DOUBLE_EQ(p1.x(),  -y01 * sin(theta01));
    ASSERT_DOUBLE_EQ(p1.y(),  -y01 * cos(theta01));
  }
}

}  // end namespace tmc_pose_2d_lib

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
