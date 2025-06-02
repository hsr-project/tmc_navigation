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
#include <chrono>   // NOLINT
#include <Eigen/Dense>
#include <gtest/gtest.h>
#include "tmc_pose_2d_lib/pose_2d.hpp"

namespace tmc_pose_2d_lib {

class EigenCompareTest : public testing::Test {
 protected:
  EigenCompareTest() {}

  virtual void SetUp() {
    x01 = 1.0;
    y01 = 1.0;
    theta01 = M_PI / 10.0;
    x12 = 1.0;
    y12 = 1.0;
    theta12 = M_PI / 10.0;

    px = 1.0;
    py = 1.0;
  }

  virtual void TearDown() {
  }

  double x01;
  double y01;
  double theta01;
  double x12;
  double y12;
  double theta12;
  double px;
  double py;
};


template<typename EigenType>
EigenType GetEigenPose(double x, double y, double theta) {
  // EigenType must be Eigen::Affine2d or Eigen::Isometry2d
  EigenType pose;
  const Eigen::Translation2d trans(x, y);
  const Eigen::Rotation2Dd rot(theta);
  pose = trans * rot;
  return pose;
}

Eigen::Vector2d GetEigenPoint(double x, double y) {
  Eigen::Vector2d p;
  p << x, y;
  return p;
}

/**
 * Disable speed test
 * Sometime, isometry and affine shows high performance than pose2d
TEST_F(EigenCompareTest, PrecisenessTest) {
  // translate point
  const Pose2d pose01(x01, y01, theta01);
  const Point2d p(px, py);
  const Point2d translated_p = pose01 * p;

  const Eigen::Affine2d pose01_eigen = GetEigenPose<Eigen::Affine2d>(x01, y01, theta01);
  const Eigen::Vector2d p_eigen = GetEigenPoint(px, py);
  const Eigen::Vector2d translated_p_eigen = pose01_eigen * p_eigen;

  ASSERT_DOUBLE_EQ(translated_p.x(), translated_p_eigen[0]);
  ASSERT_DOUBLE_EQ(translated_p.y(), translated_p_eigen[1]);


  // translate pose
  const Pose2d pose12(x12, y12, theta12);
  const Pose2d pose02 = pose01 * pose12;
  const Eigen::Affine2d pose12_eigen = GetEigenPose<Eigen::Affine2d>(x12, y12, theta12);
  const Eigen::Affine2d pose02_eigen = pose01_eigen * pose12_eigen;

  ASSERT_DOUBLE_EQ(pose02.x(), pose02_eigen.translation()[0]);
  ASSERT_DOUBLE_EQ(pose02.y(), pose02_eigen.translation()[1]);
}

TEST_F(EigenCompareTest, SpeedTest) {
  std::chrono::system_clock::time_point  start, end;
  const size_t loop_num = 100;

  // Pose2d
  start = std::chrono::system_clock::now();
  for (size_t i=0; i < loop_num; i++) {
    const Pose2d pose01(x01, y01, theta01);
    const Pose2d pose12(x12, y12, theta12);
    const Pose2d pose02 = pose01 * pose12;
  }
  end = std::chrono::system_clock::now();
  const double t_pose2d = std::chrono::duration_cast<std::chrono::microseconds>(end-start).count();


  // Eigen::Affine
  start = std::chrono::system_clock::now();
  for (int i=0; i < loop_num; i++) {
    const Eigen::Affine2d pose01_eigen = GetEigenPose<Eigen::Affine2d>(x01, y01, theta01);
    const Eigen::Affine2d pose12_eigen = GetEigenPose<Eigen::Affine2d>(x12, y12, theta12);
    const Eigen::Affine2d pose02_eigen = pose01_eigen * pose12_eigen;
  }
  end = std::chrono::system_clock::now();
  const double t_eigen_affine = std::chrono::duration_cast<std::chrono::microseconds>(end-start).count();


  // Eigen::Isometry
  start = std::chrono::system_clock::now();
  for (int i=0; i < loop_num; i++) {
    const Eigen::Isometry2d pose01_eigen = GetEigenPose<Eigen::Isometry2d>(x01, y01, theta01);
    const Eigen::Isometry2d pose12_eigen = GetEigenPose<Eigen::Isometry2d>(x12, y12, theta12);
    const Eigen::Isometry2d pose02_eigen = pose01_eigen * pose12_eigen;
  }
  end = std::chrono::system_clock::now();
  const double t_eigen_isometry = std::chrono::duration_cast<std::chrono::microseconds>(end-start).count();


  printf("t_pose2d        :  %f [micro-sec] \n", t_pose2d);
  printf("t_eigen_affine  :  %f [micro-sec] \n", t_eigen_affine);
  printf("t_eigen_isometry:  %f [micro-sec] \n", t_eigen_isometry);

  // pose2d < isometry < affine should be the order; pose2d is about 100 times faster
  ASSERT_LT(t_pose2d, t_eigen_affine);
  ASSERT_LT(t_pose2d, t_eigen_isometry);
}
**/

}  // end namespace tmc_pose_2d_lib

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
