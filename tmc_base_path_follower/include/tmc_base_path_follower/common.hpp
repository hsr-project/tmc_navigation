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
/// @file common.hpp
/// @brief Common Definitions
#ifndef TMC_BASE_PATH_FOLLOWER_COMMON_HPP_
#define TMC_BASE_PATH_FOLLOWER_COMMON_HPP_

#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <tmc_pose_2d_lib/pose_2d.hpp>

namespace tmc_base_path_follower {
using Eigen::Vector3d;
using tmc_pose_2d_lib::Pose2d;
using tmc_pose_2d_lib::Point2d;
using PoseSeq = std::vector<Pose2d>;

// Index of the cart's position and orientation
enum BasePoseCoordinates {
  kPoseX,
  kPoseY,
  kPoseTheta,
  kNumBasePoseCoordinates
};

struct PathInfo {
  // Original path
  PoseSeq origin_path;
  // Spline interpolated path
  PoseSeq splined_path;
  // Curvature [rad/m] at each point of the spline interpolated path
  std::vector<double> splined_path_curvatures;
  // Distance [m] from each point of the spline interpolated path to the goal
  std::vector<double> splined_path_left_lengths;
};
}  // namespace tmc_base_path_follower

#endif  // TMC_BASE_PATH_FOLLOWER_COMMON_HPP_
