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
#ifndef TMC_BASE_PATH_FOLLOWER_PATH_INFO_CREATOR_HPP_
#define TMC_BASE_PATH_FOLLOWER_PATH_INFO_CREATOR_HPP_
#include <memory>
#include <vector>

#include <console_bridge/console.h>
#include "common.hpp"
#include "parameter_default_value.hpp"


namespace tmc_base_path_follower {
// Minimum number of interpolation points
const int32_t kMinimumInterpolationNumber = 2;

/// Route information generation class
class PathInfoCreator {
 public:
  using Ptr = std::shared_ptr<PathInfoCreator>;
  /// Parameters
  struct Parameter {
    Parameter(const int32_t in_interpolation_number,
              const double in_passing_velocity)
        : interpolation_number(in_interpolation_number),
          passing_velocity(in_passing_velocity) {
      if (interpolation_number < kMinimumInterpolationNumber) {
        CONSOLE_BRIDGE_logWarn("Value of 'interpolation_number' is invalid. Use default value.");
        interpolation_number = kInterpolationNumberDefault;
      }
      if (passing_velocity <= 0.0) {
        CONSOLE_BRIDGE_logWarn("Value of 'passing_velocity' is invalid. Use default value.");
        passing_velocity = kMaxLinearVelocityDefault;
      }
    }
    // Number of interpolation points
    int32_t interpolation_number;
    // Maximum speed
    double passing_velocity;
  };

  /// Constructor
  /// @param [I] param Parameters
  explicit PathInfoCreator(const Parameter& param) : param_(param) {}

  /// Route information generation
  /// @param [I] path Input route
  /// @return Route information
  PathInfo CreatePathInfo(const PoseSeq& path);

 private:
  /// @brief Spline interpolation of the route
  /// @param[in] input_path Pre-interpolation route
  /// @param[out] splined_path Interpolated route
  /// @param[out] splined_path_curvatures Curvatures at each point of the interpolated route
  void SplineInterpolation(const PoseSeq& input_path, PoseSeq& splined_path,
      std::vector<double>& splined_path_curvatures);
  // Check for extrema in the spline curve
  bool CheckSplinePathExtremum(const Eigen::Vector4d& coeff, const double time);
  // Calculate the remaining playback length to the goal for all route points
  void CalculateLeftPathLengths(const PoseSeq& path, std::vector<double>& left_path_lengths);
  // Parameters
  Parameter param_;
};

}  // namespace tmc_base_path_follower

#endif  // TMC_BASE_PATH_FOLLOWER_PATH_INFO_CREATOR_HPP_
