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
#ifndef TMC_BASE_PATH_FOLLOWER_VELOCITY_CALCULATOR_HPP_
#define TMC_BASE_PATH_FOLLOWER_VELOCITY_CALCULATOR_HPP_
#include <algorithm>
#include <limits>
#include <memory>
#include <optional>

#include "common.hpp"

namespace tmc_base_path_follower {

class IVelocityCalculator {
 public:
  using Ptr = std::shared_ptr<IVelocityCalculator>;

  virtual ~IVelocityCalculator() = default;

  /// Speed Calculation
  /// @param[I] path_info Route information
  /// @param[I] global_pose Self-position
  /// @param[I] current_path_index Index on the route
  /// @param[I] last_velocity Previous velocity
  /// @param[I] time_interval Time interval since the last velocity calculation
  /// @param[I] is_arrived_goal_area Whether it has entered the goal area
  /// @param[I] transit_velocity Transit velocity
  /// @param[O] output_velocity Output velocity
  /// @return Success or failure of speed calculation
  virtual bool CalculateVelocity(const PathInfo& path_info, const Pose2d& global_pose,
                                 const uint32_t current_path_index, const Vector3d& last_velocity,
                                 const double time_interval, const bool is_arrived_goal_area,
                                 const std::optional<double>& transit_velocity, Vector3d& output_velocity) = 0;

 protected:
  // Speed limit
  void LimitVelocity(Vector3d& velocity, const Vector3d& last_velocity, const double time_interval,
                      const Vector3d& max_velocity, const Vector3d& max_acceleration) {
    for (int32_t i = 0; i < kNumBasePoseCoordinates; ++i) {
      double dv = velocity(i) - last_velocity(i);
      double sign = 0.0;
      // Apply maximum acceleration limit
      if (fabs(dv) >= max_acceleration(i) * time_interval) {
        sign = dv / fabs(dv);
        velocity(i) = last_velocity(i) + sign * max_acceleration(i) * time_interval;
      }
      // Apply maximum speed limit
      if (fabs(velocity(i)) > std::numeric_limits<double>::epsilon()) {
        sign = velocity(i) / fabs(velocity(i));
        velocity(i) = sign * std::min<double>(fabs(velocity(i)), max_velocity(i));
      } else {
        velocity(i) = 0.0;
      }
    }
  }
};
}  // namespace tmc_base_path_follower

#endif  // TMC_BASE_PATH_FOLLOWER_VELOCITY_CALCULATOR_HPP_
