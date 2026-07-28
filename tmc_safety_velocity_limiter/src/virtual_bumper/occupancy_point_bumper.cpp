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
/// @file occupancy_point_bumper.cpp
/// @brief Single point reference occupancy bumper
#include <map>
#include <string>

#include "occupancy_point_bumper.hpp"
#include "occupancy.hpp"
#include "velocity_slope/velocity_slope.hpp"

namespace {
// Maximum occupancy value
const int32_t kMaximumOccupancy = 100;
}  // anonymous namespace

namespace tmc_safety_velocity_limiter {
/// Constructor
OccupancyPointBumper::OccupancyPointBumper(
    std::map<std::string, rclcpp::Parameter>& parameters, const VelocitySlope::Ptr& velocity_slope) :
    VirtualBumper(parameters, velocity_slope) {}

/// Returns the speed limit multiplier based on the occupancy value of the grid closest to the self-position
/// Outputs the self-position as the coordinate that caused the restriction
/// @param input_velocity [I] Input velocity
/// @param obstacle_pose [O] Obstacle coordinates. Always outputs the origin
/// @return Speed limit multiplier (0.0 to 1.0)
double OccupancyPointBumper::LimitVelocityRatio(
    const Twist& input_velocity, geometry_msgs::msg::PoseStamped& obstacle_pose) {
  obstacle_pose.pose.position.x = 0.0;
  obstacle_pose.pose.position.y = 0.0;
  // Get the occupancy value of the self-position (= origin)
  int32_t value = Occupancy::GetInstance()->GetOccupancyFromBase(0.0, 0.0);

  // Treat -1 (unknown) as 0
  if (value < 0) {
    value = 0;
  }
  if (value > kMaximumOccupancy) {
    value = kMaximumOccupancy;
  }

  // Convert occupancy value to an evaluation value between 0.0 and 1.0
  const double occupancy_ratio = 1.0 - (static_cast<double>(value) / static_cast<double>(kMaximumOccupancy));
  // Calculate and return the speed multiplier from the occupancy evaluation value
  return velocity_slope_->CalcRatio(occupancy_ratio);
}
}  // namespace tmc_safety_velocity_limiter
