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
/// @file occupancy_ellipse_bumper.cpp
/// @brief Elliptical occupancy bumper
#include "occupancy_ellipse_bumper.hpp"
#include <limits>
#include <angles/angles.h>
#include "occupancy.hpp"
#include "param.hpp"
#include "velocity_slope/velocity_slope.hpp"

namespace {
// ROS parameter name
const char* kRadiusX = "radius_x";                    // Radius in the x-axis direction of the restricted area [m]
const char* kRadiusY = "radius_y";                    // Radius in the y-axis direction of the restricted area [m]
const char* kCenterPositionX = "center_position_x";   // X-coordinate of the center of the restricted area [m]
// ROS parameter default values
const double kRadiusXDef = 2.4;            // Radius in the x-axis direction of the restricted area [m]
const double kRadiusYDef = 1.2;            // Radius in the y-axis direction of the restricted area [m]
const double kCenterPositionXDef = 1.9;    // X-coordinate of the center of the restricted area [m]
// Maximum occupancy value
const int32_t kMaximumOccupancy = 100;
}  // anonymous namespace

namespace tmc_safety_velocity_limiter {
/// Constructor
OccupancyEllipseBumper::OccupancyEllipseBumper(
    std::map<std::string, rclcpp::Parameter>& parameters, VelocitySlope::Ptr& velocity_slope) :
    VirtualBumper(parameters, velocity_slope) {
  UpdateParameters(parameters);
  // Get the maximum distance within the range
  obstacle_search_distance_ = SearchLongestDistance();
}

/// Restrict speed based on occupancy values of grids within the search range
/// If restricted, output the coordinates of the obstacle that caused the restriction, which has the largest and closest occupancy value within the range
/// @param input_velocity [I] Input velocity
/// @param obstacle_pose [O] Obstacle coordinates. Outputs the coordinates with the largest and closest occupancy value within the range
/// @return Restricted speed ratio (0.0 to 1.0)
double OccupancyEllipseBumper::LimitVelocityRatio(const Twist& input_velocity,
                                                  geometry_msgs::msg::PoseStamped& obstacle_pose) {
  // Determine the bumper size ratio based on the input velocity
  const double bumper_scale = CalcBumperScale(input_velocity);
  if (bumper_scale < std::numeric_limits<double>::epsilon()) {
    return false;
  }
  int32_t largest_occupancy = 0;
  const double velocity_angle = atan2(input_velocity.linear.y, input_velocity.linear.x);
  // Tilt the elliptical search range by velocity_angle to align the search range with the direction of travel
  const double rotate_sin = sin(velocity_angle);
  const double rotate_cos = cos(velocity_angle);
  // Calculate each side of the rectangle enclosing the elliptical search range
  const double rectangle_bottom_x = (center_position_x_ - radius_x_) * bumper_scale;
  const double rectangle_top_x = (center_position_x_ + radius_x_) * bumper_scale;
  const double rectangle_right_y = - radius_y_ * bumper_scale;
  const double rectangle_left_y = radius_y_ * bumper_scale;
  // Do not restrict if there is no occupancy data
  if (!Occupancy::GetInstance()->OccupancyGrid()) {
    return 1.0;
  }
  // Calculate the resolution of the occupancy
  const double resolution = Occupancy::GetInstance()->OccupancyGrid()->info.resolution;
  // Scan the rectangle in resolution units and calculate the average occupancy value within the elliptical region
  double sum_grid_weight = 0.0;
  double sum_occupancy = 0.0;
  for (double x = rectangle_bottom_x; x < rectangle_top_x; x += resolution) {
    for (double y = rectangle_right_y; y < rectangle_left_y; y += resolution) {
      const double clue_for_ellipse_range = pow((x - center_position_x_ * bumper_scale), 2.0) /
                                            pow((radius_x_ * bumper_scale), 2.0) +
                                            pow(y, 2.0) / pow((radius_y_ * bumper_scale), 2.0);
      // Calculate whether the target is within the elliptical speed restriction range
      if (clue_for_ellipse_range < 1.0) {
        // Calculate weights based on the distance from the self-position to the grid (1.0: at the base ~ 0.0: at the edge of the range)
        double grid_weight = 1.0 - sqrt(x * x + y * y) / (obstacle_search_distance_ * bumper_scale);
        // Rotate the reference coordinates to the occupancy in the direction of velocity
        const double rotated_x = x * rotate_cos - y * rotate_sin;
        const double rotated_y = x * rotate_sin + y * rotate_cos;
        int32_t occupancy = Occupancy::GetInstance()->GetOccupancyFromBase(rotated_x, rotated_y);
        if (occupancy > kMaximumOccupancy) {
          occupancy = kMaximumOccupancy;
        }
        // Exclude -1 (unknown) from aggregation
        if (occupancy >= 0) {
          sum_grid_weight += grid_weight;
          sum_occupancy += static_cast<double>(occupancy) * grid_weight;
        }
        // Use the coordinates with the largest occupancy value within the range as the restriction factor coordinates
        if (largest_occupancy < occupancy) {
          largest_occupancy = occupancy;
          obstacle_pose.pose.position.x = rotated_x;
          obstacle_pose.pose.position.y = rotated_y;
        } else if (occupancy != 0 && largest_occupancy == occupancy) {
          // If the values are the same, adopt the closer one
          const double distance_square = rotated_x * rotated_x + rotated_y * rotated_y;
          const double factor_distance_square =
              obstacle_pose.pose.position.x * obstacle_pose.pose.position.x +
              obstacle_pose.pose.position.y * obstacle_pose.pose.position.y;
          if (distance_square < factor_distance_square) {
            obstacle_pose.pose.position.x = rotated_x;
            obstacle_pose.pose.position.y = rotated_y;
          }
        }
      }
    }
  }
  // Calculate the weighted average of the occupancy
  double average_occupancy;
  if (sum_grid_weight > 0.0) {
    average_occupancy = sum_occupancy / sum_grid_weight;
  } else {
    // If there are no valid points, set it to 0
    average_occupancy = 0.0;
  }
  // Convert the average occupancy value to the evaluation value of the Slope class (0.0: maximum obstacle ~ 1.0: no obstacle)
  const double average_occpancy_ratio = 1.0 - average_occupancy / static_cast<double>(kMaximumOccupancy);
  // Calculate and return the speed ratio from the evaluation value
  return velocity_slope_->CalcRatio(average_occpancy_ratio);
}

/// Calculate the distance from the self-position to the farthest point within the elliptical range
double OccupancyEllipseBumper::SearchLongestDistance() {
  double longest_distance = DBL_MAX;
  // Calculate the distances from the origin (self-position) to the front, side, and rear ends of the ellipse
  double ellipse_top = fabs(radius_x_ + center_position_x_);
  double ellipse_side = sqrt(center_position_x_ * center_position_x_ + radius_y_ * radius_y_);
  double ellipse_bottom = fabs(radius_x_ - center_position_x_);
  // Select the farthest of the three points
  if (ellipse_top > ellipse_side && ellipse_top > ellipse_bottom) {
    longest_distance = ellipse_top;
  } else if (ellipse_side > ellipse_bottom) {
    longest_distance = ellipse_side;
  } else {
    longest_distance = ellipse_bottom;
  }
  return longest_distance;
}

/// Retrieve ROS PARAM
void OccupancyEllipseBumper::UpdateParameters(std::map<std::string, rclcpp::Parameter>& parameters) {
  GetOptionalParam(parameters, kRadiusX, radius_x_, kRadiusXDef);
  if (radius_x_ <= 0.0) {
    RCLCPP_WARN(rclcpp::get_logger("safety_velocity_limiter"),
        "Parameter [%s] is invalid %lf. Use default value %lf", kRadiusX, radius_x_, kRadiusXDef);
    radius_x_ = kRadiusXDef;
  }
  GetOptionalParam(parameters, kRadiusY, radius_y_, kRadiusYDef);
  if (radius_y_ <= 0.0) {
    RCLCPP_WARN(rclcpp::get_logger("safety_velocity_limiter"),
        "Parameter [%s] is invalid %lf. Use default value %lf", kRadiusY, radius_y_, kRadiusYDef);
    radius_y_ = kRadiusYDef;
  }
  GetOptionalParam(parameters, kCenterPositionX, center_position_x_, kCenterPositionXDef);
}
}  // namespace tmc_safety_velocity_limiter
