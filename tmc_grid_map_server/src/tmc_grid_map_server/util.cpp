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
/// @file     util.cpp
/// @brief Functions commonly used within this package
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tmc_navigation_msgs/msg/occupancy_grid_uint.hpp>
#include <tmc_pose_2d_lib/ros_if.hpp>

#include "util.hpp"

using tmc_pose_2d_lib::DistanceMap;
using tmc_pose_2d_lib::RosMsg2DistanceMap;

namespace tmc_grid_map_server {
// Convert tmc specification map to ros specification
void ConvertMapTmcToRos(const tmc_navigation_msgs::msg::OccupancyGridUint& tmc_map,
    nav_msgs::msg::OccupancyGrid& ros_map) {
  // Map size check
  if (tmc_map.info.width * tmc_map.info.height != tmc_map.data.size()) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("grid_map_server"),
        "ConvertMapTmcToRos error! width(" << tmc_map.info.width << ")*height(" << tmc_map.info.height
        << ") is different from map data size(" << tmc_map.data.size() << "). Ignored.");
    return;
  }

  // Copy map parameters
  ros_map.header = tmc_map.header;
  ros_map.info = tmc_map.info;

  // Conversion from tmc specification to ros specification
  // Since tmc data starts from the top-left and ros starts from the bottom-left, store in reverse order in the height direction
  for (int32_t i = (tmc_map.info.height - 1); i >= 0; --i) {
    for (int32_t j = 0; j <= (tmc_map.info.width - 1); ++j) {
      const uint8_t tmc_data = tmc_map.data[tmc_map.info.width * i + j];
      int8_t data = 0;
      if (tmc_data == kTMC_UNKNOWN) {
          data = kROS_UNKNOWN;
      } else {
        // Convert from tmc range [1,255] to ros range [0,100]
        data = static_cast<int8_t>(round((static_cast<double>(tmc_data - 1) / 254.0) * 100.0));
      }
      ros_map.data.push_back(data);
    }
  }
}

// Convert ros specification map to tmc specification
void ConvertMapRosToTmc(const nav_msgs::msg::OccupancyGrid& ros_map,
    tmc_navigation_msgs::msg::OccupancyGridUint& tmc_map) {
  // Map size check
  if (ros_map.info.width * ros_map.info.height != ros_map.data.size()) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("grid_map_server"),
        "ConvertMapRosToTmc error! width(" << ros_map.info.width << ")*height(" << ros_map.info.height
        << ") is different from map data size(" << ros_map.data.size() << "). Ignored.");
    return;
  }

  // Copy map parameters
  tmc_map.header = ros_map.header;
  tmc_map.info = ros_map.info;

  // Conversion from ros specification to tmc specification
  // Since tmc data starts from the top-left and ros starts from the bottom-left, store in reverse order in the height direction
  for (int32_t i = (ros_map.info.height - 1); i >= 0; --i) {
    for (int32_t j = 0; j <= (ros_map.info.width - 1); ++j) {
      const int8_t ros_data = ros_map.data[ros_map.info.width * i + j];
      uint8_t data = 0;
      if (ros_data == kROS_UNKNOWN) {
          data = kTMC_UNKNOWN;
      } else {
        // Convert from ros range [0,100] to tmc range [1,255]
        data = static_cast<uint8_t>(round((static_cast<double>(ros_data) * 254.0) / 100.0) + 1.0);
      }
      tmc_map.data.push_back(data);
    }
  }
}

/// Generate tmc specification inflated map from ros specification map
bool CreateTmcPotentialMap(const nav_msgs::msg::OccupancyGrid& ros_map, const double potential_width,
    const double occupied_thresh, tmc_navigation_msgs::msg::OccupancyGridUint& tmc_potential_map) {
  // Map size check
  if (ros_map.info.width * ros_map.info.height != ros_map.data.size()) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("grid_map_server"),
        "CreateTmcPotentialMap error! width(" << ros_map.info.width << ")*height(" << ros_map.info.height
        << ") is different from map data size(" << ros_map.data.size() << "). Ignored.");
    return false;
  }

  nav_msgs::msg::OccupancyGrid ros_convert_wall_map = ros_map;
  if (occupied_thresh > 0.0 && occupied_thresh < 1.0) {
    // Set locations exceeding occupied_thresh as walls
    const int8_t occupied_thresh_value = static_cast<int8_t>(kROS_WALL * occupied_thresh);
    for (int32_t i = 0; i < ros_convert_wall_map.data.size(); ++i) {
      if (ros_convert_wall_map.data.at(i) > occupied_thresh_value) {
        ros_convert_wall_map.data.at(i) = kROS_WALL;
      }
    }
  }

  tmc_navigation_msgs::msg::OccupancyGridUint tmc_map;
  // Convert to tmc specification
  // If converted to DistanceMap with ros specification, range specification differences are absorbed
  // Specification differences in data order are not absorbed, so convert to tmc once
  ConvertMapRosToTmc(ros_convert_wall_map, tmc_map);

  // Convert to DistanceMap
  DistanceMap distance_map = RosMsg2DistanceMap(tmc_map);
  distance_map.InflateMap(potential_width);

  tmc_potential_map.header = tmc_map.header;
  tmc_potential_map.info = tmc_map.info;
  tmc_potential_map.data = distance_map.data();
  return true;
}
}  // namespace tmc_grid_map_server
