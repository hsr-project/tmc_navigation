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
/// @file     occupancy_grid_converter.cpp
/// @brief Conversion class from ROS format map to TMC format map with potential
#include <vector>
#include <rclcpp/qos.hpp>

#include "occupancy_grid_converter.hpp"
#include "param.hpp"
#include "util.hpp"

namespace {
// Default value of potential width [m]
constexpr double kDefaultPotentialWidth = 3.0;
// Default value of OccupiedThresh
constexpr double kDefaultOccupiedThresh = 0.65;
}  // anonymous namespace


namespace tmc_grid_map_server {
using std::placeholders::_1;

OccupancyGridConverter::OccupancyGridConverter(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("occupancy_grid_converter", options) {}

// Initialization
void OccupancyGridConverter::Init() {
  sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "~/map_ros", 1, std::bind(&OccupancyGridConverter::Callback, this, _1));
  rclcpp::QoS custom_qos(1);
  custom_qos.transient_local();
  pub_ = this->create_publisher<tmc_navigation_msgs::msg::OccupancyGridUint>("~/map_tmc", custom_qos);

  // Parameter setting
  GetOptionalParam(shared_from_this(), "potential_width", potential_width_, kDefaultPotentialWidth);
  GetOptionalParam(shared_from_this(), "occupied_thresh", occupied_thresh_, kDefaultOccupiedThresh);
}

void OccupancyGridConverter::Callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  tmc_navigation_msgs::msg::OccupancyGridUint tmc_map;
  if (!CreateTmcPotentialMap(*msg, potential_width_, occupied_thresh_, tmc_map)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to create potential map");
    throw std::runtime_error("Failed to create potential map");
  }
  pub_->publish(tmc_map);
}
}  // end of namespace tmc_grid_map_server
