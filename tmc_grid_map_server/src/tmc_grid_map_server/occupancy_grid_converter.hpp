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
/// @file     occupancy_grid_converter.hpp
/// @brief    Conversion class from ROS format map to TMC format map with potential
#ifndef TMC_GRID_MAP_SERVER_OCCUPANCY_GRID_CONVERTER_HPP_
#define TMC_GRID_MAP_SERVER_OCCUPANCY_GRID_CONVERTER_HPP_
#include <string>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tmc_navigation_msgs/msg/occupancy_grid_uint.hpp>

// TODO(kazuhito_tanaka) 自律移動システムがTMC形式を使用しなくなったタイミングで削除

namespace tmc_grid_map_server {
class OccupancyGridConverter : public rclcpp::Node {
 public:
  explicit OccupancyGridConverter(const rclcpp::NodeOptions& options);
  // Initialization
  void Init();

 private:
  void Callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_;
  rclcpp::Publisher<tmc_navigation_msgs::msg::OccupancyGridUint>::SharedPtr pub_;

  // Occupied threshold
  double occupied_thresh_;
  // Potential width
  double potential_width_;
};
}  // end of namespace tmc_grid_map_server
#endif
