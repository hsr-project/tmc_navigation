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
/// @file     occupancy_grid_converter_main.cpp
/// @brief Functionality to convert a map from ROS format to TMC format with potential
#include <memory>
#include "occupancy_grid_converter.hpp"

/// main function
int main(int32_t argc, char** argv) {
  // Initialization
  rclcpp::init(argc, argv);
  try {
    rclcpp::NodeOptions option;
    option.allow_undeclared_parameters();
    option.automatically_declare_parameters_from_overrides(true);
    auto occupancy_grid_converter = std::make_shared<tmc_grid_map_server::OccupancyGridConverter>(option);
    occupancy_grid_converter->Init();
    rclcpp::spin(occupancy_grid_converter);
  } catch (const std::exception& e) {
    rclcpp::shutdown();
    RCLCPP_FATAL(rclcpp::get_logger("occupancy_grid_converter"), "%s", e.what());
    exit(EXIT_FAILURE);
  }
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
