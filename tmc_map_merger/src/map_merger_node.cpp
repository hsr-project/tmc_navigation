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

#include "map_merger_node.hpp"

#include <chrono>
#include <map>
#include <string>

#include <rclcpp/rclcpp.hpp>


namespace tmc_map_merger {
using std::chrono::milliseconds;
using std::placeholders::_1;
using std::placeholders::_2;


MapMergerNode::MapMergerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("map_merger", options) {}

// Initialization
void MapMergerNode::Init() {
  double publish_rate;
  // Reference frame. All frames are converted to this fixed coordinate system before processing
  GetRequiredParam(shared_from_this(), "fixed_frame", fixed_frame_);
  // Center frame (position) for map publishing
  GetRequiredParam(shared_from_this(), "origin_frame", origin_frame_);
  // Publishing frequency [Hz]
  GetOptionalParam(shared_from_this(), "publish_rate", publish_rate, 1.0, Greater<double>(0.1));

  // Create map input port
  std::map<std::string, rclcpp::Parameter> input_list_parameters;
  GetRequiredGroupParam(shared_from_this(), "inputs", input_list_parameters);
  std::vector<std::string> input_names;
  for (const auto & parameter_pair : input_list_parameters) {
    std::string input_name = parameter_pair.first;
    input_name.erase(input_name.find("."));
    if (std::find(input_names.begin(), input_names.end(), input_name) == input_names.end()) {
      input_names.push_back(input_name);
      std::map<std::string, rclcpp::Parameter> input_parameters;
      GetRequiredGroupParam(shared_from_this(), "inputs." + input_name, input_parameters);
      // Create merger
      MapMerger::Ptr merger = MapMergerFactory::Create(input_parameters);
      // Create input port
      MapInput::Ptr map_input = MapInputFactory::Create(input_name, input_parameters, shared_from_this(), merger);
      map_inputs_.push_back(map_input);
    }
  }

  // Create root merger
  std::map<std::string, rclcpp::Parameter> route_parameters;
  GetRequiredGroupParam(shared_from_this(), "root", route_parameters);
  map_merger_ = MapMergerFactory::Create(route_parameters);
  publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("merged_map", 10);
  // Start TF listener
  Tf2Helper::GetInstance()->Init(shared_from_this(), fixed_frame_, rclcpp::Duration::from_seconds(1.0));

  // Start periodic processing
  timer_ = this->create_wall_timer(milliseconds(static_cast<int32_t>(1000 / publish_rate)),
      std::bind(&MapMergerNode::TimerCallback, this));

  // Publish map reset service
  reset_service_ = this->create_service<std_srvs::srv::Empty>(
      "~/reset", std::bind(&MapMergerNode::Reset, this, _1, _2));
}
}  // namespace tmc_map_merger
