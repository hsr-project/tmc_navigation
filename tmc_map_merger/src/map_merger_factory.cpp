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

#include <string>

#include "map_merger.hpp"
#include "param.hpp"
#include "temporal_memories.hpp"

namespace tmc_map_merger {

MapMerger::Ptr MapMergerFactory::Create(const std::map<std::string, rclcpp::Parameter>& parameters) {
  MapMerger::Ptr merger;
  std::string merger_type;
  double resolution;
  double map_width;
  double map_height;
  GetRequiredParam(parameters, "merger", merger_type);
  GetRequiredParam(parameters, "resolution", resolution, Greater<double>(0.0));
  GetRequiredParam(parameters, "width", map_width, Greater<double>(0.0));
  GetRequiredParam(parameters, "height", map_height, Greater<double>(0.0));

  // Reduce to grid count
  uint32_t width = static_cast<uint32_t>(std::ceil(map_width / resolution));
  uint32_t height = static_cast<uint32_t>(std::ceil(map_height / resolution));

  if (merger_type == "simple") {
    merger.reset(new SimpleMapMerger<UpdateIfGreaterMapAdapter>(width, height, resolution));
  } else if (merger_type == "simple_memory") {
    double keep_time;
    GetRequiredParam(parameters, "keep_time", keep_time, NotLess<double>(0.0));

    int default_data;
    GetRequiredParam(parameters, "default_data", default_data, InRange<int>(-1, 100));
    SimpleMemory::Option option;
    option.timeout = rclcpp::Duration::from_seconds(keep_time);
    option.default_data = default_data;
    merger.reset(new MemoryMapMerger<SimpleMemory>(width, height, resolution, option));
  } else if (merger_type == "safety_memory") {
    double keep_time;
    GetRequiredParam(parameters, "keep_time", keep_time, NotLess<double>(0.0));
    int default_data;
    GetRequiredParam(parameters, "default_data", default_data, InRange<int>(-1, 100));
    SafetyMemory::Option option;
    option.timeout = rclcpp::Duration::from_seconds(keep_time);
    option.default_data = default_data;
    merger.reset(new MemoryMapMerger<SafetyMemory>(width, height, resolution, option));
  } else if (merger_type == "hoge_memory") {
#if 0
    double keep_time;
    GetRequiredParam(parameters, "keep_time", keep_time, Greater<double>(0.0));
    SimpleMemory::Option option;
    option.timeout = ros::Duration::from_seconds(keep_time);
    merger.reset(new MemoryMapMerger<HogeMemory>(width, height, resolution, option));
#endif
  } else {
    RCLCPP_FATAL(rclcpp::get_logger("map_merger"), "Unknown merger type: %s", merger_type.c_str());
    exit(EXIT_FAILURE);
  }
  return merger;
}

}  // namespace tmc_map_merger
