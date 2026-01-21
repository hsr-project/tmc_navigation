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
#ifndef TMC_BASE_PATH_PLANNER_GRID_CELLS_PUBLISHER_HPP_
#define TMC_BASE_PATH_PLANNER_GRID_CELLS_PUBLISHER_HPP_

#include <nav_msgs/msg/grid_cells.hpp>
#include <rclcpp/rclcpp.hpp>

#include "common.hpp"

namespace tmc_base_path_planner {

/// GridCells issuing class
class GridCellsPublisher {
 public:
  /// Constructor
  explicit GridCellsPublisher(rclcpp::Node::SharedPtr node);

  /// GridCells issuance
  /// Issue GridCell where the occupancy rate of the map is greater than the threshold as a wall
  /// Exclude parts of the map that were originally walls
  /// @param[I] map Map
  /// @param[I] visualization_threshold Threshold
  void PublishGridCells(const CostMapPtr& map, const unsigned char visualization_threshold);

 private:
  /// Publisher
  rclcpp::Publisher<nav_msgs::msg::GridCells>::SharedPtr pub_grid_cells_;
  /// Get time
  rclcpp::Clock::SharedPtr clock_;
};
}  // namespace tmc_base_path_planner
#endif  // TMC_BASE_PATH_PLANNER_GRID_CELLS_PUBLISHER_HPP_
