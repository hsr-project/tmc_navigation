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
#include <tmc_base_path_planner/grid_cells_publisher.hpp>
#include <rclcpp/qos.hpp>

namespace {
/// Topic name of GridCells
constexpr const char* const kGridCellsTopic = "inflated_static_obstacle_map";
}  // anonymous namespace

namespace tmc_base_path_planner {

/// Constructor
GridCellsPublisher::GridCellsPublisher(rclcpp::Node::SharedPtr node) {
  rclcpp::QoS custom_qos(1);
  custom_qos.transient_local();
  pub_grid_cells_ = node->create_publisher<nav_msgs::msg::GridCells>(kGridCellsTopic, custom_qos);
  clock_ = node->get_clock();
}


/// Publish GridCells
/// Publish GridCell considering areas where map occupancy is greater than the threshold as walls
/// Exclude areas that were originally walls in map occupancy
void GridCellsPublisher::PublishGridCells(const CostMapPtr& map, const unsigned char visualization_threshold) {
  const uint32_t width = map->width();
  const uint32_t height = map->height();
  const double resolution = map->resolution();

  nav_msgs::msg::GridCells grid_cells;
  grid_cells.cell_width = resolution;
  grid_cells.cell_height = resolution;
  grid_cells.header.frame_id = "map";
  grid_cells.header.stamp = clock_->now();
  for (uint32_t i_h = 0; i_h < height; ++i_h) {
    for (uint32_t i_w = 0; i_w < width; ++i_w) {
      unsigned char value = 0;
      map->GetValueAt(i_w, i_h, value);
      if (value > visualization_threshold && value < kWallValue) {
        double x = i_w * resolution;
        double y = i_h * resolution;
        Pose2d cell = Pose2d(x, y, 0.0);
        // Convert from image coordinate system to map coordinate system
        map->ImageToMap(cell);
        geometry_msgs::msg::Point point;
        point.x = cell.x();
        point.y = cell.y();
        point.z = 0.0;
        grid_cells.cells.push_back(point);
      }
    }
  }
  pub_grid_cells_->publish(grid_cells);
}

}  // namespace tmc_base_path_planner
