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
/// @file     grid_map_server_node.hpp
/// @brief    Send map data for autonomous navigation

#ifndef TMC_GRID_MAP_SERVER_GRID_MAP_SERVER_NODE_HPP_
#define TMC_GRID_MAP_SERVER_GRID_MAP_SERVER_NODE_HPP_

#include <string>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>
#include <tmc_navigation_msgs/msg/occupancy_grid_uint.hpp>
#include <tmc_navigation_msgs/srv/reload_map.hpp>

namespace tmc_grid_map_server {

class GridMapServerNode : public rclcpp::Node {
 public:
  explicit GridMapServerNode(const rclcpp::NodeOptions& options);
  // Initialization
  void Init();
  virtual ~GridMapServerNode() {}
  /// Main processing
  void Run();

 private:
  /// Load map file
  void LoadMap(const std::string& file_path, nav_msgs::msg::OccupancyGrid& map);
  /// Set map metadata
  void SetMapMetaData(nav_msgs::msg::OccupancyGrid& map);
  /// Load map.yaml
  bool LoadConfig(const std::string& config_name);
  /// Convert Pgm values to three values of OccupancyGrid
  int8_t ConvertPgmToOccupancyGridTrinaryValue(const uint8_t pgm_value);
  /// Generate map
  void CreateMap(const std::string& map_file, nav_msgs::msg::OccupancyGrid& map);
  // Subscribe to a map topic for online map updates
  void CallbackMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  // Map reload service callback
  void CallbackServiceSetGoal(
      tmc_navigation_msgs::srv::ReloadMap::Request::SharedPtr req,
      tmc_navigation_msgs::srv::ReloadMap::Response::SharedPtr res);
  // Generate and distribute potential map
  void CreateMapsAndPublish(void);
  // Publish distance map and obstacle map. Publish in both TMC format and ROS format
  void PublishMaps(const nav_msgs::msg::OccupancyGrid& distance_map,
                   const nav_msgs::msg::OccupancyGrid& obstacle_map);

  /// Grid size
  double base_grid_size_;
  /// Map width
  int32_t width_;
  /// Map height
  int32_t height_;
  /// Map origin (x)
  double map_origin_x_;
  /// Map origin (y)
  double map_origin_y_;
  /// Map origin (yaw)
  double map_origin_yaw_;
  /// Map file path
  std::string base_map_file_path_;
  /// Frame ID
  std::string frame_id_;
  /// negate
  int32_t negate_;
  /// Occupied threshold
  double occupied_thresh_;
  /// Free threshold
  double free_thresh_;
  /// Potential width
  double potential_width_;
  /// Convert Unknown to Free
  bool convert_unknown_to_free_;
  /// Declare publisher, subscriber, serviceserver variables
  rclcpp::Publisher<tmc_navigation_msgs::msg::OccupancyGridUint>::SharedPtr pub_distance_map_;
  rclcpp::Publisher<tmc_navigation_msgs::msg::OccupancyGridUint>::SharedPtr pub_obstacle_map_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_distance_ros_map_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_obstacle_ros_map_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map_;
  rclcpp::Service<tmc_navigation_msgs::srv::ReloadMap>::SharedPtr reload_map_;
  std::string distance_map_file_;
  std::string obstacle_map_file_;
};
}  // namespace tmc_grid_map_server

#endif  // TMC_GRID_MAP_SERVER_GRID_MAP_SERVER_NODE_HPP_
