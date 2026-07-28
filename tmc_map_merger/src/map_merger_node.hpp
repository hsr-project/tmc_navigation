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
#ifndef TMC_MAP_MERGER_MAP_MERGER_NODE_HPP_
#define TMC_MAP_MERGER_MAP_MERGER_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include <message_filters/message_event.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>

#include "map_input.hpp"
#include "map_merger.hpp"
#include "param.hpp"
#include "tf2helper.hpp"

namespace tmc_map_merger {

/// @brief Node class
class MapMergerNode : public rclcpp::Node {
 public:
  explicit MapMergerNode(const rclcpp::NodeOptions& options);

  // Destructor
  ~MapMergerNode() {}

  // Initialization
  void Init();

 private:
  void TimerCallback() {
    // Update map center to fixed coordinate system
    rclcpp::Time current_time = now();
    geometry_msgs::msg::PoseStamped center;
    geometry_msgs::msg::PoseStamped map_center;
    center.header.frame_id = origin_frame_;
    center.header.stamp = current_time;
    center.pose.orientation.w = 1.0;
    if (!Tf2Helper::GetInstance()->GetPoseFromFixedFrame(center, map_center)) {
      return;
    }
    try {
      // Reserve message for publishing
      Map::Ptr map(std::make_shared<nav_msgs::msg::OccupancyGrid>());
      // Update
      map_merger_->Update(map_center);
      map_merger_->Clear();
      // Merge
      for (auto map_input : map_inputs_) {
        // Use only valid input
        if (map_input->IsEnabled()) {
          const MapMerger::Ptr& merger = map_input->GetMapMerger();
          merger->Update(map_center);
          map_merger_->Merge(merger->GetMap());
        }
      }
      // Publish after copying and adding header
      *map = map_merger_->GetMap();
      map->header.frame_id = fixed_frame_;
      map->header.stamp = map_center.header.stamp;
      publisher_->publish(*map);
    } catch(const std::runtime_error& ex) {
      auto clock = rclcpp::Clock(RCL_ROS_TIME);
      RCLCPP_WARN_THROTTLE(this->get_logger(), clock, 30000, "%s", ex.what());
    }
  }

  void Reset(const std_srvs::srv::Empty::Request::SharedPtr req,
             const std_srvs::srv::Empty::Response::SharedPtr res) {
    // Reset buffer of root merger
    map_merger_->Clear();

    // Reset buffer of map input port
    for (auto map_input : map_inputs_) {
      map_input->GetMapMerger()->Clear();
    }
  }

  std::string fixed_frame_;
  std::string origin_frame_;
  MapMerger::Ptr map_merger_;
  std::vector<MapInput::Ptr> map_inputs_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace tmc_map_merger

#endif
