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
#ifndef TMC_MAP_MERGER_MAP_MERGER_TF2HELPER_HPP_
#define TMC_MAP_MERGER_MAP_MERGER_TF2HELPER_HPP_
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>

namespace tmc_map_merger {

/// @brief Helper class for transforming frame coordinates into the fixed_frame coordinate system
/// Implemented as a Singleton because there is a TF Listener.
class Tf2Helper {
 public:
  Tf2Helper() :  timeout_(0, 0) {}
  Tf2Helper(const Tf2Helper&) = delete;
  Tf2Helper& operator=(const Tf2Helper&) = delete;

  static Tf2Helper* GetInstance() {
    static Tf2Helper tf2_helper;
    return &tf2_helper;
  }

  virtual ~Tf2Helper() {}
  /// @brief Initialization (Listener starts by creating a thread, so explicit initialization is required)
  void Init(const rclcpp::Node::SharedPtr& node, const std::string& fixed_frame, const rclcpp::Duration timeout) {
    buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    // Start the Listener
    fixed_frame_ = fixed_frame;
    timeout_ = timeout;
    listener_ = std::shared_ptr<tf2_ros::TransformListener>(new tf2_ros::TransformListener(*buffer_));
  }

  /// @brief Transform the given Pose into the fixed coordinate system
  bool GetPoseFromFixedFrame(const geometry_msgs::msg::PoseStamped& pose, geometry_msgs::msg::PoseStamped& new_pose) {
    // Overwrite the map coordinate system with the fixed_map coordinate system
    bool success = false;
    try {
      geometry_msgs::msg::TransformStamped transform_stamped;
      transform_stamped = buffer_->lookupTransform(fixed_frame_, pose.header.frame_id, rclcpp::Time(pose.header.stamp),
          timeout_);
      tf2::doTransform(pose, new_pose, transform_stamped);
      success = true;
    }
    catch (tf2::TransformException &ex) {
      auto steady_clock = rclcpp::Clock(RCL_ROS_TIME);
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger("map_merger"), steady_clock, 30000, "%s", ex.what());
    }
    return success;
  }

 private:
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  std::string fixed_frame_;
  rclcpp::Duration timeout_;
};

}  // namespace tmc_map_merger

#endif
