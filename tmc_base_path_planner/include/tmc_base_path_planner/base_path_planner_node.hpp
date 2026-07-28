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
#ifndef TMC_BASE_PATH_PLANNER_BASE_PATH_PLANNER_NODE_HPP_
#define TMC_BASE_PATH_PLANNER_BASE_PATH_PLANNER_NODE_HPP_
#include <memory>
#include <mutex>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav2_util/simple_action_server.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tmc_navigation_msgs/action/base_path_plan.hpp>
#include <tmc_navigation_msgs/action/path_follower.hpp>
#include <tmc_pose_2d_lib/ros_if.hpp>

#include "base_path_planner_factory.hpp"
#include "common.hpp"
#include "grid_cells_publisher.hpp"

namespace tmc_base_path_planner {
using tmc_pose_2d_lib::RosMsg2DistanceMap;
using tmc_pose_2d_lib::GetPose2dFromRosMsg;
using tmc_pose_2d_lib::GetPoseMsg;
using nav2_util::SimpleActionServer;

/// Node Class
class BasePathPlannerNode : public rclcpp::Node {
 public:
  /// Constructor
  explicit BasePathPlannerNode(const rclcpp::NodeOptions& options);
  // Initialization
  void Init();

 private:
  using BasePathPlanActionServer = tmc_navigation_msgs::action::BasePathPlan;
  using BasePathPlanGoalHandle = rclcpp_action::ServerGoalHandle<BasePathPlanActionServer>;
  using PathFollowActionClient = tmc_navigation_msgs::action::PathFollower;
  using PathFollowGoalHandle = rclcpp_action::ClientGoalHandle<PathFollowActionClient>;
  /// Parameters
  struct Parameter {
    Parameter() {}
    Parameter(const double in_rate, const double in_global_pose_timeout, const double in_dynamic_map_timeout,
              const double in_static_map_potential_width)
        : rate(in_rate), dynamic_map_timeout(in_dynamic_map_timeout), global_pose_timeout(in_global_pose_timeout),
          static_map_potential_width(in_static_map_potential_width) {
      if (rate <= 0.0) {
        RCLCPP_WARN(rclcpp::get_logger("base_path_planner"),
            "Value of '%s' is invalid. Use default value.", kRateName);
        rate = kRateDefault;
      }
      if (global_pose_timeout <= 0.0) {
        RCLCPP_WARN(rclcpp::get_logger("base_path_planner"),
            "Value of '%s' is invalid. Use default value.", kGlobalPoseTimeoutName);
        global_pose_timeout = kGlobalPoseTimeoutDefault;
      }
      if (dynamic_map_timeout <= 0.0) {
        RCLCPP_WARN(rclcpp::get_logger("base_path_planner"),
            "Value of '%s' is invalid. Use default value.", kDynamicMapTimeoutName);
        dynamic_map_timeout = kDynamicMapTimeoutDefault;
      }
      if (static_map_potential_width < 0.0) {
        RCLCPP_WARN(rclcpp::get_logger("base_path_planner"),
            "Value of '%s' is invalid. Use default value.", kStaticMapPotentialWidthName);
        static_map_potential_width = kStaticMapPotentialWidthDefault;
      }
    }
    // Drive cycle [hz]
    double rate;
    // Dynamic map timeout duration [s]
    double dynamic_map_timeout;
    // Self-position timeout duration [s]
    double global_pose_timeout;
    // Distance to expand the obstacle area of the static map from the wall [m]
    double static_map_potential_width;
  };

  /// Path Planning ACTION Server Callback
  void PathPlanActionCallback_();

  // Path Following Action Client Callback
  void goal_response_callback(const PathFollowGoalHandle::SharedPtr& future);
  void feedback_callback(PathFollowGoalHandle::SharedPtr,
      const std::shared_ptr<const PathFollowActionClient::Feedback> feedback);
  void result_callback(const PathFollowGoalHandle::WrappedResult& result);

  /// Static Map Callback
  void StaticMapCallback_(const nav_msgs::msg::OccupancyGrid::SharedPtr static_map);
  /// Dynamic Map Callback
  void DynamicMapCallback_(const nav_msgs::msg::OccupancyGrid::SharedPtr dynamic_map);
  /// Self-position Callback
  void GlobalPoseCallback_(const geometry_msgs::msg::PoseStamped::SharedPtr global_pose);
  // Parameter Reading
  void LoadParameter_();

  /// Check Topic Timeout. If timed out, perform necessary termination processing and return true
  bool CheckTopicTimeoutAndTerminate_();
  /// Check Cancel Request. If canceled, perform necessary termination processing and return true
  bool CheckCancelAndTerminate_();
  /// Check Follower Action Completion. If completed, perform necessary termination processing and return true
  bool CheckFollowActionCompleteAndTerminate_();

  /// Send Path Following Action
  void SendFollowAction_(const nav_msgs::msg::Path& path);
  /// Cancel Path Following Action
  void CancelFollowAction_();

  /// Check Error Code, Output Action Status, and Return Whether Path Planning Can Continue
  /// If Continuable, Output Action Feedback and Return True
  /// If Not Continuable, Output Action Result and Return False
  bool CheckErrorCodeAndAssignActionStatus_(const BasePathPlannerErrorCode& code,
      std::shared_ptr<BasePathPlanActionServer::Feedback>& feedback,
      std::shared_ptr<BasePathPlanActionServer::Result>& result);

  /// Planner
  BasePathPlanner::Ptr base_path_planner_;
  /// Path Planning Action Server
  std::shared_ptr<SimpleActionServer<BasePathPlanActionServer>> action_server_;
  /// Path Following Action Client
  rclcpp_action::Client<PathFollowActionClient>::SharedPtr path_follow_action_client_;
  /// Static Map Subscriber
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_static_map_;
  /// Dynamic Map Subscriber
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_dynamic_map_;
  /// Self-position Subscriber
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_global_pose_;
  /// Path Publisher
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_base_path_;


  PathFollowGoalHandle::SharedPtr follower_goal_handle_;
  /// tf Buffer
  tf2_ros::Buffer tf_buffer_;
  /// tf Listener
  tf2_ros::TransformListener tf_listener_;
  /// Whether Path Following Action is Active
  bool is_follower_goal_active_;
  /// Current Self-position
  geometry_msgs::msg::PoseStamped current_global_pose_;
  /// Current Dynamic Map
  nav_msgs::msg::OccupancyGrid current_dynamic_map_;
  /// GridCells Publishing
  std::shared_ptr<GridCellsPublisher> grid_cells_publisher_;
  /// Parameters
  Parameter param_;
  // Exclusive Control
  std::mutex path_plan_mutex_;
};

}  // namespace tmc_base_path_planner

#endif  // TMC_BASE_PATH_PLANNER_BASE_PATH_PLANNER_NODE_HPP_
