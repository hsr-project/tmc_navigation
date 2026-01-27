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
#include <string>

#include <tmc_base_path_planner/base_path_planner_node.hpp>
#include <tmc_base_path_planner/param.hpp>

namespace {
/// Static Map topic name
constexpr const char* const kStaticMapTopic = "static_obstacle_ros_map";
/// Dynamic Map topic name
constexpr const char* const kDynamicMapTopic = "dynamic_obstacle_map";
/// Self-position topic name
constexpr const char* const kGlobalPoseTopic = "global_pose";
/// Path topic name
constexpr const char* const kPathTopicName = "base_local_path";
/// Path planning action name
constexpr const char* const kPathPlanAction = "base_path_plan";
/// Path following action name
constexpr const char* const kPathFollowAction = "path_follow_action";
/// Map frame name
constexpr const char* const kMapFrameName = "map";
/// TF timeout duration [s]
constexpr double kTfWaitTime = 1.0;
}  // anonymous namespace


namespace tmc_base_path_planner {
using std::placeholders::_1;
using std::placeholders::_2;
/// Convert from PoseSeq to nav_msgs::msg::Path type
nav_msgs::msg::Path ConvertPoseSeqToPath(const PoseSeq& pose_seq, const rclcpp::Time& stamp) {
  nav_msgs::msg::Path path;
  path.header.stamp = stamp;
  path.header.frame_id = kMapFrameName;
  // Store route points
  geometry_msgs::msg::PoseStamped path_pose;
  path_pose.header = path.header;
  for (PoseSeq::const_iterator it = pose_seq.begin(); it != pose_seq.end(); ++it) {
    path_pose.pose = GetPoseMsg(*it);
    path.poses.push_back(path_pose);
  }
  return path;
}


/// Coordinate transformation
bool TransformPoseStamped(const tf2_ros::Buffer& tf_buffer, const geometry_msgs::msg::PoseStamped& in_pose,
                          const std::string& frame_id, geometry_msgs::msg::PoseStamped& out_pose) {
  try {
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped = tf_buffer.lookupTransform(
        frame_id, in_pose.header.frame_id, in_pose.header.stamp, rclcpp::Duration::from_seconds(kTfWaitTime));
    tf2::doTransform(in_pose, out_pose, transform_stamped);
  } catch (const tf2::TransformException& ex) {
    RCLCPP_ERROR(rclcpp::get_logger("base_path_planner"), "Couldn't transform \'%s\' to \'%s\': %s",
                 in_pose.header.frame_id.c_str(), frame_id.c_str(), ex.what());
    return false;
  }
  return true;
}

/// Constructor
BasePathPlannerNode::BasePathPlannerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("base_path_planner", options),
      base_path_planner_(nullptr),
      action_server_(nullptr),
      follower_goal_handle_(nullptr),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_),
      is_follower_goal_active_(false) {}

// Initialization
void BasePathPlannerNode::Init() {
  LoadParameter_();
  grid_cells_publisher_ = std::make_shared<GridCellsPublisher>(shared_from_this());
  // Subscriber registration
  sub_static_map_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      kStaticMapTopic, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      std::bind(&BasePathPlannerNode::StaticMapCallback_, this, _1));

  sub_dynamic_map_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      kDynamicMapTopic, 1, std::bind(&BasePathPlannerNode::DynamicMapCallback_, this, _1));

  sub_global_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      kGlobalPoseTopic, 1, std::bind(&BasePathPlannerNode::GlobalPoseCallback_, this, _1));

  // Publisher registration
  pub_base_path_ = this->create_publisher<nav_msgs::msg::Path>(kPathTopicName, 1);

  path_follow_action_client_ = rclcpp_action::create_client<PathFollowActionClient>(this, kPathFollowAction);
}

/// Path planning ACTION callback
void BasePathPlannerNode::PathPlanActionCallback_() {
  const auto goal = action_server_->get_current_goal();
  auto result = std::make_shared<BasePathPlanActionServer::Result>();
  auto feedback = std::make_shared<BasePathPlanActionServer::Feedback>();
  // Initialize planner
  path_plan_mutex_.lock();
  base_path_planner_->Initialize();
  path_plan_mutex_.unlock();

  const Pose2d start_pose = GetPose2dFromRosMsg(current_global_pose_.pose);
  // Convert to Map frame if the goal is specified in a frame other than the Map frame
  geometry_msgs::msg::PoseStamped goal_pose_map;

  if (goal->goal.header.frame_id != kMapFrameName) {
    if (!TransformPoseStamped(tf_buffer_, goal->goal, kMapFrameName, goal_pose_map)) {
      result->reason = BasePathPlanActionServer::Result::TRANSFORM_GOAL_ERROR;
      action_server_->terminate_current(result);
      return;
    }
  } else {
    goal_pose_map = goal->goal;
  }
  const Pose2d goal_pose = GetPose2dFromRosMsg(goal_pose_map.pose);
  rclcpp::Rate rate(param_.rate);
  while (rclcpp::ok()) {
    // Terminate if canceled
    if (CheckCancelAndTerminate_()) {
      return;
    }
    // Terminate if the required Topic has timed out
    if (CheckTopicTimeoutAndTerminate_()) {
      return;
    }
    // Terminate when the Follower's Action is completed
    if (CheckFollowActionCompleteAndTerminate_()) {
      return;
    }
    // Path planning
    const CostMapPtr dynamic_map(new CostMap(RosMsg2DistanceMap(current_dynamic_map_)));
    const Pose2d dynamic_map_origin = GetPose2dFromRosMsg(current_dynamic_map_.info.origin);
    const Pose2d global_pose = GetPose2dFromRosMsg(current_global_pose_.pose);
    PoseSeq planed_path;
    path_plan_mutex_.lock();
    const BasePathPlannerErrorCode error_code = base_path_planner_->PlanPath(
        start_pose, goal_pose, global_pose, dynamic_map, dynamic_map_origin, true, planed_path);
    path_plan_mutex_.unlock();

    if (error_code == BasePathPlannerErrorCode::kSuccess) {
      // Path planning succeeded
      // Generate nav_msgs/Path from PoseSeq
      const nav_msgs::msg::Path path = ConvertPoseSeqToPath(planed_path, this->get_clock()->now());
      // Publish path
      SendFollowAction_(path);
    } else if (error_code == BasePathPlannerErrorCode::kSkip) {
      // No update from the previous path
      // Do not publish path
    } else {
      // Path planning failed
      // Stop path following action
      CancelFollowAction_();
    }
    // Reflect the result in feedback and determine whether to continue
    if (CheckErrorCodeAndAssignActionStatus_(error_code, feedback, result)) {
      // Issue feedback if continuation is possible
      action_server_->publish_feedback(feedback);
    } else {
      // Abort and terminate if continuation is not possible
      action_server_->terminate_current(result);
      return;
    }
    rate.sleep();
  }
}

/// Static Map callback
void BasePathPlannerNode::StaticMapCallback_(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  const CostMapPtr static_map(new CostMap(RosMsg2DistanceMap(*msg)));
  // Expand the obstacle area from where there is a wall
  static_map->InflateMap(param_.static_map_potential_width);
  // Generate planner when the static Map is subscribed
  base_path_planner_ = BasePathPlannerFactory::Create(shared_from_this(),
      static_map, param_.static_map_potential_width);

  // Start action server
  if (!action_server_) {
    action_server_ = std::make_shared<SimpleActionServer<BasePathPlanActionServer>>(
        shared_from_this(),
        kPathPlanAction,
        std::bind(&BasePathPlannerNode::PathPlanActionCallback_, this));
    action_server_->activate();
  }
  grid_cells_publisher_->PublishGridCells(static_map, base_path_planner_->static_map_occupancy_threshold());
  return;
}

/// Dynamic Map callback
void BasePathPlannerNode::DynamicMapCallback_(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  current_dynamic_map_ = *msg;
  return;
}
/// Self-position callback
void BasePathPlannerNode::GlobalPoseCallback_(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  // Do not update the internal self-position of the planner if the frame ID of the self-position is different from the specified one
  if (msg->header.frame_id != kMapFrameName) {
    auto clock = rclcpp::Clock(RCL_ROS_TIME);
    RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), clock, 5000,
        "Frame_id of subscribed global pose is different from expected frame id  '"
        << kMapFrameName << "'. Couldn't update global pose.");
    return;
  }
  current_global_pose_ = *msg;
  return;
}

/// Read parameters
void BasePathPlannerNode::LoadParameter_() {
  std::map<std::string, rclcpp::Parameter> node_params;
  GetGroupParam(shared_from_this(), kBasePathPlannerNodeSpace, node_params);
  double rate;
  GetOptionalParam(node_params, kRateName, rate, kRateDefault);

  double global_pose_timeout;
  GetOptionalParam(node_params, kGlobalPoseTimeoutName, global_pose_timeout, kGlobalPoseTimeoutDefault);

  double dynamic_map_timeout;
  GetOptionalParam(node_params, kDynamicMapTimeoutName, dynamic_map_timeout, kDynamicMapTimeoutDefault);

  double static_map_potential_width;
  GetOptionalParam(node_params, kStaticMapPotentialWidthName, static_map_potential_width,
                   kStaticMapPotentialWidthDefault);

  param_ = BasePathPlannerNode::Parameter(rate, global_pose_timeout, dynamic_map_timeout, static_map_potential_width);
}

/// Send path following action
void BasePathPlannerNode::SendFollowAction_(const nav_msgs::msg::Path& path) {
  auto send_goal_options = rclcpp_action::Client<PathFollowActionClient>::SendGoalOptions();
  send_goal_options.goal_response_callback =
      std::bind(&BasePathPlannerNode::goal_response_callback, this, _1);
  send_goal_options.feedback_callback =
      std::bind(&BasePathPlannerNode::feedback_callback, this, _1, _2);
  send_goal_options.result_callback =
      std::bind(&BasePathPlannerNode::result_callback, this, _1);


  auto follower_goal = PathFollowActionClient::Goal();
  follower_goal.path = path;

  // Path following request
  auto goal_handle_future = path_follow_action_client_->async_send_goal(follower_goal, send_goal_options);
  is_follower_goal_active_ = true;
  follower_goal_handle_ = goal_handle_future.get();
  // Display path
  pub_base_path_->publish(path);
}


void BasePathPlannerNode::goal_response_callback(const PathFollowGoalHandle::SharedPtr& future) {
  // TODO(syuuhei_shiro): Goalがサーバからrejectされた場合のチェック
}

void BasePathPlannerNode::feedback_callback(PathFollowGoalHandle::SharedPtr,
    const std::shared_ptr<const PathFollowActionClient::Feedback> feedback) {
  // do nothing
}

void BasePathPlannerNode::result_callback(const PathFollowGoalHandle::WrappedResult& result) {
  // do nothing
}

/// Cancel path following action
void BasePathPlannerNode::CancelFollowAction_() {
  if (is_follower_goal_active_ && follower_goal_handle_) {
    // Cancel path following
    path_follow_action_client_->async_cancel_goal(follower_goal_handle_);
    is_follower_goal_active_ = false;
    follower_goal_handle_ = nullptr;
    // Display empty path
    nav_msgs::msg::Path path;
    path.header.stamp = this->get_clock()->now();
    path.header.frame_id = kMapFrameName;
    pub_base_path_->publish(path);
  }
}

/// Check for Topic timeout, perform necessary termination processing if timed out, and return true
bool BasePathPlannerNode::CheckTopicTimeoutAndTerminate_() {
  auto result = std::make_shared<BasePathPlanActionServer::Result>();
  const rclcpp::Time current_time = this->get_clock()->now();
  const double dynamic_map_timeout = param_.dynamic_map_timeout;
  const double global_pose_timeout = param_.global_pose_timeout;
  if ((current_time - current_dynamic_map_.header.stamp) > rclcpp::Duration::from_seconds(dynamic_map_timeout)) {
    // Dynamic map timeout
    RCLCPP_ERROR(this->get_logger(), "dynamic map has not been updated for %lf seconds.", dynamic_map_timeout);
    result->reason = BasePathPlanActionServer::Result::DYNAMIC_MAP_IS_NOT_UPDATED;
    CancelFollowAction_();
    action_server_->terminate_current(result);
    return true;
  } else if ((current_time - current_global_pose_.header.stamp) >
             rclcpp::Duration::from_seconds(global_pose_timeout)) {
    // Self-position timeout
    RCLCPP_ERROR(this->get_logger(), "global_pose has not been updated for %lf seconds.", global_pose_timeout);
    result->reason = BasePathPlanActionServer::Result::ROBOT_POSE_IS_NOT_UPDATED;
    CancelFollowAction_();
    action_server_->terminate_current(result);
    return true;
  }
  return false;
}

/// Check for cancel request, perform necessary termination processing if canceled, and return true
bool BasePathPlannerNode::CheckCancelAndTerminate_() {
  auto result = std::make_shared<BasePathPlanActionServer::Result>();
  // Check for cancel request
  if (action_server_->is_preempt_requested()) {
    // In case of goal overwrite, terminate without stopping the path_follow action to seamlessly transition to the new goal
    result->reason = BasePathPlanActionServer::Result::PREEMPTED;
    action_server_->terminate_current(result);
    return true;
  }
  if (action_server_->is_cancel_requested() || !action_server_->is_server_active()) {
    // Stop path_follow action
    CancelFollowAction_();
    result->reason = BasePathPlanActionServer::Result::CANCELED;
    action_server_->terminate_current(result);
    return true;
  }
  return false;
}


/// Check Follower's Action completion, perform necessary termination processing if completed, and return true
bool BasePathPlannerNode::CheckFollowActionCompleteAndTerminate_() {
  if (!is_follower_goal_active_ || follower_goal_handle_ == nullptr) {
    return false;
  }
  auto result = std::make_shared<BasePathPlanActionServer::Result>();
  auto state = follower_goal_handle_->get_status();
  if (state == rclcpp_action::GoalStatus::STATUS_ACCEPTED ||
      state == rclcpp_action::GoalStatus::STATUS_EXECUTING) {
    // Continuing
    return false;
  } else if (state == rclcpp_action::GoalStatus::STATUS_SUCCEEDED) {
    // Normal termination
    result->reason = BasePathPlanActionServer::Result::REACHED;
    action_server_->succeeded_current(result);
  } else if (state == rclcpp_action::GoalStatus::STATUS_CANCELED ||
             state == rclcpp_action::GoalStatus::STATUS_CANCELING) {
    // Follower was interrupted by another node
    result->reason = BasePathPlanActionServer::Result::PREEMPTED;
    action_server_->terminate_current(result);
  } else {
    // Abnormal termination
    RCLCPP_ERROR(this->get_logger(), "%s action from %s return unknown result code",
        kPathFollowAction, kPathPlanAction);
    result->reason = BasePathPlanActionServer::Result::FOLLOWER_ABORTED;
    action_server_->terminate_current(result);
  }
  is_follower_goal_active_ = false;
  follower_goal_handle_ = nullptr;
  return true;
}

/// Check error code, output action status, and return whether path planning can continue
/// Output action feedback and return true if continuation is possible
/// Output action result and return false if continuation is not possible
bool BasePathPlannerNode::CheckErrorCodeAndAssignActionStatus_(const BasePathPlannerErrorCode& code,
    std::shared_ptr<BasePathPlanActionServer::Feedback>& feedback,
    std::shared_ptr<BasePathPlanActionServer::Result>& result) {
  auto clock = rclcpp::Clock(RCL_ROS_TIME);
  switch (code) {
    case BasePathPlannerErrorCode::kSuccess:
      feedback->status = BasePathPlanActionServer::Feedback::RUNNING;
      feedback->reason = BasePathPlanActionServer::Feedback::NONE;
      return true;
    case BasePathPlannerErrorCode::kSkip:
      feedback->status = BasePathPlanActionServer::Feedback::RUNNING;
      feedback->reason = BasePathPlanActionServer::Feedback::NONE;
      return true;
    case BasePathPlannerErrorCode::kPlanningFail:
      feedback->status = BasePathPlanActionServer::Feedback::PLANNING;
      feedback->reason = BasePathPlanActionServer::Feedback::PATH_PLANNING_FAIL;
      RCLCPP_WARN_THROTTLE(this->get_logger(), clock, 5000, "Path plan failed. Can not planning.");
      return true;
    case BasePathPlannerErrorCode::kRobotIsOnDynamicObstacle:
      feedback->status = BasePathPlanActionServer::Feedback::PLANNING;
      feedback->reason = BasePathPlanActionServer::Feedback::ROBOT_IS_ON_DYNAMIC_OBSTACLE;
      RCLCPP_WARN_THROTTLE(this->get_logger(), clock, 5000, "Path plan failed. Robot is on dynamic obstacle.");
      return true;
    case BasePathPlannerErrorCode::kRobotIsOnStaticObstacle:
      feedback->status = BasePathPlanActionServer::Feedback::PLANNING;
      feedback->reason = BasePathPlanActionServer::Feedback::ROBOT_IS_ON_STATIC_OBSTACLE;
      RCLCPP_WARN_THROTTLE(this->get_logger(), clock, 5000, "Path plan failed. Robot is on static obstacle.");
      return true;
    case BasePathPlannerErrorCode::kGoalIsOnDynamicObstacle:
      feedback->status = BasePathPlanActionServer::Feedback::PLANNING;
      feedback->reason = BasePathPlanActionServer::Feedback::GOAL_IS_ON_DYNAMIC_OBSTACLE;
      RCLCPP_WARN_THROTTLE(this->get_logger(), clock, 5000, "Path plan failed. Goal is on dynamic obstacle.");
      return true;
    case BasePathPlannerErrorCode::kGoalIsOnStaticObstacle:
      result->reason = BasePathPlanActionServer::Result::GOAL_IS_ON_STATIC_OBSTACLE;
      RCLCPP_ERROR_THROTTLE(this->get_logger(), clock, 5000, "Path plan failed. Goal is on static obstacle.");
      return false;
    case BasePathPlannerErrorCode::kRobotIsOutOfMap:
      RCLCPP_ERROR_THROTTLE(this->get_logger(), clock, 5000, "Path plan failed. Robot is out of static map.");
      result->reason = BasePathPlanActionServer::Result::ROBOT_IS_OUT_OF_MAP;
      return false;
    case BasePathPlannerErrorCode::kSmoothingFail:
      RCLCPP_ERROR_THROTTLE(this->get_logger(), clock, 5000, "Path plan failed. Smoothing fail.");
      result->reason = BasePathPlanActionServer::Result::SMOOTHING_FAIL;
      return false;
  }
  return false;
}
}  // namespace tmc_base_path_planner
