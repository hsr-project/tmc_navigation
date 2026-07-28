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
#ifndef TMC_BASE_PATH_FOLLOWER_GOAL_CHECKER_HPP_
#define TMC_BASE_PATH_FOLLOWER_GOAL_CHECKER_HPP_
#include <memory>

#include <angles/angles.h>

namespace tmc_base_path_follower {
/// Interface for goal judgment
class IGoalChecker {
 public:
  using Ptr = std::shared_ptr<IGoalChecker>;
  virtual ~IGoalChecker() = default;
  virtual void Initialize() = 0;
  virtual void CheckGoal(const PoseSeq& path, const Pose2d& global_pose,
                         bool& is_arrived_goal_area, bool& is_arrived_goal) = 0;

 protected:
  // Check if the distance between the goal and the robot's position is below the threshold
  // @param [I] goal_pose Goal
  // @param [I] global_pose Robot's position
  // @param [I] distance_threshold Threshold
  // @return true: below threshold false: above threshold
  bool CheckDistanceToGoal(const Pose2d& goal_pose, const Pose2d& global_pose, const double distance_threshold) {
    const double error_length = (goal_pose.point() - global_pose.point()).norm();
    return (error_length < distance_threshold);
  }

  // Check if the angle difference between the goal and the robot's position is below the threshold
  // @param [I] goal_pose Goal
  // @param [I] global_pose Robot's position
  // @param [I] angle_threshold Threshold
  // @return true: below threshold false: above threshold
  bool CheckAngleToGoal(const Pose2d& goal_pose, const Pose2d& global_pose, const double angle_threshold) {
    const double error_angle = angles::shortest_angular_distance(goal_pose.theta(), global_pose.theta());
    return (fabs(error_angle) < angle_threshold);
  }

  // Goal area arrival judgment
  // Divide the circular area centered on the goal with a goal line considering the path direction
  // If the area beyond the goal line is entered, it is considered as reaching the goal area
  // @param [I] path Follow path
  // @param [I] global_pose Robot's position
  // @param [I] goal_area_length Size of the goal area
  // @param [I] goal_line_length Distance from the goal to the goal line
  bool CheckArrivedGoalArea(const PoseSeq& path, const Pose2d& global_pose,
                            const double goal_area_length, const double goal_line_length) {
    // Final point of the path
    const Pose2d goal_pose = path.back();

    // If the distance between the goal and the robot is above the threshold, it is outside the area
    if (!CheckDistanceToGoal(goal_pose, global_pose, goal_area_length)) {
      return false;
    }
    // If there are less than two path points, the judgment is made based only on the distance between the goal and the robot
    if (path.size() < 2) {
      return true;
    }
    // Set the goal line considering the path direction and judge whether it has been crossed
    // The point just before the final point
    const Pose2d prev_goal_pose = path[path.size() -2];
    // Calculate the direction from the point before the final point to the final point
    const double direction_to_goal_pose =
        atan2((goal_pose.y() - prev_goal_pose.y()), (goal_pose.x() - prev_goal_pose.x()));
    // A point on the straight line drawn from the goal to the point before the final point
    const Point2d judgement_point(goal_pose.x() - goal_line_length * cos(direction_to_goal_pose),
                                  goal_pose.y() - goal_line_length * sin(direction_to_goal_pose));

    // Calculate the vector from the straight line to the goal
    Eigen::Vector2d vector_to_goal(cos(direction_to_goal_pose), sin(direction_to_goal_pose));
    // Calculate the vector from the straight line to the robot
    Eigen::Vector2d vector_to_robot(global_pose.x() - judgement_point.x(), global_pose.y() - judgement_point.y());

    // If the dot product of the two vectors is positive, it is judged to be within the goal area
    return (vector_to_goal.dot(vector_to_robot) > 0.0);
  }
};
}  // namespace tmc_base_path_follower

#endif  // TMC_BASE_PATH_FOLLOWER_GOAL_CHECKER_HPP_
