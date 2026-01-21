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
/// @file bumper_set.cpp
/// @brief Class that manages multiple virtual bumpers
#include "bumper_set.hpp"
#include <map>
#include <string>
#include <vector>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include "common.hpp"
#include "param.hpp"
#include "velocity_slope/fixed_slope.hpp"
#include "velocity_slope/linear_slope.hpp"
#include "velocity_slope/logarithm_slope.hpp"
#include "velocity_slope/velocity_slope.hpp"
#include "virtual_bumper/cup_bumper.hpp"
#include "virtual_bumper/ellipse_bumper.hpp"
#include "virtual_bumper/occupancy_ellipse_bumper.hpp"
#include "virtual_bumper/occupancy_point_bumper.hpp"
#include "virtual_bumper/triangle_bumper.hpp"
#include "virtual_bumper/virtual_bumper.hpp"

namespace {
// ROS parameter name
const char* kTypeName = "type";    // Name of the bumper or slope type (class)
const char* kBumpers = "bumpers";  // Bumper definition enumeration
}  // anonymous namespace

namespace tmc_safety_velocity_limiter {

/// Constructor
BumperSet::BumperSet(std::map<std::string, rclcpp::Parameter>& parameters) {
  std::map<std::string, rclcpp::Parameter> bumpers;
  if (!GetGroupParam(parameters, kBumpers, bumpers)) {
    throw std::runtime_error("Mandatory parameter is not set:" + std::string(kBumpers));
    return;
  }
  // Interpret the bumper definitions defined under bumpers in sequence and create instances of the corresponding virtual bumpers
  for (auto it = bumpers.begin(); it != bumpers.end(); ++it) {
    const size_t sbstr_index = it->first.find(".");
    if (sbstr_index == std::string::npos) {
      RCLCPP_INFO(rclcpp::get_logger("safety_velocity_limiter"), "%s is invalid", it->first.c_str());
      continue;
    }
    const std::string bumper_name = it->first.substr(0, sbstr_index);
    if (bumper_list_.find(bumper_name) == bumper_list_.end()) {
      std::map<std::string, rclcpp::Parameter> bumper_param;
      GetGroupParam(bumpers, bumper_name, bumper_param);
      bumper_list_[bumper_name] = CreateVirtualBumper(bumper_param);
    }
  }
}

bool BumperSet::LimitVelocity(const Twist& input_velocity,
                              const std::vector<std::string>& disable_bumpers,
                              Twist& output_velocity,
                              geometry_msgs::msg::PoseStamped& observed_obstacle_pose, double& minimum_ratio) {
  bool ret = false;
  minimum_ratio = 1.0;
  // Execute the registered virtual bumpers in sequence and adopt the one that returns the smallest speed limit rate
  for (std::map<std::string, VirtualBumper::Ptr>::iterator it = bumper_list_.begin();
       it != bumper_list_.end(); ++it) {
    if (std::find(disable_bumpers.begin(),
                  disable_bumpers.end(),
                  it->first) == disable_bumpers.end()) {
      geometry_msgs::msg::PoseStamped obstacle_pose;
      obstacle_pose.header = observed_obstacle_pose.header;
      const double ratio = it->second.get()->LimitVelocityRatio(input_velocity, obstacle_pose);
      if (ratio < minimum_ratio) {
        minimum_ratio = ratio;
        observed_obstacle_pose = obstacle_pose;
        ret = true;
      }
    }
  }
  // Multiply the input speed by a factor to determine the output speed
  output_velocity = input_velocity;
  output_velocity.linear.x = output_velocity.linear.x * minimum_ratio;
  output_velocity.linear.y = output_velocity.linear.y * minimum_ratio;
  output_velocity.angular.z = output_velocity.angular.z * minimum_ratio;
  return ret;
}

/// Create an instance of a virtual bumper
VirtualBumper::Ptr BumperSet::CreateVirtualBumper(std::map<std::string, rclcpp::Parameter>& parameters) {
  std::string type_name;
  if (!GetParam(parameters, kTypeName, type_name)) {
    RCLCPP_ERROR(rclcpp::get_logger("safety_velocity_limiter"), "Mandatory parameter is not set: %s", kTypeName);
    return VirtualBumper::Ptr();
  }
  RCLCPP_INFO(rclcpp::get_logger("safety_velocity_limiter"), "Creating instance of \"%s\".", type_name.c_str());

  std::map<std::string, rclcpp::Parameter> slope_parameters;
  GetGroupParam(parameters, "velocity_slope", slope_parameters);
  VelocitySlope::Ptr velocity_slope = CreateVelocitySlope(slope_parameters);
  if (type_name == "TriangleBumper") {
    return TriangleBumper::Ptr(new TriangleBumper(parameters, velocity_slope));
  } else if (type_name == "CupBumper") {
    return CupBumper::Ptr(new CupBumper(parameters, velocity_slope));
  } else if (type_name == "EllipseBumper") {
    return EllipseBumper::Ptr(new EllipseBumper(parameters, velocity_slope));
  } else if (type_name == "OccupancyPointBumper") {
    return OccupancyPointBumper::Ptr(new OccupancyPointBumper(parameters, velocity_slope));
  } else if (type_name == "OccupancyEllipseBumper") {
    return OccupancyEllipseBumper::Ptr(new OccupancyEllipseBumper(parameters, velocity_slope));
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("safety_velocity_limiter"), "No such bumper type: \"%s\"", type_name.c_str());
    return VirtualBumper::Ptr();
  }
}

/// Create an instance of a speed gradient
VelocitySlope::Ptr BumperSet::CreateVelocitySlope(std::map<std::string, rclcpp::Parameter>& parameters) {
  std::string type_name;
  if (!GetParam(parameters, kTypeName, type_name)) {
    RCLCPP_ERROR(rclcpp::get_logger("safety_velocity_limiter"), "Mandatory parameter is not set: %s", kTypeName);
    return VelocitySlope::Ptr();
  }

  RCLCPP_INFO(rclcpp::get_logger("safety_velocity_limiter"), "Creating instance of \"%s\".", type_name.c_str());
  if (type_name == "FixedSlope") {
    return FixedSlope::Ptr(new FixedSlope(parameters));
  } else if (type_name == "LinearSlope") {
    return LinearSlope::Ptr(new LinearSlope(parameters));
  } else if (type_name == "LogarithmSlope") {
    return LogarithmSlope::Ptr(new LogarithmSlope(parameters));
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("safety_velocity_limiter"), "No such slope type: \"%s\"", type_name.c_str());
    return VelocitySlope::Ptr();
  }
}
}  // namespace tmc_safety_velocity_limiter
