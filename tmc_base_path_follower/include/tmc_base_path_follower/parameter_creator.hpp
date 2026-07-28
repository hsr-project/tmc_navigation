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
#ifndef TMC_BASE_PATH_FOLLOWER_PARAMETER_CREATOR_HPP_
#define TMC_BASE_PATH_FOLLOWER_PARAMETER_CREATOR_HPP_

#include <map>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include "goal_checker_diff_drive.hpp"
#include "goal_checker_omni.hpp"
#include "nearest_path_point_searcher.hpp"
#include "parameter_default_value.hpp"
#include "path_info_creator.hpp"
#include "path_transit_velocity_calculator.hpp"
#include "velocity_calculator_diff_drive.hpp"
#include "velocity_calculator_omni.hpp"

namespace tmc_base_path_follower {
// Retrieve required parameters
template<typename T>
bool GetParam(const rclcpp::Node::SharedPtr& node, const std::string& param_name, T& value) {
  rclcpp::Parameter param;
  if (!node->get_parameter(param_name, param)) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("base_path_follower"),
        "Parameter '" << param_name << "' is not specified.");
    return false;
  }
  value = param.get_value<T>();
  return true;
}

template<typename T>
bool GetParam(const std::map<std::string, rclcpp::Parameter>& group, const std::string& param_name, T& value) {
  auto it = group.find(param_name);
  if (it != group.end()) {
    value = it->second.get_value<T>();
    return true;
  } else {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("base_path_follower"),
        "Parameter '" << param_name << "' is not specified.");
    return false;
  }
}


// Retrieve optional parameters
template<typename T>
void GetOptionalParam(const rclcpp::Node::SharedPtr& node, const std::string& param_name, T& value,
                      const T& default_value) {
  if (!GetParam(node, param_name, value)) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("base_path_follower"),
        "Used default value: " << default_value);
    value = default_value;
  }
}

template<typename T>
void GetOptionalParam(const std::map<std::string, rclcpp::Parameter>& group, const std::string& param_name,
                      T& value, const T& default_value) {
  if (!GetParam(group, param_name, value)) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("base_path_follower"),
        "Used default value: " << default_value);
    value = default_value;
  }
}

// Retrieve parameter groups
inline bool GetGroupParam(const rclcpp::Node::SharedPtr& node, const std::string& group_name,
                          std::map<std::string, rclcpp::Parameter>& group) {
  const bool has_parameters = node->get_parameters(group_name, group);
  if (!has_parameters) {
    RCLCPP_WARN(rclcpp::get_logger("base_path_follower"),
        "Parameter '%s' group is not specified.", group_name.c_str());
  }
  return has_parameters;
}

inline bool GetGroupParam(const std::map<std::string, rclcpp::Parameter>& parameters, const std::string& group_name,
                          std::map<std::string, rclcpp::Parameter>& group) {
  const std::string prefix = group_name + ".";
  for (const auto& param : parameters) {
    if (param.first.find(prefix) == 0) {
      std::string param_name = param.first;
      param_name.erase(0, prefix.length());
      group.insert(std::make_pair(param_name, param.second));
    }
  }
  const bool ret = !(group.empty());
  if (!ret) {
    RCLCPP_WARN(rclcpp::get_logger("base_path_follower"),
        "Parameter '%s' group is not specified.", group_name.c_str());
  }
  return ret;
}

/// Generate parameters for the OmniGoalChecker class
OmniGoalChecker::Parameter CreateOmniGoalCheckerParameter(const rclcpp::Node::SharedPtr& node) {
  std::map<std::string, rclcpp::Parameter> omni_goal_checker_params;
  GetGroupParam(node, "omni_goal_checker", omni_goal_checker_params);
  double goal_area_length;
  GetOptionalParam(omni_goal_checker_params, "goal_area_length", goal_area_length, kGoalAreaLengthDefault);
  double goal_line_length;
  GetOptionalParam(omni_goal_checker_params, "goal_line_length", goal_line_length, kGoalLineLengthDefault);
  double goal_stop_error_length;
  GetOptionalParam(omni_goal_checker_params, "goal_stop_error_length",
      goal_stop_error_length, kGoalStopErrorLengthDefault);
  double goal_stop_error_angle;
  GetOptionalParam(omni_goal_checker_params, "goal_stop_error_angle",
      goal_stop_error_angle, kGoalStopErrorAngleDefault);
  return OmniGoalChecker::Parameter(goal_area_length, goal_line_length,
                                    goal_stop_error_length, goal_stop_error_angle);
}

/// Generate parameters for the DiffDriveGoalChecker class
DiffDriveGoalChecker::Parameter CreateDiffDriveGoalCheckerParameter(const rclcpp::Node::SharedPtr& node) {
  std::map<std::string, rclcpp::Parameter> diff_drive_goal_checker_params;
  GetGroupParam(node, "diff_drive_goal_checker", diff_drive_goal_checker_params);

  double goal_area_length;
  GetOptionalParam(diff_drive_goal_checker_params, "goal_area_length", goal_area_length, kGoalAreaLengthDefault);
  double goal_line_length;
  GetOptionalParam(diff_drive_goal_checker_params, "goal_line_length", goal_line_length, kGoalLineLengthDefault);
  double goal_stop_error_angle;
  GetOptionalParam(diff_drive_goal_checker_params, "goal_stop_error_angle",
      goal_stop_error_angle, kGoalStopErrorAngleDefault);
  return DiffDriveGoalChecker::Parameter(goal_area_length, goal_line_length, goal_stop_error_angle);
}

/// Generate parameters for the NearestPathPointSearcher class
NearestPathPointSearcher::Parameter CreateNearestPathPointSearcherParameter(const rclcpp::Node::SharedPtr& node) {
  std::map<std::string, rclcpp::Parameter> nearest_path_point_searcher_params;
  GetGroupParam(node, "nearest_path_point_searcher", nearest_path_point_searcher_params);

  double partial_search_range;
  GetOptionalParam(nearest_path_point_searcher_params, "partial_search_range",
      partial_search_range, kPartialSearchRangeDefault);
  double partial_search_permit_error;
  GetOptionalParam(nearest_path_point_searcher_params, "partial_search_permit_error",
      partial_search_permit_error, kPartialSearchPermitErrorDefault);
  return NearestPathPointSearcher::Parameter(partial_search_range, partial_search_permit_error);
}

/// Generate parameters for the PathInfoCreator class
PathInfoCreator::Parameter CreatePathInfoCreatorParameter(const rclcpp::Node::SharedPtr& node) {
  std::map<std::string, rclcpp::Parameter> path_info_creator_params;
  GetGroupParam(node, "path_info_creator", path_info_creator_params);

  int32_t interpolation_number;
  GetOptionalParam(path_info_creator_params, "interpolation_number",
      interpolation_number, kInterpolationNumberDefault);
  double max_linear_velocity;
  GetOptionalParam(path_info_creator_params, "max_linear_velocity",
      max_linear_velocity, kMaxLinearVelocityDefault);
  return PathInfoCreator::Parameter(interpolation_number, max_linear_velocity);
}

/// Generate parameters for the OmniVelocityCalculator class
OmniVelocityCalculator::Parameter CreateOmniVelocityCalculatorParameter(const rclcpp::Node::SharedPtr& node) {
  std::map<std::string, rclcpp::Parameter> omni_velocity_calculator_params;
  GetGroupParam(node, "omni_velocity_calculator", omni_velocity_calculator_params);

  double max_linear_velocity;
  GetOptionalParam(omni_velocity_calculator_params, "max_linear_velocity",
      max_linear_velocity, kMaxLinearVelocityDefault);
  double max_angular_velocity;
  GetOptionalParam(omni_velocity_calculator_params, "max_angular_velocity",
      max_angular_velocity, kMaxAngularVelocityDefault);
  double max_linear_acceleration;
  GetOptionalParam(omni_velocity_calculator_params, "max_linear_acceleration",
      max_linear_acceleration, kMaxLinearAccelerationDefault);
  double max_angular_acceleration;
  GetOptionalParam(omni_velocity_calculator_params, "max_angular_acceleration",
      max_angular_acceleration, kMaxAngularAccelerationDefault);
  double goal_deceleration;
  GetOptionalParam(omni_velocity_calculator_params, "linear_deceleration_near_goal",
      goal_deceleration, kGoalDecelerationDefault);
  double velocity_margin;
  GetOptionalParam(omni_velocity_calculator_params, "linear_velocity_margin",
      velocity_margin, kVelocityMarginDefault);
  double path_length_threshold;
  GetOptionalParam(omni_velocity_calculator_params, "path_length_threshold",
      path_length_threshold, kPathLengthThresholdDefault);
  double linear_p_gain;
  GetOptionalParam(omni_velocity_calculator_params, "linear_p_gain",
      linear_p_gain, kLinearPGainDefault);
  double angular_p_gain;
  GetOptionalParam(omni_velocity_calculator_params, "angular_p_gain",
      angular_p_gain, kAngularPGainDefault);
  double goal_angle_gain;
  GetOptionalParam(omni_velocity_calculator_params, "goal_angle_gain",
      goal_angle_gain, kGoalAngleGainDefault);
  return OmniVelocityCalculator::Parameter(max_linear_velocity, max_angular_velocity, max_linear_acceleration,
                                           max_angular_acceleration, goal_deceleration, velocity_margin,
                                           path_length_threshold, linear_p_gain, angular_p_gain, goal_angle_gain);
}

/// Generate parameters for the DiffDriveVelocityCalculator class
DiffDriveVelocityCalculator::Parameter CreateDiffDriveVelocityCalculatorParameter(const rclcpp::Node::SharedPtr& node) {
  std::map<std::string, rclcpp::Parameter> diff_drive_velocity_calculator_params;
  GetGroupParam(node, "diff_drive_velocity_calculator", diff_drive_velocity_calculator_params);

  double max_linear_velocity;
  GetOptionalParam(diff_drive_velocity_calculator_params, "max_linear_velocity",
      max_linear_velocity, kMaxLinearVelocityDefault);
  double max_angular_velocity;
  GetOptionalParam(diff_drive_velocity_calculator_params, "max_angular_velocity",
      max_angular_velocity, kMaxAngularVelocityDefault);
  double max_linear_acceleration;
  GetOptionalParam(diff_drive_velocity_calculator_params, "max_linear_acceleration",
      max_linear_acceleration, kMaxLinearAccelerationDefault);
  double max_angular_acceleration;
  GetOptionalParam(diff_drive_velocity_calculator_params, "max_angular_acceleration",
      max_angular_acceleration, kMaxAngularAccelerationDefault);
  double goal_deceleration;
  GetOptionalParam(diff_drive_velocity_calculator_params, "linear_deceleration_near_goal",
      goal_deceleration, kGoalDecelerationDefault);
  double velocity_margin;
  GetOptionalParam(diff_drive_velocity_calculator_params, "linear_velocity_margin",
      velocity_margin, kVelocityMarginDefault);
  double linear_alpha_gain;
  GetOptionalParam(diff_drive_velocity_calculator_params, "linear_alpha_gain",
      linear_alpha_gain, kLinearAlphaGainDefault);
  double linear_beta_gain;
  GetOptionalParam(diff_drive_velocity_calculator_params, "linear_beta_gain",
      linear_beta_gain, kLinearBetaGainDefault);
  double angle_error_angular_velocity_rate;
  GetOptionalParam(diff_drive_velocity_calculator_params, "angle_error_angular_velocity_rate",
      angle_error_angular_velocity_rate, kAngleErrorAngularVelocityRateDefault);
  double spin_start_error_angle;
  GetOptionalParam(diff_drive_velocity_calculator_params, "spin_start_error_angle",
      spin_start_error_angle, kSpinStartErrorAngleDefault);
  double spin_end_error_angle;
  GetOptionalParam(diff_drive_velocity_calculator_params, "spin_end_error_angle",
      spin_end_error_angle, kSpinEndErrorAngleDefault);
  double spin_max_angular_velocity;
  GetOptionalParam(diff_drive_velocity_calculator_params, "spin_max_angular_velocity",
      spin_max_angular_velocity, kSpinMaxAngularVelocityDefault);
  double spin_min_angular_velocity;
  GetOptionalParam(diff_drive_velocity_calculator_params, "spin_min_angular_velocity",
      spin_min_angular_velocity, kSpinMinAngularVelocityDefault);

  return DiffDriveVelocityCalculator::Parameter(
      max_linear_velocity, max_angular_velocity, max_linear_acceleration,
      max_angular_acceleration, goal_deceleration, velocity_margin,
      linear_alpha_gain, linear_beta_gain, angle_error_angular_velocity_rate,
      spin_start_error_angle, spin_end_error_angle,
      spin_max_angular_velocity, spin_min_angular_velocity);
}

/// Generate parameters for the PathTransitVelocityCalculator class
PathTransitVelocityCalculator::Parameter CreatePathTransitVelocityCalculatorParameter(
    const rclcpp::Node::SharedPtr& node) {
  std::map<std::string, rclcpp::Parameter> path_transit_velocity_calculator_params;
  GetGroupParam(node, "path_transit_velocity_calculator", path_transit_velocity_calculator_params);

  double max_linear_velocity;
  GetOptionalParam(path_transit_velocity_calculator_params, "max_linear_velocity",
      max_linear_velocity, kMaxLinearVelocityDefault);
  double max_angular_velocity;
  GetOptionalParam(path_transit_velocity_calculator_params, "max_angular_velocity",
      max_angular_velocity, kMaxAngularVelocityDefault);
  double max_linear_acceleration;
  GetOptionalParam(path_transit_velocity_calculator_params, "max_linear_acceleration",
      max_linear_acceleration, kMaxLinearAccelerationDefault);
  double max_linear_deceleration;
  GetOptionalParam(path_transit_velocity_calculator_params, "max_linear_deceleration",
      max_linear_deceleration, kMaxLinearDecelerationDefault);
  double max_angular_acceleration;
  GetOptionalParam(path_transit_velocity_calculator_params, "max_angular_acceleration",
      max_angular_acceleration, kMaxAngularAccelerationDefault);
  double max_angular_deceleration;
  GetOptionalParam(path_transit_velocity_calculator_params, "max_angular_deceleration",
      max_angular_deceleration, kMaxAngularDecelerationDefault);
  double min_linear_velocity;
  GetOptionalParam(path_transit_velocity_calculator_params, "min_linear_velocity",
      min_linear_velocity, kMinLinearVelocityDefault);
  double transit_velocity_angular_velocity_ratio;
  GetOptionalParam(path_transit_velocity_calculator_params, "transit_velocity_angular_velocity_ratio",
      transit_velocity_angular_velocity_ratio, kTransitVelocityAngularVelocityRatioDefault);

  return PathTransitVelocityCalculator::Parameter(
      max_linear_velocity, min_linear_velocity, max_angular_velocity,
      max_linear_acceleration, max_linear_deceleration,
      max_angular_acceleration, max_angular_deceleration,
      transit_velocity_angular_velocity_ratio);
}
}  // namespace tmc_base_path_follower

#endif  // TMC_BASE_PATH_FOLLOWER_PARAMETER_CREATOR_HPP_
