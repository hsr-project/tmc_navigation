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
/// @file param.hpp
/// @brief Parameter retrieval
#ifndef TMC_POINT_CLOUD_ACCUMULATOR_PARAM_HPP_
#define TMC_POINT_CLOUD_ACCUMULATOR_PARAM_HPP_
#include <map>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>

namespace tmc_point_cloud_accumulator {
// TODO(syuuhei_shiro): パラメータ取得関数は共通パッケージに置く
// Retrieval of required parameters
template<typename T>
bool GetParam(const rclcpp::Node::SharedPtr& node, const std::string& param_name, T& value) {
  rclcpp::Parameter param;
  if (!node->get_parameter(param_name, param)) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("point_cloud_accumulator"),
        "Parameter '" << param_name << "' is not specified.");
    return false;
  }
  value = param.get_value<T>();
  return true;
}

// Retrieval of optional parameters
template<typename T>
void GetOptionalParam(const rclcpp::Node::SharedPtr& node, const std::string& param_name, T& value,
                      const T& default_value) {
  if (!GetParam(node, param_name, value)) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("point_cloud_accumulator"),
        "Used default value: " << default_value);
    value = default_value;
  }
}
}  // namespace tmc_point_cloud_accumulator
#endif  // TMC_POINT_CLOUD_ACCUMULATOR_PARAM_HPP_
