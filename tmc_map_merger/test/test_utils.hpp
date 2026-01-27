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
#ifndef TMC_MAP_MERGER_TEST_UTILS_HPP_
#define TMC_MAP_MERGER_TEST_UTILS_HPP_
#include <memory>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rcl_yaml_param_parser/parser.h>
#include <rclcpp/parameter_map.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>

namespace tmc_map_merger {
// TODO(syuuhei_shiro): tmc_rostest_utilをROS2化してそこに置く
// Load parameters from yaml file
void LoadParameterFromYaml(std::shared_ptr<rclcpp::Node> node,
    const std::string& yaml_directory, const std::string& yaml_name) {
  const std::string yaml_path = yaml_directory + yaml_name;
  // Load yaml and generate ParameterMap
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rcl_params_t* yaml_params = rcl_yaml_node_struct_init(allocator);
  rcl_parse_yaml_file(yaml_path.c_str(), yaml_params);
  rclcpp::ParameterMap yaml_param_map = rclcpp::parameter_map_from(yaml_params);
  rcl_yaml_node_struct_fini(yaml_params);
  // Set ros parameters to node
  const std::string parameter_space = "/" + std::string(node->get_name());
  auto iter = yaml_param_map.find(parameter_space);
  for (auto& param : iter->second) {
    node->declare_parameter(param.get_name(), param.get_type());
    node->set_parameter(param);
  }
}

// Generate node that reads test parameters
std::shared_ptr<rclcpp::Node> CreateParameterNode(const std::string& yaml_name) {
  std::shared_ptr<rclcpp::Node> node;
  rclcpp::NodeOptions option;
  option.allow_undeclared_parameters();
  option.automatically_declare_parameters_from_overrides(true);
  node = std::make_shared<rclcpp::Node>("test_node", option);
  const std::string yaml_directory = ament_index_cpp::get_package_share_directory("tmc_map_merger") +
      "/test/parameter/";
  LoadParameterFromYaml(node, yaml_directory, yaml_name);
  return node;
}
}  // namespace tmc_map_merger
#endif  // TMC_MAP_MERGER_TEST_UTILS_HPP_
