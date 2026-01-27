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
/// @file logger.hpp
/// @brief Record logs of autonomous movement
#ifndef TMC_MOVE_BASE_LOGGER_HPP_
#define TMC_MOVE_BASE_LOGGER_HPP_
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace tmc_move_base {
// TODO(syuuhei_shiro): LoggerをROS2化する
#if 0
// Log recording class
class Logger {
 public:
  /// Constructor
  explicit Logger(const std::string& output_log_directory);
  ~Logger() {}

  /// Record logs
  /// When called, request the rosbag recording service,
  /// Dump the nodes specified by parameters and its own parameters
  void RecordLog();

 private:
  /// Dump parameters of the specified nodes
  /// Output to the specified output directory with the file name '(node name)_params_(timestamp string).yaml'
  /// If the node name starts with '/', remove it from the file name. If '/' is included in the middle, replace it with '_'
  /// @param[I] output_directory Output destination directory
  /// @param[I] timestamp Timestamp string
  /// @param[I] node_names List of node names
  void DumpRosparam(const std::string& output_directory, const std::string& timestamp,
                    const std::vector<std::string>& node_names);

  // rosbag recording service client
  ros::ServiceClient record_rosbag_client_;
  // List of node names to record parameters
  std::vector<std::string> record_parameter_node_names_;
  // log output directory
  std::string output_log_directory_;
};
#endif
}  // namespace tmc_move_base

#endif  // TMC_MOVE_BASE_LOGGER_HPP_
