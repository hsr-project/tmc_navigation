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
/// @file logger.hpp
/// @brief Record logs
#include "tmc_move_base/logger.hpp"

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/filesystem.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>

namespace fs = boost::filesystem;
namespace tmc_move_base {
#if 0  // TODO(syuuhei_shiro): LoggerをROS2化する
// Constructor
Logger::Logger(const std::string& output_log_directory) : output_log_directory_(output_log_directory) {
  ros::NodeHandle nh;
  // Read parameters
  ros::NodeHandle private_nh("~");
  // rosbag recording service name
  std::string record_rosbag_service_name;
  if (private_nh.getParam("navigation_log_record_service_name", record_rosbag_service_name)) {
    // Service client setup
    record_rosbag_client_ = nh.serviceClient<std_srvs::Empty>(record_rosbag_service_name);
    // Check directory existence
    if (!fs::exists(output_log_directory_)) {
      const std::string msg = "Specified directory is not exist. Please specify other directory";
      throw std::runtime_error(msg);
    }
    // Check directory write permissions
    fs::perms perm = fs::status(output_log_directory_).permissions();
    if ((perm & (fs::perms::owner_write | fs::perms::group_write | fs::perms::others_write)) == fs::perms::no_perms) {
      const std::string msg = "No write permission for the specified directory. Please specify other directory";
      throw std::runtime_error(msg);
    }
    // Get the list of node names to record parameters
    private_nh.getParam("record_parameter_node_names", record_parameter_node_names_);
    // Add itself to the list
    record_parameter_node_names_.push_back(private_nh.getNamespace());
  }
}

// Record logs
void Logger::RecordLog() {
  if (record_rosbag_client_.exists()) {
    // Request rosbag recording
    std_srvs::Empty srv_empty;
    record_rosbag_client_.call(srv_empty);
    // Dump parameters
    // Since rosbag and parameters are a set, if the rosbag recording service does not exist, do not dump parameters either
    boost::posix_time::ptime now = boost::posix_time::second_clock::local_time();
    std::string now_as_iso = boost::posix_time::to_iso_string(now);
    DumpRosparam(output_log_directory_, now_as_iso, record_parameter_node_names_);
  }
}

// Dump parameters of the specified node
// Output to the specified output directory with the file name '(node_name)_params_(timestamp_string).yaml'
// If the node name starts with '/', remove it from the file name. If '/' is included in the middle, replace it with '_'
// @param[I] output_directory Output directory
// @param[I] timestamp Timestamp string
// @param[I] node_names List of node names
void Logger::DumpRosparam(const std::string& output_directory, const std::string& timestamp,
                          const std::vector<std::string>& node_names) {
  ros::NodeHandle nh;
  for (std::vector<std::string>::const_iterator node_names_it = node_names.begin();
       node_names_it != node_names.end(); ++node_names_it) {
    // Generate file name
    std::string file_name = *node_names_it;
    // Remove if it starts with '/'
    if (file_name[0] == '/') {
      file_name.erase(0, 1);
    }
    // Replace '/' in the middle with '_'
    std::string::size_type pos(file_name.find('/'));
    while (pos != std::string::npos) {
      file_name.replace(pos, 1, "_");
      pos = file_name.find('/', pos + 1);
    }
    file_name = output_directory + "/" + file_name + "_params_" + timestamp + ".yaml";

    // Read parameters and output to file
    XmlRpc::XmlRpcValue param;
    nh.getParam(*node_names_it, param);
    std::stringstream ss;
    param.write(ss);
    std::ofstream write_file;
    write_file.open(file_name, std::ios::out);
    write_file << ss.str() << std::endl;
    write_file.close();
  }
}
#endif
}  // namespace tmc_move_base

