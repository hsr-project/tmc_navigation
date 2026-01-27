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
/// @file logger-test.cpp
/// @brief Test of the log recording class
#include <string>
#include <vector>

#include <boost/filesystem.hpp>
#include <gtest/gtest.h>
#include <ros/file_log.h>
#include <ros/ros.h>
#include <tmc_rostest_utils/util_function.hpp>
#include "../include/tmc_move_base/logger.hpp"

namespace fs = boost::filesystem;
namespace tmc_move_base {
// Test fixture
class LoggerTest : public ::testing::Test {
 public:
  LoggerTest()  {
    const std::string ros_log_dir = ros::file_log::getLogDirectory();
    output_log_directory_ = ros_log_dir + "/log";
    ros::NodeHandle private_nh("~");
    std::string service_name;
    private_nh.getParam("navigation_log_record_service_name", service_name);
    // Wait for the service to start
    if (!ros::service::waitForService(service_name, ros::Duration(5.0))) {
      const std::string error_msg = service_name + " service not exist.";
      throw std::runtime_error(error_msg);
    }
  }
  ~LoggerTest() {}
  virtual void SetUp() {
    // Create a directory for log output
    fs::create_directories(output_log_directory_);
    // Grant write permissions
    fs::perms perm = fs::status(output_log_directory_).permissions();
    fs::perms write_perm = perm | fs::perms::group_write | fs::perms::others_write;
    fs::permissions(output_log_directory_, write_perm);
  }
  virtual void TearDown() {
    // Delete the directory for log output
    fs::remove_all(output_log_directory_);
  }

 protected:
  /// Check if a file exists in the directory
  /// Since the name is checked with find, the file name will hit with partial match
  /// @param [I] directory_path Path of the directory
  /// @param [I] file_name File name
  bool FindFileInDirectory(const std::string& directory_path, const std::string& file_name) {
    for (const fs::directory_entry& files : fs::directory_iterator(directory_path)) {
      if (files.path().string().find(directory_path + "/" + file_name) != std::string::npos) {
        return true;
      }
    }
    return false;
  }

  /// Format the file name according to the Logger's rules
  /// Although it is pointless as a test because it only writes the same process as Logger,
  /// it is done because it may fail depending on the namespace or parameters of the test node
  /// @param [I] file_name File name
  std::string FormatFileNames(const std::string& file_name) {
    std::string format_file_name = file_name;
    // Delete if the beginning is '/'
    if (format_file_name[0] == '/') {
      format_file_name.erase(0, 1);
    }
    // Replace intermediate '/' with '_'
    std::string::size_type pos(format_file_name.find('/'));
    while (pos != std::string::npos) {
      format_file_name.replace(pos, 1, "_");
      pos = format_file_name.find('/', pos + 1);
    }
    return format_file_name;
  }
  std::string output_log_directory_;
};

/// Normal test of RecordLog
/// Confirm that the log file that should be recorded is output
/// The contents of the log file are not checked because they depend on XmlRpcValue or external services
TEST_F(LoggerTest, RecordLog) {
  ros::NodeHandle private_nh("~");
  Logger logger(output_log_directory_);
  logger.RecordLog();

  // Confirm that each node and its parameter file are output
  std::vector<std::string> node_names;
  private_nh.getParam("record_parameter_node_names", node_names);
  node_names.push_back(private_nh.getNamespace());
  for (std::vector<std::string>::const_iterator names_it = node_names.begin();
       names_it != node_names.end(); ++names_it) {
    const std::string file_name = FormatFileNames(*names_it);
    EXPECT_TRUE(tmc_rostest_utils::WaitUntil([&](){
        return FindFileInDirectory(output_log_directory_, file_name); }, 1.0));
  }
  // Confirm that the bag file is output
  // Specify a longer wait time because the output of the bag file takes time
  EXPECT_TRUE(tmc_rostest_utils::WaitUntil([&](){
      return FindFileInDirectory(output_log_directory_, "navigation_log_recorder-service_trigger"); }, 5.0));
}

/// Test that logs are not recorded if parameters are not specified
/// If the Log recording service name is not registered in the parameters, logs are not recorded
TEST_F(LoggerTest, LogRecordServiceIsNotSpecified) {
  ros::NodeHandle private_nh("~");
  std::string default_service_name;
  // Keep the original parameters to restore later
  private_nh.getParam("navigation_log_record_service_name", default_service_name);
  // Delete the service name parameter
  private_nh.deleteParam("navigation_log_record_service_name");
  Logger logger(output_log_directory_);
  logger.RecordLog();

  // Confirm that each node and its parameter file are not output
  std::vector<std::string> node_names;
  private_nh.getParam("record_parameter_node_names", node_names);
  node_names.push_back(private_nh.getNamespace());
  for (std::vector<std::string>::const_iterator names_it = node_names.begin();
       names_it != node_names.end(); ++names_it) {
    const std::string file_name = FormatFileNames(*names_it);
    EXPECT_FALSE(tmc_rostest_utils::WaitUntil([&](){
        return FindFileInDirectory(output_log_directory_, file_name); }, 1.0));
  }
  // Confirm that the bag file is not output
  // Specify a longer wait time because the output of the bag file takes time
  EXPECT_FALSE(tmc_rostest_utils::WaitUntil([&](){
      return FindFileInDirectory(output_log_directory_, "navigation_log_recorder-service_trigger"); }, 5.0));
  // Restore deleted parameters to avoid affecting the next test
  private_nh.setParam("navigation_log_record_service_name", default_service_name);
}

/// Test to confirm the existence of the output destination directory
/// If the output destination directory does not exist, the constructor throws an exception
TEST_F(LoggerTest, OutputDirectoryIsNotExist) {
  EXPECT_THROW(Logger logger(output_log_directory_ + "_hoge"), std::runtime_error);
}

/// Test to check permissions of the output destination directory
/// If there are no write permissions in the output destination directory, the constructor throws an exception
/// If there is either owner_write, others_write, or group_write permission, no exception is thrown
TEST_F(LoggerTest, OutputDirectoryPermission) {
  // Get permissions of the output destination directory
  fs::perms perm = fs::status(output_log_directory_).permissions();
  // It becomes an exception if there are no write permissions
  fs::perms no_write_perm = perm & ~(fs::perms::owner_write | fs::perms::group_write | fs::perms::others_write);
  fs::permissions(output_log_directory_, no_write_perm);
  EXPECT_THROW(Logger logger(output_log_directory_), std::runtime_error);

  // No exception if others_write is present
  fs::perms others_write_perm = no_write_perm | fs::perms::others_write;
  fs::permissions(output_log_directory_, others_write_perm);
  EXPECT_NO_THROW(Logger logger(output_log_directory_));

  // No exception if group_write is present
  fs::perms group_write_perm = no_write_perm | fs::perms::group_write;
  fs::permissions(output_log_directory_, group_write_perm);
  EXPECT_NO_THROW(Logger logger(output_log_directory_));

  // No exception if owner_write is present
  fs::perms owner_write_perm = no_write_perm | fs::perms::owner_write;
  fs::permissions(output_log_directory_, owner_write_perm);
  EXPECT_NO_THROW(Logger logger(output_log_directory_));
}

}  // namespace tmc_move_base

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "logger_test");
  testing::InitGoogleTest(&argc, argv);
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
