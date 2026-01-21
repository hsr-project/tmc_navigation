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
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "map_converter.hpp"
#include "map_input.hpp"
#include "param.hpp"
#include "point_cloud_filter.hpp"

namespace {
std::vector<tmc_map_merger::PointCloudFilter::Ptr> CreateFilters(
    const std::map<std::string, rclcpp::Parameter>& parameters) {
  std::map<std::string, rclcpp::Parameter> point_cloud_filter_params;
  GetRequiredGroupParam(parameters, "point_cloud_filters", point_cloud_filter_params);
  std::vector<std::string> filter_names;
  std::vector<std::tuple<int, std::map<std::string, rclcpp::Parameter>>> ordered_filters;
  for (const auto & point_cloud_filter_param : point_cloud_filter_params) {
    std::string filter_name = point_cloud_filter_param.first;
    filter_name.erase(filter_name.find("."));
    if (std::find(filter_names.begin(), filter_names.end(), filter_name) == filter_names.end()) {
      filter_names.push_back(filter_name);
      std::map<std::string, rclcpp::Parameter> filter_param;
      GetRequiredGroupParam(point_cloud_filter_params, filter_name, filter_param);

      // Since the constraint of the parser from ROS2 prevents point_cloud_filters from being applied as an array, explicitly load the processing order
      int order;
      GetOptionalParam(filter_param, "order", order, 9999, NotLess<int>(0));

      ordered_filters.emplace_back(order, filter_param);
    }
  }

  std::sort(
      ordered_filters.begin(), ordered_filters.end(),
      [](const auto& a, const auto& b) {
        return std::get<0>(a) < std::get<0>(b);
      });

  std::vector<tmc_map_merger::PointCloudFilter::Ptr> point_cloud_filter_ptrs;
  for (const auto& [order, filter_param] : ordered_filters) {
    tmc_map_merger::PointCloudFilter::Ptr ptr =
        tmc_map_merger::PointCloudFilterFactory::Create(filter_param);
    point_cloud_filter_ptrs.push_back(ptr);
  }

  return point_cloud_filter_ptrs;
}

}  // anonymous namespace

namespace tmc_map_merger {

MapInput::Ptr MapInputFactory::Create(const std::string& input_name,
                                      const std::map<std::string, rclcpp::Parameter>& parameters,
                                      rclcpp::Node::SharedPtr node, MapMerger::Ptr& merger) {
  MapInput::Ptr map_input;
  std::string type;
  double rate;
  GetRequiredParam(parameters, "type", type);
  GetOptionalParam(parameters, "rate", rate, 0.0, NotLess<double>(0.0));

  if (type == "nav_msgs/OccupancyGrid") {
    std::string topic_name;
    GetRequiredParam(parameters, "topic_name", topic_name);
    MapConverter<nav_msgs::msg::OccupancyGrid> converter;
    map_input.reset(new Subscriber<nav_msgs::msg::OccupancyGrid>(
        node, input_name, topic_name, rate, converter, merger));
  } else if (type == "sensor_msgs/LaserScan") {
    std::string topic_name;
    GetRequiredParam(parameters, "topic_name", topic_name);
    std::vector<PointCloudFilter::Ptr> point_cloud_filter_ptrs = CreateFilters(parameters);
    MapConverter<sensor_msgs::msg::LaserScan> converter(node, input_name, point_cloud_filter_ptrs);
    map_input.reset(new Subscriber<sensor_msgs::msg::LaserScan>(
        node, input_name, topic_name, rate, converter, merger));
  } else if (type == "sensor_msgs/PointCloud2") {
    std::string topic_name;
    GetRequiredParam(parameters, "topic_name", topic_name);
    std::vector<PointCloudFilter::Ptr> point_cloud_filter_ptrs = CreateFilters(parameters);
    MapConverter<sensor_msgs::msg::PointCloud2> converter(node, input_name, point_cloud_filter_ptrs);
    map_input.reset(new Subscriber<sensor_msgs::msg::PointCloud2>(
        node, input_name, topic_name, rate, converter, merger));
  } else {
    const std::string message = std::string("Unknown topic type: ") + type;
    throw std::logic_error(message);
  }
  return map_input;
}

}  // namespace tmc_map_merger
