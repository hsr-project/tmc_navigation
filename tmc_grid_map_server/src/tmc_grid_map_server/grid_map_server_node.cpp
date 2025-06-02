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
/// @file     grid_map_server_node.cpp
/// @brief    Send map data for autonomous navigation
#include "grid_map_server_node.hpp"

#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include <libgen.h>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <SDL/SDL_image.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <wordexp.h>
#include <yaml-cpp/yaml.h>
#include <tmc_pose_2d_lib/distance_map.hpp>
#include <tmc_pose_2d_lib/ros_if.hpp>

#include "param.hpp"
#include "util.hpp"

namespace {
/// Grid size
const char* const kParameterNameBaseGridSize = "base_grid_size";
/// Map origin position (x)
const char* const kParameterNameMapOriginX = "map_origin_x";
/// Map origin position (y)
const char* const kParameterNameMapOriginY = "map_origin_y";
/// Map origin position (yaw)
const char* const kParameterNameMapOriginYaw = "map_origin_yaw";
/// Map file path
const char* const kParameterNameBaseMapFilePath = "map_file_path";
/// Default grid size [m/grid]
double const kDefaultBaseGridSize = 0.05;
/// Default map origin (x) [m]
double const kDefaultMapOriginX = 0.0;
/// Default map origin (y) [m]
double const kDefaultMapOriginY = 0.0;
/// Default map origin (yaw) [rad]
double const kDefaultMapOriginYaw = 0.0;
/// Topic name for static distance map (tmc_navigation_msgs/OccupancyGridUint type)
const char* const kPublishTopicNameDistanceMap = "static_distance_map";
/// Topic name for static obstacle map (tmc_navigation_msgs/OccupancyGridUint type)
const char* const kPublishTopicNameObstacleMap = "static_obstacle_map";
/// Topic name for static distance map (nav_msgs/OccupancyGrid type)
const char* const kPublishTopicNameDistanceRosMap = "static_distance_ros_map";
/// Topic name for static obstacle map (nav_msgs/OccupancyGrid type)
const char* const kPublishTopicNameObstacleRosMap = "static_obstacle_ros_map";
/// Service name for map reloading
const char* const kServiceNameReloadMap = "reload_map";
/// Map frame name
const char* const kMapFrameName = "map";
/// Default value for potential width [m]
double const kDefaultPotentialWidth = 3.0;
/// Default value for converting Unknown to Free
const bool kDefaultConvertUnknownToFree = true;
/// Map topic name for online map updates
const char* const kDefaultSubscribeMapName = "update_map";
}  // namespace

namespace tmc_grid_map_server {
using std::placeholders::_1;
using std::placeholders::_2;
GridMapServerNode::GridMapServerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("grid_map_server", options),
      base_grid_size_(kDefaultBaseGridSize),
      width_(0),
      height_(0),
      map_origin_x_(0.0),
      map_origin_y_(0.0),
      map_origin_yaw_(0.0),
      base_map_file_path_(""),
      frame_id_(""),
      negate_(0),
      occupied_thresh_(0.0),
      free_thresh_(0.0),
      potential_width_(0.0),
      convert_unknown_to_free_(true) {}

// Initialization
void GridMapServerNode::Init() {
  std::string map_yaml_path;
  if (!GetParam(shared_from_this(), "map_yaml_path", map_yaml_path)) {
    std::string error_msg("Parameter of map_yaml_path is must be set.");
    throw std::runtime_error(error_msg);
  }
  // Obtain parameters from YAML file
  if (!LoadConfig(map_yaml_path)) {
    std::string error_msg("Failed to read map.yaml");
    throw std::runtime_error(error_msg);
  }

  // Set parameters
  GetOptionalParam(shared_from_this(), "potential_width", potential_width_, kDefaultPotentialWidth);
  GetOptionalParam(shared_from_this(), "convert_unknown_to_free", convert_unknown_to_free_,
                   kDefaultConvertUnknownToFree);
  // Configure Publisher, Subscriber
  pub_distance_map_ = this->create_publisher<tmc_navigation_msgs::msg::OccupancyGridUint>(
      kPublishTopicNameDistanceMap, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  pub_obstacle_map_ = this->create_publisher<tmc_navigation_msgs::msg::OccupancyGridUint>(
      kPublishTopicNameObstacleMap, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  pub_distance_ros_map_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      kPublishTopicNameDistanceRosMap, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  pub_obstacle_ros_map_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      kPublishTopicNameObstacleRosMap, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  sub_map_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(kDefaultSubscribeMapName, 1,
      std::bind(&GridMapServerNode::CallbackMap, this, _1));
  // Configure ServiceServer
  reload_map_ =  this->create_service<tmc_navigation_msgs::srv::ReloadMap>(
      kServiceNameReloadMap, std::bind(&GridMapServerNode::CallbackServiceSetGoal, this, _1, _2));
}

// Subscribe to map topic for online map updates
void GridMapServerNode::CallbackMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  PublishMaps(*msg, *msg);
  RCLCPP_INFO(rclcpp::get_logger("grid_map_server"), "update static_distance_map");
}

// Map reload service callback
void GridMapServerNode::CallbackServiceSetGoal(
    tmc_navigation_msgs::srv::ReloadMap::Request::SharedPtr req,
    tmc_navigation_msgs::srv::ReloadMap::Response::SharedPtr res) {
  try {
    // Obtain map configuration for reloading from YAML file
    if (!LoadConfig(req->new_map_yaml)) {
      std::string error_msg("Failed to read config file of new map.");
      throw std::runtime_error(error_msg);
    }
    // Generate and distribute map
    CreateMapsAndPublish();
  } catch (const std::exception& e) {
    // Return false in service response if map reload fails
    RCLCPP_ERROR(rclcpp::get_logger("grid_map_server"), "%s", e.what());
    res->is_success = false;
    return;
  }
  // Return true in service response if map reload succeeds
  res->is_success = true;
}

/// Load map configuration file (map.yaml)
bool GridMapServerNode::LoadConfig(const std::string& config_name) {
  // Allow loading home with ~/
  wordexp_t exp_result;
  wordexp(config_name.c_str(), &exp_result, 0);
  std::string full_path_config_name(exp_result.we_wordv[0]);
  wordfree(&exp_result);
  std::ifstream fin(full_path_config_name.c_str());
  if (fin.fail()) {
    RCLCPP_ERROR(rclcpp::get_logger("grid_map_server"), "Could not open %s", full_path_config_name.c_str());
    return false;
  }
  YAML::Node config = YAML::Load(fin);
  if (config["resolution"]) {
    base_grid_size_ = config["resolution"].as<double>();
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("grid_map_server"), "Could not read 'resolution' of map.yaml");
    return false;
  }

  if (config["negate"]) {
    negate_ = config["negate"].as<int32_t>();
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("grid_map_server"), "Could not read 'negate' of map.yaml");
    return false;
  }

  if (config["occupied_thresh"]) {
    occupied_thresh_ = config["occupied_thresh"].as<double>();
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("grid_map_server"), "Could not read 'occupied_thresh' of map.yaml");
    return false;
  }

  if (config["free_thresh"]) {
    free_thresh_ = config["free_thresh"].as<double>();
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("grid_map_server"), "Could not read 'free_thresh' of map.yaml");
    return false;
  }

  if (config["origin"]) {
    map_origin_x_ = config["origin"][0].as<double>();
    map_origin_y_ = config["origin"][1].as<double>();
    map_origin_yaw_ = config["origin"][2].as<double>();
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("grid_map_server"), "Could not read 'origin' of map.yaml");
    return false;
  }

  if (config["image"]) {
    std::string filename("");
    filename = config["image"].as<std::string>();
    if (filename[0] != '/') {
      char* config_name_copy = strdup(full_path_config_name.c_str());
      filename = std::string(dirname(config_name_copy)) + '/' + filename;
      free(config_name_copy);
    }
    distance_map_file_ = filename;
    obstacle_map_file_ = filename;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("grid_map_server"), "Could not read 'image' of map.yaml");
    return false;
  }
  // Prioritize distance_map if specified
  if (config["distance_map"]) {
    std::string filename("");
    filename = config["distance_map"].as<std::string>();
    if (filename[0] != '/') {
      char* config_name_copy = strdup(full_path_config_name.c_str());
      filename = std::string(dirname(config_name_copy)) + '/' + filename;
      free(config_name_copy);
    }
    distance_map_file_ = filename;
    RCLCPP_DEBUG(rclcpp::get_logger("grid_map_server"), "distance_map found");
  }

  // Prioritize obstacle_map if specified
  if (config["obstacle_map"]) {
    std::string filename("");
    filename = config["obstacle_map"].as<std::string>();
    if (filename[0] != '/') {
      char* config_name_copy = strdup(full_path_config_name.c_str());
      filename = std::string(dirname(config_name_copy)) + '/' + filename;
      free(config_name_copy);
    }
    obstacle_map_file_ = filename;

    RCLCPP_DEBUG(rclcpp::get_logger("grid_map_server"), "obstacle_map found");
  }
  return true;
}

/// Load data from map file
void GridMapServerNode::LoadMap(const std::string& file_path, nav_msgs::msg::OccupancyGrid& map) {
  SDL_Surface* map_image;
  // Load data from pgm file
  map_image = IMG_Load(file_path.c_str());
  if (!map_image) {
    // Throw an exception if loading fails
    std::stringstream error_msg;
    error_msg << "failed to open image file: " << file_path << std::endl;
    throw std::runtime_error(error_msg.str());
  }
  uint8_t* pixels = static_cast<uint8_t*>(map_image->pixels);
  uint8_t* p;
  // Data loaded from pgm file has the top-left of the image as the origin
  // OccupancyGrid sets the origin to the bottom-left corner of the image, so height is stored in reverse order
  for (int32_t j = (static_cast<int32_t>(map_image->h) - 1); j >= 0; --j) {
    for (int32_t i = 0; i <= (static_cast<int32_t>(map_image->w) - 1); ++i) {
      p = pixels + (j * map_image->pitch) + (i * map_image->format->BytesPerPixel);
      // Convert pgm values to three values of OccupancyGrid and store
      map.data.push_back(ConvertPgmToOccupancyGridTrinaryValue(*p));
    }
  }
  width_ = map_image->w;
  height_ = map_image->h;

  // Release loaded data
  SDL_FreeSurface(map_image);
}

/// Convert pgm values to three values of OccupancyGrid
int8_t GridMapServerNode::ConvertPgmToOccupancyGridTrinaryValue(const uint8_t pgm_value) {
  // Convert to three values: WALL, FREE, UNKNOWN
  // The smaller the pgm value, the higher the occupancy rate; the larger, the lower the occupancy rate
  const uint8_t free_threshold = static_cast<uint8_t>((1.0 - free_thresh_) * kPGM_FREE);
  const uint8_t occupied_threshold = static_cast<uint8_t>((1.0 - occupied_thresh_) * kPGM_FREE);
  int8_t ros_value;
  if (pgm_value < occupied_threshold) {
    // If occupancy rate is higher than occupied_threshold, it's WALL
    ros_value = kROS_WALL;
  } else if (pgm_value > free_threshold || convert_unknown_to_free_) {
    // If occupancy rate is lower than free_threshold, it's FREE
    // If convert_unknown_to_free_ is true, all except WALL are set to FREE
    ros_value = kROS_FREE;
  } else {
    // When convert_unknown_to_free_ is false
    // If occupancy rate is higher than free_threshold and lower than occupied_threshold, it's UNKNOWN
    ros_value = kROS_UNKNOWN;
  }
  return ros_value;
}

/// Set map metadata
void GridMapServerNode::SetMapMetaData(nav_msgs::msg::OccupancyGrid& map) {
  map.header.stamp = this->now();
  map.header.frame_id = kMapFrameName;
  map.info.resolution = base_grid_size_;
  map.info.width = width_;
  map.info.height = height_;
  map.info.origin.position.x = map_origin_x_;
  map.info.origin.position.y = map_origin_y_;
  map.info.origin.position.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, map_origin_yaw_);
  map.info.origin.orientation = tf2::toMsg(q);
}

// Generate map
void GridMapServerNode::CreateMap(const std::string& map_file, nav_msgs::msg::OccupancyGrid& map) {
  // Load map file
  LoadMap(map_file, map);
  // Set metadata
  SetMapMetaData(map);
}

/// Node main processing
void GridMapServerNode::Run() {
  // Generate and distribute potential map
  CreateMapsAndPublish();
}

/// Generate and distribute map
void GridMapServerNode::CreateMapsAndPublish(void) {
  // Generate distance map
  nav_msgs::msg::OccupancyGrid distance_map;
  CreateMap(distance_map_file_, distance_map);
  // Generate obstacle map
  nav_msgs::msg::OccupancyGrid obstacle_map;
  CreateMap(obstacle_map_file_, obstacle_map);
  // Publish map
  PublishMaps(distance_map, obstacle_map);
}

/// Publish distance map and obstacle map. Publish both TMC format and ROS format
void GridMapServerNode::PublishMaps(const nav_msgs::msg::OccupancyGrid& distance_map,
                                    const nav_msgs::msg::OccupancyGrid& obstacle_map) {
  // Publish ROS format map
  // Distance map
  pub_distance_ros_map_->publish(distance_map);
  // Obstacle map
  pub_obstacle_ros_map_->publish(obstacle_map);

  // Convert to TMC format expanded map and publish
  // TODO(kazuhito_tanaka) 自律移動システムがTMC形式を使用しなくなったタイミングで削除
  // Distance map
  tmc_navigation_msgs::msg::OccupancyGridUint distance_tmc_map;
  // Since ROS format map is already three-value converted, specify 1.0 for occupied_threshold to avoid unnecessary conversion
  if (CreateTmcPotentialMap(distance_map, potential_width_, 1.0, distance_tmc_map)) {
    pub_distance_map_->publish(distance_tmc_map);
  }
  // Obstacle map
  tmc_navigation_msgs::msg::OccupancyGridUint obstacle_tmc_map;
  // Since ROS format map is already three-value converted, specify 1.0 for occupied_threshold to avoid unnecessary conversion
  if (CreateTmcPotentialMap(obstacle_map, potential_width_, 1.0, obstacle_tmc_map)) {
    pub_obstacle_map_->publish(obstacle_tmc_map);
  }
}
}  // namespace tmc_grid_map_server
