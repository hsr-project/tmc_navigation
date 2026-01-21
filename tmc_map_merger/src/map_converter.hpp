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
#ifndef TMC_MAP_MERGER_MAP_MERGER_MAP_CONVERTER_HPP_
#define TMC_MAP_MERGER_MAP_MERGER_MAP_CONVERTER_HPP_
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_eigen/tf2_eigen.h>

#include "map.hpp"
#include "map_drawer.hpp"
#include "param.hpp"
#include "point_cloud_filter.hpp"

namespace { // NOLINT
// Maximum occupancy probability
const int kMaxOccupancy = 100;
}  // anonymous namespace

namespace tmc_map_merger {

struct Obstacle {
  int32_t count;
  bool raycast;
};

/// @brief Voting Adapter
struct ObstacleAdapter {
  typedef Obstacle DataType;
  void Get(Obstacle& dst, const Obstacle& src) const {
    dst = src;
  }
  void Update(Obstacle& dst, int32_t obstacle) const {
    dst.count += obstacle;
    dst.raycast = true;
  }
  void Reset(Obstacle& dst) const {
    dst.count = 0;
    dst.raycast = false;
  }
};

typedef MapOperator<ObstacleAdapter, AnyMap<Obstacle> > ObstacleMapOperator;

// @brief
template<typename T>
class MapConverter {
 public:
  geometry_msgs::msg::PoseStamped GetOrigin(const T& obj) const {}
  template<typename MapOp>
  void Convert(const geometry_msgs::msg::Pose& origin,
               const T& obj, const MapOp& op) const {}
};

template<>
class MapConverter<nav_msgs::msg::OccupancyGrid> {
 public:
  geometry_msgs::msg::PoseStamped GetOrigin(const nav_msgs::msg::OccupancyGrid& obj) const {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = obj.header;
    pose.pose = obj.info.origin;
    return pose;
  }

  template<typename MapOp>
  void Convert(const geometry_msgs::msg::Pose& origin,
               const nav_msgs::msg::OccupancyGrid& obj, const MapOp& op) const {
    // Project to create a maximum value map
    op.Reset();
    Project(ConstSimpleMapOperator(obj), op);
  }
};

// @breif
template<>
class MapConverter<PointCloud> {
 public:
  MapConverter(const rclcpp::Node::SharedPtr& node,
               const std::string& input_name,
               const std::vector<PointCloudFilter::Ptr>& filters)
      : filters_(filters) {
    obstacle_radius_param_name_ = "inputs." + input_name + ".obstacle_circle.obstacle_radius";
    forbid_radius_param_name_ = "inputs." + input_name + ".obstacle_circle.forbid_radius";
    obstacle_occupancy_param_name_ = "inputs." + input_name + ".obstacle_circle.obstacle_occupancy";
    GetOptionalParam(node, obstacle_radius_param_name_, obstacle_radius_, 0.0);
    GetRequiredParam(node, forbid_radius_param_name_, forbid_radius_);
    GetOptionalParam(node, obstacle_occupancy_param_name_, obstacle_occupancy_, int8_t{0});
    param_callback_handle_ = node->add_on_set_parameters_callback(
        std::bind(&MapConverter::on_parameter_change, this, std::placeholders::_1));
  }

  template<typename MapOp>
  void Convert(const geometry_msgs::msg::Pose& origin,
               const PointCloud& obj,
               const MapOp& op) const {
    // Project to create a maximum value map
    op.Reset();
    const MapInfo& info = op.GetInfo();

    Eigen::Affine3d origin_transform;
    Eigen::Affine3d map_transform;
    tf2::fromMsg(origin, origin_transform);
    tf2::fromMsg(info.origin, map_transform);
    const Eigen::Affine3d transform = map_transform.inverse() * origin_transform;

    double width = info.width - 1;
    double height = info.height - 1;
    double x0 = transform.translation()(0) / info.resolution - 0.5;
    double y0 = transform.translation()(1) / info.resolution - 0.5;
    // Do not draw if the origin is outside the frame (it can be done but will not be supported)
    if (x0 < 0.0 || x0 > width || y0 < 0.0 || y0 > height) {
      return;
    }

    // Pre-conversion processing
    PointCloud::Ptr processing_cloud(new PointCloud());
    PointCloud::Ptr filtered_cloud(new PointCloud(obj));
    for (std::vector<PointCloudFilter::Ptr>::const_iterator it = filters_.begin();
         it != filters_.end(); ++it) {
      std::swap(processing_cloud, filtered_cloud);
      (*it)->Filter(processing_cloud, transform, filtered_cloud);
    }

    // If invalid values are still present after pre-conversion processing, it will not be supported
    if (!filtered_cloud->is_dense) {
      auto steady_clock = rclcpp::Clock(RCL_ROS_TIME);
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger("map_merger"), steady_clock,
          30000, "PointCloud contains invalid point. Can not convert to map.");
      return;
    }

    // Create a map for obstacle voting
    AnyMap<Obstacle> obstacle_map(info);
    // Create an operator for obstacle map voting
    ObstacleMapOperator obstacle_map_op(obstacle_map);
    obstacle_map_op.Reset();
    //
    for (PointCloud::iterator it = filtered_cloud->points.begin();
         it != filtered_cloud->points.end();
         ++it) {
      double x1 = it->x / info.resolution - 0.5;
      double y1 = it->y / info.resolution - 0.5;
      drawer::OutCode outcode = drawer::ComputeOutCode(x1, y1, width, height);
      // TODO(nishino) クリップされる領域をobstacle_radiusを考慮する
      drawer::Clip(x0, y0, x1, y1, width, height);
      size_t index = (int32_t)(y1 + 0.5) * info.width + (int32_t)(x1 + 0.5);
      // If there are obstacles outside the frame, perform raycast from the edge but do not register obstacles
      obstacle_map_op.Update(index, (int32_t)(outcode ? 0 : 1));
    }
    drawer::MapPlotter<MapOp> plotter(op);
    // Range to draw obstacle range [pixel]
    const int obstacle_radius_pixel = static_cast<int>(obstacle_radius_ / info.resolution + 0.5);
    const int forbid_radius_pixel = static_cast<int>(forbid_radius_ / info.resolution + 0.5);

    int32_t ix0 = transform.translation()(0) / info.resolution;
    int32_t iy0 = transform.translation()(1) / info.resolution;
    size_t index = 0;
    for (int32_t iy1 = 0; iy1 < info.height; ++iy1) {
      for (int32_t ix1 = 0; ix1 < info.width; ++ix1) {
        const Obstacle& obstacle = obstacle_map_op.Get(index);
        if (obstacle.raycast) {
          drawer::Line(plotter, 0, ix0, iy0, ix1, iy1);
        }
        if (obstacle.count > 0) {
          // Obstacle area
          drawer::Circle(plotter, obstacle_occupancy_, ix1, iy1, obstacle_radius_pixel, true);

          // Prohibited area
          drawer::Circle(plotter, kMaxOccupancy, ix1, iy1, forbid_radius_pixel, true);
        }
        ++index;
      }
    }
  }

 private:
  std::vector<PointCloudFilter::Ptr> filters_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  std::string obstacle_radius_param_name_;
  std::string forbid_radius_param_name_;
  std::string obstacle_occupancy_param_name_;
  double obstacle_radius_;
  double forbid_radius_;
  int8_t obstacle_occupancy_;

  rcl_interfaces::msg::SetParametersResult on_parameter_change(
      const std::vector<rclcpp::Parameter> & params) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & param : params) {
      const std::string param_name = param.get_name();
      if (param_name == obstacle_radius_param_name_) {
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
          obstacle_radius_ = param.as_double();
        } else {
          RCLCPP_WARN_STREAM(rclcpp::get_logger("map_merger"),
              param_name << " should be a double. Parameter update skipped.");
          result.successful = false;
        }
      } else if (param_name == forbid_radius_param_name_) {
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
          forbid_radius_ = param.as_double();
        } else {
          RCLCPP_WARN_STREAM(rclcpp::get_logger("map_merger"),
              param_name << " should be a double. Parameter update skipped.");
          result.successful = false;
        }
      } else if (param_name == obstacle_occupancy_param_name_) {
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
          obstacle_occupancy_ = param.as_int();
        } else {
          RCLCPP_WARN_STREAM(rclcpp::get_logger("map_merger"),
              param_name << " should be an int. Parameter update skipped.");
          result.successful = false;
        }
      } else {
        // Since params also include parameter settings for other classes, do nothing in else
      }
    }
    return result;
  }
};

template<>
class MapConverter<sensor_msgs::msg::LaserScan> {
 public:
  MapConverter(const rclcpp::Node::SharedPtr& node,
               const std::string& input_name,
               const std::vector<PointCloudFilter::Ptr>& filters)
      : pcl_converter_(new MapConverter<PointCloud>(node, input_name, filters)) {}

  geometry_msgs::msg::PoseStamped GetOrigin(const sensor_msgs::msg::LaserScan& obj) const {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = obj.header;
    pose.pose.orientation.w = 1.0;
    return pose;
  }

  template<typename MapOp>
  void Convert(const geometry_msgs::msg::Pose& origin,
               const sensor_msgs::msg::LaserScan& obj, const MapOp& op) const {
    // Initialize map
    op.Reset();
    // Convert LaserScan to PointCloud
    PointCloud cloud;
    cloud.is_dense = true;
    cloud.resize(obj.ranges.size());
    for (size_t index = 0; index < obj.ranges.size(); ++index) {
      Point3f& p = cloud.points[index];
      const float r = obj.ranges[index];
      if (obj.range_min <= r && r <= obj.range_max) {
        const float a = obj.angle_min + static_cast<float>(index) * obj.angle_increment;
        p.x = r * std::cos(a);
        p.y = r * std::sin(a);
        p.z = 0;
      } else {
        p = Point3f(std::numeric_limits<float>::quiet_NaN(),
                    std::numeric_limits<float>::quiet_NaN(),
                    std::numeric_limits<float>::quiet_NaN());
        cloud.is_dense = false;
      }
    }
    cloud.width = obj.ranges.size();
    cloud.height = 1;
    // Convert PointCloud to Map
    pcl_converter_->Convert(origin, cloud, op);
  }

 private:
  std::shared_ptr<MapConverter<PointCloud> > pcl_converter_;
};

template<>
class MapConverter<sensor_msgs::msg::PointCloud2> {
 public:
  MapConverter(const rclcpp::Node::SharedPtr& node,
               const std::string& input_name,
               const std::vector<PointCloudFilter::Ptr>& filters)
      : pcl_converter_(new MapConverter<PointCloud>(node, input_name, filters)) {}

  geometry_msgs::msg::PoseStamped GetOrigin(const sensor_msgs::msg::PointCloud2& obj) const {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = obj.header;
    pose.pose.orientation.w = 1.0;
    return pose;
  }

  template<typename MapOp>
  void Convert(const geometry_msgs::msg::Pose& origin,
               const sensor_msgs::msg::PointCloud2& obj, const MapOp& op) const {
    // Initialize map
    op.Reset();
    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(obj, pcl_pc);
    PointCloud cloud;
    pcl::fromPCLPointCloud2(pcl_pc, cloud);
    // Convert PointCloud to Map
    pcl_converter_->Convert(origin, cloud, op);
  }

 private:
  std::shared_ptr<MapConverter<PointCloud> > pcl_converter_;
};

}  // namespace tmc_map_merger

#endif
