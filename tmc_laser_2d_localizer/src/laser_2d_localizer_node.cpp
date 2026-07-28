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
/// @file     laser_2d_localizer_node.cpp
/// @brief    Implementation of laser_2d_localizer
/// @version  0.x.x
/// @author   Yoshiaki Asahara
/// @date     2011.xx.xx
/// @since    2011.xx.xx
/// @note     Applied for Partner-Robot Coding Rule(Ver:x.xx)
#include "laser_2d_localizer_node.hpp"
#include <string>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.hpp>

namespace {
/// Topic name for publishing self-position estimation results
constexpr const char* const kTopicNamePubPose = "laser_2d_pose";
/// Topic name for subscribing to map data
constexpr const char* const kTopicNameSubMap = "static_distance_ros_map";
/// Topic name for subscribing to point cloud
constexpr const char* const kTopicNameSubPointCloud2 = "input_cloud";
/// Topic name for subscribing to initial position input
constexpr const char* const kTopicNameSubCorrectPose = "laser_2d_correct_pose";
/// Topic name for publishing particle positions
constexpr const char* const kTopicNamePubParticlePositions = "laser_2d_particles";
/// Topic name for publishing scores
constexpr const char* const kTopicNamePubLocalizationScore = "laser_2d_localizer/score";
/// Number of particles
constexpr double kParticleNum = 200;
/// Effective particle ratio. Resampling trigger threshold
constexpr double kEffectiveParticleRatio = 0.5;
/// Standard deviation of odometry error caused by xy movement on yx movement
constexpr double kStandardDeviationXyToYx = 0.1;
/// Standard deviation of odometry error caused by xy movement on xy movement
constexpr double kStandardDeviationXyToXy = 0.1;
/// Standard deviation of odometry error caused by xy movement on angular movement
constexpr double kStandardDeviationXyToTheta = 0.01;
/// Standard deviation of odometry error caused by angular movement on xy movement
constexpr double kStandardDeviationThetaToXy = 1.0;
/// Standard deviation of odometry error caused by angular movement on angular movement
constexpr double kStandardDeviationThetaToTheta = 0.1;
/// Standard deviation of xy error for particle dispersion during self-position reset
constexpr double kStandardDeviationInitXY = 0.0;
/// Standard deviation of angular error for particle dispersion during self-position reset
constexpr double kStandardDeviationInitTheta = 0.0;
/// (m) Initial position
constexpr double kInitX = 0.0;
/// (m) Initial position
constexpr double kInitY = 0.0;
/// (rad) Initial position
constexpr double kInitTheta = 0.0;
/// Distance threshold for executing particle filter
constexpr double kDistanceTriggeringFilter = 0.1;
/// Angular threshold for executing particle filter
constexpr double kAngleTriggeringFilter = 0.5;
/// (m)width of the potential of Distance Map.
constexpr double kPotentialWidth = 3.0;
/// Distance from the wall to consider an obstacle not on the map
constexpr double kFilteringThresh = 1.0;
/// Odometry tf name
constexpr const char* const kDefaultOdometryTfName = "odom";
/// Cart tf name
constexpr const char* const kDefaultBaseTfName = "base_footprint";
/// Robot tf name
constexpr const char* const kDefaultRobotTfName = "base_link";
/// Odometry movement limit distance [m]
constexpr double kDefaultMaxOdomDistanceThreshold = 0.5;
/// Odometry turning limit [rad]
constexpr double kDefaultMaxOdomAngleThreshold = 45.0 * M_PI / 180;
/// Warning cycle [ms] when TF cannot be read
constexpr int32_t kWarnLogPublishPeriod = 10000;
/// Cycle [ms] for outputting logs with throttle
constexpr int32_t kConsoleMessageIndicatePeriod = 5000;
// TODO(syuuhei_shiro): パラメータ取得関数は共通パッケージに置く
// Retrieve required parameters
template<typename T>
bool GetParam(const rclcpp::Node::SharedPtr& node, const std::string& param_name, T& value) {
  rclcpp::Parameter param;
  if (!node->get_parameter(param_name, param)) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("laser_2d_localizer"),
        "Parameter '" << param_name << "' is not specified.");
    return false;
  }
  value = param.get_value<T>();
  return true;
}

// Retrieve optional parameters
template<typename T>
void GetOptionalParam(const rclcpp::Node::SharedPtr& node, const std::string& param_name, T& value,
                      const T& default_value) {
  if (!GetParam(node, param_name, value)) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("laser_2d_localizer"),
        "Used default value: " << default_value);
    value = default_value;
  }
}
}  // anonymous namespace

namespace tmc_laser_2d_localizer {
using std::placeholders::_1;
using std::placeholders::_2;

Laser2dLocalizerNode::Laser2dLocalizerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("laser_2d_localizer", options),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_),
      run_mcl_(false),
      is_publisher_initialized_(false),
      frame_id_(kGlobalFrameId),
      publish_laser_2d_pose_(true),
      sum_distance_(0.0),
      sum_angle_(0.0),
      is_first_odometry_received_(false) {}

// Initialization
void Laser2dLocalizerNode::Init() {
  GetOptionalParam(shared_from_this(), "max_odom_distance_threshold", max_odom_distance_threshold_,
                   kDefaultMaxOdomDistanceThreshold);
  GetOptionalParam(shared_from_this(), "max_odom_angle_threshold", max_odom_angle_threshold_,
                   kDefaultMaxOdomAngleThreshold);

  // Retrieve parameters and set them to the object
  SetParams();

  // Set publisher
  InitializePublishers();
  // Set service
  InitializeServices();
  // Publish initial position
  PublishInitialPose();
  // Set subscriber
  InitializeSubscribers();
}

Laser2dLocalizerNode::~Laser2dLocalizerNode() {}

void Laser2dLocalizerNode::InitializePublishers() {
  // Self-position estimation result (also publishes initial position, so latch is ON)
  laser_2d_pose_publisher_ =
      this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      kTopicNamePubPose, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());  // latch ON
  particle_positions_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud>(
      kTopicNamePubParticlePositions, 1);
  score_publisher_ = this->create_publisher<std_msgs::msg::Float64>(kTopicNamePubLocalizationScore, 1);
  is_publisher_initialized_ = true;
  return;
}

void Laser2dLocalizerNode::InitializeServices() {
  // ON/OFF service for publishing laser_2d_pose
  start_localized_pose_ =
      this->create_service<std_srvs::srv::Empty>("start_sending_localized_pose",
      std::bind(&Laser2dLocalizerNode::StartLocalizedPose, this, _1, _2));
  stop_localized_pose_ =
      this->create_service<std_srvs::srv::Empty>("stop_sending_localized_pose",
      std::bind(&Laser2dLocalizerNode::StopLocalizedPose, this, _1, _2));
  check_localizer_running_ =
      this->create_service<tmc_navigation_msgs::srv::BoolResponse>("check_localizer_running",
      std::bind(&Laser2dLocalizerNode::CheckLocalizerRunning, this, _1, _2));
  set_localization_score_limit_ =
      this->create_service<tmc_navigation_msgs::srv::SetLocalizationScoreLimit>("set_localization_score_limit",
      std::bind(&Laser2dLocalizerNode::SetLocalizationScore, this, _1, _2));
  return;
}

void Laser2dLocalizerNode::InitializeSubscribers() {
  // Subscriber: Self-position map data
  static_distance_map_subscriber_ =
      this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      kTopicNameSubMap, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      std::bind(&Laser2dLocalizerNode::StaticGridMapCallback, this, _1));
  // Subscriber: 2D LRF data
  point_cloud_subscriber_ =
      this->create_subscription<sensor_msgs::msg::PointCloud2>(
      kTopicNameSubPointCloud2, rclcpp::SensorDataQoS(),
      std::bind(&Laser2dLocalizerNode::PointCloud2Callback, this, _1));
  // Subscriber: Initial position input from rviz
  correct_pose_subscriber_ =
      this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      kTopicNameSubCorrectPose, 1, std::bind(&Laser2dLocalizerNode::CorrectPoseCallback, this, _1));
  return;
}

/**
 * \brief  Callback function for retrieving odometry values
 * \param  odom_msg Odometry data message
 * Accumulate odometry movement and monitor if it exceeds the threshold.<br>
 * Set the latest odometry to the object.<br>
 * Threshold determination of odometry movement is also part of the self-position estimation function,<br>
 * and it will be incorporated into the class in the future
 */
void Laser2dLocalizerNode::UpdateOdometry(const Pose2d& odom) {
  // Skip movement calculation for the first time
  if (!is_first_odometry_received_) {
    is_first_odometry_received_ = true;
    laser_2d_mcl_.set_odometry(odom);
  } else {
    // Calculate movement
    sum_distance_ += (odom.point() - previous_odom_.point()).norm();
    // Calculate turning amount (normalize angle)
    double diff_angle = odom.theta() - previous_odom_.theta();
    if (fabs(diff_angle) > M_PI) {
      if (diff_angle > 0) {
        diff_angle = -(2 * M_PI - diff_angle);
      } else {
        diff_angle = (2 * M_PI - fabs(diff_angle));
      }
    }
    sum_angle_ += fabs(diff_angle);
    // If movement exceeds the minimum threshold, set the particle filter execution flag
    // If movement exceeds the maximum threshold, move particles with zero noise
    if ((sum_distance_ >= laser_2d_mcl_.params().distance_triggering_filter &&
         sum_distance_ < max_odom_distance_threshold_) ||
        (sum_angle_ >= laser_2d_mcl_.params().angle_triggering_filter && sum_angle_ < max_odom_angle_threshold_)) {
      laser_2d_mcl_.set_odometry(odom);
      run_mcl_ = true;
      sum_distance_ = 0.0;
      sum_angle_ = 0.0;
    } else if (sum_distance_ > max_odom_distance_threshold_ || sum_angle_ > max_odom_angle_threshold_) {
      RCLCPP_DEBUG(this->get_logger(), "move particles with 0 noise");
      laser_2d_mcl_.set_odometry(odom);
      run_mcl_ = true;
      sum_distance_ = 0.0;
      sum_angle_ = 0.0;
      // Move particles with zero noise
      laser_2d_mcl_.set_no_noise_model();
    }
  }
  previous_odom_ = odom;
}

/**
 * \brief  Callback function for retrieving grid map data
 * \param  distance_map_msg Grid map data message
 */
void Laser2dLocalizerNode::StaticGridMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr distance_map_msg) {
  // Check if the number of map data arrays is correct
  if (distance_map_msg->info.width * distance_map_msg->info.height != distance_map_msg->data.size()) {
    RCLCPP_WARN(this->get_logger(), "Map size is not correct. width=%d height=%d size=%d", distance_map_msg->info.width,
             distance_map_msg->info.height, static_cast<uint32_t>(distance_map_msg->data.size()));
    return;
  }
  // Convert to DistanceMap and expand obstacle areas to potential distances from walls
  distance_map_ = std::make_shared<DistanceMap>(
      tmc_pose_2d_lib::RosMsg2DistanceMap(*distance_map_msg));
  distance_map_->InflateMap(laser_2d_mcl_.params().potential_width);

  /// DistanceMap for passing to MCL
  /// To calculate in MCL based on the origin of the DistanceMap,
  /// set the origin to Pose2d(0.0, 0.0, 0.0)
  std::shared_ptr<DistanceMap> mcl_distance_map = std::make_shared<DistanceMap>(
      DistanceMap(Pose2d(0.0, 0.0, 0.0), distance_map_->resolution(), distance_map_->width(), distance_map_->height(),
                  distance_map_->data()));

  // Set to object
  laser_2d_mcl_.set_distance_map(mcl_distance_map);

  frame_id_ = distance_map_msg->header.frame_id;
  if (frame_id_.empty()) {
    frame_id_ = kGlobalFrameId;
  }

  // Convert initial position to map coordinates
  Pose2d init_pose = laser_2d_mcl_.init_pose_param();
  distance_map_->MapToImage(init_pose);
  laser_2d_mcl_.set_init_pose_param(init_pose);

  // Re-execute Monte Carlo when the map is updated
  run_mcl_ = true;
  return;
}


/**
 * \brief  Callback function for retrieving point_cloud2 data
 * \param  point_cloud2_msg 2D LRF data
 */
void Laser2dLocalizerNode::PointCloud2Callback(const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud2_msg) {
  sensor_msgs::msg::PointCloud2 transformed_cloud;
  tf2::Stamped<tf2::Transform> odom_to_base;
  try {
    // Convert PointCloud2 to robot coordinate system
    TransformPointCloud(robot_tf_name_, *point_cloud2_msg, transformed_cloud);
    // Retrieve odometry from tf to match the timestamp with pointcloud
    if (!tf_buffer_.canTransform(odometry_tf_name_, base_tf_name_, rclcpp::Time(transformed_cloud.header.stamp),
                                 rclcpp::Duration::from_seconds(1.0))) {
      throw std::runtime_error("canTransform Error from " + odometry_tf_name_ + " to " + base_tf_name_);
    }
    geometry_msgs::msg::TransformStamped odom_to_base_stamped = tf_buffer_.lookupTransform(
        odometry_tf_name_, base_tf_name_, rclcpp::Time(transformed_cloud.header.stamp));
    tf2::fromMsg(odom_to_base_stamped, odom_to_base);
  } catch (const std::exception& e) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), kWarnLogPublishPeriod, "%s", e.what());
    return;
  }
  Pose2d odom(odom_to_base.getOrigin().getX(), odom_to_base.getOrigin().getY(),
              tf2::getYaw(odom_to_base.getRotation()));
  // Threshold determination of movement is performed in UpdateOdometry()
  // The flag for run_mcl_ is toggled within this method
  UpdateOdometry(odom);

  // Do nothing if the map is not initialized
  if (!(laser_2d_mcl_.IsMapInitialized())) {
    return;
  }

  // Check the MCL execution flag.
  // Do nothing if it is not true.
  if (!run_mcl_) {
    return;
  }
  run_mcl_ = false;

  // Convert PointCloud2 (compressed format) to PointCloud (data entity).
  // Message type may have changed from cturtle to diamondback.
  sensor_msgs::msg::PointCloud point_cloud_msg;
  if (!sensor_msgs::convertPointCloud2ToPointCloud(transformed_cloud, point_cloud_msg)) {
    RCLCPP_ERROR(this->get_logger(), "Translation from PointCloud2 to PointCloud failed. Data is ignored.");
    return;
  }

  // Confirm that the number of point cloud data does not exceed the array size of the C structure member
  if (point_cloud_msg.points.size() > ICSLAM_D_MAX_NUM_LASER) {
    RCLCPP_ERROR(this->get_logger(), "Num of points(%d) is more than MCL capacity(%d). Data is ignored.",
              static_cast<uint32_t>(point_cloud_msg.points.size()), ICSLAM_D_MAX_NUM_LASER);
    return;
  }

  // Point cloud structure for object setting.
  // Always make it static as it is managed by pointers within the object.
  // (Would prefer vector, but due to MCL library not being in C++, use a fixed-size array structure)
  /// @todo Avoid using static
  static icSlam_tagRangeXY s_pc_2d;
  static icSlam_tagRangeXY s_pc_2d_tmp;

  // Copy message to a local variable of a different type
  for (uint32_t i = 0; i < point_cloud_msg.points.size(); ++i) {
    s_pc_2d_tmp.t_laser[i].f_x = point_cloud_msg.points[i].x;
    s_pc_2d_tmp.t_laser[i].f_y = point_cloud_msg.points[i].y;
  }
  s_pc_2d_tmp.d_numLaser = point_cloud_msg.points.size();

  // Filter laser points (remove points below the interval threshold)
  double resolution = laser_2d_mcl_.GetMapResolution();
  s_pc_2d = laser_2d_mcl_.FilterPointCloud(s_pc_2d_tmp, resolution);

  // Set 2D point cloud to object
  laser_2d_mcl_.set_point_cloud_2d(s_pc_2d);

  // Execute MCL
  Pose2d result = laser_2d_mcl_.CorrectPosition();

  distance_map_->ImageToMap(result);
  // Copy results to message
  geometry_msgs::msg::PoseWithCovarianceStamped pose_with_covariance_stamped;
  pose_with_covariance_stamped.pose.pose = GetPoseMsg(result);

  // @todo Add covariance matrix copy here
  // Match timestamp with point cloud
  pose_with_covariance_stamped.header.stamp = transformed_cloud.header.stamp;
  pose_with_covariance_stamped.header.frame_id = frame_id_;

  // Publish
  if (publish_laser_2d_pose_) {
    laser_2d_pose_publisher_->publish(pose_with_covariance_stamped);
  } else {
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), kConsoleMessageIndicatePeriod, "STOP laser_2d_pose");
  }

  // Retrieve and publish matching score
  std_msgs::msg::Float64 score_msg;
  score_msg.data = laser_2d_mcl_.GetScore();
  score_publisher_->publish(score_msg);

  // Display current particle distribution
  sensor_msgs::msg::PointCloud particle_positions;
  std::vector<Point2d> particles = laser_2d_mcl_.GetParticlePositions();
  // Match timestamp with point cloud
  particle_positions.header.stamp = transformed_cloud.header.stamp;
  particle_positions.header.frame_id = frame_id_;
  for (uint32_t i = 0; i < particles.size(); ++i) {
    Pose2d particle = Pose2d(Rotation2d(0.0), particles[i]);
    distance_map_->ImageToMap(particle);

    geometry_msgs::msg::Point32 point;
    point.x = particle.x();
    point.y = particle.y();

    point.z = 0.0;
    particle_positions.points.push_back(point);
  }
  particle_positions_publisher_->publish(particle_positions);
  return;
}

/**
 *  @brief Callback function to set initial position from rviz
 *  @param PoseWithCovarianceStamped Initial position issued from rviz
*/
void Laser2dLocalizerNode::CorrectPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr correct_pose_msg) {
  // Check frame
  if (correct_pose_msg->header.frame_id != frame_id_) {
    RCLCPP_WARN(this->get_logger(),
        "Frame id of external estimated pose(%s) is not correct\n"
        "  Expected: %s\n  Actual: %s",
        kTopicNameSubCorrectPose, frame_id_.c_str(), correct_pose_msg->header.frame_id.c_str());
    return;
  }
  Pose2d correct_pose = GetPose2dFromRosMsg(*correct_pose_msg);
  RCLCPP_INFO(this->get_logger(), "[laser_2d_localizer] correct pose (%f %f %f)",
           correct_pose.x(), correct_pose.y(), correct_pose.theta());
  distance_map_->MapToImage(correct_pose);
  // Force adoption of estimation results
  laser_2d_mcl_.set_is_manual_reset(true);
  // Input initial value
  laser_2d_mcl_.set_initial_pose(correct_pose);
  // Turn ON MCL execution flag
  run_mcl_ = true;
  return;
}

/**
 *  @brief Service to allow publishing of laser_2d_pose
 *  @param res.is_success  true:success false:fail
*/
void Laser2dLocalizerNode::StartLocalizedPose(
    std_srvs::srv::Empty::Request::SharedPtr req,
    std_srvs::srv::Empty::Response::SharedPtr res) {
  publish_laser_2d_pose_ = true;
  RCLCPP_INFO(this->get_logger(), "Publish Localized Pose");
}


/**
 *  @brief Service to stop publishing of laser_2d_pose
 *  @param res.is_success  true:success false:fail
*/
void Laser2dLocalizerNode::StopLocalizedPose(
    std_srvs::srv::Empty::Request::SharedPtr req,
    std_srvs::srv::Empty::Response::SharedPtr res) {
  publish_laser_2d_pose_ = false;
  RCLCPP_INFO(this->get_logger(), "Stop Publishing Localized Pose");
}


/**
 *  @brief Service to check the publishing status of laser_2d_pose
 *  @param res.is_localization_running true: Publishing false: Stopped
*/
void Laser2dLocalizerNode::CheckLocalizerRunning(
    tmc_navigation_msgs::srv::BoolResponse::Request::SharedPtr req,
    tmc_navigation_msgs::srv::BoolResponse::Response::SharedPtr res) {
  res->result = publish_laser_2d_pose_;
  return;
}


/**
 *  @brief Service to set localization_score_limit
 *  @param res.is_success  true:success false:fail
*/
void Laser2dLocalizerNode::SetLocalizationScore(
    tmc_navigation_msgs::srv::SetLocalizationScoreLimit::Request::SharedPtr req,
    tmc_navigation_msgs::srv::SetLocalizationScoreLimit::Response::SharedPtr res) {
  // Set value
  laser_2d_mcl_.set_localization_score_limit(req->localization_score_limit);
  RCLCPP_INFO(this->get_logger(), "Set localization_score_limit: %lf", req->localization_score_limit);
  res->result = true;
}


/**
 * @brief Function to set parameters
 * @par
 * Retrieve default parameters, overwrite desired parameters, and set them
 */
void Laser2dLocalizerNode::SetParams() {
  Laser2dMclParams params = laser_2d_mcl_.default_params();

  GetOptionalParam(shared_from_this(), "number_of_particles", params.number_of_particles,
                   kParticleNum);
  GetOptionalParam(shared_from_this(), "effective_particle_ratio", params.effective_particle_ratio,
                   kEffectiveParticleRatio);
  GetOptionalParam(shared_from_this(), "standard_deviation_xy_to_yx", params.standard_deviation_xy_to_yx,
                   kStandardDeviationXyToYx);
  GetOptionalParam(shared_from_this(), "standard_deviation_xy_to_xy", params.standard_deviation_xy_to_xy,
                   kStandardDeviationXyToXy);
  GetOptionalParam(shared_from_this(), "standard_deviation_xy_to_theta", params.standard_deviation_xy_to_theta,
                   kStandardDeviationXyToTheta);
  GetOptionalParam(shared_from_this(), "standard_deviation_theta_to_xy", params.standard_deviation_theta_to_xy,
                   kStandardDeviationThetaToXy);
  GetOptionalParam(shared_from_this(), "standard_deviation_theta_to_theta", params.standard_deviation_theta_to_theta,
                   kStandardDeviationThetaToTheta);
  GetOptionalParam(shared_from_this(), "standard_deviation_init_xy", params.standard_deviation_init_xy,
                   kStandardDeviationInitXY);
  GetOptionalParam(shared_from_this(), "standard_deviation_init_theta", params.standard_deviation_init_theta,
                   kStandardDeviationInitTheta);
  double init_x;
  GetOptionalParam(shared_from_this(), "init_x", init_x, kInitX);
  double init_y;
  GetOptionalParam(shared_from_this(), "init_y", init_y, kInitY);
  double init_theta;
  GetOptionalParam(shared_from_this(), "init_theta_deg", init_theta, kInitTheta);
  init_theta = init_theta * M_PI / 180.0;  // Convert from deg to rad
  params.init_pose = Pose2d(init_x, init_y, init_theta);

  GetOptionalParam(shared_from_this(), "distance_triggering_filter", params.distance_triggering_filter,
                   kDistanceTriggeringFilter);
  GetOptionalParam(shared_from_this(), "angle_triggering_filter", params.angle_triggering_filter,
                   kAngleTriggeringFilter);
  GetOptionalParam(shared_from_this(), "potential_width", params.potential_width, kPotentialWidth);
  GetOptionalParam(shared_from_this(), "laser_filter", params.filtering_thresh, kFilteringThresh);
  double localization_score_limit;
  GetOptionalParam(shared_from_this(), "localization_score_limit", localization_score_limit, 0.0);
  GetOptionalParam(shared_from_this(), "odometry_tf_name", odometry_tf_name_, std::string(kDefaultOdometryTfName));
  GetOptionalParam(shared_from_this(), "base_tf_name", base_tf_name_, std::string(kDefaultBaseTfName));
  GetOptionalParam(shared_from_this(), "robot_tf_name", robot_tf_name_, std::string(kDefaultRobotTfName));
  // Set parameters to object
  laser_2d_mcl_.set_params(params);
  // Set localization score
  laser_2d_mcl_.set_localization_score_limit(localization_score_limit);
  return;
}

/**
 * @brief Publish initial position
 * @par
 * Reason: grid_map_server publishes maps based on self-position,
 * and this node, which publishes self-position, requires the map.
 * To avoid locking, publish self-position first from here.
 */
void Laser2dLocalizerNode::PublishInitialPose() {
  if (is_publisher_initialized_) {
    Laser2dMclParams params = laser_2d_mcl_.params();
    geometry_msgs::msg::PoseWithCovarianceStamped pose_with_covariance_stamped;
    pose_with_covariance_stamped.pose.pose = GetPoseMsg(params.init_pose);
    pose_with_covariance_stamped.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
    pose_with_covariance_stamped.header.frame_id = frame_id_;
    laser_2d_pose_publisher_->publish(pose_with_covariance_stamped);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Publishers have not initialized. Initial pose is not published.");
  }
  return;
}

/// @brief Convert PointCloud to specified frame_id
/// @param frame_id[in] Target frame for conversion
/// @param input_cloud[in] PointCloud before conversion
/// @param output_cloud[out] PointCloud after conversion
void Laser2dLocalizerNode::TransformPointCloud(const std::string& frame_id,
                                               const sensor_msgs::msg::PointCloud2& input_cloud,
                                               sensor_msgs::msg::PointCloud2& output_cloud) {
  bool transform_is_found =
      tf_buffer_.canTransform(frame_id,
                              input_cloud.header.frame_id,
                              rclcpp::Time(input_cloud.header.stamp),
                              rclcpp::Duration::from_seconds(1.0));
  if (transform_is_found) {
    geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_.lookupTransform(
        frame_id, input_cloud.header.frame_id, rclcpp::Time(input_cloud.header.stamp),
        rclcpp::Duration::from_seconds(1.0));
    tf2::doTransform(input_cloud, output_cloud, transform_stamped);
  } else {
    throw std::runtime_error("canTransform Error from " + frame_id + " to " + input_cloud.header.frame_id);
  }
}

}  // end namespace tmc_laser_2d_localizer
