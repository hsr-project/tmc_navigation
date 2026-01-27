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
/// @file     laser_2d_mcl.cpp
/// @brief    Self-localization using 2D point cloud
/// @version  0.x.x
/// @author   Yoshiaki Asahara
/// @date     2011.xx.xx
/// @since    2011.xx.xx
/// @note     Applied for Partner-Robot Coding Rule(Ver:x.xx)
/// @attention Do not write code that depends on platforms like ROS or FSM

#include "laser_2d_mcl.hpp"

#include <cassert>
#include <console_bridge/console.h>

/**
  @brief Default parameters
  @par
  If parameter settings are not available, default parameters are used, but
  It might be better to throw an error and terminate if no external settings are provided.
*/
namespace {
/// Number of particles
double const kParticleNum = 200;
/// Effective particle ratio. Resampling trigger threshold
double const kEffectiveParticleRatio = 0.5;
/// Standard deviation of odometry error caused by xy movement affecting yx movement
double const kStandardDeviationXyToYx = 0.1;
/// Standard deviation of odometry error caused by xy movement affecting xy movement
double const kStandardDeviationXyToXy = 0.1;
/// Standard deviation of odometry error caused by xy movement affecting angular movement
double const kStandardDeviationXyToTheta = 0.01;
/// Standard deviation of odometry error caused by angular movement affecting xy movement
double const kStandardDeviationThetaToXy = 1.0;
/// Standard deviation of odometry error caused by angular movement affecting angular movement
double const kStandardDeviationThetaToTheta = 0.1;
/// Standard deviation of xy error in particle dispersion during self-localization reset
double const kStandardDeviationInitXY = 0.0;
/// Standard deviation of angular error in particle dispersion during self-localization reset
double const kStandardDeviationInitTheta = 0.0;
/// (m) Initial position
double const kInitX = 0.0;
/// (m) Initial position
double const kInitY = 0.0;
/// (rad) Initial position
double const kInitTheta = 0.0;
/// Distance threshold for executing particle filter
double const kDistanceTriggeringFilter = 0.1;
/// Angular threshold for executing particle filter
double const kAngleTriggeringFilter = 0.5;
/// (m)width of the potential of Distance Map.
double const kPotentialWidth = 3.0;
/// At what distance from the wall should an obstacle be considered not on the map
double const kFilteringThresh = 1.0;
}  // anonimous namespace

namespace tmc_laser_2d_localizer {
/**
  @brief Constructor
  @par
  Initialization of each member variable, allocation of data area
*/
Laser2dMcl::Laser2dMcl()
    : is_range_data_initialized_(false),
      is_pose_initialized_(false),
      is_manual_reset_(false),
      is_no_noise_(false),
      p_distance_map_(nullptr) {
  // Default parameters
  default_params_.number_of_particles = kParticleNum;
  default_params_.effective_particle_ratio = kEffectiveParticleRatio;
  default_params_.standard_deviation_xy_to_yx = kStandardDeviationXyToYx;
  default_params_.standard_deviation_xy_to_xy = kStandardDeviationXyToXy;
  default_params_.standard_deviation_xy_to_theta = kStandardDeviationXyToTheta;
  default_params_.standard_deviation_theta_to_xy = kStandardDeviationThetaToXy;
  default_params_.standard_deviation_theta_to_theta = kStandardDeviationThetaToTheta;
  default_params_.standard_deviation_init_xy = kStandardDeviationInitXY;
  default_params_.standard_deviation_init_theta = kStandardDeviationInitTheta;
  default_params_.init_pose = Pose2d(kInitX, kInitY, kInitTheta);
  default_params_.potential_width = kPotentialWidth;
  default_params_.filtering_thresh = kFilteringThresh;
  params_ = default_params_;

  // Odometry initialization
  odometry_ = Pose2d(0.0, 0.0, 0.0);
  init_odometry_ = Pose2d(0.0, 0.0, 0.0);
  // Initialization of 2D LRF self-localization data area
  laser_2d_pose_ = Pose2d(0.0, 0.0, 0.0);

  return;
}

/**
  @brief Destructor
  @par
  Release of data area allocated in the constructor
*/
Laser2dMcl::~Laser2dMcl() {}

/**
  @brief Parameter data acquisition
  @param value Parameter data
  @return None

*/
void Laser2dMcl::set_params(const Laser2dMclParams& value) {
  memcpy(&params_, &value, sizeof(value));
  // Since self-localization is included in the parameters, set the initialization flag to true
  is_pose_initialized_ = true;
  return;
}

void Laser2dMcl::set_init_pose_param(const Pose2d& init_pose) {
  params_.init_pose = init_pose;
}

Pose2d Laser2dMcl::init_pose_param(void) const {
  return params_.init_pose;
}

/**
  @brief Odometry data acquisition
  @param value Odometry data
*/

void Laser2dMcl::set_odometry(const Pose2d& value) {
  // Save only the first time to init_odometry_
  static bool s_is_first = true;
  if (s_is_first) {
    init_odometry_ = value;
    s_is_first = false;
  }
  odometry_ = value;
  is_odometry_initialized_ = true;
}

/**
  @brief Self-localization map data acquisition
  @param value Self-localization map data
*/
void Laser2dMcl::set_distance_map(const std::shared_ptr<DistanceMap>& value) {
  p_distance_map_ = value;
}

/**
  @brief Set 2D point cloud
  Input data using the robot center (=self-localization) as the coordinate system.
  Generally referred to as the robot coordinate system.
*/
void Laser2dMcl::set_point_cloud_2d(const icSlam_tagRangeXY& value) {
  p_point_cloud_2d_ = &value;
  is_range_data_initialized_ = true;
  return;
}

/**
   @brief Self-localization reset
   Align particles to the input position
*/
void Laser2dMcl::set_initial_pose(const Pose2d& value) {
  params_.init_pose = value;
  is_pose_initialized_ = true;
  return;
}

/**
  @brief Execute MCL and obtain self-localization estimation result
  @return Self-localization estimation result in global coordinates
*/
Pose2d Laser2dMcl::CorrectPosition() {
  // Self-localization estimation result
  static icSlam_tagOrientedPoint s_result_pose = { 0.0, 0.0, 0.0 };

  static bool s_only_first = true;

  // Check if the necessary data initialization is complete
  if (!is_odometry_initialized_ || !is_range_data_initialized_ || p_distance_map_ == nullptr) {
    if (!is_odometry_initialized_) {
      CONSOLE_BRIDGE_logWarn("Odometry has not been initialized. Do nothing.");
    } else if (!is_range_data_initialized_) {
      CONSOLE_BRIDGE_logError("PointCloud has not been initialized. Do nothing.");
    } else if (p_distance_map_ == nullptr) {
      CONSOLE_BRIDGE_logError("Map has not been initialized. Do nothing.");
    }
    // If external data is not initialized, return the initial value and terminate
    return params_.init_pose;
  }

  // Update particle initial position
  // Assign initial position to the estimated value
  if (is_pose_initialized_) {
    icSlam_tagOrientedPoint initPose;
    initPose.f_x = params_.init_pose.x();
    initPose.f_y = params_.init_pose.y();
    initPose.f_theta = params_.init_pose.theta();
    icSlam_Fd_setInitParticles(initPose, params_.number_of_particles, params_.standard_deviation_init_xy,
                               params_.standard_deviation_init_theta);
    is_pose_initialized_ = false;

    s_result_pose.f_x = params_.init_pose.x();
    s_result_pose.f_y = params_.init_pose.y();
    s_result_pose.f_theta = params_.init_pose.theta();
  }

  // Calculate odometry movement
  static bool s_is_first = true;
  static icSlam_tagOrientedPoint s_oldOdom;
  // Copy init_odometry_ to oldOdom only the first time
  if (s_is_first) {
    s_oldOdom.f_x = init_odometry_.x();
    s_oldOdom.f_y = init_odometry_.y();
    s_oldOdom.f_theta = init_odometry_.theta();
    s_is_first = false;
  }
  icSlam_tagOrientedPoint newOdom;
  newOdom.f_x = odometry_.x();
  newOdom.f_y = odometry_.y();
  newOdom.f_theta = odometry_.theta();
  // Odometry movement
  icSlam_tagOrientedPoint dOdom = icSlam_Ft_absoluteDiff(s_oldOdom, newOdom);
  s_oldOdom = newOdom;

  if (is_no_noise_) {
    // Set motion model noise to 0
    icSlam_Fd_setMotionNoise(0, 0, 0, 0, 0);
    is_no_noise_ = false;
  } else {
    // Set motion model noise parameters
    icSlam_Fd_setMotionNoise(params_.standard_deviation_xy_to_yx, params_.standard_deviation_xy_to_theta,
                             params_.standard_deviation_theta_to_theta, params_.standard_deviation_xy_to_xy,
                             params_.standard_deviation_theta_to_xy);
  }
  // Move particles with the motion model
  icSlam_Fd_predict(dOdom);

  // Set map
  icSlam_Fd_setMapSize(p_distance_map_->width(), p_distance_map_->height(), p_distance_map_->resolution(),
                       params_.potential_width, params_.filtering_thresh);

  // Redistribute particles that are out of the map
  icSlam_Fd_killRingOut(static_cast<uint8_t*>(p_distance_map_->data().data()), &s_result_pose,
                        p_distance_map_->origin().x(), p_distance_map_->origin().y());

  // Set 2D point cloud
  icSlam_Fd_setRangePoints(p_point_cloud_2d_);

  // Weight calculation
  best_matching_score_ =
      icSlam_Ff_likelihood(static_cast<uint8_t*>(p_distance_map_->data().data()),
                                                 p_distance_map_->origin().x(), p_distance_map_->origin().y());


  // Adopt estimation result if likelihood is high or laser_2d_correct_pose is used
  if (best_matching_score_ > localization_score_limit_ || is_manual_reset_ || s_only_first) {
    is_manual_reset_ = false;
    s_only_first = false;
    // Normalize particle weights
    icSlam_Fd_normalizeWeights();

    // Obtain self-localization estimation position
    icSlam_Fd_getExpectedPose(&s_result_pose);
    laser_2d_pose_ = Pose2d(s_result_pose.f_x, s_result_pose.f_y, s_result_pose.f_theta);

    // Check Neff and resample
    int32_t ret = icSlam_Fd_checkNeff(params_.effective_particle_ratio);
    if (ret == TRUE) {
      icSlam_Fd_resample();
    }
    CONSOLE_BRIDGE_logDebug("localization success (score: %lf, limit: %lf)",
        best_matching_score_, localization_score_limit_);
  } else {
    // Discard result if likelihood is low and adopt odometry
    Point2d diff_odometry(dOdom.f_x, dOdom.f_y);
    Point2d rotate_diff_odometry = previous_laser_2d_pose_.rot() * diff_odometry;
    laser_2d_pose_ = Pose2d(rotate_diff_odometry.x() + previous_laser_2d_pose_.x(),
                            rotate_diff_odometry.y() + previous_laser_2d_pose_.y(),
                            dOdom.f_theta + previous_laser_2d_pose_.theta());

    // Redistribute particles near self-localization using odometry
    icSlam_tagOrientedPoint initPose;
    initPose.f_x = laser_2d_pose_.x();
    initPose.f_y = laser_2d_pose_.y();
    initPose.f_theta = laser_2d_pose_.theta();
    icSlam_Fd_setInitParticles(initPose, params_.number_of_particles, params_.standard_deviation_init_xy,
                               params_.standard_deviation_init_theta);
    CONSOLE_BRIDGE_logDebug("odom mode (score: %lf, limit: %lf)", best_matching_score_, localization_score_limit_);
  }

  // Save estimation result
  previous_laser_2d_pose_ = laser_2d_pose_;

  // Calculate covariance matrix
  return laser_2d_pose_;
}

/**
 * @brief Secure current particle status
 * @return Particle distribution in global coordinates
 */
std::vector<Point2d> Laser2dMcl::GetParticlePositions() {
  int32_t result = ICSLAM_D_RET_ERROR;
  std::vector<Point2d> particle_positions;
  icSlam_tagParticleSet t_particle;  // Actual particle values

  // Read actual particles
  result = icSlam_Fd_GetParticlePositions(&t_particle);

  // Terminate abnormally if the number of particles is not between 0 and 200
  assert(((t_particle.d_numPart) >= 0) && ((t_particle.d_numPart) <= params_.number_of_particles));

  if (result == ICSLAM_D_RET_SUCCESS) {
    particle_positions.resize(static_cast<uint32_t>(t_particle.d_numPart));
    // Convert obtained particles to point cloud
    for (uint32_t i = 0; i < static_cast<uint32_t>(t_particle.d_numPart); ++i) {
      // Expand contents
      particle_positions[i] = Point2d(t_particle.t_particle[i].t_pose.f_x, t_particle.t_particle[i].t_pose.f_y);
    }
  }

  return particle_positions;
}

/**
 * @brief Delete one of the adjacent points if they are within a certain interval.
 * @param pc Point cloud
 * @param interval Interval threshold (m)
 * @note Copying has a large overhead, so I want to modify it to reference with vector
 */
icSlam_tagRangeXY Laser2dMcl::FilterPointCloud(const icSlam_tagRangeXY& point_cloud, double interval) {
  // Terminate if the number of lasers is negative
  assert(point_cloud.d_numLaser >= 0);

  icSlam_tagRangeXY ret_pc;
  // Initialize the number of lasers to 0
  ret_pc.d_numLaser = 0;

  // Return immediately if the number of lasers is 0.
  if (point_cloud.d_numLaser == 0) {
    CONSOLE_BRIDGE_logInform("no laser. filtering is skipped.");
    return ret_pc;
  }

  // Copy and return if the number of lasers is 1.
  if (point_cloud.d_numLaser == 1) {
    CONSOLE_BRIDGE_logInform("1 laser point doesn't need filtering. filtering is skipped.");
    ret_pc.d_numLaser = 1;
    ret_pc.t_laser[0] = point_cloud.t_laser[0];
    return ret_pc;
  }

  // Execute the following process if the number of lasers is 2 or more.
  ret_pc.t_laser[0] = point_cloud.t_laser[0];
  icSlam_tagPoint base = point_cloud.t_laser[0];
  int32_t index = 1;  // Array number of ret_pc to write
  // Skip the first point
  for (uint32_t i = 1; i < static_cast<uint32_t>(point_cloud.d_numLaser); ++i) {
    if (fabs(point_cloud.t_laser[i].f_x - base.f_x) > interval ||
        fabs(point_cloud.t_laser[i].f_y - base.f_y) > interval) {
      ret_pc.t_laser[index] = point_cloud.t_laser[i];
      base = point_cloud.t_laser[i];
      index += 1;
    }
  }
  ret_pc.d_numLaser = index;

  return ret_pc;
}

/**
 * @brief Obtain map resolution
 * @return Resolution of the map used by self-localization (m)
 */
double Laser2dMcl::GetMapResolution() const {
  assert(p_distance_map_ != nullptr);
  return p_distance_map_->resolution();
}

/**
 * @brief Map initialization confirmation
 * @return true: Map received, false: Map not yet received
 */
bool Laser2dMcl::IsMapInitialized() const { return p_distance_map_ != nullptr; }
}  // namespace tmc_laser_2d_localizer
