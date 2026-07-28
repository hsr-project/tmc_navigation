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
/// @File obstacle_input.hpp
/// @brief Obstacle topic input
#ifndef TMC_BASE_VELOCITY_ADJUSTER_OBSTACLE_INPUT_HPP_
#define TMC_BASE_VELOCITY_ADJUSTER_OBSTACLE_INPUT_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "obstacle_converter.hpp"

namespace tmc_base_velocity_adjuster {
using std::placeholders::_1;
/// @brief Obstacle topic input class
/// @tparam RosMsg The type of Ros message used as input
/// @tparam Data The type of obstacle data used for optimization internally
template<class RosMsg, class Data>
class ObstacleInput {
 public:
  /// @brief Constructor
  /// @param [in] node Node
  explicit ObstacleInput(const rclcpp::Node::SharedPtr node)
      : converter_(new ObstacleConverter<RosMsg, Data>(node)),
        obstacle_(new Data()) {
    subscriber_ = node->create_subscription<RosMsg>("obstacle", rclcpp::SensorDataQoS(),
                                                    std::bind(&ObstacleInput::Callback, this, _1));
  }
  ~ObstacleInput() = default;

  /// @brief Obstacle data output
  /// @return Obstacle data
  typename Data::Ptr GetObstacle() const { return obstacle_; }

 private:
  /// @brief Obstacle input callback Converts and saves data
  /// @param [in] msg Input obstacle topic
  void Callback(const typename RosMsg::SharedPtr msg) {
    converter_->Convert(msg, obstacle_);
  }
  typename rclcpp::Subscription<RosMsg>::SharedPtr subscriber_;
  // Ros message converter
  std::unique_ptr<ObstacleConverter<RosMsg, Data>> converter_;
  // Obstacle data
  typename Data::Ptr obstacle_;
};
}  // namespace tmc_base_velocity_adjuster

#endif  // TMC_BASE_VELOCITY_ADJUSTER_OBSTACLE_INPUT_HPP_
