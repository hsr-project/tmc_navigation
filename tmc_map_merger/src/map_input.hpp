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
#ifndef TMC_MAP_MERGER_MAP_MERGER_SUBSCRIBER_HPP_
#define TMC_MAP_MERGER_MAP_MERGER_SUBSCRIBER_HPP_
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <message_filters/message_event.h>
#include <rclcpp/rclcpp.hpp>

#include "map_converter.hpp"
#include "map_merger.hpp"
#include "param.hpp"
#include "tf2helper.hpp"

namespace tmc_map_merger {
using std::placeholders::_1;

/// @brief
class MapInput {
 public:
  typedef std::shared_ptr<MapInput> Ptr;
  explicit MapInput(rclcpp::Node::SharedPtr node, const std::string& input_name, MapMerger::Ptr merger)
      : merger_(merger) {
    enable_param_name_ = "inputs." + input_name + ".enable";
    GetOptionalParam(node, enable_param_name_, is_enabled_, true);
    param_callback_handle_ = node->add_on_set_parameters_callback(
        std::bind(&MapInput::on_parameter_change, this, std::placeholders::_1));
  }
  virtual ~MapInput() {}
  const MapMerger::Ptr& GetMapMerger() const { return merger_; }
  bool IsEnabled() const { return is_enabled_; }

 protected:
  MapMerger::Ptr merger_;

 private:
  bool is_enabled_;
  std::string enable_param_name_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  rcl_interfaces::msg::SetParametersResult on_parameter_change(
      const std::vector<rclcpp::Parameter> & params) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & param : params) {
      if (param.get_name() == enable_param_name_) {
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
          is_enabled_ = param.as_bool();
        } else {
          RCLCPP_WARN_STREAM(rclcpp::get_logger("map_merger"),
            param.get_name() << " should be a bool. Parameter update skipped.");
          result.successful = false;
        }
      } else {
        // params also include parameter settings for other classes, so do nothing in else
      }
    }
    return result;
  }
};

/// @brief
template<typename RosMsg>
class Subscriber : public MapInput {
 public:
  Subscriber(rclcpp::Node::SharedPtr node,
             const std::string& input_name,
             const std::string& topic_name,
             double rate,
             const MapConverter<RosMsg>& converter,
             MapMerger::Ptr merger)
      : MapInput(node, input_name, merger), node_(node), converter_(converter), step_(0, 0) {
    const rclcpp::QoS qos = rclcpp::QoS(1).best_effort().durability_volatile();
    // Manage with shared_ptr to ensure that the this registered in the subscriber does not change
    subscriber_ = node_->create_subscription<RosMsg>(
        topic_name, qos, std::bind(&Subscriber::Callback, this, _1));
    if (rate > 0.0) {
      step_ = rclcpp::Duration::from_seconds(1.0 / rate);
    } else {
      step_ = rclcpp::Duration::from_seconds(0.0);
    }
    next_time_ = node_->now();
  }
  virtual ~Subscriber() {}

 private:
  void Callback(const std::shared_ptr<RosMsg> msg) {
    geometry_msgs::msg::PoseStamped origin;
    origin = converter_.GetOrigin(*msg);
    if (origin.header.stamp.sec == 0 && origin.header.stamp.nanosec == 0) {
      // Substitute the current time of the node as an alternative
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(),
          *node_->get_clock(),
          5000,
          "Header has no timestamp; using the current time as a fallback");
      origin.header.stamp = node_->now();
    }
    // Sample the merge cycle
    if (rclcpp::Time(origin.header.stamp) > next_time_) {
      geometry_msgs::msg::PoseStamped new_origin;
      bool updated = Tf2Helper::GetInstance()->GetPoseFromFixedFrame(origin, new_origin);
      if (updated) {
        Map map;
        map.header = new_origin.header;
        map.info = merger_->GetMap().info;
        converter_.Convert(new_origin.pose, *msg,
                           MapOperator<UpdateIfGreaterMapAdapter>(map));
        merger_->Merge(map);
        // If the next publishing cycle has also passed, step_ from now is the next publishing time
        // Set the ideal time if it has not passed
        if (rclcpp::Time(origin.header.stamp) > next_time_ + step_) {
          next_time_ = rclcpp::Time(origin.header.stamp) + step_;
        } else {
          next_time_ += step_;
        }
      }
    }
  }
  rclcpp::Node::SharedPtr node_;
  rclcpp::Duration step_;
  rclcpp::Time next_time_;
  typename rclcpp::Subscription<RosMsg>::SharedPtr subscriber_;
  const MapConverter<RosMsg> converter_;
};

class MapInputFactory {
 public:
  static MapInput::Ptr Create(const std::string& input_name,
                              const std::map<std::string, rclcpp::Parameter>& parameters,
                              rclcpp::Node::SharedPtr node,
                              MapMerger::Ptr& merger);
};

}  // namespace tmc_map_merger

#endif
