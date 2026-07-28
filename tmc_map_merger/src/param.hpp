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
#ifndef TMC_MAP_MERGER_PARAM_HPP_
#define TMC_MAP_MERGER_PARAM_HPP_

#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace { // NOLINT

// Check if it is greater than the specified value
template<typename T>
struct Greater {
  explicit Greater(const T& t) : t_(t) {}
  bool validate(const T& v) const {return v > t_;}
  std::string message() const {
    std::stringstream ss;
    ss << "should be greater than " << t_;
    return ss.str();
  }
  T t_;
};

// Check if it is not less than the specified value
template<typename T>
struct NotLess {
  explicit NotLess(const T& t) : t_(t) {}
  bool validate(const T& v) const {return !(v < t_);}
  std::string message() const {
    std::stringstream ss;
    ss << "should not be less than " << t_;
    return ss.str();
  }
  T t_;
};

// Check if it is within the specified range
template<typename T>
struct InRange {
  InRange(const T& min, const T& max) : min_(min), max_(max) {}
  bool validate(const T& v) const {return min_ <= v && v <= max_;}
  std::string message() const {
    std::stringstream ss;
    ss << "should be within the range between " << min_ << " and " << max_;
    return ss.str();
  }
  T min_;
  T max_;
};

template<typename T>
bool IsEqual(const T& lhs, const T& rhs) {
  return lhs == rhs;
}

template<>
bool IsEqual<float>(const float& lhs, const float& rhs) {
  return std::abs(lhs - rhs) < std::numeric_limits<float>::epsilon();
}

template<>
bool IsEqual<double>(const double& lhs, const double& rhs) {
  return std::abs(lhs - rhs) < std::numeric_limits<double>::epsilon();
}

// Check if it is equal to the registered value
template<typename T>
struct Equal {
  explicit Equal(const T& t) : t_(t) {}
  bool validate(const T& v) const {return IsEqual(v, t_);}
  bool operator()(const T& v) const {return validate(v);}
  std::string message() const {
    std::stringstream ss;
    ss << "should be equal to " << t_;
    return ss.str();
  }
  T t_;
};

// Check if it is included in the registered value
template<typename T>
struct OneOf {
  typedef typename std::vector<T>::const_iterator ArrayConstIterator;
  explicit OneOf(const std::vector<T>& references) : references_(references) {
    if (references.size() < 2) {
      throw std::logic_error("This validator needs a vector that have over 2 elements");
    }
  }
  bool validate(const T& v) const {
    ArrayConstIterator result = std::find_if(references_.begin(), references_.end(), Equal<T>(v));
    return result != references_.end();
  }
  std::string message() const {
    std::stringstream ss;
    ss << "should be one of ";
    if (references_.size() == 2) {
      // A or B
      ss << references_[0] << " or " << references_[1];
    } else {
      // A, B, ... , or Z
      for (ArrayConstIterator it = references_.begin(); it != references_.end(); ++it) {
        if (it == (references_.end() - 1)) {
          ss << " or ";
        }
        ss << *it;
        if (it < (references_.end() - 2)) {
          ss << ", ";
        }
      }
    }
    return ss.str();
  }
  std::vector<T> references_;
};

// Retrieve required parameters
template<typename T>
void GetRequiredParam(const rclcpp::Node::SharedPtr& node, const std::string& param_name, T& value) {
  rclcpp::Parameter param;
  if (!node->get_parameter(param_name, param)) {
    const std::string message = std::string("Parameter '") + param_name + "' is required";
    throw std::logic_error(message.c_str());
  }
  value = param.get_value<T>();
}

// Retrieve required ROS parameters (with validation)
// In C++0x, template default arguments cannot be used, so separate them
template<typename T, typename Checker>
void GetRequiredParam(const rclcpp::Node::SharedPtr& node, const std::string& param_name, T& value, const Checker& c) {
  T tmp;
  GetRequiredParam(node, param_name, tmp);
  if (!c.validate(tmp)) {
    const std::string message = std::string("Parameter '") + param_name + "' " + c.message();
    throw std::logic_error(message.c_str());
  }
  value = tmp;
}

// Retrieve optional parameters
template<typename T>
void GetOptionalParam(const rclcpp::Node::SharedPtr& node, const std::string& param_name, T& value,
                      const T& default_value) {
  rclcpp::Parameter param;
  if (!node->get_parameter(param_name, param)) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("map_merger"),
        "Parameter '" << param_name <<
        "' is not specified. Declared parameter and used default value: " << default_value);
    node->declare_parameter<T>(param_name, default_value);
    value = default_value;
  } else {
    value = param.get_value<T>();
  }
}

// Retrieve optional parameters (with validation)
template<typename T, typename Checker>
void GetOptionalParam(const rclcpp::Node::SharedPtr& node, const std::string& param_name, T& value,
                      const T& default_value, const Checker& c) {
  // Treat as an exception if the checker throws an error with the default value
  if (!c.validate(default_value)) {
    const std::string message = std::string("Default value of '") + param_name + "' is invalid";
    throw std::logic_error(message.c_str());
  }

  T tmp;
  GetOptionalParam(node, param_name, tmp, default_value);
  if (!c.validate(tmp)) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("map_merger"),
        "Parameter '" << param_name << "' is invalid. Used default value: " << default_value);
    value = default_value;
  } else {
    value = tmp;
  }
}

void GetRequiredGroupParam(const rclcpp::Node::SharedPtr& node, const std::string& group_name,
                           std::map<std::string, rclcpp::Parameter>& group) {
  const bool has_parameters = node->get_parameters(group_name, group);
  if (!has_parameters) {
    const std::string message = std::string("Parameter group '") + group_name + "' is required";
    throw std::logic_error(message.c_str());
  }
}

void GetRequiredGroupParam(const std::map<std::string, rclcpp::Parameter>& parameters, const std::string& group_name,
                           std::map<std::string, rclcpp::Parameter>& group) {
  const std::string prefix = group_name + ".";
  for (auto param : parameters) {
    if (param.first.find(prefix) == 0) {
      std::string param_name = param.first;
      param_name.erase(0, prefix.length());
      group.insert(std::make_pair(param_name, param.second));
    }
  }
}

template<typename T>
void GetRequiredParam(const std::map<std::string, rclcpp::Parameter>& group, const std::string& param_name, T& value) {
  auto it = group.find(param_name);
  if (it != group.end()) {
    value = it->second.get_value<T>();
  } else {
    const std::string message = std::string("Parameter '") + param_name + "' is required";
    throw std::logic_error(message.c_str());
  }
}

template<typename T, typename Checker>
void GetRequiredParam(const std::map<std::string, rclcpp::Parameter>& group, const std::string& param_name,
                      T& value, const Checker& c) {
  T tmp;
  GetRequiredParam(group, param_name, tmp);
  if (!c.validate(tmp)) {
    const std::string message = std::string("Parameter '") + param_name + "' " + c.message();
    throw std::logic_error(message.c_str());
  }
  value = tmp;
}

template<typename T>
void GetOptionalParam(const std::map<std::string, rclcpp::Parameter>& group, const std::string& param_name,
                      T& value, const T& default_value) {
  auto it = group.find(param_name);
  if (it != group.end()) {
    value = it->second.get_value<T>();
  } else {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("map_merger"),
        "Parameter '" << param_name << "' is invalid. Used default value: " << default_value);
    value = default_value;
  }
}

template<typename T, typename Checker>
void GetOptionalParam(const std::map<std::string, rclcpp::Parameter>& group, const std::string& param_name,
                      T& value, const T& default_value, const Checker& c) {
  // Treat as an exception if the checker throws an error with the default value
  if (!c.validate(default_value)) {
    const std::string message = std::string("Default value of '") + param_name + "' is invalid";
    throw std::logic_error(message.c_str());
  }

  T tmp;
  GetOptionalParam(group, param_name, tmp, default_value);
  if (!c.validate(tmp)) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("map_merger"),
        "Parameter '" << param_name << "' is invalid. Used default value: " << default_value);
    value = default_value;
  } else {
    value = tmp;
  }
}
}  // anonymous namespace

#endif
