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
#ifndef TMC_MAP_MERGER_TEMPORAL_MEMORIES_HPP_
#define TMC_MAP_MERGER_TEMPORAL_MEMORIES_HPP_

#include <cmath>
#include <limits>
#include <rclcpp/rclcpp.hpp>

namespace tmc_map_merger {

/// @brief A class implementing a simple Memory that directly stores observed information
/// Becomes Unknown after a certain period of time since the last update
/// A class with implemented Get, Update, and Reset methods
/// @see MapMemoryMerger
struct SimpleMemory {
  struct Option {
    Option() : timeout(0, 0) {}
    rclcpp::Duration timeout;
    int8_t default_data;
  };
  SimpleMemory()
      : memory(kUnknown) {
  }

  /// @brief Retrieve the current memory
  virtual int8_t Get(const rclcpp::Time& time, const Option& option) const {
    if (memory != kUnknown && time - last_valid < option.timeout ||
        option.timeout.seconds() < std::numeric_limits<double>::epsilon()) {
      return memory;
    }
    return option.default_data;
  }

  /// @brief Update memory with observed information
  virtual void Update(int8_t data, const rclcpp::Time& time, const Option& option) {
    // Stores all information except unobserved data without exception
    if (data >= kFree) {
      memory = data;
      last_valid = time;
    }
  }

  /// @brief Clear the memory
  virtual void Reset(const rclcpp::Time& time, const Option& option) {
    memory = option.default_data;
    last_valid = rclcpp::Time(0);
  }

  int8_t memory;
  rclcpp::Time last_valid;
};


/// @brief A class implementing a safer Memory that stores observed information only if its value is greater than the stored information
/// Becomes Unknown after a certain period of time since the last update
/// A class with implemented Get, Update, and Reset methods
/// @see MapMemoryMerger
struct SafetyMemory : public SimpleMemory {
  SafetyMemory()
      : SimpleMemory() {
  }

  /// @brief Update memory with observed information
  void Update(int8_t data, const rclcpp::Time& time, const Option& option) {
    // Stores all unoccupied information without exception
    if (data == kFree) {
      memory = data;
      last_valid = time;
    // If it is assumed to be occupied, stores only when the value is greater
    } else if (data > kFree) {
      if (data > memory) {
        memory = data;
      }
      // Even if the observed value is not large, the fact of observation is recorded, so the time is updated
      last_valid = time;
    }
  }
};

}  // namespace tmc_map_merger

#endif
