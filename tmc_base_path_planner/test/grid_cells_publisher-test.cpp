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
#include <memory>
#include <vector>

#include <gtest/gtest.h>
#include <nav_msgs/msg/grid_cells.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tmc_base_path_planner/common.hpp>
#include <tmc_base_path_planner/grid_cells_publisher.hpp>

namespace {
/// Topic name of GridCells
const char* const kGridCellsTopic = "inflated_static_obstacle_map";
/// Timeout duration
const double kTimeout = 5.0;
}  // anonymous namespace

namespace tmc_base_path_planner {

/// Test target node
class TargetNode : public rclcpp::Node {
 public:
  /// Constructor
  explicit TargetNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) :
      Node("target_node", options) {}

  void Init() {
    grid_cells_publisher_ = std::make_shared<GridCellsPublisher>(shared_from_this());
  }

  void PublishGridCells(const CostMapPtr& map, const unsigned char visualization_threshold) {
    grid_cells_publisher_->PublishGridCells(map, visualization_threshold);
  }

 private:
  std::shared_ptr<GridCellsPublisher> grid_cells_publisher_;
};

/// Test node
class TestNode : public rclcpp::Node {
 public:
  /// Constructor
  explicit TestNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) :
      Node("test_node", options), grid_cells_(nullptr) {}

  void Init() {
    sub_grid_cells_ = this->create_subscription<nav_msgs::msg::GridCells>(kGridCellsTopic, 1,
        std::bind(&TestNode::GridCellsCallback_, this, std::placeholders::_1));
  }

  std::shared_ptr<nav_msgs::msg::GridCells> grid_cells() { return grid_cells_; }

 private:
  /// GridCells callback
  void GridCellsCallback_(const nav_msgs::msg::GridCells::SharedPtr msg) {
    grid_cells_ = msg;
  }

  rclcpp::Subscription<nav_msgs::msg::GridCells>::SharedPtr sub_grid_cells_;
  std::shared_ptr<nav_msgs::msg::GridCells> grid_cells_;
};

class GridCellsPublisherTest : public testing::Test {
 public:
  GridCellsPublisherTest() {}

  virtual ~GridCellsPublisherTest() = default;

 protected:
  virtual void SetUp() {
    test_node_ = std::make_shared<TestNode>();
    test_node_->Init();
    target_node_ = std::make_shared<TargetNode>();
    target_node_->Init();
  }
  virtual void TearDown() {}

  // Wait until some condition is met
  bool WaitUntil(std::function<bool()> condition_function, double timeout_sec, double rate_hz = 10.0) {
    // Error check for arguments
    if (!condition_function) {
      throw std::invalid_argument("Function for waiting is empty.");
    }
    if (timeout_sec < 0.0) {
      throw std::invalid_argument("Timeout must must have fully value");
    }
    if (rate_hz < std::numeric_limits<double>::epsilon()) {
      throw std::invalid_argument("Rate to validate must have fully value");
    }

    const rclcpp::Time end_time = test_node_->get_clock()->now() + rclcpp::Duration::from_seconds(timeout_sec);
    rclcpp::Rate rate(rate_hz);
    while (rclcpp::ok()) {
      rclcpp::spin_some(test_node_);
      rclcpp::spin_some(target_node_);
      if (condition_function()) return true;
      if (test_node_->get_clock()->now() >= end_time) break;
      rate.sleep();
    }
    return false;
  }

  std::shared_ptr<TestNode> test_node_;
  std::shared_ptr<TargetNode> target_node_;
};



TEST_F(GridCellsPublisherTest, PublishGridCells) {
  // setup
  // Create a 16x16 map
  const uint32_t map_height = 16;
  const uint32_t map_width = 16;
  const double map_resolution = 0.05;
  // Set the data to the same value as the index [0, 255]
  std::vector<unsigned char> data;
  data.resize(map_height * map_width);
  for (uint32_t i_h = 0; i_h < map_height; ++i_h) {
    for (uint32_t i_w = 0; i_w < map_width; ++i_w) {
      const uint32_t index = i_w + i_h * map_width;
      data[index] = static_cast<unsigned char>(index);
    }
  }
  CostMapPtr map(new CostMap(Pose2d(0.0, 0.0, 0.0), map_resolution, map_width, map_height, data));
  const unsigned char visualization_threshold = 200;
  // exercise
  // Request publication
  target_node_->PublishGridCells(map, visualization_threshold);

  // verify
  // GridCells are expected to be published
  ASSERT_TRUE(WaitUntil([&]() {
      return (test_node_->grid_cells() != nullptr);
    }, kTimeout));
  // Is the number of Grids registered in the published GridCells as expected?
  // Grids larger than visualization_threshold and less than kWallValue are expected to be registered
  const uint32_t expect_num = kWallValue - visualization_threshold - 1;
  const uint32_t grid_num = test_node_->grid_cells()->cells.size();
  EXPECT_EQ(expect_num, grid_num);
  // Are only the locations on the map with values set larger than visualization_threshold and less than kWallValue registered?
  for (uint32_t i = 0; i < grid_num; ++i) {
    const geometry_msgs::msg::Point grid = test_node_->grid_cells()->cells[i];
    const uint32_t grid_x = static_cast<uint32_t>(grid.x / map_resolution);
    const uint32_t grid_y = static_cast<uint32_t>(grid.y / map_resolution);
    unsigned char value;
    map->GetValueAt(grid_x, grid_y, value);
    EXPECT_GT(value, visualization_threshold);
    EXPECT_LT(value, kWallValue);
  }
}
}  // namespace tmc_base_path_planner

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
