#!/usr/bin/env python3
# Copyright (c) 2026 TOYOTA MOTOR CORPORATION
# All rights reserved.
# Redistribution and use in source and binary forms, with or without
# modification, are permitted (subject to the limitations in the disclaimer
# below) provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of the copyright holder nor the names of its contributors may be used
#   to endorse or promote products derived from this software without specific
#   prior written permission.
# NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
# LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
# GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
# OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
# DAMAGE.
# -*- coding: utf-8 -*-
# @file point_cloud_accumulator-test.py
# @brief Test for point cloud accumulation node

import os
from struct import unpack
import threading
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
import launch_ros.actions
import launch_testing
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter_client import AsyncParameterClient
from rclpy.wait_for_message import wait_for_message
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# Point cloud acquisition monitoring frequency [Hz]
TEST_RATE = 100.0
# Maximum waiting time for point cloud acquisition [s]
WAIT_POINT_CLOUD_PERIOD = 0.5
# Maximum waiting time for tf acquisition [s]
WAIT_TF_PERIOD = 0.5
# Time interval for testing [s]
TEST_PERIOD = 18.0
# Movement distance [m] considered as the robot starting to move
MOVE_START_THRESHOLD = 0.005
# Margin applied to point cloud saving time [s]
TIME_MERGIN = 2.0
# Movement distance [m] considered as the point cloud disappearing
MOVE_PCL_THRESHOLD = 0.03


@pytest.mark.launch_test
def generate_test_description():
    share_dir = get_package_share_directory('tmc_point_cloud_accumulator')
    params_file = os.path.join(get_package_share_directory('tmc_point_cloud_accumulator'),
                               'test/parameter', 'point_cloud_accumulator_config_test.yaml')
    node = launch_ros.actions.Node(
        package='tmc_point_cloud_accumulator',
        executable='point_cloud_accumulator_node',
        name='point_cloud_accumulator', output='screen',
        remappings=[('input_point_cloud', '/urg_cloud'),
                    ('accumulated_point_cloud', 'obstacle_test_cloud')],
        parameters=[params_file,
                    {'use_sim_time': True}]
    )

    # To prevent the test from failing to end when rosbag playback finishes during the test, loop playback of rosbag
    rosbag_play_node = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', share_dir + '/test/rosbag/hsrb_hcr2013_test', '--clock', '-l'],
        output='screen'
    )

    return LaunchDescription([
        node,
        rosbag_play_node,
        launch_testing.actions.ReadyToTest()])


class PointCloudAccumulatorTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self._node = rclpy.create_node('point_cloud_accumulator_system_test')
        use_sim_time_param = rclpy.parameter.Parameter(
            'use_sim_time',
            rclpy.Parameter.Type.BOOL,
            True
        )
        self._node.set_parameters([use_sim_time_param])

        # Setting test parameters
        self._parameter_client = AsyncParameterClient(self._node, '/point_cloud_accumulator')
        self._setup_test_parameter()
        self._is_point_cloud_subscribed = False
        self._rate = self._node.create_rate(TEST_RATE)

        # Wait until receiving the clock from rosbag,
        # Ensure the test is conducted using the clock from rosbag
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1)
        result, _ = wait_for_message(Clock, self._node, '/clock', qos_profile=qos_policy, time_to_wait=10)
        self.assertTrue(result)

        self._point_cloud_subscriber = self._node.create_subscription(PointCloud2, self._point_cloud_name,
                                                                      self._point_cloud_callback, 1)

        self._tf_buffer = Buffer()
        TransformListener(self._tf_buffer, self._node)

        self._executor = SingleThreadedExecutor()
        self._thread = threading.Thread(target=rclpy.spin, args=(self._node, self._executor), daemon=True)
        self._thread.start()

    # Callback function for point cloud
    def _point_cloud_callback(self, msg):
        self._point_cloud_x_coordinate_list = []
        self._x_coordinate_from_point_cloud(msg)
        self._is_point_cloud_subscribed = True

    # Extract the x-coordinate from the point cloud
    def _x_coordinate_from_point_cloud(self, msg):
        for i in range(int(len(msg.data) / 16)):
            msg_data_x_coordinate = msg.data[i * 16:i * 16 + 4]
            x_coordinate_byte_data = msg_data_x_coordinate.tobytes()
            x_coordinate_data = unpack('<f', x_coordinate_byte_data)[0]
            self._point_cloud_x_coordinate_list.append(x_coordinate_data)

    # Obtain robot position from TF
    def _get_base_pose(self):
        can_transform = self._tf_buffer.can_transform(self._map_frame, self._base_frame, rclpy.time.Time(),
                                                      rclpy.duration.Duration(seconds=WAIT_TF_PERIOD))
        if can_transform:
            self._base_tf = self._tf_buffer.lookup_transform(self._map_frame, self._base_frame, rclpy.time.Time())
        else:
            self._node.get_logger().error('Cannot transform ' + str(self._map_frame) + ' to ' + str(self._base_frame))

    # Wait until point cloud can be acquired
    def _wait_for_point_cloud(self, timeout):
        self._is_point_cloud_subscribed = False
        wait_start_time = self._node.get_clock().now().nanoseconds
        while (not self._is_point_cloud_subscribed
               and (self._node.get_clock().now().nanoseconds - wait_start_time) < timeout * 1e9):
            self._rate.sleep()
        if not self._is_point_cloud_subscribed:
            return False
        return True

    # Setting test parameters
    def _setup_test_parameter(self):
        self._point_cloud_name = 'obstacle_test_cloud'
        result = self._parameter_client.wait_for_services(timeout_sec=10.0)
        self.assertTrue(result)
        get_future = self._parameter_client.get_parameters(['map_frame_name',
                                                            'base_frame_name',
                                                            'point_cloud_keep_period'])
        rclpy.spin_until_future_complete(self._node, get_future)
        parames = get_future.result().values
        self._map_frame = parames[0].string_value
        self._base_frame = parames[1].string_value
        self._keep_period = parames[2].double_value

    def test_keep_point_cloud_in_given_period(self):
        # Wait until point cloud becomes available
        while not self._is_point_cloud_subscribed:
            self._rate.sleep()

        # Obtain current robot position
        self._get_base_pose()
        last_x = self._base_tf.transform.translation.x

        is_base_moving = False
        start_time = self._node.get_clock().now().nanoseconds
        last_time = 0

        # Conduct the test while reading data replayed from rosbag
        while (rclpy.ok()
               and self._wait_for_point_cloud(WAIT_POINT_CLOUD_PERIOD)
               and (self._node.get_clock().now().nanoseconds - start_time < TEST_PERIOD * 1e9)):
            # Confirm that rosbag has not finished playback (loop)
            now_time = self._node.get_clock().now().nanoseconds
            self.assertLessEqual(last_time, now_time)
            last_time = now_time

            # Extract the coordinates of the point with the maximum x-coordinate in the output point cloud
            max_x = max(self._point_cloud_x_coordinate_list)

            # Obtain current robot position
            self._get_base_pose()
            current_x = self._base_tf.transform.translation.x

            # Save the maximum x-coordinate of the point cloud when the robot starts moving
            diff_x = current_x - last_x
            if not is_base_moving:
                if diff_x < -MOVE_START_THRESHOLD:
                    max_x_start = max_x
                    move_start = self._node.get_clock().now().nanoseconds
                    is_base_moving = True
            else:
                time_from_start = self._node.get_clock().now().nanoseconds - move_start
                diff_max_x = max_x - max_x_start
                # Check if the point cloud is retained after the robot starts moving
                if time_from_start < (self._keep_period - TIME_MERGIN) * 1e9:
                    # Verify if the maximum x-coordinate of the point cloud does not change during the retention period
                    # In other words, check if the point cloud is retained relative to the start of movement
                    self.assertGreater(diff_max_x, -MOVE_PCL_THRESHOLD)
                elif time_from_start > (self._keep_period + TIME_MERGIN) * 1e9:
                    # Verify if the maximum x-coordinate of the point cloud changes after the retention period
                    # In other words, check if the point cloud correctly disappears after the retention period
                    self.assertLess(diff_max_x, -MOVE_PCL_THRESHOLD)
            last_x = current_x
