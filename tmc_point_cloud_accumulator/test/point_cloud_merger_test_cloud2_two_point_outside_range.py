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
import os
from test import point_cloud_merger_test_node
import threading
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions
import launch_testing
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor


@pytest.mark.launch_test
def generate_test_description():
    params_file = os.path.join(get_package_share_directory('tmc_point_cloud_accumulator'),
                               'test/parameter', 'point_cloud_merger_config_test.yaml')

    node_tf = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '-0.228', '--y', '0.022', '--z', '0.808',
            '--qx', '-0.5', '--qy', '0.5', '--qz', '-0.5', '--qw', '0.5',
            '--frame-id', '/frame1', '--child-frame-id', '/frame2']
    )

    # Evaluate by switching the effective measurement range [m] (cloud_1_range, cloud_2_range) of the point cloud
    # Determine points within the effective range where x^2 + y^2 + z^2 < range^2 holds true for each point's coordinates
    #   Input point cloud
    #     input_1 : [0.5, 0.5, 0.0]
    #     input_2 : [1.0, 1.0, 0.0], [1.5, 1.5, 0.0]
    # In this test item, two points of input_2 ([1.0, 1.0, 0.0], [1.5, 1.5, 0.0]) are determined to be outside the effective range, and one point of input_1 ([0.5, 0.5, 0.0]) is merged
    node = launch_ros.actions.Node(
        package='tmc_point_cloud_accumulator',
        executable='point_cloud_merger_node',
        name='point_cloud_merger', output='screen',
        remappings=[('input_point_cloud_1', 'input_1'),
                    ('input_point_cloud_2', 'input_2')],
        parameters=[params_file,
                    {'cloud_1_range': 1.0},
                    {'cloud_2_range': 1.0}]
    )

    return LaunchDescription([
        node_tf,
        node,
        launch_testing.actions.ReadyToTest()])


class PointCloudMergerTestCloud2TwoPointOutsideRange(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self._test_node = point_cloud_merger_test_node.PointCloudMergerTestNode()
        self._executor = SingleThreadedExecutor()
        self._thread = threading.Thread(target=rclpy.spin, args=(self._test_node, self._executor), daemon=True)
        self._thread.start()

    def test_cloud2_two_point_outside_range(self):
        wait_period = 10  # s
        # input_2 is outside the effective range
        result = self._test_node.wait_for_merged_cloud(wait_period)
        self.assertTrue(result)
        # The reference coordinates are based on the coordinate system of input_1
        merged_cloud = self._test_node.merged_cloud()
        self.assertEqual(merged_cloud.header.frame_id, 'frame1')
        # One point from input_1 is output
        self.assertEqual(self._test_node.count_cloud_points(merged_cloud), 1)
