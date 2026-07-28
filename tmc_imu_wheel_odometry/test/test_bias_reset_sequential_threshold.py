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
import test.ros_test_utils as ros_test_utils
import threading
import unittest

import launch
import launch_ros.actions
import launch_testing
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor


@pytest.mark.launch_test
def generate_test_description():
    imu_reset_bias_node = launch_ros.actions.Node(
        package='tmc_imu_wheel_odometry',
        executable='imu_reset_bias_node',
        parameters=[{'reset_data_num': 250},
                    {'reset_srv_timeout': 5},
                    {'imu_angular_threshold': [0.02, 0.01]}],
        remappings=[('data_raw_br', 'data_br'),
                    ('data_raw', 'data_raw'),
                    ('cmd_vel', 'command_velocity')],
        output='screen'
    )
    return launch.LaunchDescription(
        [imu_reset_bias_node, launch_testing.actions.ReadyToTest()])


class ParamAndResult(object):
    def __init__(self, ang_vel, result, bias, canceled_vel):
        self.ang_vel = [ang_vel for i in range(3)]
        self.result = result
        self.bias = bias
        self.canceled_vel = canceled_vel


class BiasResetSequentialThresholdTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self._node = rclpy.create_node("test_bias_reset_sequential_threshold")
        self._test_io = ros_test_utils.TestIO(self._node)
        self._executor = SingleThreadedExecutor()
        self._thread = threading.Thread(target=rclpy.spin, args=(self._node, self._executor), daemon=True)
        self._thread.start()

    def test_behavior(self):
        # setup
        self._test_io.connection_check()
        # Can be reset with this value for the first time
        # From the second time onward, it fails if it exceeds the range of bias + new threshold
        # Must be smaller than the updated threshold
        param_and_results = [
            ParamAndResult(0.015, True, 0.015, 0.),
            ParamAndResult(0.015 + 0.011, False, 0.015, 0.011),
            ParamAndResult(0.015 + 0.009, True, 0.015 + 0.009, 0.)]

        # exercise
        output_bias_list = []
        output_imu_list = []
        for param_and_result in param_and_results:
            self._test_io.set_test_data(
                imu_angular=param_and_result.ang_vel,
                command_vel_angular=[0.0009, 0.0009, 0.0009],
                command_vel_linear=[0.0009, 0.0009, 0.0009])

            last_bias = self._test_io.bias()
            rate = self._node.create_rate(100.0)
            # Issue slightly more counts than the number (250) required for bias reset, considering potential misses
            for i in range(255):
                self._test_io.publish_once()
                rate.sleep()
            self.assertTrue(
                ros_test_utils.wait_until(
                    self._node,
                    lambda: True if last_bias != self._test_io.bias() else False, 3.0)
                == param_and_result.result)

            output_bias_list.append(self._test_io.bias())
            output_imu_list.append(self._test_io.imu().angular_velocity)

        # verify
        for i in range(len(param_and_results)):
            self.assertAlmostEqual(
                param_and_results[i].bias, output_bias_list[i].x)
            self.assertAlmostEqual(
                param_and_results[i].bias, output_bias_list[i].y)
            self.assertAlmostEqual(
                param_and_results[i].bias, output_bias_list[i].z)
            self.assertAlmostEqual(
                param_and_results[i].canceled_vel, output_imu_list[i].x)
            self.assertAlmostEqual(
                param_and_results[i].canceled_vel, output_imu_list[i].y)
            self.assertAlmostEqual(
                param_and_results[i].canceled_vel, output_imu_list[i].z)
