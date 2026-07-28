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
import time
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
                    {'imu_angular_threshold': [0.02]}],
        remappings=[('data_raw_br', 'data_br'),
                    ('data_raw', 'data_raw'),
                    ('cmd_vel', 'command_velocity')],
        output='screen'
    )
    return launch.LaunchDescription(
        [imu_reset_bias_node, launch_testing.actions.ReadyToTest()])


class BiasResetFailCasesTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self._node = rclpy.create_node("test_bias_reset_fail_case")

        self._test_io = ros_test_utils.TestIO(self._node)
        self._executor = SingleThreadedExecutor()
        self._thread = threading.Thread(target=rclpy.spin, args=(self._node, self._executor), daemon=True)
        self._thread.start()

    def _test_fail_case(self):
        u"""Fail bias reset case for some reasons"""
        # setup
        # val_list order is ...
        # [imu_angular_velocities.x, y, z,
        #  twist_linear.x, y, z,
        #  twist_angular.x, y, z,
        #  sleep]
        val_list = [
            [0.0, 0.0, 0.0,
             0.0, 0.0, 0.0,
             0.0, 0.0, 0.0,
             2.0],  # Case where the time for bias reset is too short
            [0.1, 0.0, 0.0,
             0.0, 0.0, 0.0,
             0.0, 0.0, 0.0,
             3.0],  # Case where imu_angular_velocities.x is too large
            [0.0, 0.1, 0.0,
             0.0, 0.0, 0.0,
             0.0, 0.0, 0.0,
             3.0],  # Case where imu_angular_velocities.y is too large
            [0.0, 0.0, 0.1,
             0.0, 0.0, 0.0,
             0.0, 0.0, 0.0,
             3.0],  # Case where imu_angular_velocities.z is too large
            [0.0, 0.0, 0.0,
             0.1, 0.0, 0.0,
             0.0, 0.0, 0.0,
             3.0],  # Case where twist_linear.x is too large
            [0.0, 0.0, 0.0,
             0.0, 0.1, 0.0,
             0.0, 0.0, 0.0,
             3.0],  # Case where twist_linear.y is too large
            [0.0, 0.0, 0.0,
             0.0, 0.0, 0.1,
             0.0, 0.0, 0.0,
             3.0],  # Case where twist_linear.z is too large
            [0.0, 0.0, 0.0,
             0.0, 0.0, 0.0,
             0.1, 0.0, 0.0,
             3.0],  # Case where twist_angular.x is too large
            [0.0, 0.0, 0.0,
             0.0, 0.0, 0.0,
             0.0, 0.1, 0.0,
             3.0],  # Case where twist_angular.y is too large
            [0.0, 0.0, 0.0,
             0.0, 0.0, 0.0,
             0.0, 0.0, 0.1,
             3.0]  # Case where twist_angular.z is too large
        ]

        for val in val_list:
            self._test_io.set_test_data(
                imu_angular=val[0:3],
                command_vel_angular=val[3:6],
                command_vel_linear=val[6:9])

            self._test_io.start_publish()
            time.sleep(val[9])
            self._test_io.stop_publish()

            # Since bias_reset has not been completed, the message has not arrived yet
            self.assertEqual(self._test_io.imu(), None)
            self.assertEqual(self._test_io.bias(), None)
