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
from math import cos, pi, sin, sqrt
import test.ros_test_utils as ros_test_utils
import threading
import time
import unittest

import launch
import launch_ros.actions
import launch_testing

from nav_msgs.msg import Odometry

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from tmc_imu_wheel_odometry import lower_imu_odom

WHEEL_RADIUS = 0.1


@pytest.mark.launch_test
def generate_test_description():
    lower_imu_odom_node = launch_ros.actions.Node(
        package='tmc_imu_wheel_odometry',
        executable='lower_imu_odom_node',
        parameters=[{'wheel_radius_l': WHEEL_RADIUS},
                    {'wheel_radius_r': WHEEL_RADIUS},
                    {'caster_offset': 0.2},
                    {'vel_std': [0.05, 0.05, 0.2]},
                    {'valid_joint_vel_ths': [100.0, 100.0, 100.0]}],
        output='screen'
    )
    return launch.LaunchDescription(
        [lower_imu_odom_node, launch_testing.actions.ReadyToTest()])


class LowerImuOdomTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self._node = rclpy.create_node("test_lower_imu_odom")
        self.first_odom = None
        self.last_odom = None

        # publisher
        self.pub_imu_ = ros_test_utils.VariableImu(
            self._node, topic='imu', rate_hz=100.0)
        self.pub_joint_ = ros_test_utils.VariableJoint(
            self._node, topic='joint_state', rate_hz=50.0)

        # Subscriber
        self.sub_odom_ = self._node.create_subscription(
            Odometry, 'imu_odom', self.odom_cb, 10)

        self._executor = SingleThreadedExecutor()
        self._thread = threading.Thread(target=rclpy.spin, args=(self._node, self._executor), daemon=True)
        self._thread.start()

    def odom_cb(self, odom_msg):
        if self.first_odom is None:
            self.first_odom = odom_msg

        self.last_odom = odom_msg

    def run_checking_odom(
            self, duration, expected_x, expected_y, expected_yaw, delta=None):
        # run pub/sub
        self.pub_imu_.start()
        self.pub_joint_.start()
        time.sleep(duration)
        self.pub_imu_.stop()
        self.pub_joint_.stop()
        # check
        d_trans, d_euler = ros_test_utils.get_mat_pose_diff(
            self.first_odom, self.last_odom
        )

        # trans check
        self.assertAlmostEqual(d_trans[0], expected_x, delta=delta)
        self.assertAlmostEqual(d_trans[1], expected_y, delta=delta)
        self.assertAlmostEqual(d_trans[2], 0.0)

        # angle check
        self.assertAlmostEqual(d_euler[0], 0.0)
        self.assertAlmostEqual(d_euler[1], 0.0)
        self.assertAlmostEqual(d_euler[2], expected_yaw, delta=delta)

    def connection_check(self):
        # Sometimes the topic published immediately after creating a publisher/subscriber cannot be received
        # Add a wait at the start of the test to stabilize the test
        # TODO(kazuki_shibamiya) : トピックが漏れなく受信できるようになったらwaitを削除する
        time.sleep(1.0)
        # connection test
        self.assertTrue(self.pub_imu_.check_connection())
        self.assertTrue(self.pub_joint_.check_connection())
        self.assertTrue(ros_test_utils.check_subscription_connection(self._node, self.sub_odom_))

    def test_normalize_angle(self):
        u"""Are angles in range of pi?"""
        # setup
        input_angles = [
            -3.0 * pi,
            -2.0 * pi,
            -1.0 * pi,
            0.0 * pi,
            1.0 * pi,
            2.0 * pi,
            3.0 * pi]
        expected_angles = [
            -1.0 * pi,
            0.0 * pi,
            -1.0 * pi,
            0.0 * pi,
            1.0 * pi,
            0.0 * pi,
            1.0 * pi]

        # exercise
        output_angles = [
            lower_imu_odom.normalize_angle(angle)
            for angle in input_angles]

        # verify
        for o, e in zip(output_angles, expected_angles):
            self.assertAlmostEqual(o, e)

    def test_pose_2_odom(self):
        u"""Can I get expected nav_msgs::Odometry?"""
        # setup
        input_pose = [
            1.,  # x
            2.,  # y
            pi / 3.]  # theta[rad]
        input_stamp = self._node.get_clock().now().to_msg()
        input_frame_id = 'input_frame_id'
        input_child_frame_id = 'input_child_frame_id'

        # exercise
        output_odom = lower_imu_odom.pose_2_odom(
            input_pose,
            input_stamp,
            input_frame_id,
            input_child_frame_id)

        # verify
        self.assertIsInstance(output_odom, Odometry)
        self.assertAlmostEqual(
            output_odom.pose.pose.position.x, input_pose[0])
        self.assertAlmostEqual(
            output_odom.pose.pose.position.y, input_pose[1])
        self.assertAlmostEqual(
            output_odom.pose.pose.position.z, 0.)
        self.assertAlmostEqual(
            output_odom.pose.pose.orientation.x, 0.)
        self.assertAlmostEqual(
            output_odom.pose.pose.orientation.y, 0.)
        self.assertAlmostEqual(
            output_odom.pose.pose.orientation.z,
            sin(input_pose[2] / 2.))
        self.assertAlmostEqual(
            output_odom.pose.pose.orientation.w,
            cos(input_pose[2] / 2.))
        self.assertAlmostEqual(
            output_odom.header.stamp, input_stamp)
        self.assertMultiLineEqual(
            output_odom.header.frame_id, input_frame_id)
        self.assertMultiLineEqual(
            output_odom.child_frame_id, input_child_frame_id)

    def test_to_twist_covariance(self):
        u"""Can I get expected covariance matrix?"""
        # setup
        input_stddev = [sqrt(2.), sqrt(3.), sqrt(4.)]
        expected_cov = [
            2., 0., 0., 0., 0., 0.,
            0., 3., 0., 0., 0., 0.,
            0., 0., 0., 0., 0., 0.,
            0., 0., 0., 0., 0., 0.,
            0., 0., 0., 0., 0., 0.,
            0., 0., 0., 0., 0., 4.]

        # exercise
        output_cov = lower_imu_odom.to_twist_covariance(input_stddev)

        # verify
        for o, e in zip(output_cov.tolist(), expected_cov):
            self.assertAlmostEqual(o, e)

    def test_no_move(self):
        self.connection_check()

        self.pub_imu_.set_angular_velocities(0.0, 0.0, 0.0)
        self.pub_joint_.set_joint_velocities(0.0, 0.0, 0.0)
        self.pub_joint_.set_joint_positions(0.0, 0.0, 0.0)

        self.run_checking_odom(1.0, 0.0, 0.0, 0.0)

    def test_go_straight(self):
        self.connection_check()

        v_rl = 1.0  # rad/sec
        duration = 1.0  # sec
        self.pub_imu_.set_angular_velocities(0.0, 0.0, 0.0)
        self.pub_joint_.set_joint_velocities(v_rl, v_rl, 0.0)

        self.run_checking_odom(
            duration,
            v_rl * WHEEL_RADIUS * duration, 0.0, 0.0,
            0.01)

    def test_go_straight_while_rotating(self):
        self.connection_check()

        v_rl = 1.0  # rad/sec
        duration = 1.0  # sec
        yaw_vel = 0.2
        self.pub_imu_.set_angular_velocities(0.0, 0.0, yaw_vel)
        self.pub_joint_.set_joint_velocities(v_rl, v_rl, -yaw_vel)

        self.run_checking_odom(
            duration,
            v_rl * WHEEL_RADIUS * duration, 0.0, yaw_vel * duration,
            0.01)
