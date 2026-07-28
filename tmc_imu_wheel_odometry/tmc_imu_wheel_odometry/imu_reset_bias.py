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

from geometry_msgs.msg import Twist, Vector3
import numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSDurabilityPolicy, QoSProfile
from sensor_msgs.msg import Imu


##
# @brief Provide sequential value for bias threshold
class SequentialThreshold(object):
    ##
    # @brief Initialize
    #
    # @param list_threshold list of float
    #
    def __init__(self, list_threshold):
        if type(list_threshold) is not list:
            raise TypeError('Input threshold type is not list')
        for th in list_threshold:
            if type(th) is not float:
                raise TypeError('An element is not floating point value')
        self._thresholds = list_threshold
        self._id = 0

    ##
    # @brief Getter of value
    #
    # @return float
    def get(self):
        return self._thresholds[self._id]

    ##
    # @brief Indicate next value
    #
    # @return None
    def increment(self):
        if self._id < len(self._thresholds) - 1:
            self._id += 1


##
# @brief A storage that filter outlier against S.D.3
class NormalDistributionModel(object):
    ##
    # @brief Initialize buffer
    #
    def __init__(self):
        self.initialize()

    ##
    # @brief Initialize buffer
    #
    # @return None
    def initialize(self):
        self._data_list = []

    ##
    # @brief Get data num
    #
    # @return int
    def data_list_len(self):
        return len(self._data_list)

    ##
    # @brief Add data to buffer
    #
    # @param data float
    #
    # @return None
    def append(self, data):
        if len(self._data_list) == 0:
            self._data_list = np.array([data])
            return
        self._data_list = np.append(self._data_list, data)

    ##
    # @brief Return mean without outlier
    #
    # @return
    #
    # np.nan If no data in buffer
    #
    # float mean
    def mean(self):
        if len(self._data_list) == 0:
            return np.nan

        self._remove_outer_3sigma()
        return self._data_list.mean()

    ##
    # @brief Remove larger values than S.D.3
    #
    # @return None
    def _remove_outer_3sigma(self):
        # Do nothing if the standard deviation is extremely small
        # Evaluating with the max-min difference is safer when there are more data points
        if self._data_list.max() - self._data_list.min() < 1e-6:
            return

        # Estimate the interval where 99.7% of the data is likely to fall
        mean = self._data_list.mean()
        sigma3 = self._data_list.std() * 3.
        th_lower = mean - sigma3
        th_higher = mean + sigma3

        self._data_list = self._data_list[th_lower < self._data_list]
        self._data_list = self._data_list[self._data_list < th_higher]


##
# @brief 3D ND estimator
class Vector3MeanEstimator(object):
    ##
    # @brief Initialize xyz buffers
    #
    def __init__(self):
        self._data_x = NormalDistributionModel()
        self._data_y = NormalDistributionModel()
        self._data_z = NormalDistributionModel()

    ##
    # @brief Initialize xyz buffers
    #
    # @return None
    def initialize(self):
        self._data_x.initialize()
        self._data_y.initialize()
        self._data_z.initialize()

    ##
    # @brief Get data num
    #
    # @return int
    def data_list_len(self):
        return self._data_x.data_list_len()

    ##
    # @brief Add data to buffer
    #
    # @param vec3 Vector3
    #
    # @return None
    def append(self, vec3):
        self._data_x.append(vec3.x)
        self._data_y.append(vec3.y)
        self._data_z.append(vec3.z)

    ##
    # @brief Return mean vector without outlier
    #
    # @return Vector3
    def mean(self):
        return Vector3(
            x=self._data_x.mean(),
            y=self._data_y.mean(),
            z=self._data_z.mean())


##
# @brief Publisher of imu bias
class ImuResetBias(Node):
    ##
    # @brief Initialize buffer and ROS I/O
    #
    def __init__(self):
        super().__init__('imu_reset_bias_node')
        self._reset_flag = True
        self._vec3_mean_estimator = Vector3MeanEstimator()

        self._bias = Vector3(x=0., y=0., z=0.)

        # Publishers
        self._pub_imu = self.create_publisher(
            Imu, 'data_raw_br', 10)
        latching_qos = QoSProfile(
            depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._pub_bias = self.create_publisher(
            Vector3, 'bias', qos_profile=latching_qos)

        # Parameters
        self._reset_data_num = self.declare_parameter(
            'reset_data_num', 1000).value
        self._cmd_vel_angular_threshold = self.declare_parameter(
            'cmd_vel_angular_threshold', 0.001).value
        self._cmd_vel_linear_threshold = self.declare_parameter(
            'cmd_vel_linear_threshold', 0.001).value
        imu_angular_threshold = self.declare_parameter(
            'imu_angular_threshold', [0.010]).value

        self._imu_angular_threshold = SequentialThreshold(
            imu_angular_threshold)

        use_init_bias = self.declare_parameter(
            'use_init_bias', False).value
        init_bias = self.declare_parameter(
            'init_bias', [0.0, 0.0, 0.0]).value
        if use_init_bias and (init_bias is not None) and \
           (type(init_bias) is list) and (len(init_bias) == 3):
            self._bias.x = init_bias[0]
            self._bias.y = init_bias[1]
            self._bias.z = init_bias[2]
            self._initialized = True
            self.get_logger().info('Initial bias is set to {}'.format(self._bias))
            self._pub_bias.publish(self._bias)
        else:
            self._initialized = False

        # Subscribers
        self._sub_cmd_vel = self.create_subscription(
            Twist, 'cmd_vel', self._cmd_vel_cb, 10)
        self._sub_imu = self.create_subscription(
            Imu, 'data_raw', self.imu_cb, qos_profile_sensor_data)

    ##
    # @brief Switch flag that represent device moving or not
    #
    # @param msg geometry_msgs.msg.Twist
    #
    # @return None
    def _cmd_vel_cb(self, msg):
        if (abs(msg.linear.x) > self._cmd_vel_linear_threshold
           or abs(msg.linear.y) > self._cmd_vel_linear_threshold
           or abs(msg.linear.z) > self._cmd_vel_linear_threshold):
            self._reset_flag = False
            return
        if (abs(msg.angular.x) > self._cmd_vel_angular_threshold
           or abs(msg.angular.y) > self._cmd_vel_angular_threshold
           or abs(msg.angular.z) > self._cmd_vel_angular_threshold):
            self._reset_flag = False
            return

    ##
    # @brief Accumulate imu angular velocity without outlier
    #
    # Append data if reset flag is true else ignore
    #
    # Publish mean of accumulate vector if data num is enough
    #
    # @param msg sensor_msgs.msg.Imu
    #
    # @return None
    def imu_cb(self, msg):
        ang_vel = msg.angular_velocity
        # cancel bias
        # If it has never been reset, only 0 will be subtracted
        # The value after reset will be used
        ang_vel.x -= self._bias.x
        ang_vel.y -= self._bias.y
        ang_vel.z -= self._bias.z

        if (abs(ang_vel.x) > self._imu_angular_threshold.get()
           or abs(ang_vel.y) > self._imu_angular_threshold.get()
           or abs(ang_vel.z) > self._imu_angular_threshold.get()):
            self._reset_flag = False

        if self._reset_flag:
            self._vec3_mean_estimator.append(ang_vel)
        else:
            self._vec3_mean_estimator.initialize()
            self._reset_flag = True
        if self._vec3_mean_estimator.data_list_len() >= self._reset_data_num:
            if not self._initialized:
                self._initialized = True

            additional_bias = self._vec3_mean_estimator.mean()
            self._vec3_mean_estimator.initialize()

            # Additionally cancel or reflect the bias
            ang_vel.x -= additional_bias.x
            ang_vel.y -= additional_bias.y
            ang_vel.z -= additional_bias.z

            # Use the total bias for subsequent calculations
            self._bias.x += additional_bias.x
            self._bias.y += additional_bias.y
            self._bias.z += additional_bias.z

            # Set the threshold for future use
            self._imu_angular_threshold.increment()

            self._pub_bias.publish(self._bias)

        if self._initialized:
            # Start publishing only after bias reseted
            self._pub_imu.publish(msg)
