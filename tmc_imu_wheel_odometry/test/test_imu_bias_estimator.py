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
import unittest

from geometry_msgs.msg import Vector3
import numpy as np

from tmc_imu_wheel_odometry.imu_reset_bias import (
    NormalDistributionModel,
    Vector3MeanEstimator)


class NormalDistributionModelTest(unittest.TestCase):
    def test_empty_operation(self):
        # setup
        target = NormalDistributionModel()

        # excercise
        r = target.mean()

        # verify
        self.assertTrue(np.isnan(r))

    def test_length_behavior(self):
        # setup
        target = NormalDistributionModel()

        # exercise and verify
        expect_zero_init = target.data_list_len()
        target.append(1.)
        expect_one = target.data_list_len()
        target.initialize()
        expect_zero = target.data_list_len()

        # verify
        self.assertEqual(0, expect_zero_init)
        self.assertEqual(1, expect_one)
        self.assertEqual(0, expect_zero)

    def test_expected_mean(self):
        # setup
        test_data = np.array([np.random.normal(1., 1.) for i in range(1000)])
        expected_mean = test_data.mean()
        for outlier in [1e10, -1e10]:
            test_data = np.append(test_data, outlier)
        test_data_len = len(test_data)
        target = NormalDistributionModel()

        # excercise
        for e in test_data:
            target.append(e)
        mean = target.mean()

        # verify
        self.assertLess(target.data_list_len(), test_data_len)
        self.assertAlmostEqual(expected_mean, mean)


class Vector3MeanEstimatorTest(unittest.TestCase):
    def test_empty_operation(self):
        # setup
        target = Vector3MeanEstimator()

        # excercise
        r = target.mean()

        # verify
        self.assertTrue(np.isnan(r.x))
        self.assertTrue(np.isnan(r.y))
        self.assertTrue(np.isnan(r.z))

    def test_length_behavior(self):
        # setup
        target = Vector3MeanEstimator()

        # exercise and verify
        expect_zero_init = target.data_list_len()
        target.append(Vector3())
        expect_one = target.data_list_len()
        target.initialize()
        expect_zero = target.data_list_len()

        # verify
        self.assertEqual(0, expect_zero_init)
        self.assertEqual(1, expect_one)
        self.assertEqual(0, expect_zero)

    def test_expected_mean(self):
        # setup
        test_vectors = []
        test_elements = []
        for i in range(1000):
            v = np.random.normal(1., 1.)
            test_elements.append(v)
            test_vectors.append(Vector3(x=v, y=v, z=v))
        expected_mean = np.array(test_elements).mean()
        for outlier in [1e10, -1e10]:
            test_vectors.append(Vector3(x=outlier, y=outlier, z=outlier))
        test_vectors_len = len(test_vectors)
        target = Vector3MeanEstimator()

        # excercise
        for e in test_vectors:
            target.append(e)
        mean = target.mean()

        # verify
        self.assertLess(target.data_list_len(), test_vectors_len)
        self.assertAlmostEqual(expected_mean, mean.x)
        self.assertAlmostEqual(expected_mean, mean.y)
        self.assertAlmostEqual(expected_mean, mean.z)
