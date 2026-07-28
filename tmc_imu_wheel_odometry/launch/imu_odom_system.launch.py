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
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution


def generate_launch_description():
    share_dir = get_package_share_directory("tmc_imu_wheel_odometry")
    reset_data_num = DeclareLaunchArgument("reset_data_num", default_value="300")
    use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="false")
    imu_raw_topic = DeclareLaunchArgument(
        "imu_raw_topic", default_value=TextSubstitution(text="base_accurate_imu/data_raw"))
    br_imu_topic = DeclareLaunchArgument(
        "br_imu_topic", default_value=TextSubstitution(text="base_accurate_imu/data_br"))
    imu_topic = DeclareLaunchArgument(
        "imu_topic", default_value=TextSubstitution(text="base_accurate_imu/data"))
    cmd_vel_topic = DeclareLaunchArgument(
        "cmd_vel_topic", default_value=TextSubstitution(text="command_velocity"))
    imu_bias_topic = DeclareLaunchArgument(
        "imu_bias_topic", default_value=TextSubstitution(text="base_accurate_imu/bias"))
    joint_state_topic = DeclareLaunchArgument(
        "joint_state_topic", default_value=TextSubstitution(text="omni_base_controller/internal_state"))
    imu_odom_topic = DeclareLaunchArgument(
        "imu_odom_topic", default_value=TextSubstitution(text="imu_odom"))

    bias_estimator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([share_dir + "/launch/bias_estimator.launch.py"]),
        launch_arguments={
            "reset_data_num": LaunchConfiguration('reset_data_num'),
            "use_sim_time": LaunchConfiguration('use_sim_time'),
            "data_raw_topic": LaunchConfiguration('imu_raw_topic'),
            "data_raw_br_topic": LaunchConfiguration('br_imu_topic'),
            "bias_topic": LaunchConfiguration('imu_bias_topic'),
            "cmd_vel_topic": LaunchConfiguration('cmd_vel_topic')
        }.items()
    )
    # TODO(syuuhei_shiro): typeの復活
    # In ROS1 launch, the argument type allowed choosing between madgwick or simple, but
    # currently, only simple is implemented
    orientation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([share_dir + "/launch/orientation.launch.py"]),
        launch_arguments={
            "use_sim_time": LaunchConfiguration('use_sim_time'),
            "imu_in_topic": LaunchConfiguration('br_imu_topic'),
            "imu_out_topic": LaunchConfiguration('imu_topic'),
        }.items()
    )
    # TODO(syuuhei_shiro): typeの復活
    # In ROS1 launch, the argument type allowed choosing between lower, simple, or robot_localization, but
    # currently, only lower is implemented
    lower_imu_odom = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([share_dir + "/launch/lower_imu_odom.launch.py"]),
        launch_arguments={
            "imu_topic": LaunchConfiguration('imu_topic'),
            "joint_state_topic": LaunchConfiguration('joint_state_topic'),
            "base_angle_offset": "0.0",
            "wheel_radius_l": "0.0445",
            "wheel_radius_r": "0.0445",
            "caster_offset": "0.11",
            "use_sim_time": LaunchConfiguration('use_sim_time'),
            "imu_odom_topic": LaunchConfiguration('imu_odom_topic')
        }.items()
    )

    return LaunchDescription([
        reset_data_num, use_sim_time, imu_raw_topic, br_imu_topic, imu_topic,
        cmd_vel_topic, imu_bias_topic, joint_state_topic, imu_odom_topic,
        bias_estimator,
        orientation,
        lower_imu_odom
    ])
