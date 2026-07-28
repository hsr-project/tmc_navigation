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

from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
import launch_ros.actions


def generate_launch_description():
    rate = DeclareLaunchArgument("rate", default_value="30.0")
    frame_id = DeclareLaunchArgument("frame_id", default_value=TextSubstitution(text="odom"))
    child_frame_id = DeclareLaunchArgument("child_frame_id", default_value=TextSubstitution(text="base_footprint"))
    static_odom_topic = DeclareLaunchArgument("static_odom_topic", default_value=TextSubstitution(text="static_odom"))
    node = launch_ros.actions.Node(
        package='tmc_imu_wheel_odometry',
        executable='static_odom_node',
        name='static_odom_node', output='screen',
        parameters=[{'rate': LaunchConfiguration('rate')},
                    {'frame_id': LaunchConfiguration('frame_id')},
                    {'child_frame_id': LaunchConfiguration('child_frame_id')}],
        remappings=[('static_odom', LaunchConfiguration('static_odom_topic'))])
    return LaunchDescription([
        rate, frame_id, child_frame_id, static_odom_topic,
        node])
