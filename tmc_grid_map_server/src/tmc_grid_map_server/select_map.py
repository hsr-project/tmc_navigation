#!/usr/bin/env python
# Copyright (c) 2025 TOYOTA MOTOR CORPORATION
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

import sys

import rospkg


def yes_or_no(ans):
    ans = ans.lower()
    if ans in ("yes", "y"):
        return True
    elif ans in ("no", "n"):
        return False
    raise ValueError("Please answer yes or no")


def create_map_link(map_name):

    rp = rospkg.RosPack()
    path = rp.get_path('tmc_potential_maps')

    src = '{0}/maps/{1}'.format(path, map_name)

    if os.environ.get("ROS_HOME") is None:
        dst = '{0}/.ros/map'.format(os.environ.get("HOME"))
        sys.stderr.write("ROS_HOME not exists")
    else:
        dst = '{0}/map'.format(os.environ.get("ROS_HOME"))

    if not os.path.exists(src):
        sys.stderr.write("No map file exists")
        sys.exit()

    if os.path.exists(dst):
        print("Create new symbolic link?\n(%s -> %s)\n yes/no" % (dst, src))

        try:
            if yes_or_no(input('-->')):
                print("Yes")
                os.remove(dst)
            else:
                print("No")
                sys.exit()
        except ValueError as e:
            print(e)

    os.symlink(src, dst)
    print("ln -s %s %s" % (src, dst))
