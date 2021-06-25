# Copyright (c) 2019, Open Source Robotics Foundation, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os
import sys
import time
import unittest

from launch import LaunchDescription

from launch_ros.actions import Node

import launch_testing

import pytest

import rclpy

from stereo_msgs.msg import DisparityImage


@pytest.mark.rostest
def generate_test_description():

    path_to_stereo_image_publisher_fixture = os.path.join(
        os.path.dirname(__file__), 'fixtures', 'stereo_image_publisher.py')
    path_to_test_data = os.path.join(os.path.dirname(__file__), 'data')
    path_to_left_image = os.path.join(path_to_test_data, 'aloe-L.png')
    path_to_right_image = os.path.join(path_to_test_data, 'aloe-R.png')

    return LaunchDescription([
        # Stereo image publisher
        Node(
            executable=sys.executable,
            arguments=[
                path_to_stereo_image_publisher_fixture,
                path_to_left_image,
                path_to_right_image
            ],
            output='screen'
        ),
        # DisparityNode
        Node(
            package='stereo_image_proc',
            executable='disparity_node',
            name='disparity_node',
            parameters=[
                {"use_system_default_qos": True}
            ],
            output='screen'
        ),
        launch_testing.actions.ReadyToTest(),
    ])


class TestDisparityNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_disparity_node')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_message_received(self):
        # Expect the disparity node to publish on '/disparity' topic
        msgs_received = []
        self.node.create_subscription(
            DisparityImage,
            'disparity',
            lambda msg: msgs_received.append(msg),
            1
        )

        # Wait up to 60 seconds to receive message
        start_time = time.time()
        while len(msgs_received) == 0 and (time.time() - start_time) < 60:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        assert len(msgs_received) > 0

        # TODO(jacobperron): Compare received disparity image against expected
