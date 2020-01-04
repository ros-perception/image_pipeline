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
from launch.actions import ExecuteProcess
from launch.actions import OpaqueFunction

from launch_ros.actions import Node

import pytest

import rclpy

from sensor_msgs.msg import PointCloud2


@pytest.mark.rostest
def generate_test_description(ready_fn):

    path_to_disparity_image_publisher_fixture = os.path.join(
        os.path.dirname(__file__), 'fixtures', 'disparity_image_publisher.py')
    path_to_test_data = os.path.join(os.path.dirname(__file__), 'data')
    path_to_left_image = os.path.join(path_to_test_data, 'aloe-L.png')
    path_to_disparity_image = os.path.join(path_to_test_data, 'aloe-disp.png')

    return LaunchDescription([
        # Disparity image publisher
        # TODO(jacobperron): we can use Node in Eloquent
        ExecuteProcess(
            cmd=[
                sys.executable,
                path_to_disparity_image_publisher_fixture,
                path_to_left_image,
                path_to_disparity_image,
            ],
            output='screen'
        ),
        # PointCloudNode
        Node(
            package='stereo_image_proc',
            node_executable='point_cloud_node',
            node_name='point_cloud_node',
            output='screen'
        ),
        # TODO(jacobperron): In Eloquent, use 'launch_testing.actions.ReadyToTest()'
        OpaqueFunction(function=lambda context: ready_fn()),
    ])


class TestPointCloudNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        # TODO(jacobperron): Instead of handling the init/shutdown cycle, as of Eloqeunt
        #                    we can use the node 'launch_service.context.locals.launch_ros_node'
        rclpy.init()
        cls.node = rclpy.create_node('test_point_cloud_node')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_message_received(self):
        # Expect the point cloud node to publish on '/points2' topic
        msgs_received = []
        self.node.create_subscription(
            PointCloud2,
            'points2',
            lambda msg: msgs_received.append(msg),
            1
        )

        # Wait up to 10 seconds to receive message
        start_time = time.time()
        while len(msgs_received) == 0 and (time.time() - start_time) < 10:
            rclpy.spin_once(self.node, timeout_sec=(0.1))

        assert len(msgs_received) > 0
