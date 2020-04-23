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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # TODO(jacobperron): Include image_proc launch file when it exists
    return LaunchDescription([
        DeclareLaunchArgument(
            name='approximate_sync', default_value='False',
            description='Whether to use approximate synchronization of topics. Set to true if '
                        'the left and right cameras do not produce exactly synced timestamps.'
        ),
        DeclareLaunchArgument(
            name='use_system_default_qos', default_value='False',
            description='Use the RMW QoS settings for the image and camera info subscriptions.'
        ),
        ComposableNodeContainer(
            package='rclcpp_components', node_executable='component_container',
            node_name='stereo_image_proc_container', node_namespace='',
            composable_node_descriptions=[
                ComposableNode(
                    package='stereo_image_proc',
                    node_plugin='stereo_image_proc::DisparityNode',
                    parameters=[{
                        'approximate_sync': LaunchConfiguration('approximate_sync'),
                        'use_system_default_qos': LaunchConfiguration('use_system_default_qos'),
                    }]
                ),
                ComposableNode(
                    package='stereo_image_proc',
                    node_plugin='stereo_image_proc::PointCloudNode',
                    parameters=[{
                        'approximate_sync': LaunchConfiguration('approximate_sync'),
                        'use_system_default_qos': LaunchConfiguration('use_system_default_qos'),
                    }]
                ),
            ],
        ),
    ])
