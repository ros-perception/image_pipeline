# Copyright (c) 2020, Open Source Robotics Foundation, Inc.
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
from launch_ros import actions

from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    ld = LaunchDescription()

    # Load composable container
    image_processing = actions.ComposableNodeContainer(
        node_name='image_proc_container',
        package='rclcpp_components',
        node_executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='image_proc',
                node_plugin='image_proc::DebayerNode',
                node_name='debayer_node',
            ),
            # Example of rectifying an image
            ComposableNode(
                package='image_proc',
                node_plugin='image_proc::RectifyNode',
                node_name='rectify_mono_node',
                # Remap subscribers and publishers
                remappings=[
                    # Subscriber remap
                    ('image', 'image_mono'),
                    ('camera_info', 'camera_info'),
                    ('image_rect', 'image_rect')
                ],
            ),
            # Example of rectifying an image
            ComposableNode(
                package='image_proc',
                node_plugin='image_proc::RectifyNode',
                node_name='rectify_color_node',
                # Remap subscribers and publishers
                remappings=[
                    # Subscriber remap
                    ('image', 'image_color'),
                    # Publisher remap
                    ('image_rect', 'image_rect_color')
                ],
            )],
        output='screen'
    )

    ld.add_action(image_processing)

    return ld
