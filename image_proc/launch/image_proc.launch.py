# Copyright 2018 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

""" A simple launch file for the nmea_serial_driver node. """

import os
import sys

import launch
from launch import LaunchDescription, LaunchIntrospector, LaunchService
from launch_ros import actions, get_default_launch_description

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

NAMESPACE = "/camera"

def generate_launch_description():
    """
    Generate a launch description for image processing nodes.
    """
    ld = LaunchDescription()

    image_processing = actions.ComposableNodeContainer(
        node_name="image_proc_container",
        node_namespace=NAMESPACE,
        package='rclcpp_components',
        node_executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='image_proc',
                node_plugin='image_proc::DebayerNode',
                node_name='debayer_node',
                node_namespace=NAMESPACE,
                parameters=[
                    {"camera_namespace": NAMESPACE}
                ]
            ),
            ComposableNode(
                package='image_proc',
                node_plugin='image_proc::RectifyNode',
                node_name='rectify_mono_node',
                node_namespace=NAMESPACE,
                parameters=[
                    {"camera_namespace": NAMESPACE},
                    {"image_mono": "/image_mono"}
                ]
            ),
            ComposableNode(
                package='image_proc',
                node_plugin='image_proc::RectifyNode',
                node_name='rectify_color_node',
                node_namespace=NAMESPACE,
                parameters=[
                    {"camera_namespace": NAMESPACE},
                    {"image_color": "/image_color"}
                ]
            )],
        output='screen'
    )

    ld.add_action(image_processing)

    return ld


def main(argv):
    ld = generate_launch_description()

    print('Starting introspection of launch description...')
    print('')

    print(LaunchIntrospector().format_launch_description(ld))

    print('')
    print('Starting launch of launch description...')
    print('')

    ls = LaunchService()
    ls.include_launch_description(get_default_launch_description())
    ls.include_launch_description(ld)

    return ls.run()


if __name__ == '__main__':
    main(sys.argv)