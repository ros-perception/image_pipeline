# Copyright (c) 2008, Willow Garage, Inc.
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
#  * Neither the name of the Willow Garage nor the names of its
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

# uncomment if you want to launch rviz too in this launch
# import os
# from ament_index_python.packages import get_package_share_directory
import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription

import launch_ros.actions
import launch_ros.descriptions


def generate_launch_description():
    default_rviz = os.path.join(get_package_share_directory('depth_image_proc'),
                                'launch', 'rviz/point_cloud_xyzrgb.rviz')
    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='rviz', default_value='False',
            description='Launch RViz for viewing point cloud xyzrgb radial data'
        ),
        launch_ros.actions.Node(
            condition=launch.conditions.IfCondition(
                launch.substitutions.LaunchConfiguration('rviz')
            ),
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['--display-config', default_rviz]
        ),
        # launch plugin through rclcpp_components container
        # make sure remapped topics are available
        launch_ros.actions.ComposableNodeContainer(
            name='container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # Driver itself
                launch_ros.descriptions.ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyzrgbRadialNode',
                    name='point_cloud_xyzrgb_node',
                    remappings=[('rgb/image_raw', '/camera/color/image_raw'),
                                ('depth/image_raw', '/camera/aligned_depth_to_color/image_raw'),
                                ('points', '/camera/depth_registered/points')]
                ),
            ],
            output='screen',
        ),

        # rviz
        # launch_ros.actions.Node(
        #    package='rviz2', executable='rviz2', output='screen',
        #    arguments=['--display-config', default_rviz]),
    ])
