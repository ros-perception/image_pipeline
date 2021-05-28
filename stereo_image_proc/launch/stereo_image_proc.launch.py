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
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import LaunchConfigurationEquals
from launch.conditions import LaunchConfigurationNotEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import PushRosNamespace
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    composable_nodes = [
        ComposableNode(
            package='stereo_image_proc',
            plugin='stereo_image_proc::DisparityNode',
            parameters=[{
                'approximate_sync': LaunchConfiguration('approximate_sync'),
                'use_system_default_qos': LaunchConfiguration('use_system_default_qos'),
                'stereo_algorithm': LaunchConfiguration('stereo_algorithm'),
                'prefilter_size': LaunchConfiguration('prefilter_size'),
                'prefilter_cap': LaunchConfiguration('prefilter_cap'),
                'correlation_window_size': LaunchConfiguration('correlation_window_size'),
                'min_disparity': LaunchConfiguration('min_disparity'),
                'disparity_range': LaunchConfiguration('disparity_range'),
                'texture_threshold': LaunchConfiguration('texture_threshold'),
                'speckle_size': LaunchConfiguration('speckle_size'),
                'speckle_range': LaunchConfiguration('speckle_range'),
                'disp12_max_diff': LaunchConfiguration('disp12_max_diff'),
                'uniqueness_ratio': LaunchConfiguration('uniqueness_ratio'),
                'P1': LaunchConfiguration('P1'),
                'P2': LaunchConfiguration('P2'),
                'full_dp': LaunchConfiguration('full_dp'),
            }],
            remappings=[
                ('left/image_rect', [LaunchConfiguration('left_namespace'), '/image_rect']),
                ('left/camera_info', [LaunchConfiguration('left_namespace'), '/camera_info']),
                ('right/image_rect', [LaunchConfiguration('right_namespace'), '/image_rect']),
                ('right/camera_info', [LaunchConfiguration('right_namespace'), '/camera_info']),
            ]
        ),
        ComposableNode(
            package='stereo_image_proc',
            plugin='stereo_image_proc::PointCloudNode',
            parameters=[{
                'approximate_sync': LaunchConfiguration('approximate_sync'),
                'avoid_point_cloud_padding': LaunchConfiguration('avoid_point_cloud_padding'),
                'use_color': LaunchConfiguration('use_color'),
                'use_system_default_qos': LaunchConfiguration('use_system_default_qos'),
            }],
            remappings=[
                ('left/camera_info', [LaunchConfiguration('left_namespace'), '/camera_info']),
                ('right/camera_info', [LaunchConfiguration('right_namespace'), '/camera_info']),
                (
                    'left/image_rect_color',
                    [LaunchConfiguration('left_namespace'), '/image_rect_color']
                ),
            ]
        ),
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            name='approximate_sync', default_value='False',
            description='Whether to use approximate synchronization of topics. Set to true if '
                        'the left and right cameras do not produce exactly synced timestamps.'
        ),
        DeclareLaunchArgument(
            name='avoid_point_cloud_padding', default_value='False',
            description='Avoid alignment padding in the generated point cloud.'
                        'This reduces bandwidth requirements, as the point cloud size is halved.'
                        'Using point clouds without alignment padding might degrade performance '
                        'for some algorithms.'
        ),
        DeclareLaunchArgument(
            name='use_color', default_value='True',
            description='Generate point cloud with rgb data.'
        ),
        DeclareLaunchArgument(
            name='use_system_default_qos', default_value='False',
            description='Use the RMW QoS settings for the image and camera info subscriptions.'
        ),
        DeclareLaunchArgument(
            name='launch_image_proc', default_value='True',
            description='Whether to launch debayer and rectify nodes from image_proc.'
        ),
        DeclareLaunchArgument(
            name='left_namespace', default_value='left',
            description='Namespace for the left camera'
        ),
        DeclareLaunchArgument(
            name='right_namespace', default_value='right',
            description='Namespace for the right camera'
        ),
        DeclareLaunchArgument(
            name='container', default_value='',
            description=(
                'Name of an existing node container to load launched nodes into. '
                'If unset, a new container will be created.'
            )
        ),
        # Stereo algorithm parameters
        DeclareLaunchArgument(
            name='stereo_algorithm', default_value='0',
            description='Stereo algorithm: Block Matching (0) or Semi-Global Block Matching (1)'
        ),
        DeclareLaunchArgument(
            name='prefilter_size', default_value='9',
            description='Normalization window size in pixels (must be odd)'
        ),
        DeclareLaunchArgument(
            name='prefilter_cap', default_value='31',
            description='Bound on normalized pixel values'
        ),
        DeclareLaunchArgument(
            name='correlation_window_size', default_value='15',
            description='SAD correlation window width in pixels (must be odd)'
        ),
        DeclareLaunchArgument(
            name='min_disparity', default_value='0',
            description='Disparity to begin search at in pixels'
        ),
        DeclareLaunchArgument(
            name='disparity_range', default_value='64',
            description='Number of disparities to search in pixels (must be a multiple of 16)'
        ),
        DeclareLaunchArgument(
            name='texture_threshold', default_value='10',
            description='Filter out if SAD window response does not exceed texture threshold'
        ),
        DeclareLaunchArgument(
            name='speckle_size', default_value='100',
            description='Reject regions smaller than this size in pixels'
        ),
        DeclareLaunchArgument(
            name='speckle_range', default_value='4',
            description='Maximum allowed difference between detected disparities'
        ),
        DeclareLaunchArgument(
            name='disp12_max_diff', default_value='0',
            description='Maximum allowed difference in the left-right disparity check in pixels '
                        '(Semi-Global Block Matching only)'
        ),
        DeclareLaunchArgument(
            name='uniqueness_ratio', default_value='15.0',
            description='Filter out if best match does not sufficiently exceed the next-best match'
        ),
        DeclareLaunchArgument(
            name='P1', default_value='200.0',
            description='The first parameter ccontrolling the disparity smoothness '
                        '(Semi-Global Block Matching only)'
        ),
        DeclareLaunchArgument(
            name='P2', default_value='400.0',
            description='The second parameter ccontrolling the disparity smoothness '
                        '(Semi-Global Block Matching only)'
        ),
        DeclareLaunchArgument(
            name='full_dp', default_value='False',
            description='Run the full variant of the algorithm (Semi-Global Block Matching only)'
        ),
        ComposableNodeContainer(
            condition=LaunchConfigurationEquals('container', ''),
            package='rclcpp_components',
            executable='component_container',
            name='stereo_image_proc_container',
            namespace='',
            composable_node_descriptions=composable_nodes,
        ),
        LoadComposableNodes(
            condition=LaunchConfigurationNotEquals('container', ''),
            composable_node_descriptions=composable_nodes,
            target_container=LaunchConfiguration('container'),
        ),
        # If a container name is not provided,
        # set the name of the container launched above for image_proc nodes
        SetLaunchConfiguration(
            condition=LaunchConfigurationEquals('container', ''),
            name='container',
            value='stereo_image_proc_container'
        ),
        GroupAction(
            [
                PushRosNamespace(LaunchConfiguration('left_namespace')),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        FindPackageShare('image_proc'), '/launch/image_proc.launch.py'
                    ]),
                    launch_arguments={'container': LaunchConfiguration('container')}.items()
                ),
            ],
            condition=IfCondition(LaunchConfiguration('launch_image_proc')),
        ),
        GroupAction(
            [
                PushRosNamespace(LaunchConfiguration('right_namespace')),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        FindPackageShare('image_proc'), '/launch/image_proc.launch.py'
                    ]),
                    launch_arguments={'container': LaunchConfiguration('container')}.items()
                ),
            ],
            condition=IfCondition(LaunchConfiguration('launch_image_proc')),
        )
    ])
