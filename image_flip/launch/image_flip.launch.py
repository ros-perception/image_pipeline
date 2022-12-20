# Copyright (c) 2022, CHRISLab, Christopher Newport University
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

from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='image_flip', executable='image_flip_node',
            output='screen', name='camera_flip',
            remappings=[("image",                'camera/rgb/image_raw'),
                        ('camera_info',          'camera/rgb/camera_info'),
                        ('rotated/image',        'camera_rotated/image_rotated'),
                        ('rotated/camera_info',  'camera_rotated/camera_info'),
                        ('rotated/image/compressed',      'camera_rotated/image_rotated/compressed'),
                        ('rotated/image/compressedDepth', 'camera_rotated/image_rotated/compressedDepth'),
                        ('rotated/image/theora',          'camera_rotated/image_rotated/theora')
                       ],
            parameters=[{'output_frame_id': "camera_rotated",
                         'rotation_steps': 2,
                         'use_camera_info': True,
                         'input_qos': 'best_effort',
                         'output_qos': 'default',
                        }]
        )
    ])
