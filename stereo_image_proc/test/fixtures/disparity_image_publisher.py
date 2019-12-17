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

import array
import sys

import cv2

import numpy

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from stereo_msgs.msg import DisparityImage


class DisparityImagePublisher(Node):
    """
    Disparity image publisher test fixture.

    Publishes test data on topics expected by stereo_image_proc nodes:

       - left image
       - left camera info
       - right camera info
       - disparity image
    """

    def __init__(
        self,
        left_image: numpy.ndarray,
        disparity_image: numpy.ndarray,
        *,
        timer_period: float = 0.1
    ):
        """
        Construct a stereo image publisher.

        :param: left_image The image to publish on the left topic.
        :param: right_image The image to publish on the right topic.
        :param: timer_period The period in seconds at which messages are published.
        """
        super().__init__('image_publisher')
        self.left_image_and_info = self._create_image_and_info_messages(left_image)
        self.disparity_image = DisparityImage()
        self.disparity_image.image.height = disparity_image.shape[0]
        self.disparity_image.image.width = disparity_image.shape[1]
        self.disparity_image.image.step = self.disparity_image.image.width
        self.disparity_image.image.data = array.array('B', disparity_image.tobytes())

        self.left_image_pub = self.create_publisher(Image, 'left/image_rect_color', 1)
        self.left_camera_info_pub = self.create_publisher(CameraInfo, 'left/camera_info', 1)
        self.right_camera_info_pub = self.create_publisher(CameraInfo, 'right/camera_info', 1)
        self.disparity_image_pub = self.create_publisher(DisparityImage, 'disparity', 1)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def _create_image_and_info_messages(self, image):
        image_msg = Image()
        image_msg.height = image.shape[0]
        image_msg.width = image.shape[1]
        image_msg.encoding = 'bgr8'
        image_msg.step = image_msg.width * 3
        image_msg.data = array.array('B', image.tobytes())

        camera_info_msg = CameraInfo()
        camera_info_msg.height = image.shape[0]
        camera_info_msg.width = image.shape[1]

        return (image_msg, camera_info_msg)

    def timer_callback(self):
        now = self.get_clock().now().to_msg()
        self.left_image_and_info[0].header.stamp = now
        self.left_image_and_info[1].header.stamp = now
        self.disparity_image.header.stamp = now

        self.left_image_pub.publish(self.left_image_and_info[0])
        self.left_camera_info_pub.publish(self.left_image_and_info[1])
        self.right_camera_info_pub.publish(self.left_image_and_info[1])
        self.disparity_image_pub.publish(self.disparity_image)


if __name__ == '__main__':
    rclpy.init()
    left_image = cv2.imread(sys.argv[1])
    disparity_image = cv2.imread(sys.argv[2], cv2.IMREAD_GRAYSCALE).astype(float)
    publisher = DisparityImagePublisher(left_image, disparity_image)
    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        rclpy.shutdown()
