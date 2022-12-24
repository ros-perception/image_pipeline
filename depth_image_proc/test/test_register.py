#!/usr/bin/env python

import copy
import unittest

import numpy as np
import rospy
import rostest
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image


class TestRegister(unittest.TestCase):
    def test_register(self):
        # publish a 2x2 float depth image with depths like
        # [1.0, 2.0], [2.0, 1.0]
        # publish a camera info with a aov of around 90 degrees
        # publish two static tfs in the .test file, one shifted
        # by 1.0 in x from the depth camera_info frame
        # another by 2.0 in x, and 1.5 in z, with a 90 degree rotation
        # looking back at those depth points
        # publish another camera info, in series, have it shift from one frame to the other
        # subscribe to the depth_image_proc node and verify output
        # (make numbers come out even with 90 degree aov)

        self.output_depth = rospy.get_param("~output_depth", 1.0)
        self.expected_depth = rospy.get_param("~expected_depth", 7.0)
        rospy.loginfo(f"published depth: {self.output_depth}, expected input depth: {self.expected_depth}")

        self.cv_bridge = CvBridge()
        self.depth_image_pub = rospy.Publisher("depth/image_rect", Image, queue_size=2)
        self.depth_ci_pub = rospy.Publisher("depth/camera_info", CameraInfo, queue_size=2)
        self.rgb_ci_pub = rospy.Publisher("rgb/camera_info", CameraInfo, queue_size=2)

        self.received_msg = None

        # TODO(lucasw) use time sync subscriber
        self.depth_sub = rospy.Subscriber("depth_registered/image_rect", Image, self.depth_callback, queue_size=2)
        self.ci_sub = rospy.Subscriber("depth_registered/camera_info", CameraInfo, self.ci_callback, queue_size=2)

        ci_msg = CameraInfo()
        wd = 3
        ht = 3
        ci_msg.header.frame_id = "station1"
        ci_msg.height = ht
        ci_msg.width = wd
        ci_msg.distortion_model = "plumb_bob"

        cx = 1.5
        cy = 1.5
        fx = 1.5
        fy = 1.5
        ci_msg.K = [fx, 0.0, cx,
                    0.0, fy, cy,
                    0.0, 0.0, 1.0]
        ci_msg.R = [1, 0, 0,
                    0, 1, 0,
                    0, 0, 1]
        ci_msg.P = [fx, 0.0, cx, 0.0,
                    0.0, fy, cy, 0.0,
                    0.0, 0.0, 1.0, 0.0]

        depth = np.ones((ht, wd, 1), np.float32) * self.output_depth
        rospy.loginfo(depth)

        depth_msg = self.cv_bridge.cv2_to_imgmsg(depth, "32FC1")
        ci_msg.header.stamp = rospy.Time.now()
        rgb_ci_msg = copy.deepcopy(ci_msg)
        rgb_ci_msg.header.frame_id = "station2"
        depth_msg.header = ci_msg.header

        for i in range(6):
            rospy.loginfo(f"{i} waiting for register connections...")
            num_im_sub = self.depth_image_pub.get_num_connections()
            num_ci_sub = self.depth_ci_pub.get_num_connections()
            num_rgb_ci_sub = self.rgb_ci_pub.get_num_connections()
            rospy.sleep(1)
            if num_im_sub > 0 and num_ci_sub > 0 and num_rgb_ci_sub > 0:
                break

        self.assertGreater(self.depth_image_pub.get_num_connections(), 0)
        self.assertGreater(self.depth_ci_pub.get_num_connections(), 0)
        self.assertGreater(self.rgb_ci_pub.get_num_connections(), 0)
        rospy.sleep(1.0)

        self.count = 0
        rospy.loginfo("publishing depth and ci, wait for callbacks")
        for i in range(4):
            self.depth_image_pub.publish(depth_msg)
            self.depth_ci_pub.publish(ci_msg)
            self.rgb_ci_pub.publish(rgb_ci_msg)
            rospy.sleep(0.1)

        while i in range(6):
            if self.count > 1:
                break
            rospy.sleep(1)
        rospy.loginfo("done waiting")

        self.assertIsNotNone(self.received_msg, "no depth message received")
        registered_depth = self.cv_bridge.imgmsg_to_cv2(self.received_msg, "32FC1")
        # the edges of the expected 3x3 image may be nans, but the center should be valid
        valid_depth = registered_depth[1, 1]
        self.assertAlmostEqual(valid_depth, self.expected_depth, places=1)

    def depth_callback(self, msg):
        rospy.loginfo("received depth")
        self.received_msg = msg
        self.count += 1

    def ci_callback(self, msg):
        rospy.logdebug("received camera info")


if __name__ == "__main__":
    node_name = 'test_register'
    rospy.init_node(node_name)
    # rostest.rosrun("depth_image_proc", node_name, sys.argv)
    rostest.rosrun("depth_image_proc", node_name, TestRegister)
