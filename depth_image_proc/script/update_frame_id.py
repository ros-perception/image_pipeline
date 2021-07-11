#!/usr/bin/env python 

import rospy
from sensor_msgs.msg import Image 
from sensor_msgs.msg import CameraInfo

rospy.init_node("update_frame_id")

#Updating frame id for the error depth_front frame id does not match rgb_front frame id
class update_frame_id:
    def __init__(self):
        self.image = Image()
        #subscribe to your specific sensors
        self.sub_raw = rospy.Subscriber("/carla/ego_vehicle/rgb_front/image", Image, self.callback_raw)
        self.sub_info = rospy.Subscriber("/carla/ego_vehicle/rgb_front/camera_info", CameraInfo, self.callback_info)
        self.pub_raw = rospy.Publisher("/rgb/image_rect_color", Image, queue_size = 1)
        self.pub_info = rospy.Publisher("/rgb/camera_info", CameraInfo, queue_size = 1)
    def callback_raw(self, message):
        message.header.frame_id = "ego_vehicle/depth_front"
        self.pub_raw.publish(message)
    def callback_info(self, message):
        message.header.frame_id = "ego_vehicle/depth_front"
        self.pub_info.publish(message)

update_frame_id = update_frame_id()
rospy.spin()

print("\nNode shutdown\n")
