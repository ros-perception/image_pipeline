#! /usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
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

import rospy
import sensor_msgs.msg
import sensor_msgs.srv
import time
import random

def rfill(a):
    for i in range(len(a)):
        a[i] = random.randint(0, 10)

def random_camerainfo():
    m = sensor_msgs.msg.CameraInfo()
    m.height = 480
    m.width = 640
    rfill(m.D)
    rfill(m.K)
    rfill(m.R)
    rfill(m.P)
    return m

class CamInfoTracker:
    def __init__(self):
        self.val = None
        sub = rospy.Subscriber(rospy.remap_name('info'), sensor_msgs.msg.CameraInfo, self.setcam)
    def setcam(self, caminfo):
        self.val = caminfo
        self.age = time.time()

rospy.init_node('camera_hammer')
track = CamInfoTracker()

service = rospy.ServiceProxy("%s/set_camera_info" % rospy.remap_name("camera"), sensor_msgs.srv.SetCameraInfo)
for i in range(1000):
    print("\nIteration", i)
    m = random_camerainfo()
    print(m)
    response = service(m)
    print(response)
    start = rospy.get_time()
    outcome = False
    while True:
        try:
            outcome = list(track.val.P) == list(m.P)
        except:
            pass
        if outcome:
            break
        if rospy.get_time() - start > 5:
            break
        rospy.sleep(rospy.Duration(0.1))
    print(track.val.P)
    print('Outcome ====>', outcome)
    assert outcome
    assert response.success
