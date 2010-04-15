PKG = 'camera_calibration' # this package name
import roslib; roslib.load_manifest(PKG)

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
for i in range(10):
    m = random_camerainfo()
    print m
    response = service(m)
    print response
    time.sleep(1)
    print track.val.P
    outcome = list(track.val.P) == list(m.P)
    print 'Outcome ====>', outcome
    assert outcome
