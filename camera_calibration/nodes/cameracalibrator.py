#!/usr/bin/python
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

PKG = 'camera_calibration' # this package name
import roslib; roslib.load_manifest(PKG)

import rospy
import sensor_msgs.msg
import sensor_msgs.srv
import cv_bridge

import math
import os
import sys
import operator
import time
import Queue
import threading
import tarfile
import pickle
import random
import StringIO
import functools

import cv

import message_filters
from camera_calibration.approxsync import ApproximateSynchronizer

ID_LOAD=101
ID_SAVE=102
ID_BUTTON1=110
ID_EXIT=200

# /wg/osx/rosCode/ros-pkg/ros-pkg/stacks/image_pipeline/image_view/preCalib

from camera_calibration.calibrator import cvmat_iterator, MonoCalibrator, StereoCalibrator, ChessboardInfo
from std_msgs.msg import String
from std_srvs.srv import Empty

class ConsumerThread(threading.Thread):
    def __init__(self, queue, function):
        threading.Thread.__init__(self)
        self.queue = queue
        self.function = function

    def run(self):
        while True:
            while True:
                m = self.queue.get()
                if self.queue.empty():
                    break
            self.function(m)


class CalibrationNode:
    def __init__(self, boards, service_check = True, synchronizer = message_filters.TimeSynchronizer):
        if service_check:
            # assume any non-default service names have been set.  Wait for the service to become ready
            for svcname in ["camera", "left_camera", "right_camera"]:
                remapped = rospy.remap_name(svcname)
                if remapped != svcname:
                    fullservicename = "%s/set_camera_info" % remapped
                    print "Waiting for service", fullservicename, "..."
                    try:
                        rospy.wait_for_service(fullservicename, 5)
                        print "OK"
                    except rospy.ROSException:
                        print "Service not found"
                        rospy.signal_shutdown('Quit')

        self._boards = boards
        lsub = message_filters.Subscriber('left', sensor_msgs.msg.Image)
        rsub = message_filters.Subscriber('right', sensor_msgs.msg.Image)
        ts = synchronizer([lsub, rsub], 4)
        ts.registerCallback(self.queue_stereo)

        msub = message_filters.Subscriber('image', sensor_msgs.msg.Image)
        msub.registerCallback(self.queue_monocular)
        
        self.set_camera_info_service = rospy.ServiceProxy("%s/set_camera_info" % rospy.remap_name("camera"), sensor_msgs.srv.SetCameraInfo)
        self.set_left_camera_info_service = rospy.ServiceProxy("%s/set_camera_info" % rospy.remap_name("left_camera"), sensor_msgs.srv.SetCameraInfo)
        self.set_right_camera_info_service = rospy.ServiceProxy("%s/set_camera_info" % rospy.remap_name("right_camera"), sensor_msgs.srv.SetCameraInfo)

        self.q_mono = Queue.Queue()
        self.q_stereo = Queue.Queue()

        self.c = None

        mth = ConsumerThread(self.q_mono, self.handle_monocular)
        mth.setDaemon(True)
        mth.start()

        sth = ConsumerThread(self.q_stereo, self.handle_stereo)
        sth.setDaemon(True)
        sth.start()
        
    def redraw_stereo(self, *args):
        pass
    def redraw_monocular(self, *args):
        pass

    def queue_monocular(self, msg):
        self.q_mono.put(msg)

    def queue_stereo(self, lmsg, rmsg):
        self.q_stereo.put((lmsg, rmsg))

    def handle_monocular(self, msg):
        if self.c == None:
            self.c = MonoCalibrator(self._boards)

        # This should just call the MonoCalibrator
        drawable = self.c.handle_msg(msg)
        self.displaywidth = drawable.scrib.cols
        self.redraw_monocular(drawable)

    def handle_stereo(self, msg):
        (lmsg, rmsg) = msg
        if self.c == None:
            self.c = StereoCalibrator(self._boards)
            
        drawable = self.c.handle_msg(msg)
        self.displaywidth = drawable.lscrib.cols + drawable.rscrib.cols
        self.redraw_stereo(drawable)
            
 
    def check_set_camera_info(self, response):
        if response.success:
            return True

        for i in range(10):
            print "!" * 80
        print
        print "Attempt to set camera info failed: " + response.status_message
        print
        for i in range(10):
            print "!" * 80
        print
        rospy.logerr('Unable to set camera info for calibration. Failure message: %s' % response.status_message)
        return False

    def do_upload(self):
        self.c.report()
        print self.c.ost()
        info = self.c.as_message()

        rv = True
        if self.c.is_mono:
            response = self.set_camera_info_service(info)
            rv = self.check_set_camera_info(response)
        else:
            response = self.set_left_camera_info_service(info[0])
            rv = rv and self.check_set_camera_info(response)
            response = self.set_right_camera_info_service(info[1])
            rv = rv and self.check_set_camera_info(response)
        return rv


class OpenCVCalibrationNode(CalibrationNode):
    """ Calibration node with an OpenCV Gui """

    def __init__(self, *args):

        CalibrationNode.__init__(self, *args)
        cv.NamedWindow("display")
        self.font = cv.InitFont(cv.CV_FONT_HERSHEY_SIMPLEX, 0.20, 1, thickness = 2)
        #self.button = cv.LoadImage("%s/button.jpg" % roslib.packages.get_pkg_dir(PKG))
        cv.SetMouseCallback("display", self.on_mouse)
        cv.CreateTrackbar("scale", "display", 0, 100, self.on_scale)

    def on_mouse(self, event, x, y, flags, param):
        if event == cv.CV_EVENT_LBUTTONDOWN and self.displaywidth < x:
            if self.c.goodenough:
                if 180 <= y < 280:
                    self.c.do_calibration()
            if self.c.calibrated:
                if 280 <= y < 380:
                    self.c.do_save()
                elif 380 <= y < 480:
                    # Only shut down if we set camera info correctly, #3993
                    if self.do_upload():
                        rospy.signal_shutdown('Quit')
                        
                        

    def waitkey(self):
        k = cv.WaitKey(6)
        if k in [27, ord('q')]:
            rospy.signal_shutdown('Quit')
        return k

    def on_scale(self, scalevalue):
        self.c.set_scale(scalevalue / 100.0)


    def button(self, dst, label, enable):
        cv.Set(dst, (255, 255, 255))
        size = cv.GetSize(dst)
        if enable:
            color = cv.RGB(155, 155, 80)
        else:
            color = cv.RGB(224, 224, 224)
        cv.Circle(dst, (size[0] / 2, size[1] / 2), min(size) / 2, color, -1)
        ((w, h), _) = cv.GetTextSize(label, self.font)
        cv.PutText(dst, label, ((size[0] - w) / 2, (size[1] + h) / 2), self.font, (255,255,255))

    def buttons(self, display):
        x = self.displaywidth
        self.button(cv.GetSubRect(display, (x,180,100,100)), "CALIBRATE", self.c.goodenough)
        self.button(cv.GetSubRect(display, (x,280,100,100)), "SAVE", self.c.calibrated)
        self.button(cv.GetSubRect(display, (x,380,100,100)), "COMMIT", self.c.calibrated)

    def y(self, i):
        """Set up right-size images"""
        return 30 + 40 * i
        
    def screendump(self, im):
        i = 0
        while os.access("/tmp/dump%d.png" % i, os.R_OK):
            i += 1
        cv.SaveImage("/tmp/dump%d.png" % i, im)

    def redraw_monocular(self, drawable):
        width, height = cv.GetSize(drawable.scrib)

        display = cv.CreateMat(max(480, height), width + 100, cv.CV_8UC3)
        cv.Zero(display)
        cv.Copy(drawable.scrib, cv.GetSubRect(display, (0,0,width,height)))
        cv.Set(cv.GetSubRect(display, (width,0,100,height)), (255, 255, 255))


        self.buttons(display)
        if not self.c.calibrated:
            if len(drawable.params) != 0:
                 for i, (label, lo, hi) in enumerate(drawable.params):
                    (w,_),_ = cv.GetTextSize(label, self.font)
                    cv.PutText(display, label, (width + (100 - w) / 2, self.y(i)), self.font, (0,0,0))
                    cv.Line(display,
                            (int(width + lo * 100), self.y(i) + 20),
                            (int(width + hi * 100), self.y(i) + 20),
                            (0,0,0),
                            4)

        else:
            cv.PutText(display, "lin.", (width, self.y(0)), self.font, (0,0,0))
            linerror = drawable.linear_error
            if linerror < 0:
                msg = "?"
            else:
                msg = "%.2f" % linerror
                print "linear", linerror
            cv.PutText(display, msg, (width, self.y(1)), self.font, (0,0,0))

        self.show(display)

    def redraw_stereo(self, drawable):
        width, height = cv.GetSize(drawable.lscrib)

        display = cv.CreateMat(max(480, height), 2 * width + 100, cv.CV_8UC3)
        cv.Zero(display)
        cv.Copy(drawable.lscrib, cv.GetSubRect(display, (0,0,width,height)))
        cv.Copy(drawable.rscrib, cv.GetSubRect(display, (width,0,width,height)))
        cv.Set(cv.GetSubRect(display, (2 * width,0,100,height)), (255, 255, 255))

        self.buttons(display)

        if not self.c.calibrated:
            if len(drawable.params) != 0:
                for i, (label, lo, hi) in enumerate(drawable.params):
                    (label_width,_),_ = cv.GetTextSize(label, self.font)
                    cv.PutText(display, label, (2 * width + (100 - label_width) / 2, self.y(i)), self.font, (0,0,0))
                    cv.Line(display,
                            (int(2 * width + lo * 100), self.y(i) + 20),
                            (int(2 * width + hi * 100), self.y(i) + 20),
                            (0,0,0),
                            4)

        else:
            cv.PutText(display, "epi.", (2 * width, self.y(0)), self.font, (0,0,0))
            if drawable.epierror == -1:
                msg = "?"
            else:
                msg = "%.2f" % drawable.epierror
            cv.PutText(display, msg, (2 * width, self.y(1)), self.font, (0,0,0))
            if drawable.epierror > -1:
                cv.PutText(display, "dim", (2 * width, self.y(2)), self.font, (0,0,0))
                cv.PutText(display, "%.3f" % drawable.dim, (2 * width, self.y(3)), self.font, (0,0,0))

        self.show(display)

    def show(self, im):
        cv.ShowImage("display", im)
        if self.waitkey() == ord('s'):
            self.screendump(im)


def main():
    from optparse import OptionParser
    parser = OptionParser("%prog --size SIZE1 --square SQUARE1 [ --size SIZE2 --square SQUARE2 ]")
    parser.add_option("-s", "--size", default=[], action="append", dest="size",
                      help="specify chessboard size as NxM [default: 8x6]")
    parser.add_option("-q", "--square", default=[], action="append", dest="square",
                      help="specify chessboard square size in meters [default: 0.108]")
    parser.add_option("", "--no-service-check", dest="service_check", action="store_false", default=True, help="disable check for set_camera_info services at startup")
    parser.add_option("", "--approximate", dest="approximate", type="float", default=0.0, help="Use approximate time synchronizer for incoming stereo images")
    options, args = parser.parse_args()

    rospy.init_node('cameracalibrator') 
    
    if len(options.size) != len(options.square):
        parser.error("Number of size and square inputs must be the same!")
    
    if not options.square:
        options.square.append("0.108")
        options.size.append("8x6")

    boards = []
    for (sz, sq) in zip(options.size, options.square):
        info = ChessboardInfo()
        info.dim = float(sq)
        size = tuple([int(c) for c in sz.split('x')])
        info.n_cols = size[0]
        info.n_rows = size[1]

        boards.append(info)

    if not boards:
        parser.error("Must supply at least one chessboard")

    if options.approximate == 0.0:
        sync = message_filters.TimeSynchronizer
    else:
        sync = functools.partial(ApproximateSynchronizer, options.approximate)
    node = OpenCVCalibrationNode(boards, options.service_check, sync)
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except Exception, e:
        import traceback
        traceback.print_exc()
