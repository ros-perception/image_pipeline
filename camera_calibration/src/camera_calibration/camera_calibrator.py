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

import cv2
import message_filters
import numpy
import os
import rospy
import sensor_msgs.msg
import sensor_msgs.srv
import threading
import time
from camera_calibration.calibrator import MonoCalibrator, StereoCalibrator, ChessboardInfo, Patterns
from collections import deque
from message_filters import ApproximateTimeSynchronizer
from std_msgs.msg import String
from std_srvs.srv import Empty


class DisplayThread(threading.Thread):
    """
    Thread that displays the current images
    It is its own thread so that all display can be done
    in one thread to overcome imshow limitations and
    https://github.com/ros-perception/image_pipeline/issues/85
    """
    def __init__(self, queue, opencv_calibration_node):
        threading.Thread.__init__(self)
        self.queue = queue
        self.opencv_calibration_node = opencv_calibration_node

    def run(self):
        cv2.namedWindow("display", cv2.WINDOW_NORMAL)
        cv2.setMouseCallback("display", self.opencv_calibration_node.on_mouse)
        cv2.createTrackbar("scale", "display", 0, 100, self.opencv_calibration_node.on_scale)
        while True:
            # wait for an image (could happen at the very beginning when the queue is still empty)
            while len(self.queue) == 0:
                time.sleep(0.1)
            im = self.queue[0]
            cv2.imshow("display", im)
            k = cv2.waitKey(6) & 0xFF
            if k in [27, ord('q')]:
                rospy.signal_shutdown('Quit')
            elif k == ord('s'):
                self.opencv_calibration_node.screendump(im)

class ConsumerThread(threading.Thread):
    def __init__(self, queue, function):
        threading.Thread.__init__(self)
        self.queue = queue
        self.function = function

    def run(self):
        while True:
            # wait for an image (could happen at the very beginning when the queue is still empty)
            while len(self.queue) == 0:
                time.sleep(0.1)
            self.function(self.queue[0])


class CalibrationNode:
    def __init__(self, boards, service_check = True, synchronizer = message_filters.TimeSynchronizer, flags = 0,
                 pattern=Patterns.Chessboard, camera_name='', checkerboard_flags = 0, max_chessboard_speed = -1):
        if service_check:
            # assume any non-default service names have been set.  Wait for the service to become ready
            for svcname in ["camera", "left_camera", "right_camera"]:
                remapped = rospy.remap_name(svcname)
                if remapped != svcname:
                    fullservicename = "%s/set_camera_info" % remapped
                    print("Waiting for service", fullservicename, "...")
                    try:
                        rospy.wait_for_service(fullservicename, 5)
                        print("OK")
                    except rospy.ROSException:
                        print("Service not found")
                        rospy.signal_shutdown('Quit')

        self._boards = boards
        self._calib_flags = flags
        self._checkerboard_flags = checkerboard_flags
        self._pattern = pattern
        self._camera_name = camera_name
        self._max_chessboard_speed = max_chessboard_speed
        lsub = message_filters.Subscriber('left', sensor_msgs.msg.Image)
        rsub = message_filters.Subscriber('right', sensor_msgs.msg.Image)
        ts = synchronizer([lsub, rsub], 4)
        ts.registerCallback(self.queue_stereo)

        msub = message_filters.Subscriber('image', sensor_msgs.msg.Image)
        msub.registerCallback(self.queue_monocular)

        self.set_camera_info_service = rospy.ServiceProxy("%s/set_camera_info" % rospy.remap_name("camera"),
                                                          sensor_msgs.srv.SetCameraInfo)
        self.set_left_camera_info_service = rospy.ServiceProxy("%s/set_camera_info" % rospy.remap_name("left_camera"),
                                                               sensor_msgs.srv.SetCameraInfo)
        self.set_right_camera_info_service = rospy.ServiceProxy("%s/set_camera_info" % rospy.remap_name("right_camera"),
                                                                sensor_msgs.srv.SetCameraInfo)

        self.q_mono = deque([], 1)
        self.q_stereo = deque([], 1)

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
        self.q_mono.append(msg)

    def queue_stereo(self, lmsg, rmsg):
        self.q_stereo.append((lmsg, rmsg))

    def handle_monocular(self, msg):
        if self.c == None:
            if self._camera_name:
                self.c = MonoCalibrator(self._boards, self._calib_flags, self._pattern, name=self._camera_name,
                                        checkerboard_flags=self._checkerboard_flags,
                                        max_chessboard_speed = self._max_chessboard_speed)
            else:
                self.c = MonoCalibrator(self._boards, self._calib_flags, self._pattern,
                                        checkerboard_flags=self.checkerboard_flags,
                                        max_chessboard_speed = self._max_chessboard_speed)

        # This should just call the MonoCalibrator
        drawable = self.c.handle_msg(msg)
        self.displaywidth = drawable.scrib.shape[1]
        self.redraw_monocular(drawable)

    def handle_stereo(self, msg):
        if self.c == None:
            if self._camera_name:
                self.c = StereoCalibrator(self._boards, self._calib_flags, self._pattern, name=self._camera_name,
                                          checkerboard_flags=self._checkerboard_flags,
                                          max_chessboard_speed = self._max_chessboard_speed)
            else:
                self.c = StereoCalibrator(self._boards, self._calib_flags, self._pattern,
                                          checkerboard_flags=self._checkerboard_flags,
                                          max_chessboard_speed = self._max_chessboard_speed)

        drawable = self.c.handle_msg(msg)
        self.displaywidth = drawable.lscrib.shape[1] + drawable.rscrib.shape[1]
        self.redraw_stereo(drawable)


    def check_set_camera_info(self, response):
        if response.success:
            return True

        for i in range(10):
            print("!" * 80)
        print()
        print("Attempt to set camera info failed: " + response.status_message)
        print()
        for i in range(10):
            print("!" * 80)
        print()
        rospy.logerr('Unable to set camera info for calibration. Failure message: %s' % response.status_message)
        return False

    def do_upload(self):
        self.c.report()
        print(self.c.ost())
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
    FONT_FACE = cv2.FONT_HERSHEY_SIMPLEX
    FONT_SCALE = 0.6
    FONT_THICKNESS = 2

    def __init__(self, *args, **kwargs):

        CalibrationNode.__init__(self, *args, **kwargs)

        self.queue_display = deque([], 1)
        self.display_thread = DisplayThread(self.queue_display, self)
        self.display_thread.setDaemon(True)
        self.display_thread.start()

    @classmethod
    def putText(cls, img, text, org, color = (0,0,0)):
        cv2.putText(img, text, org, cls.FONT_FACE, cls.FONT_SCALE, color, thickness = cls.FONT_THICKNESS)

    @classmethod
    def getTextSize(cls, text):
        return cv2.getTextSize(text, cls.FONT_FACE, cls.FONT_SCALE, cls.FONT_THICKNESS)[0]

    def on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and self.displaywidth < x:
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

    def on_scale(self, scalevalue):
        if self.c.calibrated:
            self.c.set_alpha(scalevalue / 100.0)

    def button(self, dst, label, enable):
        dst.fill(255)
        size = (dst.shape[1], dst.shape[0])
        if enable:
            color = (155, 155, 80)
        else:
            color = (224, 224, 224)
        cv2.circle(dst, (size[0] // 2, size[1] // 2), min(size) // 2, color, -1)
        (w, h) = self.getTextSize(label)
        self.putText(dst, label, ((size[0] - w) // 2, (size[1] + h) // 2), (255,255,255))

    def buttons(self, display):
        x = self.displaywidth
        self.button(display[180:280,x:x+100], "CALIBRATE", self.c.goodenough)
        self.button(display[280:380,x:x+100], "SAVE", self.c.calibrated)
        self.button(display[380:480,x:x+100], "COMMIT", self.c.calibrated)

    def y(self, i):
        """Set up right-size images"""
        return 30 + 40 * i

    def screendump(self, im):
        i = 0
        while os.access("/tmp/dump%d.png" % i, os.R_OK):
            i += 1
        cv2.imwrite("/tmp/dump%d.png" % i, im)

    def redraw_monocular(self, drawable):
        height = drawable.scrib.shape[0]
        width = drawable.scrib.shape[1]

        display = numpy.zeros((max(480, height), width + 100, 3), dtype=numpy.uint8)
        display[0:height, 0:width,:] = drawable.scrib
        display[0:height, width:width+100,:].fill(255)


        self.buttons(display)
        if not self.c.calibrated:
            if drawable.params:
                 for i, (label, lo, hi, progress) in enumerate(drawable.params):
                    (w,_) = self.getTextSize(label)
                    self.putText(display, label, (width + (100 - w) // 2, self.y(i)))
                    color = (0,255,0)
                    if progress < 1.0:
                        color = (0, int(progress*255.), 255)
                    cv2.line(display,
                            (int(width + lo * 100), self.y(i) + 20),
                            (int(width + hi * 100), self.y(i) + 20),
                            color, 4)

        else:
            self.putText(display, "lin.", (width, self.y(0)))
            linerror = drawable.linear_error
            if linerror < 0:
                msg = "?"
            else:
                msg = "%.2f" % linerror
                #print "linear", linerror
            self.putText(display, msg, (width, self.y(1)))

        self.queue_display.append(display)

    def redraw_stereo(self, drawable):
        height = drawable.lscrib.shape[0]
        width = drawable.lscrib.shape[1]

        display = numpy.zeros((max(480, height), 2 * width + 100, 3), dtype=numpy.uint8)
        display[0:height, 0:width,:] = drawable.lscrib
        display[0:height, width:2*width,:] = drawable.rscrib
        display[0:height, 2*width:2*width+100,:].fill(255)

        self.buttons(display)

        if not self.c.calibrated:
            if drawable.params:
                for i, (label, lo, hi, progress) in enumerate(drawable.params):
                    (w,_) = self.getTextSize(label)
                    self.putText(display, label, (2 * width + (100 - w) // 2, self.y(i)))
                    color = (0,255,0)
                    if progress < 1.0:
                        color = (0, int(progress*255.), 255)
                    cv2.line(display,
                            (int(2 * width + lo * 100), self.y(i) + 20),
                            (int(2 * width + hi * 100), self.y(i) + 20),
                            color, 4)

        else:
            self.putText(display, "epi.", (2 * width, self.y(0)))
            if drawable.epierror == -1:
                msg = "?"
            else:
                msg = "%.2f" % drawable.epierror
            self.putText(display, msg, (2 * width, self.y(1)))
            # TODO dim is never set anywhere. Supposed to be observed chessboard size?
            if drawable.dim != -1:
                self.putText(display, "dim", (2 * width, self.y(2)))
                self.putText(display, "%.3f" % drawable.dim, (2 * width, self.y(3)))

        self.queue_display.append(display)
