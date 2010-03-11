#!/usr/bin/python

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
import StringIO

import cv

import message_filters
import image_geometry

from camera_calibration.calibrator import cvmat_iterator, MonoCalibrator, StereoCalibrator

def mean(seq):
    return sum(seq) / len(seq)

def lmin(seq1, seq2):
    """ Pairwise minimum of two sequences """
    return [min(a, b) for (a, b) in zip(seq1, seq2)]

def lmax(seq1, seq2):
    """ Pairwise maximum of two sequences """
    return [max(a, b) for (a, b) in zip(seq1, seq2)]

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

class CameraCheckerNode:

    def __init__(self, chess_size, dim):
        self.chess_size = chess_size
        self.dim = dim

        image_topic = rospy.resolve_name("monocular") + "/" + rospy.resolve_name("image")
        camera_topic = rospy.resolve_name("monocular") + "/" + "camera_info"

        tosync_mono = [
            (image_topic, sensor_msgs.msg.Image),
            (camera_topic, sensor_msgs.msg.CameraInfo),
        ]

        tsm = message_filters.TimeSynchronizer([message_filters.Subscriber(topic, type) for (topic, type) in tosync_mono], 10)
        tsm.registerCallback(self.queue_monocular)

        left_topic = rospy.resolve_name("stereo") + "/left/" + rospy.resolve_name("image")
        left_camera_topic = rospy.resolve_name("stereo") + "/left/" + "camera_info"
        right_topic = rospy.resolve_name("stereo") + "/right/" + rospy.resolve_name("image")
        right_camera_topic = rospy.resolve_name("stereo") + "/right/" + "camera_info"
                 
        tosync_stereo = [
            (left_topic, sensor_msgs.msg.Image),
            (left_camera_topic, sensor_msgs.msg.CameraInfo),
            (right_topic, sensor_msgs.msg.Image),
            (right_camera_topic, sensor_msgs.msg.CameraInfo)
        ]

        tss = message_filters.TimeSynchronizer([message_filters.Subscriber(topic, type) for (topic, type) in tosync_stereo], 10)
        tss.registerCallback(self.queue_stereo)

        self.br = cv_bridge.CvBridge()

        self.q_mono = Queue.Queue()
        self.q_stereo = Queue.Queue()

        mth = ConsumerThread(self.q_mono, self.handle_monocular)
        mth.setDaemon(True)
        mth.start()

        sth = ConsumerThread(self.q_stereo, self.handle_stereo)
        sth.setDaemon(True)
        sth.start()

        self.mc = MonoCalibrator(self.chess_size, self.dim)

    def queue_monocular(self, msg, cmsg):
        self.q_mono.put((msg, cmsg))

    def queue_stereo(self, lmsg, lcmsg, rmsg, rcmsg):
        self.q_stereo.put((lmsg, lcmsg, rmsg, rcmsg))

    def mkgray(self, msg):
        return self.br.imgmsg_to_cv(msg, "bgr8")

    def image_corners(self, im):
        (ok, corners) = self.mc.get_corners(im)
        if ok:
            return list(cvmat_iterator(cv.Reshape(self.mc.mk_image_points([corners]), 2)))
        else:
            return None

    def handle_monocular(self, msg):

        def pt2line(x0, y0, x1, y1, x2, y2):
            """ point is (x0, y0), line is (x1, y1, x2, y2) """
            return abs((x2 - x1) * (y1 - y0) - (x1 - x0) * (y2 - y1)) / math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        (image, camera) = msg
        rgb = self.mkgray(image)
        C = self.image_corners(rgb)
        if C:
            cc = self.mc.chessboard_n_cols
            cr = self.mc.chessboard_n_rows
            errors = []
            for r in range(cr):
                (x1, y1) = C[(cc * r) + 0]
                (x2, y2) = C[(cc * r) + cc - 1]
                for i in range(1, cc - 1):
                    (x0, y0) = C[(cc * r) + i]
                    errors.append(pt2line(x0, y0, x1, y1, x2, y2))
            print "error", math.sqrt(sum([e**2 for e in errors]) / len(errors)), "pixels"

    def handle_stereo(self, msg):

        (lmsg, lcmsg, rmsg, rcmsg) = msg
        lrgb = self.mkgray(lmsg)
        rrgb = self.mkgray(rmsg)

        sc = StereoCalibrator(self.chess_size, self.dim)

        L = self.image_corners(lrgb)
        R = self.image_corners(rrgb)
        scm = image_geometry.StereoCameraModel()
        scm.fromCameraInfo(lcmsg, rcmsg)
        if L and R:
            d = [(y0 - y1) for ((_, y0), (_, y1)) in zip(L, R)]
            epipolar = math.sqrt(sum([i**2 for i in d]) / len(d))

            disparities = [(x0 - x1) for ((x0, y0), (x1, y1)) in zip(L, R)]
            pt3d = [scm.projectPixelTo3d((x, y), d) for ((x, y), d) in zip(L, disparities)]
            def l2(p0, p1):
                return math.sqrt(sum([(c0 - c1) ** 2 for (c0, c1) in zip(p0, p1)]))

            # Compute the length from each horizontal and vertical lines, and return the mean
            cc = self.mc.chessboard_n_cols
            cr = self.mc.chessboard_n_rows
            lengths = (
                [l2(pt3d[cc * r + 0], pt3d[cc * r + (cc - 1)]) / (cc - 1) for r in range(cr)] +
                [l2(pt3d[c + 0], pt3d[c + (cc * (cr - 1))]) / (cr - 1) for c in range(cc)])
            dimension = sum(lengths) / len(lengths)

            print "epipolar error: %f pixels   dimension: %f m" % (epipolar, dimension)
        else:
            print "no chessboard"

def main():
    from optparse import OptionParser
    rospy.init_node('cameracheck')
    parser = OptionParser()
    parser.add_option("-s", "--size", default="8x6", help="specify chessboard size as nxm [default: %default]")
    parser.add_option("-q", "--square", default=".108", help="specify chessboard square size in meters [default: %default]")
    options, args = parser.parse_args()
    size = tuple([int(c) for c in options.size.split('x')])
    dim = float(options.square)
    CameraCheckerNode(size, dim)
    rospy.spin()

if __name__ == "__main__":
    main()
