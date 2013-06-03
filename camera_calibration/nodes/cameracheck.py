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
import StringIO

import cv

import message_filters
import image_geometry
import numpy

from camera_calibration.calibrator import cvmat_iterator, MonoCalibrator, StereoCalibrator, ChessboardInfo

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
        while not rospy.is_shutdown():
            while not rospy.is_shutdown():
                m = self.queue.get()
                if self.queue.empty():
                    break
            self.function(m)

class CameraCheckerNode:

    def __init__(self, chess_size, dim):
        self.board = ChessboardInfo()
        self.board.n_cols = chess_size[0]
        self.board.n_rows = chess_size[1]
        self.board.dim = dim

        image_topic = rospy.resolve_name("monocular") + "/image_rect"
        camera_topic = rospy.resolve_name("monocular") + "/camera_info"

        tosync_mono = [
            (image_topic, sensor_msgs.msg.Image),
            (camera_topic, sensor_msgs.msg.CameraInfo),
        ]

        tsm = message_filters.TimeSynchronizer([message_filters.Subscriber(topic, type) for (topic, type) in tosync_mono], 10)
        tsm.registerCallback(self.queue_monocular)

        left_topic = rospy.resolve_name("stereo") + "/left/image_rect"
        left_camera_topic = rospy.resolve_name("stereo") + "/left/camera_info"
        right_topic = rospy.resolve_name("stereo") + "/right/image_rect"
        right_camera_topic = rospy.resolve_name("stereo") + "/right/camera_info"

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

        self.mc = MonoCalibrator([self.board])

    def queue_monocular(self, msg, cmsg):
        self.q_mono.put((msg, cmsg))

    def queue_stereo(self, lmsg, lcmsg, rmsg, rcmsg):
        self.q_stereo.put((lmsg, lcmsg, rmsg, rcmsg))

    def mkgray(self, msg):
        return self.mc.mkgray(msg)

    def image_corners(self, im):
        (ok, corners, b) = self.mc.get_corners(im)
        if ok:
            return list(cvmat_iterator(cv.Reshape(self.mc.mk_image_points([(corners, b)]), 2)))
        else:
            return None

    def handle_monocular(self, msg):

        def pt2line(x0, y0, x1, y1, x2, y2):
            """ point is (x0, y0), line is (x1, y1, x2, y2) """
            return abs((x2 - x1) * (y1 - y0) - (x1 - x0) * (y2 - y1)) / math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        (image, camera) = msg
        gray = self.mkgray(image)
        C = self.image_corners(gray)
        if C:
            cc = self.board.n_cols
            cr = self.board.n_rows
            errors = []
            for r in range(cr):
                (x1, y1) = C[(cc * r) + 0]
                (x2, y2) = C[(cc * r) + cc - 1]
                for i in range(1, cc - 1):
                    (x0, y0) = C[(cc * r) + i]
                    errors.append(pt2line(x0, y0, x1, y1, x2, y2))
            linearity_rms = math.sqrt(sum([e**2 for e in errors]) / len(errors))

            # Add in reprojection check
            A = cv.CreateMat(3,3,0)
            image_points = cv.fromarray(numpy.array(C))
            object_points = cv.fromarray(numpy.zeros([cc*cr, 3]))
            for i in range(cr):
                for j in range(cc):
                    object_points[i*cc + j, 0] = j * self.board.dim
                    object_points[i*cc + j, 1] = i * self.board.dim
                    object_points[i*cc + j, 2] = 0.0
            dist_coeffs = cv.fromarray(numpy.array([ [0.0, 0.0, 0.0, 0.0] ]))
            camera_matrix = numpy.array( [ [ camera.P[0], camera.P[1], camera.P[2]  ],
                                           [ camera.P[4], camera.P[5], camera.P[6]  ],
                                           [ camera.P[8], camera.P[9], camera.P[10] ] ] )
            rot = cv.CreateMat(3, 1, cv.CV_32FC1)
            trans = cv.CreateMat(3, 1, cv.CV_32FC1)
            cv.FindExtrinsicCameraParams2(object_points, image_points, cv.fromarray(camera_matrix),
                                          dist_coeffs, rot, trans)
            # Convert rotation into a 3x3 Rotation Matrix
            rot3x3 = cv.CreateMat(3, 3, cv.CV_32FC1)
            cv.Rodrigues2(rot, rot3x3)
            # Reproject model points into image
            object_points_world = numpy.asmatrix(rot3x3) * (numpy.asmatrix(object_points).T) + numpy.asmatrix(trans)
            reprojected_h = camera_matrix * object_points_world
            reprojected   = (reprojected_h[0:2, :] / reprojected_h[2, :])
            reprojection_errors = image_points - reprojected.T
            reprojection_rms = numpy.sqrt(numpy.sum(numpy.array(reprojection_errors) ** 2) / numpy.product(reprojection_errors.shape))

            # Print the results
            print "Linearity RMS Error: %.3f Pixels      Reprojection RMS Error: %.3f Pixels" % (linearity_rms, reprojection_rms)
        else:
            print 'no chessboard'

    def handle_stereo(self, msg):

        (lmsg, lcmsg, rmsg, rcmsg) = msg
        lgray = self.mkgray(lmsg)
        rgray = self.mkgray(rmsg)

        sc = StereoCalibrator([self.board])

        L = self.image_corners(lgray)
        R = self.image_corners(rgray)
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
            cc = self.board.n_cols
            cr = self.board.n_rows
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
