#!/usr/bin/env python
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

import roslib
PKG = 'camera_calibration'
roslib.load_manifest(PKG)
import rostest
import rospy
import cv
import cv2
import unittest
import tarfile
import copy
import os, sys

from camera_calibration.calibrator import cvmat_iterator, MonoCalibrator, StereoCalibrator, CalibrationException, ChessboardInfo, image_from_archive

board = ChessboardInfo()
board.n_cols = 8
board.n_rows = 6
board.dim = 0.108

class TestDirected(unittest.TestCase):
    def setUp(self):
        self.tar = tarfile.open('camera_calibration.tar.gz', 'r')
        self.limages = [image_from_archive(self.tar, "wide/left%04d.pgm" % i) for i in range(3, 15)]
        self.rimages = [image_from_archive(self.tar, "wide/right%04d.pgm" % i) for i in range(3, 15)]
        self.l = {}
        self.r = {}
        self.sizes = [(320,240), (640,480), (800,600), (1024,768)]
        for dim in self.sizes:
            self.l[dim] = []
            self.r[dim] = []
            for li,ri in zip(self.limages, self.rimages):
                rli = cv.CreateMat(dim[1], dim[0], cv.CV_8UC3)
                rri = cv.CreateMat(dim[1], dim[0], cv.CV_8UC3)
                cv.Resize(li, rli)
                cv.Resize(ri, rri)
                self.l[dim].append(rli)
                self.r[dim].append(rri)
                
    def assert_good_mono(self, c, dim):
        #c.report()
        self.assert_(len(c.ost()) > 0)
        img = self.l[dim][0]
        lin_err = c.linear_error_from_image(img)
        self.assert_(0.0 < lin_err)
        self.assert_(lin_err < 1.0)

        flat = c.remap(img)
        self.assertEqual(cv.GetSize(img), cv.GetSize(flat))

    def test_monocular(self):
        # Run the calibrator, produce a calibration, check it
        mc = MonoCalibrator([ board ], flags=cv2.CALIB_FIX_K3)
        for dim in self.sizes:
            mc.cal(self.l[dim])
            self.assert_good_mono(mc, dim)

            # Make another calibration, import previous calibration as a message,
            # and assert that the new one is good.

            mc2 = MonoCalibrator([board])
            mc2.from_message(mc.as_message())
            self.assert_good_mono(mc2, dim)

    def test_stereo(self):
        for dim in self.sizes:
            print "Dim =", dim
            sc = StereoCalibrator([board], cv2.CALIB_FIX_K3)
            sc.cal(self.l[dim], self.r[dim])

            sc.report()
            #print sc.ost()

            # NOTE: epipolar error currently increases with resolution.
            # At highest res expect error ~0.75
            epierror = sc.epipolar_error_from_images(self.l[dim][0], self.r[dim][0])
            print "Epipolar error =", epierror
            self.assert_(epierror < 0.8)

            self.assertAlmostEqual(sc.chessboard_size_from_images(self.l[dim][0], self.r[dim][0]), .108, 2)

            #print sc.as_message()

            img = self.l[dim][0]
            flat = sc.l.remap(img)
            self.assertEqual(cv.GetSize(img), cv.GetSize(flat))
            flat = sc.r.remap(img)
            self.assertEqual(cv.GetSize(img), cv.GetSize(flat))

            sc2 = StereoCalibrator([board])
            sc2.from_message(sc.as_message())
            # sc2.set_alpha(1.0)
            #sc2.report()
            self.assert_(len(sc2.ost()) > 0)

    def test_nochecker(self):
        # Run with same images, but looking for an incorrect chessboard size (8, 7).
        # Should raise an exception because of lack of input points.
        new_board = copy.deepcopy(board)
        new_board.n_cols = 8
        new_board.n_rows = 7

        sc = StereoCalibrator([new_board])
        self.assertRaises(CalibrationException, lambda: sc.cal(self.limages, self.rimages))
        mc = MonoCalibrator([new_board])
        self.assertRaises(CalibrationException, lambda: mc.cal(self.limages))


if __name__ == '__main__':
    if 1:
        rostest.unitrun('camera_calibration', 'directed', TestDirected)
    else:
        suite = unittest.TestSuite()
        suite.addTest(TestDirected('test_stereo'))
        unittest.TextTestRunner(verbosity=2).run(suite)
