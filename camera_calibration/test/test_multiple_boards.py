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

import rclpy
import ament_index_python
import requests
import unittest
import tarfile
import os

from camera_calibration.calibrator import StereoCalibrator, ChessboardInfo, image_from_archive

# Large board used for PR2 calibration
board = ChessboardInfo()
board.n_cols = 7
board.n_rows = 6
board.dim = 0.108

class TestMultipleBoards(unittest.TestCase):
    def test_multiple_boards(self):
        small_board = ChessboardInfo()
        small_board.n_cols = 5
        small_board.n_rows = 4
        small_board.dim = 0.025

        stereo_cal = StereoCalibrator([board, small_board])
        if not os.path.isfile('/tmp/multi_board_calibration.tar.gz'):
            url = 'http://download.ros.org/data/camera_calibration/multi_board_calibration.tar.gz'
            r = requests.get(url, allow_redirects=True)
            with open('/tmp/multi_board_calibration.tar.gz', 'wb') as mcf:
                mcf.write(r.content)

        my_archive_name = '/tmp/multi_board_calibration.tar.gz'
        stereo_cal.do_tarfile_calibration(my_archive_name)

        stereo_cal.report()
        stereo_cal.ost()
        
        # Check error for big image
        archive = tarfile.open(my_archive_name)
        l1_big = image_from_archive(archive, "left-0000.png")
        r1_big = image_from_archive(archive, "right-0000.png")
        epi_big = stereo_cal.epipolar_error_from_images(l1_big, r1_big)
        self.assertTrue(epi_big < 1.0, "Epipolar error for large checkerboard > 1.0. Error: %.2f" % epi_big)

        # Small checkerboard has larger error threshold for now
        l1_sm = image_from_archive(archive, "left-0012-sm.png")
        r1_sm = image_from_archive(archive, "right-0012-sm.png")
        epi_sm = stereo_cal.epipolar_error_from_images(l1_sm, r1_sm)
        self.assertTrue(epi_sm < 2.0, "Epipolar error for small checkerboard > 2.0. Error: %.2f" % epi_sm)



if __name__ == '__main__':
    unittest.main(verbosity=2)