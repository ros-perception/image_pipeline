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

import cv2
import functools
import message_filters
import rospy
from camera_calibration.camera_calibrator import OpenCVCalibrationNode
from camera_calibration.calibrator import ChessboardInfo, Patterns
from message_filters import ApproximateTimeSynchronizer

def optionsValidCharuco(options, parser):
    """
    Validates the provided options when the pattern type is 'charuco'
    """
    if options.pattern != 'charuco': return False

    n_boards = len(options.size)
    if (n_boards != len(options.square) or n_boards != len(options.charuco_marker_size) or n_boards !=
            len(options.aruco_dict)):
        parser.error("When using ChArUco boards, --size, --square, --charuco_marker_size, and --aruco_dict " +
        "must be specified for each board")
        return False

    # TODO: check for fisheye and stereo (not implemented with ChArUco)
    return True


def main():
    from optparse import OptionParser, OptionGroup
    parser = OptionParser("%prog --size SIZE1 --square SQUARE1 [ --size SIZE2 --square SQUARE2 ]",
                          description=None)
    parser.add_option("-c", "--camera_name",
                     type="string", default='narrow_stereo',
                     help="name of the camera to appear in the calibration file")
    group = OptionGroup(parser, "Chessboard Options",
                        "You must specify one or more chessboards as pairs of --size and --square options.")
    group.add_option("-p", "--pattern",
                     type="string", default="chessboard",
                     help="calibration pattern to detect - 'chessboard', 'circles', 'acircles', 'charuco'\n" +
                     "  if 'charuco' is used, a --charuco_marker_size and --aruco_dict argument must be supplied\n" +
                     "  with each --size and --square argument")
    group.add_option("-s", "--size",
                     action="append", default=[],
                     help="chessboard size as NxM, counting interior corners (e.g. a standard chessboard is 7x7)")
    group.add_option("-q", "--square",
                     action="append", default=[],
                     help="chessboard square size in meters")
    group.add_option("-m", "--charuco_marker_size",
                     action="append", default=[],
                     help="ArUco marker size (meters); only valid with `-p charuco`")
    group.add_option("-d", "--aruco_dict",
                     action="append", default=[],
                     help="ArUco marker dictionary; only valid with `-p charuco`; one of 'aruco_orig', '4x4_250', " +
                     "'5x5_250', '6x6_250', '7x7_250'")
    parser.add_option_group(group)
    group = OptionGroup(parser, "ROS Communication Options")
    group.add_option("--approximate",
                     type="float", default=0.0,
                     help="allow specified slop (in seconds) when pairing images from unsynchronized stereo cameras")
    group.add_option("--no-service-check",
                     action="store_false", dest="service_check", default=True,
                     help="disable check for set_camera_info services at startup")
    group.add_option("--queue-size",
                     type="int", default=1,
                     help="image queue size (default %default, set to 0 for unlimited)")
    parser.add_option_group(group)
    group = OptionGroup(parser, "Calibration Optimizer Options")
    group.add_option("--fix-principal-point",
                     action="store_true", default=False,
                     help="for pinhole, fix the principal point at the image center")
    group.add_option("--fix-aspect-ratio",
                     action="store_true", default=False,
                     help="for pinhole, enforce focal lengths (fx, fy) are equal")
    group.add_option("--zero-tangent-dist",
                     action="store_true", default=False,
                     help="for pinhole, set tangential distortion coefficients (p1, p2) to zero")
    group.add_option("-k", "--k-coefficients",
                     type="int", default=2, metavar="NUM_COEFFS",
                     help="for pinhole, number of radial distortion coefficients to use (up to 6, default %default)")

    group.add_option("--fisheye-recompute-extrinsicsts",
                     action="store_true", default=False,
                     help="for fisheye, extrinsic will be recomputed after each iteration of intrinsic optimization")
    group.add_option("--fisheye-fix-skew",
                     action="store_true", default=False,
                     help="for fisheye, skew coefficient (alpha) is set to zero and stay zero")
    group.add_option("--fisheye-fix-principal-point",
                     action="store_true", default=False,
                     help="for fisheye,fix the principal point at the image center")
    group.add_option("--fisheye-k-coefficients",
                     type="int", default=4, metavar="NUM_COEFFS",
                     help="for fisheye, number of radial distortion coefficients to use fixing to zero the rest (up to 4, default %default)")

    group.add_option("--fisheye-check-conditions",
                     action="store_true", default=False,
                     help="for fisheye, the functions will check validity of condition number")

    group.add_option("--disable_calib_cb_fast_check", action='store_true', default=False,
                     help="uses the CALIB_CB_FAST_CHECK flag for findChessboardCorners")
    group.add_option("--max-chessboard-speed", type="float", default=-1.0,
                     help="Do not use samples where the calibration pattern is moving faster \
                     than this speed in px/frame. Set to eg. 0.5 for rolling shutter cameras.")

    parser.add_option_group(group)

    options, args = parser.parse_args()

    if (len(options.size) != len(options.square)):
        parser.error("Number of size and square inputs must be the same!")

    if not options.square:
        options.square.append("0.108")
        options.size.append("8x6")

    boards = []
    if options.pattern == "charuco" and optionsValidCharuco(options, parser):
        for (sz, sq, ms, ad) in zip(options.size, options.square, options.charuco_marker_size, options.aruco_dict):
            size = tuple([int(c) for c in sz.split('x')])
            boards.append(ChessboardInfo('charuco', size[0], size[1], float(sq), float(ms), ad))
    else:
        for (sz, sq) in zip(options.size, options.square):
            size = tuple([int(c) for c in sz.split('x')])
            boards.append(ChessboardInfo(options.pattern, size[0], size[1], float(sq)))

    if options.approximate == 0.0:
        sync = message_filters.TimeSynchronizer
    else:
        sync = functools.partial(ApproximateTimeSynchronizer, slop=options.approximate)

    # Pinhole opencv calibration options parsing
    num_ks = options.k_coefficients

    calib_flags = 0
    if options.fix_principal_point:
        calib_flags |= cv2.CALIB_FIX_PRINCIPAL_POINT
    if options.fix_aspect_ratio:
        calib_flags |= cv2.CALIB_FIX_ASPECT_RATIO
    if options.zero_tangent_dist:
        calib_flags |= cv2.CALIB_ZERO_TANGENT_DIST
    if (num_ks > 3):
        calib_flags |= cv2.CALIB_RATIONAL_MODEL
    if (num_ks < 6):
        calib_flags |= cv2.CALIB_FIX_K6
    if (num_ks < 5):
        calib_flags |= cv2.CALIB_FIX_K5
    if (num_ks < 4):
        calib_flags |= cv2.CALIB_FIX_K4
    if (num_ks < 3):
        calib_flags |= cv2.CALIB_FIX_K3
    if (num_ks < 2):
        calib_flags |= cv2.CALIB_FIX_K2
    if (num_ks < 1):
        calib_flags |= cv2.CALIB_FIX_K1

    # Opencv calibration flags parsing:
    num_ks = options.fisheye_k_coefficients
    fisheye_calib_flags = 0
    if options.fisheye_fix_principal_point:
        fisheye_calib_flags |= cv2.fisheye.CALIB_FIX_PRINCIPAL_POINT
    if options.fisheye_fix_skew:
        fisheye_calib_flags |= cv2.fisheye.CALIB_FIX_SKEW
    if options.fisheye_recompute_extrinsicsts:
        fisheye_calib_flags |= cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC
    if options.fisheye_check_conditions:
        fisheye_calib_flags |= cv2.fisheye.CALIB_CHECK_COND
    if (num_ks < 4):
        fisheye_calib_flags |= cv2.fisheye.CALIB_FIX_K4
    if (num_ks < 3):
        fisheye_calib_flags |= cv2.fisheye.CALIB_FIX_K3
    if (num_ks < 2):
        fisheye_calib_flags |= cv2.fisheye.CALIB_FIX_K2
    if (num_ks < 1):
        fisheye_calib_flags |= cv2.fisheye.CALIB_FIX_K1

    pattern = Patterns.Chessboard
    if options.pattern == 'circles':
        pattern = Patterns.Circles
    elif options.pattern == 'acircles':
        pattern = Patterns.ACircles
    elif options.pattern == 'charuco':
        pattern = Patterns.ChArUco
    elif options.pattern != 'chessboard':
        print('Unrecognized pattern %s, defaulting to chessboard' % options.pattern)

    if options.disable_calib_cb_fast_check:
        checkerboard_flags = 0
    else:
        checkerboard_flags = cv2.CALIB_CB_FAST_CHECK

    rospy.init_node('cameracalibrator')
    node = OpenCVCalibrationNode(boards, options.service_check, sync, calib_flags, fisheye_calib_flags, pattern, options.camera_name,
                                 checkerboard_flags=checkerboard_flags, max_chessboard_speed=options.max_chessboard_speed,
                                 queue_size=options.queue_size)
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        import traceback
        traceback.print_exc()
