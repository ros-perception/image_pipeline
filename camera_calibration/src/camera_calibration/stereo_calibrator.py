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

from io import BytesIO
import math
import pickle
import random
import sys
import tarfile
import time

from camera_calibration.calibrator import (
    CalibrationException,
    Calibrator,
    CAMERA_MODEL,
    image_from_archive,
    ImageDrawable,
    Patterns,
)
from camera_calibration.mono_calibrator import MonoCalibrator
import cv2
import image_geometry
import numpy.linalg
from semver import VersionInfo


class StereoDrawable(ImageDrawable):

    def __init__(self):
        ImageDrawable.__init__(self)
        self.lscrib = None
        self.rscrib = None
        self.epierror = -1
        self.dim = -1


# TODO Replicate MonoCalibrator improvements in stereo


class StereoCalibrator(Calibrator):
    """
    Calibration class for stereo cameras.

    Example::

        limages = [cv2.imread("left%d.png") for i in range(8)]
        rimages = [cv2.imread("right%d.png") for i in range(8)]
        sc = StereoCalibrator()
        sc.cal(limages, rimages)
        print sc.as_message()
    """

    is_mono = False

    def __init__(self, *args, **kwargs):
        if 'name' not in kwargs:
            kwargs['name'] = 'narrow_stereo'
        super(StereoCalibrator, self).__init__(*args, **kwargs)
        self.l = MonoCalibrator(*args, **kwargs)  # noqa: E741
        self.r = MonoCalibrator(*args, **kwargs)
        # Collecting from two cameras in a horizontal stereo rig, can't get
        # full X range in the left camera.
        self.param_ranges[0] = 0.4

    # override
    def set_cammodel(self, modeltype):
        super(StereoCalibrator, self).set_cammodel(modeltype)
        self.l.set_cammodel(modeltype)
        self.r.set_cammodel(modeltype)

    def cal(self, limages, rimages):
        """
        Find chessboards in images, and run the OpenCV calibration solver.

        :param limages: source left images containing chessboards
        :type limages: list of :class:`cvMat`
        :param rimages: source right images containing chessboards
        :type rimages: list of :class:`cvMat`
        """
        goodcorners = self.collect_corners(limages, rimages)
        self.size = (limages[0].shape[1], limages[0].shape[0])
        self.l.size = self.size
        self.r.size = self.size
        self.cal_fromcorners(goodcorners)
        self.calibrated = True

    def collect_corners(self, limages, rimages):
        """
        Find pairs of left/right images that both contain a chessboard.

        Returns their corners as a list of pairs.
        """
        # Pick out (corners, ids, board) tuples
        lcorners = []
        rcorners = []
        for img in limages:
            (_, corners, _, ids, board, _) = self.downsample_and_detect(img)
            lcorners.append((corners, ids, board))
        for img in rimages:
            (_, corners, _, ids, board, _) = self.downsample_and_detect(img)
            rcorners.append((corners, ids, board))

        good = [
            (lco, rco, lid, rid, b)
            for ((lco, lid, b), (rco, rid, br)) in zip(lcorners, rcorners)
            if (lco is not None and rco is not None)
        ]

        if len(good) == 0:
            raise CalibrationException('No corners found in images!')
        return good

    def cal_fromcorners(self, good):
        # Perform monocular calibrations
        lcorners = [(lco, lid, b) for (lco, rco, lid, rid, b) in good]
        rcorners = [(rco, rid, b) for (lco, rco, lid, rid, b) in good]
        self.l.cal_fromcorners(lcorners)
        self.r.cal_fromcorners(rcorners)

        (lipts, ripts, _, _, boards) = zip(*good)

        opts = self.mk_object_points(boards, True)

        flags = cv2.CALIB_FIX_INTRINSIC

        self.T = numpy.zeros((3, 1), dtype=numpy.float64)
        self.R = numpy.eye(3, dtype=numpy.float64)

        if self.camera_model == CAMERA_MODEL.PINHOLE:
            print('stereo pinhole calibration...')
            if VersionInfo.parse(cv2.__version__).major < 3:
                ret_values = cv2.stereoCalibrate(
                    opts,
                    lipts,
                    ripts,
                    self.size,
                    self.l.intrinsics,
                    self.l.distortion,
                    self.r.intrinsics,
                    self.r.distortion,
                    self.R,  # R
                    self.T,  # T
                    criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 1, 1e-5),
                    flags=flags,
                )
            else:
                ret_values = cv2.stereoCalibrate(
                    opts,
                    lipts,
                    ripts,
                    self.l.intrinsics,
                    self.l.distortion,
                    self.r.intrinsics,
                    self.r.distortion,
                    self.size,
                    self.R,  # R
                    self.T,  # T
                    criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 1, 1e-5),
                    flags=flags,
                )
            print(f'Stereo RMS re-projection error: {ret_values[0]}')
        elif self.camera_model == CAMERA_MODEL.FISHEYE:
            print('stereo fisheye calibration...')
            if VersionInfo.parse(cv2.__version__).major < 3:
                print('ERROR: You need OpenCV >3 to use fisheye camera model')
                sys.exit()
            else:
                # WARNING: cv2.fisheye.stereoCalibrate wants float64 points
                lipts64 = numpy.asarray(lipts, dtype=numpy.float64)
                lipts = lipts64
                ripts64 = numpy.asarray(ripts, dtype=numpy.float64)
                ripts = ripts64
                opts64 = numpy.asarray(opts, dtype=numpy.float64)
                opts = opts64

                cv2.fisheye.stereoCalibrate(
                    opts,
                    lipts,
                    ripts,
                    self.l.intrinsics,
                    self.l.distortion,
                    self.r.intrinsics,
                    self.r.distortion,
                    self.size,
                    self.R,  # R
                    self.T,  # T
                    # 30, 1e-6
                    criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 1, 1e-5),
                    flags=flags,
                )

        self.set_alpha(0.0)

    def set_alpha(self, a):
        """
        Set the alpha value for the calibrated camera solution.

        The alpha value is a zoom, and ranges from 0 (zoomed in, all pixels
        in calibrated image are valid) to 1 (zoomed out, all pixels in
        original image are in calibrated image).
        """
        if self.camera_model == CAMERA_MODEL.PINHOLE:
            cv2.stereoRectify(
                self.l.intrinsics,
                self.l.distortion,
                self.r.intrinsics,
                self.r.distortion,
                self.size,
                self.R,
                self.T,
                self.l.R,
                self.r.R,
                self.l.P,
                self.r.P,
                alpha=a,
            )

            cv2.initUndistortRectifyMap(
                self.l.intrinsics,
                self.l.distortion,
                self.l.R,
                self.l.P,
                self.size,
                cv2.CV_32FC1,
                self.l.mapx,
                self.l.mapy,
            )
            cv2.initUndistortRectifyMap(
                self.r.intrinsics,
                self.r.distortion,
                self.r.R,
                self.r.P,
                self.size,
                cv2.CV_32FC1,
                self.r.mapx,
                self.r.mapy,
            )

        elif self.camera_model == CAMERA_MODEL.FISHEYE:
            self.Q = numpy.zeros((4, 4), dtype=numpy.float64)

            # Operation flags that may be zero or CALIB_ZERO_DISPARITY .
            flags = cv2.CALIB_ZERO_DISPARITY
            # If the flag is set, the function makes the principal points of each camera
            # have the same pixel coordinates in the rectified views.
            # And if the flag is not set, the function may still shift the images in the
            # horizontal or vertical direction
            # (depending on the orientation of epipolar lines) to maximize the useful image area.

            cv2.fisheye.stereoRectify(
                self.l.intrinsics,
                self.l.distortion,
                self.r.intrinsics,
                self.r.distortion,
                self.size,
                self.R,
                self.T,
                flags,
                self.l.R,
                self.r.R,
                self.l.P,
                self.r.P,
                self.Q,
                self.size,
                a,
                1.0,
            )
            self.l.P[:3, :3] = numpy.dot(self.l.intrinsics, self.l.R)
            self.r.P[:3, :3] = numpy.dot(self.r.intrinsics, self.r.R)
            cv2.fisheye.initUndistortRectifyMap(
                self.l.intrinsics,
                self.l.distortion,
                self.l.R,
                self.l.intrinsics,
                self.size,
                cv2.CV_32FC1,
                self.l.mapx,
                self.l.mapy,
            )
            cv2.fisheye.initUndistortRectifyMap(
                self.r.intrinsics,
                self.r.distortion,
                self.r.R,
                self.r.intrinsics,
                self.size,
                cv2.CV_32FC1,
                self.r.mapx,
                self.r.mapy,
            )

    def as_message(self):
        """
        Return the camera calibration as a pair of CameraInfo messages.

        Messages are for the left and right cameras respectively.
        """
        return (
            self.lrmsg(
                self.l.distortion,
                self.l.intrinsics,
                self.l.R,
                self.l.P,
                self.size,
                self.l.camera_model,
            ),
            self.lrmsg(
                self.r.distortion,
                self.r.intrinsics,
                self.r.R,
                self.r.P,
                self.size,
                self.r.camera_model,
            ),
        )

    def from_message(self, msgs, alpha=0.0):
        """Initialize the camera calibration from a pair of CameraInfo messages."""
        self.size = (msgs[0].width, msgs[0].height)

        self.T = numpy.zeros((3, 1), dtype=numpy.float64)
        self.R = numpy.eye(3, dtype=numpy.float64)

        self.l.from_message(msgs[0])
        self.r.from_message(msgs[1])
        # Need to compute self.T and self.R here, using the monocular parameters above
        if False:
            self.set_alpha(0.0)

    def report(self):
        print('\nLeft:')
        self.lrreport(self.l.distortion, self.l.intrinsics, self.l.R, self.l.P)
        print('\nRight:')
        self.lrreport(self.r.distortion, self.r.intrinsics, self.r.R, self.r.P)
        print('self.T =', numpy.ravel(self.T).tolist())
        print('self.R =', numpy.ravel(self.R).tolist())

    def ost(self):
        return self.lrost(
            self.name + '/left',
            self.l.distortion,
            self.l.intrinsics,
            self.l.R,
            self.l.P,
            self.size,
        ) + self.lrost(
            self.name + '/right',
            self.r.distortion,
            self.r.intrinsics,
            self.r.R,
            self.r.P,
            self.size,
        )

    def yaml(self, suffix, info):
        return self.lryaml(
            self.name + suffix,
            info.distortion,
            info.intrinsics,
            info.R,
            info.P,
            self.size,
            self.camera_model,
        )

    # TODO Get rid of "from_images" versions of these, instead have function to get
    # undistorted corners
    def epipolar_error_from_images(self, limage, rimage):
        """
        Detect the checkerboard in both images and compute the epipolar error.

        Mainly for use in tests.
        """
        lcorners = self.downsample_and_detect(limage)[1]
        rcorners = self.downsample_and_detect(rimage)[1]
        if lcorners is None or rcorners is None:
            return None

        lundistorted = self.l.undistort_points(lcorners)
        rundistorted = self.r.undistort_points(rcorners)

        return self.epipolar_error(lundistorted, rundistorted)

    def epipolar_error(self, lcorners, rcorners):
        """Compute the epipolar error from two sets of matching undistorted points."""
        d = lcorners[:, :, 1] - rcorners[:, :, 1]
        return numpy.sqrt(numpy.square(d).sum() / d.size)

    def chessboard_size_from_images(self, limage, rimage):
        _, lcorners, _, _, board, _ = self.downsample_and_detect(limage)
        _, rcorners, _, _, board, _ = self.downsample_and_detect(rimage)
        if lcorners is None or rcorners is None:
            return None

        lundistorted = self.l.undistort_points(lcorners)
        rundistorted = self.r.undistort_points(rcorners)

        return self.chessboard_size(lundistorted, rundistorted, board)

    def chessboard_size(self, lcorners, rcorners, board, msg=None):
        """
        Compute the square edge length from two sets of matching undistorted points.

        Uses the current calibration.

        :param msg: a tuple of (left_msg, right_msg).
        """
        # Project the points to 3d
        cam = image_geometry.StereoCameraModel()
        if msg is None:
            msg = self.as_message()
        cam.from_camera_info(*msg)
        disparities = lcorners[:, :, 0] - rcorners[:, :, 0]
        pt3d = [
            cam.project_pixel_to_3d((lcorners[i, 0, 0], lcorners[i, 0, 1]), disparities[i, 0])
            for i in range(lcorners.shape[0])
        ]

        def l2(p0, p1):
            return math.sqrt(sum([(c0 - c1) ** 2 for (c0, c1) in zip(p0, p1)]))

        # Compute the length from each horizontal and vertical line, and return the mean
        cc = board.n_cols
        cr = board.n_rows
        lengths = [l2(pt3d[cc * r + 0], pt3d[cc * r + (cc - 1)]) / (cc - 1) for r in range(cr)] + [
            l2(pt3d[c + 0], pt3d[c + (cc * (cr - 1))]) / (cr - 1) for c in range(cc)
        ]
        return sum(lengths) / len(lengths)

    def update_db(self, lgray, rgray, lcorners, rcorners, lids, rids, lboard):
        """Update database with images and good corners if good samples are detected."""
        params = self.get_parameters(lcorners, lids, lboard, (lgray.shape[1], lgray.shape[0]))
        if self.is_good_sample(
            params, lcorners, lids, self.last_frame_corners, self.last_frame_ids
        ):
            self.db.append((params, lgray, rgray))
            self.good_corners.append((lcorners, rcorners, lids, rids, lboard))
            print(
                (
                    '*** Added sample %d, p_x = %.3f, p_y = %.3f, p_size = %.3f, skew = %.3f'
                    % tuple([len(self.db)] + params)
                )
            )

    def handle_msg(self, msg):
        # TODO Various asserts that images have same dimension, same board detected...
        (lmsg, rmsg) = msg
        lgray = self.mkgray(lmsg)
        rgray = self.mkgray(rmsg)
        epierror = -1

        # Get display-images-to-be and detections of the calibration target
        lscrib_mono, lcorners, ldownsampled_corners, lids, lboard, (x_scale, y_scale) = (
            self.downsample_and_detect(lgray)
        )
        rscrib_mono, rcorners, rdownsampled_corners, rids, rboard, _ = self.downsample_and_detect(
            rgray
        )

        if self.calibrated:
            # Show rectified images
            lremap = self.l.remap(lgray)
            rremap = self.r.remap(rgray)
            lrect = lremap
            rrect = rremap
            if x_scale != 1.0 or y_scale != 1.0:
                lrect = cv2.resize(lremap, (lscrib_mono.shape[1], lscrib_mono.shape[0]))
                rrect = cv2.resize(rremap, (rscrib_mono.shape[1], rscrib_mono.shape[0]))

            lscrib = cv2.cvtColor(lrect, cv2.COLOR_GRAY2BGR)
            rscrib = cv2.cvtColor(rrect, cv2.COLOR_GRAY2BGR)

            # Draw rectified corners
            if lcorners is not None:
                lundistorted = self.l.undistort_points(lcorners)
                scrib_src = lundistorted.copy()
                scrib_src[:, :, 0] /= x_scale
                scrib_src[:, :, 1] /= y_scale
                cv2.drawChessboardCorners(lscrib, (lboard.n_cols, lboard.n_rows), scrib_src, True)

            if rcorners is not None:
                rundistorted = self.r.undistort_points(rcorners)
                scrib_src = rundistorted.copy()
                scrib_src[:, :, 0] /= x_scale
                scrib_src[:, :, 1] /= y_scale
                cv2.drawChessboardCorners(rscrib, (rboard.n_cols, rboard.n_rows), scrib_src, True)

            # Report epipolar error
            if lcorners is not None and rcorners is not None and len(lcorners) == len(rcorners):
                epierror = self.epipolar_error(lundistorted, rundistorted)

        else:
            lscrib = cv2.cvtColor(lscrib_mono, cv2.COLOR_GRAY2BGR)
            rscrib = cv2.cvtColor(rscrib_mono, cv2.COLOR_GRAY2BGR)
            # Draw any detected chessboards onto display (downsampled) images
            if lcorners is not None:
                cv2.drawChessboardCorners(
                    lscrib, (lboard.n_cols, lboard.n_rows), ldownsampled_corners, True
                )
            if rcorners is not None:
                cv2.drawChessboardCorners(
                    rscrib, (rboard.n_cols, rboard.n_rows), rdownsampled_corners, True
                )

            # Add sample to database only if it's sufficiently different from any previous sample
            if lcorners is not None and rcorners is not None and len(lcorners) == len(rcorners):
                # Add samples only with entire board in view if charuco
                if self.pattern == Patterns.ChArUco:
                    if len(lcorners) == lboard.charuco_board.chessboardCorners.shape[0]:
                        self.update_db(lgray, rgray, lcorners, rcorners, lids, rids, lboard)
                else:
                    self.update_db(lgray, rgray, lcorners, rcorners, lids, rids, lboard)

        self.last_frame_corners = lcorners
        self.last_frame_ids = lids
        rv = StereoDrawable()
        rv.lscrib = lscrib
        rv.rscrib = rscrib
        rv.params = self.compute_goodenough()
        rv.epierror = epierror
        return rv

    def do_calibration(self, dump=False):
        # TODO MonoCalibrator collects corners if needed here
        # TODO Needs to be set externally
        self.size = (self.db[0][1].shape[1], self.db[0][1].shape[0])
        # Dump should only occur if user wants it
        if dump:
            pickle.dump(
                (self.is_mono, self.size, self.good_corners),
                open('/tmp/camera_calibration_%08x.pickle' % random.getrandbits(32), 'w'),
            )
        self.l.size = self.size
        self.r.size = self.size
        self.cal_fromcorners(self.good_corners)
        self.calibrated = True
        # DEBUG
        print((self.report()))
        print((self.ost()))

    def do_tarfile_save(self, tf):
        """Write images and calibration solution to a tarfile object."""
        ims = [('left-%04d.png' % i, im) for i, (_, im, _) in enumerate(self.db)] + [
            ('right-%04d.png' % i, im) for i, (_, _, im) in enumerate(self.db)
        ]

        def taradd(name, buf):
            if isinstance(buf, str):
                s = BytesIO(buf.encode('utf-8'))
            else:
                s = BytesIO(buf)
            ti = tarfile.TarInfo(name)
            ti.size = len(s.getvalue())
            ti.uname = 'calibrator'
            ti.mtime = int(time.time())
            tf.addfile(tarinfo=ti, fileobj=s)

        for name, im in ims:
            taradd(name, cv2.imencode('.png', im)[1].tostring())
        taradd('left.yaml', self.yaml('/left', self.l))
        taradd('right.yaml', self.yaml('/right', self.r))
        taradd('ost.txt', self.ost())

    def do_tarfile_calibration(self, filename):
        archive = tarfile.open(filename, 'r')
        limages = [
            image_from_archive(archive, f)
            for f in archive.getnames()
            if (f.startswith('left') and (f.endswith('pgm') or f.endswith('png')))
        ]
        rimages = [
            image_from_archive(archive, f)
            for f in archive.getnames()
            if (f.startswith('right') and (f.endswith('pgm') or f.endswith('png')))
        ]

        if not len(limages) == len(rimages):
            raise CalibrationException(
                "Left, right images don't match. %d left images, %d right"
                % (len(limages), len(rimages))
            )

        # \todo Check that the filenames match and stuff

        self.cal(limages, rimages)
