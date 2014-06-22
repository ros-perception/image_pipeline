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

try:
    from cStringIO import StringIO
except ImportError:
    from io import StringIO
import cv
import cv2
import cv_bridge
import image_geometry
import math
import numpy.linalg
import pickle
import random
import sensor_msgs.msg
import tarfile
import time



# Supported calibration patterns
class Patterns:
    Chessboard, Circles, ACircles = list(range(3))

class CalibrationException(Exception):
    pass

def cvmat_iterator(cvmat):
    for i in range(cvmat.rows):
        for j in range(cvmat.cols):
            yield cvmat[i,j]

# TODO: Make pattern per-board?
class ChessboardInfo(object):
    def __init__(self, n_cols = 0, n_rows = 0, dim = 0.0):
        self.n_cols = n_cols
        self.n_rows = n_rows
        self.dim = dim

def _get_num_pts(good):
    num_pts = 0
    for (_i, corners) in enumerate(good):
        num_pts += len(corners)
    return num_pts

# Make all private!!!!!
def lmin(seq1, seq2):
    """ Pairwise minimum of two sequences """
    return [min(a, b) for (a, b) in zip(seq1, seq2)]

def lmax(seq1, seq2):
    """ Pairwise maximum of two sequences """
    return [max(a, b) for (a, b) in zip(seq1, seq2)]

def mean(seq):
    return sum(seq) / len(seq)

def _pdist(p1, p2):
    """
    Distance bwt two points. p1 = (x, y), p2 = (x, y)
    """
    return math.sqrt(math.pow(p1[0] - p2[0], 2) + math.pow(p1[1] - p2[1], 2))

def _get_outside_corners(corners, board):
    """
    Return the four corners of the board as a whole, as (up_left, up_right, down_right, down_left).
    """
    xdim = board.n_cols
    ydim = board.n_rows

    if len(corners) != xdim * ydim:
        raise Exception("Invalid number of corners! %d corners. X: %d, Y: %d" % (len(corners),
                                                                                 xdim, ydim))

    up_left    = numpy.array( corners[0] )
    up_right   = numpy.array( corners[xdim - 1] )
    down_right = numpy.array( corners[-1] )
    down_left  = numpy.array( corners[-xdim] )

    return (up_left, up_right, down_right, down_left)

def _get_skew(corners, board):
    """
    Get skew for given checkerboard detection. 
    Scaled to [0,1], which 0 = no skew, 1 = high skew
    Skew is proportional to the divergence of three outside corners from 90 degrees.
    """
    # TODO Using three nearby interior corners might be more robust, outside corners occasionally
    # get mis-detected
    up_left, up_right, down_right, _ = _get_outside_corners(corners, board)

    def angle(a, b, c):
        """
        Return angle between lines ab, bc
        """
        ab = a - b
        cb = c - b
        return math.acos(numpy.dot(ab,cb) / (numpy.linalg.norm(ab) * numpy.linalg.norm(cb)))

    skew = min(1.0, 2. * abs((math.pi / 2.) - angle(up_left, up_right, down_right)))
    return skew

def _get_area(corners, board):
    """
    Get 2d image area of the detected checkerboard.
    The projected checkerboard is assumed to be a convex quadrilateral, and the area computed as
    |p X q|/2; see http://mathworld.wolfram.com/Quadrilateral.html.
    """
    (up_left, up_right, down_right, down_left) = _get_outside_corners(corners, board)
    a = up_right - up_left
    b = down_right - up_right
    c = down_left - down_right
    p = b + c
    q = a + b
    return abs(p[0]*q[1] - p[1]*q[0]) / 2.

def _get_corners(img, board, refine = True):
    """
    Get corners for a particular chessboard for an image
    """
    w, h = cv.GetSize(img)
    if img.channels == 3:
        mono = cv.CreateMat(h, w, cv.CV_8UC1)
        cv.CvtColor(img, mono, cv.CV_BGR2GRAY)
    else:
        mono = img
    (ok, corners) = cv.FindChessboardCorners(mono, (board.n_cols, board.n_rows), cv.CV_CALIB_CB_ADAPTIVE_THRESH | cv.CV_CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK)
    
    # If any corners are within BORDER pixels of the screen edge, reject the detection by setting ok to false
    # NOTE: This may cause problems with very low-resolution cameras, where 8 pixels is a non-negligible fraction
    # of the image size. See http://answers.ros.org/question/3155/how-can-i-calibrate-low-resolution-cameras
    BORDER = 8
    if not all([(BORDER < x < (w - BORDER)) and (BORDER < y < (h - BORDER)) for (x, y) in corners]):
        ok = False
        
    if refine and ok:
        # Use a radius of half the minimum distance between corners. This should be large enough to snap to the
        # correct corner, but not so large as to include a wrong corner in the search window.
        min_distance = float("inf")
        for row in range(board.n_rows):
            for col in range(board.n_cols - 1):
                index = row*board.n_rows + col
                min_distance = min(min_distance, _pdist(corners[index], corners[index + 1]))
        for row in range(board.n_rows - 1):
            for col in range(board.n_cols):
                index = row*board.n_rows + col
                min_distance = min(min_distance, _pdist(corners[index], corners[index + board.n_cols]))
        radius = int(math.ceil(min_distance * 0.5))
        corners = cv.FindCornerSubPix(mono, corners, (radius,radius), (-1,-1),
                                      ( cv.CV_TERMCRIT_EPS+cv.CV_TERMCRIT_ITER, 30, 0.1 ))
        
    return (ok, corners)

def _get_circles(img, board, pattern):
    """
    Get circle centers for a symmetric or asymmetric grid
    """
    w, h = cv.GetSize(img)
    if img.channels == 3:
        mono = cv.CreateMat(h, w, cv.CV_8UC1)
        cv.CvtColor(img, mono, cv.CV_BGR2GRAY)
    else:
        mono = img

    flag = cv2.CALIB_CB_SYMMETRIC_GRID
    if pattern == Patterns.ACircles:
        flag = cv2.CALIB_CB_ASYMMETRIC_GRID
    mono_arr = numpy.array(mono)
    (ok, corners) = cv2.findCirclesGrid(mono_arr, (board.n_cols, board.n_rows), flags=flag)

    # In symmetric case, findCirclesGrid does not detect the target if it's turned sideways. So we try
    # again with dimensions swapped - not so efficient.
    # TODO Better to add as second board? Corner ordering will change.
    if not ok and pattern == Patterns.Circles:
        (ok, corners) = cv2.findCirclesGrid(mono_arr, (board.n_rows, board.n_cols), flags=flag)

    # For some reason findCirclesGrid returns centers as [[x y]] instead of (x y) like FindChessboardCorners
    if corners is not None:
        corners = [(x, y) for [[x, y]] in corners]

    return (ok, corners)

def _get_total_num_pts(boards):
    rv = 0
    for b in boards:
        rv += b.n_cols * b.n_rows
    return rv


# TODO self.size needs to come from CameraInfo, full resolution
class Calibrator(object):
    """
    Base class for calibration system
    """
    def __init__(self, boards, flags=0, pattern=Patterns.Chessboard, name=''):
        # Ordering the dimensions for the different detectors is actually a minefield...
        if pattern == Patterns.Chessboard:
            # Make sure n_cols > n_rows to agree with OpenCV CB detector output
            self._boards = [ChessboardInfo(max(i.n_cols, i.n_rows), min(i.n_cols, i.n_rows), i.dim) for i in boards]
        elif pattern == Patterns.ACircles:
            # 7x4 and 4x7 are actually different patterns. Assume square-ish pattern, so n_rows > n_cols.
            self._boards = [ChessboardInfo(min(i.n_cols, i.n_rows), max(i.n_cols, i.n_rows), i.dim) for i in boards]
        elif pattern == Patterns.Circles:
            # We end up having to check both ways anyway
            self._boards = boards

        # Set to true after we perform calibration
        self.calibrated = False
        self.calib_flags = flags
        self.pattern = pattern
        self.br = cv_bridge.CvBridge()

        # self.db is list of (parameters, image) samples for use in calibration. parameters has form
        # (X, Y, size, skew) all normalized to [0,1], to keep track of what sort of samples we've taken
        # and ensure enough variety.
        self.db = []
        # For each db sample, we also record the detected corners.
        self.good_corners = []
        # Set to true when we have sufficiently varied samples to calibrate
        self.goodenough = False
        self.param_ranges = [0.7, 0.7, 0.4, 0.5]
        self.name = name

    def mkgray(self, msg):
        """
        Convert a message into a 8-bit 1 channel monochrome OpenCV image
        """
        # as cv_bridge automatically scales, we need to remove that behavior
        if msg.encoding.endswith('16'):
            mono16 = self.br.imgmsg_to_cv(msg, "mono16")
            mono8 = cv.CreateMat(mono16.rows, mono16.cols, cv.CV_8UC1)
            cv.ConvertScale(mono16, mono8)
            return mono8
        elif 'FC1' in msg.encoding:
            # floating point image handling
            img = self.br.imgmsg_to_cv(msg, "passthrough")
            mono_img = cv.CreateMat(img.rows, img.cols, cv.CV_8UC1)
            _, max_val, _, _ = cv.MinMaxLoc(img)
            scale = 255.0 / max_val if max_val > 0 else 1.0
            cv.ConvertScale(img, mono_img, scale)
            return mono_img
        else:
            return self.br.imgmsg_to_cv(msg, "mono8")

    def get_parameters(self, corners, board, size):
        """
        Return list of parameters [X, Y, size, skew] describing the checkerboard view.
        """
        (width, height) = size
        Xs = [x for (x, y) in corners]
        Ys = [y for (x, y) in corners]
        area = _get_area(corners, board)
        border = math.sqrt(area)
        # For X and Y, we "shrink" the image all around by approx. half the board size.
        # Otherwise large boards are penalized because you can't get much X/Y variation.
        p_x = min(1.0, max(0.0, (mean(Xs) - border / 2) / (width  - border)))
        p_y = min(1.0, max(0.0, (mean(Ys) - border / 2) / (height - border)))
        p_size = math.sqrt(area / (width * height))
        skew = _get_skew(corners, board)
        params = [p_x, p_y, p_size, skew]
        return params

    def is_good_sample(self, params):
        """
        Returns true if the checkerboard detection described by params should be added to the database.
        """
        if not self.db:
            return True

        def param_distance(p1, p2):
            return sum([abs(a-b) for (a,b) in zip(p1, p2)])

        db_params = [sample[0] for sample in self.db]
        d = min([param_distance(params, p) for p in db_params])
        #print "d = %.3f" % d #DEBUG
        # TODO What's a good threshold here? Should it be configurable?
        return d > 0.2

    _param_names = ["X", "Y", "Size", "Skew"]

    def compute_goodenough(self):
        if not self.db:
            return None

        # Find range of checkerboard poses covered by samples in database
        all_params = [sample[0] for sample in self.db]
        min_params = all_params[0]
        max_params = all_params[0]
        for params in all_params[1:]:
            min_params = lmin(min_params, params)
            max_params = lmax(max_params, params)
        # Don't reward small size or skew
        min_params = [min_params[0], min_params[1], 0., 0.]

        # For each parameter, judge how much progress has been made toward adequate variation
        progress = [min((hi - lo) / r, 1.0) for (lo, hi, r) in zip(min_params, max_params, self.param_ranges)]
        # If we have lots of samples, allow calibration even if not all parameters are green
        # TODO Awkward that we update self.goodenough instead of returning it
        self.goodenough = (len(self.db) >= 40) or all([p == 1.0 for p in progress])

        return list(zip(self._param_names, min_params, max_params, progress))

    def mk_object_points(self, boards, use_board_size = False):
        opts = cv.CreateMat(_get_total_num_pts(boards), 3, cv.CV_32FC1)
        idx = 0
        for i, b in enumerate(boards):
            num_pts = b.n_cols * b.n_rows
            for j in range(num_pts):
                if use_board_size:
                    opts[idx + j, 0] = (j / b.n_cols) * b.dim
                    opts[idx + j, 1] = (j % b.n_cols) * b.dim
                    opts[idx + j, 2] = 0
                else:
                    opts[idx + j, 0] = (j / b.n_cols) * 1.0
                    opts[idx + j, 1] = (j % b.n_cols) * 1.0
                    opts[idx + j, 2] = 0
            idx += num_pts
        return opts

    def mk_image_points(self, good):
        total_pts = _get_total_num_pts( [ b for (_, b) in good ] )
        ipts = cv.CreateMat(total_pts, 2, cv.CV_32FC1)

        idx = 0
        for (corners, _) in good:
            for j in range(len(corners)):
                ipts[idx + j, 0] = corners[j][0]
                ipts[idx + j, 1] = corners[j][1]
            idx += len(corners)

        return cv.Reshape(ipts, 2)

    def mk_point_counts(self, boards):
        npts = cv.CreateMat(len(boards), 1, cv.CV_32SC1)
        for i, board in enumerate(boards):
            npts[i, 0] = board.n_cols * board.n_rows
        return npts



    def get_corners(self, img, refine = True):
        """
        Use cvFindChessboardCorners to find corners of chessboard in image.

        Check all boards. Return corners for first chessboard that it detects
        if given multiple size chessboards.

        Returns (ok, corners, board)
        """

        for b in self._boards:
            if self.pattern == Patterns.Chessboard:
                (ok, corners) = _get_corners(img, b, refine)
            else:
                (ok, corners) = _get_circles(img, b, self.pattern)
            if ok:
                return (ok, corners, b)
        return (False, None, None)

    def downsample_and_detect(self, img):
        """
        Downsample the input image to approximately VGA resolution and detect the
        calibration target corners in the full-size image.

        Combines these apparently orthogonal duties as an optimization. Checkerboard
        detection is too expensive on large images, so it's better to do detection on
        the smaller display image and scale the corners back up to the correct size.

        Returns (scrib, corners, downsampled_corners, board, (x_scale, y_scale)).
        """
        # Scale the input image down to ~VGA size
        (width, height) = cv.GetSize(img)
        scale = math.sqrt( (width*height) / (640.*480.) )
        if scale > 1.0:
            scrib = cv.CreateMat(int(height / scale), int(width / scale), cv.GetElemType(img))
            cv.Resize(img, scrib)
        else:
            scrib = cv.CloneMat(img)
        # Due to rounding, actual horizontal/vertical scaling may differ slightly
        x_scale = float(width) / scrib.cols
        y_scale = float(height) / scrib.rows

        if self.pattern == Patterns.Chessboard:
            # Detect checkerboard
            (ok, downsampled_corners, board) = self.get_corners(scrib, refine = True)

            # Scale corners back to full size image
            corners = None
            if ok:
                if scale > 1.0:
                    # Refine up-scaled corners in the original full-res image
                    # TODO Does this really make a difference in practice?
                    corners_unrefined = [(c[0]*x_scale, c[1]*y_scale) for c in downsampled_corners]
                    radius = int(math.ceil(scale))
                    if img.channels == 3:
                        mono = cv.CreateMat(img.rows, img.cols, cv.CV_8UC1)
                        cv.CvtColor(img, mono, cv.CV_BGR2GRAY)
                    else:
                        mono = img
                    corners = cv.FindCornerSubPix(mono, corners_unrefined, (radius,radius), (-1,-1),
                                                  ( cv.CV_TERMCRIT_EPS+cv.CV_TERMCRIT_ITER, 30, 0.1 ))
                else:
                    corners = downsampled_corners
        else:
            # Circle grid detection is fast even on large images
            (ok, corners, board) = self.get_corners(img)
            # Scale corners to downsampled image for display
            downsampled_corners = None
            if ok:
                print(corners)
                if scale > 1.0:
                    downsampled_corners = [(c[0]/x_scale, c[1]/y_scale) for c in corners]
                else:
                    downsampled_corners = corners

        return (scrib, corners, downsampled_corners, board, (x_scale, y_scale))


    def lrmsg(self, d, k, r, p):
        """ Used by :meth:`as_message`.  Return a CameraInfo message for the given calibration matrices """
        msg = sensor_msgs.msg.CameraInfo()
        (msg.width, msg.height) = self.size
        if d.rows > 5:
            msg.distortion_model = "rational_polynomial"
        else:
            msg.distortion_model = "plumb_bob"
        msg.D = [d[i,0] for i in range(d.rows)]
        msg.K = list(cvmat_iterator(k))
        msg.R = list(cvmat_iterator(r))
        msg.P = list(cvmat_iterator(p))
        return msg

    def lrreport(self, d, k, r, p):
        print(("D = ", list(cvmat_iterator(d))))
        print(("K = ", list(cvmat_iterator(k))))
        print(("R = ", list(cvmat_iterator(r))))
        print(("P = ", list(cvmat_iterator(p))))

    # TODO Get rid of OST format, show output as YAML instead
    def lrost(self, name, d, k, r, p):
        calmessage = (
        "# oST version 5.0 parameters\n"
        + "\n"
        + "\n"
        + "[image]\n"
        + "\n"
        + "width\n"
        + str(self.size[0]) + "\n"
        + "\n"
        + "height\n"
        + str(self.size[1]) + "\n"
        + "\n"
        + "[%s]" % name + "\n"
        + "\n"
        + "camera matrix\n"
        + " ".join(["%8f" % k[0,i] for i in range(3)]) + "\n"
        + " ".join(["%8f" % k[1,i] for i in range(3)]) + "\n"
        + " ".join(["%8f" % k[2,i] for i in range(3)]) + "\n"
        + "\n"
        + "distortion\n"
        + " ".join(["%8f" % d[i,0] for i in range(d.rows)]) + "\n"
        + "\n"
        + "rectification\n"
        + " ".join(["%8f" % r[0,i] for i in range(3)]) + "\n"
        + " ".join(["%8f" % r[1,i] for i in range(3)]) + "\n"
        + " ".join(["%8f" % r[2,i] for i in range(3)]) + "\n"
        + "\n"
        + "projection\n"
        + " ".join(["%8f" % p[0,i] for i in range(4)]) + "\n"
        + " ".join(["%8f" % p[1,i] for i in range(4)]) + "\n"
        + " ".join(["%8f" % p[2,i] for i in range(4)]) + "\n"
        + "\n")
        assert len(calmessage) < 525, "Calibration info must be less than 525 bytes"
        return calmessage

    def do_save(self):
        filename = '/tmp/calibrationdata.tar.gz'
        tf = tarfile.open(filename, 'w:gz')
        self.do_tarfile_save(tf) # Must be overridden in subclasses
        tf.close()
        print(("Wrote calibration data to", filename))

def image_from_archive(archive, name):
    """
    Load image PGM file from tar archive. 

    Used for tarfile loading and unit test.
    """
    member = archive.getmember(name)
    filedata = archive.extractfile(member).read()
    imagefiledata = cv.CreateMat(1, len(filedata), cv.CV_8UC1)
    cv.SetData(imagefiledata, filedata, len(filedata))
    return cv.DecodeImageM(imagefiledata)

class ImageDrawable(object):
    """
    Passed to CalibrationNode after image handled. Allows plotting of images
    with detected corner points
    """
    def __init__(self):
        self.params = None

class MonoDrawable(ImageDrawable):
    def __init__(self):
        ImageDrawable.__init__(self)
        self.scrib = None
        self.linear_error = -1.0
                

class StereoDrawable(ImageDrawable):
    def __init__(self):
        ImageDrawable.__init__(self)
        self.lscrib = None
        self.rscrib = None
        self.epierror = -1
        self.dim = -1


class MonoCalibrator(Calibrator):
    """
    Calibration class for monocular cameras::

        images = [cv.LoadImage("mono%d.png") for i in range(8)]
        mc = MonoCalibrator()
        mc.cal(images)
        print mc.as_message()
    """

    is_mono = True  # TODO Could get rid of is_mono

    def __init__(self, *args, **kwargs):
        if 'name' not in kwargs:
            kwargs['name'] = 'narrow_stereo/left'
        super(MonoCalibrator, self).__init__(*args, **kwargs)

    def cal(self, images):
        """
        Calibrate camera from given images
        """
        goodcorners = self.collect_corners(images)
        self.cal_fromcorners(goodcorners)
        self.calibrated = True

    def collect_corners(self, images):
        """
        :param images: source images containing chessboards
        :type images: list of :class:`cvMat`

        Find chessboards in all images.

        Return [ (corners, ChessboardInfo) ]
        """
        self.size = cv.GetSize(images[0])
        corners = [self.get_corners(i) for i in images]

        goodcorners = [(co, b) for (ok, co, b) in corners if ok]
        if not goodcorners:
            raise CalibrationException("No corners found in images!")
        return goodcorners

    def cal_fromcorners(self, good):
        """
        :param good: Good corner positions and boards 
        :type good: [(corners, ChessboardInfo)]

        
        """
        boards = [ b for (_, b) in good ]

        ipts = self.mk_image_points(good)
        opts = self.mk_object_points(boards)
        npts = self.mk_point_counts(boards)

        intrinsics = cv.CreateMat(3, 3, cv.CV_64FC1)
        if self.calib_flags & cv2.CALIB_RATIONAL_MODEL:
            distortion = cv.CreateMat(8, 1, cv.CV_64FC1) # rational polynomial
        else:
            distortion = cv.CreateMat(5, 1, cv.CV_64FC1) # plumb bob
        cv.SetZero(intrinsics)
        cv.SetZero(distortion)
        # If FIX_ASPECT_RATIO flag set, enforce focal lengths have 1/1 ratio
        intrinsics[0,0] = 1.0
        intrinsics[1,1] = 1.0
        cv.CalibrateCamera2(opts, ipts, npts,
                   self.size, intrinsics,
                   distortion,
                   cv.CreateMat(len(good), 3, cv.CV_32FC1),
                   cv.CreateMat(len(good), 3, cv.CV_32FC1),
                   flags = self.calib_flags)
        self.intrinsics = intrinsics
        self.distortion = distortion

        # R is identity matrix for monocular calibration
        self.R = cv.CreateMat(3, 3, cv.CV_64FC1)
        cv.SetIdentity(self.R)
        self.P = cv.CreateMat(3, 4, cv.CV_64FC1)
        cv.SetZero(self.P)

        self.mapx = cv.CreateImage(self.size, cv.IPL_DEPTH_32F, 1)
        self.mapy = cv.CreateImage(self.size, cv.IPL_DEPTH_32F, 1)
        self.set_alpha(0.0)

    def set_alpha(self, a):
        """
        Set the alpha value for the calibrated camera solution.  The alpha
        value is a zoom, and ranges from 0 (zoomed in, all pixels in
        calibrated image are valid) to 1 (zoomed out, all pixels in
        original image are in calibrated image).
        """

        # NOTE: Prior to Electric, this code was broken such that we never actually saved the new
        # camera matrix. In effect, this enforced P = [K|0] for monocular cameras.
        # TODO: Verify that OpenCV #1199 gets applied (improved GetOptimalNewCameraMatrix)
        ncm = cv.GetSubRect(self.P, (0, 0, 3, 3))
        cv.GetOptimalNewCameraMatrix(self.intrinsics, self.distortion, self.size, a, ncm)
        cv.InitUndistortRectifyMap(self.intrinsics, self.distortion, self.R, ncm, self.mapx, self.mapy)

    def remap(self, src):
        """
        :param src: source image
        :type src: :class:`cvMat`

        Apply the post-calibration undistortion to the source image
        """
        r = cv.CloneMat(src)
        cv.Remap(src, r, self.mapx, self.mapy)
        return r

    def undistort_points(self, src):
        """
        :param src: N source pixel points (u,v) as an Nx2 matrix
        :type src: :class:`cvMat`

        Apply the post-calibration undistortion to the source points
        """

        dst = cv.CloneMat(src)
        cv.UndistortPoints(src, dst, self.intrinsics, self.distortion, R = self.R, P = self.P)
        return dst

    def as_message(self):
        """ Return the camera calibration as a CameraInfo message """
        return self.lrmsg(self.distortion, self.intrinsics, self.R, self.P)

    def from_message(self, msg, alpha = 0.0):
        """ Initialize the camera calibration from a CameraInfo message """

        self.size = (msg.width, msg.height)
        self.intrinsics = cv.CreateMat(3, 3, cv.CV_64FC1)
        self.distortion = cv.CreateMat(len(msg.D), 1, cv.CV_64FC1)
        self.R = cv.CreateMat(3, 3, cv.CV_64FC1)
        self.P = cv.CreateMat(3, 4, cv.CV_64FC1)
        for i in range(len(msg.D)):
            self.distortion[i, 0] = msg.D[i]
        for i in range(9):
            cv.Set1D(self.intrinsics, i, msg.K[i])
        for i in range(9):
            cv.Set1D(self.R, i, msg.R[i])
        for i in range(12):
            cv.Set1D(self.P, i, msg.P[i])

        self.mapx = cv.CreateImage(self.size, cv.IPL_DEPTH_32F, 1)
        self.mapy = cv.CreateImage(self.size, cv.IPL_DEPTH_32F, 1)
        self.set_alpha(0.0)

    def report(self):
        self.lrreport(self.distortion, self.intrinsics, self.R, self.P)

    def ost(self):
        return self.lrost(self.name, self.distortion, self.intrinsics, self.R, self.P)

    def linear_error_from_image(self, image):
        """
        Detect the checkerboard and compute the linear error.
        Mainly for use in tests.
        """
        _, corners, _, board, _ = self.downsample_and_detect(image)
        if not corners:
            return None

        src = self.mk_image_points([ (corners, board) ])
        undistorted = list(cvmat_iterator(self.undistort_points(src)))
        return self.linear_error(undistorted, board)

    def linear_error(self, corners, b):

        """
        Returns the linear error for a set of corners detected in the unrectified image.
        """

        P = list(corners)
        # P is checkerboard vertices as a list of (x,y) image points

        def pt2line(x0, y0, x1, y1, x2, y2):
            """ point is (x0, y0), line is (x1, y1, x2, y2) """
            return abs((x2 - x1) * (y1 - y0) - (x1 - x0) * (y2 - y1)) / math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

        cc = b.n_cols
        cr = b.n_rows
        errors = []
        for r in range(cr):
            (x1, y1) = P[(cc * r) + 0]
            (x2, y2) = P[(cc * r) + cc - 1]
            for i in range(1, cc - 1):
                (x0, y0) = P[(cc * r) + i]
                errors.append(pt2line(x0, y0, x1, y1, x2, y2))
        return math.sqrt(sum([e**2 for e in errors]) / len(errors))


    def handle_msg(self, msg):
        """
        Detects the calibration target and, if found and provides enough new information,
        adds it to the sample database.

        Returns a MonoDrawable message with the display image and progress info.
        """
        gray = self.mkgray(msg)
        linear_error = -1

        # Get display-image-to-be (scrib) and detection of the calibration target
        scrib_mono, corners, downsampled_corners, board, (x_scale, y_scale) = self.downsample_and_detect(gray)

        # make sure scrib has 3 channels for display
        scrib = cv.CreateMat(scrib_mono.rows, scrib_mono.cols, cv.CV_8UC3)

        if self.calibrated:
            # Show rectified image
            # TODO Pull out downsampling code into function
            gray_remap = self.remap(gray)
            gray_rect = gray_remap
            if x_scale != 1.0 or y_scale != 1.0:
                gray_rect = cv.CreateMat(scrib.rows, scrib.cols, cv.CV_8UC1)
                cv.Resize(gray_remap, gray_rect)

            cv.CvtColor(gray_rect, scrib, cv.CV_GRAY2BGR)

            if corners:
                # Report linear error
                src = self.mk_image_points([ (corners, board) ])
                undistorted = list(cvmat_iterator(self.undistort_points(src)))
                linear_error = self.linear_error(undistorted, board)

                # Draw rectified corners
                scrib_src = [(x/x_scale, y/y_scale) for (x,y) in undistorted]
                cv.DrawChessboardCorners(scrib, (board.n_cols, board.n_rows), scrib_src, True)

        else:
            cv.CvtColor(scrib_mono, scrib, cv.CV_GRAY2BGR)
            if corners:
                # Draw (potentially downsampled) corners onto display image
                src = self.mk_image_points([ (downsampled_corners, board) ])
                cv.DrawChessboardCorners(scrib, (board.n_cols, board.n_rows), cvmat_iterator(src), True)

                # Add sample to database only if it's sufficiently different from any previous sample.
                params = self.get_parameters(corners, board, cv.GetSize(gray))
                if self.is_good_sample(params):
                    self.db.append((params, gray))
                    self.good_corners.append((corners, board))
                    print(("*** Added sample %d, p_x = %.3f, p_y = %.3f, p_size = %.3f, skew = %.3f" % tuple([len(self.db)] + params)))

        rv = MonoDrawable()
        rv.scrib = scrib
        rv.params = self.compute_goodenough()
        rv.linear_error = linear_error
        return rv

    def do_calibration(self, dump = False):
        if not self.good_corners:
            print("**** Collecting corners for all images! ****") #DEBUG
            images = [i for (p, i) in self.db]
            self.good_corners = self.collect_corners(images)
        # Dump should only occur if user wants it
        if dump:
            pickle.dump((self.is_mono, self.size, self.good_corners),
                        open("/tmp/camera_calibration_%08x.pickle" % random.getrandbits(32), "w"))
        self.size = cv.GetSize(self.db[0][1]) # TODO Needs to be set externally
        self.cal_fromcorners(self.good_corners)
        self.calibrated = True
        # DEBUG
        print((self.report()))
        print((self.ost()))

    def do_tarfile_save(self, tf):
        """ Write images and calibration solution to a tarfile object """

        def taradd(name, buf):
            s = StringIO(buf)
            ti = tarfile.TarInfo(name)
            ti.size = len(s.getvalue())
            ti.uname = 'calibrator'
            ti.mtime = int(time.time())
            tf.addfile(tarinfo=ti, fileobj=s)

        ims = [("left-%04d.png" % i, im) for i,(_, im) in enumerate(self.db)]
        for (name, im) in ims:
            taradd(name, cv.EncodeImage(".png", im).tostring())

        taradd('ost.txt', self.ost())

    def do_tarfile_calibration(self, filename):
        archive = tarfile.open(filename, 'r')

        limages = [ image_from_archive(archive, f) for f in archive.getnames() if (f.startswith('left') and (f.endswith('.pgm') or f.endswith('png'))) ]

        self.cal(limages)

# TODO Replicate MonoCalibrator improvements in stereo
class StereoCalibrator(Calibrator):
    """
    Calibration class for stereo cameras::

        limages = [cv.LoadImage("left%d.png") for i in range(8)]
        rimages = [cv.LoadImage("right%d.png") for i in range(8)]
        sc = StereoCalibrator()
        sc.cal(limages, rimages)
        print sc.as_message()
    """

    is_mono = False

    def __init__(self, *args, **kwargs):
        if 'name' not in kwargs:
            kwargs['name'] = 'narrow_stereo'
        super(StereoCalibrator, self).__init__(*args, **kwargs)
        self.l = MonoCalibrator(*args, **kwargs)
        self.r = MonoCalibrator(*args, **kwargs)
        # Collecting from two cameras in a horizontal stereo rig, can't get
        # full X range in the left camera.
        self.param_ranges[0] = 0.4

    def cal(self, limages, rimages):
        """
        :param limages: source left images containing chessboards
        :type limages: list of :class:`cvMat`
        :param rimages: source right images containing chessboards
        :type rimages: list of :class:`cvMat`

        Find chessboards in images, and runs the OpenCV calibration solver.
        """
        goodcorners = self.collect_corners(limages, rimages)
        self.size = cv.GetSize(limages[0])
        self.l.size = self.size
        self.r.size = self.size
        self.cal_fromcorners(goodcorners)
        self.calibrated = True

    def collect_corners(self, limages, rimages):
        """
        For a sequence of left and right images, find pairs of images where both
        left and right have a chessboard, and return  their corners as a list of pairs.
        """
        # Pick out (corners, board) tuples
        lcorners = [ self.downsample_and_detect(i)[1:4:2] for i in limages]
        rcorners = [ self.downsample_and_detect(i)[1:4:2] for i in rimages]
        good = [(lco, rco, b) for ((lco, b), (rco, br)) in zip( lcorners, rcorners) if (lco and rco)]

        if len(good) == 0:
            raise CalibrationException("No corners found in images!")
        return good

    def cal_fromcorners(self, good):
        # Perform monocular calibrations
        lcorners = [(l, b) for (l, r, b) in good]
        rcorners = [(r, b) for (l, r, b) in good]
        self.l.cal_fromcorners(lcorners)
        self.r.cal_fromcorners(rcorners)

        lipts = self.mk_image_points(lcorners)
        ripts = self.mk_image_points(rcorners)
        boards = [ b for (_, _, b) in good]
        
        opts = self.mk_object_points(boards, True)
        npts = self.mk_point_counts(boards)

        flags = cv2.CALIB_FIX_INTRINSIC

        self.T = cv.CreateMat(3, 1, cv.CV_64FC1)
        self.R = cv.CreateMat(3, 3, cv.CV_64FC1)
        cv.SetIdentity(self.T)
        cv.SetIdentity(self.R)
        cv.StereoCalibrate(opts, lipts, ripts, npts,
                           self.l.intrinsics, self.l.distortion,
                           self.r.intrinsics, self.r.distortion,
                           self.size,
                           self.R,                            # R
                           self.T,                            # T
                           cv.CreateMat(3, 3, cv.CV_32FC1),   # E
                           cv.CreateMat(3, 3, cv.CV_32FC1),   # F
                           (cv.CV_TERMCRIT_ITER + cv.CV_TERMCRIT_EPS, 1, 1e-5),
                           flags)

        self.set_alpha(0.0)

    def set_alpha(self, a):
        """
        Set the alpha value for the calibrated camera solution. The
        alpha value is a zoom, and ranges from 0 (zoomed in, all pixels
        in calibrated image are valid) to 1 (zoomed out, all pixels in
        original image are in calibrated image).
        """

        cv.StereoRectify(self.l.intrinsics,
                         self.r.intrinsics,
                         self.l.distortion,
                         self.r.distortion,
                         self.size,
                         self.R,
                         self.T,
                         self.l.R, self.r.R, self.l.P, self.r.P,
                         alpha = a)
        
        cv.InitUndistortRectifyMap(self.l.intrinsics, self.l.distortion, self.l.R, self.l.P,
                                   self.l.mapx, self.l.mapy)
        cv.InitUndistortRectifyMap(self.r.intrinsics, self.r.distortion, self.r.R, self.r.P,
                                   self.r.mapx, self.r.mapy)

    def as_message(self):
        """
        Return the camera calibration as a pair of CameraInfo messages, for left
        and right cameras respectively.
        """

        return (self.lrmsg(self.l.distortion, self.l.intrinsics, self.l.R, self.l.P),
                self.lrmsg(self.r.distortion, self.r.intrinsics, self.r.R, self.r.P))

    def from_message(self, msgs, alpha = 0.0):
        """ Initialize the camera calibration from a pair of CameraInfo messages.  """
        self.size = (msgs[0].width, msgs[0].height)

        self.T = cv.CreateMat(3, 1, cv.CV_64FC1)
        self.R = cv.CreateMat(3, 3, cv.CV_64FC1)
        cv.SetIdentity(self.T)
        cv.SetIdentity(self.R)

        self.l.from_message(msgs[0])
        self.r.from_message(msgs[1])
        # Need to compute self.T and self.R here, using the monocular parameters above
        if False:
            self.set_alpha(0.0)

    def report(self):
        print("\nLeft:")
        self.lrreport(self.l.distortion, self.l.intrinsics, self.l.R, self.l.P)
        print("\nRight:")
        self.lrreport(self.r.distortion, self.r.intrinsics, self.r.R, self.r.P)
        print(("self.T", list(cvmat_iterator(self.T))))
        print(("self.R", list(cvmat_iterator(self.R))))

    def ost(self):
        return (self.lrost(self.name + "/left", self.l.distortion, self.l.intrinsics, self.l.R, self.l.P) +
          self.lrost(self.name + "/right", self.r.distortion, self.r.intrinsics, self.r.R, self.r.P))

    # TODO Get rid of "from_images" versions of these, instead have function to get undistorted corners
    def epipolar_error_from_images(self, limage, rimage):
        """
        Detect the checkerboard in both images and compute the epipolar error.
        Mainly for use in tests.
        """
        _, lcorners, _, board, _ = self.downsample_and_detect(limage)
        _, rcorners, _, board, _ = self.downsample_and_detect(rimage)
        if not lcorners or not rcorners:
            return None

        src = self.mk_image_points( [(lcorners, board)] )
        lundistorted = list(cvmat_iterator(self.l.undistort_points(src)))
        src = self.mk_image_points( [(rcorners, board)] )
        rundistorted = list(cvmat_iterator(self.r.undistort_points(src)))

        return self.epipolar_error(lundistorted, rundistorted, board)

    def epipolar_error(self, lcorners, rcorners, board):
        """
        Compute the epipolar error from two sets of matching undistorted points
        given the current calibration.
        """
        d = [(y0-y1) for ((_, y0), (_, y1)) in zip(lcorners, rcorners)]
        return math.sqrt(sum([i**2 for i in d]) / len(d))

    def chessboard_size_from_images(self, limage, rimage):
        _, lcorners, _, board, _ = self.downsample_and_detect(limage)
        _, rcorners, _, board, _ = self.downsample_and_detect(rimage)
        if not lcorners or not rcorners:
            return None

        src = self.mk_image_points( [(lcorners, board)] )
        lundistorted = list(cvmat_iterator(self.l.undistort_points(src)))
        src = self.mk_image_points( [(rcorners, board)] )
        rundistorted = list(cvmat_iterator(self.r.undistort_points(src)))

        return self.chessboard_size(lundistorted, rundistorted, board)

    def chessboard_size(self, lcorners, rcorners, board):
        """
        Compute the square edge length from two sets of matching undistorted points
        given the current calibration.
        """
        # Project the points to 3d
        cam = image_geometry.StereoCameraModel()
        cam.fromCameraInfo(*self.as_message())
        disparities = [(x0 - x1) for ((x0, y0), (x1, y1)) in zip(lcorners, rcorners)]
        pt3d = [cam.projectPixelTo3d((x, y), d) for ((x, y), d) in zip(lcorners, disparities)]
        def l2(p0, p1):
            return math.sqrt(sum([(c0 - c1) ** 2 for (c0, c1) in zip(p0, p1)]))

        # Compute the length from each horizontal and vertical line, and return the mean
        cc = board.n_cols
        cr = board.n_rows
        lengths = (
            [l2(pt3d[cc * r + 0], pt3d[cc * r + (cc - 1)]) / (cc - 1) for r in range(cr)] +
            [l2(pt3d[c + 0], pt3d[c + (cc * (cr - 1))]) / (cr - 1) for c in range(cc)])
        return sum(lengths) / len(lengths)

    def handle_msg(self, msg):
        # TODO Various asserts that images have same dimension, same board detected...
        (lmsg, rmsg) = msg
        lgray = self.mkgray(lmsg)
        rgray = self.mkgray(rmsg)
        epierror = -1

        # Get display-images-to-be and detections of the calibration target
        lscrib_mono, lcorners, ldownsampled_corners, lboard, (x_scale, y_scale) = self.downsample_and_detect(lgray)
        rscrib_mono, rcorners, rdownsampled_corners, rboard, _ = self.downsample_and_detect(rgray)

        lscrib = cv.CreateMat(lscrib_mono.rows, lscrib_mono.cols, cv.CV_8UC3)
        rscrib = cv.CreateMat(rscrib_mono.rows, rscrib_mono.cols, cv.CV_8UC3)

        if self.calibrated:
            # Show rectified images
            lremap = self.l.remap(lgray)
            rremap = self.r.remap(rgray)
            lrect = lremap
            rrect = rremap
            if x_scale != 1.0 or y_scale != 1.0:
                lrect = cv.CreateMat(lscrib.rows, lscrib.cols, cv.CV_8UC1)
                rrect = cv.CreateMat(rscrib.rows, rscrib.cols, cv.CV_8UC1)
                cv.Resize(lremap, lrect)
                cv.Resize(rremap, rrect)

            cv.CvtColor(lrect, lscrib, cv.CV_GRAY2BGR)
            cv.CvtColor(rrect, rscrib, cv.CV_GRAY2BGR)

            # Draw rectified corners
            if lcorners:
                src = self.mk_image_points( [(lcorners, lboard)] )
                lundistorted = list(cvmat_iterator(self.l.undistort_points(src)))
                scrib_src = [(x/x_scale, y/y_scale) for (x,y) in lundistorted]
                cv.DrawChessboardCorners(lscrib, (lboard.n_cols, lboard.n_rows), scrib_src, True)

            if rcorners:
                src = self.mk_image_points( [(rcorners, rboard)] )
                rundistorted = list(cvmat_iterator(self.r.undistort_points(src)))
                scrib_src = [(x/x_scale, y/y_scale) for (x,y) in rundistorted]
                cv.DrawChessboardCorners(rscrib, (rboard.n_cols, rboard.n_rows), scrib_src, True)

            # Report epipolar error
            if lcorners and rcorners:
                epierror = self.epipolar_error(lundistorted, rundistorted, lboard)

        else:
            cv.CvtColor(lscrib_mono, lscrib, cv.CV_GRAY2BGR)
            cv.CvtColor(rscrib_mono, rscrib, cv.CV_GRAY2BGR)
            # Draw any detected chessboards onto display (downsampled) images
            if lcorners:
                src = self.mk_image_points([ (ldownsampled_corners, lboard) ])
                cv.DrawChessboardCorners(lscrib, (lboard.n_cols, lboard.n_rows),
                                         cvmat_iterator(src), True)
            if rcorners:
                src = self.mk_image_points([ (rdownsampled_corners, rboard) ])
                cv.DrawChessboardCorners(rscrib, (rboard.n_cols, rboard.n_rows),
                                         cvmat_iterator(src), True)

            # Add sample to database only if it's sufficiently different from any previous sample
            if lcorners and rcorners:
                params = self.get_parameters(lcorners, lboard, cv.GetSize(lgray))
                if self.is_good_sample(params):
                    self.db.append( (params, lgray, rgray) )
                    self.good_corners.append( (lcorners, rcorners, lboard) )
                    print(("*** Added sample %d, p_x = %.3f, p_y = %.3f, p_size = %.3f, skew = %.3f" % tuple([len(self.db)] + params)))

        rv = StereoDrawable()
        rv.lscrib = lscrib
        rv.rscrib = rscrib
        rv.params = self.compute_goodenough()
        rv.epierror = epierror
        return rv

    def do_calibration(self, dump = False):
        # TODO MonoCalibrator collects corners if needed here
        # Dump should only occur if user wants it
        if dump:
            pickle.dump((self.is_mono, self.size, self.good_corners),
                        open("/tmp/camera_calibration_%08x.pickle" % random.getrandbits(32), "w"))
        self.size = cv.GetSize(self.db[0][1]) # TODO Needs to be set externally
        self.l.size = self.size
        self.r.size = self.size
        self.cal_fromcorners(self.good_corners)
        self.calibrated = True
        # DEBUG
        print((self.report()))
        print((self.ost()))

    def do_tarfile_save(self, tf):
        """ Write images and calibration solution to a tarfile object """
        ims = ([("left-%04d.png"  % i, im) for i,(_, im, _) in enumerate(self.db)] +
               [("right-%04d.png" % i, im) for i,(_, _, im) in enumerate(self.db)])

        def taradd(name, buf):
            s = StringIO(buf)
            ti = tarfile.TarInfo(name)
            ti.size = len(s.getvalue())
            ti.uname = 'calibrator'
            ti.mtime = int(time.time())
            tf.addfile(tarinfo=ti, fileobj=s)

        for (name, im) in ims:
            taradd(name, cv.EncodeImage(".png", im).tostring())

        taradd('ost.txt', self.ost())

    def do_tarfile_calibration(self, filename):
        archive = tarfile.open(filename, 'r')
        limages = [ image_from_archive(archive, f) for f in archive.getnames() if (f.startswith('left') and (f.endswith('pgm') or f.endswith('png'))) ]
        rimages = [ image_from_archive(archive, f) for f in archive.getnames() if (f.startswith('right') and (f.endswith('pgm') or f.endswith('png'))) ]

        if not len(limages) == len(rimages):
            raise CalibrationException("Left, right images don't match. %d left images, %d right" % (len(limages), len(rimages)))
        
        ##\todo Check that the filenames match and stuff

        self.cal(limages, rimages)
