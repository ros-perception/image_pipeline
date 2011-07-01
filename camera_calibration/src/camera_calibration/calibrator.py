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

import math
import operator

import cv
import cv2
import cv_bridge
import numpy
import numpy.linalg

import image_geometry
import sensor_msgs.msg
import pickle
import tarfile
import StringIO
import time

class CalibrationException(Exception):
    pass

def cvmat_iterator(cvmat):
    for i in range(cvmat.rows):
        for j in range(cvmat.cols):
            yield cvmat[i,j]

class ChessboardInfo(object):
    def __init__(self, n_cols = 0, n_rows = 0, dim = 0.0):
        self.n_cols = n_cols
        self.n_rows = n_rows
        self.dim = dim

def _get_num_pts(good):
    num_pts = 0
    for (i, corners) in enumerate(good):
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
    mono = cv.CreateMat(h, w, cv.CV_8UC1)
    cv.CvtColor(img, mono, cv.CV_BGR2GRAY)
    (ok, corners) = cv.FindChessboardCorners(mono, (board.n_cols, board.n_rows), cv.CV_CALIB_CB_ADAPTIVE_THRESH | cv.CV_CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK)
    
    # If any corners are within BORDER pixels of the screen edge, reject the detection by setting ok to false
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
        corners = cv.FindCornerSubPix(mono, corners, (radius,radius), (-1,-1), ( cv.CV_TERMCRIT_EPS+cv.CV_TERMCRIT_ITER, 30, 0.1 ))
        
    return (ok, corners)


def _get_total_num_pts(boards):
    rv = 0
    for b in boards:
        rv += b.n_cols * b.n_rows
    return rv


# TODO self.size needs to come from CameraInfo, full resolution
class Calibrator:
    """
    Base class for calibration system
    """
    def __init__(self, boards):
        # Make sure n_cols > n_rows to agree with OpenCV CB detector output
        self._boards = [ChessboardInfo(max(i.n_cols, i.n_rows), min(i.n_cols, i.n_rows), i.dim) for i in boards]
        self.calibrated = False
        self.br = cv_bridge.CvBridge()
        self.db = []
        self.good_corners = []
        self.goodenough = False

    def mkgray(self, msg):
        """
        Convert a message into a bgr8 OpenCV bgr8 *monochrome* image.
        Deal with bayer images by converting to color, then to monochrome.
        """
        if 'bayer' in msg.encoding:
            converter = {
                "bayer_rggb8" : cv.CV_BayerBG2BGR,
                "bayer_bggr8" : cv.CV_BayerRG2BGR,
                "bayer_gbrg8" : cv.CV_BayerGR2BGR,
                "bayer_grbg8" : cv.CV_BayerGB2BGR }[msg.encoding]
            msg.encoding = "mono8"
            raw = self.br.imgmsg_to_cv(msg)
            rgb = cv.CreateMat(raw.rows, raw.cols, cv.CV_8UC3)
            mono = cv.CreateMat(raw.rows, raw.cols, cv.CV_8UC1)

            cv.CvtColor(raw, rgb, converter)
            cv.CvtColor(rgb, mono, cv.CV_BGR2GRAY)
            cv.CvtColor(mono, rgb, cv.CV_GRAY2BGR)
        else:
            rgb = self.br.imgmsg_to_cv(msg, "bgr8")

        return rgb

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
        return d > 0.3

    _param_names = ["X", "Y", "Size", "Skew"]
    _param_ranges = [0.6, 0.6, 0.4, 0.6]

    def compute_goodenough(self):
        if not self.db:
            return None

        # Find range of checkerboard poses covered by samples in database
        all_params = [sample[0] for sample in self.db]
        min_params = reduce(lmin, all_params)
        max_params = reduce(lmax, all_params)
        # Don't reward small size or skew
        min_params = [min_params[0], min_params[1], 0., 0.]

        # For each parameter, judge how much progress has been made toward adequate variation
        progress = [min((hi - lo) / r, 1.0) for (lo, hi, r) in zip(min_params, max_params, self._param_ranges)]
        # TODO Awkward that we update self.goodenough instead of returning it
        self.goodenough = all([p == 1.0 for p in progress])

        return zip(self._param_names, min_params, max_params, progress)

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

        Returns ok 
        """

        for b in self._boards:
            (ok, corners) = _get_corners(img, b, refine)
            if ok:
                return (ok, corners, b)
        return (False, None, None)


    def lrmsg(self, d, k, r, p):
        """ Used by :meth:`as_message`.  Return a CameraInfo message for the given calibration matrices """
        msg = sensor_msgs.msg.CameraInfo()
        (msg.width, msg.height) = self.size
        msg.distortion_model = "plumb_bob"
        msg.D = [d[i,0] for i in range(d.rows)]
        while len(msg.D)<5:
	        msg.D.append(0)
        msg.K = list(cvmat_iterator(k))
        msg.R = list(cvmat_iterator(r))
        msg.P = list(cvmat_iterator(p))
        return msg

    def lrreport(self, d, k, r, p):
        print "D = ", list(cvmat_iterator(d))
        print "K = ", list(cvmat_iterator(k))
        print "R = ", list(cvmat_iterator(r))
        print "P = ", list(cvmat_iterator(p))

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
        + "[narrow_stereo/%s]" % name + "\n"
        + "\n"
        + "camera matrix\n"
        + " ".join(["%8f" % k[0,i] for i in range(3)]) + "\n"
        + " ".join(["%8f" % k[1,i] for i in range(3)]) + "\n"
        + " ".join(["%8f" % k[2,i] for i in range(3)]) + "\n"
        + "\n"
        + "distortion\n"
        + " ".join(["%8f" % d[i,0] for i in range(4)]) + " 0.0000\n"
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
        print "Wrote calibration data to", filename
        


    def set_scale(self, a):
        if self.calibrated:
            self.set_alpha(a)

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

    is_mono = True

    def cal(self, images):
        """
        Calibrate camera from given images
        """
        goodcorners = self.collect_corners(images)
        self.cal_fromcorners(goodcorners)

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
        distortion = cv.CreateMat(4, 1, cv.CV_64FC1)
        cv.SetZero(intrinsics)
        cv.SetZero(distortion)
        # focal lengths have 1/1 ratio
        intrinsics[0,0] = 1.0
        intrinsics[1,1] = 1.0
        cv.CalibrateCamera2(opts, ipts, npts,
                   self.size, intrinsics,
                   distortion,
                   cv.CreateMat(len(good), 3, cv.CV_32FC1),
                   cv.CreateMat(len(good), 3, cv.CV_32FC1),
                   flags = 0) # cv.CV_CALIB_ZERO_TANGENT_DIST)
        self.intrinsics = intrinsics
        self.distortion = distortion

        # R is identity matrix for monocular calibration
        self.R = cv.CreateMat(3, 3, cv.CV_64FC1)
        cv.SetIdentity(self.R)
        self.P = cv.CreateMat(3, 4, cv.CV_64FC1)

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

        if 0:
            ncm = cv.CreateMat(3, 3, cv.CV_64FC1)
            cv.GetOptimalNewCameraMatrix(self.intrinsics, self.distortion, self.size, a, ncm)
            cv.InitUndistortRectifyMap(self.intrinsics, self.distortion, self.R, ncm, self.mapx, self.mapy)

            cv.SetZero(self.P)
            cv.Copy(ncm, cv.GetSubRect(self.P, (0, 0, 3, 3)))
        else:
            cv.InitUndistortRectifyMap(self.intrinsics, self.distortion, self.R, self.intrinsics, self.mapx, self.mapy)
            cv.SetZero(self.P)
            cv.Copy(self.intrinsics, cv.GetSubRect(self.P, (0, 0, 3, 3)))

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
        self.distortion = cv.CreateMat(4, 1, cv.CV_64FC1)
        self.R = cv.CreateMat(3, 3, cv.CV_64FC1)
        self.P = cv.CreateMat(3, 4, cv.CV_64FC1)
        for i in range(4):
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
        return self.lrost("left", self.distortion, self.intrinsics, self.R, self.P)


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
        Returns a MonoDrawable message with image data
        """
        rgb = self.mkgray(msg)
        # TODO width, height should be local vars
        (self.width, self.height) = cv.GetSize(rgb)
        scrib = rgb

        # Checkerboard detection is too expensive on a large image, so scale it down to ~VGA size
        scale = math.sqrt( (self.width*self.height) / (640.*480.) )
        if scale > 1.0:
            scrib = cv.CreateMat(int(self.height / scale), int(self.width / scale), cv.GetElemType(rgb))
            cv.Resize(rgb, scrib)
        else:
            scrib = cv.CloneMat(rgb)
        # Due to rounding, actual horizontal/vertical scaling may differ slightly
        x_scale = float(self.width) / scrib.cols
        y_scale = float(self.height) / scrib.rows

        # Detect checkerboard
        (ok, downsampled_corners, b) = self.get_corners(scrib, refine = True)
        found_checkerboard = ok and downsampled_corners # TODO Should just need ok?
        linear_error = -1

        # Scale corners back to full size image
        corners = None
        corners_unrefined = None
        if found_checkerboard:
            corners = [(c[0]*x_scale, c[1]*y_scale) for c in downsampled_corners]
            #corners_unrefined = [(c[0]*x_scale, c[1]*y_scale) for c in downsampled_corners]
            # TODO Put this logic in _get_corners
            #mono = cv.CreateMat(rgb.rows, rgb.cols, cv.CV_8UC1)
            #cv.CvtColor(rgb, mono, cv.CV_BGR2GRAY)
            #radius = int(math.ceil(scale))
            #corners = cv.FindCornerSubPix(mono, corners_unrefined, (radius,radius), (-1,-1), ( cv.CV_TERMCRIT_EPS+cv.CV_TERMCRIT_ITER, 30, 0.1 ))
            #corners = cv.FindCornerSubPix(mono, corners_unrefined, (radius,radius), (-1,-1), (cv.CV_TERMCRIT_ITER, 30, 0))

        if self.calibrated:
            # Show rectified image
            # TODO Pull out downsampling code into function
            if scale > 1.0:
                rgb_rect = self.remap(rgb)
                cv.Resize(rgb_rect, scrib)
            else:
                scrib = self.remap(rgb)

            if found_checkerboard:
                # Draw rectified corners and report linear error
                src = self.mk_image_points([ (corners, b) ])
                undistorted = list(cvmat_iterator(self.undistort_points(src)))
                linear_error = self.linear_error(undistorted, b)

                scrib_src = [(x/x_scale, y/y_scale) for (x,y) in undistorted]
                cv.DrawChessboardCorners(scrib, (b.n_cols, b.n_rows), scrib_src, True)

        elif found_checkerboard:
            # Draw (potentially downsampled) corners onto display image
            src = self.mk_image_points([ (downsampled_corners, b) ])
            cv.DrawChessboardCorners(scrib, (b.n_cols, b.n_rows), cvmat_iterator(src), True)

            # Compute some parameters for this chessboard
            Xs = [x for (x, y) in corners]
            Ys = [y for (x, y) in corners]
            # TODO Mean has nice invariants but penalizes using large/close-up checkerboard views. Is there a better way?
            p_x = mean(Xs) / self.width
            p_y = mean(Ys) / self.height
            p_size = math.sqrt(_get_area(corners, b) / (self.width * self.height))
            skew = _get_skew(corners, b)
            params = [p_x, p_y, p_size, skew]

            # Add sample to database only if it's sufficiently different from any previous sample.
            if self.is_good_sample(params):
                # DEBUG Save CB image also
                scrib2 = None
                if 1:
                    src = self.mk_image_points([ (corners, b) ])
                    scrib2 = cv.CloneMat(rgb)
                    cv.DrawChessboardCorners(scrib2, (b.n_cols, b.n_rows), cvmat_iterator(src), True)
                self.db.append((params, rgb, scrib2))
                self.good_corners.append((corners, b))
                print "*** Added sample %d, p_x = %.3f, p_y = %.3f, p_size = %.3f, skew = %.3f" % tuple([len(self.db)] + params)

        rv = MonoDrawable()
        rv.scrib = scrib
        rv.params = self.compute_goodenough()
        rv.linear_error = linear_error
        return rv

    def do_calibration(self, dump = False):
        if not self.good_corners:
            print "**** Collecting corners for all images! ****" #DEBUG
            images = [i for (p, i, _) in self.db]
            self.good_corners = self.collect_corners(images)
        # Dump should only occur if user wants it
        if dump:
            pickle.dump((self.is_mono, self.size, self.good_corners),
                        open("/tmp/camera_calibration_%08x.pickle" % random.getrandbits(32), "w"))
        self.size = cv.GetSize(self.db[0][1]) # TODO Needs to be set externally
        self.cal_fromcorners(self.good_corners)
        self.calibrated = True
        print self.ost() # DEBUG

    def do_tarfile_save(self, tf):
        """ Write images and calibration solution to a tarfile object """

        def taradd(name, buf):
            s = StringIO.StringIO(buf)
            ti = tarfile.TarInfo(name)
            ti.size = len(s.buf)
            ti.uname = 'calibrator'
            ti.mtime = int(time.time())
            tf.addfile(tarinfo=ti, fileobj=s)

        ims = [("left-%04d.png" % i, im) for i,(_, im, _) in enumerate(self.db)]
        for (name, im) in ims:
            taradd(name, cv.EncodeImage(".png", im).tostring())

        # DEBUG
        if 1:
            scribs = [("scrib-%04d.png" % i, scrib) for i,(_, _, scrib) in enumerate(self.db)]
            for (name, scrib) in scribs:
                taradd(name, cv.EncodeImage(".png", scrib).tostring())

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
    def __init__(self, *args):
        self.l = MonoCalibrator(*args)
        self.r = MonoCalibrator(*args)
        Calibrator.__init__(self, *args)

    def cal(self, limages, rimages):
        """
        :param limages: source left images containing chessboards
        :type limages: list of :class:`cvMat`
        :param rimages: source right images containing chessboards
        :type rimages: list of :class:`cvMat`

        Find chessboards in images, and runs the OpenCV calibration solver.
        """
        goodcorners = self.collect_corners(limages, rimages)
        self.cal_fromcorners(goodcorners)

    def collect_corners(self, limages, rimages):
        """
        For a sequence of left and right images, find pairs of images where both left and right have a chessboard, and return 
        their corners as a list of pairs.
        """
        self.size = cv.GetSize(limages[0])
        self.l.cal(limages)
        self.r.cal(rimages)

        lcorners = [ self.get_corners(i) for i in limages]
        rcorners = [ self.get_corners(i) for i in rimages]
        good = [(lco, rco, b) for ((lok, lco, b), (rok, rco, br)) in zip( lcorners, rcorners) if (lok and rok)]

        if len(good) == 0:
            raise CalibrationException("No corners found in images!")
        return good

    def cal_fromcorners(self, good):
        lipts = self.mk_image_points([(l, b) for (l, r, b) in good])
        ripts = self.mk_image_points([(r, b) for (l, r, b) in good])
        boards = [ b for (_, _, b) in good]
        
        opts = self.mk_object_points(boards, True)
        npts = self.mk_point_counts(boards)

        flags = cv.CV_CALIB_FIX_ASPECT_RATIO | cv.CV_CALIB_FIX_INTRINSIC

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
        self.lR = cv.CreateMat(3, 3, cv.CV_64FC1)
        self.rR = cv.CreateMat(3, 3, cv.CV_64FC1)
        self.lP = cv.CreateMat(3, 4, cv.CV_64FC1)
        self.rP = cv.CreateMat(3, 4, cv.CV_64FC1)
        for m in [self.lR, self.rR, self.lP, self.rP]:
            cv.SetZero(m)
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
                         self.lR, self.rR, self.lP, self.rP,
                         alpha = a)
        
        self.lmapx = cv.CreateImage(self.size, cv.IPL_DEPTH_32F, 1)
        self.lmapy = cv.CreateImage(self.size, cv.IPL_DEPTH_32F, 1)
        self.rmapx = cv.CreateImage(self.size, cv.IPL_DEPTH_32F, 1)
        self.rmapy = cv.CreateImage(self.size, cv.IPL_DEPTH_32F, 1)
        cv.InitUndistortRectifyMap(self.l.intrinsics, self.l.distortion, self.lR, self.lP, self.lmapx, self.lmapy)
        cv.InitUndistortRectifyMap(self.r.intrinsics, self.r.distortion, self.rR, self.rP, self.rmapx, self.rmapy)

    def as_message(self):
        """
        Return the camera calibration as a pair of CameraInfo messages, for left and right cameras respectively.
        """

        return (self.lrmsg(self.l.distortion, self.l.intrinsics, self.lR, self.lP),
                self.lrmsg(self.r.distortion, self.r.intrinsics, self.rR, self.rP))

    def from_message(self, msgs, alpha = 0.0):
        """ Initialize the camera calibration from a pair of CameraInfo messages.  """
        self.size = (msgs[0].width, msgs[0].height)

        self.T = cv.CreateMat(3, 1, cv.CV_64FC1)
        self.R = cv.CreateMat(3, 3, cv.CV_64FC1)
        cv.SetIdentity(self.T)
        cv.SetIdentity(self.R)

        self.l.from_message(msgs[0])
        self.r.from_message(msgs[1])
        self.lR = self.l.R
        self.lP = self.l.P
        self.rR = self.r.R
        self.rP = self.r.P
        # Need to compute self.T and self.R here, using the monocular parameters above
        if False:
            self.set_alpha(0.0)

    def report(self):
        print "\nLeft:"
        self.lrreport(self.l.distortion, self.l.intrinsics, self.lR, self.lP)
        print "\nRight:"
        self.lrreport(self.r.distortion, self.r.intrinsics, self.rR, self.rP)
        print "self.T", list(cvmat_iterator(self.T))
        print "self.R", list(cvmat_iterator(self.R))
        print "self.l.R = ", list(cvmat_iterator(self.l.R))
        print "self.r.R = ", list(cvmat_iterator(self.r.R))
        print "self.l.P = ", list(cvmat_iterator(self.l.P))
        print "self.r.P = ", list(cvmat_iterator(self.r.P))

    def ost(self):
        return (self.lrost("left", self.l.distortion, self.l.intrinsics, self.lR, self.lP) +
          self.lrost("right", self.r.distortion, self.r.intrinsics, self.rR, self.rP))

    def epipolar1(self, li, ri):
        """
        :param li: source left image containing chessboard
        :type li: :class:`cvMat`
        :param ri: source left image containing chessboard
        :type ri: :class:`cvMat`

        Applies current calibration to stereo pair (li, ri), finds the checkerbard, and returns their epipolar error.
        Returns -1 if checkerboard not found.

        """

        (ok, corners, b) = self.get_corners(li)
        if not ok:
            return -1
        src = self.mk_image_points([(corners, b)])
        L = list(cvmat_iterator(self.lundistort_points(src)))
        (ok, corners, b) = self.get_corners(ri)
        if not ok:
            return -1
        src = self.mk_image_points([(corners, b)])
        R = list(cvmat_iterator(self.rundistort_points(src)))

        # print 'diffs', [abs(y0-y1) for ((_, y0), (_, y1)) in zip(L, R)]
        d = [(y0-y1) for ((_, y0), (_, y1)) in zip(L, R)]
        return math.sqrt(sum([i**2 for i in d]) / len(d))

    def chessboard_size(self, li, ri):
        """
        :param li: source left image containing chessboard
        :type li: :class:`cvMat`
        :param ri: source left image containing chessboard
        :type ri: :class:`cvMat`

        Post calibration, return the square edge length, in meters, or -1 if not possible.
        """
        (ok, corners, b) = self.get_corners(li)
        if not ok:
            return -1
        src = self.mk_image_points([(corners, b)])
        L = list(cvmat_iterator(self.lundistort_points(src)))
        (ok, corners, b) = self.get_corners(ri)
        if not ok:
            return -1
        src = self.mk_image_points([(corners, b)])
        R = list(cvmat_iterator(self.rundistort_points(src)))

        # Project the points to 3d
        cam = image_geometry.StereoCameraModel()
        cam.fromCameraInfo(*self.as_message())
        disparities = [(x0 - x1) for ((x0, y0), (x1, y1)) in zip(L, R)]
        pt3d = [cam.projectPixelTo3d((x, y), d) for ((x, y), d) in zip(L, disparities)]
        def l2(p0, p1):
            return math.sqrt(sum([(c0 - c1) ** 2 for (c0, c1) in zip(p0, p1)]))

        # Compute the length from each horizontal and vertical line, and return the mean
        cc = b.n_cols
        cr = b.n_rows
        lengths = (
            [l2(pt3d[cc * r + 0], pt3d[cc * r + (cc - 1)]) / (cc - 1) for r in range(cr)] +
            [l2(pt3d[c + 0], pt3d[c + (cc * (cr - 1))]) / (cr - 1) for c in range(cc)])
        return sum(lengths) / len(lengths)
        
    def lremap(self, src):
        r = cv.CloneMat(src)
        cv.Remap(src, r, self.lmapx, self.lmapy)
        return r

    def rremap(self, src):
        r = cv.CloneMat(src)
        cv.Remap(src, r, self.rmapx, self.rmapy)
        return r

    def lundistort_points(self, src):
        dst = cv.CloneMat(src)
        cv.UndistortPoints(src, dst, self.l.intrinsics, self.l.distortion, self.lR, self.lP)
        return dst

    def rundistort_points(self, src):
        dst = cv.CloneMat(src)
        cv.UndistortPoints(src, dst, self.r.intrinsics, self.r.distortion, self.rR, self.rP)
        return dst

    def handle_msg(self, msg):
        rv = StereoDrawable()

        (lmsg, rmsg) = msg
        lrgb = self.mkgray(lmsg)
        rrgb = self.mkgray(rmsg)
        (self.width, self.height) = cv.GetSize(lrgb)
        lscrib = lrgb
        rscrib = rrgb

        if self.calibrated:
            epierror = self.epipolar1(lrgb, rrgb)
            if epierror == -1:
                print "Cannot find checkerboard"
            else:
                print "epipolar error:", epierror
            lscrib = self.lremap(lrgb)
            rscrib = self.rremap(rrgb)
        else:
            lscrib = cv.CloneMat(lrgb)
            rscrib = cv.CloneMat(rrgb)
        self.displaywidth = lscrib.cols + rscrib.cols

        rv.lscrib = lscrib
        rv.rscrib = rscrib

        rv.load_params(self.db)
        (lok, lcorners, b) = self.get_corners(lrgb, refine = True)
        if not lok:
            return rv
        (rok, rcorners, b) = self.get_corners(rrgb, refine = True)
        if not rok:
            return rv

        # Compute some parameters for this chessboard
        Xs = [x for (x, y) in lcorners]
        Ys = [y for (x, y) in lcorners]
        p_x = mean(Xs) / self.width
        p_y = mean(Ys) / self.height
        p_size = (max(Xs) - min(Xs)) / self.width
        skew = _get_skew(lcorners, b)
        params = [p_x, p_y, p_size, skew]
        if self.p_mins == None:
            self.p_mins = params
        else:
            self.p_mins = lmin(self.p_mins, params)
        if self.p_maxs == None:
            self.p_maxs = params
        else:
            self.p_maxs = lmax(self.p_maxs, params)
        is_min = [(abs(p - m) < .1) for (p, m) in zip(params, self.p_mins)]
        is_max = [(abs(p - m) < .1) for (p, m) in zip(params, self.p_maxs)]
        
        for (co, im, udm) in [(lcorners, lscrib, self.lundistort_points), (rcorners, rscrib, self.rundistort_points)]:
            src = self.mk_image_points([(co, b)])
            if self.calibrated:
                src = udm(src)
            cv.DrawChessboardCorners(im, (b.n_cols, b.n_rows), cvmat_iterator(src), True)

        # If the image is a min or max in any parameter, add to the collection
        if any(is_min) or any(is_max):
            self.db[str(is_min + is_max)] = (params, lrgb, rrgb)

        self.compute_goodenough()

        rv.load_params(self.db)
        return rv

    def do_calibration(self, dump = False):
        self.calibrated = True
        vv = list(self.db.values())
        limages = [ l for (p, l, r) in vv ]
        rimages = [ r for (p, l, r) in vv ]
        corners = self.collect_corners(limages, rimages)
        # Dump should only occur if user wants it
        if dump:
            pickle.dump((self.is_mono, self.size, corners), open("/tmp/camera_calibration_%08x.pickle" % random.getrandbits(32), "w"))
        self.cal_fromcorners(corners)

    def do_tarfile_save(self, tf):
        """ Write images and calibration solution to a tarfile object """
        vv = list(self.db.values())
        # vv is a list of pairs (p, i) for monocular, and triples (p, l, r) for stereo
        ims = ([("left-%04d.png"  % i, im) for i,(_, im, _) in enumerate(vv)] +
               [("right-%04d.png" % i, im) for i,(_, _, im) in enumerate(vv)])

        def taradd(name, buf):
            s = StringIO.StringIO(buf)
            ti = tarfile.TarInfo(name)
            ti.size = len(s.buf)
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
