import math

import cv

import image_geometry
import sensor_msgs.msg
import pickle

class CalibrationException(Exception):
    pass

def cvmat_iterator(cvmat):
    for i in range(cvmat.rows):
        for j in range(cvmat.cols):
            yield cvmat[i,j]

class Calibrator:

    def __init__(self, size, dim):
        self.chessboard_n_cols = size[0]
        self.chessboard_n_rows = size[1]
        self.dim = dim

    def mk_object_points(self, nimages, squaresize = 1):
        num_pts = self.chessboard_n_cols * self.chessboard_n_rows
        opts = cv.CreateMat(nimages * num_pts, 3, cv.CV_32FC1)
        for i in range(nimages):
            for j in range(num_pts):
                opts[i * num_pts + j, 0] = (j / self.chessboard_n_cols) * squaresize
                opts[i * num_pts + j, 1] = (j % self.chessboard_n_cols) * squaresize
                opts[i * num_pts + j, 2] = 0
        return opts

    def mk_image_points(self, good):
        num_pts = self.chessboard_n_cols * self.chessboard_n_rows
        ipts = cv.CreateMat(len(good) * num_pts, 2, cv.CV_32FC1)
        for (i, co) in enumerate(good):
            for j in range(num_pts):
                ipts[i * num_pts + j, 0] = co[j][0]
                ipts[i * num_pts + j, 1] = co[j][1]
        return ipts

    def mk_point_counts(self, nimages):
        num_pts = self.chessboard_n_cols * self.chessboard_n_rows
        npts = cv.CreateMat(nimages, 1, cv.CV_32SC1)
        for i in range(nimages):
            npts[i, 0] = num_pts
        return npts

    def get_corners(self, img, refine = True):
        w, h = cv.GetSize(img)
        mono = cv.CreateMat(h, w, cv.CV_8UC1)
        cv.CvtColor(img, mono, cv.CV_BGR2GRAY)
        (ok, corners) = cv.FindChessboardCorners(mono, (self.chessboard_n_cols, self.chessboard_n_rows), cv.CV_CALIB_CB_ADAPTIVE_THRESH | cv.CV_CALIB_CB_NORMALIZE_IMAGE)

        # If any corners are within BORDER pixels of the screen edge, reject the detection by setting ok to false
        BORDER = 8
        if not all([(BORDER < x < (w - BORDER)) and (BORDER < y < (h - BORDER)) for (x, y) in corners]):
            ok = False

        if refine and ok:
            corners = cv.FindCornerSubPix(mono, corners, (5,5), (-1,-1), ( cv.CV_TERMCRIT_EPS+cv.CV_TERMCRIT_ITER, 30, 0.1 ))
        return (ok, corners)

    def lrmsg(self, d, k, r, p):
        """ Used by :meth:`as_message`.  Return a CameraInfo message for the given calibration matrices """
        msg = sensor_msgs.msg.CameraInfo()
        (msg.width, msg.height) = self.size
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
        goodcorners = self.collect_corners(images)
        self.cal_fromcorners(goodcorners)

    def collect_corners(self, images):
        """
        :param images: source images containing chessboards
        :type images: list of :class:`cvMat`

        Find chessboards in images, and runs the OpenCV calibration solver.
        """
        self.size = cv.GetSize(images[0])
        corners = [self.get_corners(i) for i in images]

        goodcorners = [co for (im, (ok, co)) in zip(images, corners) if ok]
        if len(goodcorners) == 0:
            raise CalibrationException
        return goodcorners

    def cal_fromcorners(self, good):

        ipts = self.mk_image_points(good)
        opts = self.mk_object_points(len(good))
        npts = self.mk_point_counts(len(good))

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

        ncm = cv.CreateMat(3, 3, cv.CV_64FC1)
        R = cv.CreateMat(3, 3, cv.CV_64FC1)
        cv.SetIdentity(R)
        cv.GetOptimalNewCameraMatrix(self.intrinsics, self.distortion, self.size, a, ncm)
        cv.InitUndistortRectifyMap(self.intrinsics, self.distortion, R, ncm, self.mapx, self.mapy)

        self.R = cv.CreateMat(3, 3, cv.CV_64FC1)
        self.P = cv.CreateMat(3, 4, cv.CV_64FC1)
        cv.SetIdentity(self.R)
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
        cv.UndistortPoints(src, dst, self.intrinsics, self.distortion, P = self.intrinsics)
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


    def linear_error(self, im):

        """
        :param im: source image containing chessboard
        :type li: :class:`cvMat`

        Applies current calibration to image, finds the checkerbard, and returns the linear error.
        Returns -1 if checkerboard not found.

        """

        (ok, corners) = self.get_corners(im)
        if not ok:
            return -1
        if 1:
            src = cv.Reshape(self.mk_image_points([corners]), 2)
            P = list(cvmat_iterator(self.undistort_points(src)))
        else:
            src = cv.Reshape(self.mk_image_points([corners]), 2)
            P = list(cvmat_iterator(src))
        # P is checkerboard vertices as a list of (x,y) image points

        def pt2line(x0, y0, x1, y1, x2, y2):
            """ point is (x0, y0), line is (x1, y1, x2, y2) """
            return abs((x2 - x1) * (y1 - y0) - (x1 - x0) * (y2 - y1)) / math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

        cc = self.chessboard_n_cols
        cr = self.chessboard_n_rows
        errors = []
        for r in range(cr):
            (x1, y1) = P[(cc * r) + 0]
            (x2, y2) = P[(cc * r) + cc - 1]
            for i in range(1, cc - 1):
                (x0, y0) = P[(cc * r) + i]
                errors.append(pt2line(x0, y0, x1, y1, x2, y2))
        return math.sqrt(sum([e**2 for e in errors]) / len(errors))

class CalibrationException(Exception):
    pass

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

    def goodpairs(self, limages, rimages):
        """
        For a sequence of left and right images, find pairs of images where both left and right have a chessboard, and return 
        their corners as a list of pairs.
        """
        sz = (self.chessboard_n_cols, self.chessboard_n_rows)
        lcorners = [cv.FindChessboardCorners(i, sz, cv.CV_CALIB_CB_ADAPTIVE_THRESH | cv.CV_CALIB_CB_NORMALIZE_IMAGE) for i in limages]
        rcorners = [cv.FindChessboardCorners(i, sz, cv.CV_CALIB_CB_ADAPTIVE_THRESH | cv.CV_CALIB_CB_NORMALIZE_IMAGE) for i in rimages]
        good = [(lco, rco) for ((lok, lco), (rok, rco)) in zip(lcorners, rcorners) if (lok and rok)]
        return good

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
        self.size = cv.GetSize(limages[0])
        self.l.cal(limages)
        self.r.cal(rimages)

        sz = (self.chessboard_n_cols, self.chessboard_n_rows)
        lcorners = [cv.FindChessboardCorners(i, sz, cv.CV_CALIB_CB_ADAPTIVE_THRESH | cv.CV_CALIB_CB_NORMALIZE_IMAGE) for i in limages]
        rcorners = [cv.FindChessboardCorners(i, sz, cv.CV_CALIB_CB_ADAPTIVE_THRESH | cv.CV_CALIB_CB_NORMALIZE_IMAGE) for i in rimages]
        good = [(lco, rco) for ((lok, lco), (rok, rco)) in zip( lcorners, rcorners) if (lok and rok)]

        if len(good) == 0:
            raise CalibrationException
        return good

    def cal_fromcorners(self, good):
        lipts = self.mk_image_points([l for (l, r) in good])
        ripts = self.mk_image_points([r for (l, r) in good])
        opts = self.mk_object_points(len(good), self.dim)
        npts = self.mk_point_counts(len(good))

        flags = cv.CV_CALIB_FIX_ASPECT_RATIO | cv.CV_CALIB_FIX_INTRINSIC

        self.T = cv.CreateMat(3, 1, cv.CV_64FC1)
        self.R = cv.CreateMat(3, 3, cv.CV_64FC1)
        cv.SetIdentity(self.T)
        cv.SetIdentity(self.R)
        cv.StereoCalibrate(opts, lipts, ripts, npts,
                           self.l.intrinsics, self.l.distortion,
                           self.r.intrinsics, self.r.distortion,
                           self.size,
                           self.R,                                  # R
                           self.T,                                  # T
                           cv.CreateMat(3, 3, cv.CV_32FC1),    # E
                           cv.CreateMat(3, 3, cv.CV_32FC1),    # F
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
        
        self.lmapx = cv.CreateImage((640, 480), cv.IPL_DEPTH_32F, 1)
        self.lmapy = cv.CreateImage((640, 480), cv.IPL_DEPTH_32F, 1)
        self.rmapx = cv.CreateImage((640, 480), cv.IPL_DEPTH_32F, 1)
        self.rmapy = cv.CreateImage((640, 480), cv.IPL_DEPTH_32F, 1)
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

        (ok, corners) = self.get_corners(li)
        if not ok:
            return -1
        src = cv.Reshape(self.mk_image_points([corners]), 2)
        L = list(cvmat_iterator(self.lundistort_points(src)))
        (ok, corners) = self.get_corners(ri)
        if not ok:
            return -1
        src = cv.Reshape(self.mk_image_points([corners]), 2)
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
        (ok, corners) = self.get_corners(li)
        if not ok:
            return -1
        src = cv.Reshape(self.mk_image_points([corners]), 2)
        L = list(cvmat_iterator(self.lundistort_points(src)))
        (ok, corners) = self.get_corners(ri)
        if not ok:
            return -1
        src = cv.Reshape(self.mk_image_points([corners]), 2)
        R = list(cvmat_iterator(self.rundistort_points(src)))

        # Project the points to 3d
        cam = image_geometry.StereoCameraModel()
        cam.fromCameraInfo(*self.as_message())
        disparities = [(x0 - x1) for ((x0, y0), (x1, y1)) in zip(L, R)]
        pt3d = [cam.projectPixelTo3d((x, y), d) for ((x, y), d) in zip(L, disparities)]
        def l2(p0, p1):
            return math.sqrt(sum([(c0 - c1) ** 2 for (c0, c1) in zip(p0, p1)]))

        # Compute the length from each horizontal and vertical line, and return the mean
        cc = self.chessboard_n_cols
        cr = self.chessboard_n_rows
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
