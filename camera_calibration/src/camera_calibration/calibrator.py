import math

import cv

num_x_ints = 8
num_y_ints = 6
num_pts = num_x_ints * num_y_ints

import image_geometry
import sensor_msgs.msg

def mk_object_points(nimages, squaresize = 1):
    opts = cv.CreateMat(nimages * num_pts, 3, cv.CV_32FC1)
    for i in range(nimages):
        for j in range(num_pts):
            opts[i * num_pts + j, 0] = (j / num_x_ints) * squaresize
            opts[i * num_pts + j, 1] = (j % num_x_ints) * squaresize
            opts[i * num_pts + j, 2] = 0
    return opts

def mk_image_points(good):
    ipts = cv.CreateMat(len(good) * num_pts, 2, cv.CV_32FC1)
    for (i, co) in enumerate(good):
        for j in range(num_pts):
            ipts[i * num_pts + j, 0] = co[j][0]
            ipts[i * num_pts + j, 1] = co[j][1]
    return ipts

def mk_point_counts(nimages):
    npts = cv.CreateMat(nimages, 1, cv.CV_32SC1)
    for i in range(nimages):
        npts[i, 0] = num_pts
    return npts

def get_corners(img, refine = True):
    w, h = cv.GetSize(img)
    mono = cv.CreateMat(h, w, cv.CV_8UC1)
    cv.CvtColor(img, mono, cv.CV_BGR2GRAY)
    (ok, corners) = cv.FindChessboardCorners(mono, (num_x_ints, num_y_ints), cv.CV_CALIB_CB_ADAPTIVE_THRESH | cv.CV_CALIB_CB_NORMALIZE_IMAGE)
    if refine and ok:
        corners = cv.FindCornerSubPix(mono, corners, (5,5), (-1,-1), ( cv.CV_TERMCRIT_EPS+cv.CV_TERMCRIT_ITER, 30, 0.1 ))
    return (ok, corners)

def cvmat_iterator(cvmat):
    for i in range(cvmat.rows):
        for j in range(cvmat.cols):
            yield cvmat[i,j]

class Calibrator:
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
        print calmessage

class MonoCalibrator(Calibrator):

    is_mono = True
    def __init__(self):
        pass

    def cal(self, images):
        self.size = cv.GetSize(images[0])
        corners = [get_corners(i) for i in images]

        good = [co for (im, (ok, co)) in zip(images, corners) if ok]

        ipts = mk_image_points(good)
        opts = mk_object_points(len(good))
        npts = mk_point_counts(len(good))

        intrinsics = cv.CreateMat(3, 3, cv.CV_64FC1)
        distortion = cv.CreateMat(4, 1, cv.CV_64FC1)
        cv.SetZero(intrinsics)
        cv.SetZero(distortion)
        # focal lengths have 1/1 ratio
        intrinsics[0,0] = 1.0
        intrinsics[1,1] = 1.0
        cv.CalibrateCamera2(opts, ipts, npts,
                   cv.GetSize(images[0]), intrinsics,
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
        ncm = cv.CreateMat(3, 3, cv.CV_64FC1)
        R = cv.CreateMat(3, 3, cv.CV_64FC1)
        cv.SetIdentity(R)
        cv.GetOptimalNewCameraMatrix(self.intrinsics, self.distortion, self.size, a, ncm)
        cv.InitUndistortRectifyMap(self.intrinsics, self.distortion, R, ncm, self.mapx, self.mapy)

        self.R = cv.CreateMat(3, 3, cv.CV_64FC1)
        self.P = cv.CreateMat(3, 4, cv.CV_64FC1)
        cv.SetIdentity(self.R)
        cv.SetIdentity(self.P)

    def remap(self, src):
        r = cv.CloneMat(src)
        cv.Remap(src, r, self.mapx, self.mapy)
        return r

    def undistort_points(self, src):
        dst = cv.CloneMat(src)
        cv.UndistortPoints(src, dst, self.intrinsics, self.distortion, P = self.intrinsics)
        return dst

    def as_message(self):
        return self.lrmsg(self.distortion, self.intrinsics, self.R, self.P)

    def report(self):
        self.lrreport(self.distortion, self.intrinsics, self.R, self.P)

    def ost(self):
        self.lrost("left", self.distortion, self.intrinsics, self.R, self.P)

class CalibrationException(Exception):
    pass

class StereoCalibrator(Calibrator):

    is_mono = False
    def __init__(self):
        self.l = MonoCalibrator()
        self.r = MonoCalibrator()

    def goodpairs(self, limages, rimages):
        """
        For a sequence of left and right images, find pairs of images where both left and right have a chessboard, and return 
        their corners as a list of pairs.
        """
        lcorners = [cv.FindChessboardCorners(i, (8, 6), cv.CV_CALIB_CB_ADAPTIVE_THRESH | cv.CV_CALIB_CB_NORMALIZE_IMAGE) for i in limages]
        rcorners = [cv.FindChessboardCorners(i, (8, 6), cv.CV_CALIB_CB_ADAPTIVE_THRESH | cv.CV_CALIB_CB_NORMALIZE_IMAGE) for i in rimages]
        good = [(lco, rco) for ((lok, lco), (rok, rco)) in zip(lcorners, rcorners) if (lok and rok)]
        return good

    def cal(self, limages, rimages):
        self.size = cv.GetSize(limages[0])
        self.l.cal(limages)
        self.r.cal(rimages)

        lcorners = [cv.FindChessboardCorners(i, (8, 6), cv.CV_CALIB_CB_ADAPTIVE_THRESH | cv.CV_CALIB_CB_NORMALIZE_IMAGE) for i in limages]
        rcorners = [cv.FindChessboardCorners(i, (8, 6), cv.CV_CALIB_CB_ADAPTIVE_THRESH | cv.CV_CALIB_CB_NORMALIZE_IMAGE) for i in rimages]
        good = [(lco, rco) for ((lok, lco), (rok, rco)) in zip( lcorners, rcorners) if (lok and rok)]

        lipts = mk_image_points([l for (l, r) in good])
        ripts = mk_image_points([r for (l, r) in good])
        opts = mk_object_points(len(good), .108)
        npts = mk_point_counts(len(good))

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
        return (self.lrmsg(self.l.distortion, self.l.intrinsics, self.lR, self.lP),
                self.lrmsg(self.r.distortion, self.r.intrinsics, self.rR, self.rP))

    def report(self):
        print "\nLeft:"
        self.lrreport(self.l.distortion, self.l.intrinsics, self.lR, self.lP)
        print "\nRight:"
        self.lrreport(self.r.distortion, self.r.intrinsics, self.rR, self.rP)

    def ost(self):
        self.lrost("left", self.l.distortion, self.l.intrinsics, self.lR, self.lP)
        self.lrost("right", self.r.distortion, self.r.intrinsics, self.rR, self.rP)


    def epipolar1(self, li, ri):
        # Find corners in both images and undistort them, results in lists L,R

        (ok, corners) = get_corners(li)
        if not ok:
            return -1
        src = cv.Reshape(mk_image_points([corners]), 2)
        L = list(cvmat_iterator(self.lundistort_points(src)))
        (ok, corners) = get_corners(ri)
        if not ok:
            return -1
        src = cv.Reshape(mk_image_points([corners]), 2)
        R = list(cvmat_iterator(self.rundistort_points(src)))

        # print 'diffs', [abs(y0-y1) for ((_, y0), (_, y1)) in zip(L, R)]
        d = [(y0-y1) for ((_, y0), (_, y1)) in zip(L, R)]
        return math.sqrt(sum([i**2 for i in d]) / len(d))

    def chessboard_size(self, li, ri):
        """
        Return the square edge length, in meters, or -1 if not possible.
        """
        (ok, corners) = get_corners(li)
        if not ok:
            return -1
        src = cv.Reshape(mk_image_points([corners]), 2)
        L = list(cvmat_iterator(self.lundistort_points(src)))
        (ok, corners) = get_corners(ri)
        if not ok:
            return -1
        src = cv.Reshape(mk_image_points([corners]), 2)
        R = list(cvmat_iterator(self.rundistort_points(src)))


        # Project the points to 3d
        cam = image_geometry.StereoCameraModel()
        cam.fromCameraInfo(*self.as_message())
        disparities = [(x0 - x1) for ((x0, y0), (x1, y1)) in zip(L, R)]
        pt3d = [cam.projectPixelTo3d((x, y), d) for ((x, y), d) in zip(L, disparities)]
        def l2(p0, p1):
            return math.sqrt(sum([(c0 - c1) ** 2 for (c0, c1) in zip(p0, p1)]))

        # Compute the length from each horizontal and vertical lines, and return the mean
        lengths = (
            [l2(pt3d[8 * r + 0], pt3d[8 * r + 7]) / 7 for r in range(6)] +
            [l2(pt3d[c + 0], pt3d[c + 40]) / 5 for c in range(7)])
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


