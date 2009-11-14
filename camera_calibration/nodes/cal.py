#!/usr/bin/python

PKG = 'camera_calibration' # this package name
import roslib; roslib.load_manifest(PKG)

import rospy
import sensor_msgs.msg
import cv_bridge

import math
import os
import sys

import Image

import wx
import cv

import message_filters

ID_LOAD=101
ID_SAVE=102
ID_BUTTON1=110
ID_EXIT=200

# /wg/osx/rosCode/ros-pkg/ros-pkg/stacks/image_pipeline/image_view/preCalib

num_x_ints = 8
num_y_ints = 6
num_pts = num_x_ints * num_y_ints

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

def mean(seq):
    return sum(seq) / len(seq)

def lmin(seq1, seq2):
    """ Pairwise minimum of two sequences """
    return [min(a, b) for (a, b) in zip(seq1, seq2)]

def lmax(seq1, seq2):
    """ Pairwise maximum of two sequences """
    return [max(a, b) for (a, b) in zip(seq1, seq2)]

def cvmat_iterator(cvmat):
    for i in range(cvmat.rows):
        for j in range(cvmat.cols):
            yield cvmat[i,j]

class VideoBox(wx.Panel):

    def __init__(self, parent, imagefile):
        wx.Panel.__init__(self, parent, size = (640, 480))
        self.Bind(wx.EVT_PAINT, self.OnPaint)
        self.img = cv.LoadImage(imagefile)

        images = [cv.LoadImage("left%04d.pgm" % d) for d in range(6)]
        (intrinsics, distortion) = calibrate_mono(images)

        mapx = cv.CreateImage(cv.GetSize(images[0]), cv.IPL_DEPTH_32F, 1 );
        mapy = cv.CreateImage(cv.GetSize(images[0]), cv.IPL_DEPTH_32F, 1 );
        cv.InitUndistortMap(intrinsics, distortion, mapx, mapy)
        self.img = cv.LoadImage(imagefile)
        rect = cv.CloneImage(self.img)
        cv.Remap(self.img, rect, mapx, mapy);
        self.img = rect

    def OnPaint(self, e):

        # (ok, verts) = cv.FindChessboardCorners(self.img, (8, 6), cv.CV_CALIB_CB_ADAPTIVE_THRESH | cv.CV_CALIB_CB_NORMALIZE_IMAGE)
        # cv.DrawChessboardCorners(self.img, (8, 6), verts, ok)


        dc = wx.PaintDC(self)
        (w, h) = cv.GetSize(self.img)
        image = wx.EmptyImage(w, h)
        image.SetData(self.img.tostring())
        dc.DrawBitmap(image.ConvertToBitmap(), 0, 0)

class MainWindow(wx.Frame):

    def __init__(self,parent,id,title):
        self.dirname=''
        wx.Frame.__init__(self, parent, wx.ID_ANY, title)
        self.control = wx.TextCtrl(self, 1, style=wx.TE_MULTILINE)
        self.CreateStatusBar() # A Statusbar in the bottom of the window

        # Setting up the menu.
        filemenu= wx.Menu()
        filemenu.Append(ID_LOAD, "&Load images", " Load images")
        filemenu.AppendSeparator()
        filemenu.Append(ID_SAVE, "&Save images", " Save images")
        filemenu.AppendSeparator()
        filemenu.Append(ID_EXIT,"E&xit"," Terminate the program")
        # Creating the menubar.
        menuBar = wx.MenuBar()
        menuBar.Append(filemenu,"&File") # Adding the "filemenu" to the MenuBar
        self.SetMenuBar(menuBar)  # Adding the MenuBar to the Frame content.
        wx.EVT_MENU(self, ID_EXIT, self.OnExit)
        wx.EVT_MENU(self, ID_LOAD, self.OnOpen)

        self.panel0 = VideoBox(self, "left0000.pgm")
        self.panel1 = VideoBox(self, "right0000.pgm")

        # Use some sizers to see layout options
        self.sizer=wx.BoxSizer(wx.HORIZONTAL)
        self.sizer.Add(self.panel0, 1,wx.EXPAND)
        self.sizer.Add(self.panel1, 1,wx.EXPAND)
        self.SetSizer(self.sizer)
        self.SetAutoLayout(1)
        self.sizer.Fit(self)
        self.Show(1)

    def OnAbout(self,e):
        d= wx.MessageDialog( self, " A sample editor \n"
                            " in wxPython","About Sample Editor", wx.OK)
                            # Create a message dialog box
        d.ShowModal() # Shows it
        d.Destroy() # finally destroy it when finished.

    def OnExit(self, e):
        self.Close(True)  # Close the frame.

    def OnOpen(self,e):
        """ Open a file"""
        dlg = wx.FileDialog(self, "Choose a file", self.dirname, "", "*.*", wx.OPEN)
        if dlg.ShowModal() == wx.ID_OK:
            self.filename=dlg.GetFilename()
            self.dirname=dlg.GetDirectory()
            f=open(os.path.join(self.dirname, self.filename),'r')
            self.control.SetValue(f.read())
            f.close()
        dlg.Destroy()

#app = wx.PySimpleApp()
#frame = MainWindow(None, -1, "Calibration")
#app.MainLoop()

class MonoCalibrator:

    def __init__(self):
        pass

    def cal(self, images):
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

class CalibrationException(Exception):
    pass

class StereoCalibrator:

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

        T = cv.CreateMat(3, 1, cv.CV_64FC1)
        R = cv.CreateMat(3, 3, cv.CV_64FC1)
        cv.SetIdentity(T)
        cv.SetIdentity(R)
        cv.StereoCalibrate(opts, lipts, ripts, npts,
                           self.l.intrinsics, self.l.distortion,
                           self.r.intrinsics, self.r.distortion,
                           cv.GetSize(limages[0]),
                           R,                                  # R
                           T,                                  # T
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
        cv.StereoRectify(self.l.intrinsics,
                         self.r.intrinsics,
                         self.l.distortion,
                         self.r.distortion,
                         cv.GetSize(limages[0]),
                         R,
                         T,
                         self.lR, self.rR, self.lP, self.rP)
        
        self.lmapx = cv.CreateImage((640, 480), cv.IPL_DEPTH_32F, 1)
        self.lmapy = cv.CreateImage((640, 480), cv.IPL_DEPTH_32F, 1)
        self.rmapx = cv.CreateImage((640, 480), cv.IPL_DEPTH_32F, 1)
        self.rmapy = cv.CreateImage((640, 480), cv.IPL_DEPTH_32F, 1)
        cv.InitUndistortRectifyMap(self.l.intrinsics, self.l.distortion, self.lR, self.lP, self.lmapx, self.lmapy)
        cv.InitUndistortRectifyMap(self.r.intrinsics, self.r.distortion, self.rR, self.rP, self.rmapx, self.rmapy)

    def lrmsg(self, d, k, r, p):
        """ Used by :meth:`as_message`.  Return a CameraInfo message for the given calibration matrices """
        msg = sensor_msgs.msg.CameraInfo()
        (msg.width, msg.height) = self.size
        msg.D = [d[i,0] for i in range(d.rows)]
        msg.K = list(cvmat_iterator(k))
        msg.R = list(cvmat_iterator(r))
        msg.P = list(cvmat_iterator(p))
        return msg

    def as_message(self):
        return (self.lrmsg(self.l.distortion, self.l.intrinsics, self.lR, self.lP),
                self.lrmsg(self.r.distortion, self.r.intrinsics, self.rR, self.rP))

    def lrreport(self, d, k, r, p):
        print "D = ", list(cvmat_iterator(d))
        print "K = ", list(cvmat_iterator(k))
        print "R = ", list(cvmat_iterator(r))
        print "P = ", list(cvmat_iterator(p))

    def report(self):
        print "\nLeft:"
        self.lrreport(self.l.distortion, self.l.intrinsics, self.lR, self.lP)
        print "\nRight:"
        self.lrreport(self.r.distortion, self.r.intrinsics, self.rR, self.rP)
        

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


class CalibrationNode:

    def __init__(self):
        #lsub = message_filters.Subscriber('/wide_stereo/left/image_raw', sensor_msgs.msg.Image)
        #rsub = message_filters.Subscriber('/wide_stereo/right/image_raw', sensor_msgs.msg.Image)
        lsub = message_filters.Subscriber('left', sensor_msgs.msg.Image)
        rsub = message_filters.Subscriber('right', sensor_msgs.msg.Image)
        ts = message_filters.TimeSynchronizer([lsub, rsub], 4)
        ts.registerCallback(self.handle)
        cv.NamedWindow("display")
        self.font = cv.InitFont(cv.CV_FONT_HERSHEY_SIMPLEX, 1, 1, thickness = 2, line_type = cv.CV_AA)
        self.br = cv_bridge.CvBridge()
        self.p_mins = None
        self.p_maxs = None
        self.db = {}
        self.sc = StereoCalibrator()
        self.calibrated = False
        self.button = cv.LoadImage("%s/button.jpg" % roslib.packages.get_pkg_dir(PKG))
        cv.SetMouseCallback("display", self.on_mouse)

    def on_mouse(self, event, x, y, flags, param):
        if event == cv.CV_EVENT_LBUTTONDOWN:
            limages = [ l for (p, l, r) in self.db.values() ]
            rimages = [ r for (p, l, r) in self.db.values() ]
            self.sc.cal(limages, rimages)
            self.calibrated = True

            for (i, (p, limg, rimg)) in enumerate(self.db.values()):
                cv.SaveImage("/tmp/cal%04d.png" % i, self.sc.lremap(limg))

            self.sc.report()

    def handle(self, lmsg, rmsg):

        def mkgray(msg):
            if 'bayer' in msg.encoding:
                msg.encoding = "mono8"
                raw = self.br.imgmsg_to_cv(msg)
                rgb = cv.CreateMat(raw.rows, raw.cols, cv.CV_8UC3)
                mono = cv.CreateMat(raw.rows, raw.cols, cv.CV_8UC1)
                cv.CvtColor(raw, rgb, cv.CV_BayerRG2BGR)
                cv.CvtColor(rgb, mono, cv.CV_BGR2GRAY)
                cv.CvtColor(mono, rgb, cv.CV_GRAY2BGR)
            else:
                rgb = self.br.imgmsg_to_cv(msg, "bgr8")

            return rgb

        lrgb = mkgray(lmsg)
        rrgb = mkgray(rmsg)
        lscrib = lrgb
        rscrib = rrgb

        if not self.calibrated:
            (lok, lcorners) = get_corners(lrgb, refine = False)
            if lok:
                (rok, rcorners) = get_corners(rrgb, refine = False)
                if lok and rok:
                    # Compute some parameters for this chessboard
                    Xs = [x for (x, y) in lcorners]
                    Ys = [y for (x, y) in lcorners]
                    p_x = mean(Xs) / 640
                    p_y = mean(Ys) / 480
                    p_size = (max(Xs) - min(Xs)) / 640
                    params = [p_x, p_y, p_size]
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
                    
                    lscrib = cv.CloneMat(lrgb)
                    rscrib = cv.CloneMat(rrgb)
                    for (co, im) in [(lcorners, lscrib), (rcorners, rscrib)]:
                        src = cv.Reshape(mk_image_points([co]), 2)
                        cv.DrawChessboardCorners(im, (8, 6), cvmat_iterator(src), True)

                    # If the image is a min or max in every parameter, add to the collection
                    if any(is_min) or any(is_max):
                        self.db[str(is_min + is_max)] = (params, lrgb, rrgb)
        else:
            epierror = self.sc.epipolar1(lrgb, rrgb)
            if epierror == -1:
                print "Cannot find checkerboard"
            else:
                print "epipolar error:", epierror
            lscrib = self.sc.lremap(lrgb)
            rscrib = self.sc.rremap(rrgb)

        display = cv.CreateMat(480, 1280 + 100, cv.CV_8UC3)
        cv.Copy(lscrib, cv.GetSubRect(display, (0,0,640,480)))
        cv.Copy(rscrib, cv.GetSubRect(display, (640,0,640,480)))
        cv.Set(cv.GetSubRect(display, (1280,0,100,480)), (255, 255, 255))
        cv.Resize(self.button, cv.GetSubRect(display, (1280,380,100,100)))

        # Report dimensions of the n-polytope
        Ps = [p for (p, _, _) in self.db.values()]
        Pmins = reduce(lmin, Ps)
        Pmaxs = reduce(lmax, Ps)
        ranges = [(x-n) for (x, n) in zip(Pmaxs, Pmins)]

        if not self.calibrated:
            for i, (label, lo, hi) in enumerate(zip(["X", "Y", "Size"], Pmins, Pmaxs)):
                y = 100 + 100 * i
                (width,_),_ = cv.GetTextSize(label, self.font)
                cv.PutText(display, label, (1280 + (100 - width) / 2, 100 + 100 * i), self.font, (0,0,0))
                cv.Line(display,
                        (1280 + lo * 100, y + 20),
                        (1280 + hi * 100, y + 20),
                        (0,0,0),
                        4)
        else:
            cv.PutText(display, "error", (1280, 100), self.font, (0,0,0))
            epierror = self.sc.epipolar1(lrgb, rrgb)
            if epierror == -1:
                msg = "?"
            else:
                msg = "%.2f" % epierror
            cv.PutText(display, msg, (1280, 200), self.font, (0,0,0))

        cv.ShowImage("display", display)
        k = cv.WaitKey(6)

if 1:
    rospy.init_node('calibrationnode')
    node = CalibrationNode()
    rospy.spin()
else:
    dir = "/u/jamesb/ros/pkgs/wg-ros-pkg-trunk/deprecated/dcam/"
    limages = [cv.GetMat(cv.LoadImage(dir + "wide/left%04d.pgm" % d)) for d in range(3, 15)]
    rimages = [cv.GetMat(cv.LoadImage(dir + "wide/right%04d.pgm" % d)) for d in range(3, 15)]

    sc = StereoCalibrator()
    sc.cal(limages, rimages)
    sc.report()
    #print sc.as_message()[0]

    print sc.epipolar1(limages[0], rimages[0])

    if 1:

        for img in limages[:]:

            (_, corners) = get_corners(img)
            src = cv.Reshape(mk_image_points([corners]), 2)
            src = sc.lundistort_points(src)
            img = sc.lremap(img)

            cv.DrawChessboardCorners(img, (8, 6), cvmat_iterator(src), 1)
            cv.NamedWindow("snap")
            cv.ShowImage("snap", img)
            cv.WaitKey()

        sys.exit(0)
