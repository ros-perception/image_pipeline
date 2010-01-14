#!/usr/bin/python

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

ID_LOAD=101
ID_SAVE=102
ID_BUTTON1=110
ID_EXIT=200

# /wg/osx/rosCode/ros-pkg/ros-pkg/stacks/image_pipeline/image_view/preCalib

from camera_calibration.calibrator import cvmat_iterator, MonoCalibrator, StereoCalibrator
from std_msgs.msg import String
from std_srvs.srv import Empty

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
        while True:
            while True:
                m = self.queue.get()
                if self.queue.empty():
                    break
            self.function(m)

class CalibrationNode:

    def __init__(self, chess_size, dim):
        self.chess_size = chess_size
        self.dim = dim
        lsub = message_filters.Subscriber('left', sensor_msgs.msg.Image)
        rsub = message_filters.Subscriber('right', sensor_msgs.msg.Image)
        ts = message_filters.TimeSynchronizer([lsub, rsub], 4)
        ts.registerCallback(self.queue_stereo)

        rospy.Subscriber('image', sensor_msgs.msg.Image, self.queue_monocular)

        self.set_camera_info_service = rospy.ServiceProxy("%s/set_camera_info" % rospy.remap_name("camera"), sensor_msgs.srv.SetCameraInfo)
        self.set_left_camera_info_service = rospy.ServiceProxy("%s/set_camera_info" % rospy.remap_name("left_camera"), sensor_msgs.srv.SetCameraInfo)
        self.set_right_camera_info_service = rospy.ServiceProxy("%s/set_camera_info" % rospy.remap_name("right_camera"), sensor_msgs.srv.SetCameraInfo)

        self.br = cv_bridge.CvBridge()
        self.p_mins = None
        self.p_maxs = None
        self.db = {}
        self.c = None
        self.calibrated = False

        self.q_mono = Queue.Queue()
        self.q_stereo = Queue.Queue()

        self.goodenough = False

        mth = ConsumerThread(self.q_mono, self.handle_monocular)
        mth.setDaemon(True)
        mth.start()

        sth = ConsumerThread(self.q_stereo, self.handle_stereo)
        sth.setDaemon(True)
        sth.start()

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

    def queue_monocular(self, msg):
        self.q_mono.put(msg)

    def queue_stereo(self, lmsg, rmsg):
        self.q_stereo.put((lmsg, rmsg))

    def compute_goodenough(self):
        if len(self.db) > 0:
            Ps = [v[0] for v in self.db.values()]
            Pmins = reduce(lmin, Ps)
            Pmaxs = reduce(lmax, Ps)
            if reduce(operator.__mul__, [(hi - lo) for (lo, hi) in zip(Pmins, Pmaxs)]) > .1:
                self.goodenough = True

    def handle_monocular(self, msg):

        if self.c == None:
            self.c = MonoCalibrator(self.chess_size)

        rgb = self.mkgray(msg)
        (self.width, self.height) = cv.GetSize(rgb)
        scrib = rgb

        scale = int(math.ceil(self.width / 640))
        if scale != 1:
            scrib = cv.CreateMat(self.height / scale, self.width / scale, cv.GetElemType(rgb))
            cv.Resize(rgb, scrib)
        else:
            scrib = cv.CloneMat(rgb)
        self.displaywidth = scrib.cols

        (ok, corners) = self.c.get_corners(rgb, refine = False)
        if ok:
            # Compute some parameters for this chessboard
            Xs = [x for (x, y) in corners]
            Ys = [y for (x, y) in corners]
            p_x = mean(Xs) / self.width
            p_y = mean(Ys) / self.height
            p_size = (max(Xs) - min(Xs)) / self.width
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

            src = cv.Reshape(self.c.mk_image_points([corners]), 2)

            cv.DrawChessboardCorners(scrib, self.chess_size, [ (x/scale, y/scale) for (x, y) in cvmat_iterator(src)], True)

            # If the image is a min or max in every parameter, add to the collection
            if any(is_min) or any(is_max):
                self.db[str(is_min + is_max)] = (params, rgb)

        if self.calibrated:
            rgb_remapped = self.c.remap(rgb)
            cv.Resize(rgb_remapped, scrib)

        self.compute_goodenough()

        self.redraw_monocular(scrib, rgb)

    def handle_stereo(self, msg):

        (lmsg, rmsg) = msg
        if self.c == None:
            self.c = StereoCalibrator(self.chess_size)
        lrgb = self.mkgray(lmsg)
        rrgb = self.mkgray(rmsg)
        (self.width, self.height) = cv.GetSize(lrgb)
        lscrib = lrgb
        rscrib = rrgb

        if self.calibrated:
            epierror = self.c.epipolar1(lrgb, rrgb)
            if epierror == -1:
                print "Cannot find checkerboard"
            else:
                print "epipolar error:", epierror
            lscrib = self.c.lremap(lrgb)
            rscrib = self.c.rremap(rrgb)
        else:
            lscrib = cv.CloneMat(lrgb)
            rscrib = cv.CloneMat(rrgb)
        self.displaywidth = lscrib.cols + rscrib.cols

        (lok, lcorners) = self.c.get_corners(lrgb, refine = True)
        if lok:
            (rok, rcorners) = self.c.get_corners(rrgb, refine = True)
            if lok and rok:
                # Compute some parameters for this chessboard
                Xs = [x for (x, y) in lcorners]
                Ys = [y for (x, y) in lcorners]
                p_x = mean(Xs) / self.width
                p_y = mean(Ys) / self.height
                p_size = (max(Xs) - min(Xs)) / self.width
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

                for (co, im, udm) in [(lcorners, lscrib, self.c.lundistort_points), (rcorners, rscrib, self.c.rundistort_points)]:
                    src = cv.Reshape(self.c.mk_image_points([co]), 2)
                    if self.calibrated:
                        src = udm(src)
                    cv.DrawChessboardCorners(im, self.chess_size, cvmat_iterator(src), True)

                # If the image is a min or max in every parameter, add to the collection
                if any(is_min) or any(is_max):
                    self.db[str(is_min + is_max)] = (params, lrgb, rrgb)

        self.compute_goodenough()

        self.redraw_stereo(lscrib, rscrib, lrgb, rrgb)

    def do_calibration(self):
        self.calibrated = True
        vv = list(self.db.values())
        # vv is a list of pairs (p, i) for monocular, and triples (p, l, r) for stereo
        if self.c.is_mono:
            images = [i for (p, i) in vv]
            self.c.cal(images)
        else:
            limages = [ l for (p, l, r) in vv ]
            rimages = [ r for (p, l, r) in vv ]
            self.c.cal(limages, rimages)

    def do_tarfile_save(self, tf):
        """ Write images and calibration solution to a tarfile object """
        vv = list(self.db.values())
        # vv is a list of pairs (p, i) for monocular, and triples (p, l, r) for stereo
        if self.c.is_mono:
            ims = [("left-%04d.png" % i, im) for i,(_, im) in enumerate(vv)]
        else:
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

        taradd('ost.txt', self.c.ost())

    def do_save(self):
        filename = '/tmp/calibrationdata.tar.gz'
        tf = tarfile.open(filename, 'w:gz')
        self.do_tarfile_save(tf)
        tf.close()
        print "Wrote calibration data to", filename

    def do_upload(self):
        vv = list(self.db.values())
        self.c.report()
        print self.c.ost()
        info = self.c.as_message()
        if self.c.is_mono:
            self.set_camera_info_service(info)
        else:
            self.set_left_camera_info_service(info[0])
            self.set_right_camera_info_service(info[1])

    def set_scale(self, a):
        if self.calibrated:
            vv = list(self.db.values())
            self.c.set_alpha(a)

class WebCalibrationNode(CalibrationNode):
    """ Calibration node backend for a web-based UI """

    def __init__(self, *args):
        CalibrationNode.__init__(self, *args)
        self.img_pub = rospy.Publisher("calibration_image", sensor_msgs.msg.Image)
        self.meta_pub = rospy.Publisher("calibration_meta", String)
        self.calibration_done = rospy.Service('calibration_done', Empty, self.calibrate)

    def calibrate(self, req):
        self.do_calibration()

    def publish_meta(self):
        if not self.calibrated:
            if len(self.db) != 0:
                # Report dimensions of the n-polytope
                Ps = [v[0] for v in self.db.values()]
                Pmins = reduce(lmin, Ps)
                Pmaxs = reduce(lmax, Ps)
                vals = ['%s:%s:%s' % (label, lo, hi) for (label, lo, hi) in zip(["X", "Y", "Size"], Pmins, Pmaxs)]
                self.meta_pub.publish(String(",".join(vals)))

    def redraw_monocular(self, scrib, _):
        msg = self.br.cv_to_imgmsg(scrib, "bgr8")
        msg.header.stamp = rospy.rostime.get_rostime()
        self.img_pub.publish(msg)
        self.publish_meta()
                   

    def redraw_stereo(self, lscrib, rscrib, lrgb, rrgb):
        display = cv.CreateMat(lscrib.height, lscrib.width + rscrib.width, cv.CV_8UC3)
        cv.Copy(lscrib, cv.GetSubRect(display, (0,0,lscrib.width,lscrib.height)))
        cv.Copy(rscrib, cv.GetSubRect(display, (lscrib.width,0,rscrib.width,rscrib.height)))
        msg = self.br.cv_to_imgmsg(display, "bgr8")
        msg.header.stamp = rospy.rostime.get_rostime()
        self.img_pub.publish(msg)
        self.publish_meta()

class OpenCVCalibrationNode(CalibrationNode):
    """ Calibration node with an OpenCV Gui """

    def __init__(self, *args):

        CalibrationNode.__init__(self, *args)
        cv.NamedWindow("display")
        self.font = cv.InitFont(cv.CV_FONT_HERSHEY_SIMPLEX, 0.20, 1, thickness = 2, line_type = cv.CV_AA)
        #self.button = cv.LoadImage("%s/button.jpg" % roslib.packages.get_pkg_dir(PKG))
        cv.SetMouseCallback("display", self.on_mouse)
        cv.CreateTrackbar("scale", "display", 0, 100, self.on_scale)

    def on_mouse(self, event, x, y, flags, param):
        if event == cv.CV_EVENT_LBUTTONDOWN:
            if 180 <= y < 280:
                self.do_calibration()
            elif 280 <= y < 380:
                self.do_save()
            elif 380 <= y < 480:
                self.do_upload()

    def waitkey(self):
        k = cv.WaitKey(6)
        if k == ord('q'):
            rospy.signal_shutdown('Quit')
        return k

    def on_scale(self, scalevalue):
        self.set_scale(scalevalue / 100.0)


    def button(self, dst, label, enable):
        cv.Set(dst, (255, 255, 255))
        size = cv.GetSize(dst)
        if enable:
            color = cv.RGB(155, 155, 80)
        else:
            color = cv.RGB(224, 224, 224)
        cv.Circle(dst, (size[0] / 2, size[1] / 2), min(size) / 2, color, -1)
        ((w, h), _) = cv.GetTextSize(label, self.font)
        cv.PutText(dst, label, ((size[0] - w) / 2, (size[1] + h) / 2), self.font, (255,255,255))

    def buttons(self, display):
        x = self.displaywidth
        self.button(cv.GetSubRect(display, (x,180,100,100)), "CALIBRATE", self.goodenough)
        self.button(cv.GetSubRect(display, (x,280,100,100)), "SAVE", self.calibrated)
        self.button(cv.GetSubRect(display, (x,380,100,100)), "UPLOAD", self.calibrated)

    def y(self, i):
        return 30 + 50 * i
        
    def screendump(self, im):
        i = 0
        while os.access("/tmp/dump%d.png" % i, os.R_OK):
            i += 1
        cv.SaveImage("/tmp/dump%d.png" % i, im)

    def redraw_monocular(self, scrib, _):
        width, height = cv.GetSize(scrib)

        display = cv.CreateMat(height, width + 100, cv.CV_8UC3)
        cv.Copy(scrib, cv.GetSubRect(display, (0,0,width,height)))
        cv.Set(cv.GetSubRect(display, (width,0,100,height)), (255, 255, 255))


        self.buttons(display)
        if not self.calibrated:
            if len(self.db) != 0:
                # Report dimensions of the n-polytope
                Ps = [v[0] for v in self.db.values()]
                Pmins = reduce(lmin, Ps)
                Pmaxs = reduce(lmax, Ps)
                ranges = [(x-n) for (x, n) in zip(Pmaxs, Pmins)]

                for i, (label, lo, hi) in enumerate(zip(["X", "Y", "Size"], Pmins, Pmaxs)):
                    (w,_),_ = cv.GetTextSize(label, self.font)
                    cv.PutText(display, label, (width + (100 - w) / 2, self.y(i)), self.font, (0,0,0))
                    cv.Line(display,
                            (int(width + lo * 100), self.y(i) + 20),
                            (int(width + hi * 100), self.y(i) + 20),
                            (0,0,0),
                            4)

        else:
            cv.PutText(display, "acc.", (width, self.y(0)), self.font, (0,0,0))

        self.show(display)

    def redraw_stereo(self, lscrib, rscrib, lrgb, rrgb):
        display = cv.CreateMat(self.height, 2 * self.width + 100, cv.CV_8UC3)
        cv.Copy(lscrib, cv.GetSubRect(display, (0,0,self.width,self.height)))
        cv.Copy(rscrib, cv.GetSubRect(display, (self.width,0,self.width,self.height)))
        cv.Set(cv.GetSubRect(display, (2 * self.width,0,100,self.height)), (255, 255, 255))

        self.buttons(display)

        if not self.calibrated:
            if len(self.db) != 0:
                # Report dimensions of the n-polytope
                Ps = [v[0] for v in self.db.values()]
                Pmins = reduce(lmin, Ps)
                Pmaxs = reduce(lmax, Ps)
                ranges = [(x-n) for (x, n) in zip(Pmaxs, Pmins)]
                for i, (label, lo, hi) in enumerate(zip(["X", "Y", "Size"], Pmins, Pmaxs)):
                    (width,_),_ = cv.GetTextSize(label, self.font)
                    cv.PutText(display, label, (2 * self.width + (100 - width) / 2, self.y(i)), self.font, (0,0,0))
                    #print label, hi, lo
                    cv.Line(display,
                            (int(2 * self.width + lo * 100), self.y(i) + 20),
                            (int(2 * self.width + hi * 100), self.y(i) + 20),
                            (0,0,0),
                            4)

        else:
            cv.PutText(display, "epi.", (2 * self.width, self.y(0)), self.font, (0,0,0))
            epierror = self.c.epipolar1(lrgb, rrgb)
            if epierror == -1:
                msg = "?"
            else:
                msg = "%.2f" % epierror
            cv.PutText(display, msg, (2 * self.width, self.y(1)), self.font, (0,0,0))
            if epierror != -1:
                cv.PutText(display, "dim", (2 * self.width, self.y(2)), self.font, (0,0,0))
                dim = self.c.chessboard_size(lrgb, rrgb)
                cv.PutText(display, "%.3f" % dim, (2 * self.width, self.y(3)), self.font, (0,0,0))

        self.show(display)

    def show(self, im):
        cv.ShowImage("display", im)
        if self.waitkey() == ord('s'):
            self.screendump(im)

def main():
    from optparse import OptionParser
    rospy.init_node('cameracalibrator')
    parser = OptionParser()
    parser.add_option("-w", "--web", dest="web", action="store_true", default=False, help="create backend for web-based calibration")
    parser.add_option("-o", "--opencv", dest="web", action="store_false", help="use OpenCV-based GUI for calibration (default)")
    parser.add_option("-s", "--size", default="8x6", help="specify chessboard size as nxm [default: %default]")
    parser.add_option("-q", "--square", default=".108", help="specify chessboard square size in meters [default: %default]")
    options, args = parser.parse_args()
    size = tuple([int(c) for c in options.size.split('x')])
    dim = float(options.square)
    if options.web:
        node = WebCalibrationNode(size, dim)
    else:
        node = OpenCVCalibrationNode(size, dim)
    rospy.spin()

if __name__ == "__main__":
    main()
