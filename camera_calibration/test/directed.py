import roslib
PKG = 'camera_calibration'
roslib.load_manifest(PKG)
import rostest
import rospy
import cv
import unittest
import tarfile

from camera_calibration.calibrator import cvmat_iterator, MonoCalibrator, StereoCalibrator, CalibrationException

class TestDirected(unittest.TestCase):

    def setUp(self):
        self.mydir = roslib.packages.get_pkg_dir(PKG)
        self.tar = tarfile.open('%s/camera_calibration.tar.gz' % self.mydir, 'r')
        self.limages = [self.image_from_archive("wide/left%04d.pgm" % i) for i in range(3, 15)]
        self.rimages = [self.image_from_archive("wide/right%04d.pgm" % i) for i in range(3, 15)]

    def image_from_archive(self, name):
        member = self.tar.getmember(name)
        filedata = self.tar.extractfile(member).read()
        imagefiledata = cv.CreateMat(1, len(filedata), cv.CV_8UC1)
        cv.SetData(imagefiledata, filedata, len(filedata))
        return cv.DecodeImageM(imagefiledata)
        
    def test_monocular(self):
        mc = MonoCalibrator((8,6), .108)
        mc.cal(self.limages)
        if 0:
            cv.NamedWindow("display")
            for im in images:
                (ok, corners) = get_corners(im)
                if ok:
                    src = cv.Reshape(mk_image_points([corners]), 2)
                    rm = mc.remap(cv.GetMat(im))

                    L = list(cvmat_iterator(mc.undistort_points(src)))

                    cv.DrawChessboardCorners(rm, (8, 6), L, True)
                    cv.ShowImage("display", rm)
                    if cv.WaitKey() == ord('q'):
                        assert 0

    def test_stereo(self):
        print self.image_from_archive('wide/left0003.pgm')
        limages = [self.image_from_archive("wide/left%04d.pgm" % i) for i in range(3, 15)]
        rimages = [self.image_from_archive("wide/right%04d.pgm" % i) for i in range(3, 15)]
        mc = StereoCalibrator((8, 6), .108)
        mc.cal(self.limages, self.rimages)

        mc.report()
        mc.ost()

        self.assert_(mc.epipolar1(self.limages[0], self.rimages[0]) < 0.5)
        self.assertAlmostEqual(mc.chessboard_size(self.limages[0], self.rimages[0]), .108, 2)

    def test_nochecker(self):

        # Run with same images, but looking for an incorrect chessboard size (8, 7).
        # Should raise an exception because of lack of input points.

        size = (8, 7)
        sc = StereoCalibrator(size, .108)
        self.assertRaises(CalibrationException, lambda: sc.cal(self.limages, self.rimages))
        mc = MonoCalibrator(size, .108)
        self.assertRaises(CalibrationException, lambda: mc.cal(self.limages))

if __name__ == '__main__':
    if 1:
        rostest.unitrun('camera_calibration', 'directed', TestDirected)
    else:
        suite = unittest.TestSuite()
        suite.addTest(TestDirected('test_monocular'))
        unittest.TextTestRunner(verbosity=2).run(suite)
