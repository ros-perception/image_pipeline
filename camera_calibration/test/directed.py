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

        mc2 = MonoCalibrator((8,6), .108)
        mc2.from_message(mc.as_message())
        mc2.report()
        self.assert_(len(mc2.ost()) > 0)

    def test_stereo(self):
        limages = [self.image_from_archive("wide/left%04d.pgm" % i) for i in range(3, 15)]
        rimages = [self.image_from_archive("wide/right%04d.pgm" % i) for i in range(3, 15)]
        sc = StereoCalibrator((8, 6), .108)
        sc.cal(self.limages, self.rimages)

        sc.report()
        sc.ost()

        self.assert_(sc.epipolar1(self.limages[0], self.rimages[0]) < 0.5)
        self.assertAlmostEqual(sc.chessboard_size(self.limages[0], self.rimages[0]), .108, 2)

        print sc.as_message()

        sc2 = StereoCalibrator((8, 6), .108)
        sc2.from_message(sc.as_message())
        # sc2.set_alpha(1.0)
        sc2.report()
        self.assert_(len(sc2.ost()) > 0)

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
        suite.addTest(TestDirected('test_stereo'))
        unittest.TextTestRunner(verbosity=2).run(suite)
