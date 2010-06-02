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
        self.l = {}
        self.r = {}
        self.sizes = [(320,240), (640,480), (800,600), (1024,768)]
        for dim in self.sizes:
            self.l[dim] = []
            self.r[dim] = []
            for li,ri in zip(self.limages, self.rimages):
                rli = cv.CreateMat(dim[1], dim[0], cv.CV_8UC3)
                rri = cv.CreateMat(dim[1], dim[0], cv.CV_8UC3)
                cv.Resize(li, rli)
                cv.Resize(ri, rri)
                self.l[dim].append(rli)
                self.r[dim].append(rri)

    def image_from_archive(self, name):
        member = self.tar.getmember(name)
        filedata = self.tar.extractfile(member).read()
        imagefiledata = cv.CreateMat(1, len(filedata), cv.CV_8UC1)
        cv.SetData(imagefiledata, filedata, len(filedata))
        return cv.DecodeImageM(imagefiledata)
        
    def assert_good_mono(self, c, dim):
        c.report()
        self.assert_(len(c.ost()) > 0)
        img = self.l[dim][0]
        lin_err = c.linear_error(img)
        self.assert_(0.0 < lin_err)
        self.assert_(lin_err < 1.0)

        flat = c.remap(img)
        self.assertEqual(cv.GetSize(img), cv.GetSize(flat))

    def test_monocular(self):
        # Run the calibrator, produce a calibration, check it
        mc = MonoCalibrator((8,6), .108)
        for dim in self.sizes:
            mc.cal(self.l[dim])
            self.assert_good_mono(mc, dim)

            # Make another calibration, import previous calibration as a message,
            # and assert that the new one is good.

            mc2 = MonoCalibrator((8,6), .108)
            mc2.from_message(mc.as_message())
            self.assert_good_mono(mc2, dim)

    def test_stereo(self):
        for dim in self.sizes:
            sc = StereoCalibrator((8, 6), .108)
            sc.cal(self.l[dim], self.r[dim])

            sc.report()
            sc.ost()

            self.assert_(sc.epipolar1(self.l[dim][0], self.r[dim][0]) < 0.5)
            self.assertAlmostEqual(sc.chessboard_size(self.l[dim][0], self.r[dim][0]), .108, 2)

            print sc.as_message()

            img = self.l[dim][0]
            flat = sc.lremap(img)
            self.assertEqual(cv.GetSize(img), cv.GetSize(flat))
            flat = sc.rremap(img)
            self.assertEqual(cv.GetSize(img), cv.GetSize(flat))

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
