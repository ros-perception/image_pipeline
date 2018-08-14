camera_calibration
==================

The camera_calibration package contains a user-friendly calibration tool,
cameracalibrator.  This tool uses the following Python classes, which
conveniently hide some of the complexities of using OpenCV's calibration
process and chessboard detection, and the details of constructing a ROS
CameraInfo message.  These classes are documented here for people who
need to extend or make a new calibration tool.

For details on the camera model and camera calibration process, see
http://docs.opencv.org/master/d9/d0c/group__calib3d.html

.. autoclass:: camera_calibration.calibrator.MonoCalibrator
    :members:

.. autoclass:: camera_calibration.calibrator.StereoCalibrator
    :members:
