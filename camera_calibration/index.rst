camera_calibration
==================

The camera_calibration package contains a user-friendly calibration tool,
cameracalibrator.  This tool uses the following Python classes, which
conveniently hide some of the complexities of using OpenCV's calibration
process and chessboard detection, and the details of constructing a ROS
CameraInfo message.  These classes are documented here for people who
need to extend or make a new calibration tool.

For details on the camera model and camera calibration process, see
http://opencv.willowgarage.com/documentation/camera_calibration_and_3d_reconstruction.html.

.. autoclass:: camera_calibration.calibrator.MonoCalibrator
    :members: cal, set_alpha, remap, undistort_points, as_message, from_message

.. autoclass:: camera_calibration.calibrator.StereoCalibrator
    :members: cal, set_alpha, lremap, rremap, as_message, from_message
