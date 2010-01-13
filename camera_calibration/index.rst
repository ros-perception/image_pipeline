camera_calibration
==================

For details on the camera model and camera calibration process, see
http://opencv.willowgarage.com/documentation/camera_calibration_and_3d_reconstruction.html.

.. autoclass:: camera_calibration.calibrator.MonoCalibrator
    :members: cal, set_alpha, remap, undistort_points, as_message

.. autoclass:: camera_calibration.calibrator.StereoCalibrator
    :members: cal, set_alpha, as_message
