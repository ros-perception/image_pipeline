Overview
========

This package uses OpenCV camera calibration, described
`here <https://docs.opencv.org/4.5.4/d9/d0c/group__calib3d.html>`_.
For detailed information on the parameters produced by the calibration, see this
`description <http://docs.ros.org/en/rolling/p/image_pipeline/camera_info.html>`_.

The code API listed for this package is for convenience only.
This package has no supported code API.

For pinhole type cameras this package names the distortion model as **plumb_bob**
or **rational_polynomial**, depending on number of parameters used. For fisheye
type cameras this package uses the **equidistant** distortion model.

Tutorials
---------
There are tutorials on how to run the calibration tool for monocular and
stereo cameras.

Camera Calibrator Usage
-----------------------
To run the ``cameracalibrator`` node for a monocular camera using an 8x6
chessboard with 108mm squares:

.. code-block:: bash

   ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.108 image:=/my_camera/image camera:=/my_camera

When you click on the **Save** button after a succesfull calibration,
the data (calibration data and images used for calibration) will
be written to ``/tmp/calibrationdata.tar.gz``.

To run the ``cameracalibrator`` node for a stereo camera:

.. code-block:: bash

    ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.108 right:=/my_stereo/right/image_raw left:=/my_stereo/left/image_raw left_camera:=/my_stereo/left right_camera:=/my_stereo/right


``cameracalibrator`` supports the following options:

  Chessboard Options:
    You must specify one or more chessboards as pairs of --size and
    --square options.

    -p PATTERN, --pattern=PATTERN
                        calibration pattern to detect - 'chessboard',
                        'circles', 'acircles'
    -s SIZE, --size=SIZE
                        chessboard size as NxM, counting interior corners
                        (e.g. a standard chessboard is 7x7)
    -q SQUARE, --square=SQUARE
                        chessboard square size in meters

  ROS Communication Options:
    --approximate=APPROXIMATE
                        allow specified slop (in seconds) when pairing images
                        from unsynchronized stereo cameras
    --no-service-check  disable check for set_camera_info services at startup
    --queue-size=QUEUE_SIZE
                        image queue size (default 1, set to 0 for unlimited)

  Calibration Optimizer Options:
    --fix-principal-point
                        for pinhole, fix the principal point at the image
                        center
    --fix-aspect-ratio  for pinhole, enforce focal lengths (fx, fy) are equal
    --zero-tangent-dist
                        for pinhole, set tangential distortion coefficients
                        (p1, p2) to zero
    -k NUM_COEFFS, --k-coefficients=NUM_COEFFS
                        for pinhole, number of radial distortion coefficients
                        to use (up to 6, default 2)
    --fisheye-recompute-extrinsicsts
                        for fisheye, extrinsic will be recomputed after each
                        iteration of intrinsic optimization
    --fisheye-fix-skew  for fisheye, skew coefficient (alpha) is set to zero
                        and stay zero
    --fisheye-fix-principal-point
                        for fisheye,fix the principal point at the image
                        center
    --fisheye-k-coefficients=NUM_COEFFS
                        for fisheye, number of radial distortion coefficients
                        to use fixing to zero the rest (up to 4, default 4)
    --fisheye-check-conditions
                        for fisheye, the functions will check validity of
                        condition number
    --disable_calib_cb_fast_check
                        uses the CALIB_CB_FAST_CHECK flag for
                        findChessboardCorners
    --max-chessboard-speed=MAX_CHESSBOARD_SPEED
                        Do not use samples where the calibration pattern is
                        moving faster                      than this speed in
                        px/frame. Set to eg. 0.5 for rolling shutter cameras.

Unsynchronized Stereo
---------------------
By default, the ``image_pipeline`` assumes that stereo cameras are triggered
to capture images simultaneously, and that matching image pairs have identical
timestamps. This is the ideal situation, but requires hardware support.

If your stereo pairs are not (or inexactly) synchronized, enable approximate
timestamp matching ``--approximate=0.01`` option. This permits a "slop" of
0.01s between image pairs. If you still don't see a display window, or it
is sporadically updated, try increasing the slop.

Dual Checkerboards
------------------
It is possible to use multiple size checkerboards to calibrate a camera.

To use multiple checkerboards, give multiple ``--size`` and ``--square``
options for additional boards. Make sure the boards have different
dimensions, so the calibration system can tell them apart.

Camera Check
------------
To run the command-line utility to check the calibration of a monocular camera:

.. code-block:: bash

    ros2 run camera_calibration cameracheck --size 8x6 monocular:=/forearm image:=image_rect

To run the command-line utility to check the calibration of a stereo camera:

.. code-block:: bash

    ros2 run camera_calibration cameracheck --size 8x6 stereo:=/wide_stereo image:=image_rect

.. toctree::
   :maxdepth: 2

   self
   components
   tutorial_mono
   tutorial_stereo
   api
