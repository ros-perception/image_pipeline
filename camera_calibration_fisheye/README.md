# camera_calibration_fisheye

Same as [camera_calibration](http://wiki.ros.org/camera_calibration) but implementing OpenCV3 fisheye calibration functionality for monoCameras


# Usage
Camera Calibrator
To run the cameracalibrator.py node for a monocular camera using an 8x6 chessboard with 108mm squares:


```
rosrun camera_calibration_fisheye cameracalibrator.py --size 8x6 --square 0.108 image:=/my_camera/image camera:=/my_camera
```
When you click on the "Save" button after a succesfull calibration, the data (calibration data and images used for calibration) will be written to /tmp/calibrationdata.tar.gz.

To run the cameracalibrator.py node for a stereo camera:

```
rosrun camera_calibration_fisheye cameracalibrator.py --size 8x6 --square 0.108 right:=/my_stereo/right/image_raw left:=/my_stereo/left/image_raw left_camera:=/my_stereo/left right_camera:=/my_stereo/right
```

cameracalibrator.py supports the following options:

```
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

  Calibration Optimizer Options:
    --fix-principal-point
                        fix the principal point at the image center
    --fix-aspect-ratio  enforce focal lengths (fx, fy) are equal
    --zero-tangent-dist
                        set tangential distortion coefficients (p1, p2) to
                        zero
    -k NUM_COEFFS, --k-coefficients=NUM_COEFFS
                        number of radial distortion coefficients to use (up to
                        6, default 2)
						
```
