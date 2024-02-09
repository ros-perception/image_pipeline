Nodes
=====

This package includes a number of ROS 2 components that can be assembled
into image processing pipelines.
See the tutorial :ref:`Launch image_proc Components`.

Alternatively, each component can be run as a standalone node.

camera_calibrator
-----------------
``cameracalibrator`` subscribes to ROS raw image topics, and presents a
calibration window. It can run in both monocular and stereo modes.
The calibration window shows the current images from the cameras,
highlighting the checkerboard. When the user presses the **CALIBRATE**
button, the node computes the camera calibration parameters. When the
user clicks **COMMIT**, the node uploads these new calibration parameters
to the camera driver using a service call.

Subscribed Topics
^^^^^^^^^^^^^^^^^
 * **image** (sensor_msgs/Image): Raw image topic, for monocular cameras.
 * **left** (sensor_msgs/Image): Raw left image topic, for stereo cameras.
 * **right** (sensor_msgs/Image): Raw right image topic, for stereo cameras.

Services Called
^^^^^^^^^^^^^^^
 * **camera/set_camera_info** (sensor_msgs/SetCameraInfo): Sets the camera
   info for a monocular camera.
 * **left_camera/set_camera_info** (sensor_msgs/SetCameraInfo): Sets the camera
   info for the left camera of a stereo pair.
 * **right_camera/set_camera_info** (sensor_msgs/SetCameraInfo): Sets the camera
   info for the right camera of a stereo pair.

camera_check
------------
``cameracheck`` subscribes to ROS rectified image topics and their associated
camera_info, and prints out an error estimate. It can run in both monocular
and stereo modes. The program expects to see a standard checkerboard target.

Subscribed Topics
^^^^^^^^^^^^^^^^^
 * **monocular/image** (sensor_msgs/Image): Rectified image topic, for
   monocular camera.
 * **monocular/camera_info** (sensor_msgs/CameraInfo): Camera info for
   the monocular camera.
 * **stereo/left/image** (sensor_msgs/Image): Rectified left image topic,
   for stereo cameras.
 * **stereo/right/image** (sensor_msgs/Image): Rectified right image topic,
   for stereo cameras.
 * **stereo/camera_info** (sensor_msgs/CameraInfo): Camera info for the
   stereo pair.
