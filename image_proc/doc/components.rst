Nodes and Components
====================

This package includes a number of ROS 2 components that can be assembled
into image processing pipelines.
See the tutorial :ref:`Launch image_proc Components`.

Alternatively, each component can be run as a standalone node.

image_proc::CropDecimateNode
----------------------------
Applies decimation (software binning) and ROI to a raw camera image
post-capture. Remap camera and camera_out to the desired input/output
camera namespaces. Also available as a standalone node with the name
``crop_decimate_node``.

Subscribed Topics
^^^^^^^^^^^^^^^^^
 * **in/image_raw** (sensor_msgs/Image): Raw image stream from the camera driver.
 * **in/camera_info** (sensor_msgs/CameraInfo): Camera metadata.

Published Topics
^^^^^^^^^^^^^^^^
 * **out/image_raw** (sensor_msgs/Image): Cropped and decimated "raw" image.
 * **out/camera_info** (sensor_msgs/CameraInfo): Camera metadata, with binning and
   ROI fields adjusted to match output raw image.

Parameters
^^^^^^^^^^
 * **decimation_x** (int, default: 1): Number of pixels to decimate to one
   horizontally. Range: 1 to 16
 * **decimation_y** (int, default: 1): Number of pixels to decimate to one
   vertically. Range: 1 to 16
 * **image_transport** (string, default: raw): Image transport to use.
 * **interpolation** (int, default: 0): Sampling algorithm. Possible values are:

   * NN (0): Nearest-neighbor sampling
   * Linear (1): Bilinear interpolation
   * Cubic (2): Bicubic interpolation over 4x4 neighborhood
   * Area (3): Resampling using pixel area relation
   * Lanczos4 (4): Lanczos interpolation over 8x8 neighborhood
 * **queue_size** (int, default: 5): Size of message queue for synchronizing
   image and camera_info topics. You may need to raise this if images take
   significantly longer to travel over the network than camera info.
 * **offset_x** (int, default: 0): X offset of the region of interest. Range: 0 to 2447
 * **offset_y** (int, default: 0): Y offset of the region of interest. Range: 0 to 2049
 * **width** (int, default: 0): Width of the region of interest. Range: 0 to 2448
 * **height** (int, default: 0): Height of the region of interest. Range: 0 to 2050

image_proc::CropNonZeroNode
---------------------------
Takes a monochrome image and crops the largest non zero area. Can be used
on depth images. Also available as a standalone node with the name
``crop_non_zero_node``.

Subscribed Topics
^^^^^^^^^^^^^^^^^
 * **image_raw** (sensor_msgs/Image): Monochrome or depth image.

Published Topics
^^^^^^^^^^^^^^^^
 * **image** (sensor_msgs/Image): Cropped image.

Parameters
^^^^^^^^^^
 * **image_transport** (string, default: raw): Image transport to use.

image_proc::DebayerNode
-----------------------
Takes a raw camera stream and publishes monochrome and color versions
of it. If the raw images are Bayer pattern, it debayers using bilinear
interpolation. Also available as a standalone node with the name
``debayer_node``.

Subscribed Topics
^^^^^^^^^^^^^^^^^
 * **image_raw** (sensor_msgs/Image): Raw image stream from the camera driver.

Published Topics
^^^^^^^^^^^^^^^^
 * **image_mono** (sensor_msgs/Image): Monochrome image.
 * **image_color** (sensor_msgs/Image): Color unrectified image.

Parameters
^^^^^^^^^^
 * **debayer** (int, default: 3): Debayering algorithm. Possible values are:

   * Bilinear (0): Fast algorithm using bilinear interpolation
   * EdgeAware (1): Edge-aware algorithm
   * EdgeAwareWeighted (2): Weighted edge-aware algorithm
   * VNG (3): Slow but high quality Variable Number of Gradients algorithm
 * **image_transport** (string, default: raw): Image transport to use.

image_proc::RectifyNode
-----------------------
Takes an unrectified image stream and its associated calibration parameters,
and produces rectified images. Also available as a standalone node with the
name ``recify_node``.

Subscribed Topics
^^^^^^^^^^^^^^^^^
 * **image** (sensor_msgs/Image): Unrectified image stream.
 * **camera_info** (sensor_msgs/CameraInfo): Camera metadata.

Published Topics
^^^^^^^^^^^^^^^^
 * **image_rect** (sensor_msgs/Image): Rectified image.

Parameters
^^^^^^^^^^
 * **queue_size** (int, default: 5): Size of message queue for synchronizing
   ``image`` and ``camera_info`` topics. You may need to raise this if images
   take significantly longer to travel over the network than camera info.
 * **image_transport** (string, default: raw): Image transport to use.
 * **interpolation** (int, default: 1): Interpolation algorithm between source
   image pixels. Possible values are:

   * NN (0): Nearest neighbor
   * Linear (1): Linear
   * Cubic (2): Cubic
   * Lanczos4 (4): Lanczos4

image_proc::ResizeNode
----------------------
Takes image and camera info and resize them. Also available as
standalone node with the name ``resize_node``.

Subscribed Topics
^^^^^^^^^^^^^^^^^
 * **image/image_raw** (sensor_msgs/Image): Arbitrary image.
 * **image/camera_info** (sensor_msgs/CameraInfo): Camera parameters.

Published Topics
^^^^^^^^^^^^^^^^
 * **resized/image_raw** (sensor_msgs/Image): Resized image.
 * **resized/camera_info** (sensor_msgs/CameraInfo): Resized camera info.

Parameters
^^^^^^^^^^
 * **image_transport** (string, default: raw): Image transport to use.
 * **interpolation** (int, default: 0): Sampling algorithm. Possible values are:

   * NN (0): Nearest-neighbor sampling
   * Linear (1): Bilinear interpolation
   * Cubic (2): Bicubic interpolation over 4x4 neighborhood
   * Area (3): Resampling using pixel area relation
   * Lanczos4 (4): Lanczos interpolation over 8x8 neighborhood
 * **use_scale** (bool, default: True): Use scale parameters, or absolute height/width.
 * **scale_height** (float, default: 1.0): Height scaling of image.
 * **scale_width** (float, default: 1.0): Width scaling of image.
 * **height** (float): Absolute height of resized image, if ``use_scale`` is false.
 * **width** (float): Absolute width of resized image, if ``use_scale`` is false.

image_proc::TrackMarkerNode
---------------------------
Takes an image, detects an Aruco marker and publishes a ``geometry_msgs/PoseStamped``
of where the marker is located. Also available as standalone node with the name
``track_marker_node``.

Subscribed Topics
^^^^^^^^^^^^^^^^^
 * **image** (sensor_msgs/Image): Image topic to process.
 * **camera_info** (sensor_msgs/CameraInfo): Camera metadata.

Published Topics
^^^^^^^^^^^^^^^^
 * **tracked_pose** (geometry_msgs/PoseStamped): Pose of the marker.

Parameters
^^^^^^^^^^
 * **dictionary** (int, default: 10): Marker dictionary to use.
   Values correspond to cv.aruco enum.
   The default of 10 corresponds to the DICT_6X6_250 dictionary.
 * **image_transport** (string, default: raw): Image transport to use.
 * **marker_id** (int, default: 0): The ID of the marker to use.
 * **marker_size** (double, default: 0.05): Size of the marker edge,
   in meters.
