Nodes and Components
====================

image_rotate::ImageRotateNode
-----------------------------
Node to rotate an image for visualization. The node takes a source
vector and a target vector, and projects them onto the camera image.
It then rotates the image by the angle neded to align the projection
of the source vector with the projection of the target vector.
The source and target vectors are specified in arbitrary TF
frames allowing the rotation angle to vary dynamically as frames
move relative to one another. With the default settings, the image
will be rotated so that the top of the image matches the up
direction the base_link.

Subscribed Topics
^^^^^^^^^^^^^^^^^
 * **image** (sensor_msgs/Image): Image to be rotated.
 * **camera_info** (sensor_msgs/CameraInfo): Camera metadata, only
   used if ``use_camera_info`` is set to true.

Published Topics
^^^^^^^^^^^^^^^^
 * **rotated/image** (sensor_msgs/Image): Rotated image.
 * **out/camera_info** (sensor_msgs/CameraInfo): Camera metadata, with binning and
   ROI fields adjusted to match output raw image.

Parameters
^^^^^^^^^^
 * **target_frame_id** (str, default: base_link): Frame in which the target
   vector is specified. Empty means the input frame.
 * **target_x** (double, default: 0.0): X coordinate of the target vector.
   Range: -10.0 to 10.0.
 * **target_y** (double, default: 0.0): Y coordinate of the target vector.
   Range: -10.0 to 10.0.
 * **target_z** (double, default: 1.0): Z coordinate of the target vector,
   Range: -10.0 to 10.0.
 * **source_frame_id** (str, default: ""): Frame in which the source vector
   is specified. Empty means the input frame.
 * **source_x** (double, default: 0.0): X coordinate of the direction the
   target should be aligned with. Range: -10.0 to 10.0.
 * **source_y** (double, default: 0.0): Y coordinate of the direction the
   target should be aligned with. Range: -10.0 to 10.0.
 * **source_z** (double, default: 1.0): Z coordinate of the direction the
   target should be aligned with. Range: -10.0 to 10.0.
 * **input_frame_id** (str, default: ""): Frame to use for the original camera
   image. Empty means that the frame in the image or camera_info should be
   used depending on use_camera_info.
 * **output_frame_id** (str, default: ""): Frame to publish for the image's
   new orientation. Empty means add '_rotated' suffix to the image frame.
 * **use_camera_info** (bool, default: True): Indicates that the camera_info
   topic should be subscribed to to get the default input_frame_id.
   Otherwise the frame from the image message will be used.
 * **max_angular_rate** (double, default: 10.0): Limits the rate at which
   the image can rotate (rad/s). Zero means no limit. Range: 0.0 to 100.0.
 * **output_image_size** (double, default: 2.0): Size of the output image
   as a function of the input image size. Can be varied continuously between
   the following special settings:

   * 0 ensures no black ever appears
   * 1 is small image dimension
   * 2 is large image dimension
   * 3 is image diagonal
