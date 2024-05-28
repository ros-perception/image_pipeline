Nodes and Components
====================

image_publisher::ImagePublisher
-------------------------------
Component to publish sensor_msgs/Image, requires filename argument.
Also avialable as a ROS 2 node named ``image_publisher``.

Published Topics
^^^^^^^^^^^^^^^^
 * **image_raw** (sensor_msgs/Image): ROS Image message of your input file.
 * **camera_info** (sensor_msgs/CameraInfo): CameraInfo published along with Image.

Parameters
^^^^^^^^^^
 * **filename** (string, default: ""): Name of image file to be published.
 * **field_of_view** (double, default: 0): Camera field of view (deg) used to calculate focal length for camera info topic.
 * **flip_horizontal** (bool, default: false): Flip output image horizontally.
 * **flip_vertical** (bool, default: false): Flip output image vertically.
 * **frame_id** (string, default: "camera") Frame id inserted in published
   image and camera_info.
 * **publish_rate** (double, default: 10): Rate to publish image (hz).
 * **camera_info_uri** (string, default: ""): Path to camera info.
