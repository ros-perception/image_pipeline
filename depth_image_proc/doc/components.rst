Nodes and Components
====================

This package includes a number of ROS 2 components that can be assembled
into depth image processing pipelines.
See the tutorial :ref:`Launch depth_image_proc Components`.

Alternatively, each component can be run as a standalone node.

depth_image_proc::ConvertMetricNode
-----------------------------------
Component to convert raw uint16 depth image in mm to float depth image in m.
Also available as a standalone node with the name ``convert_metric_node``.

Subscribed Topics
^^^^^^^^^^^^^^^^^
 * **image_raw** (sensor_msgs/Image): ``uint16`` depth image in mm, the native
   OpenNI format.

Published Topics
^^^^^^^^^^^^^^^^
 * **image** (sensor_msgs/Image): ``float`` depth image in m, the recommended
   format for processing in ROS.

Parameters
^^^^^^^^^^
 * **image_transport** (string, default: raw): Image transport to use.

depth_image_proc::CropForemostNode
----------------------------------
TODO

depth_image_proc::DisparityNode
-------------------------------
Comoonent to convert depth image to disparity image.

TODO: copy images in here

Subscribed Topics
^^^^^^^^^^^^^^^^^
 * **left/image_rect** (sensor_msgs/Image): Rectified depth image.
 * **right/camera_info** (sensor_msgs/CameraInfo): Camera calibration and
   metadata. Must contain the baseline, which conventionally is encoded in
   the right camera P matrix.

Published Topics
^^^^^^^^^^^^^^^^
 * **left/disparity** (stereo_msgs/DisparityImage): Disparity image
   (inversely related to depth), for interop with stereo processing nodes.
   For all other purposes use depth images instead.

Parameters
^^^^^^^^^^
 * **delta_d** (double, default: 0.125): Smallest allowed disparity increment,
   which relates to the achievable depth range resolution. Defaults to 1/8 pixel.
 * **image_transport** (string, default: raw): Image transport to use.
 * **min_range** (double, default: 0.0): Minimum detectable distance.
 * **max_range** (double, default: +Inf): Maximum detectable distance.
 * **queue_size** (int, default: 5): Size of message queue for synchronizing
   subscribed topics.

depth_image_proc::PointCloudXyzNode
-----------------------------------
Component to convert depth image to XYZ point cloud.

TODO: copy images

Subscribed Topics
^^^^^^^^^^^^^^^^^
 * **depth/image_rect** (sensor_msgs/Image): Rectified depth image.
 * **intensity/image_rect** (sensor_msgs/Image): Rectified intensity image.
 * **camera_info** (sensor_msgs/CameraInfo): Camera calibration and metadata.

Published Topics
^^^^^^^^^^^^^^^^
 * **points** (sensor_msgs/PointCloud2): XYZ point cloud. If using PCL,
   subscribe as PointCloud<PointXYZ>.

Parameters
^^^^^^^^^^
 * **depth_image_transport** (string, default: raw): Image transport to use
   for the depth topic subscriber.
 * **image_transport** (string, default: raw): Image transport to use for
   the intensity image subscriber.
 * **queue_size** (int, default: 5): Size of message queue for synchronizing
   subscribed topics.

depth_image_proc::PointCloudXyzRadialNode
-----------------------------------------
TODO

depth_image_proc::PointCloudXyziNode
------------------------------------
Component to convert depth image to XYZI point cloud.

Subscribed Topics
^^^^^^^^^^^^^^^^^
 * **image_rect** (sensor_msgs/Image): Rectified depth image.
 * **camera_info** (sensor_msgs/CameraInfo): Camera calibration and metadata.

Published Topics
^^^^^^^^^^^^^^^^
 * **points** (sensor_msgs/PointCloud2): XYZ point cloud. If using PCL,
   subscribe as PointCloud<PointXYZI>.

Parameters
^^^^^^^^^^
 * **depth_image_transport** (string, default: raw): Image transport to use.
 * **queue_size** (int, default: 5): Size of message queue for synchronizing
   subscribed topics.

depth_image_proc::PointCloudXyziRadialNode
------------------------------------------
TODO  

depth_image_proc::PointCloudXyzrgbNode
--------------------------------------
Component combine registered depth image and RGB image into XYZRGB point cloud.

TODO: copy images

Subscribed Topics
^^^^^^^^^^^^^^^^^
 * **depth_registered/image_rect** (sensor_msgs/Image): Rectified depth image,
   registered to the RGB camera
 * **rgb/image_rect_color** (sensor_msgs/Image): Rectified color image.
 * **rgb/camera_info** (sensor_msgs/CameraInfo): RGB camera calibration and metadata.

Published Topics
^^^^^^^^^^^^^^^^
 * **points** (sensor_msgs/PointCloud2): XYZ point cloud. If using PCL,
   subscribe as PointCloud<PointXYZRGB>.

Parameters
^^^^^^^^^^
 * **depth_image_transport** (string, default: raw): Image transport to use
   for depth_registered subscriber.
 * **image_transport** (string, default: raw): Image transport to use for
   rgb/image_rect_color subscriber.
 * **exact_sync** (bool, default: False): Whether to use exact synchronizer.
 * **queue_size** (int, default: 5): Size of message queue for synchronizing
   subscribed topics.

depth_image_proc::PointCloudXyzrgbRadialNode
--------------------------------------------
TODO

depth_image_proc::RegisterNode
------------------------------
Component to "register" a depth image to another camera frame. Reprojecting the
depths requires the calibration parameters of both cameras and, from tf, and the
extrinsic transform between them.

Subscribed Topics
^^^^^^^^^^^^^^^^^
 * **depth/image_rect** (sensor_msgs/Image): Rectified depth image.
 * **depth/camera_info** (sensor_msgs/CameraInfo): Depth camera calibration and metadata.
 * **rgb/camera_info** (sensor_msgs/CameraInfo): RGB camera calibration and metadata.

Published Topics
^^^^^^^^^^^^^^^^
 * **depth_registered/camera_info** (sensor_msgs/CameraInfo): Camera calibration and
   metadata. Same as rgb/camera_info but time-synced to depth_registered/image_rect.
 * **depth_registered/image_rect** (sensor_msgs/Image): Reprojected depth image in the
   RGB camera frame.

Parameters
^^^^^^^^^^
 * **depth_image_transport** (string, default: raw): Image transport to use
   for depth subscriber.
 * **queue_size** (int, default: 5): Size of message queue for synchronizing
   subscribed topics.

Required TF Transforms
^^^^^^^^^^^^^^^^^^^^^^
 * /depth_optical_frame → /rgb_optical_frame: The transform between the depth and
   RGB camera optical frames as specified in the headers of the subscribed topics
   (rendered here as /depth_optical_frame and /rgb_optical_frame).
