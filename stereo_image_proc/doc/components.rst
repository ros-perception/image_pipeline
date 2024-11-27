Nodes and Components
====================

This package includes a number of ROS 2 components that can be assembled
into stereo image processing pipelines.
See the tutorial :ref:`Launch stereo_image_proc Components`.

Alternatively, each component can be run as a standalone node.

stereo_image_proc::DisparityNode
--------------------------------
Performs block matching on a pair of rectified stereo images, producing a
disparity image. Also available as a standalone node with the name
``disparity_node``.

Subscribed Topics
^^^^^^^^^^^^^^^^^
 * **left/camera_info** (sensor_msgs/CameraInfo): Left camera metadata.
 * **left/image_rect** (sensor_msgs/Image): Left monochrome rectified
   image stream.
 * **right/camera_info** (sensor_msgs/CameraInfo): Right camera metadata.
 * **right/image_rect** (sensor_msgs/Image): Right monochrome rectified
   image stream.

Published Topics
^^^^^^^^^^^^^^^^
 * **disparity** (sensor_msgs/DisparityImage): Floating point disparity
   image with metadata.

Parameters
^^^^^^^^^^

*Disparity algorithm variant*
 * **sgbm_mode** (int, default: 0): Stereo matching algorithm variation:

   * SGBM (0)
   * HH (1)
   * SGBM_3WAY (2)
   * HH4 (3)

*Disparity pre-filtering* 

 * **prefilter_size** (int, default: 9): Normalization window size, pixels.
 * **prefilter_cap** (int, default: 31): Bound on normalized pixel values.

*Disparity correlation*

 * **correlation_window_size** (int, default: 15): Edge size (pixels) of the
   correlation window for matching. Values must be odd, in the range 5 to 255
   (but with an extra performance hit for values larger than 21). Larger values
   have smoother disparity results, but smear out small features and depth
   discontinuities.
 * **min_disparity** (int, default: 0): Minimum disparity, or the offset to the
   disparity search window. By setting to a positive value, the cameras become
   more "cross-eyed" and will find objects closer to the cameras. If the cameras
   are "verged" (inclined toward each other), it may be appropriate to set
   min_disparity to a negative value. When min_disparity is greater than 0,
   objects at large distances will not be found.
 * **disparity_range** (int, default: 64): The size of the disparity search
   window (pixels). Together with min_disparity, this defines the "horopter,"
   the 3D volume that is visible to the stereo algorithm.

*Disparity post-filtering*

 * **uniqueness_ratio** (double, default: 15.0): Filters disparity readings
   based on a comparison to the next-best correlation along the epipolar
   line. Matches where uniqueness_ratio > (best_match - next_match) / next_match
   are filtered out.
 * **texture_threshold** (int, default: 10): Filters disparity readings based on
   the amount of texture in the correlation window.
 * **speckle_size** (int, default: 100): Filters disparity regions that are less
   than this number of pixels. For example, a value of 100 means that all regions
   with fewer than 100 pixels will be rejected.
 * **speckle_range** (int, default: 4): Groups disparity regions based on their
   connectedness. Disparities are grouped together in the same region if they are
   within this distance in pixels.

*Synchronization*

 * **approximate_sync** (bool, default: false): Whether to use approximate
   synchronization. Set to true if the left and right cameras do not produce
   exactly synced timestamps.
 * **approximate_sync_tolerance_seconds** (double, default: 0.0): Tolerance
   when using approximate sync.
 * **image_transport** (string, default: raw): Image transport to use for left
   image subscriber.
 * **queue size** (int, default: 5): Size of message queue for each synchronized
   topic. You may need to raise this if images take significantly longer to travel
   over the network than camera info and/or the delay from disparity processing
   is too long.

stereo_image_proc::PointCloudNode
---------------------------------
Combines a rectified color image and disparity image to produce a
``sensor_msgs::PointCloud2`` point cloud. Also available as a standalone
node ``point_cloud_node``.

Note: ROS 1 had both a ``point_cloud`` and ``point_cloud2`` version to
handle the now deprecated and removed ``sensor_msgs::PointCloud`` and
``stereo_msgs::PointCloud2``. This node corresponds to the ``point_cloud2``
version although we have dropped the ``2``.

Subscribed Topics
^^^^^^^^^^^^^^^^^
 * **disparity** (stereo_msgs/DisparityImage): Floating point disparity
   image with metadata.
 * **left/camera_info** (sensor_msgs/CameraInfo): Left camera metadata.
 * **left/image_rect_color** (sensor_msgs/Image): Rectified image.
 * **right/camera_info** (sensor_msgs/CameraInfo): Right camera metada.

Published Topics
^^^^^^^^^^^^^^^^
 * **points2** (sensor_msgs/PointCloud2): Stereo point cloud with RGB color.

Parameters
^^^^^^^^^^
 * **approximate_sync** (bool, default: false): Whether to use approximate
   synchronization. Set to true if the left and right cameras do not produce
   exactly synced timestamps.
 * **avoid_point_cloud_padding** (bool, default: false): Avoid using alignment
   padding in the generated point cloud. This reduces bandwidth requirements,
   as the point cloud size is halved. Using point clouds without alignment
   padding might degrade performance for some algorithms.
 * **image_transport** (string, default: raw): Image transport to use for left
   image subscriber.
 * **queue size** (int, default: 5): Size of message queue for each synchronized
   topic. You may need to raise this if images take significantly longer to travel
   over the network than camera info and/or the delay from disparity processing
   is too long.
 * **use_color** (oool, default: true): If false, point cloud will be XYZ only.
