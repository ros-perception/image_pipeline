Overview
========

``depth_image_proc`` provides basic processing for depth images, much as
``image_proc`` does for traditional 2D images. The two packages are
complementary; for example, you can (and should!) rectify your depth
image before converting it to a point cloud.

A variety of camera technologies can produce depth images:

 * The Kinect and related devices
 * Traditional stereo cameras
 * Time-of-flight cameras

See REP 118 for details on depth image representation. The REP recommends
that, wherever possible, producers and consumers of depth data use depth
images (of type sensor_msgs/Image) instead of sensor_msgs/DisparityImage.

All ROS 2 components (besides ``ConvertMetricNode``) in this package support
both standard floating point depth images and OpenNI-specific uint16 depth
images. Thus when working with OpenNI cameras (e.g. the Kinect), you can
save a few CPU cycles by using the uint16 raw topics instead of the float
topics.

For an example of ``depth_image_proc`` in practice, examine the contents of
``openni2_launch``.

.. toctree::
   :maxdepth: 2

   self
   components
   tutorials
   image_proc <generated/index>

Indices and tables
==================

* :ref:`genindex`
* :ref:`search`
