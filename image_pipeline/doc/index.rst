Overview
========

The ``image_pipeline`` stack is designed to process raw camera images
into useful inputs to vision algorithms: rectified mono/color images,
stereo disparity images, and stereo point clouds. Components include:

 * **Calibration**: Cameras must be calibrated in order to relate the
   images they produce to the three-dimensional world. The
   ``camera_calibration`` package provides tools to calibrate monocular
   and stereo cameras in your ROS system. The :ref:`Camera Info` page
   provides a detailed description of the parameters used by the
   pipeline.
 * **Monocular processing**: The raw image stream can be piped through
   the ``image_proc`` node to remove camera distortion. The node also
   performs color interpolation for Bayer pattern color cameras.
 * **Stereo processing**: The ``stereo_image_proc`` package performs
   the duties of ``image_proc`` for a pair of cameras co-calibrated
   for stereo vision. It also uses stereo processing to produce
   disparity images and point clouds.
 * **Depth processing**: ``depth_image_proc`` provides components
   for processing depth images (as produced by the Kinect,
   time-of-flight cameras, etc.), such as producing point clouds.
 * **Visualization**: The ``image_view`` package provides a lightweight
   alternative to ``rviz2`` for viewing an image topic. It also includes
   a ``stereo_view`` tool for viewing stereo pairs and disparity images.

.. toctree::
   :maxdepth: 2

   self
   camera_info
   tutorials
   changelog
   camera_calibration <../camera_calibration/index.html#http://>
   depth_image_proc <../depth_image_proc/index.html#http://>
   image_proc <../image_proc/index.html#http://>
   image_publisher <../image_publisher/index.html#http://>
   image_rotate <../image_rotate/index.html#http://>
   image_view <../image_view/index.html#http://>
   stereo_image_proc <../stereo_image_proc/index.html#http://>

Indices and tables
==================

* :ref:`genindex`
* :ref:`search`
