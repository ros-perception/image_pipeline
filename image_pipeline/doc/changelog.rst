Changelog Notes
===============

While there is an official changelog for each package, this page summarizes
the major changes between distributions

Changes in Jazzy Jalisco
------------------------
There are several major change between ``Iron`` and ``Jazzy``:

 * All components now properly support ``image_transport`` paramter,
   or ``depth_image_transport`` parameter if the topic is a depth image.
   In most places, this consists of simply adding the parameter, or making
   the parameter work, however two cases should be noted where the
   parameter was renamed:

   * image_view::ExtractImages: incorrectly named parameter ``transport``
     was renamed to more consistent ``image_transport``.
   * imaeg_view::StereoView: incorrectly named parameter ``transport``
     was renamed to more consistent ``image_transport``.

 * Improvements to QoS support:

   * Most components now support QoS overrides via ROS 2 parameters
   * The ``use_system_default_qos`` parameter has been removed from
     stereo_image_proc::DisparityNode and stereo_image_proc::PointCloudNode
     as the QoS overrides are the newer, preferred method.

 * All components now properly support remapping the ``camera_info`` topic
   for an associated ``image`` topic. For instance, if you remap ``image``
   to ``my/image`` then ``my/camera_info`` will be used. Previously you
   would have to manually remap the ``camera_info`` topic. See also
   :ref:`Remapping camera_info Topics`.
 * The input of ``depth_image_proc/point_cloud_xyz_radial`` is renamed
   from ``image_raw`` to ``depth/image_raw`` for consistency.
 * The inputs of ``depth_image_proc/point_cloud_xyzrgb_radial`` are renamed
   from ``depth_registered/image_rect`` to ``depth/image_raw`` and
   ``rgb/image_rect_color`` to ``rgb/image_raw`` to make clear that the
   unrectified camera projection matrix is used, and for consistency with
   other radial nodes.
 * The boolen parameter ``full_dp`` from the DisparityNode has been deleted
   and a new integer parameter ``sgbm_mode`` added to enable all the 
   variations of the stereo matching algorithm SGBM available from the
   OpenCV library.
