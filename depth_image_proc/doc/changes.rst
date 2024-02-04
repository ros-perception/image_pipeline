Changelog Notes
===============

Jazzy Jalisco
-------------
There are several major change between ``Iron`` and ``Jazzy``:

 * All components now properly support ``image_transport`` parameter, or
   ``depth_image_transport`` if the topic is a depth image.
 * All components now properly support remapping the ``camera_info`` topic
   for an associated ``image`` topic. For instance, if you remap ``image``
   to ``my/image`` then ``my/camera_info`` will be used. Previously you
   would have to manually remap the ``camera_info`` topic.
 * The input of ``point_cloud_xyz_radial`` is renamed from ``image_raw``
   to ``depth/image_raw`` for consistency.
 * The input of ``point_cloud_xyzrgb_radial`` is renamed from
   ``depth_registered/image_rect`` to ``depth/image_raw`` and
   ``rgb/image_rect_color`` to ``rgb/image_raw``.
