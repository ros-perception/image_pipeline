Changelog Notes
===============

Jazzy Jalisco
-------------
There are several major change between ``Iron`` and ``Jazzy``:

 * All components now properly support ``image_transport`` parameter.
 * All components now properly support remapping the ``camera_info`` topic
   for an associated ``image`` topic. For instance, if you remap ``image``
   to ``my/image`` then ``my/camera_info`` will be used. Previously you
   would have to manually remap the ``camera_info`` topic.
