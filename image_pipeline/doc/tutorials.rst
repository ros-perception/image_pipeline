Tutorials
=========

.. _`Remapping camera_info Topics`:

Remapping camera_info Topics
----------------------------
When a ``camera_info`` topic is needed, an image_transport camera subscriber
is typically used. ROS convention for naming ``camera_info`` topics is:

 * **camera/image** - an image in namespace ``camera``.
 * **camera/camera_info** - the associated camera info.

So if a node subscribes to a topic called ``image``, and the user remaps this
to ``my_camera/image``, then the associated camera info will be automatically
remapped to ``mycamera/camera_info``.

Most ROS 2 camera drivers will follow the convention, but occasionally they do
not. In this case, you will have to manually remap the camera_info - but due
to the way that ROS 2 remapping works you have to use the fully resolved
camera info topic. An example:

 * ``image`` is remapped to ``my_camera/image``.
 * The fully resolved name for the camera info is now ``my_camera/camera_info``.
 * If your camera driver actually publishes ``another_ns/camera_info``, then
   you would have to remap ``my_camera/camera_info`` to ``another_ns/camera_info``.
