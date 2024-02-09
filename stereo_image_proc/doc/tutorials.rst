Tutorials
=========

.. _Launch stereo_image_proc Components:

Launching stereo_image_proc Components
--------------------------------------
While each of the components is available as a ROS 2 node, the
recommended way to build pipelines is using the components as
this will save overhead by not having to serialize messages
between components.

For a detailed example, see ``stereo_image_proc.launch.py``.

Using Compressed Image Transport
--------------------------------
All of the components and nodes in ``stereo_image_proc`` support
``image_transport``. This allows a subscriber to specify the transport to
be used. By default, this is ``raw``, which means an uncompressed
``sensor_msgs/Image``. When transmitting images over limited bandwidth
networks, such as WiFi, it can be helpful to use ``compressed`` format.

.. code-block:: bash

    $ ros2 run stereo_image_proc disparity_node --ros-args -p image_transport:=compressed

Remapping camera_info Topics
----------------------------
When a ``camera_info`` topic is needed, an image_transport camera subscriber
is typically used. ROS 2 convention for naming ``camera_info`` topics is:

 * camera/image - an image in namespace ``camera``.
 * camera/camera_info - the associated camera info

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
