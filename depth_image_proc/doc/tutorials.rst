Tutorials
=========

.. _Launch depth_image_proc Components:

Launching depth_image_proc Components
-------------------------------------
While each of the components is available as a ROS 2 node, the
recommended way to build pipelines is using the components as
this will save overhead by not having to serialize messages
between components.

.. code-block:: python

    from launch import LaunchDescription
    from launch_ros.actions import ComposableNodeContainer
    from launch_ros.descriptions import ComposableNode

    def generate_launch_description():

        container = ComposableNodeContainer(
            name='depth_image_proc_container',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyzrgbNode',
                    name='point_cloud_xyzrgb_node',
                    remappings=[
                        ('rgb/image_rect_color', 'rgb/image'),
                        ('depth_registered/image_rect', 'depth_registered/image'),
                        ('points', 'depth_registered/points'),
                    ],
                ),
            ]
        )

        return LaunchDescription([container])

Using Compressed Image Transport
--------------------------------
All of the components and nodes in ``depth_image_proc`` support
``image_transport``. This allows a subscriber to specify the transport to
be used. By default, this is ``raw``, which means an uncompressed
``sensor_msgs/Image``. When transmitting images over limited bandwidth
networks, such as WiFi, it can be helpful to use ``compressed`` format.

For the depth images, use the ``depth_image_transport`` parameter. For
setting the transport for intensity or rgb images, the ``image_transport``
parameter is used:

.. code-block:: bash

    $ ros2 run depth_image_proc point_cloud_xyz_node --ros-args -p depth_image_transport:=compressed

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
