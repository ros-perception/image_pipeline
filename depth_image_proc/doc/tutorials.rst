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
See `tutorial in image_pipline <https://docs.ros.org/en/rolling/p/image_pipeline/tutorials.html#remapping-camera-info-topics>`_.

Using QoS Overrides
-------------------
See `tutorial in image_pipline <https://docs.ros.org/en/rolling/p/image_pipeline/tutorials.html#using-qos-overrides>`_.
