Tutorials
=========

.. _Launch image_proc Components:

Launching image_proc Components
-------------------------------
While each of the components is available as a ROS 2 node, the
recommended way to build pipelines is using the components as
this will save overhead by not having to serialize messages
between components.

The example below creates two composable nodes:
 * ``rectify_node`` subscribes to ``image_raw``, rectifies the
   image and publishes it at ``image_rect``.
 * ``crop_decimage_node`` subscribes to the rectified
   ``image_rect``, decimates it to half the size and publishes
   the ``image_downsized``.

.. code-block:: python

    from launch import LaunchDescription
    from launch_ros.actions import ComposableNodeContainer
    from launch_ros.descriptions import ComposableNode

    def generate_launch_description():

        composable_nodes = [
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='rectify_node',
                remappings=[
                    ('image', 'image_raw'),
                    ('image_rect', 'image_rect')
                ],
            ),
            ComposableNode(
                package='image_proc',
                plugin='image_proc::CropDecimateNode',
                name='crop_decimate_node',
                remappings=[
                    ('in/image_raw', 'image_rect'),
                    ('out/image_raw', 'image_downsized')
                ],
                parameters={
                    'decimation_x': 2,
                    'decimation_y': 2,
                }
            )
        ]

        container = ComposableNodeContainer(
            name='image_proc_container',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=composable_nodes,
        )

        return LaunchDescription([container])

Using Compressed Image Transport
--------------------------------
All of the components and nodes in ``image_proc`` support ``image_transport``.
This allows a subscriber to specify the transport to
be used. By default, this is ``raw``, which means an uncompressed
``sensor_msgs/Image``. When transmitting images over limited bandwidth
networks, such as WiFi, it can be helpful to use ``compressed`` format.

.. code-block:: bash

    $ ros2 run image_proc rectify_node --ros-args -p image_transport:=compressed

Remapping camera_info Topics
----------------------------
See `tutorial in image_pipline <https://docs.ros.org/en/rolling/p/image_pipeline/tutorials.html#remapping-camera-info-topics>`_.

Using QoS Overrides
-------------------
See `tutorial in image_pipline <https://docs.ros.org/en/rolling/p/image_pipeline/tutorials.html#using-qos-overrides>`_.

.. _Using image_proc Launch File:

Using image_proc Launch File
----------------------------
Make sure your camera driver is running. To see the available raw
image topics from compatible drivers you can check:

.. code-block:: bash

    $ ros2 topic list | grep image_raw

Normally the raw image from the camera driver is not what you want
for visual processing, but rather an undistorted and (if necessary)
debayered image. This is the job of ``image_proc``. If you are
running on a robot, it's probably best to run ``image_proc`` there.
For example, if the driver is publishing topics ``/my_camera/image_raw``
and ``/my_camera/camera_info`` you would do:

.. code-block:: bash

    $ ros2 launch image_proc image_proc.launch.py namespace:=my_camera

Notice that we push our ``image_proc`` launch file down into the
``/my_camera`` namespace, in which it subscribes to the ``image_raw``
and ``camera_info`` topics. All output topics are likewise published
within the ``/my_camera`` namespace.

In a separate terminal (on your home machine, if you are running on a robot):

.. code-block:: bash

    $ ros2 run image_view image_view --ros-args -r image:=my_camera/image_rect_color

This will display an undistorted color image from ``my_camera``.

Using the TrackMarkerNode
-------------------------
When generating markers, be sure to pay attention to the selection
of the dictionary. The default dictionary is ``DICT_6X6_250`` which
means you want your marker to be of the 6X6 size, with an ID of 0-249.

There are two ways to generate markers:

 * The `OpenCV Tutorial <https://docs.opencv.org/4.5.4/d5/dae/tutorial_aruco_detection.html>`_
   shows programmatic ways to generate markers.
 * There are a variety of online Aruco marker generation webpages,
   `this one <https://chev.me/arucogen/>` is very easy to generate
   individual markers.

Once the marker is printed, be sure to set the ``marker_id`` and
``marker_size`` parameters for the node. It is recommended to measure
the marker size as printing the marker could incur scaling.
