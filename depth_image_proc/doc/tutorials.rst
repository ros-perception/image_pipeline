Tutorials
=========

.. _Launch depth_image_proc Components:

Launching depth_image_proc Components
-------------------------------------
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

Using image_proc Launch File
----------------------------
Make sure your camera driver is running. To see the available raw
image topics from compatible drivers you can check:

```
ros2 topic list | grep image_raw
```

Normally the raw image from the camera driver is not what you want
for visual processing, but rather an undistorted and (if necessary)
debayered image. This is the job of ``image_proc``. If you are
running on a robot, it's probably best to run ``image_proc`` there.
For example, if the driver is publishing topics ``/my_camera/image_raw``
and ``/my_camera/camera_info`` you would do:

```
$ ros2 launch image_proc image_proc.launch.py namespace:=my_camera
```

Notice that we push our ``image_proc`` launch file down into the
``/my_camera`` namespace, in which it subscribes to the ``image_raw``
and ``camera_info`` topics. All output topics are likewise published
within the ``/my_camera`` namespace.

In a separate terminal (on your home machine, if you are running on a robot):

```
$ ros2 run image_view image_view --ros-args -r image:=my_camera/image_rect_color
```

This will display an undistorted color image from ``my_camera``.
