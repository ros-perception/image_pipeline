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

.. _`Using QoS Overrides`:

Using QoS Overrides
-------------------
Most components in image_pipeline follow the Quality of Service (QoS) recommendations
of `REP-2003 <https://ros.org/reps/rep-2003.html>`_ by default. This means that
subscribers are configured with the "Sensor Data" QoS (which uses "best effort"),
and publishers are configured with "System Default" QoS (which uses "reliable" transport).

These QoS settings work well for many applications, but can be overridden using the
standard features of recent ROS 2 releases. This involves adding additional parameters
to your YAML or launch file. For example, we could update the
`image_publisher_file.launch.py` launch file to change the QoS settings:

.. code-block:: python

    import os

    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchDescription
    import launch_ros.actions

    def generate_launch_description():
        filename = os.path.join(get_package_share_directory('image_publisher'), 'launch',
                                'splash.png')
        return LaunchDescription([

            launch_ros.actions.Node(
                package='image_publisher', executable='image_publisher_node', output='screen',
                arguments=[filename],
                parameters=[{
                    'qos_overrides': {
                        '/camera/image_raw': {
                            'publisher': {
                                'reliability': 'best_effort',
                                'history': 'keep_last',
                                'depth': 100,
                            }
                        }
                    },
                }],
                remappings=[('image_raw', '/camera/image_raw'),
                            ('camera_info', '/camera/camera_info')]),
        ])

If we then run the launch file, we can see our settings are applied:

.. code-block:: bash

    $ ros2 topic info /camera/image_raw -v
    Type: sensor_msgs/msg/Image

    Publisher count: 1

    Node name: ImagePublisher
    Node namespace: /
    Topic type: sensor_msgs/msg/Image
    Topic type hash: RIHS01_d31d41a9a4c4bc8eae9be757b0beed306564f7526c88ea6a4588fb9582527d47
    Endpoint type: PUBLISHER
    GID: 01.10.bf.bd.b7.85.a8.33.58.34.5c.ae.00.00.17.03
    QoS profile:
      Reliability: BEST_EFFORT
      History (Depth): KEEP_LAST (100)
      Durability: VOLATILE
      Lifespan: Infinite
      Deadline: Infinite
      Liveliness: AUTOMATIC
      Liveliness lease duration: Infinite

    Subscription count: 0

A few things to note:

 * The topic name (``/camera/image_raw``) must be the fully resolved topic name,
   and therefore we use the remapped topic name rather than the name in the code
   for the component.
 * Only ``reliability``, ``history``, and ``depth`` can be overwritten.

For more information on QoS overrides, see the `design doc <https://design.ros2.org/articles/qos_configurability.html>`_.
