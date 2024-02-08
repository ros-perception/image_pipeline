Nodes and Components
====================

image_view::DisparityViewNode
-----------------------------
Simple image viewer for ROS stereo_msgs/DisparityImage topics.
Color-maps the disparity image for visualization.
Node name is ``disparity_view``.

Subscribed Topics
^^^^^^^^^^^^^^^^^
 * **image** (sensor_msgs/DisparityImage): The disparity image topic.

Parameters
^^^^^^^^^^
 * **autosize** (bool, default: false): Whether the window should autosize
   itself to the image or be resizeable by the user.
 * **window_name** (string, default: name of the image topic):
   The name of the display window.

image_view::ExtractImagesNode
-----------------------------
This tool also allows you to save images as jpg/png file from
streaming (ROS sensor_msgs/Image topic) to a file.
``image_saver`` node provide very similar functionalities,
such as providing service call to trigger the node to save
images, save images other than JPEG format, etc.

This tool allows you to save images as jpg/png file from streaming
(ROS sensor_msgs/Image topic) to a file. From command line, you
can run with:

.. code-block:: bash
    
    ros2 run image_view image_saver --ros-args -r image:=[your topic]

or see this answer to control the timing of capture.

Subscribed Topics
^^^^^^^^^^^^^^^^^
 * **image** (sensor_msgs/Image): Image topic to visualize.

Parameters
----------
 * **filename_format** (string, default: "frame%04i.jpg"): File name for
   saved images, you must add use '%04i' for sequence number.
 * **image_transport** (string, default: raw): Image transport to use.
 * **sec_per_frame** (double, default: 0.1): Seconds between saved frames.

image_view::ImageViewNode
-------------------------
Simple image viewer for ROS sensor_msgs/Image topics. Node name
is ``image_view``.

Subscribed Topics
^^^^^^^^^^^^^^^^^
 * **image** (sensor_msgs/Image): Image topic to visualize.

Parameters
^^^^^^^^^^
 * **autosize** (bool, default: false): Whether the window should autosize
   itself to the image or be resizeable by the user.
 * **filename_format** (string, default: "frame%04i.jpg"): printf-style
   format for saved image names. Use to control name, location and format
   of saved images.
 * **image_transport** (string, default: raw): Image transport to use.
 * **window_name** (string, default: name of the image topic):
   The name of the display window.

image_view::ImageSaverNode
--------------------------
This tool allows you to save images as jpg/png file from streaming
(ROS sensor_msgs/Image topic) to a file. From command line, you
can run with:

.. code-block:: bash
    
    ros2 run image_view image_saver --ros-args -r image:=[your topic]

or see this answer to control the timing of capture.

Subscribed Topics
^^^^^^^^^^^^^^^^^
 * **image** (sensor_msgs/Image): Image topic to visualize.

Services
^^^^^^^^
 * **save** (std_srvs/Empty): Save images, you need to set
   the ``save_all_images`` parameter to false.
 * **start** (std_srvs/Trigger): Start saving images.
 * **end** (std_srvs/Trigger): Stop saving images.

Parameters
----------
 * **encoding** (string, default:"bgr8"): Encoding type of input image topic.
 * **filename_format** (string, default: "left%04i.jpg"): File name for
   saved images, you can use '%04i' for sequence number, and '%s' for default
   file format, you can use 'jpg' ,'png', 'pgm' as filename suffixes.
 * **image_transport** (string, default: raw): Image transport to use.
 * **save_all_images** (bool, default: true): If set to false, images
   are only saved when 'save' service is called.
 * **stamped_filename** (bool, default: false): If set to true, a timestamp
   is appended to each filename.
 * **request_start_end** (bool, default: false): If set to true, the start
   and end services will be advertised and can be used to start and Stop
   saving images. NOTE: ``save_all_images`` must be set to true, or these
   services won't do anything.

image_view::StereoImageViewNode
-------------------------------
Viewer for stereo images. Shows the synchronized left/right image pair
and the disparity image (color-mapped) computed from them.
Node name is ``stereo_image_view``.

It is expected that ``<stereo>`` and ``<image>`` will be remapped to the
appropriate names (as show in :ref:`Viewing Stereo Images`).

Subscribed Topics
^^^^^^^^^^^^^^^^^
 * **<stereo>/left/<image>** (sensor_msgs/Image): The left image topic.
 * **<stereo>/right/<image>** (sensor_msgs/Image): The right image topic.
 * **<stereo>/disparity** (stereo_msgs/DisparityImage): The disparity image
   computed from the left/right stereo pair.

Parameters
^^^^^^^^^^
 * **approximate_sync** (bool, default: false): Whether to use approximate
   synchronization. Set to true if the left and right cameras do not
   produce exactly synced timestamps.
 * **autosize** (bool, default: false): Whether the window should autosize
   itself to the image or be resizeable by the user.
 * **filename_format** (string, default: "%s%04i.jpg"): printf-style
   format for saved image names. Use to control name, location and format
   of saved images. The string argument is "left" or "right".
 * **image_transport** (string, default: raw): Image transport to use.
 * **queue_size** (int, default: 5): Size of message queue for each
   synchronized topic. You may need to raise this if disparity processing
   takes too long, or if there are significant network delays.

image_view::VideoRecorderNode
-----------------------------
This tool allows you to record a video stream (ROS sensor_msgs/Image topic)
to a file. It relies on OpenCV's VideoWriter class. With the default options,
it encodes the video as MPG, encapsulated in a AVI container at 15 fps,
and produces a file called output.avi in the current directory.

Subscribed Topics
^^^^^^^^^^^^^^^^^
 * **image** (sensor_msgs/Image): Image topic to save to file.

Parameters
^^^^^^^^^^
 * **codec** (string, default: MJPG): The FOURCC identifier of the codec.
 * **encoding** (string, default:"bgr8"): Encoding type of input image topic.
 * **filename** (string, default: output.avi): Path and name of the
   output video.
 * **fps** (int, default: 15): Framerate of the video.
 * **image_transport** (string, default: raw): Image transport to use.
 * **queue_size** (int, default: 5): Size of message queue for each
   synchronized topic. You may need to raise this if disparity processing
   takes too long, or if there are significant network delays.
