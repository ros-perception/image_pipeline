Overview
========

This package contains a number of ROS 2 components and nodes for viewing image topics.

Viewing a Single Image Topic
----------------------------

.. code-block:: bash

    ros 2 run image_view image_view --ros-args -r image:=<image topic>

For example, to view raw images on the topic ``/camera/image``, use:

.. code-block:: bash

    ros2 run image_view image_view --ros-args -r image:=/camera/image

You may save the current image by right-clicking on the display window. By default,
images will be saved as ``frame0000.jpg``, ``frame0001.jpg``, ....

If you want to view a compressed image stream (usually a good idea over wireless!)
using the capabilities of ``image_transport``, specify the transport type as a
command-line parameter. For example, if ``theora_image_transport`` is built on the
publisher's side, you can use theora transport:

.. code-block:: bash

    ros2 run image_view image_view --ros-args -r image:=/camera/image -p image_transport:=theora

.. _`Viewing Stereo Images`:

Viewing Stereo Images
---------------------

.. code-block:: bash

    ros2 run image_view stereo_view --ros-args -p stereo:=<stereo namespace> -p image:=<image topic identifier>

For example, to view stereo image pairs on topics
``/my_stereo_cam/left/image_rect_color`` and ``/my_stereo_cam/right/image_rect_color``,
use:

.. code-block:: bash

    ros2 run image_view stereo_view --ros-args -r stereo:=/my_stereo_cam -r image:=image_rect_color

``stereo_view`` also shows the disparity image computed from the stereo pair
 color-mapped for clarity.

You may save the current image pair by right-clicking on any display window.
By default, images will be saved as ``left0000.jpg``, ``right0000.jpg``, ``disp0000.jpg``,
``left0001.jpg``, ``right0001.jpg``, ``disp0001.jpg``.... As with ``image_view``,
you can specify an image transport to use for the left and right image as an optional
parameter.

.. toctree::
   :maxdepth: 2

   self
   components
   image_view <generated/index>

Indices and tables
==================

* :ref:`genindex`
* :ref:`search`
