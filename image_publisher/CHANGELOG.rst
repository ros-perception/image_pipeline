^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package image_publisher
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0 (2018-12-09)
------------------
* port image_publisher on ROS2 (`#366 <https://github.com/ros-perception/image_pipeline/issues/366>`_)
* Initial ROS2 commit.
* Contributors: Chris Ye, Michael Carroll

2.1.0 (2020-07-27)
------------------
* Opencv 3 compatibility (`#564 <https://github.com/ros-perception/image_pipeline/issues/564>`_) (`#565 <https://github.com/ros-perception/image_pipeline/issues/565>`_)
  * Remove GTK from image_view.
  It is no longer used at all in image_view.
  * Reinstate OpenCV 3 compatibility.
  While Foxy only supports Ubuntu 20.04 (and hence OpenCV 4),
  we still strive to maintain Ubuntu 18.04 (which has OpenCV 3).
  In this case, it is trivial to keep keep image_pipeline working
  with OpenCV 3, so reintroduce compatibility with it.
  * Fixes from review.
  * One more fix.
  Co-authored-by: Chris Lalancette <clalancette@gmail.com>
* cleanup any last reference to nodelets & register image publisher as a component (`#473 <https://github.com/ros-perception/image_pipeline/issues/473>`_)
  * cleanup any last reference to nodelets & register image publisher as a component properly
  * moving depth image proc's launch files to main dir
  * linting
* Merge pull request `#470 <https://github.com/ros-perception/image_pipeline/issues/470>`_ from ros-perception/crop_ros2
  Hot hot hot patches for ROS2
* properly adding dependencies for image publisher
* add launch examples for image and mono and stereo (`#386 <https://github.com/ros-perception/image_pipeline/issues/386>`_)
  * [image_publisher] launch examples for file and mono and stereo
  launch examples to support:
  1) load local image file and publish to the ros topic
  1) load mono usb camera /dev/video0 and publish to the ros topic
  1) load two usb cameras /dev/video0 amd /dev/video1, and publish to the left and right ros topic, this could simulate stereo camera.
  * Publish image file to topic
  $ros2 launch image_publisher image_publisher_file.launch.py
  /camera/camera_info
  /camera/image_raw
  * Publish monocular camera (/dev/video0)
  $ros2 launch image_publisher image_publisher_mono.launch.py
  /camera/camera_info
  /camera/image_raw
  * Publish stereo camera (/dev/video0 and /dev/video1)
  $ros2 launch image_publisher image_publisher_stereo.launch.py
  /left/camera_info
  /left/image_raw
  /right/camera_info
  /right/image_raw
  * [image_publisher] fix for lint check issue
  fix lint check error in launch file
* Fix uninitialised parameters in image_publisher (`#452 <https://github.com/ros-perception/image_pipeline/issues/452>`_)
  declare_parameters() returns either the parameter value or the default. Since it was ignored the parameters were uninitialised (especially publish_rate, which lead to undefined behavior).
* Merge pull request `#425 <https://github.com/ros-perception/image_pipeline/issues/425>`_ from klintan/ros2
  Dashing: Adapted for Dashing
* image_proc updates, PR comments
* fixed parameter changes
* Fixed CMakeLists.txt for Dashing
* 2.0.0
* Changelogs.
* port image_publisher on ROS2 (`#366 <https://github.com/ros-perception/image_pipeline/issues/366>`_)
  * port image_publisher on ROS2
  * switch to use cmake 3.5
  * change nodelet to classloader
  * change ros::param to ros2 parameter APIs
  * use ros2 code style
  * enable ros2 camera_info_manager
* Initial ROS2 commit.
* Merge pull request `#358 <https://github.com/ros-perception/image_pipeline/issues/358>`_ from lucasw/image_pub_dr_private_namespace
  Use a shared_ptr for the dynamic reconfigure pointer, and create it w…
* Use a shared_ptr for the dynamic reconfigure pointer, and create it with the private node handle so that the parameters for the dynamic reconfigure server are in the private namespace and two image publishers can coexist in the same manager `#357 <https://github.com/ros-perception/image_pipeline/issues/357>`_
* Merge branch 'indigo' into fix_image_view_nodelet
* Merge pull request `#395 <https://github.com/ros-perception/image_pipeline/issues/395>`_ from ros-perception/steve_maintain
  adding stevemacenski as maintainer to get emails on release issues
* adding autonomoustuff mainainer
* adding stevemacenski as maintainer to get emails
* Contributors: Andreas Klintberg, Chris Ye, Joshua Whitley, Luca Della Vedova, Lucas Walter, Michael Carroll, Steven Macenski, Yoshito Okada, stevemacenski

1.12.23 (2018-05-10)
--------------------
* fix 'VideoCapture' undefined symbol error (`#318 <https://github.com/ros-perception/image_pipeline/issues/318>`_)
  * fix 'VideoCapture' undefined symbol error
  The following error occured when trying to run image_publisher:
  [...]/devel/lib/image_publisher/image_publisher: symbol lookup error: [...]/devel/lib//libimage_publisher.so: undefined symbol: _ZN2cv12VideoCaptureC1Ev
  Probably, changes in cv_bridge reducing the OpenCV component dependencies led to the error. See
  https://github.com/ros-perception/vision_opencv/commit/8b5bbcbc1ce65734dc600695487909e0c67c1033
  This is fixed by manually finding OpenCV with the required components and adding the dependencies to the library, not just the node.
  * add image_publisher opencv 2 compatibility
* Contributors: hannometer

1.12.22 (2017-12-08)
--------------------

1.12.21 (2017-11-05)
--------------------

1.12.20 (2017-04-30)
--------------------
* explicitly cast to std::vector<double> to make gcc6 happy
  With gcc6, compiling image_publisher fails with this error:
  ```
  /[...]/image_publisher/src/nodelet/image_publisher_nodelet.cpp: In member function 'virtual void image_publisher::ImagePublisherNodelet::onInit()':
  /[...]/image_publisher/src/nodelet/image_publisher_nodelet.cpp:180:43: error: ambiguous overload for 'operator=' (operand types are 'sensor_msgs::CameraInfo\_<std::allocator<void> >::_D_type {aka std::vector<double>}' and 'boost::assign_detail::generic_list<int>')
  camera_info\_.D = list_of(0)(0)(0)(0)(0);
  ```
  After adding an initial explicit type cast for the assignment,
  compiling fails further with:
  ```
  | /[...]/image_publisher/src/nodelet/image_publisher_nodelet.cpp: In member function 'virtual void image_publisher::ImagePublisherNodelet::onInit()':
  | /[...]/image_publisher/src/nodelet/image_publisher_nodelet.cpp:180:65: error: call of overloaded 'vector(boost::assign_detail::generic_list<int>&)' is ambiguous
  |      camera_info\_.D = std::vector<double> (list_of(0)(0)(0)(0)(0));
  ```
  Various sources on the internet [1, 2, 3] point to use the
  `convert_to_container` method; hence, this commit follows those
  suggestions and with that image_publisher compiles with gcc6.
  [1] http://stackoverflow.com/questions/16211410/ambiguity-when-using-boostassignlist-of-to-construct-a-stdvector
  [2] http://stackoverflow.com/questions/12352692/`ambiguous-call-with-list-of-in-vs2010/12362548#12362548 <https://github.com/ambiguous-call-with-list-of-in-vs2010/12362548/issues/12362548>`_
  [3] http://stackoverflow.com/questions/13285272/using-boostassignlist-of?rq=1
  Signed-off-by: Lukas Bulwahn <lukas.bulwahn@oss.bmw-carit.de>
* address gcc6 build error
  With gcc6, compiling fails with `stdlib.h: No such file or directory`,
  as including '-isystem /usr/include' breaks with gcc6, cf.,
  https://gcc.gnu.org/bugzilla/show_bug.cgi?id=70129.
  This commit addresses this issue for this package in the same way
  it was addressed in various other ROS packages. A list of related
  commits and pull requests is at:
  https://github.com/ros/rosdistro/issues/12783
  Signed-off-by: Lukas Bulwahn <lukas.bulwahn@oss.bmw-carit.de>
* Contributors: Lukas Bulwahn

1.12.19 (2016-07-24)
--------------------
* add image_publisher
* Contributors: Kei Okada

* add image_publisher
* Contributors: Kei Okada
