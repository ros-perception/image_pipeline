^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package image_publisher
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.3.0 (2022-12-04)
------------------
* Add retry video capture feature with timeout
  Retry onInit() if the loading image fails or if the image is empty.
  This is useful if the stream is lost for a while or if the stream
  is not ready in the beginning.
* changes per comments
* fix for stereo_image_proc_tests
* Add maintainer (`#667 <https://github.com/ros-perception/image_pipeline/issues/667>`_)
* Contributors: Ashwin Sushil, Jacob Perron, Patrick Musau

2.2.1 (2020-08-27)
------------------
* remove email blasts from steve macenski (`#596 <https://github.com/ros-perception/image_pipeline/issues/596>`_)
* [Foxy][Image Publisher] Update launch file (`#579 <https://github.com/ros-perception/image_pipeline/issues/579>`_)
  Co-authored-by: louis <louis.tran@otsaw.com>
* [Foxy] Use ament_auto Macros (`#573 <https://github.com/ros-perception/image_pipeline/issues/573>`_)
  * Fixing version and maintainer problems in camera_calibration.
  * Applying ament_auto macros to depth_image_proc.
  * Cleaning up package.xml in image_pipeline.
  * Applying ament_auto macros to image_proc.
  * Applying ament_auto macros to image_publisher.
  * Applying ament_auto macros to image_rotate.
  * Applying ament_auto macros to image_view.
  * Replacing some deprecated headers in image_view.
  * Fixing some build warnings in image_view.
  * Applying ament_auto macros to stereo_image_proc.
  * Adding some linter tests to image_pipeline.
  * Updating package URLs to point to ROS Index.
* Contributors: Joshua Whitley, Steve Macenski, trthanhquang

2.2.0 (2020-07-27)
------------------
* Replacing deprecated header includes with new HPP versions. (`#566 <https://github.com/ros-perception/image_pipeline/issues/566>`_)
  * Replacing deprecated header includes with new HPP versions.
  * CI: Switching to official Foxy Docker container.
  * Fixing headers which don't work correctly.
* Opencv 3 compatibility (`#564 <https://github.com/ros-perception/image_pipeline/issues/564>`_)
  * Remove GTK from image_view.
  It is no longer used at all in image_view.
  * Reinstate OpenCV 3 compatibility.
  While Foxy only supports Ubuntu 20.04 (and hence OpenCV 4),
  we still strive to maintain Ubuntu 18.04 (which has OpenCV 3).
  In this case, it is trivial to keep keep image_pipeline working
  with OpenCV 3, so reintroduce compatibility with it.
  * Fixes from review.
  * One more fix.
* Use newer 'add_on_set_parameters_callback' API (`#562 <https://github.com/ros-perception/image_pipeline/issues/562>`_)
  The old API was deprecated in Foxy and since removed in https://github.com/ros2/rclcpp/pull/1199.
* Remove redundant install call in CMakeLists.txt (`#555 <https://github.com/ros-perception/image_pipeline/issues/555>`_)
* Contributors: Chris Lalancette, Jacob Perron, Joshua Whitley, sgvandijk

2.0.0 (2018-12-09)
------------------
* port image_publisher on ROS2 (`#366 <https://github.com/ros-perception/image_pipeline/issues/366>`_)
* Initial ROS2 commit.
* Contributors: Chris Ye, Michael Carroll

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
