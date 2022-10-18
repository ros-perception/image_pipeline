^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package image_publisher
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
1.17.0 (2022-10-17)
-------------------
* Switch to hpp headers of pluginlib
* Switch to new boost/bind/bind.hpp
* Contributors: Jochen Sprickerhof

1.16.0 (2021-11-12)
-------------------

1.15.3 (2020-12-11)
-------------------
* remove email blasts from steve macenski (`#595 <https://github.com/ros-perception/image_pipeline/issues/595>`_)
* Contributors: Steve Macenski

1.15.2 (2020-05-19)
-------------------

1.15.1 (2020-05-18)
-------------------

1.15.0 (2020-05-14)
-------------------
* Python 3 compatibility (`#530 <https://github.com/ros-perception/image_pipeline/issues/530>`_)
* cmake_minimum_required to 3.0.2
* Adapted to OpenCV4
* import setup from setuptools instead of distutils-core
* updated install locations for better portability. (`#500 <https://github.com/ros-perception/image_pipeline/issues/500>`_)
* Contributors: Joshua Whitley, Sean Yen

1.14.0 (2020-01-12)
-------------------

1.13.0 (2019-06-12)
-------------------
* Merge pull request `#358 <https://github.com/ros-perception/image_pipeline/issues/358>`_ from lucasw/image_pub_dr_private_namespace
* Use a shared_ptr for the dynamic reconfigure pointer, and create it with the private node handle so that the parameters for the dynamic reconfigure server are in the private namespace and two image publishers can coexist in the same manager `#357 <https://github.com/ros-perception/image_pipeline/issues/357>`_
* Merge pull request `#395 <https://github.com/ros-perception/image_pipeline/issues/395>`_ from ros-perception/steve_maintain
* adding autonomoustuff mainainer
* adding stevemacenski as maintainer to get emails
* Contributors: Joshua Whitley, Lucas Walter, Yoshito Okada, stevemacenski

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
