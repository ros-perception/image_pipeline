2.3.0 (2022-12-04)
------------------
* Add missing test dependency
* Add color param to stereo_image_proc (`#661 <https://github.com/ros-perception/image_pipeline/issues/661>`_)
  * Add color param to point cloud node in stereo_image_proc
  * Add test for point cloud node with color disabled
  * Fix style issues
  * Rename test node
  * Remove repeated line
* fix for stereo_image_proc_tests
* Add maintainer (`#667 <https://github.com/ros-perception/image_pipeline/issues/667>`_)
* Add disparity node parameters to launch file
  This makes the launch file more flexible for including in other launch files.
* Fix disparity node parameter name
  Rename the parameter 'texture_ratio' to 'texture_threshold'.
  The parameter was incorrectly named during the port from ROS 1 to ROS 2.
* Expose avoid_point_cloud_padding parameter in stereo_image_proc launch file (`#599 <https://github.com/ros-perception/image_pipeline/issues/599>`_)
* Refactor image_proc and stereo_image_proc launch files (`#583 <https://github.com/ros-perception/image_pipeline/issues/583>`_)
  * Allow passing container name to image_proc launch file
  If a container name is provided, then load the image_proc nodes
  into that container. Otherwise, launch a container to load the nodes into.
  * Include the image_proc launch file in the stereo_image_proc launch file
  This resolves a TODO, making the launch file have similar behavior as the version from ROS 1.
  Also expose a new launch argument for optionally providing a container (similar to image_proc's launch file).
  * Minor refactor to stereo_image_proc launch file
  * Fix lint errors
  Removing vestigial imports.
  * Rename namespace launch arguments
  * Make image_proc nodes optional
  Default to launching the image_proc nodes.
  * Remap topics from stereo nodes based on namespace arguments
* Contributors: Jacob Perron, Patrick Musau, Rebecca Butler

2.2.1 (2020-08-27)
------------------
* remove email blasts from steve macenski (`#596 <https://github.com/ros-perception/image_pipeline/issues/596>`_)
* Refactor stereo_image_proc tests (`#588 <https://github.com/ros-perception/image_pipeline/issues/588>`_)
* [Foxy] Use ament_auto Macros (`#573 <https://github.com/ros-perception/image_pipeline/issues/573>`_)
* Contributors: Jacob Perron, Joshua Whitley, Steve Macenski

2.2.0 (2020-07-27)
------------------
* Replacing deprecated header includes with new HPP versions. (`#566 <https://github.com/ros-perception/image_pipeline/issues/566>`_)
* Use newer 'add_on_set_parameters_callback' API (`#562 <https://github.com/ros-perception/image_pipeline/issues/562>`_)
  The old API was deprecated in Foxy and since removed in https://github.com/ros2/rclcpp/pull/1199.
* Contributors: Jacob Perron, Joshua Whitley

* Initial ROS2 commit.
* Contributors: Michael Carroll

1.12.23 (2018-05-10)
--------------------
* Removed unused mutable scratch buffers (`#315 <https://github.com/ros-perception/image_pipeline/issues/315>`_)
  The uint32_t buffers conflicted with newer release of OpenCV3, as explained here https://github.com/ros-perception/image_pipeline/issues/310
* Contributors: Miquel Massot

1.12.22 (2017-12-08)
--------------------

1.12.21 (2017-11-05)
--------------------
* Updated fix for traits change. (`#303 <https://github.com/ros-perception/image_pipeline/issues/303>`_)
* Fix C++11 compilation
  This fixes `#292 <https://github.com/ros-perception/image_pipeline/issues/292>`_ and `#291 <https://github.com/ros-perception/image_pipeline/issues/291>`_
* Contributors: Mike Purvis, Vincent Rabaud

1.12.20 (2017-04-30)
--------------------
* fix doc jobs
  This is a proper fix for `#233 <https://github.com/ros-perception/image_pipeline/issues/233>`_
* address gcc6 build error
  With gcc6, compiling fails with `stdlib.h: No such file or directory`,
  as including '-isystem /usr/include' breaks with gcc6, cf.,
  https://gcc.gnu.org/bugzilla/show_bug.cgi?id=70129.
  This commit addresses this issue for this package in the same way
  it was addressed in various other ROS packages. A list of related
  commits and pull requests is at:
  https://github.com/ros/rosdistro/issues/12783
  Signed-off-by: Lukas Bulwahn <lukas.bulwahn@oss.bmw-carit.de>
* Contributors: Lukas Bulwahn, Vincent Rabaud

1.12.19 (2016-07-24)
--------------------

1.12.18 (2016-07-12)
--------------------

1.12.17 (2016-07-11)
--------------------

1.12.16 (2016-03-19)
--------------------
* clean OpenCV dependency in package.xml
* Contributors: Vincent Rabaud

1.12.15 (2016-01-17)
--------------------
* simplify OpenCV3 conversion
* Contributors: Vincent Rabaud

1.12.14 (2015-07-22)
--------------------
* add StereoSGBM and it can be chosen from dynamic_reconfigure
* Contributors: Ryohei Ueda

1.12.13 (2015-04-06)
--------------------
* get code to compile with OpenCV3
* modify pointcloud data format of stereo_image_proc using point_cloud2_iterator
* Contributors: Hiroaki Yaguchi, Vincent Rabaud

1.12.12 (2014-12-31)
--------------------

1.12.11 (2014-10-26)
--------------------

1.12.10 (2014-09-28)
--------------------

1.12.9 (2014-09-21)
-------------------
* get code to compile with OpenCV3
  fixes `#96 <https://github.com/ros-perception/image_pipeline/issues/96>`_
* Contributors: Vincent Rabaud

1.12.8 (2014-08-19)
-------------------

1.12.6 (2014-07-27)
-------------------

1.12.4 (2014-04-28)
-------------------

1.12.3 (2014-04-12)
-------------------

1.12.2 (2014-04-08)
-------------------

1.12.0 (2014-04-04)
-------------------
* remove PointCloud1 nodelets

1.11.5 (2013-12-07 13:42:55 +0100)
----------------------------------
- fix compilation on OSX (#50)

1.11.4 (2013-11-23 13:10:55 +0100)
----------------------------------
- convert images to MONO8 when computing disparity if needed (#49)
