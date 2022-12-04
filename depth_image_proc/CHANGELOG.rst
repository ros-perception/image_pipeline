2.3.0 (2022-12-04)
------------------
* Fix linker error caused by templating in the conversions.cpp file (`#718 <https://github.com/ros-perception/image_pipeline/issues/718>`_)
* Fixed typo in pointcloudxyz launch file
* use unique_ptrs, remove unused code, add back in missing initMatrix call
* add xyzrgb radial node
* Use RCLCPP_WARN_THROTTLE (10 secs) to avoid terminal spam
* Fix tiny error in comment
* Warning instead of fatal error when frames are differents
* revert a293252
* Replace deprecated geometry2 headers
  tf2_geometry_msgs.h was deprecated in https://github.com/ros2/geometry2/pull/418
  tf2_eigen.h was deprecated in https://github.com/ros2/geometry2/pull/413
* Add maintainer (`#667 <https://github.com/ros-perception/image_pipeline/issues/667>`_)
* move to hpp/cpp structure, create conversions file
* Fix deprecation warning calling declare_parameter
  As of https://github.com/ros2/rclcpp/pull/1522, we must also declare the type of the parameter.
  We can do this implicitly by providing a default value.
  Prior to this change, distance\_ was being default-initialized to 0.0 anyways.
* Contributors: Evan Flynn, Francisco Martin Rico, Francisco Martín Rico, Harshal Deshpande, Jacob Perron, Patrick Musau

2.2.1 (2020-08-27)
------------------
* remove email blasts from steve macenski (`#596 <https://github.com/ros-perception/image_pipeline/issues/596>`_)
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
* Add rclcpp and rclcpp_components dependencies to package.xml. (`#569 <https://github.com/ros-perception/image_pipeline/issues/569>`_) (`#570 <https://github.com/ros-perception/image_pipeline/issues/570>`_)
  I noticed that these are listed in CMakeLists.txt but not in package.xml
  and this is causing a build failure for the binary releases on
  build.ros2.org:
  http://build.ros2.org/view/Dbin_ubhf_uBhf/job/Dbin_uB64__depth_image_proc__ubuntu_bionic_amd64__binary/
  Co-authored-by: Steven! Ragnarök <nuclearsandwich@users.noreply.github.com>
* Contributors: Joshua Whitley, Steve Macenski

2.2.0 (2020-07-27)
------------------
* Replacing deprecated header includes with new HPP versions. (`#566 <https://github.com/ros-perception/image_pipeline/issues/566>`_)
  * Replacing deprecated header includes with new HPP versions.
  * CI: Switching to official Foxy Docker container.
  * Fixing headers which don't work correctly.
* Contributors: Joshua Whitley

* make parameters work in depth_image_proc (`#544 <https://github.com/ros-perception/image_pipeline/issues/544>`_)
* update depth_image_proc components (`#543 <https://github.com/ros-perception/image_pipeline/issues/543>`_)
  * update depth_image_proc components
  This makes them loadable with the rclcpp_components
  interface. I've fully tested PointCloudXYZRGB and
  ConvertMetric, my use case doesn't use the others.
  I also lack a setup to test the launch files fully,
  but ran them with the realsense commented out and
  they appear to be OK.
  * fix linting
* Contributors: Michael Ferguson

2.0.0 (2018-12-09)
------------------
* enable rclcpp_register_node_plugins (`#368 <https://github.com/ros-perception/image_pipeline/issues/368>`_)
* Port depth image proc on ROS2 (`#362 <https://github.com/ros-perception/image_pipeline/issues/362>`_)
* Initial ROS2 commit.
* Contributors: Chris Ye, Michael Carroll

1.12.23 (2018-05-10)
--------------------

1.12.22 (2017-12-08)
--------------------

1.12.21 (2017-11-05)
--------------------
* Fix C++11 compilation
  This fixes `#292 <https://github.com/ros-perception/image_pipeline/issues/292>`_ and `#291 <https://github.com/ros-perception/image_pipeline/issues/291>`_
* Contributors: Vincent Rabaud

1.12.20 (2017-04-30)
--------------------
* Fix CMake warnings about Eigen.
* Convert depth image metric from [m] to [mm]
* address gcc6 build error
  With gcc6, compiling fails with `stdlib.h: No such file or directory`,
  as including '-isystem /usr/include' breaks with gcc6, cf.,
  https://gcc.gnu.org/bugzilla/show_bug.cgi?id=70129.
  This commit addresses this issue for this package in the same way
  it was addressed in various other ROS packages. A list of related
  commits and pull requests is at:
  https://github.com/ros/rosdistro/issues/12783
  Signed-off-by: Lukas Bulwahn <lukas.bulwahn@oss.bmw-carit.de>
* Contributors: Kentaro Wada, Lukas Bulwahn, Vincent Rabaud

1.12.19 (2016-07-24)
--------------------

1.12.18 (2016-07-12)
--------------------

1.12.17 (2016-07-11)
--------------------

1.12.16 (2016-03-19)
--------------------
* check number of channels before the process
* search minimum value with OpenCV
* Use OpenCV to be faster
* Add a feature for a depth image to crop foremost image
* Contributors: Kenta Yonekura

1.12.15 (2016-01-17)
--------------------
* Add option for exact time sync for point_cloud_xyzrgb
* simplify OpenCV3 conversion
* Contributors: Kentaro Wada, Vincent Rabaud

1.12.14 (2015-07-22)
--------------------

1.12.13 (2015-04-06)
--------------------
* Add radial point cloud processors
* Contributors: Hunter Laux

1.12.12 (2014-12-31)
--------------------
* adds range_max
* exports depth_conversions
  with convert for xyz PC only
* exports DepthTraits
* Contributors: enriquefernandez

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
* Add point_cloud_xyzi nodelet
  This is for cameras that output depth and intensity images.
  It's based on the point_cloud_xyzrgb nodelet.
* Missing runtime dependency - eigen_conversions
  `libdepth_image_proc` is missing this dependency at runtime
  ```
  > ldd libdepth_image_proc.so  | grep eigen
  libeigen_conversions.so => not found
  ```
  Which causes the following error on loading depth_image_proc:
  ```
  [ INFO] [1402564815.530736554]: /camera/rgb/camera_info -> /camera/rgb/camera_info
  [ERROR] [1402564815.727176562]: Failed to load nodelet [/camera/depth_metric_rect] of type
  [depth_image_proc/convert_metric]: Failed to load library /opt/ros/indigo/lib//libdepth_image_proc.so.
  Make sure that you are calling the PLUGINLIB_EXPORT_CLASS macro in the library code, and that
  names are consistent between this macro and your XML. Error string: Could not load library (Poco
  exception = libeigen_conversions.so: cannot open shared object file: No such file or directory)
  [FATAL] [1402564815.727410623]: Service call failed!
  ```
* Contributors: Daniel Stonier, Hunter Laux

1.12.4 (2014-04-28)
-------------------
* depth_image_proc: fix missing symbols in nodelets
* Contributors: Michael Ferguson

1.12.3 (2014-04-12)
-------------------

1.12.2 (2014-04-08)
-------------------

1.12.1 (2014-04-06)
-------------------
* replace tf usage by tf2 usage

1.12.0 (2014-04-04)
-------------------
* remove PCL dependency
