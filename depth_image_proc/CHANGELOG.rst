2.0.0 (2018-12-09)
------------------
* enable rclcpp_register_node_plugins (`#368 <https://github.com/ros-perception/image_pipeline/issues/368>`_)
  this may be remarked while code debugging, should be enabled to build node plugin file
  and added points remap in point_cloud_xyzrgb.launch.py
* Port depth image proc on ROS2 (`#362 <https://github.com/ros-perception/image_pipeline/issues/362>`_)
  * Port depth_image_proc on ROS2
  * Port depth_image_proc of image_pipeline on ROS2
  * rename Nodelets as Node
  * add launch examples, such as "ros2 launch depth_image_proc point_cloud_xyzi.launch.py"
  * verified point_cloud_xyzrgb, point_cloud_xyz, convert_metric based on Realsense camera(https://github.com/intel/ros2_intel_realsense).
  Signed-off-by: Chris Ye <chris.ye@intel.com>
  * add ament_lint_auto test and adjust code style
  Signed-off-by: Chris Ye <chris.ye@intel.com>
  * update test example for depth_image_proc
  - rename raw topic as image_transport fixed the issue (https://github.com/ros-perception/image_common/issues/96)
  - update test example of point_cloud_xyzrgb.launch
  Signed-off-by: Chris Ye <chris.ye@intel.com>
  * remove unused dependence in cmakelist
  * remove boost which is unused on ROS2
  * remove cv_bridge version check logic as setted in find_package
  * update maintainer in package.xml
  Signed-off-by: Chris Ye <chris.ye@intel.com>
  * added all example launchers for demo test
  pass test:
  ros2 launch depth_image_proc point_cloud_xyzrgb.launch.py
  ros2 launch depth_image_proc point_cloud_xyz.launch.py
  ros2 launch depth_image_proc convert_metric.launch.py
  ros2 launch depth_image_proc crop_foremost.launch.py
  ros2 launch depth_image_proc point_cloud_xyz_radial.launch.py
  ros2 launch depth_image_proc disparity.launch.py
  ros2 launch depth_image_proc register.launch.py
  ros2 launch depth_image_proc point_cloud_xyzi.launch.py
  ros2 launch depth_image_proc point_cloud_xyzi_radial.launch.py
  Signed-off-by: Chris Ye <chris.ye@intel.com>
  * @wip update to use raw pointers.
  * continue to update to use raw pointer
  Signed-off-by: Chris Ye <chris.ye@intel.com>
* Initial ROS2 commit.
* Contributors: Chris Ye, Michael Carroll

Forthcoming
-----------
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
