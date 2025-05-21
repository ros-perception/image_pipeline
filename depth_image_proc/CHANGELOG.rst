^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package depth_image_proc
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

5.0.11 (2025-05-21)
-------------------

5.0.10 (2025-04-22)
-------------------

5.0.9 (2025-02-27)
------------------
* fix depth_image_proc launch files (backport `#1077 <https://github.com/ros-perception/image_pipeline/issues/1077>`_) (`#1080 <https://github.com/ros-perception/image_pipeline/issues/1080>`_)
* Contributors: mergify[bot]

5.0.8 (2025-02-13)
------------------

5.0.7 (2025-02-10)
------------------

5.0.6 (2024-12-11)
------------------
* Support QoS override parameters in depth_image_proc/register (backport `#1043 <https://github.com/ros-perception/image_pipeline/issues/1043>`_) (`#1044 <https://github.com/ros-perception/image_pipeline/issues/1044>`_)
  This PR adds support to the `depth_image_proc` - `register` node for
  setting External QoS Configuration on topic _subscriptions\_.
  Co-authored-by: Stuart Alldritt <s.k.alldritt@gmail.com>
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
* Contributors: mergify[bot]

5.0.5 (2024-10-31)
------------------

5.0.4 (2024-08-20)
------------------
* Finish QoS updates (backport `#1019 <https://github.com/ros-perception/image_pipeline/issues/1019>`_) (`#1024 <https://github.com/ros-perception/image_pipeline/issues/1024>`_)
  This implements the remainder of `#847 <https://github.com/ros-perception/image_pipeline/issues/847>`_:
  - Make sure publishers default to system defaults (reliable)
  - Add QoS overriding where possible (some of the image_transport /
  message_filters stuff doesn't really support that)
  - Use the matching heuristic for subscribers consistently
* fix signature issue from `#943 <https://github.com/ros-perception/image_pipeline/issues/943>`_ (backport `#1018 <https://github.com/ros-perception/image_pipeline/issues/1018>`_) (`#1023 <https://github.com/ros-perception/image_pipeline/issues/1023>`_)
  Without this, we get
  ```
  symbol lookup error: /home/ubr/jazzy/install/depth_image_proc/lib/libdepth_image_proc.so: undefined symbol: _ZN16depth_image_proc10convertRgbERKSt10shared_ptrIKN11sensor_msgs3msg6Image_ISaIvEEEES0_INS2_12PointCloud2_IS4_EEEiiii
  c++filt _ZN16depth_image_proc10convertRgbERKSt10shared_ptrIKN11sensor_msgs3msg6Image_ISaIvEEEES0_INS2_12PointCloud2_IS4_EEEiiii
  depth_image_proc::convertRgb(std::shared_ptr<sensor_msgs::msg::Image\_<std::allocator<void> > const> const&, std::shared_ptr<sensor_msgs::msg::PointCloud2\_<std::allocator<void> > >, int, int, int, int)
  ```
  This is an automatic backport of pull request `#1018 <https://github.com/ros-perception/image_pipeline/issues/1018>`_ done by
  [Mergify](https://mergify.com).
  Co-authored-by: Michael Ferguson <mfergs7@gmail.com>
* Contributors: mergify[bot]

5.0.3 (2024-07-16)
------------------

5.0.2 (2024-05-27)
------------------

5.0.1 (2024-03-26)
------------------
* Update depth_image_proc::RegisterNode documentation (`#957 <https://github.com/ros-perception/image_pipeline/issues/957>`_)
  Adding missing parameters from register node of depth_image_proc
  package.
  Related to Issue `#956 <https://github.com/ros-perception/image_pipeline/issues/956>`_
* add invalid_depth param (`#943 <https://github.com/ros-perception/image_pipeline/issues/943>`_)
  Add option to set all invalid depth pixels to a specified value, typically the maximum range.
  * Updates convertDepth parameter name and optimizes use of the parameter.
  * Updates PointCloudXYZ, PointCloudXYZI, and PointCloudXYZRGB with new invalid_depth parameter
* fix image publisher remapping (`#941 <https://github.com/ros-perception/image_pipeline/issues/941>`_)
  Addresses `#940 <https://github.com/ros-perception/image_pipeline/issues/940>`_ - fixes the compressed/etc topic remapping for publishers
* unified changelog, add missing image, deduplicate tutorials (`#938 <https://github.com/ros-perception/image_pipeline/issues/938>`_)
  Last bit of documentation updates - putting together a single changelog
  summary for the whole release (rather than scattering among packages).
  Unified the camera_info tutorial so it isn't duplicated. Added a missing
  image from image_rotate (was on local disk, but hadn't committed it)
* migrate image_pipeline docs (`#929 <https://github.com/ros-perception/image_pipeline/issues/929>`_)
  * Migrates image_pipeline overview page
  * Migrates CameraInfo wiki page
  * Adds links to the other packages in this stack
  * Updates depth_image_proc and image_proc to have the overview page properly named and in the TOC
* migrate depth_image_proc docs (`#926 <https://github.com/ros-perception/image_pipeline/issues/926>`_)
* Fixed image types in depth_image_proc
* Contributors: Alejandro Hernández Cordero, Alessio Parmeggiani, Michael Ferguson, philipp.polterauer

5.0.0 (2024-01-24)
------------------
* radial nodes: should all sub to raw topics (`#906 <https://github.com/ros-perception/image_pipeline/issues/906>`_)
  Per findings in
  https://github.com/ros-perception/image_pipeline/issues/388#issuecomment-1902487162
  - instead of renaming xyz_radial and xyzi_radial to image_rect, I should
  have made the xyzrgb_radial use image_raw (since these nodes use
  matrices K & D):
  * Revert the change in xyzi_radial - topic is depth/image_raw as it has
  always been
  * Revert the change in xyz_radial, although it is still changed slightly
  from the old "image_raw" -> "depth/image_raw" for consistency with the
  other nodes.
  * Update xyzrgb_radial:
  * depth_registered/image_rect -> depth/image_raw
  * rgb/image_rect_color -> rgb/image_raw
  * update launch files accordingly (and remove camera_info since it no
  longer needs to be renamed, happens automagically). Note: these launch
  files are probably epically bad since realsense doesn't output radial
  images... but we'll leave them as documentation for these nodes.
* depth_image_proc: update launch files (`#905 <https://github.com/ros-perception/image_pipeline/issues/905>`_)
  * follow up to `#900 <https://github.com/ros-perception/image_pipeline/issues/900>`_ - had not noticed these launch files at the time
  * remove camera_info topics that auto remap now
* depth_image_proc: consistent image_transport (`#900 <https://github.com/ros-perception/image_pipeline/issues/900>`_)
  * all node support image_transport and/or depth_image_transport parameters.
  * point cloud nodes use depth_image_transport parameter for all depth inputs
  * fixes so that remapping works appropriately for image topics, even when using transports other than raw
  * fixes so that remapping works appropriately for image_transport outputs (crop/convert nodes)
  * support remapping camera_info topics
* support rgba8 and bgra8 encodings by skipping alpha channel (`#869 <https://github.com/ros-perception/image_pipeline/issues/869>`_)
  Related with the change in ROS 1
  https://github.com/ros-perception/image_pipeline/pull/671/files
  ---------
* ROS 2: Add option to use the RGB image timestamp for the registered depth image (`#872 <https://github.com/ros-perception/image_pipeline/issues/872>`_)
  Related with this PR in ROS 1
  https://github.com/ros-perception/image_pipeline/pull/871
* Support MONO16 image encodings: point_cloud_xyz (`#868 <https://github.com/ros-perception/image_pipeline/issues/868>`_)
  Related with this change in ROS 1
  https://github.com/ros-perception/image_pipeline/pull/630
* ROS 2: depth_image_proc/point_cloud_xyzi_radial Add intensity conversion (copy) for float (`#867 <https://github.com/ros-perception/image_pipeline/issues/867>`_)
  Ported from ROS 1
  https://github.com/ros-perception/image_pipeline/pull/336/files
* make remaining components lazy (`#853 <https://github.com/ros-perception/image_pipeline/issues/853>`_)
  missed a few components in `#815 <https://github.com/ros-perception/image_pipeline/issues/815>`_
* allow use as component or node (`#852 <https://github.com/ros-perception/image_pipeline/issues/852>`_)
  This addresses
  https://github.com/ros-perception/image_pipeline/issues/823:
  * depth_image_proc was never implemented properly this way
  * image_proc might have once worked this way, but it appears upstream
  has changed over time and it was no longer doing the job.
  * stereo_image_proc is actually implemented correctly - I just added a
  comment
  With this PR:
  ```
  $ ros2 pkg executables image_proc
  image_proc crop_decimate_node
  image_proc crop_non_zero_node
  image_proc debayer_node
  image_proc image_proc
  image_proc rectify_node
  image_proc resize_node
  ```
  ```
  $ ros2 pkg executables depth_image_proc
  depth_image_proc convert_metric_node
  depth_image_proc crop_foremost_node
  depth_image_proc disparity_node
  depth_image_proc point_cloud_xyz_node
  depth_image_proc point_cloud_xyz_radial_node
  depth_image_proc point_cloud_xyzi_node
  depth_image_proc point_cloud_xyzi_radial_node
  depth_image_proc point_cloud_xyzrgb_node
  depth_image_proc point_cloud_xyzrgb_radial_node
  depth_image_proc register_node
  ```
* add support for lazy subscribers (`#815 <https://github.com/ros-perception/image_pipeline/issues/815>`_)
  This implements `#780 <https://github.com/ros-perception/image_pipeline/issues/780>`_ for ROS 2 distributions after Iron, where we have:
  * Connect/disconnect callbacks, per https://github.com/ros2/rmw/issues/330 (this made it into Iron)
  * Updated APIs in https://github.com/ros-perception/image_common/pull/272 (this is only in Rolling currently)
* add myself as a maintainer (`#846 <https://github.com/ros-perception/image_pipeline/issues/846>`_)
* Depth image transport configure susbcribers (`#844 <https://github.com/ros-perception/image_pipeline/issues/844>`_) (`#845 <https://github.com/ros-perception/image_pipeline/issues/845>`_)
* Updated depth_image_proc for ros2
  Instantiated template for convertDepth, added options to register, and
  changed register from a class loader to an RCLPP component.
* Contributors: Alejandro Hernández Cordero, Michael Ferguson, ksommerkohrt

3.0.1 (2022-12-04)
------------------
* Replace deprecated headers
  Fixing compiler warnings.
* Contributors: Jacob Perron

3.0.0 (2022-04-29)
------------------
* Cleanup of depth_image_proc.
* Fix linker error caused by templating in the conversions.cpp file (`#718 <https://github.com/ros-perception/image_pipeline/issues/718>`_)
* Port upsampling interpolation from `#363 <https://github.com/ros-perception/image_pipeline/issues/363>`_ to ROS2 (`#692 <https://github.com/ros-perception/image_pipeline/issues/692>`_)
* Fix uncrustify errors
* allow loading depth_image_proc::RegisterNode as a component
* Replace deprecated geometry2 headers
* Fixed typo in pointcloudxyz launch file
* use unique_ptrs, remove unused code, add back in missing initMatrix call
* add xyzrgb radial node
* Use RCLCPP_WARN_THROTTLE (10 secs) to avoid terminal spam
* Fix tiny error in comment
* Warning instead of fatal error when frames are differents
* revert a293252
* Replace deprecated geometry2 headers
* Add maintainer (`#667 <https://github.com/ros-perception/image_pipeline/issues/667>`_)
* move to hpp/cpp structure, create conversions file
* Fix deprecation warning calling declare_parameter
* Contributors: Chris Lalancette, Evan Flynn, Francisco Martin Rico, Francisco Martín Rico, Harshal Deshpande, Jacob Perron, Joe Schornak, Joseph Schornak, Joshua Whitley, Patrick Musau

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
