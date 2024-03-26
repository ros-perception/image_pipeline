^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package stereo_image_proc
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

5.0.1 (2024-03-26)
------------------
* DisparityNode: replace full_dp parameter with sgbm_mode (`#945 <https://github.com/ros-perception/image_pipeline/issues/945>`_)
  Previously, only the SGBM and HH modes were allowed
* unified changelog, add missing image, deduplicate tutorials (`#938 <https://github.com/ros-perception/image_pipeline/issues/938>`_)
  Last bit of documentation updates - putting together a single changelog
  summary for the whole release (rather than scattering among packages).
  Unified the camera_info tutorial so it isn't duplicated. Added a missing
  image from image_rotate (was on local disk, but hadn't committed it)
* migrate stereo_image_proc docs (`#928 <https://github.com/ros-perception/image_pipeline/issues/928>`_)
* QoS improvements for image_proc and stereo_image_proc (`#922 <https://github.com/ros-perception/image_pipeline/issues/922>`_)
  First part of `#847 <https://github.com/ros-perception/image_pipeline/issues/847>`_
  * Add QoS overrides for all publishers (in the new, standard way)
  * stereo_image_proc: Default subscriber QoS to SensorDataQoS
  * Clean up some of the comments around lazy subscribers, make them more
  consistent across nodes
* Contributors: Michael Ferguson, Pablo David Aranda Rodríguez

5.0.0 (2024-01-24)
------------------
* stereo_image_proc: cleanup cmake (`#904 <https://github.com/ros-perception/image_pipeline/issues/904>`_)
  This was supposed to be switched over when e-turtle rolled out. J-turtle
  ain't that late...
* stereo_image_proc: consistent image_transport (`#903 <https://github.com/ros-perception/image_pipeline/issues/903>`_)
  * make image_transport work
  * make remap work as expected with image_transport
  * make subscribers lazy
* support rgba8 and bgra8 encodings by skipping alpha channel (`#869 <https://github.com/ros-perception/image_pipeline/issues/869>`_)
  Related with the change in ROS 1
  https://github.com/ros-perception/image_pipeline/pull/671/files
  ---------
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
* fix: change type for epsilon (`#822 <https://github.com/ros-perception/image_pipeline/issues/822>`_)
* add myself as a maintainer (`#846 <https://github.com/ros-perception/image_pipeline/issues/846>`_)
* Contributors: Alejandro Hernández Cordero, Daisuke Nishimatsu, Michael Ferguson

3.0.1 (2022-12-04)
------------------
* Replace deprecated headers
  Fixing compiler warnings.
* Add support for ApproximateEpsilonTime in stereo_image_proc and disparity_node
* Forward container namespace from stereo_image_proc -> image_proc (`#752 <https://github.com/ros-perception/image_pipeline/issues/752>`_)
* Contributors: Brian, Ivan Santiago Paunovic, Jacob Perron

3.0.0 (2022-04-29)
------------------
* Fix the tests for stereo_image_proc.
* Cleanup stereo_image_proc
* Populate CameraInfo camera matrix in test fixture
* Use with_default_policies
* Improve formatting
* Use SubscriptionOptions
* Add subscriber qos overrides
* Remove QosPolicyKind::Invalid
* Allow QoS overrides for publishers
* Add missing test dependency
* Add color param to stereo_image_proc (`#661 <https://github.com/ros-perception/image_pipeline/issues/661>`_)
* changes per comments
* fix for stereo_image_proc_tests
* Add maintainer (`#667 <https://github.com/ros-perception/image_pipeline/issues/667>`_)
* Add disparity node parameters to launch file
* Fix disparity node parameter name
* Expose avoid_point_cloud_padding parameter in stereo_image_proc launch file (`#599 <https://github.com/ros-perception/image_pipeline/issues/599>`_)
* Refactor image_proc and stereo_image_proc launch files (`#583 <https://github.com/ros-perception/image_pipeline/issues/583>`_)
* Contributors: Audrow Nash, Chris Lalancette, Jacob Perron, Patrick Musau, Rebecca Butler

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
