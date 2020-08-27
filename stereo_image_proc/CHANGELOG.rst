2.1.1 (2020-08-27)
------------------
* Bump test timeouts to 60 seconds (`#594 <https://github.com/ros-perception/image_pipeline/issues/594>`_)
* [Dashing] Use ament_auto Macros (`#573 <https://github.com/ros-perception/image_pipeline/issues/573>`_) (`#575 <https://github.com/ros-perception/image_pipeline/issues/575>`_)
* Contributors: Jacob Perron, Joshua Whitley

2.1.0 (2020-07-27)
------------------
* Add parameter to avoid points2 padding (`#524 <https://github.com/ros-perception/image_pipeline/issues/524>`_)
  * Add parameter to allow avoiding padding in the generated pointcloud message
  Co-authored-by: Jacob Perron <jacob@openrobotics.org>
  * Actually use the created parameter descriptor
  Co-authored-by: Jacob Perron <jacob@openrobotics.org>
* Add parameter to use system default QoS for image subscriptions in stereo_image_proc (`#521 <https://github.com/ros-perception/image_pipeline/issues/521>`_)
  * Add parameter for image subscription reliability QoS to point cloud node
  * Add parameter for image subscription reliability QoS to disparity node
  * Add argument for setting reliability QoS to stereo_image_proc launch file
  * Change parameter to use_system_default_qos
  * Only use system default QoS for subscriptions
* Use sensor data QoS profile (`#494 <https://github.com/ros-perception/image_pipeline/issues/494>`_)
* Expand range for min_disparity and disparity_range (`#495 <https://github.com/ros-perception/image_pipeline/issues/495>`_)
  Forward port of `#431 <https://github.com/ros-perception/image_pipeline/issues/431>`_
* Add reconfigurable parameters to disparity node (`#490 <https://github.com/ros-perception/image_pipeline/issues/490>`_)
  * Add reconfigurable parameters to disparity node
  * Remove old dynamic reconfigure config file
* Fix get/set speckle size methods (`#491 <https://github.com/ros-perception/image_pipeline/issues/491>`_)
* Port stereo_image_proc to ROS 2 (`#486 <https://github.com/ros-perception/image_pipeline/issues/486>`_)
* Initial ROS2 commit.
* Contributors: Ivan Santiago Paunovic, Jacob Perron, Joshua Whitley, Kei Okada, Michael Carroll, Steven Macenski, Yoshito Okada, bknight-i3drobotics, stevemacenski

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
