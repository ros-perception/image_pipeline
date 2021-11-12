1.16.0 (2021-11-12)
-------------------
* Fix includes
  In the following commit in vision_opencv, the include
  opencv2/calib3d/calib3d.hpp was removed from pinhole_camera_model.h :
  https://github.com/ros-perception/vision_opencv/commit/51ca54354a8353fc728fcc8bd8ead7d2b6cf7444
  Since we indirectly depended on this include, we now have to add it
  directly.
* support rgba8 and bgra8 encodings by skipping alpha channel
* downsampling original img / upsampling disparity img
* Contributors: Avinash Thakur, Martin Günther, choi0330

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
* Expand range for min_disparity and disparity_range. (`#431 <https://github.com/ros-perception/image_pipeline/issues/431>`_)
* Contributors: Terry Welsh, Tim Übelhör

1.13.0 (2019-06-12)
-------------------
* Merge pull request `#375 <https://github.com/ros-perception/image_pipeline/issues/375>`_ from fizyr-forks/opencv4
* Fix OpenCV4 compatibility.
* Merge pull request `#338 <https://github.com/ros-perception/image_pipeline/issues/338>`_ from k-okada/arg_sync
* add approximate_sync args in stereo_image_proc.launch
* Merge pull request `#395 <https://github.com/ros-perception/image_pipeline/issues/395>`_ from ros-perception/steve_maintain
* adding autonomoustuff mainainer
* adding stevemacenski as maintainer to get emails
* Merge pull request `#392 <https://github.com/ros-perception/image_pipeline/issues/392>`_ from bknight-i3drobotics/patch-1
* Fix typo
  Typo in line: 14. Changed 'sterel algorithm' to 'stereo algorithm'
* add approximate_sync args in stereo_image_proc.launch
* Contributors: Hans Gaiser, Joshua Whitley, Kei Okada, Steven Macenski, Yoshito Okada, bknight-i3drobotics, stevemacenski

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
