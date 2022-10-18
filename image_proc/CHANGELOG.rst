1.17.0 (2022-10-17)
-------------------
* Switch to hpp headers of pluginlib
* Drop old C++ standard compiler flag
* Switch to new boost/bind/bind.hpp
* Update ROI parameters on resize
* Contributors: Jochen Sprickerhof, Yuki Furuta

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
* resize.cpp: fix memory leak (`#489 <https://github.com/ros-perception/image_pipeline/issues/489>`_)
* Try catch around cvtColor to avoid demosaicing src.empty() error (`#463 <https://github.com/ros-perception/image_pipeline/issues/463>`_)
* Merge pull request `#436 <https://github.com/ros-perception/image_pipeline/issues/436>`_ from ros-perception/throttle_warnings
* adding throttled warnings to not blast the users
* Merge pull request `#423 <https://github.com/ros-perception/image_pipeline/issues/423>`_ from lucasw/crop_decimate_resolution_change
  Avoid crashing when the x or y offset is too large
* Merge pull request `#435 <https://github.com/ros-perception/image_pipeline/issues/435>`_ from ros-perception/patch_resize_copy
* patch extra copy for nodelet users of resize
* Merge pull request `#411 <https://github.com/ros-perception/image_pipeline/issues/411>`_ from Tuebel/fix_409
  Fix 409 based on melodic branch
* Need to look at x and y offset
* Simplified copying of the camera_info message.
* Independent resize of image and camera_info
* removed unused infoCb
* Rename infoCb to cameraCb matching subscribeCamera
* replaced boost mutex & shared_ptr with std
* Removed hard coded image encoding.
  Using toCvCopy instead of toCvShared (copy is needed anyway).
* Contributors: Joshua Whitley, Lucas Walter, Tim Übelhör, Yuki Furuta, stevemacenski

1.13.0 (2019-06-12)
-------------------
* Merge pull request `#395 <https://github.com/ros-perception/image_pipeline/issues/395>`_ from ros-perception/steve_maintain
* adding autonomoustuff mainainer
* adding stevemacenski as maintainer to get emails
* Contributors: Joshua Whitley, Yoshito Okada, stevemacenski

1.12.23 (2018-05-10)
--------------------

1.12.22 (2017-12-08)
--------------------
* Merge pull request `#311 <https://github.com/ros-perception/image_pipeline/issues/311>`_ from knorth55/revert-299
  Revert "Fix image_resize nodelet (`#299 <https://github.com/ros-perception/image_pipeline/issues/299>`_)"
  This reverts commit 32e19697ebce47101b063c6a02b95dfa2c5dbc52.
* Contributors: Shingo Kitagawa, Tully Foote

1.12.21 (2017-11-05)
--------------------
* Fix image_resize nodelet (`#299 <https://github.com/ros-perception/image_pipeline/issues/299>`_)
  Update interpolation types
  Add arguments to enable disable each nodelet
  Add default arguments for image_resize and image_rect
  Use toCVShare instead of toCVCopy
  Include image_resize in image_proc
* Updated fix for traits change. (`#303 <https://github.com/ros-perception/image_pipeline/issues/303>`_)
* Fix C++11 compilation
  This fixes `#292 <https://github.com/ros-perception/image_pipeline/issues/292>`_ and `#291 <https://github.com/ros-perception/image_pipeline/issues/291>`_
* [image_proc][crop_decimate] support changing target image frame_id (`#276 <https://github.com/ros-perception/image_pipeline/issues/276>`_)
* Contributors: Furushchev, Mike Purvis, Vincent Rabaud, bikramak

1.12.20 (2017-04-30)
--------------------
* Add nodelet to resize image and camera_info (`#273 <https://github.com/ros-perception/image_pipeline/issues/273>`_)
  * Add nodelet to resize image and camera_info
  * Depends on nodelet_topic_tools
  * Use recursive_mutex for mutex guard for dynamic reconfiguring
* Fix nodelet name: crop_nonZero ->  crop_non_zero (`#269 <https://github.com/ros-perception/image_pipeline/issues/269>`_)
  Fix https://github.com/ros-perception/image_pipeline/issues/217
* Fix permission of executable files unexpectedly (`#260 <https://github.com/ros-perception/image_pipeline/issues/260>`_)
* address gcc6 build error
  With gcc6, compiling fails with `stdlib.h: No such file or directory`,
  as including '-isystem /usr/include' breaks with gcc6, cf.,
  https://gcc.gnu.org/bugzilla/show_bug.cgi?id=70129.
  This commit addresses this issue for this package in the same way
  it was addressed in various other ROS packages. A list of related
  commits and pull requests is at:
  https://github.com/ros/rosdistro/issues/12783
  Signed-off-by: Lukas Bulwahn <lukas.bulwahn@oss.bmw-carit.de>
* Contributors: Kentaro Wada, Lukas Bulwahn

1.12.19 (2016-07-24)
--------------------

1.12.18 (2016-07-12)
--------------------

1.12.17 (2016-07-11)
--------------------

1.12.16 (2016-03-19)
--------------------
* clean OpenCV dependency in package.xml
* issue `#180 <https://github.com/ros-perception/image_pipeline/issues/180>`_ Check if all distortion coefficients are zero.
  Test with:
  rostest --reuse-master --text image_proc test_rectify.xml
  Can also test interactively with vimjay image_rect.launch, which brings up an rqt gui and camera info distortion coefficients can be dynamically reconfigured.
* Add a feature to crop the largest valid (non zero) area
  Remove unnecessary headers
  change a filename to fit for the ROS convention
* Contributors: Kenta Yonekura, Lucas Walter, Vincent Rabaud

1.12.15 (2016-01-17)
--------------------
* simplify OpenCV3 conversion
* Contributors: Vincent Rabaud

1.12.14 (2015-07-22)
--------------------

1.12.13 (2015-04-06)
--------------------
* fix dependencies
* Contributors: Vincent Rabaud

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

1.12.1 (2014-04-06)
-------------------
* get proper opencv dependency
* Contributors: Vincent Rabaud

1.11.7 (2014-03-28)
-------------------

1.11.6 (2014-01-29 00:38:55 +0100)
----------------------------------
- fix bad OpenCV linkage (#53)
